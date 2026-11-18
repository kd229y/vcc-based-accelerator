// Build engine for environment format v0 (PLE-CSE cse_env_mem write port).
// Bit-exact with sw_rrtc/fx/fx_env.c::fx_env_build for every input it accepts:
//   * one representative per fine cell (3.125 cm): smallest S=4 centre distance, ties keep the first
//   * points ordered by query-cell Morton key, then by fine-cell Morton key inside the query cell
//   * CELL slots in query-cell Morton order: {base, count, AABB as 11-bit offsets from the cell origin}
//     AABB offsets are kept with 8 bits (low 3 dropped) and written rounded outward (lo & ~7, hi | 7):
//     the box only grows, so CSE culling is more conservative and every query answer is unchanged.
//   * the hash slot keeps only {valid, q}; the selector distance d is recomputed from q[8:5]
//   * PTS words are 11-bit offsets from the query-cell origin (33 bits, lossless; CSE adds the origin back)
//   * DIR (20^3, index (z*20+y)*20+x) = {occ, slot}; HDR = {err,valid}, n_pts, n_cells, glo, ghi
//
// Phases: CLEAR (occupancy masks; + hash table after reset / a failed build) -> FILTER (hash, 2 cycles
// per point, records a 64-bit fine-cell occupancy mask per query cell) -> PREFIX (query cells in Morton
// order: popcount -> base, slot; writes all 8000 DIR words) -> SCATTER (each kept point goes straight to
// PTS[base + rank], rank = popcount of the mask below its fine cell; AABBs accumulated per slot; hash
// table cleared on the way) -> CELLS -> HDR.  No point array is kept inside the engine.
//
// Limits (reported as CAPACITY, hdr err = 1): kept points >= 3/4 * 2^HASH_AW or > NPT, cells > NCELL.
`timescale 1ns/1ps
module build_engine_v0 #(
  parameter HASH_AW  = 14,              // 16384 slots -> up to 12288 points (PLE-CSE max seen 11670)
  parameter NPT      = 16384,           // cse_env_mem PT_DEPTH
  parameter NCELL    = 1024,            // cse_env_mem CELL_DEPTH
  parameter RAM_STYLE = "block"
) (
  input  wire          clk,
  input  wire          rst,
  input  wire          start,
  input  wire [31:0]   npoints,
  input  wire          wbuf,            // environment buffer to write (0/1)
  // raw points: 32-bit words (x, y, z, ...) or one point per beat
  input  wire          w_valid,
  output wire          w_ready,
  input  wire [31:0]   w_data,
  input  wire          pt_valid,
  output wire          pt_ready,
  input  wire [95:0]   pt_data,         // x [31:0], y [63:32], z [95:64]
  // status
  output reg           busy,
  output reg           done_pulse,
  output reg  [3:0]    status,          // 0 SUCCESS, 4 CAPACITY_EXCEEDED, 5 INVALID_INPUT
  output reg  [15:0]   kept,
  output reg  [12:0]   ncells,
  output reg  [31:0]   culled,
  output reg  [31:0]   probes,
  output reg  [31:0]   cycles,
  // cse_env_mem write port (same encoding: region 0 HDR, 1 DIR, 2 CELL, 3 PTS)
  output reg           env_wr_en,
  output reg           env_wr_buf,
  output reg  [1:0]    env_wr_region,
  output reg  [15:0]   env_wr_addr,
  output reg  [127:0]  env_wr_data
);
  localparam C = (1 << HASH_AW);
  localparam [3:0] ST_OK = 4'd0, ST_CAP = 4'd4, ST_INV = 4'd5;
  localparam [1:0] R_HDR = 2'd0, R_DIR = 2'd1, R_CELL = 2'd2, R_PTS = 2'd3;
  localparam NQ = 8000;

  localparam [4:0]
    B_IDLE = 5'd0, B_CLEAR = 5'd1, F_WAIT = 5'd2, F_CMP = 5'd3, PF_RUN = 5'd4, PF_DRAIN = 5'd5,
    SC_RD = 5'd6, SC_EVAL = 5'd7, SC_S1 = 5'd8, SC_S2 = 5'd9, CL_RUN = 5'd10, HD_RUN = 5'd11, B_FIN = 5'd12;
  reg [4:0] st;

  // ---------------- memories ----------------
  // hash slot {valid, q z,y,x} (d is recomputed from q)
  reg              h_we;   reg [HASH_AW-1:0] h_waddr; reg [48:0] h_wdata;
  reg [HASH_AW-1:0] h_raddr; wire [48:0] h_rdata;
  ram_sdp #(.DW(49), .AW(HASH_AW), .RAM_STYLE(RAM_STYLE)) u_hash (
    .clk(clk), .we(h_we), .waddr(h_waddr), .wdata(h_wdata), .raddr(h_raddr), .rdata(h_rdata));
  // per query cell: fine-cell occupancy mask
  reg        m_we; reg [12:0] m_waddr; reg [63:0] m_wdata;
  reg [12:0] m_raddr; wire [63:0] m_rdata;
  ram_sdp #(.DW(64), .AW(13), .DEPTH(NQ), .RAM_STYLE(RAM_STYLE)) u_mask (
    .clk(clk), .we(m_we), .waddr(m_waddr), .wdata(m_wdata), .raddr(m_raddr), .rdata(m_rdata));
  // per query cell: slot id (valid only for occupied cells)
  reg        so_we; reg [12:0] so_waddr; reg [9:0] so_wdata;
  reg [12:0] so_raddr; wire [9:0] so_rdata;
  ram_sdp #(.DW(10), .AW(13), .DEPTH(NQ), .RAM_STYLE(RAM_STYLE)) u_slotof (
    .clk(clk), .we(so_we), .waddr(so_waddr), .wdata(so_wdata), .raddr(so_raddr), .rdata(so_rdata));
  // per slot: {count[6:0], base[15:0]} and AABB {hi z,y,x, lo z,y,x} (11-bit offsets)
  reg        sb_we; reg [9:0] sb_waddr; reg [22:0] sb_wdata;
  reg [9:0]  sb_raddr; wire [22:0] sb_rdata;
  ram_sdp #(.DW(23), .AW(10), .RAM_STYLE(RAM_STYLE)) u_sbase (
    .clk(clk), .we(sb_we), .waddr(sb_waddr), .wdata(sb_wdata), .raddr(sb_raddr), .rdata(sb_rdata));
  reg        sx_we; reg [9:0] sx_waddr; reg [47:0] sx_wdata;
  reg [9:0]  sx_raddr; wire [47:0] sx_rdata;
  ram_sdp #(.DW(48), .AW(10), .RAM_STYLE(RAM_STYLE)) u_sbox (
    .clk(clk), .we(sx_we), .waddr(sx_waddr), .wdata(sx_wdata), .raddr(sx_raddr), .rdata(sx_rdata));

  // ---------------- input: word assembler + stage A ----------------
  reg  [31:0] wx, wy;
  reg  [1:0]  wcnt;
  reg  [31:0] words_left;
  wire        in_filter = (st == F_WAIT) || (st == F_CMP);
  wire        sa_in_ready;
  wire        use_pt = pt_valid && (wcnt == 2'd0);
  assign pt_ready = in_filter && (words_left != 0) && (wcnt == 2'd0) && sa_in_ready;
  assign w_ready  = in_filter && (words_left != 0) && !use_pt && ((wcnt != 2'd2) || sa_in_ready);
  wire        sa_in_valid = in_filter && (words_left != 0) && (use_pt || (w_valid && (wcnt == 2'd2)));

  wire        sa_valid, sa_abort, sa_cull;
  wire [47:0] sa_q;
  wire [9:0]  sa_d;
  wire [15:0] sa_hash;
  wire [12:0] sa_qidx;
  wire [5:0]  sa_low6;
  wire        sa_ready = (st == F_WAIT);
  stage_a_v0 u_sa (
    .clk(clk), .rst(rst || st == B_IDLE),
    .in_valid(sa_in_valid), .in_ready(sa_in_ready),
    .in_x(use_pt ? pt_data[31:0] : wx), .in_y(use_pt ? pt_data[63:32] : wy), .in_z(use_pt ? pt_data[95:64] : w_data),
    .out_valid(sa_valid), .out_ready(sa_ready), .out_abort(sa_abort), .out_cull(sa_cull),
    .out_q(sa_q), .out_d(sa_d), .out_hash(sa_hash), .out_qidx(sa_qidx), .out_low6(sa_low6));

  // ---------------- helpers ----------------
  function [12:0] qidx_of; input [47:0] q;          // (z*20 + y)*20 + x of the query cell
    qidx_of = ({8'd0, q[47:43]} * 13'd400) + ({8'd0, q[31:27]} * 13'd20) + {8'd0, q[15:11]};
  endfunction
  function [5:0] low6_of; input [47:0] q;
    low6_of = {q[42], q[26], q[10], q[41], q[25], q[9]};
  endfunction
  function [7:0] esq; input [3:0] o; reg signed [5:0] t;
    begin t = $signed({1'b0, o, 1'b1}) - 6'sd16; esq = t * t; end
  endfunction
  function [9:0] dsel; input [47:0] q;     // S=4 selector distance of a stored point
    dsel = esq(q[8:5]) + esq(q[24:21]) + esq(q[40:37]);
  endfunction
  function [6:0] popc64; input [63:0] v; integer i;
    begin popc64 = 7'd0; for (i = 0; i < 64; i = i + 1) popc64 = popc64 + v[i]; end
  endfunction
  function [7:0] umin8; input [7:0] a, b; umin8 = (a < b) ? a : b; endfunction
  function [7:0] umax8; input [7:0] a, b; umax8 = (a > b) ? a : b; endfunction
  function [15:0] umin16; input [15:0] a, b; umin16 = (a < b) ? a : b; endfunction
  function [15:0] umax16; input [15:0] a, b; umax16 = (a > b) ? a : b; endfunction

  // ---------------- registers ----------------
  reg               wbuf_r;
  reg [31:0]        consumed, npts;
  reg [3:0]         err;
  reg               dirty;
  reg [47:0]        cur_q;
  reg [9:0]         cur_d;
  reg [12:0]        cur_qidx;
  reg [5:0]         cur_low6;
  reg [HASH_AW-1:0] probe;
  reg [HASH_AW:0]   tries;
  reg [15:0]        count;
  reg               m_pend;
  reg [15:0]        k;               // clear / prefix Morton counter / cell / hdr counter
  // prefix pipeline
  reg               p1v, p2v;
  reg [12:0]        p1i, p2i;
  reg               p1in, p2in;      // coordinate inside the 20^3 grid
  reg [6:0]         p2c;
  reg [15:0]        sum;
  reg [10:0]        nslot;
  // scatter
  reg [HASH_AW:0]   si;
  reg [47:0]        sq;
  reg [12:0]        sqi;
  reg [6:0]         srank;
  reg [9:0]         sslot;
  reg [47:0]        glo, ghi;

  wire        hv     = h_rdata[48];
  wire [9:0]  hd     = dsel(h_rdata[47:0]);
  wire [47:0] hq     = h_rdata[47:0];
  wire        hmatch = (hq[47:41] == cur_q[47:41]) && (hq[31:25] == cur_q[31:25]) && (hq[15:9] == cur_q[15:9]);
  wire        cap_full = (count * 4 >= 3 * C) || (count >= NPT);

  // Morton key k (5 bits per axis) -> query cell coordinates
  wire [4:0]  mx = {k[12], k[9], k[6], k[3], k[0]};
  wire [4:0]  my = {k[13], k[10], k[7], k[4], k[1]};
  wire [4:0]  mz = {k[14], k[11], k[8], k[5], k[2]};
  wire        m_in = (mx < 5'd20) && (my < 5'd20) && (mz < 5'd20);
  wire [12:0] m_idx = ({8'd0, mz} * 13'd400) + ({8'd0, my} * 13'd20) + {8'd0, mx};

  // AABB of the slot being updated in scatter (offsets inside the query cell)
  // 8-bit offsets: q[10:3] inside the 2048-LSB query cell
  wire [7:0]  ox = sq[10:3], oy = sq[26:19], oz = sq[42:35];
  wire [47:0] box_upd = {umax8(sx_rdata[47:40], oz), umax8(sx_rdata[39:32], oy), umax8(sx_rdata[31:24], ox),
                         umin8(sx_rdata[23:16], oz), umin8(sx_rdata[15:8], oy), umin8(sx_rdata[7:0], ox)};

  // ---------------- combinational memory / env port control ----------------
  always @* begin
    h_we = 1'b0; h_waddr = probe; h_wdata = {1'b1, cur_q};
    h_raddr = si[HASH_AW-1:0];
    m_we = 1'b0; m_waddr = cur_qidx; m_wdata = m_rdata | (64'd1 << cur_low6);
    m_raddr = cur_qidx;
    so_we = 1'b0; so_waddr = p2i; so_wdata = nslot[9:0]; so_raddr = qidx_of(h_rdata[47:0]);
    sb_we = 1'b0; sb_waddr = nslot[9:0]; sb_wdata = {p2c, sum}; sb_raddr = so_rdata;
    sx_we = 1'b0; sx_waddr = sslot; sx_wdata = box_upd; sx_raddr = so_rdata;
    env_wr_en = 1'b0; env_wr_buf = wbuf_r; env_wr_region = R_HDR; env_wr_addr = 16'd0; env_wr_data = 128'd0;
    case (st)
      B_CLEAR: begin
        m_we = (k < NQ); m_waddr = k[12:0]; m_wdata = 64'd0;
        h_we = dirty && (k < C); h_waddr = k[HASH_AW-1:0]; h_wdata = 49'd0;
      end
      F_WAIT: h_raddr = sa_hash[HASH_AW-1:0];
      F_CMP: begin
        if (err == ST_OK && !hv && !cap_full) h_we = 1'b1;
        else if (err == ST_OK && hv && hmatch && (cur_d < hd)) h_we = 1'b1;
        else if (hv && !hmatch) h_raddr = probe + 1'b1;
      end
      PF_RUN, PF_DRAIN: begin
        m_raddr = m_idx;
        if (p2v && p2in) begin
          env_wr_en = 1'b1; env_wr_region = R_DIR; env_wr_addr = {3'd0, p2i};
          env_wr_data = (p2c != 7'd0) ? {112'd0, 1'b1, 3'd0, 2'd0, nslot[9:0]} : 128'd0;
          if (p2c != 7'd0 && nslot < NCELL) begin
            so_we = 1'b1;
            sb_we = 1'b1;
            sx_we = 1'b1; sx_waddr = nslot[9:0]; sx_wdata = {24'd0, {24{1'b1}}};   // hi = 0, lo = 0xff
          end
        end
      end
      SC_EVAL: begin
        h_we = 1'b1; h_waddr = si[HASH_AW-1:0]; h_wdata = 49'd0;          // leave the hash table clean
        so_raddr = qidx_of(h_rdata[47:0]);
        m_raddr  = qidx_of(h_rdata[47:0]);
        if (!h_rdata[48]) h_raddr = si[HASH_AW-1:0] + 1'b1;
      end
      SC_S1: begin sb_raddr = so_rdata; sx_raddr = so_rdata; end
      SC_S2: begin
        env_wr_en = 1'b1; env_wr_region = R_PTS; env_wr_addr = sb_rdata[15:0] + {9'd0, srank};
        env_wr_data = {95'd0, sq[42:32], sq[26:16], sq[10:0]};   // 11-bit offsets in the query cell
        sx_we = 1'b1; sx_waddr = sslot; sx_wdata = box_upd;
      end
      CL_RUN: begin
        sb_raddr = k[9:0]; sx_raddr = k[9:0];
        if (p1v) begin
          env_wr_en = 1'b1; env_wr_region = R_CELL; env_wr_addr = {5'd0, p1i[10:0]};
          env_wr_data = {36'd0, sx_rdata[47:40], 3'b111, sx_rdata[39:32], 3'b111, sx_rdata[31:24], 3'b111,
                                sx_rdata[23:16], 3'b000, sx_rdata[15:8], 3'b000, sx_rdata[7:0], 3'b000,
                         3'd0, sb_rdata[22:16], sb_rdata[15:0]};
        end
      end
      HD_RUN: begin
        env_wr_en = 1'b1; env_wr_region = R_HDR; env_wr_addr = k;
        case (k[3:0])
          4'd0: env_wr_data = (err == ST_OK) ? 128'd1 : 128'd2;
          4'd1: env_wr_data = (err == ST_OK) ? {112'd0, count} : 128'd0;
          4'd2: env_wr_data = (err == ST_OK) ? {117'd0, nslot} : 128'd0;
          4'd3: env_wr_data = (err == ST_OK) ? {112'd0, glo[15:0]}  : 128'd0;
          4'd4: env_wr_data = (err == ST_OK) ? {112'd0, glo[31:16]} : 128'd0;
          4'd5: env_wr_data = (err == ST_OK) ? {112'd0, glo[47:32]} : 128'd0;
          4'd6: env_wr_data = (err == ST_OK) ? {112'd0, ghi[15:0]}  : 128'd0;
          4'd7: env_wr_data = (err == ST_OK) ? {112'd0, ghi[31:16]} : 128'd0;
          default: env_wr_data = (err == ST_OK) ? {112'd0, ghi[47:32]} : 128'd0;
        endcase
      end
      default: ;
    endcase
    if (m_pend) m_we = 1'b1;     // pending mask update (the cycle after a new fine cell in F_CMP)
  end

  // ---------------- main FSM ----------------
  always @(posedge clk) begin
    if (rst) begin
      st <= B_IDLE; busy <= 1'b0; done_pulse <= 1'b0; status <= ST_OK; dirty <= 1'b1; m_pend <= 1'b0;
      kept <= 0; ncells <= 0; culled <= 0; probes <= 0; cycles <= 0; wcnt <= 2'd0; words_left <= 0;
      err <= ST_OK; wbuf_r <= 1'b0; p1v <= 1'b0; p2v <= 1'b0;
    end else begin
      done_pulse <= 1'b0;
      m_pend <= 1'b0;
      if (busy) cycles <= cycles + 32'd1;
      if (pt_valid && pt_ready) words_left <= words_left - 32'd1;
      if (w_valid && w_ready) begin
        case (wcnt)
          2'd0: begin wx <= w_data; wcnt <= 2'd1; end
          2'd1: begin wy <= w_data; wcnt <= 2'd2; end
          default: begin wcnt <= 2'd0; words_left <= words_left - 32'd1; end
        endcase
      end

      case (st)
        B_IDLE: if (start) begin
          busy <= 1'b1; status <= ST_OK; err <= ST_OK; wbuf_r <= wbuf;
          kept <= 0; ncells <= 0; culled <= 0; probes <= 0; cycles <= 0;
          count <= 0; consumed <= 0; npts <= npoints; words_left <= npoints; wcnt <= 2'd0;
          glo <= {48{1'b1}}; ghi <= 48'd0; k <= 16'd0; st <= B_CLEAR;
        end

        B_CLEAR: begin
          k <= k + 16'd1;
          if (k == ((dirty && C > NQ) ? C - 1 : NQ - 1)) st <= F_WAIT;
        end

        F_WAIT: begin
          if (consumed == npts) begin
            if (err != ST_OK) st <= HD_RUN;
            else begin k <= 16'd0; p1v <= 1'b0; p2v <= 1'b0; sum <= 16'd0; nslot <= 11'd0; st <= PF_RUN; end
            if (err != ST_OK) k <= 16'd0;
          end else if (sa_valid) begin
            consumed <= consumed + 32'd1;
            if (err != ST_OK) begin
            end else if (sa_abort) err <= ST_INV;
            else if (sa_cull) culled <= culled + 32'd1;
            else begin
              cur_q <= sa_q; cur_d <= sa_d; cur_qidx <= sa_qidx; cur_low6 <= sa_low6;
              probe <= sa_hash[HASH_AW-1:0]; tries <= 0; st <= F_CMP;
            end
          end
        end

        F_CMP: begin
          probes <= probes + 32'd1;
          tries <= tries + 1'b1;
          if (!hv) begin
            if (cap_full) begin err <= ST_CAP; st <= F_WAIT; end
            else begin count <= count + 16'd1; m_pend <= 1'b1; st <= F_WAIT; end
          end else if (hmatch) st <= F_WAIT;
          else begin
            probe <= probe + 1'b1;
            if (tries + 1 >= C) begin err <= ST_CAP; st <= F_WAIT; end
          end
        end

        // query cells in Morton order: k issues (mask read), p1 = popcount, p2 = DIR / slot writes
        PF_RUN, PF_DRAIN: begin
          p1v <= (st == PF_RUN); p1i <= m_idx; p1in <= m_in;
          p2v <= p1v; p2i <= p1i; p2in <= p1in; p2c <= popc64(m_rdata);
          if (p2v && p2in && p2c != 7'd0) begin
            if (nslot >= NCELL) err <= ST_CAP;
            nslot <= nslot + 11'd1;
            sum <= sum + {9'd0, p2c};
          end
          if (st == PF_RUN) begin
            k <= k + 16'd1;
            if (k == 16'd32767) st <= PF_DRAIN;
          end else if (!p1v && !p2v) begin
            si <= 0; st <= (err != ST_OK) ? HD_RUN : SC_RD; k <= 16'd0;
          end
        end

        SC_RD: begin
          if (si == C) begin k <= 16'd0; p1v <= 1'b0; st <= CL_RUN; end
          else st <= SC_EVAL;
        end
        SC_EVAL: begin
          sq <= h_rdata[47:0]; sqi <= qidx_of(h_rdata[47:0]);
          if (h_rdata[48]) st <= SC_S1;
          else begin si <= si + 1'b1; st <= (si + 1 == C) ? SC_RD : SC_EVAL; end
        end
        SC_S1: begin        // so_rdata = slot, m_rdata = mask of this query cell
          sslot <= so_rdata;
          srank <= popc64(m_rdata & ((64'd1 << low6_of(sq)) - 64'd1));
          st <= SC_S2;
        end
        SC_S2: begin        // PTS write + AABB update (combinational ports)
          glo <= {umin16(glo[47:32], sq[47:32]), umin16(glo[31:16], sq[31:16]), umin16(glo[15:0], sq[15:0])};
          ghi <= {umax16(ghi[47:32], sq[47:32]), umax16(ghi[31:16], sq[31:16]), umax16(ghi[15:0], sq[15:0])};
          si <= si + 1'b1; st <= SC_RD;
        end

        CL_RUN: begin       // k issues slot reads, p1 writes CELL
          p1v <= (k < nslot); p1i <= k[12:0];
          if (k < nslot) k <= k + 16'd1;
          else if (!p1v) begin k <= 16'd0; st <= HD_RUN; end
        end

        HD_RUN: begin
          k <= k + 16'd1;
          if (k == 16'd8) st <= B_FIN;
        end

        B_FIN: begin
          busy <= 1'b0; done_pulse <= 1'b1; status <= err;
          kept <= (err == ST_OK) ? count : 16'd0;
          ncells <= (err == ST_OK) ? nslot[9:0] : 13'd0;
          dirty <= (err != ST_OK);
          st <= B_IDLE;
        end
        default: st <= B_IDLE;
      endcase
    end
  end
endmodule
