// Construction engine (spec ch.3 / 6.3, scheme A: hash filter -> counting sort -> cell table).
// Bit-exact with soc_model/core.cpp Model::build() for the restored user profile
// (Q14, 3.125 cm fine / 12.5 cm coarse, S=4, strict-less representative update,
// hash = key*2654435761 mod C with linear probing, CAPACITY when kept*4 >= 3C).
//
// Output environment (sorted by fine Morton key, identical to golden):
//   P   [kept]       : {z,y,x} workspace-relative u16 representative points
//   CNT [32768]      : after build, CNT[k] = offsets[k+1]  (offsets[0] = 0)
//   CID [32768]      : dense coarse-cell id, 16'hFFFF = empty   (golden "lookup")
//   SC  [MAX_CELLS]  : {start, count}
//   BOX [MAX_CELLS]  : {hi z,y,x, lo z,y,x}  (tight AABB of representative points)
//   global AABB      : registers
//
// Phases: CLEAR -> FILTER (streaming, single pass over raw points) -> PREFIX
//         (histogram prefix sum, cell id allocation) -> SCATTER (counting sort by
//         coarse key) -> CELLFIX (in-cell order by low 6 key bits + AABBs) -> DONE.
`timescale 1ns/1ps
module build_engine #(
  parameter HASH_AW    = 15,                       // C = 2^HASH_AW (golden: 32768)
  parameter MAX_POINTS = (3 << HASH_AW) / 4,       // golden capacity rule
  parameter PTS_AW     = HASH_AW,
  parameter CELL_AW    = 12,                       // MAX_CELLS = 2^CELL_AW (HW limit, reported as CAPACITY)
  parameter [31:0] RANGE_SQ = 32'd380133009,
  parameter RAM_STYLE  = "block"
) (
  input  wire                clk,
  input  wire                rst,
  input  wire                start,
  input  wire [31:0]         npoints,
  // raw point word stream: x, y, z binary32 words, in order
  input  wire                w_valid,
  output wire                w_ready,
  input  wire [31:0]         w_data,
  // one whole point per beat (x = [31:0], y = [63:32], z = [95:64]); e.g. 128-bit AXI-Stream from DMA
  input  wire                pt_valid,
  output wire                pt_ready,
  input  wire [95:0]         pt_data,
  // status
  output reg                 busy,
  output reg                 env_valid,
  output reg                 done_pulse,
  output reg  [3:0]          status,       // 0 SUCCESS, 4 CAPACITY_EXCEEDED, 5 INVALID_INPUT
  output reg                 cell_overflow,
  output reg  [PTS_AW:0]     kept,
  output reg  [CELL_AW:0]    ncells,
  output reg  [31:0]         culled,
  output reg  [31:0]         probes,
  output reg  [31:0]         cycles,
  output reg  [47:0]         glo,
  output reg  [47:0]         ghi,
  // external read ports (valid when !busy), 1-cycle latency
  input  wire [PTS_AW-1:0]   ext_p_raddr,
  output wire [47:0]         ext_p_rdata,
  input  wire [14:0]         ext_cnt_raddr,
  output wire [15:0]         ext_cnt_rdata,
  input  wire [14:0]         ext_cid_raddr,
  output wire [15:0]         ext_cid_rdata,
  input  wire [CELL_AW-1:0]  ext_cell_raddr,
  output wire [31:0]         ext_sc_rdata,    // {count[15:0], start[15:0]}
  output wire [95:0]         ext_box_rdata    // {hi z,y,x, lo z,y,x}
);
  localparam C = (1 << HASH_AW);
  localparam MAX_CELLS = (1 << CELL_AW);
  localparam [15:0] CLR_LAST = ((C > 32768) ? C : 32768) - 1;
  localparam [3:0] ST_OK = 4'd0, ST_CAP = 4'd4, ST_INV = 4'd5;

  localparam [4:0]
    B_IDLE   = 5'd0,  B_CLEAR  = 5'd1,  F_WAIT  = 5'd2,  F_ISSUE = 5'd3,  F_CMP = 5'd4,
    F_CNTRD  = 5'd5,  F_CNTWR  = 5'd6,  PF_RUN  = 5'd7,  PF_LAST = 5'd8,  SC_RD = 5'd9,
    SC_EVAL  = 5'd10, SC_WR    = 5'd11, CF_RD   = 5'd12, CF_INIT = 5'd13, CF_LOAD = 5'd14,
    CF_ACC   = 5'd15, CF_EMIT  = 5'd16, B_FIN   = 5'd17;
  reg [4:0] st;

  // ---------------- memories ----------------
  // H slot: {valid, d[9:0], z, y, x} = 59 bits
  reg              h_we;   reg [HASH_AW-1:0] h_waddr; reg [58:0] h_wdata;
  reg [HASH_AW-1:0] h_raddr; wire [58:0] h_rdata;
  ram_sdp #(.DW(59), .AW(HASH_AW), .RAM_STYLE(RAM_STYLE)) u_hash (
    .clk(clk), .we(h_we), .waddr(h_waddr), .wdata(h_wdata), .raddr(h_raddr), .rdata(h_rdata));

  reg        cnt_we; reg [14:0] cnt_waddr; reg [15:0] cnt_wdata;
  reg [14:0] cnt_raddr; wire [15:0] cnt_rdata;
  ram_sdp #(.DW(16), .AW(15), .RAM_STYLE(RAM_STYLE)) u_cnt (
    .clk(clk), .we(cnt_we), .waddr(cnt_waddr), .wdata(cnt_wdata), .raddr(cnt_raddr), .rdata(cnt_rdata));

  reg        cid_we; reg [14:0] cid_waddr; reg [15:0] cid_wdata;
  ram_sdp #(.DW(16), .AW(15), .RAM_STYLE(RAM_STYLE)) u_cid (
    .clk(clk), .we(cid_we), .waddr(cid_waddr), .wdata(cid_wdata), .raddr(ext_cid_raddr), .rdata(ext_cid_rdata));

  reg               p_we; reg [PTS_AW-1:0] p_waddr; reg [47:0] p_wdata;
  reg [PTS_AW-1:0]  p_raddr; wire [47:0] p_rdata;
  ram_sdp #(.DW(48), .AW(PTS_AW), .DEPTH(MAX_POINTS), .RAM_STYLE(RAM_STYLE)) u_pts (
    .clk(clk), .we(p_we), .waddr(p_waddr), .wdata(p_wdata), .raddr(p_raddr), .rdata(p_rdata));
  assign ext_p_rdata = p_rdata;

  reg               sc_we; reg [CELL_AW-1:0] sc_waddr; reg [31:0] sc_wdata;
  reg [CELL_AW-1:0] sc_raddr; wire [31:0] sc_rdata;
  ram_sdp #(.DW(32), .AW(CELL_AW), .RAM_STYLE(RAM_STYLE)) u_sc (
    .clk(clk), .we(sc_we), .waddr(sc_waddr), .wdata(sc_wdata), .raddr(sc_raddr), .rdata(sc_rdata));
  assign ext_sc_rdata = sc_rdata;

  reg               box_we; reg [CELL_AW-1:0] box_waddr; reg [95:0] box_wdata;
  ram_sdp #(.DW(96), .AW(CELL_AW), .RAM_STYLE(RAM_STYLE)) u_box (
    .clk(clk), .we(box_we), .waddr(box_waddr), .wdata(box_wdata), .raddr(ext_cell_raddr), .rdata(ext_box_rdata));

  assign ext_cnt_rdata = {1'b0, cnt_rdata[14:0]};

  // ---------------- word assembler + stage A ----------------
  reg  [31:0] wx, wy;
  reg  [1:0]  wcnt;
  reg  [31:0] words_left;      // points still to be received
  wire        in_filter = (st == F_WAIT) || (st == F_CMP);
  wire        sa_in_ready;
  wire        use_pt = pt_valid && (wcnt == 2'd0);
  assign pt_ready = in_filter && (words_left != 0) && (wcnt == 2'd0) && sa_in_ready;
  assign w_ready = in_filter && (words_left != 0) && !use_pt && ((wcnt != 2'd2) || sa_in_ready);
  wire        sa_in_valid = in_filter && (words_left != 0) && (use_pt || (w_valid && (wcnt == 2'd2)));

  wire        sa_valid, sa_abort, sa_cull;
  wire [15:0] sa_x, sa_y, sa_z;
  wire [20:0] sa_key;
  wire [9:0]  sa_d;
  wire [15:0] sa_hash;
  wire        sa_ready = (st == F_WAIT);
  stage_a #(.RANGE_SQ(RANGE_SQ)) u_sa (
    .clk(clk), .rst(rst || st == B_IDLE),
    .in_valid(sa_in_valid), .in_ready(sa_in_ready), .in_x(use_pt ? pt_data[31:0] : wx), .in_y(use_pt ? pt_data[63:32] : wy), .in_z(use_pt ? pt_data[95:64] : w_data),
    .out_valid(sa_valid), .out_ready(sa_ready), .out_abort(sa_abort), .out_cull(sa_cull),
    .out_x(sa_x), .out_y(sa_y), .out_z(sa_z), .out_key(sa_key), .out_d(sa_d), .out_hash(sa_hash));

  // ---------------- filter registers ----------------
  reg [31:0]        consumed, npts;
  reg [3:0]         err;
  reg [47:0]        cur_p;       // {z,y,x}
  reg [9:0]         cur_d;
  reg [14:0]        cur_coarse;
  reg [HASH_AW-1:0] probe;
  reg [HASH_AW:0]   tries;
  reg [PTS_AW:0]    count;       // distinct fine voxels kept
  // epoch-tagged coarse counters: CNT[k] = {tag, value}; value is valid only if tag == epoch.
  // Avoids clearing the 32768-entry table every build (a full clear only after reset / a failed build).
  reg               epoch, dirty;
  reg               cnt_pend;       // pending count increment (read issued in F_CMP, written next cycle)
  wire [15:0]       cval = (cnt_rdata[15] == epoch) ? {1'b0, cnt_rdata[14:0]} : 16'd0;

  wire        hv     = h_rdata[58];
  wire [9:0]  hd     = h_rdata[57:48];
  wire [47:0] hp     = h_rdata[47:0];
  // fine coords equal <=> same fine voxel (key is a bijection of fine coords)
  wire        hmatch = (hp[47:41] == cur_p[47:41]) && (hp[31:25] == cur_p[31:25]) && (hp[15:9] == cur_p[15:9]);

  function [14:0] coarse_of;  // coarse Morton key (= fine key >> 6) from {z,y,x}
    input [47:0] p;
    begin
      // Direct concatenation is the same interleave as the loop above and is accepted by
      // both iverilog and Verilator when this engine is integrated with SystemVerilog CSE RTL.
      coarse_of = {p[47], p[31], p[15], p[46], p[30], p[14], p[45], p[29], p[13],
                   p[44], p[28], p[12], p[43], p[27], p[11]};
    end
  endfunction

  // ---------------- prefix / scatter / cellfix registers ----------------
  reg [15:0]        k;            // 0..32768
  reg               pf_v;
  reg [14:0]        pf_k;
  reg [15:0]        sum;
  reg [HASH_AW:0]   si;
  reg [58:0]        sh;           // latched hash entry during scatter
  reg [CELL_AW:0]   ci;
  reg [15:0]        c_start;
  reg [6:0]         c_count, cj, cw;
  reg [63:0]        vmask;
  reg [47:0]        scratch [0:63];
  reg [47:0]        blo, bhi;
  reg [5:0]         ffs;
  wire [5:0]        p_low6 = {p_rdata[42], p_rdata[26], p_rdata[10],
                             p_rdata[41], p_rdata[25], p_rdata[9]};
  integer           fi;
  always @* begin
    ffs = 6'd0;
    for (fi = 63; fi >= 0; fi = fi - 1) if (vmask[fi]) ffs = fi[5:0];
  end

  function [15:0] umin; input [15:0] a, b; umin = (a < b) ? a : b; endfunction
  function [15:0] umax; input [15:0] a, b; umax = (a > b) ? a : b; endfunction

  // ---------------- combinational memory port control ----------------
  always @* begin
    h_we = 1'b0; h_waddr = probe; h_wdata = {1'b1, cur_d, cur_p};
    h_raddr = si[HASH_AW-1:0];
    cnt_we = 1'b0; cnt_waddr = cur_coarse; cnt_wdata = {epoch, cval[14:0] + 15'd1};
    cnt_raddr = ext_cnt_raddr;
    cid_we = 1'b0; cid_waddr = pf_k; cid_wdata = 16'hFFFF;
    p_we = 1'b0; p_waddr = cnt_rdata[PTS_AW-1:0]; p_wdata = sh[47:0];
    p_raddr = ext_p_raddr;
    sc_we = 1'b0; sc_waddr = ncells[CELL_AW-1:0]; sc_wdata = {16'd0, sum};
    sc_raddr = ext_cell_raddr;
    box_we = 1'b0; box_waddr = ci[CELL_AW-1:0]; box_wdata = {bhi, blo};
    case (st)
      B_CLEAR: begin
        h_we = (k < C); h_waddr = k[HASH_AW-1:0]; h_wdata = 59'd0;
        cnt_we = 1'b1; cnt_waddr = k[14:0]; cnt_wdata = {~epoch, 15'd0};
      end
      F_WAIT: h_raddr = sa_hash[HASH_AW-1:0];          // probe read issued as the point is accepted
      F_CMP: begin
        if (err == ST_OK && !hv && !(count * 4 >= 3 * C)) begin h_we = 1'b1; cnt_raddr = cur_coarse; end
        else if (err == ST_OK && hv && hmatch && (cur_d < hd)) h_we = 1'b1;
        else if (hv && !hmatch) h_raddr = probe + 1'b1;   // linear probing, 1 cycle per extra probe
      end
      PF_RUN, PF_LAST: begin
        cnt_raddr = k[14:0];
        if (pf_v) begin
          cnt_we = 1'b1; cnt_waddr = pf_k; cnt_wdata = {epoch, sum[14:0]};
          cid_we = 1'b1;
          if (cval != 16'd0) begin
            cid_wdata = {{(16-CELL_AW-1){1'b0}}, ncells};
            if (ncells < MAX_CELLS) begin
              sc_we = 1'b1; sc_wdata = {cval, sum};
            end
          end
        end
      end
      SC_EVAL: begin
        cnt_raddr = coarse_of(h_rdata[47:0]);
        h_we = 1'b1; h_waddr = si[HASH_AW-1:0]; h_wdata = 59'd0;     // leave the hash table clean
        if (!h_rdata[58]) h_raddr = si[HASH_AW-1:0] + 1'b1;           // empty slot: 1 cycle
      end
      SC_WR: begin
        p_we = 1'b1; p_waddr = cnt_rdata[PTS_AW-1:0]; p_wdata = sh[47:0];
        cnt_we = 1'b1; cnt_waddr = coarse_of(sh[47:0]); cnt_wdata = {epoch, cnt_rdata[14:0] + 15'd1};
      end
      CF_RD: sc_raddr = ci[CELL_AW-1:0];
      CF_LOAD: p_raddr = c_start[PTS_AW-1:0] + cj;
      CF_EMIT: begin
        if (vmask != 64'd0) begin
          p_we = 1'b1; p_waddr = c_start[PTS_AW-1:0] + cw; p_wdata = scratch[ffs];
        end else begin
          box_we = 1'b1;
        end
      end
      default: ;
    endcase
    if (cnt_pend) cnt_we = 1'b1;     // waddr = cur_coarse, wdata = {epoch, cval+1}
  end

  // ---------------- main FSM ----------------
  always @(posedge clk) begin
    if (rst) begin
      st <= B_IDLE; busy <= 1'b0; env_valid <= 1'b0; done_pulse <= 1'b0; status <= ST_OK;
      cell_overflow <= 1'b0; kept <= 0; ncells <= 0; culled <= 0; probes <= 0; cycles <= 0;
      wcnt <= 2'd0; words_left <= 0; err <= ST_OK;
      glo <= 48'd0; ghi <= 48'd0; epoch <= 1'b0; dirty <= 1'b1; cnt_pend <= 1'b0;
    end else begin
      done_pulse <= 1'b0;
      cnt_pend <= 1'b0;
      if (busy) cycles <= cycles + 32'd1;

      // word assembler
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
          busy <= 1'b1; env_valid <= 1'b0; status <= ST_OK; cell_overflow <= 1'b0;
          kept <= 0; ncells <= 0; culled <= 0; probes <= 0; cycles <= 0; err <= ST_OK;
          count <= 0; consumed <= 0; npts <= npoints; words_left <= npoints; wcnt <= 2'd0;
          glo <= {48{1'b1}}; ghi <= 48'd0;
          k <= 16'd0; epoch <= ~epoch;
          st <= dirty ? B_CLEAR : F_WAIT;      // tables are already clean after a successful build
        end

        B_CLEAR: begin
          k <= k + 16'd1;
          // clear max(C, 32768) entries: the hash table AND the full 32768-entry coarse count table
          if (k == CLR_LAST) st <= F_WAIT;
        end

        F_WAIT: begin
          if (consumed == npts) begin
            if (err != ST_OK) st <= B_FIN;
            else begin k <= 16'd0; pf_v <= 1'b0; sum <= 16'd0; st <= PF_RUN; end
          end else if (sa_valid) begin
            consumed <= consumed + 32'd1;
            if (err != ST_OK) begin
              // draining after an error (golden returns at the first error)
            end else if (sa_abort) begin
              err <= ST_INV;
            end else if (sa_cull) begin
              culled <= culled + 32'd1;
            end else begin
              cur_p <= {sa_z, sa_y, sa_x};
              cur_d <= sa_d;
              cur_coarse <= sa_key[20:6];
              probe <= sa_hash[HASH_AW-1:0];
              tries <= 0;
              st <= F_CMP;
            end
          end
        end

        F_CMP: begin
          probes <= probes + 32'd1;
          tries <= tries + 1'b1;
          if (!hv) begin
            if (count * 4 >= 3 * C) begin
              err <= ST_CAP; st <= F_WAIT;
            end else begin
              count <= count + 1'b1; cnt_pend <= 1'b1; st <= F_WAIT;
            end
          end else if (hmatch) begin
            st <= F_WAIT;
          end else begin
            probe <= probe + 1'b1;
            if (tries + 1 >= C) begin err <= ST_CAP; st <= F_WAIT; end
          end
        end

        // prefix sum over 32768 coarse counters; k issues reads, pf_k is the returning entry
        PF_RUN: begin
          pf_v <= 1'b1; pf_k <= k[14:0];
          k <= k + 16'd1;
          if (pf_v) begin
            sum <= sum + cval;
            if (cval != 16'd0) begin
              if (ncells >= MAX_CELLS) begin err <= ST_CAP; cell_overflow <= 1'b1; end
              else ncells <= ncells + 1'b1;
            end
          end
          if (k == 16'd32767) st <= PF_LAST;
        end
        PF_LAST: begin
          pf_v <= 1'b0;
          sum <= sum + cval;
          if (cval != 16'd0) begin
            if (ncells >= MAX_CELLS) begin err <= ST_CAP; cell_overflow <= 1'b1; end
            else ncells <= ncells + 1'b1;
          end
          si <= 0;
          st <= SC_RD;
        end

        SC_RD: begin
          if (err != ST_OK) st <= B_FIN;
          else if (si == C) begin ci <= 0; st <= (ncells == 0) ? B_FIN : CF_RD; end
          else st <= SC_EVAL;
        end
        SC_EVAL: begin
          sh <= h_rdata;
          if (h_rdata[58]) st <= SC_WR;
          else begin si <= si + 1'b1; st <= (si + 1 == C) ? SC_RD : SC_EVAL; end
        end
        SC_WR: begin si <= si + 1'b1; st <= SC_RD; end

        CF_RD: st <= CF_INIT;
        CF_INIT: begin
          c_start <= sc_rdata[15:0];
          c_count <= sc_rdata[22:16];
          cj <= 7'd0; cw <= 7'd0; vmask <= 64'd0;
          blo <= {48{1'b1}}; bhi <= 48'd0;
          st <= CF_LOAD;
        end
        CF_LOAD: begin
          if (cj == c_count) st <= CF_EMIT;
          else begin cj <= cj + 7'd1; st <= CF_ACC; end
        end
        CF_ACC: begin
          scratch[p_low6] <= p_rdata;
          vmask[p_low6] <= 1'b1;
          blo <= {umin(blo[47:32], p_rdata[47:32]), umin(blo[31:16], p_rdata[31:16]), umin(blo[15:0], p_rdata[15:0])};
          bhi <= {umax(bhi[47:32], p_rdata[47:32]), umax(bhi[31:16], p_rdata[31:16]), umax(bhi[15:0], p_rdata[15:0])};
          st <= CF_LOAD;
        end
        CF_EMIT: begin
          if (vmask != 64'd0) begin
            vmask[ffs] <= 1'b0;
            cw <= cw + 7'd1;
          end else begin
            glo <= {umin(glo[47:32], blo[47:32]), umin(glo[31:16], blo[31:16]), umin(glo[15:0], blo[15:0])};
            ghi <= {umax(ghi[47:32], bhi[47:32]), umax(ghi[31:16], bhi[31:16]), umax(ghi[15:0], bhi[15:0])};
            ci <= ci + 1'b1;
            st <= (ci + 1 == ncells) ? B_FIN : CF_RD;
          end
        end

        B_FIN: begin
          busy <= 1'b0; done_pulse <= 1'b1;
          status <= err;
          env_valid <= (err == ST_OK);
          kept <= (err == ST_OK) ? count : 0;
          dirty <= (err != ST_OK);       // failed build: tables not reset by scatter -> full clear next time
          if (err != ST_OK) ncells <= 0;
          st <= B_IDLE;
        end
        default: st <= B_IDLE;
      endcase
    end
  end
endmodule
