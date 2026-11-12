// VCC Morton accelerator core: build engine + collision query engine + fixed-point FK
// + RRT-Connect planner behind one simple 32-bit register bus (and an optional
// AXI-Stream style point input for DMA).  Board wrappers: vcc_uart_top, vcc_axil_top.
//
// Byte address map (bus_addr):
//   0x000 CTRL     W  bit0 build  bit1 query  bit2 fk  bit3 plan   (self-clearing)
//   0x004 STATUS   R  [0] build_busy [1] env_valid [7:4] build_status [8] q_busy [9] q_hit
//                     [12] fk_busy [16] plan_busy [23:20] plan_status [24] cell_overflow
//   0x008 NPOINTS  RW points expected by the next build
//   0x00C KEPT  0x010 NCELLS  0x014 CULLED  0x018 BUILD_CYCLES  0x01C PROBES          R
//   0x020 GLO {y,x} 0x024 GLO {-,z} 0x028 GHI {y,x} 0x02C GHI {-,z}                  R
//   0x030 Q_XY {cy,cx}   0x034 Q_ZR {radius,cz}  (writing 0x034 starts a query)      RW
//   0x038 Q_RESULT R  [0] hit [1] busy [31:8] cycles
//   0x03C PT_WORD  W  push one binary32 word of the raw point stream (x,y,z,x,y,z,...)
//   0x040..0x058 Q0..Q6   RW joint angles (Q24) for FK / plan start
//   0x060..0x078 G0..G6   RW plan goal (Q24)
//   0x080 RANGE 0x084 MAX_NODES 0x088 MAX_ITER 0x08C BUDGET 0x090 SEED 0x094 GOAL_BIAS 0x098 MARGIN  RW
//   0x0A0 PATH_LEN 0x0A4 ITERS 0x0A8 NODES 0x0AC STATES 0x0B0 EDGES 0x0B4 QUERIES 0x0B8 PLAN_CYCLES R
//   0x0BC FK_CYCLES R      0x0FC ID R
//   0x100000 + 8*i (+4)       points P[i]      {y,x} / {0,z}   (workspace-relative u16 Q14)
//   0x200000 + 4*k            offsets[k], k = 0..32768
//   0x300000 + 4*k            cell id of coarse key k (0xFFFF empty)
//   0x400000 + 16*c (+4,8,12) cell c  {count,start} {lo_y,lo_x} {hi_x,lo_z} {hi_z,hi_y}
//   0x500000 + 8*s (+4)       FK sphere centre s {y,x} / {0,z}
//   0x600000 + 32*i + 4*j     path waypoint i, joint j (Q24)
// Memory windows are only valid while the engines are idle.
`timescale 1ns/1ps
module vcc_core #(
  parameter HASH_AW = 15,
  parameter CELL_AW = 12,
  parameter MAXN_AW = 10,
  parameter RAM_STYLE = "block"
) (
  input  wire        clk,
  input  wire        rst,
  // register bus: one request per cycle while !busy; ack pulses with rdata
  input  wire        bus_req,
  input  wire        bus_we,
  input  wire [23:0] bus_addr,
  input  wire [31:0] bus_wdata,
  output reg         bus_ack,
  output reg  [31:0] bus_rdata,
  // optional streaming point input (e.g. AXI DMA -> AXI-Stream), 32-bit words
  input  wire        s_valid,
  output wire        s_ready,
  input  wire [127:0] s_data,     // one point per beat: x [31:0], y [63:32], z [95:64] (binary32)
  output wire        irq
);
  localparam PTS_AW = HASH_AW;
  localparam PATH_AW = MAXN_AW + 1;

  // ---------------- registers ----------------
  reg [31:0] npoints;
  reg [15:0] qx, qy, qz, qr;
  reg [31:0] jq [0:6];
  reg [31:0] gq [0:6];
  reg [31:0] c_range, c_iter, c_budget, c_seed, c_bias;
  reg [15:0] c_nodes, c_margin;
  reg        go_build, go_query, go_fk, go_plan;

  wire [223:0] q_pack = {jq[6], jq[5], jq[4], jq[3], jq[2], jq[1], jq[0]};
  wire [223:0] g_pack = {gq[6], gq[5], gq[4], gq[3], gq[2], gq[1], gq[0]};

  // ---------------- build engine ----------------
  wire        b_busy, env_valid, b_done, cell_overflow;
  wire [3:0]  b_status;
  wire [PTS_AW:0] kept; wire [CELL_AW:0] ncells;
  wire [31:0] culled, probes, b_cycles;
  wire [47:0] glo, ghi;
  reg  [PTS_AW-1:0] h_p_raddr; reg [14:0] h_cnt_raddr, h_cid_raddr; reg [CELL_AW-1:0] h_cell_raddr;
  wire [47:0] p_rdata; wire [15:0] cnt_rdata, cid_rdata; wire [31:0] sc_rdata; wire [95:0] box_rdata;
  wire [PTS_AW-1:0] q_p_raddr; wire [14:0] q_cid_raddr; wire [CELL_AW-1:0] q_cell_raddr;
  wire        q_busy;

  reg         pt_pending;
  reg  [31:0] pt_word;
  wire        w_ready;
  wire        w_valid = pt_pending;
  wire [31:0] w_data  = pt_word;

  build_engine #(.HASH_AW(HASH_AW), .CELL_AW(CELL_AW), .PTS_AW(PTS_AW), .RAM_STYLE(RAM_STYLE)) u_build (
    .clk(clk), .rst(rst), .start(go_build), .npoints(npoints),
    .w_valid(w_valid), .w_ready(w_ready), .w_data(w_data),
    .pt_valid(s_valid), .pt_ready(s_ready), .pt_data(s_data[95:0]),
    .busy(b_busy), .env_valid(env_valid), .done_pulse(b_done), .status(b_status), .cell_overflow(cell_overflow),
    .kept(kept), .ncells(ncells), .culled(culled), .probes(probes), .cycles(b_cycles), .glo(glo), .ghi(ghi),
    .ext_p_raddr(q_busy ? q_p_raddr : h_p_raddr), .ext_p_rdata(p_rdata),
    .ext_cnt_raddr(h_cnt_raddr), .ext_cnt_rdata(cnt_rdata),
    .ext_cid_raddr(q_busy ? q_cid_raddr : h_cid_raddr), .ext_cid_rdata(cid_rdata),
    .ext_cell_raddr(q_busy ? q_cell_raddr : h_cell_raddr), .ext_sc_rdata(sc_rdata), .ext_box_rdata(box_rdata));

  // ---------------- planner ----------------
  wire        p_busy, p_done, p_fk_start, p_qe_start;
  wire [3:0]  p_status;
  wire [PATH_AW:0] path_len;
  reg  [PATH_AW-1:0] h_path_raddr;
  wire [223:0] path_rdata, p_fk_q;
  wire [31:0] st_iters, st_nodes, st_states, st_edges, st_queries, p_cycles;
  wire [47:0] p_qe_c; wire [15:0] p_qe_r;
  wire        fk_busy, fk_done, fk_we; wire [5:0] fk_idx; wire [47:0] fk_xyz; wire [31:0] fk_cycles;
  wire        q_done, q_hit; wire [31:0] q_cycles, q_cells, q_dist;

  rrt_engine #(.MAXN_AW(MAXN_AW)) u_rrt (
    .clk(clk), .rst(rst), .start(go_plan), .q_start(q_pack), .q_goal(g_pack),
    .cfg_range(c_range), .cfg_max_nodes(c_nodes), .cfg_max_iter(c_iter), .cfg_budget(c_budget),
    .cfg_seed(c_seed), .cfg_goal_bias(c_bias), .cfg_margin(c_margin),
    .busy(p_busy), .done_pulse(p_done), .status(p_status), .path_len(path_len),
    .path_raddr(h_path_raddr), .path_rdata(path_rdata),
    .st_iters(st_iters), .st_nodes(st_nodes), .st_states(st_states), .st_edges(st_edges),
    .st_queries(st_queries), .cycles(p_cycles),
    .fk_start(p_fk_start), .fk_q(p_fk_q), .fk_done(fk_done), .fk_we(fk_we), .fk_idx(fk_idx), .fk_xyz(fk_xyz),
    .qe_start(p_qe_start), .qe_c(p_qe_c), .qe_r(p_qe_r), .qe_done(q_done), .qe_hit(q_hit),
    .env_ok(env_valid && kept != 0), .env_glo(glo), .env_ghi(ghi));

  // ---------------- FK engine (shared host / planner) ----------------
  fk_engine u_fk (
    .clk(clk), .rst(rst), .start(go_fk | p_fk_start), .q(p_busy ? p_fk_q : q_pack),
    .busy(fk_busy), .done_pulse(fk_done), .cycles(fk_cycles),
    .out_we(fk_we), .out_idx(fk_idx), .out_xyz(fk_xyz));
  reg [47:0] fk_buf [0:63];
  always @(posedge clk) if (fk_we) fk_buf[fk_idx] <= fk_xyz;

  // ---------------- query engine (shared host / planner) ----------------
  query_engine #(.PTS_AW(PTS_AW), .CELL_AW(CELL_AW)) u_q (
    .clk(clk), .rst(rst), .start(go_query | p_qe_start),
    .cx(p_busy ? p_qe_c[15:0] : qx), .cy(p_busy ? p_qe_c[31:16] : qy), .cz(p_busy ? p_qe_c[47:32] : qz),
    .radius(p_busy ? p_qe_r : qr),
    .env_valid(env_valid), .kept(kept), .glo(glo), .ghi(ghi),
    .busy(q_busy), .done_pulse(q_done), .hit(q_hit), .cycles(q_cycles), .cells_visited(q_cells), .dist_tests(q_dist),
    .cid_raddr(q_cid_raddr), .cid_rdata(cid_rdata), .cell_raddr(q_cell_raddr), .sc_rdata(sc_rdata), .box_rdata(box_rdata),
    .p_raddr(q_p_raddr), .p_rdata(p_rdata));

  assign irq = b_done | p_done;

  // ---------------- bus ----------------
  localparam [2:0] BI = 3'd0, BRD1 = 3'd1, BRD2 = 3'd2, BPT = 3'd3;
  reg [2:0]  bst;
  reg [23:0] a;
  wire [3:0] win = a[23:20];
  wire [7:0] reg_a = a[7:0];
  reg [31:0] reg_rd;
  integer i;

  always @* begin
    case (reg_a)
      8'h04: reg_rd = {7'd0, cell_overflow, p_status, 3'd0, p_busy, 3'd0, fk_busy, 2'd0, q_hit, q_busy, b_status, 2'd0, env_valid, b_busy};
      8'h08: reg_rd = npoints;
      8'h0C: reg_rd = kept;
      8'h10: reg_rd = ncells;
      8'h14: reg_rd = culled;
      8'h18: reg_rd = b_cycles;
      8'h1C: reg_rd = probes;
      8'h20: reg_rd = glo[31:0];
      8'h24: reg_rd = {16'd0, glo[47:32]};
      8'h28: reg_rd = ghi[31:0];
      8'h2C: reg_rd = {16'd0, ghi[47:32]};
      8'h30: reg_rd = {qy, qx};
      8'h34: reg_rd = {qr, qz};
      8'h38: reg_rd = {q_cycles[23:0], 6'd0, q_busy, q_hit};
      8'h40, 8'h44, 8'h48, 8'h4C, 8'h50, 8'h54, 8'h58: reg_rd = jq[(reg_a - 8'h40) >> 2];
      8'h60, 8'h64, 8'h68, 8'h6C, 8'h70, 8'h74, 8'h78: reg_rd = gq[(reg_a - 8'h60) >> 2];
      8'h80: reg_rd = c_range;
      8'h84: reg_rd = c_nodes;
      8'h88: reg_rd = c_iter;
      8'h8C: reg_rd = c_budget;
      8'h90: reg_rd = c_seed;
      8'h94: reg_rd = c_bias;
      8'h98: reg_rd = c_margin;
      8'hA0: reg_rd = path_len;
      8'hA4: reg_rd = st_iters;
      8'hA8: reg_rd = st_nodes;
      8'hAC: reg_rd = st_states;
      8'hB0: reg_rd = st_edges;
      8'hB4: reg_rd = st_queries;
      8'hB8: reg_rd = p_cycles;
      8'hBC: reg_rd = fk_cycles;
      8'hFC: reg_rd = {8'h5C, 4'd1, HASH_AW[3:0], 4'd0, CELL_AW[3:0], 4'd0, MAXN_AW[3:0]};
      default: reg_rd = 32'd0;
    endcase
  end

  // memory read addresses (presented during BRD1)
  always @* begin
    h_p_raddr    = a[PTS_AW+2:3];
    h_cnt_raddr  = a[16:2] - 15'd1;          // offsets[k] = CNT[k-1]
    h_cid_raddr  = a[16:2];
    h_cell_raddr = a[CELL_AW+3:4];
    h_path_raddr = a[PATH_AW+4:5];
  end

  reg [31:0] mem_rd;
  wire [47:0] fkv = fk_buf[a[8:3]];
  wire [31:0] pathw = path_rdata[32*a[4:2] +: 32];
  always @* begin
    case (win)
      4'h1: mem_rd = a[2] ? {16'd0, p_rdata[47:32]} : p_rdata[31:0];
      4'h2: mem_rd = (a[19:2] == 18'd0) ? 32'd0 : {16'd0, cnt_rdata};
      4'h3: mem_rd = {16'd0, cid_rdata};
      4'h4: case (a[3:2])
              2'd0: mem_rd = sc_rdata;
              2'd1: mem_rd = box_rdata[31:0];
              2'd2: mem_rd = box_rdata[63:32];
              default: mem_rd = box_rdata[95:64];
            endcase
      4'h5: mem_rd = a[2] ? {16'd0, fkv[47:32]} : fkv[31:0];
      4'h6: mem_rd = (a[4:2] == 3'd7) ? 32'd0 : pathw;
      default: mem_rd = reg_rd;
    endcase
  end

  always @(posedge clk) begin
    if (rst) begin
      bst <= BI; bus_ack <= 1'b0; pt_pending <= 1'b0;
      go_build <= 0; go_query <= 0; go_fk <= 0; go_plan <= 0;
      npoints <= 0; qx <= 0; qy <= 0; qz <= 0; qr <= 0;
      for (i = 0; i < 7; i = i + 1) begin jq[i] <= 0; gq[i] <= 0; end
      c_range <= 32'd5872025; c_nodes <= 16'd4096; c_iter <= 32'd300; c_budget <= 32'd30000;
      c_seed <= 32'd1; c_bias <= 32'd0; c_margin <= 16'd1;
    end else begin
      bus_ack <= 1'b0;
      go_build <= 0; go_query <= 0; go_fk <= 0; go_plan <= 0;
      if (pt_pending && w_ready) pt_pending <= 1'b0;
      case (bst)
        BI: if (bus_req) begin
          a <= bus_addr;
          if (bus_we) begin
            if (bus_addr[23:20] == 4'h0) begin
              case (bus_addr[7:0])
                8'h00: begin go_build <= bus_wdata[0]; go_query <= bus_wdata[1]; go_fk <= bus_wdata[2]; go_plan <= bus_wdata[3]; end
                8'h08: npoints <= bus_wdata;
                8'h30: begin qx <= bus_wdata[15:0]; qy <= bus_wdata[31:16]; end
                8'h34: begin qz <= bus_wdata[15:0]; qr <= bus_wdata[31:16]; go_query <= 1'b1; end
                8'h3C: begin pt_word <= bus_wdata; pt_pending <= 1'b1; end
                8'h40, 8'h44, 8'h48, 8'h4C, 8'h50, 8'h54, 8'h58: jq[(bus_addr[7:0] - 8'h40) >> 2] <= bus_wdata;
                8'h60, 8'h64, 8'h68, 8'h6C, 8'h70, 8'h74, 8'h78: gq[(bus_addr[7:0] - 8'h60) >> 2] <= bus_wdata;
                8'h80: c_range <= bus_wdata;
                8'h84: c_nodes <= bus_wdata[15:0];
                8'h88: c_iter <= bus_wdata;
                8'h8C: c_budget <= bus_wdata;
                8'h90: c_seed <= bus_wdata;
                8'h94: c_bias <= bus_wdata;
                8'h98: c_margin <= bus_wdata[15:0];
                default: ;
              endcase
            end
            if (bus_addr[23:20] == 4'h0 && bus_addr[7:0] == 8'h3C) bst <= BPT;
            else bus_ack <= 1'b1;
          end else bst <= BRD1;
        end
        BPT: if (!pt_pending || w_ready) begin bus_ack <= 1'b1; bst <= BI; end
        BRD1: bst <= BRD2;                  // memory address presented, data next cycle
        BRD2: begin bus_rdata <= mem_rd; bus_ack <= 1'b1; bst <= BI; end
        default: bst <= BI;
      endcase
    end
  end
endmodule
