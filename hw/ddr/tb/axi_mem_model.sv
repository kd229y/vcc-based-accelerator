// axi_mem_model: AXI4 slave memory for the DDR testbenches (128-bit data, INCR only, in-order).
// Memory starts with ddr_tb_pkg::init_beat (non-zero); writes obey WSTRB.
// Random gaps (fixed seed): AWREADY / WREADY / ARREADY low and RVALID gaps with probability GAP percent.
// Read data comes LAT cycles after AR; B comes BLAT cycles after WLAST. W is accepted only after its AW.
// The model also checks the slave-visible rules: WLAST position, size, burst type, 4 KB, address window.
`timescale 1ns/1ps
module axi_mem_model #(
  parameter int          AW       = 32,
  parameter logic [AW-1:0] BASE   = '0,
  parameter int          MEM_BYTES = 1 << 27,
  parameter int          ID_W     = 4
) (
  input  logic          clk,
  input  logic          rst,
  input  int            gap,       // percent 0..99
  input  int            lat,       // read latency, cycles >= 1
  input  int            blat,      // write response latency, cycles >= 1
  input  logic [31:0]   seed,
  input  logic          inj_slverr, // answer every response with SLVERR while high
  input  logic [AW-1:0] s_awaddr,
  input  logic [7:0]    s_awlen,
  input  logic [2:0]    s_awsize,
  input  logic [1:0]    s_awburst,
  input  logic          s_awvalid,
  output logic          s_awready,
  input  logic [127:0]  s_wdata,
  input  logic [15:0]   s_wstrb,
  input  logic          s_wlast,
  input  logic          s_wvalid,
  output logic          s_wready,
  output logic [ID_W-1:0] s_bid,
  output logic [1:0]    s_bresp,
  output logic          s_bvalid,
  input  logic          s_bready,
  input  logic [AW-1:0] s_araddr,
  input  logic [7:0]    s_arlen,
  input  logic [2:0]    s_arsize,
  input  logic [1:0]    s_arburst,
  input  logic          s_arvalid,
  output logic          s_arready,
  output logic [ID_W-1:0] s_rid,
  output logic [127:0]  s_rdata,
  output logic [1:0]    s_rresp,
  output logic          s_rlast,
  output logic          s_rvalid,
  input  logic          s_rready,
  output int            errors
);
  import ddr_tb_pkg::*;
  typedef struct { logic [31:0] beat; int len; longint t; } burst_t;

  logic [127:0] mem [logic [31:0]];
  burst_t aw_q[$], ar_q[$];
  longint b_q[$];
  longint cyc;
  logic [31:0] rs;
  int w_beat, r_beat;

  // if/else, not ?: -- Verilator evaluates both arms and the mem[b] read creates a zero entry
  function automatic logic [127:0] rd_beat(input logic [31:0] b);
    if (mem.exists(b)) return mem[b];
    return init_beat(b);
  endfunction

  // testbench access (Verilator does not handle hierarchical reads of the associative array)
  function automatic logic [127:0] peek(input logic [31:0] b); return rd_beat(b); endfunction
  function automatic void poke(input logic [31:0] b, input logic [127:0] v); mem[b] = v; endfunction

  function automatic logic coin();   // 1 = stall this cycle
    rs = xs32(rs);
    return (rs % 100) < gap;
  endfunction

  function automatic void check_cmd(string ch, logic [AW-1:0] a, logic [7:0] len, logic [2:0] sz, logic [1:0] bt);
    logic [AW-1:0] off = a - BASE;
    if (sz != 3'd4)  begin errors++; $display("ERROR %s size=%0d", ch, sz); end
    if (bt != 2'b01) begin errors++; $display("ERROR %s burst=%0d", ch, bt); end
    if (a[3:0] != 0) begin errors++; $display("ERROR %s addr %h not 16 B aligned", ch, a); end
    if (a < BASE || {32'd0, off} + (len + 1) * 16 > MEM_BYTES) begin errors++; $display("ERROR %s addr %h len %0d outside window", ch, a, len); end
    if ({20'd0, a[11:0]} + (len + 1) * 16 > 4096) begin errors++; $display("ERROR %s addr %h len %0d crosses 4 KB", ch, a, len + 1); end
  endfunction

  assign s_bid = '0;
  assign s_rid = '0;

  always_ff @(posedge clk) begin
    if (rst) begin
      s_awready <= 0; s_wready <= 0; s_arready <= 0; s_bvalid <= 0; s_rvalid <= 0;
      aw_q.delete(); ar_q.delete(); b_q.delete(); cyc <= 0; rs = seed | 1; w_beat = 0; r_beat = 0; errors = 0;
    end else begin
      cyc <= cyc + 1;
      // AW
      if (s_awvalid && s_awready) begin
        check_cmd("AW", s_awaddr, s_awlen, s_awsize, s_awburst);
        aw_q.push_back('{beat: 32'((s_awaddr - BASE) >> 4), len: int'(s_awlen) + 1, t: 0});
      end
      s_awready <= !coin();
      // W (only after its AW)
      if (s_wvalid && s_wready) begin
        if (aw_q.size() == 0) begin errors++; $display("ERROR W beat with no AW"); end
        else begin
          logic [31:0] b = aw_q[0].beat + w_beat;
          logic [127:0] v = rd_beat(b);
          for (int k = 0; k < 16; k++) if (s_wstrb[k]) v[8*k +: 8] = s_wdata[8*k +: 8];
          mem[b] = v;
          if (s_wlast != (w_beat == aw_q[0].len - 1)) begin
            errors++; $display("ERROR WLAST=%0d at beat %0d of %0d", s_wlast, w_beat, aw_q[0].len);
          end
          if (w_beat == aw_q[0].len - 1) begin
            b_q.push_back(cyc + blat); void'(aw_q.pop_front()); w_beat = 0;
          end else w_beat++;
        end
      end
      s_wready <= (aw_q.size() != 0 || (s_awvalid && s_awready)) && !coin();
      // B
      if (s_bvalid && s_bready) s_bvalid <= 0;
      if ((!s_bvalid || s_bready) && b_q.size() != 0 && b_q[0] <= cyc) begin
        s_bvalid <= 1; s_bresp <= inj_slverr ? 2'b10 : 2'b00; void'(b_q.pop_front());
      end
      // AR
      if (s_arvalid && s_arready) begin
        check_cmd("AR", s_araddr, s_arlen, s_arsize, s_arburst);
        ar_q.push_back('{beat: 32'((s_araddr - BASE) >> 4), len: int'(s_arlen) + 1, t: cyc + lat});
      end
      s_arready <= !coin();
      // R (hold until RREADY)
      if (!s_rvalid || s_rready) begin
        if (ar_q.size() != 0 && ar_q[0].t <= cyc && !coin()) begin
          s_rvalid <= 1;
          s_rdata  <= rd_beat(ar_q[0].beat + r_beat);
          s_rresp  <= inj_slverr ? 2'b10 : 2'b00;
          s_rlast  <= (r_beat == ar_q[0].len - 1);
          if (r_beat == ar_q[0].len - 1) begin void'(ar_q.pop_front()); r_beat = 0; end
          else r_beat++;
        end else s_rvalid <= 0;
      end
    end
  end
endmodule
