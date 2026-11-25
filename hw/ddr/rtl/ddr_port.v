// ddr_port: accelerator-side DDR access port (docs/DDR_PORT.md section 4), AXI4 master, one clock (ui_clk).
// User side: independent write and read command channels; addresses in bytes (16 B aligned), lengths in beats.
// AXI side: INCR bursts of <= BURST_MAX beats that never cross 4 KB; one ID (0), so responses are in order.
// Write: AW goes out only when the W FIFO already holds the whole burst (no stalled burst on the bus).
// Read:  AR goes out only when the read FIFO has room for every beat in flight, so RREADY is always 1.
// Bad command (address not 16 B aligned, or len = 0): sets err; the address is aligned down, len = 0 completes at once.
`timescale 1ns/1ps
module ddr_port #(
  parameter AXI_AW         = 32,
  parameter [AXI_AW-1:0] AXI_BASE = {AXI_AW{1'b0}},
  parameter ID_W           = 4,
  parameter BURST_MAX      = 16,   // 1..256 beats per AXI burst
  parameter RD_OUTSTANDING = 4,    // read bursts in flight
  parameter RD_DEPTH       = 32,   // read FIFO beats, >= BURST_MAX
  parameter WR_DEPTH       = 16    // write FIFO beats, >= BURST_MAX; AXI_AW >= 27, AXI_BASE 4 KB aligned
) (
  input  wire              clk,
  input  wire              rst,
  input  wire              calib_done,
  // write command and data
  input  wire              wr_cmd_valid,
  output wire              wr_cmd_ready,
  input  wire [26:0]       wr_cmd_addr,
  input  wire [15:0]       wr_cmd_len,
  input  wire              wr_valid,
  output wire              wr_ready,
  input  wire [127:0]      wr_data,
  input  wire [15:0]       wr_strb,
  output reg               wr_done,
  // read command and data
  input  wire              rd_cmd_valid,
  output wire              rd_cmd_ready,
  input  wire [26:0]       rd_cmd_addr,
  input  wire [15:0]       rd_cmd_len,
  output wire              rd_valid,
  input  wire              rd_ready,
  output wire [127:0]      rd_data,
  output wire              rd_last,
  output reg               err,
  // AXI4 master
  output wire [ID_W-1:0]   m_axi_awid,
  output reg  [AXI_AW-1:0] m_axi_awaddr,
  output reg  [7:0]        m_axi_awlen,
  output wire [2:0]        m_axi_awsize,
  output wire [1:0]        m_axi_awburst,
  output wire              m_axi_awlock,
  output wire [3:0]        m_axi_awcache,
  output wire [2:0]        m_axi_awprot,
  output wire [3:0]        m_axi_awqos,
  output reg               m_axi_awvalid,
  input  wire              m_axi_awready,
  output wire [127:0]      m_axi_wdata,
  output wire [15:0]       m_axi_wstrb,
  output wire              m_axi_wlast,
  output wire              m_axi_wvalid,
  input  wire              m_axi_wready,
  /* verilator lint_off UNUSEDSIGNAL */
  input  wire [ID_W-1:0]   m_axi_bid,
  input  wire [1:0]        m_axi_bresp,
  /* verilator lint_on UNUSEDSIGNAL */
  input  wire              m_axi_bvalid,
  output wire              m_axi_bready,
  output wire [ID_W-1:0]   m_axi_arid,
  output reg  [AXI_AW-1:0] m_axi_araddr,
  output reg  [7:0]        m_axi_arlen,
  output wire [2:0]        m_axi_arsize,
  output wire [1:0]        m_axi_arburst,
  output wire              m_axi_arlock,
  output wire [3:0]        m_axi_arcache,
  output wire [2:0]        m_axi_arprot,
  output wire [3:0]        m_axi_arqos,
  output reg               m_axi_arvalid,
  input  wire              m_axi_arready,
  /* verilator lint_off UNUSEDSIGNAL */
  input  wire [ID_W-1:0]   m_axi_rid,
  input  wire [1:0]        m_axi_rresp,
  /* verilator lint_on UNUSEDSIGNAL */
  input  wire [127:0]      m_axi_rdata,
  input  wire              m_axi_rlast,
  input  wire              m_axi_rvalid,
  output wire              m_axi_rready
);
  localparam integer BMAX_I = BURST_MAX;
  localparam integer RD_ROOM_I = RD_DEPTH;
  localparam integer RD_OUT_I = RD_OUTSTANDING;
  localparam [15:0] BMAX = BMAX_I[15:0];
  localparam WCW = $clog2(WR_DEPTH + 1);
  localparam RCW = $clog2(RD_DEPTH + 1);
  localparam OCW = $clog2(RD_OUTSTANDING + 1);
  localparam [16:0] RD_ROOM = RD_ROOM_I[16:0];
  localparam [OCW-1:0] RD_OUT_MAX = RD_OUT_I[OCW-1:0];
  localparam integer ONE_I = 1;
  localparam [WCW-1:0] W_ONE = ONE_I[WCW-1:0];
  localparam [OCW-1:0] O_ONE = ONE_I[OCW-1:0];

  // Illegal parameters stop elaboration (the missing module is the error message).
  generate
    if (BURST_MAX < 1 || BURST_MAX > 256 || RD_DEPTH < BURST_MAX || WR_DEPTH < BURST_MAX ||
        RD_OUTSTANDING < 1 || AXI_AW < 27 || AXI_BASE[11:0] != 12'd0) begin : g_bad_param
      ddr_port_illegal_parameters u_stop ();
    end
  endgenerate

  // Fixed AXI attributes: 16 B beats, INCR, normal non-cacheable bufferable.
  assign m_axi_awid = {ID_W{1'b0}};    assign m_axi_arid = {ID_W{1'b0}};
  assign m_axi_awsize = 3'd4;          assign m_axi_arsize = 3'd4;
  assign m_axi_awburst = 2'b01;        assign m_axi_arburst = 2'b01;
  assign m_axi_awlock = 1'b0;          assign m_axi_arlock = 1'b0;
  assign m_axi_awcache = 4'b0011;      assign m_axi_arcache = 4'b0011;
  assign m_axi_awprot = 3'b000;        assign m_axi_arprot = 3'b000;
  assign m_axi_awqos = 4'd0;           assign m_axi_arqos = 4'd0;

  // Beats in the next burst: min(remaining, BURST_MAX, beats to the 4 KB boundary); a4k = address bits [11:4].
  function [8:0] blen_f(input [7:0] a4k, input [15:0] rem);
    reg [8:0] b4k; reg [15:0] m;
    begin
      b4k = 9'd256 - {1'b0, a4k};
      m = (rem < BMAX) ? rem : BMAX;
      blen_f = (m < {7'd0, b4k}) ? m[8:0] : b4k;
    end
  endfunction

  // AXI address = AXI_BASE + zero-extended byte address (works for any AXI_AW).
  /* verilator lint_off UNUSEDSIGNAL */
  wire [AXI_AW+26:0] aw_z, ar_z;
  /* verilator lint_on UNUSEDSIGNAL */

  wire bad_wr = (wr_cmd_addr[3:0] != 4'd0) || (wr_cmd_len == 16'd0);
  wire bad_rd = (rd_cmd_addr[3:0] != 4'd0) || (rd_cmd_len == 16'd0);

  // ---------------- write ----------------
  reg              wr_act;
  reg  [26:0]      aw_addr;      // next AW byte address
  reg  [15:0]      aw_rem;       // beats not yet covered by an AW
  reg  [15:0]      in_rem;       // beats not yet taken from the user
  reg  [WCW-1:0]   w_owed;       // beats of issued AWs not yet sent on W
  reg  [16:0]      b_pend;       // AWs issued minus B received
  reg  [7:0]       w_beat;       // beat index inside the current W burst

  wire             wr_cmd_hs = wr_cmd_valid && wr_cmd_ready;
  assign wr_cmd_ready = calib_done && !wr_act;

  wire [WCW-1:0]   wf_count;
  wire             wf_full, wf_empty;
  wire [143:0]     wf_dout;
  wire             w_hs = m_axi_wvalid && m_axi_wready;
  assign wr_ready = wr_act && (in_rem != 16'd0) && !wf_full;
  wire             wr_hs = wr_valid && wr_ready;

  ddr_fifo #(.W(144), .DEPTH(WR_DEPTH)) u_wf (
    .clk(clk), .rst(rst), .push(wr_hs), .din({wr_strb, wr_data}), .pop(w_hs),
    .dout(wf_dout), .empty(wf_empty), .full(wf_full), .count(wf_count));

  // Burst lengths (minus 1) of issued AWs, consumed by the W channel.
  wire [7:0]       lq_head;
  wire             lq_full, lq_empty;
  wire [2:0]       lq_count;
  assign aw_z = {{AXI_AW{1'b0}}, aw_addr};
  wire [8:0]       wb = blen_f(aw_addr[11:4], aw_rem);
  wire [WCW-1:0]   w_avail = wf_count - w_owed;
  wire             aw_go = wr_act && (aw_rem != 16'd0) && (!m_axi_awvalid || m_axi_awready) &&
                           ({{(17-WCW){1'b0}}, w_avail} >= {8'd0, wb}) && !lq_full;
  wire [7:0]       wb_m1 = wb[7:0] - 8'd1;
  assign m_axi_wvalid = (w_owed != {WCW{1'b0}});
  assign m_axi_wdata  = wf_dout[127:0];
  assign m_axi_wstrb  = wf_dout[143:128];
  assign m_axi_wlast  = (w_beat == lq_head);
  assign m_axi_bready = 1'b1;

  ddr_fifo #(.W(8), .DEPTH(4)) u_lq (
    .clk(clk), .rst(rst), .push(aw_go), .din(wb_m1), .pop(w_hs && m_axi_wlast),
    .dout(lq_head), .empty(lq_empty), .full(lq_full), .count(lq_count));

  wire wr_fin = wr_act && (aw_rem == 16'd0) && (w_owed == {WCW{1'b0}}) && (b_pend == 17'd0) && !m_axi_awvalid;

  always @(posedge clk) begin
    if (rst) begin
      wr_act <= 1'b0; wr_done <= 1'b0; m_axi_awvalid <= 1'b0;
      aw_rem <= 16'd0; in_rem <= 16'd0; w_owed <= {WCW{1'b0}}; b_pend <= 17'd0; w_beat <= 8'd0;
    end else begin
      wr_done <= 1'b0;
      if (wr_cmd_hs) begin
        aw_addr <= {wr_cmd_addr[26:4], 4'd0};
        aw_rem  <= wr_cmd_len;
        in_rem  <= wr_cmd_len;
        if (wr_cmd_len == 16'd0) wr_done <= 1'b1;
        else wr_act <= 1'b1;
      end
      if (wr_fin) begin wr_act <= 1'b0; wr_done <= 1'b1; end
      if (wr_hs) in_rem <= in_rem - 16'd1;
      if (aw_go) begin
        m_axi_awvalid <= 1'b1;
        m_axi_awaddr  <= AXI_BASE + aw_z[AXI_AW-1:0];
        m_axi_awlen   <= wb_m1;
        aw_addr       <= aw_addr + {14'd0, wb, 4'd0};
        aw_rem        <= aw_rem - {7'd0, wb};
      end else if (m_axi_awready) m_axi_awvalid <= 1'b0;
      w_owed <= w_owed + (aw_go ? wb[WCW-1:0] : {WCW{1'b0}}) - (w_hs ? W_ONE : {WCW{1'b0}});
      b_pend <= b_pend + {16'd0, aw_go} - {16'd0, m_axi_bvalid};
      if (w_hs) w_beat <= m_axi_wlast ? 8'd0 : w_beat + 8'd1;
    end
  end

  // ---------------- read ----------------
  reg              rd_act;
  reg  [26:0]      ar_addr;
  reg  [15:0]      ar_rem;       // beats not yet covered by an AR
  reg  [15:0]      out_rem;      // beats not yet given to the user
  reg  [16:0]      resv;         // FIFO beats reserved by issued ARs and not yet popped
  reg  [OCW-1:0]   ar_out;       // read bursts in flight

  wire             rd_cmd_hs = rd_cmd_valid && rd_cmd_ready;
  assign rd_cmd_ready = calib_done && !rd_act;
  assign m_axi_rready = 1'b1;
  wire             r_hs = m_axi_rvalid;

  wire [RCW-1:0]   rf_count;
  wire             rf_full, rf_empty;
  ddr_fifo #(.W(128), .DEPTH(RD_DEPTH)) u_rf (
    .clk(clk), .rst(rst), .push(r_hs), .din(m_axi_rdata), .pop(rd_valid && rd_ready),
    .dout(rd_data), .empty(rf_empty), .full(rf_full), .count(rf_count));

  assign rd_valid = !rf_empty;
  assign rd_last  = (out_rem == 16'd1);
  wire             rd_pop = rd_valid && rd_ready;

  assign ar_z = {{AXI_AW{1'b0}}, ar_addr};
  wire [8:0]       rb = blen_f(ar_addr[11:4], ar_rem);
  wire             ar_go = rd_act && (ar_rem != 16'd0) && (!m_axi_arvalid || m_axi_arready) &&
                           (resv + {8'd0, rb} <= RD_ROOM) && (ar_out < RD_OUT_MAX);

  always @(posedge clk) begin
    if (rst) begin
      rd_act <= 1'b0; m_axi_arvalid <= 1'b0; ar_rem <= 16'd0; out_rem <= 16'd0;
      resv <= 17'd0; ar_out <= {OCW{1'b0}};
    end else begin
      if (rd_cmd_hs) begin
        ar_addr <= {rd_cmd_addr[26:4], 4'd0};
        ar_rem  <= rd_cmd_len;
        out_rem <= rd_cmd_len;
        if (rd_cmd_len != 16'd0) rd_act <= 1'b1;
      end
      if (ar_go) begin
        m_axi_arvalid <= 1'b1;
        m_axi_araddr  <= AXI_BASE + ar_z[AXI_AW-1:0];
        m_axi_arlen   <= rb[7:0] - 8'd1;
        ar_addr       <= ar_addr + {14'd0, rb, 4'd0};
        ar_rem        <= ar_rem - {7'd0, rb};
      end else if (m_axi_arready) m_axi_arvalid <= 1'b0;
      resv   <= resv + (ar_go ? {8'd0, rb} : 17'd0) - {16'd0, rd_pop};
      ar_out <= ar_out + (ar_go ? O_ONE : {OCW{1'b0}})
                       - ((r_hs && m_axi_rlast) ? O_ONE : {OCW{1'b0}});
      if (rd_pop) begin
        out_rem <= out_rem - 16'd1;
        if (out_rem == 16'd1) rd_act <= 1'b0;
      end
    end
  end

  // ---------------- errors ----------------
  always @(posedge clk) begin
    if (rst) err <= 1'b0;
    else if ((wr_cmd_hs && bad_wr) || (rd_cmd_hs && bad_rd) ||
             (m_axi_bvalid && m_axi_bresp[1]) || (m_axi_rvalid && m_axi_rresp[1])) err <= 1'b1;
  end

  /* verilator lint_off UNUSEDSIGNAL */
  wire unused_ok = &{1'b0, wf_empty, lq_empty, lq_count, rf_full, rf_count};
  /* verilator lint_on UNUSEDSIGNAL */
endmodule
