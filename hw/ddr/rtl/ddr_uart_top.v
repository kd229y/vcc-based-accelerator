// ddr_uart_top: Nexys A7-100T DDR2 test design (docs/DDR_PORT.md section 3).
// 100 MHz -> MMCME2_BASE -> 300 MHz sys_clk_i + 200 MHz clk_ref_i -> MIG (mig_ddr2) -> ui_clk 75 MHz.
// All user logic runs on ui_clk: uart_bridge -> ddr_test -> ddr_port -> MIG S_AXI.
// LEDs: [0] ui_clk heartbeat, [1] calibration done, [2] last test passed, [3] last test failed or AXI error.
// DDR2 port names must match mig_ddr2.prj (the MIG xdc places them by name).
`timescale 1ns/1ps
module ddr_uart_top #(
  parameter UI_HZ = 75_000_000,
  parameter BAUD  = 115_200
) (
  input  wire        clk,          // 100 MHz board clock
  input  wire        rst_btn,      // active high
  input  wire        uart_rx,
  output wire        uart_tx,
  output wire [3:0]  led,
  inout  wire [15:0] ddr2_dq,
  inout  wire [1:0]  ddr2_dqs_n,
  inout  wire [1:0]  ddr2_dqs_p,
  output wire [12:0] ddr2_addr,
  output wire [2:0]  ddr2_ba,
  output wire        ddr2_ras_n,
  output wire        ddr2_cas_n,
  output wire        ddr2_we_n,
  output wire [0:0]  ddr2_ck_p,
  output wire [0:0]  ddr2_ck_n,
  output wire [0:0]  ddr2_cke,
  output wire [0:0]  ddr2_cs_n,
  output wire [1:0]  ddr2_dm,
  output wire [0:0]  ddr2_odt
);
  // ---- clocks for the MIG: VCO 1200 MHz, /4 = 300 MHz, /6 = 200 MHz ----
  wire fb, c300_u, c200_u, c300, c200, locked;
  MMCME2_BASE #(
    .CLKIN1_PERIOD(10.0), .DIVCLK_DIVIDE(1), .CLKFBOUT_MULT_F(12.0),
    .CLKOUT0_DIVIDE_F(4.0), .CLKOUT1_DIVIDE(6)
  ) u_mmcm (
    .CLKIN1(clk), .CLKFBIN(fb), .CLKFBOUT(fb), .CLKFBOUTB(),
    .CLKOUT0(c300_u), .CLKOUT0B(), .CLKOUT1(c200_u), .CLKOUT1B(), .CLKOUT2(), .CLKOUT2B(),
    .CLKOUT3(), .CLKOUT3B(), .CLKOUT4(), .CLKOUT5(), .CLKOUT6(),
    .LOCKED(locked), .PWRDWN(1'b0), .RST(1'b0));
  BUFG u_bg300 (.I(c300_u), .O(c300));
  BUFG u_bg200 (.I(c200_u), .O(c200));

  // ---- MIG ----
  wire ui_clk, ui_rst, calib_done, mmcm_locked;
  reg  aresetn = 1'b0;
  always @(posedge ui_clk) aresetn <= !ui_rst;

  wire [3:0]   awid, arid, bid, rid;
  wire [26:0]  awaddr, araddr;
  wire [7:0]   awlen, arlen;
  wire [2:0]   awsize, arsize, awprot, arprot;
  wire [1:0]   awburst, arburst, bresp, rresp;
  wire         awlock, arlock;
  wire [3:0]   awcache, arcache, awqos, arqos;
  wire         awvalid, awready, wvalid, wready, wlast, bvalid, bready, arvalid, arready, rvalid, rready, rlast;
  wire [127:0] wdata, rdata;
  wire [15:0]  wstrb;

  mig_ddr2 u_mig (
    .ddr2_addr(ddr2_addr), .ddr2_ba(ddr2_ba), .ddr2_cas_n(ddr2_cas_n), .ddr2_ck_n(ddr2_ck_n), .ddr2_ck_p(ddr2_ck_p),
    .ddr2_cke(ddr2_cke), .ddr2_ras_n(ddr2_ras_n), .ddr2_we_n(ddr2_we_n), .ddr2_dq(ddr2_dq), .ddr2_dqs_n(ddr2_dqs_n),
    .ddr2_dqs_p(ddr2_dqs_p), .init_calib_complete(calib_done), .ddr2_cs_n(ddr2_cs_n), .ddr2_dm(ddr2_dm),
    .ddr2_odt(ddr2_odt),
    .ui_clk(ui_clk), .ui_clk_sync_rst(ui_rst), .mmcm_locked(mmcm_locked), .aresetn(aresetn),
    .app_sr_req(1'b0), .app_ref_req(1'b0), .app_zq_req(1'b0), .app_sr_active(), .app_ref_ack(), .app_zq_ack(),
    .s_axi_awid(awid), .s_axi_awaddr(awaddr), .s_axi_awlen(awlen), .s_axi_awsize(awsize), .s_axi_awburst(awburst),
    .s_axi_awlock(awlock), .s_axi_awcache(awcache), .s_axi_awprot(awprot), .s_axi_awqos(awqos),
    .s_axi_awvalid(awvalid), .s_axi_awready(awready),
    .s_axi_wdata(wdata), .s_axi_wstrb(wstrb), .s_axi_wlast(wlast), .s_axi_wvalid(wvalid), .s_axi_wready(wready),
    .s_axi_bid(bid), .s_axi_bresp(bresp), .s_axi_bvalid(bvalid), .s_axi_bready(bready),
    .s_axi_arid(arid), .s_axi_araddr(araddr), .s_axi_arlen(arlen), .s_axi_arsize(arsize), .s_axi_arburst(arburst),
    .s_axi_arlock(arlock), .s_axi_arcache(arcache), .s_axi_arprot(arprot), .s_axi_arqos(arqos),
    .s_axi_arvalid(arvalid), .s_axi_arready(arready),
    .s_axi_rid(rid), .s_axi_rdata(rdata), .s_axi_rresp(rresp), .s_axi_rlast(rlast), .s_axi_rvalid(rvalid),
    .s_axi_rready(rready),
    .sys_clk_i(c300), .clk_ref_i(c200), .sys_rst(locked && !rst_btn));

  // ---- UART register bus -> test engine -> DDR port ----
  wire        req, we, ack;
  wire [23:0] addr;
  wire [31:0] wd, rd;
  uart_bridge #(.CLKS_PER_BIT(UI_HZ / BAUD)) u_br (
    .clk(ui_clk), .rst(ui_rst), .rx(uart_rx), .tx(uart_tx),
    .bus_req(req), .bus_we(we), .bus_addr(addr), .bus_wdata(wd), .bus_ack(ack), .bus_rdata(rd));

  wire         wcv, wcr, wv, wr, wdn, rcv, rcr, rv, rr, rl, perr, busy, pass, fail;
  wire [26:0]  wca, rca;
  wire [15:0]  wcl, rcl, ws;
  wire [127:0] wdat, rdat;
  ddr_test u_test (
    .clk(ui_clk), .rst(ui_rst), .bus_req(req), .bus_we(we), .bus_addr(addr), .bus_wdata(wd), .bus_ack(ack),
    .bus_rdata(rd), .calib_done(calib_done), .port_err(perr),
    .wr_cmd_valid(wcv), .wr_cmd_ready(wcr), .wr_cmd_addr(wca), .wr_cmd_len(wcl), .wr_valid(wv), .wr_ready(wr),
    .wr_data(wdat), .wr_strb(ws), .wr_done(wdn), .rd_cmd_valid(rcv), .rd_cmd_ready(rcr), .rd_cmd_addr(rca),
    .rd_cmd_len(rcl), .rd_valid(rv), .rd_ready(rr), .rd_data(rdat), .rd_last(rl), .busy(busy), .pass(pass), .fail(fail));

  ddr_port #(.AXI_AW(27)) u_port (
    .clk(ui_clk), .rst(ui_rst), .calib_done(calib_done),
    .wr_cmd_valid(wcv), .wr_cmd_ready(wcr), .wr_cmd_addr(wca), .wr_cmd_len(wcl), .wr_valid(wv), .wr_ready(wr),
    .wr_data(wdat), .wr_strb(ws), .wr_done(wdn), .rd_cmd_valid(rcv), .rd_cmd_ready(rcr), .rd_cmd_addr(rca),
    .rd_cmd_len(rcl), .rd_valid(rv), .rd_ready(rr), .rd_data(rdat), .rd_last(rl), .err(perr),
    .m_axi_awid(awid), .m_axi_awaddr(awaddr), .m_axi_awlen(awlen), .m_axi_awsize(awsize), .m_axi_awburst(awburst),
    .m_axi_awlock(awlock), .m_axi_awcache(awcache), .m_axi_awprot(awprot), .m_axi_awqos(awqos),
    .m_axi_awvalid(awvalid), .m_axi_awready(awready),
    .m_axi_wdata(wdata), .m_axi_wstrb(wstrb), .m_axi_wlast(wlast), .m_axi_wvalid(wvalid), .m_axi_wready(wready),
    .m_axi_bid(bid), .m_axi_bresp(bresp), .m_axi_bvalid(bvalid), .m_axi_bready(bready),
    .m_axi_arid(arid), .m_axi_araddr(araddr), .m_axi_arlen(arlen), .m_axi_arsize(arsize), .m_axi_arburst(arburst),
    .m_axi_arlock(arlock), .m_axi_arcache(arcache), .m_axi_arprot(arprot), .m_axi_arqos(arqos),
    .m_axi_arvalid(arvalid), .m_axi_arready(arready),
    .m_axi_rid(rid), .m_axi_rdata(rdata), .m_axi_rresp(rresp), .m_axi_rlast(rlast), .m_axi_rvalid(rvalid),
    .m_axi_rready(rready));

  reg [25:0] hb = 26'd0;
  always @(posedge ui_clk) hb <= hb + 26'd1;
  assign led = {fail | perr, pass, calib_done, hb[25]};

  /* verilator lint_off UNUSEDSIGNAL */
  wire unused_ok = &{1'b0, mmcm_locked, busy};
  /* verilator lint_on UNUSEDSIGNAL */
endmodule
