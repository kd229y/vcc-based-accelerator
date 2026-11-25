// tb_ddr_port: ddr_port against axi_mem_model (L1 in docs/DDR_PORT.md section 7), Verilator --timing.
// Plusargs: +seed=N +gap=P (AXI stall %) +lat=N (read latency) +blat=N (B latency) +ugap=P (user stall %) +n=N (commands).
// Every read beat is compared with a reference memory; the master-side AXI rules are checked every cycle.
`timescale 1ns/1ps

// Master-side AXI4 rules that ddr_port promises (docs/DDR_PORT.md section 4.2).
module axi_master_checker #(parameter int AW = 32, parameter int RD_OUT = 4) (
  input logic clk, rst,
  input logic [AW-1:0] awaddr, input logic [7:0] awlen, input logic awvalid, awready,
  input logic [127:0] wdata, input logic [15:0] wstrb, input logic wlast, wvalid, wready,
  input logic bready,
  input logic [AW-1:0] araddr, input logic [7:0] arlen, input logic arvalid, arready,
  input logic rlast, rvalid, rready,
  output int errors
);
  logic p_awv, p_wv, p_arv;
  logic [AW+7:0] p_aw, p_ar;
  logic [144:0] p_w;
  logic in_wburst;
  int ar_out;
  always_ff @(posedge clk) begin
    if (rst) begin
      p_awv <= 0; p_wv <= 0; p_arv <= 0; in_wburst <= 0; ar_out <= 0; errors <= 0;
    end else begin
      // valid stays high and the payload stays stable until ready
      if (p_awv && !(awvalid && {awaddr, awlen} == p_aw)) begin errors <= errors + 1; $display("ERROR proto: AW changed before AWREADY"); end
      if (p_wv  && !(wvalid  && {wstrb, wlast, wdata} == p_w)) begin errors <= errors + 1; $display("ERROR proto: W changed before WREADY"); end
      if (p_arv && !(arvalid && {araddr, arlen} == p_ar)) begin errors <= errors + 1; $display("ERROR proto: AR changed before ARREADY"); end
      p_awv <= awvalid && !awready; p_aw <= {awaddr, awlen};
      p_wv  <= wvalid && !wready;   p_w  <= {wstrb, wlast, wdata};
      p_arv <= arvalid && !arready; p_ar <= {araddr, arlen};
      // a started W burst has no master bubbles; B and R are never back-pressured
      if (in_wburst && !wvalid) begin errors <= errors + 1; $display("ERROR proto: WVALID gap inside a burst"); end
      if (wvalid && wready) in_wburst <= !wlast;
      if (!bready) begin errors <= errors + 1; $display("ERROR proto: BREADY low"); end
      if (rvalid && !rready) begin errors <= errors + 1; $display("ERROR proto: RREADY low with RVALID"); end
      ar_out <= ar_out + int'(arvalid && arready) - int'(rvalid && rready && rlast);
      if (ar_out > RD_OUT) begin errors <= errors + 1; $display("ERROR proto: %0d read bursts in flight > %0d", ar_out, RD_OUT); end
    end
  end
endmodule

module tb_ddr_port #(
  parameter int BURST_MAX      = 16,
  parameter int RD_OUTSTANDING = 4,
  parameter int RD_DEPTH       = 32,
  parameter int WR_DEPTH       = 16,
  parameter logic [31:0] AXI_BASE = 32'h6000_0000
);
  import ddr_tb_pkg::*;
  localparam int AW = 32;

  logic clk = 0, rst = 1, mrst = 1, calib = 0;   // mrst: model and checker, only at start
  always #5 clk = ~clk;

  logic wr_cmd_valid = 0, wr_cmd_ready, wr_valid = 0, wr_ready, wr_done;
  logic [26:0] wr_cmd_addr = 0; logic [15:0] wr_cmd_len = 0;
  logic [127:0] wr_data = 0; logic [15:0] wr_strb = 0;
  logic rd_cmd_valid = 0, rd_cmd_ready, rd_valid, rd_ready = 0, rd_last, err;
  logic [26:0] rd_cmd_addr = 0; logic [15:0] rd_cmd_len = 0;
  logic [127:0] rd_data;

  logic [3:0] awid, arid, bid, rid; logic [AW-1:0] awaddr, araddr; logic [7:0] awlen, arlen;
  logic [2:0] awsize, arsize, awprot, arprot; logic [1:0] awburst, arburst, bresp, rresp;
  logic awlock, arlock; logic [3:0] awcache, arcache, awqos, arqos;
  logic awvalid, awready, wvalid, wready, wlast, bvalid, bready, arvalid, arready, rvalid, rready, rlast;
  logic [127:0] wdata, rdata; logic [15:0] wstrb;

  int gap = 30, lat = 20, blat = 8, ugap = 30, n = 300;
  logic [31:0] seed = 1;
  logic inj = 0;
  int mem_err, chk_err, errors = 0, beats_rd = 0, beats_wr = 0, n_wr = 0, n_rd = 0;

  ddr_port #(.AXI_AW(AW), .AXI_BASE(AXI_BASE), .BURST_MAX(BURST_MAX), .RD_OUTSTANDING(RD_OUTSTANDING),
             .RD_DEPTH(RD_DEPTH), .WR_DEPTH(WR_DEPTH)) dut (
    .clk, .rst, .calib_done(calib),
    .wr_cmd_valid, .wr_cmd_ready, .wr_cmd_addr, .wr_cmd_len, .wr_valid, .wr_ready, .wr_data, .wr_strb, .wr_done,
    .rd_cmd_valid, .rd_cmd_ready, .rd_cmd_addr, .rd_cmd_len, .rd_valid, .rd_ready, .rd_data, .rd_last, .err,
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

  axi_mem_model #(.AW(AW), .BASE(AXI_BASE)) u_mem (
    .clk, .rst(mrst), .gap, .lat, .blat, .seed, .inj_slverr(inj),
    .s_awaddr(awaddr), .s_awlen(awlen), .s_awsize(awsize), .s_awburst(awburst), .s_awvalid(awvalid), .s_awready(awready),
    .s_wdata(wdata), .s_wstrb(wstrb), .s_wlast(wlast), .s_wvalid(wvalid), .s_wready(wready),
    .s_bid(bid), .s_bresp(bresp), .s_bvalid(bvalid), .s_bready(bready),
    .s_araddr(araddr), .s_arlen(arlen), .s_arsize(arsize), .s_arburst(arburst), .s_arvalid(arvalid), .s_arready(arready),
    .s_rid(rid), .s_rdata(rdata), .s_rresp(rresp), .s_rlast(rlast), .s_rvalid(rvalid), .s_rready(rready),
    .errors(mem_err));

  axi_master_checker #(.AW(AW), .RD_OUT(RD_OUTSTANDING)) u_chk (
    .clk, .rst(mrst), .awaddr, .awlen, .awvalid, .awready, .wdata, .wstrb, .wlast, .wvalid, .wready, .bready,
    .araddr, .arlen, .arvalid, .arready, .rlast, .rvalid, .rready, .errors(chk_err));

  // ---- reference memory and PRNG ----
  logic [127:0] ref_mem [logic [31:0]];
  logic [31:0] ts;     // tb PRNG state
  function automatic logic [31:0] rnd(); ts = xs32(ts); return ts; endfunction
  function automatic logic [127:0] ref_rd(input logic [31:0] b);   // if/else: see axi_mem_model rd_beat
    if (ref_mem.exists(b)) return ref_mem[b];
    return init_beat(b);
  endfunction
  function automatic logic [127:0] rnd128(); return {rnd(), rnd(), rnd(), rnd()}; endfunction

  // Fixed AXI attributes and side checks.
  always @(posedge clk) if (!rst) begin
    if (awvalid && (awcache != 4'b0011 || awprot != 0 || awqos != 0 || awlock || awid != 0)) begin errors++; $display("ERROR AW attributes"); end
    if (arvalid && (arcache != 4'b0011 || arprot != 0 || arqos != 0 || arlock || arid != 0)) begin errors++; $display("ERROR AR attributes"); end
    if (rvalid && dut.u_rf.full) begin errors++; $display("ERROR read FIFO overflow"); end
  end

  // B and AW counters, to check that wr_done comes only after every B.
  int aw_hs = 0, b_hs = 0;
  always @(posedge clk) if (!rst) begin
    if (awvalid && awready) aw_hs++;
    if (bvalid && bready) b_hs++;
  end

  // Random command: addresses near 4 KB edges half of the time, lengths from short to multi-burst.
  function automatic logic [26:0] pick_addr(input logic [26:0] lo, input int span_beats, input int len);
    int b = (rnd() % span_beats);
    if (rnd() % 2) b = ((b >> 8) << 8) + 256 - 1 - (rnd() % 4);     // a few beats before a 4 KB edge
    if (b + len > span_beats) b = span_beats - len;
    if (b < 0) b = 0;
    return lo + 27'(b * 16);
  endfunction
  function automatic int pick_len(input int maxl);
    int c = rnd() % 10;
    int l = (c < 4) ? 1 + rnd() % 4 : (c < 7) ? 1 + rnd() % 40 : (c < 9) ? 1 + rnd() % 300 : 1 + rnd() % 1200;
    return (l > maxl) ? maxl : l;
  endfunction

  // ---- user-side drivers (drive at negedge, a handshake happens at the next posedge) ----
  task automatic do_write(input logic [26:0] a, input int len, input bit rnd_strb);
    int aw0, b0, done_seen = 0;
    @(negedge clk);
    wr_cmd_valid = 1; wr_cmd_addr = a; wr_cmd_len = 16'(len);
    while (!wr_cmd_ready) @(negedge clk);
    @(negedge clk); wr_cmd_valid = 0;
    aw0 = aw_hs; b0 = b_hs;
    for (int i = 0; i < len; i++) begin
      logic [127:0] d = rnd128(), v;
      logic [15:0] s = rnd_strb ? 16'(rnd()) : 16'hFFFF;
      logic [31:0] bi = 32'(a >> 4) + i;
      while ((rnd() % 100) < ugap) @(negedge clk);
      wr_valid = 1; wr_data = d; wr_strb = s;
      while (!wr_ready) @(negedge clk);
      v = ref_rd(bi);
      for (int k = 0; k < 16; k++) if (s[k]) v[8*k +: 8] = d[8*k +: 8];
      ref_mem[bi] = v;
      @(negedge clk); wr_valid = 0;
      if (wr_done) done_seen++;
    end
    while (!wr_done) @(negedge clk);
    // wr_done is high in this half cycle; all B of this command must be in
    if (aw_hs - aw0 != b_hs - b0) begin errors++; $display("ERROR wr_done with %0d AW and %0d B", aw_hs - aw0, b_hs - b0); end
    if (done_seen) begin errors++; $display("ERROR wr_done before the last data beat"); end
    @(negedge clk);
    if (wr_done) begin errors++; $display("ERROR wr_done longer than one cycle"); end
    n_wr++; beats_wr += len;
  endtask

  task automatic do_read(input logic [26:0] a, input int len);
    @(negedge clk);
    rd_cmd_valid = 1; rd_cmd_addr = a; rd_cmd_len = 16'(len);
    while (!rd_cmd_ready) @(negedge clk);
    @(negedge clk); rd_cmd_valid = 0;
    for (int i = 0; i < len; i++) begin
      logic [31:0] bi = 32'(a >> 4) + i;
      while ((rnd() % 100) < ugap) begin rd_ready = 0; @(negedge clk); end
      rd_ready = 1;
      while (!rd_valid) @(negedge clk);
      if (rd_data !== ref_rd(bi)) begin
        errors++;
        if (errors < 10) $display("ERROR read beat %0d of %0d at %h: got %h exp %h", i, len, {bi, 4'd0}, rd_data, ref_rd(bi));
      end
      if (rd_last !== (i == len - 1)) begin errors++; if (errors < 10) $display("ERROR rd_last=%0d at beat %0d of %0d", rd_last, i, len); end
      @(negedge clk); rd_ready = 0;
    end
    n_rd++; beats_rd += len;
  endtask

  // Regions: A written while B is read, C never written (initial pattern).
  localparam logic [26:0] RA = 27'h000_0000, RB = 27'h001_0000, RC = 27'h7FF_0000;
  localparam int SPAN = 4096;   // beats (64 KB) per region

  initial begin
    if ($value$plusargs("seed=%d", seed)) ;
    if ($value$plusargs("gap=%d", gap)) ;
    if ($value$plusargs("lat=%d", lat)) ;
    if ($value$plusargs("blat=%d", blat)) ;
    if ($value$plusargs("ugap=%d", ugap)) ;
    if ($value$plusargs("n=%d", n)) ;
    ts = seed * 32'h9E37_79B9 + 1;
    repeat (5) @(negedge clk);
    rst = 0; mrst = 0;
    // no command is taken before calibration
    repeat (40) begin
      @(negedge clk);
      wr_cmd_valid = 1; rd_cmd_valid = 1;
      if (wr_cmd_ready || rd_cmd_ready) begin errors++; $display("ERROR cmd_ready before calib_done"); end
    end
    wr_cmd_valid = 0; rd_cmd_valid = 0;
    calib = 1;
    // fill region B
    for (int b = 0; b < SPAN; ) begin
      int l = pick_len(SPAN - b);
      do_write(RB + 27'(b * 16), l, 0);
      b += l;
    end
    // write A while reading B and C
    fork
      for (int i = 0; i < n; i++) begin int l = pick_len(SPAN); do_write(pick_addr(RA, SPAN, l), l, (rnd() % 3) == 0); end
      for (int i = 0; i < n; i++) begin
        int l = pick_len(SPAN);
        do_read(pick_addr((rnd() % 4 == 0) ? RC : RB, SPAN, l), l);
      end
    join
    // read A back
    for (int b = 0; b < SPAN; ) begin
      int l = pick_len(SPAN - b);
      do_read(RA + 27'(b * 16), l);
      b += l;
    end
    if (err) begin errors++; $display("ERROR err set on legal traffic"); end
    // bad commands: unaligned write (aligned down) and len = 0 read must set err and not hang
    do_write(27'h20_0004, 1, 0);
    @(negedge clk);
    if (!err) begin errors++; $display("ERROR unaligned write did not set err"); end
    do_read(27'h20_0000, 1);
    rst = 1; @(negedge clk); rst = 0; calib = 1;
    @(negedge clk); rd_cmd_valid = 1; rd_cmd_addr = 0; rd_cmd_len = 0;
    while (!rd_cmd_ready) @(negedge clk);
    @(negedge clk); rd_cmd_valid = 0; @(negedge clk);
    if (!err || !rd_cmd_ready) begin errors++; $display("ERROR len=0 read: err=%0d ready=%0d", err, rd_cmd_ready); end
    // SLVERR on B and R sets err
    rst = 1; @(negedge clk); rst = 0; inj = 1;
    do_write(27'h30_0000, 2, 0);
    if (!err) begin errors++; $display("ERROR SLVERR on B did not set err"); end
    rst = 1; @(negedge clk); rst = 0;
    do_read(27'h30_0000, 1);
    if (!err) begin errors++; $display("ERROR SLVERR on R did not set err"); end
    inj = 0;
    repeat (20) @(negedge clk);
    $display("TB_DDR_PORT burst_max=%0d rd_out=%0d seed=%0d gap=%0d lat=%0d ugap=%0d writes=%0d reads=%0d wbeats=%0d rbeats=%0d errors=%0d proto_errors=%0d",
             BURST_MAX, RD_OUTSTANDING, seed, gap, lat, ugap, n_wr, n_rd, beats_wr, beats_rd, errors + mem_err, chk_err);
    $finish;
  end

  // watchdog
  initial begin
    #(64'd400_000_000);
    $display("TB_DDR_PORT TIMEOUT errors=1");
    $finish;
  end
endmodule
