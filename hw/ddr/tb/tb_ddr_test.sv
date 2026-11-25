// tb_ddr_test: ddr_test + ddr_port + axi_mem_model (step 3 in docs/DDR_PORT.md section 11), Verilator --timing.
// Drives the register bus directly. Checks MODE 0-4 and the window against a pattern model written here,
// reads the memory model directly, and plants corrupt beats to prove that ERRS / ERR_ADDR / ERR_GOT / ERR_EXP work.
// Plusargs: +gap=P +lat=N +blat=N (memory model).
`timescale 1ns/1ps
module tb_ddr_test;
  import ddr_tb_pkg::*;
  localparam int AW = 27;

  logic clk = 0, rst = 1, mrst = 1;
  always #5 clk = ~clk;
  int gap = 30, lat = 20, blat = 8, errors = 0, mem_err;
  logic inj = 0;

  logic req = 0, we = 0, ack; logic [23:0] addr = 0; logic [31:0] wd = 0, rd;
  logic wcv, wcr, wv, wr, wdn, rcv, rcr, rv, rr, rl, perr, busy, pass, fail;
  logic [26:0] wca, rca; logic [15:0] wcl, rcl, ws; logic [127:0] wdat, rdat;

  logic [3:0] awid, arid, bid, rid; logic [AW-1:0] awaddr, araddr; logic [7:0] awlen, arlen;
  logic [2:0] awsize, arsize, awprot, arprot; logic [1:0] awburst, arburst, bresp, rresp;
  logic awlock, arlock; logic [3:0] awcache, arcache, awqos, arqos;
  logic awvalid, awready, wvalid, wready, wlast, bvalid, bready, arvalid, arready, rvalid, rready, rlast;
  logic [127:0] wdata, rdata; logic [15:0] wstrb;

  ddr_test u_t (.clk, .rst, .bus_req(req), .bus_we(we), .bus_addr(addr), .bus_wdata(wd), .bus_ack(ack), .bus_rdata(rd),
    .calib_done(1'b1), .port_err(perr),
    .wr_cmd_valid(wcv), .wr_cmd_ready(wcr), .wr_cmd_addr(wca), .wr_cmd_len(wcl), .wr_valid(wv), .wr_ready(wr),
    .wr_data(wdat), .wr_strb(ws), .wr_done(wdn), .rd_cmd_valid(rcv), .rd_cmd_ready(rcr), .rd_cmd_addr(rca),
    .rd_cmd_len(rcl), .rd_valid(rv), .rd_ready(rr), .rd_data(rdat), .rd_last(rl), .busy, .pass, .fail);

  ddr_port #(.AXI_AW(AW)) u_p (.clk, .rst, .calib_done(1'b1),
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

  axi_mem_model #(.AW(AW), .BASE('0)) u_mem (.clk, .rst(mrst), .gap, .lat, .blat, .seed(32'd7), .inj_slverr(inj),
    .s_awaddr(awaddr), .s_awlen(awlen), .s_awsize(awsize), .s_awburst(awburst), .s_awvalid(awvalid), .s_awready(awready),
    .s_wdata(wdata), .s_wstrb(wstrb), .s_wlast(wlast), .s_wvalid(wvalid), .s_wready(wready),
    .s_bid(bid), .s_bresp(bresp), .s_bvalid(bvalid), .s_bready(bready),
    .s_araddr(araddr), .s_arlen(arlen), .s_arsize(arsize), .s_arburst(arburst), .s_arvalid(arvalid), .s_arready(arready),
    .s_rid(rid), .s_rdata(rdata), .s_rresp(rresp), .s_rlast(rlast), .s_rvalid(rvalid), .s_rready(rready),
    .errors(mem_err));

  // ---- pattern model (docs/DDR_PORT.md section 5; same formula as hw/ddr/host/ddr_host.py) ----
  function automatic logic [31:0] hw(input logic [31:0] widx, input logic [31:0] s);
    logic [31:0] x = widx ^ s;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    x += {s[15:0], s[31:16]} + 32'h9E37_79B9;
    return x ^ (x >> 15);
  endfunction
  function automatic logic [127:0] pat(input logic [26:0] a, input logic [31:0] s);
    logic [31:0] w = {5'd0, a} >> 2;
    return {hw(w + 3, s), hw(w + 2, s), hw(w + 1, s), hw(w, s)};
  endfunction
  function automatic logic [127:0] mem_rd(input logic [26:0] a);
    logic [31:0] b = {5'd0, a} >> 4;
    return u_mem.peek(b);
  endfunction

  // read command addresses (MODE 4 address check)
  logic [26:0] rd_addrs[$];
  always @(posedge clk) if (!rst && rcv && rcr) rd_addrs.push_back(rca);

  // ---- register bus ----
  task automatic bus(input logic w, input logic [11:0] a, input logic [31:0] d, output logic [31:0] q);
    int t = 0;
    @(negedge clk); req = 1; we = w; addr = {12'd0, a}; wd = d;
    @(negedge clk); req = 0;
    while (!ack && t < 2_000_000) begin @(negedge clk); t++; end
    if (!ack) begin errors++; $display("ERROR bus timeout at %h", a); end
    q = rd;
  endtask
  task automatic wreg(input logic [11:0] a, input logic [31:0] d); logic [31:0] q; bus(1, a, d, q); endtask
  function automatic void expect_eq(string what, logic [31:0] got, logic [31:0] exp);
    if (got !== exp) begin errors++; $display("ERROR %s: got %h exp %h", what, got, exp); end
  endfunction
  logic [31:0] q;

  task automatic run(input int mode, input logic [26:0] base, input int len, input int burst, input logic [31:0] seed);
    int t = 0;
    wreg(12'h008, {5'd0, base}); wreg(12'h00C, len); wreg(12'h010, burst); wreg(12'h014, mode); wreg(12'h018, seed);
    wreg(12'h000, 1);
    do begin bus(0, 12'h004, 0, q); t++; end while (q[1] && t < 100000);
    if (q[1]) begin errors++; $display("ERROR mode %0d still busy", mode); end
  endtask
  task automatic rd_stats(output logic [31:0] st, output logic [31:0] errs);
    bus(0, 12'h004, 0, st); bus(0, 12'h020, 0, errs);
  endtask

  // check the memory model against pattern(seed) over [base, base + len beats)
  function automatic int mem_check(input logic [26:0] base, input int len, input logic [31:0] seed);
    int bad = 0;
    for (int i = 0; i < len; i++) begin
      logic [26:0] a = base + 27'(i * 16);
      if (mem_rd(a) !== pat(a, seed)) begin
        if (bad < 3) $display("ERROR mem at %h: %h exp %h", a, mem_rd(a), pat(a, seed));
        bad++;
      end
    end
    return bad;
  endfunction

  initial begin
    logic [31:0] st, e, lmin, lmax, lsum;
    if ($value$plusargs("gap=%d", gap)) ;
    if ($value$plusargs("lat=%d", lat)) ;
    if ($value$plusargs("blat=%d", blat)) ;
    repeat (5) @(negedge clk);
    rst = 0; mrst = 0;
    bus(0, 12'h004, 0, st); expect_eq("STATUS after reset", st, 32'h1);

    // MODE 1 writes the documented pattern; untouched neighbours keep the initial content
    run(1, 27'h000_0FD0, 100, 7, 32'h1234_5678);
    if (mem_check(27'h000_0FD0, 100, 32'h1234_5678)) errors++;
    if (mem_rd(27'h000_0FC0) !== init_beat(32'hFC) || mem_rd(27'h000_0FD0 + 100 * 16) !== init_beat(32'hFD + 100)) begin
      errors++; $display("ERROR MODE 1 wrote outside [BASE, BASE + LEN): below %h/%h above %h/%h", mem_rd(27'h000_0FC0), init_beat(32'hFC),
                         mem_rd(27'h000_0FD0 + 100 * 16), init_beat(32'hFD + 100));
    end
    // MODE 2 on the same region passes
    run(2, 27'h000_0FD0, 100, 16, 32'h1234_5678); rd_stats(st, e);
    expect_eq("MODE 2 STATUS", st, 32'h5); expect_eq("MODE 2 ERRS", e, 0);

    // MODE 0 over burst lengths and odd bases
    for (int i = 0; i < 6; i++) begin
      int bls[6] = '{1, 4, 16, 64, 256, 1000};
      int bl = bls[i];
      logic [26:0] base = 27'h010_0000 + 27'(i * 27'h1_0010);
      run(0, base, 700, bl, 32'hC0DE_0000 + i); rd_stats(st, e);
      expect_eq($sformatf("MODE 0 burst %0d STATUS", bl), st, 32'h5); expect_eq("MODE 0 ERRS", e, 0);
      if (mem_check(base, 700, 32'hC0DE_0000 + i)) errors++;
      bus(0, 12'h030, 0, q); if (q == 0) begin errors++; $display("ERROR WR_CYC = 0"); end
      bus(0, 12'h034, 0, q); if (q == 0) begin errors++; $display("ERROR RD_CYC = 0"); end
    end

    // planted errors: corrupt 3 beats, MODE 2 must count 3 and report the first one
    begin
      logic [26:0] base = 27'h020_0000;
      run(1, base, 64, 16, 32'hAAAA_0001);
      u_mem.poke(32'((base >> 4) + 5), u_mem.peek(32'((base >> 4) + 5)) ^ 128'h1);            // first: low word differs
      u_mem.poke(32'((base >> 4) + 9), u_mem.peek(32'((base >> 4) + 9)) ^ (128'h1 << 77));    // high bits only
      u_mem.poke(32'((base >> 4) + 63), u_mem.peek(32'((base >> 4) + 63)) ^ 128'h100);
      run(2, base, 64, 16, 32'hAAAA_0001); rd_stats(st, e);
      expect_eq("planted STATUS", st, 32'h9); expect_eq("planted ERRS", e, 3);
      bus(0, 12'h024, 0, q); expect_eq("planted ERR_ADDR", q, {5'd0, base + 27'd80});
      bus(0, 12'h028, 0, q); expect_eq("planted ERR_GOT", q, 32'(pat(base + 27'd80, 32'hAAAA_0001)) ^ 32'h1);
      bus(0, 12'h02C, 0, q); expect_eq("planted ERR_EXP", q, 32'(pat(base + 27'd80, 32'hAAAA_0001)));
      // a pass after a fail clears the result
      run(2, base, 5, 16, 32'hAAAA_0001); rd_stats(st, e);
      expect_eq("re-run STATUS", st, 32'h5); expect_eq("re-run ERRS", e, 0);
    end

    // MODE 3: every byte is from A or B as the strobe hash says
    begin
      logic [26:0] base = 27'h030_0F00;
      logic [31:0] sd = 32'h0BAD_F00D;
      int bad = 0;
      run(3, base, 300, 16, sd); rd_stats(st, e);
      expect_eq("MODE 3 STATUS", st, 32'h5); expect_eq("MODE 3 ERRS", e, 0);
      for (int i = 0; i < 300; i++) begin
        logic [26:0] a = base + 27'(i * 16);
        logic [127:0] pa = pat(a, sd), pb = pat(a, ~sd), m = mem_rd(a), x;
        logic [15:0] sb = 16'(hw({5'd0, a} >> 2, sd ^ 32'h5A5A_A5A5));
        for (int k = 0; k < 16; k++) x[8*k +: 8] = sb[k] ? pb[8*k +: 8] : pa[8*k +: 8];
        if (m !== x) bad++;
      end
      if (bad) begin errors++; $display("ERROR MODE 3 memory: %0d bad beats", bad); end
    end

    // MODE 4: reads inside [BASE, BASE + LEN), data checked, latency >= model latency
    begin
      logic [26:0] base = 27'h010_0000;   // written by MODE 0 burst 1 above (seed C0DE0000)
      int distinct[logic [26:0]];
      rd_addrs.delete();
      run(4, base, 700, 200, 32'hC0DE_0000); rd_stats(st, e);
      // every address follows BASE + 16 * floor(h * LEN / 2^16), h = hw({i[8:0], i}, SEED) >> 16
      if (rd_addrs.size() != 200) begin errors++; $display("ERROR MODE 4 issued %0d reads", rd_addrs.size()); end
      foreach (rd_addrs[i]) begin
        logic [39:0] p = 40'(hw({7'd0, 9'(i), 16'(i)}, 32'hC0DE_0000) >> 16) * 40'd700;
        logic [26:0] ea = base + 27'(p[39:16] * 16);
        if (rd_addrs[i] !== ea) begin errors++; if (errors < 10) $display("ERROR MODE 4 read %0d at %h exp %h", i, rd_addrs[i], ea); end
        distinct[rd_addrs[i]] = 1;
      end
      if (distinct.num() < 150) begin errors++; $display("ERROR MODE 4 only %0d distinct addresses", distinct.num()); end
      expect_eq("MODE 4 STATUS", st, 32'h5); expect_eq("MODE 4 ERRS", e, 0);
      bus(0, 12'h038, 0, lmin); bus(0, 12'h03C, 0, lmax); bus(0, 12'h040, 0, lsum);
      if (!(lmin >= 32'(lat) && lmin <= lmax && lsum >= 200 * lmin && lsum <= 200 * lmax)) begin
        errors++; $display("ERROR MODE 4 latency min=%0d max=%0d sum=%0d", lmin, lmax, lsum);
      end
      $display("MODE4 lat min=%0d max=%0d avg=%0d (model lat=%0d)", lmin, lmax, lsum / 200, lat);
      // wrong seed: every read is an error
      run(4, base, 700, 50, 32'h0000_0001); rd_stats(st, e);
      expect_eq("MODE 4 wrong seed ERRS", e, 50);
    end

    // window: one word written with a 4-byte strobe, read back; the other 12 bytes keep their value
    begin
      logic [127:0] m_old = mem_rd(27'h040_0000), m_new;
      wreg(12'h100, 32'h040_0008); wreg(12'h104, 32'hDEAD_BEEF);
      bus(0, 12'h104, 0, q); expect_eq("WIN read", q, 32'hDEAD_BEEF);
      m_new = mem_rd(27'h040_0000);
      if (m_new !== ((m_old & ~(128'hFFFF_FFFF << 64)) | (128'hDEAD_BEEF << 64))) begin
        errors++; $display("ERROR WIN write: %h -> %h", m_old, m_new);
      end
      bus(0, 12'h100, 0, q); expect_eq("WIN_ADDR", q, 32'h040_0008);
    end

    // AXI error -> axi_err and fail
    inj = 1;
    run(1, 27'h050_0000, 4, 4, 1); bus(0, 12'h004, 0, st);
    expect_eq("SLVERR STATUS", st, 32'h19);
    inj = 0;

    if (mem_err) errors += mem_err;
    $display("TB_DDR_TEST gap=%0d lat=%0d errors=%0d", gap, lat, errors);
    $finish;
  end

  initial begin
    #(64'd2_000_000_000);
    $display("TB_DDR_TEST TIMEOUT errors=1");
    $finish;
  end
endmodule
