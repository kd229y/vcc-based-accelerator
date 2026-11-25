// tb_ddr_mig: L2 (docs/DDR_PORT.md section 7), xsim. Whole board top ddr_uart_top (MMCM + real MIG RTL with
// SIM_BYPASS_INIT_CAL = FAST + uart_bridge + ddr_test + ddr_port) + Micron ddr2_model, driven over the UART.
// BAUD is raised to UI_HZ / 10 so a register access takes ~1 us. Zero board delays.
`timescale 1ps/1ps
module tb_ddr_mig;
  localparam int BAUD = 7_500_000;
  localparam realtime BIT = 1.0e12 / BAUD * 1ps;

  reg clk = 0, rst_btn = 0, rx = 1;
  wire tx;
  wire [3:0] led;
  wire [15:0] dq; wire [1:0] dqs_p, dqs_n, dm; wire [12:0] a; wire [2:0] ba;
  wire ras_n, cas_n, we_n; wire [0:0] ck_p, ck_n, cke, cs_n, odt;
  always #5000 clk = ~clk;   // 100 MHz

  ddr_uart_top #(.BAUD(BAUD)) dut (.clk(clk), .rst_btn(rst_btn), .uart_rx(rx), .uart_tx(tx), .led(led),
    .ddr2_dq(dq), .ddr2_dqs_n(dqs_n), .ddr2_dqs_p(dqs_p), .ddr2_addr(a), .ddr2_ba(ba), .ddr2_ras_n(ras_n),
    .ddr2_cas_n(cas_n), .ddr2_we_n(we_n), .ddr2_ck_p(ck_p), .ddr2_ck_n(ck_n), .ddr2_cke(cke), .ddr2_cs_n(cs_n),
    .ddr2_dm(dm), .ddr2_odt(odt));

  ddr2_model u_mem (.ck(ck_p[0]), .ck_n(ck_n[0]), .cke(cke[0]), .cs_n(cs_n[0]), .ras_n(ras_n), .cas_n(cas_n),
    .we_n(we_n), .dm_rdqs(dm), .ba(ba), .addr(a), .dq(dq), .dqs(dqs_p), .dqs_n(dqs_n), .rdqs_n(), .odt(odt[0]));

  int errors = 0;

  // ---- UART host ----
  task automatic tx_byte(input [7:0] b);
    rx = 0; #(BIT);
    for (int i = 0; i < 8; i++) begin rx = b[i]; #(BIT); end
    rx = 1; #(BIT);
  endtask
  task automatic rx_byte(output [7:0] b);
    fork : wait_rx
      begin @(negedge tx); end
      begin #(64'd2_000_000_000); $display("ERROR UART rx timeout"); errors++; end
    join_any
    disable wait_rx;
    #(BIT * 1.5);
    for (int i = 0; i < 8; i++) begin b[i] = tx; #(BIT); end
  endtask
  task automatic wreg(input [31:0] ad, input [31:0] d);
    reg [7:0] k;
    tx_byte("W");
    for (int i = 0; i < 4; i++) tx_byte(ad[8*i +: 8]);
    for (int i = 0; i < 4; i++) tx_byte(d[8*i +: 8]);
    rx_byte(k);
    if (k != "K") begin errors++; $display("ERROR write %h: reply %h", ad, k); end
  endtask
  task automatic rreg(input [31:0] ad, output [31:0] d);
    tx_byte("R");
    for (int i = 0; i < 4; i++) tx_byte(ad[8*i +: 8]);
    for (int i = 0; i < 4; i++) rx_byte(d[8*i +: 8]);
  endtask

  function automatic void expect_eq(string what, logic [31:0] got, logic [31:0] exp);
    if (got !== exp) begin errors++; $display("ERROR %s: got %h exp %h", what, got, exp); end
    else $display("ok %s = %h", what, got);
  endfunction

  task automatic run(input int mode, input [31:0] base, input [31:0] len, input [31:0] burst, input [31:0] seed,
                     output [31:0] st);
    int t = 0;
    wreg(32'h008, base); wreg(32'h00C, len); wreg(32'h010, burst); wreg(32'h014, mode); wreg(32'h018, seed);
    wreg(32'h000, 1);
    do begin rreg(32'h004, st); t++; end while (st[1] && t < 2000);
  endtask

  initial begin
    reg [31:0] st, q, wc, rc, lmin, lmax, lsum;
    $display("L2: waiting for calibration");
    fork : cal
      begin wait (led[1] === 1'b1); end
      begin #(64'd1_000_000_000); $display("ERROR calibration timeout"); errors++; end
    join_any
    disable cal;
    $display("L2: calibrated at %0t ps", $time);
    rreg(32'h004, st); expect_eq("STATUS idle", st, 32'h1);

    // MODE 0 across a 4 KB boundary, 16-beat commands
    run(0, 32'h0000_0F00, 64, 16, 32'h1357_9BDF, st);
    expect_eq("MODE 0 STATUS", st, 32'h5);
    rreg(32'h020, q); expect_eq("MODE 0 ERRS", q, 0);
    rreg(32'h030, wc); rreg(32'h034, rc);
    $display("MODE 0: 64 beats, WR_CYC=%0d RD_CYC=%0d", wc, rc);
    // MODE 0 with 1-beat commands
    run(0, 32'h0001_0000, 16, 1, 32'h2468_ACE0, st);
    expect_eq("MODE 0 burst 1 STATUS", st, 32'h5);
    // MODE 3 strobes
    run(3, 32'h0002_0000, 32, 8, 32'h0F0F_1234, st);
    expect_eq("MODE 3 STATUS", st, 32'h5);
    rreg(32'h020, q); expect_eq("MODE 3 ERRS", q, 0);
    // MODE 2 with the wrong seed must fail (the checker sees real DDR data)
    run(2, 32'h0000_0F00, 8, 8, 32'h1357_9BDE, st);
    expect_eq("MODE 2 wrong seed STATUS", st, 32'h9);
    rreg(32'h020, q); expect_eq("MODE 2 wrong seed ERRS", q, 8);
    // MODE 4 random reads in the MODE 0 region
    run(4, 32'h0000_0F00, 64, 20, 32'h1357_9BDF, st);
    expect_eq("MODE 4 STATUS", st, 32'h5);
    rreg(32'h038, lmin); rreg(32'h03C, lmax); rreg(32'h040, lsum);
    $display("MODE 4: latency min=%0d max=%0d avg=%0d cycles", lmin, lmax, lsum / 20);
    // window
    wreg(32'h100, 32'h0003_0004); wreg(32'h104, 32'hCAFE_F00D);
    rreg(32'h104, q); expect_eq("WIN read", q, 32'hCAFE_F00D);
    wreg(32'h100, 32'h0003_0008); wreg(32'h104, 32'h1234_5678);
    wreg(32'h100, 32'h0003_0004); rreg(32'h104, q); expect_eq("WIN neighbour kept", q, 32'hCAFE_F00D);

    $display("TB_DDR_MIG errors=%0d", errors);
    $finish;
  end

  initial begin
    #(64'd20_000_000_000);   // 20 ms
    $display("TB_DDR_MIG TIMEOUT errors=1");
    $finish;
  end
endmodule
