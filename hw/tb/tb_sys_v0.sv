// System test: our build_engine_v0 writes the environment into PLE-CSE's ple_cse_top (real RTL CSE,
// N_CU from the command line), then the PLE plans over AXI-Lite. Every run is compared with PLE-CSE's golden
// fx_ref_solve (status, iterations, tree sizes, edges, DD rejects, NN queries, path length and every waypoint).
// Consecutive problems alternate environment buffers (built in one, MODE.env_buf selects it).
//   +probs=<comma-free list via +p0=.. +p1=..>  +vec=<hw/vectors_v0 dir>  +iters=<max> (default 3000)
module tb_sys_v0;
  import ple_pkg::*;
  import ple_cse_pkg::*;

  import "DPI-C" function int fx_ref_env_load(input int i, input int flat);
  import "DPI-C" function int fx_ref_solve(input int i, input int seed, input int max_iter, input int range_h, output int stats[8],
                                           output shortint sg[14], output int shift[7], output shortint path[448]);
  import "DPI-C" function int sys_n_problems();
  import "DPI-C" function int sys_load(input int i);
  import "DPI-C" function int unsigned sys_word(input int k);
  import "DPI-C" function int sys_golden_valid();
  import "DPI-C" function int sys_golden_pts();
  import "DPI-C" function int sys_golden_cells();

  parameter int N_CU = 1;
  logic clk = 1'b0, rst_n = 1'b0;
  always #5 clk = ~clk;

  // ---------------- AXI-Lite ----------------
  logic [11:0] s_axil_awaddr = '0, s_axil_araddr = '0;
  logic s_axil_awvalid = 0, s_axil_awready, s_axil_wvalid = 0, s_axil_wready, s_axil_bvalid, s_axil_bready = 0;
  logic [31:0] s_axil_wdata = '0, s_axil_rdata;
  logic [3:0] s_axil_wstrb = '0;
  logic [1:0] s_axil_bresp, s_axil_rresp;
  logic s_axil_arvalid = 0, s_axil_arready, s_axil_rvalid, s_axil_rready = 0, irq;

  // ---------------- build engine -> environment write port ----------------
  logic         be_start = 0, be_wbuf = 0, be_ptv = 0, be_ptr, be_busy, be_done, be_wr_en, be_wr_buf;
  logic [31:0]  be_n = 0;
  logic [95:0]  be_pt = '0;
  logic [3:0]   be_status;
  logic [15:0]  be_kept;
  logic [12:0]  be_cells;
  logic [31:0]  be_culled, be_probes, be_cycles;
  logic [1:0]   be_wr_region;
  logic [15:0]  be_wr_addr;
  logic [127:0] be_wr_data;
  logic         w_ready_unused;

  build_engine_v0 #(.HASH_AW(14), .NPT(PT_DEPTH), .NCELL(CELL_DEPTH)) u_be (
    .clk(clk), .rst(!rst_n), .start(be_start), .npoints(be_n), .wbuf(be_wbuf),
    .w_valid(1'b0), .w_ready(w_ready_unused), .w_data(32'd0), .pt_valid(be_ptv), .pt_ready(be_ptr), .pt_data(be_pt),
    .busy(be_busy), .done_pulse(be_done), .status(be_status), .kept(be_kept), .ncells(be_cells),
    .culled(be_culled), .probes(be_probes), .cycles(be_cycles),
    .env_wr_en(be_wr_en), .env_wr_buf(be_wr_buf), .env_wr_region(be_wr_region), .env_wr_addr(be_wr_addr),
    .env_wr_data(be_wr_data));

  ple_cse_top #(.N_CU(N_CU)) u_dut (
    .clk, .rst_n,
    .s_axil_awaddr, .s_axil_awvalid, .s_axil_awready, .s_axil_wdata, .s_axil_wstrb, .s_axil_wvalid, .s_axil_wready,
    .s_axil_bresp, .s_axil_bvalid, .s_axil_bready, .s_axil_araddr, .s_axil_arvalid, .s_axil_arready,
    .s_axil_rdata, .s_axil_rresp, .s_axil_rvalid, .s_axil_rready, .irq,
    .env_wr_en(be_wr_en), .env_wr_buf(be_wr_buf), .env_wr_region(env_region_e'(be_wr_region)),
    .env_wr_addr(be_wr_addr), .env_wr_data(be_wr_data));

  int errs = 0;
  task automatic check(input string what, input longint rtl, input longint gold);
    if (rtl != gold) begin
      if (errs < 30) $display("ERROR %s rtl=%0d expected=%0d", what, rtl, gold);
      errs++;
    end
  endtask

  task automatic axw(input logic [11:0] a, input logic [31:0] d);
    @(negedge clk);
    s_axil_awaddr = a; s_axil_wdata = d; s_axil_wstrb = 4'hF; s_axil_awvalid = 1; s_axil_wvalid = 1;
    do @(posedge clk); while (!(s_axil_awready && s_axil_wready));
    @(negedge clk);
    s_axil_awvalid = 0; s_axil_wvalid = 0; s_axil_bready = 1;
    do @(posedge clk); while (!s_axil_bvalid);
    @(negedge clk);
    s_axil_bready = 0;
  endtask
  task automatic axr(input logic [11:0] a, output logic [31:0] d);
    @(negedge clk);
    s_axil_araddr = a; s_axil_arvalid = 1;
    do @(posedge clk); while (!s_axil_arready);
    @(negedge clk);
    s_axil_arvalid = 0; s_axil_rready = 1;
    do @(posedge clk); while (!s_axil_rvalid);
    d = s_axil_rdata;
    @(negedge clk);
    s_axil_rready = 0;
  endtask
  function automatic logic [31:0] pair(input shortint a, input shortint b);
    return {b, a};
  endfunction

  // ---------------- point stream (one point per cycle) ----------------
  logic [31:0] win [0:700000];
  logic [31:0] meta [0:3];
  int widx = 0, npts = 0;
  always @(posedge clk) begin
    if (be_ptv && be_ptr) widx <= widx + 3;
  end
  always_comb begin
    be_ptv = (widx < 3 * npts) && be_busy;
    be_pt  = {win[widx + 2], win[widx + 1], win[widx]};
  end

  initial begin
    int iters = 3000, nprob = 0;
    int plist[16];
    int st[8], sh[7];
    shortint sg[14], path[448];
    logic [31:0] v, bcyc, pcyc;
    string vec, key, d;
    longint total_build = 0, total_plan = 0, total_raw = 0, total_edges = 0, total_iters = 0, total_queries = 0;
    int first = -1, last = -1, solved = 0, nrun = 0;
    logic [31:0] nq;
    void'($value$plusargs("iters=%d", iters));
    if (!$value$plusargs("vec=%s", vec)) vec = "hw/vectors_v0";
    for (int k = 0; k < 16; ++k) begin
      key = $sformatf("p%0d=%%d", k);
      if ($value$plusargs(key, plist[k])) nprob = k + 1; else break;
    end
    // all-problem mode: +first=<i> +last=<j> (clouds and golden counts from the C model through DPI)
    if ($value$plusargs("first=%d", first) && $value$plusargs("last=%d", last)) nprob = last - first + 1;
    else if (nprob == 0) begin plist[0] = 0; nprob = 1; end
    repeat (3) @(posedge clk);
    rst_n = 1'b1;
    repeat (3) @(posedge clk);

    for (int p = 0; p < nprob; ++p) begin
      int b, pi, sd;
      b = p % 2;
      pi = (first >= 0) ? first + p : plist[p];
      sd = (first >= 0) ? pi : p;
      // ---- build the environment of problem pi into buffer b ----
      if (first >= 0) begin
        npts = sys_load(pi);
        for (int k = 0; k < 3 * npts; ++k) win[k] = sys_word(k);
        meta[0] = npts; meta[1] = sys_golden_valid(); meta[2] = sys_golden_pts(); meta[3] = sys_golden_cells();
      end else begin
        d = $sformatf("%s/p%0d", vec, pi);
        $readmemh({d, "/meta.hex"}, meta);
        npts = int'(meta[0]);
        $readmemh({d, "/points_in.hex"}, win, 0, 3 * npts - 1);
      end
      @(negedge clk); widx = 0; be_n = meta[0]; be_wbuf = b[0]; be_start = 1; @(negedge clk); be_start = 0;
      wait (be_done); @(negedge clk);
      check($sformatf("p%0d build status", pi), be_status, 0);
      check($sformatf("p%0d build kept", pi), be_kept, meta[2]);
      check($sformatf("p%0d build cells", pi), be_cells, meta[3]);
      bcyc = be_cycles;
      // ---- plan on buffer b, compare with fx_ref_solve ----
      if (fx_ref_env_load(pi, 0) < 0) $fatal(1, "golden environment build failed");
      void'(fx_ref_solve(pi, sd, iters, 1 << 24, st, sg, sh, path));
      for (int k = 0; k < 4; ++k) begin
        axw(12'h040 + 12'(4 * k), k < 3 ? pair(sg[2*k], sg[2*k+1]) : {16'd0, sg[6]});
        axw(12'h050 + 12'(4 * k), k < 3 ? pair(sg[7+2*k], sg[7+2*k+1]) : {16'd0, sg[13]});
      end
      for (int j = 0; j < 7; ++j) axw(12'h060 + 12'(4 * j), sh[j]);
      axw(12'h028, iters);
      axw(12'h010, 32'hC | b);                              // MODE: dd + balance, env_buf = b
      axw(12'h000, 32'h5);                                  // start + irq enable
      while (!irq) @(negedge clk);
      axr(12'h004, v); check("result", v[7:4], st[0]);
      axr(12'h080, v); check("iterations", v, st[1]);
      axr(12'h084, v); check("size_start", v[15:0], st[2]); check("size_goal", v[31:16], st[3]);
      axr(12'h088, v); check("path_len", v, st[7]);
      axr(12'h0C4, v); check("edges", v, st[5]);
      axr(12'h0D8, v); check("dd_rejects", v, st[4]);
      axr(12'h0DC, v); check("nn_queries", v, st[6]);
      axr(12'h0C0, pcyc);
      axr(12'h0CC, nq);
      for (int i = 0; i < st[7]; ++i)
        for (int k = 0; k < 4; ++k) begin
          axr(12'h400 + 12'(16 * i + 4 * k), v);
          check($sformatf("path[%0d] word %0d", i, k), v,
                k < 3 ? pair(path[i*7+2*k], path[i*7+2*k+1]) : {16'd0, path[i*7+6]});
        end
      axw(12'h008, 32'h1);
      total_build += bcyc; total_plan += pcyc; total_raw += npts; total_edges += st[5]; total_iters += st[1];
      total_queries += nq; solved += int'(st[0] == 0); nrun++;
      $display("TB_SYS p%0d buf%0d: raw=%0d build kept=%0d cells=%0d cycles=%0d | plan result=%0d iters=%0d edges=%0d queries=%0d path=%0d cycles=%0d errs=%0d",
               pi, b, npts, be_kept, be_cells, bcyc, st[0], st[1], st[5], nq, st[7], pcyc, errs);
    end
    $display("TB_SYS N_CU=%0d problems=%0d solved=%0d build_cycles=%0d plan_cycles=%0d errors=%0d -> %s",
             N_CU, nrun, solved, total_build, total_plan, errs, errs == 0 ? "PASS" : "FAIL");
    $display("TB_SYS_AVG build: %0.2f cycles/raw point, %0.0f cycles/problem | plan: %0.0f cycles/problem, %0.0f cycles/edge, %0.0f cycles/iteration, %0.1f cycles/query",
             real'(total_build) / real'(total_raw), real'(total_build) / nrun, real'(total_plan) / nrun,
             real'(total_plan) / real'(total_edges), real'(total_plan) / real'(total_iters), real'(total_plan) / real'(total_queries));
    $finish;
  end
endmodule
