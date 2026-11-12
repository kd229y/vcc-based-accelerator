// Bit-exact check of build_engine + query_engine against golden vectors from core.cpp.
// plusargs: +dir=<vector dir>   (compile with -P tb_build_query.HASH_AW=<n> to match meta[12])
`timescale 1ns/1ps
module tb_build_query;
  parameter HASH_AW = 15;
  parameter CELL_AW = 12;
  parameter PTS_AW  = HASH_AW;
  parameter MAXW    = 200000;

  reg clk = 0, rst = 1;
  always #5 clk = ~clk;

  reg  [8*256-1:0] dir;
  reg  [31:0] meta [0:13];
  reg  [31:0] win  [0:MAXW-1];
  reg  [47:0] exp_p [0:32767];
  reg  [31:0] exp_off [0:32768];
  reg  [15:0] exp_cid [0:32767];
  reg  [127:0] exp_cell [0:32767];
  reg  [63:0] qv [0:4095];
  reg  [0:0]  qe [0:4095];

  // DUT
  reg         start = 0;
  reg  [31:0] npoints = 0;
  reg         w_valid = 0;
  reg  [31:0] w_data = 0;
  wire        w_ready;
  reg         pt_valid = 0;
  reg  [95:0] pt_data = 0;
  wire        pt_ready;
  integer     fullrate;
  wire        busy, env_valid, done_pulse, cell_overflow;
  wire [3:0]  status;
  wire [PTS_AW:0] kept;
  wire [CELL_AW:0] ncells;
  wire [31:0] culled, probes, bcycles;
  wire [47:0] glo, ghi;
  reg  [PTS_AW-1:0] tb_p_raddr = 0;
  reg  [14:0] tb_cnt_raddr = 0, tb_cid_raddr = 0;
  reg  [CELL_AW-1:0] tb_cell_raddr = 0;
  wire [47:0] p_rdata; wire [15:0] cnt_rdata, cid_rdata; wire [31:0] sc_rdata; wire [95:0] box_rdata;
  wire [PTS_AW-1:0] q_p_raddr; wire [14:0] q_cid_raddr; wire [CELL_AW-1:0] q_cell_raddr;
  wire q_busy;
  wire use_q = q_busy;

  build_engine #(.HASH_AW(HASH_AW), .CELL_AW(CELL_AW), .PTS_AW(PTS_AW)) dut (
    .clk(clk), .rst(rst), .start(start), .npoints(npoints),
    .w_valid(w_valid), .w_ready(w_ready), .w_data(w_data),
    .pt_valid(pt_valid), .pt_ready(pt_ready), .pt_data(pt_data),
    .busy(busy), .env_valid(env_valid), .done_pulse(done_pulse), .status(status), .cell_overflow(cell_overflow),
    .kept(kept), .ncells(ncells), .culled(culled), .probes(probes), .cycles(bcycles), .glo(glo), .ghi(ghi),
    .ext_p_raddr(use_q ? q_p_raddr : tb_p_raddr), .ext_p_rdata(p_rdata),
    .ext_cnt_raddr(tb_cnt_raddr), .ext_cnt_rdata(cnt_rdata),
    .ext_cid_raddr(use_q ? q_cid_raddr : tb_cid_raddr), .ext_cid_rdata(cid_rdata),
    .ext_cell_raddr(use_q ? q_cell_raddr : tb_cell_raddr), .ext_sc_rdata(sc_rdata), .ext_box_rdata(box_rdata));

  reg         qstart = 0;
  reg  [15:0] qx, qy, qz, qr;
  wire        q_done, q_hit;
  wire [31:0] q_cycles, q_cells, q_dist;
  query_engine #(.PTS_AW(PTS_AW), .CELL_AW(CELL_AW)) uq (
    .clk(clk), .rst(rst), .start(qstart), .cx(qx), .cy(qy), .cz(qz), .radius(qr),
    .env_valid(env_valid), .kept(kept), .glo(glo), .ghi(ghi),
    .busy(q_busy), .done_pulse(q_done), .hit(q_hit), .cycles(q_cycles), .cells_visited(q_cells), .dist_tests(q_dist),
    .cid_raddr(q_cid_raddr), .cid_rdata(cid_rdata), .cell_raddr(q_cell_raddr), .sc_rdata(sc_rdata), .box_rdata(box_rdata),
    .p_raddr(q_p_raddr), .p_rdata(p_rdata));

  integer i, n, nq, errors, widx, seed, hits;
  reg [63:0] qcyc_total, qcyc_max;
  reg [31:0] off;
  reg [127:0] cellv;

  // stream driver with random gaps
  reg hs = 0, phs = 0;
  always @(posedge clk) begin hs <= w_valid && w_ready; phs <= pt_valid && pt_ready; end
  always @(negedge clk) begin
    if (hs) widx = widx + 1;
    if (phs) widx = widx + 3;
    if (fullrate) begin
      w_valid <= 0;
      if (widx < 3 * n) begin pt_valid <= 1; pt_data <= {win[widx + 2], win[widx + 1], win[widx]}; end
      else pt_valid <= 0;
    end else if (widx < 3 * n && ($random(seed) & 3) != 0) begin w_valid <= 1; w_data <= win[widx]; end
    else w_valid <= 0;
  end

  task check(input cond, input [8*64-1:0] what, input [31:0] idx, input [127:0] got, input [127:0] expv);
    begin
      if (!cond) begin
        errors = errors + 1;
        if (errors < 20) $display("MISMATCH %0s [%0d]: got %h expected %h", what, idx, got, expv);
      end
    end
  endtask

  initial begin
    errors = 0; widx = 0; seed = 1;
    fullrate = $test$plusargs("fullrate");
    if (!$value$plusargs("dir=%s", dir)) begin $display("need +dir="); $finish; end
    $readmemh({dir, "/meta.hex"}, meta);
    n = meta[0]; nq = meta[4];
    if (n > 0) $readmemh({dir, "/points_in.hex"}, win);
    if (meta[2] > 0) $readmemh({dir, "/exp_points.hex"}, exp_p);
    $readmemh({dir, "/exp_offsets.hex"}, exp_off);
    $readmemh({dir, "/exp_cellid.hex"}, exp_cid);
    if (meta[3] > 0) $readmemh({dir, "/exp_cells.hex"}, exp_cell);
    $readmemh({dir, "/queries.hex"}, qv);
    $readmemh({dir, "/exp_query.hex"}, qe);
    if (meta[12] != (1 << HASH_AW)) $display("WARNING: vectors made for hash cap %0d, DUT has %0d", meta[12], 1 << HASH_AW);
    $display("scene %0s: n=%0d expect status=%0d kept=%0d cells=%0d", dir, n, meta[1], meta[2], meta[3]);

    repeat (5) @(posedge clk);
    rst = 0;
    @(negedge clk);
    npoints = n; start = 1;
    @(negedge clk); start = 0;
    wait (done_pulse);
    @(posedge clk);
    $display("build done: status=%0d kept=%0d cells=%0d culled=%0d probes=%0d cycles=%0d", status, kept, ncells, culled, probes, bcycles);
    check(status == meta[1], "status", 0, status, meta[1]);
    if (meta[1] == 0) begin
      check(kept == meta[2], "kept", 0, kept, meta[2]);
      check(ncells == meta[3], "ncells", 0, ncells, meta[3]);
      check(culled == meta[5], "culled", 0, culled, meta[5]);
      if (meta[2] > 0) begin
        check(glo == {meta[8][15:0], meta[7][15:0], meta[6][15:0]}, "glo", 0, glo, {meta[8][15:0], meta[7][15:0], meta[6][15:0]});
        check(ghi == {meta[11][15:0], meta[10][15:0], meta[9][15:0]}, "ghi", 0, ghi, {meta[11][15:0], meta[10][15:0], meta[9][15:0]});
      end
      for (i = 0; i < meta[2]; i = i + 1) begin
        tb_p_raddr = i; @(posedge clk); #1;
        check(p_rdata == exp_p[i], "point", i, p_rdata, exp_p[i]);
      end
      for (i = 0; i < 32768; i = i + 1) begin
        tb_cnt_raddr = i; tb_cid_raddr = i; @(posedge clk); #1;
        check(cid_rdata == exp_cid[i], "cellid", i, cid_rdata, exp_cid[i]);
        check({16'd0, cnt_rdata} == exp_off[i + 1], "offset", i + 1, cnt_rdata, exp_off[i + 1]);
      end
      for (i = 0; i < meta[3]; i = i + 1) begin
        tb_cell_raddr = i; @(posedge clk); #1;
        cellv = {sc_rdata[15:0], sc_rdata[31:16], box_rdata};
        check(cellv == exp_cell[i], "cellv", i, cellv, exp_cell[i]);
      end
    end
    // queries
    hits = 0; qcyc_total = 0; qcyc_max = 0;
    for (i = 0; i < nq; i = i + 1) begin
      {qr, qz, qy, qx} = qv[i];
      @(negedge clk); qstart = 1; @(negedge clk); qstart = 0;
      wait (q_done); @(negedge clk);
      hits = hits + q_hit;
      qcyc_total = qcyc_total + q_cycles; if (q_cycles > qcyc_max) qcyc_max = q_cycles;
      check(q_hit == qe[i], "query", i, {qx, qy, qz, qr}, qe[i]);
    end
    $display("queries: %0d run, %0d hits, avg %0d cycles, max %0d cycles", nq, hits, nq ? qcyc_total / nq : 0, qcyc_max);
    if (errors == 0) $display("PASS %0s", dir);
    else $display("FAIL %0s (%0d mismatches)", dir, errors);
    $finish;
  end
  initial begin #20000000; $display("TIMEOUT st=%0d widx=%0d consumed=%0d", dut.st, widx, dut.consumed); $finish; end
endmodule
