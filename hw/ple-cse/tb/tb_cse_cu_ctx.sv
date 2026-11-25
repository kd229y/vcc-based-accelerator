// cse_cu context entries: query X (tag 0, HIT only in its last voxel) is followed by query Y (tag 1, many voxels);
// Y's tag is killed d = 0..MAXD cycles after Y is accepted. Every stage must use its own query's kill state, so X
// must answer HIT and Y HIT or KILLED for every d. Also runs with NCTX = 1 (the next query waits for a free entry).
// Second part: a long MISS query Z gets K(all) d = 0..MAXZ cycles after it is accepted (one K, or two K in a row),
// or in the same cycle as its accept (born before that K); Z must answer KILLED each time.
module tb_cse_cu_ctx #(
    parameter int unsigned NCTX = 4
);
  import ple_cse_pkg::*;
  import cse_cu_pkg::*;

  localparam int MAXD = 40;
  localparam int PX = 2048 + 100, PY = 1000, PZ = 1000;   // the only point, in voxel (1, 0, 0)

  logic clk = 1'b0;
  logic rst_n = 1'b0;
  always #5 clk = ~clk;

  logic q_valid, q_ready;
  cse_query_meta_t q_payload;
  logic r_valid, r_ready;
  resp_t r_payload;
  logic [(1<<TAG_W)-1:0][KILL_GEN_W-1:0] kill_gen;    // as cse_top's kill_gen_eff: includes this cycle's K
  logic kill_fire;
  kill_t kill_payload;
  logic cse_idle, cse_err;
  logic mem_rd_buf;
  logic [DIR_AW-1:0] mem_dir_addr;
  logic [15:0] mem_dir_data;
  logic [SLOT_W-1:0] mem_cell_addr;
  logic [127:0] mem_cell_data;
  logic [PT_AW-1:0] mem_pt_addr;
  logic [PT_DATA_W-1:0] mem_pt_data;
  logic [1:0] hdr_valid, hdr_err;
  logic [1:0][PT_AW-1:0] hdr_n_pts;
  logic [1:0][2:0][PT_W-1:0] hdr_glo, hdr_ghi;
  cell_t the_cell;
  int errors = 0, n_hit_y = 0, n_killed_y = 0, n_z = 0;
  localparam int MAXZ = 20;
  localparam int ZOFF = 1800;

  cse_cu #(.NCTX(NCTX)) dut (.*);

  always_comb begin
    the_cell = '0;
    the_cell.count = COUNT_W'(1);
    the_cell.base  = '0;
    the_cell.lo_x = AOFF_W'(PX - 2048);
    the_cell.lo_y = AOFF_W'(PY);
    the_cell.lo_z = AOFF_W'(PZ);
    the_cell.hi_x = AOFF_W'(PX - 2048);
    the_cell.hi_y = AOFF_W'(PY);
    the_cell.hi_z = AOFF_W'(PZ);
  end

  // one-cycle reads; only voxel (1, 0, 0) is occupied (slot 0)
  always_ff @(posedge clk) begin
    mem_dir_data  <= (mem_dir_addr == DIR_AW'(1)) ? 16'h8000 : 16'h0000;
    mem_cell_data <= the_cell;
    mem_pt_data   <= {16'(PZ), 16'(PY), 16'(PX)};
  end

  int z_st = -1;                                       // Z's response, captured whenever it comes
  always @(posedge clk) if (rst_n && r_valid && r_payload.tag == 2) z_st = int'(r_payload.status);

  task automatic restart();
    z_st = -1;
    rst_n = 1'b0;
    kill_gen = '0;
    kill_fire = 1'b0;
    kill_payload = '0;
    repeat (3) @(posedge clk);
    rst_n = 1'b1;
  endtask

  task automatic kill_all();                            // one K(all) cycle, as cse_top drives it
    for (int t = 0; t < (1 << TAG_W); ++t) kill_gen[t] = kill_gen[t] + 1'b1;
    kill_fire = 1'b1;
    kill_payload = '{all:1'b1, tag:'0};
  endtask

  task automatic expect_z(input string what, input int d);
    int st;
    while (z_st < 0) @(posedge clk);
    st = z_st;
    n_z++;
    if (st != int'(ST_KILLED)) begin errors++; $display("TB_CSE_CU_CTX ERROR %s d=%0d Z status=%0d (KILLED expected)", what, d, st); end
  endtask

  task automatic send(input int tag, input int user, input int cx, input int r, input int cy = PY, input int cz = PZ);
    @(negedge clk);
    q_payload = '0;
    q_payload.query.tag  = TAG_W'(tag);
    q_payload.query.user = USER_W'(user);
    q_payload.query.cx   = COORD_W'(cx);
    q_payload.query.cy   = COORD_W'(cy);
    q_payload.query.cz   = COORD_W'(cz);
    q_payload.query.r    = RQ_W'(r);
    q_payload.birth_gen  = kill_gen[tag];
    q_valid = 1'b1;
    do @(posedge clk); while (!q_ready);
    @(negedge clk);
    q_valid = 1'b0;
  endtask

  initial begin
    q_valid = 1'b0;
    q_payload = '0;
    r_ready = 1'b1;
    hdr_valid = 2'b01;
    hdr_err = '0;
    hdr_n_pts = '0;
    hdr_n_pts[0] = 1;
    hdr_glo = '0;
    hdr_ghi = '0;
    hdr_glo[0] = {16'(PZ), 16'(PY), 16'(PX)};
    hdr_ghi[0] = {16'(PZ), 16'(PY), 16'(PX)};

    for (int d = 0; d <= MAXD; ++d) begin
      int got_x, got_y;
      rst_n = 1'b0;
      kill_gen = '0;
      kill_fire = 1'b0;
      kill_payload = '0;
      repeat (3) @(posedge clk);
      rst_n = 1'b1;
      got_x = -1; got_y = -1;
      fork
        begin
          send(0, 'h100 + d, PX - 50, 200);          // X: voxels (0,0,0), (1,0,0); the point is 50 away
          send(1, 'h200 + d, PX - 50, 4000);         // Y: 3 x 3 x 3 voxels around it
          repeat (d) @(posedge clk);
          @(negedge clk);                                // K(1) for one cycle, as cse_top drives it
          kill_gen[1] = kill_gen[1] + 1'b1;
          kill_fire = 1'b1;
          kill_payload = '{all:1'b0, tag:TAG_W'(1)};
          @(negedge clk) kill_fire = 1'b0;
        end
        begin
          while (got_x < 0 || got_y < 0) begin
            @(posedge clk);
            if (r_valid) begin
              if (r_payload.tag == 0) got_x = int'(r_payload.status);
              else got_y = int'(r_payload.status);
            end
          end
        end
      join
      if (got_x != int'(ST_HIT)) begin errors++; $display("TB_CSE_CU_CTX ERROR d=%0d X status=%0d (HIT expected)", d, got_x); end
      if (got_y != int'(ST_HIT) && got_y != int'(ST_KILLED)) begin
        errors++; $display("TB_CSE_CU_CTX ERROR d=%0d Y status=%0d", d, got_y);
      end
      n_hit_y += int'(got_y == int'(ST_HIT));
      n_killed_y += int'(got_y == int'(ST_KILLED));
    end
    // Z: 1800 from the point on each axis (box overlaps the global AABB, 4 x 3 x 3 voxels), 3118 away in all: MISS
    for (int twice = 0; twice < 2; ++twice)
      for (int d = 0; d <= MAXZ; ++d) begin
        restart();
        send(2, 'h300 + d, PX + ZOFF, 2500, PY + ZOFF, PZ + ZOFF);
        for (int i = 0; i < d; ++i) @(posedge clk);
        @(negedge clk) kill_all();
        @(negedge clk) if (twice != 0) kill_all(); else kill_fire = 1'b0;
        @(negedge clk) kill_fire = 1'b0;
        expect_z(twice != 0 ? "two K(all)" : "K(all)", d);
      end
    // K(all) in the same cycle as Z's accept; Z was born before it (birth_gen = the old generation)
    restart();
    @(negedge clk);
    q_payload = '0;
    q_payload.query.tag = TAG_W'(2);
    q_payload.query.user = USER_W'('h3ff);
    q_payload.query.cx = COORD_W'(PX + ZOFF);
    q_payload.query.cy = COORD_W'(PY + ZOFF);
    q_payload.query.cz = COORD_W'(PZ + ZOFF);
    q_payload.query.r = RQ_W'(2500);
    q_payload.birth_gen = kill_gen[2];
    q_valid = 1'b1;
    kill_all();
    @(posedge clk);
    if (!q_ready) begin errors++; $display("TB_CSE_CU_CTX ERROR Z not accepted in the K cycle"); end
    @(negedge clk);
    q_valid = 1'b0;
    kill_fire = 1'b0;
    expect_z("K(all) at accept", 0);
    $display("TB_CSE_CU_CTX nctx=%0d offsets=%0d y_hit=%0d y_killed=%0d z_killed=%0d errors=%0d", NCTX, MAXD + 1, n_hit_y, n_killed_y, n_z, errors);
    if (errors != 0) $fatal(1, "context test failed");
    $finish;
  end

  initial begin
    #2_000_000;
    $fatal(1, "TB_CSE_CU_CTX timeout");
  end

  logic unused;
  assign unused = ^{cse_idle, cse_err, mem_rd_buf, mem_cell_addr, mem_pt_addr};

endmodule
