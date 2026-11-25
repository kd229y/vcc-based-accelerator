// cse_cu context entries: query X (tag 0, HIT only in its last voxel) is followed by query Y (tag 1, many voxels);
// Y's tag is killed d = 0..MAXD cycles after Y is accepted. Every stage must use its own query's kill state, so X
// must answer HIT and Y HIT or KILLED for every d. Also runs with NCTX = 1 (the next query waits for a free entry).
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
  logic [(1<<TAG_W)-1:0][KILL_GEN_W-1:0] kill_gen;
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
  int errors = 0, n_hit_y = 0, n_killed_y = 0;

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

  task automatic send(input int tag, input int user, input int cx, input int r);
    @(negedge clk);
    q_payload = '0;
    q_payload.query.tag  = TAG_W'(tag);
    q_payload.query.user = USER_W'(user);
    q_payload.query.cx   = COORD_W'(cx);
    q_payload.query.cy   = COORD_W'(PY);
    q_payload.query.cz   = COORD_W'(PZ);
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
      repeat (3) @(posedge clk);
      rst_n = 1'b1;
      got_x = -1; got_y = -1;
      fork
        begin
          send(0, 'h100 + d, PX - 50, 200);          // X: voxels (0,0,0), (1,0,0); the point is 50 away
          send(1, 'h200 + d, PX - 50, 4000);         // Y: 3 x 3 x 3 voxels around it
          repeat (d) @(posedge clk);
          @(negedge clk) kill_gen[1] = kill_gen[1] + 1'b1;
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
    $display("TB_CSE_CU_CTX nctx=%0d offsets=%0d y_hit=%0d y_killed=%0d errors=%0d", NCTX, MAXD + 1, n_hit_y, n_killed_y, errors);
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
