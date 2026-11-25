// Replay one golden CSE trace through the CU array with response backpressure and unordered checking.
module tb_cse_trace #(
    parameter int unsigned NC = 4,
    parameter int unsigned ID = 8,
    parameter int unsigned RD = 8,
    parameter int unsigned NB = 1,                       // environment buffers; 2 with +env1 and +queries1
    parameter int unsigned CX = 4                        // context entries per CU
);
  import ple_cse_pkg::*;

  localparam int MAX_QUERY = 200000;

  logic clk = 1'b0;
  logic rst_n = 1'b0;
  always #5 clk = ~clk;

  logic q_valid, q_ready;
  logic [TAG_W-1:0] q_tag;
  logic [USER_W-1:0] q_user;
  logic q_kill_on_hit, q_env_buf;
  logic signed [COORD_W-1:0] q_cx, q_cy, q_cz;
  logic [RQ_W-1:0] q_r;
  logic r_valid, r_ready;
  logic [TAG_W-1:0] r_tag;
  logic [USER_W-1:0] r_user;
  logic [1:0] r_status;
  logic k_valid, k_ready, k_all;
  logic [TAG_W-1:0] k_tag;
  logic cse_idle, cse_err, eot;

  typedef struct packed {
    logic [TAG_W-1:0] tag;
    status_e status;
  } expected_t;
  expected_t expected[MAX_QUERY];
  bit seen[MAX_QUERY];
  int unsigned n_sent, n_recv, errors;
  logic [15:0] lfsr;

  cse_system #(.N_CU(NC), .INPUT_FIFO_DEPTH(ID), .RESPONSE_FIFO_DEPTH(RD), .N_BUF(NB), .CU_NCTX(CX)) dut (.*);
  ple_cse_if_checker #(.AUTO_KILL(1'b0)) u_checker (.*);

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      lfsr   <= 16'h1;
      r_ready <= 1'b0;
    end else begin
      lfsr   <= {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};
      r_ready <= lfsr[0] | lfsr[3];
    end
  end

  // +kill=N: queries get tags 0..3 (round robin over tags not blocked by P2) and a random K(tag) is sent with
  // probability 1/N per cycle. A response may then be KILLED only if a K of its tag fired after it was accepted;
  // any other status must equal the golden one.
  int unsigned kill_n;
  bit accepted[MAX_QUERY], kill_seen[MAX_QUERY];
  int unsigned out_tag[4];
  bit blocked[4], allow[4];                        // same P2 bookkeeping as ple_cse_if_checker
  int unsigned n_kills, n_killed_resp;

  always @(posedge clk) begin
    if (rst_n && k_valid && k_ready) begin
      n_kills++;
      for (int unsigned i = 0; i < n_sent + 1 && i < MAX_QUERY; ++i)
        if (accepted[i] && !seen[i] && (k_all || expected[i].tag == k_tag)) kill_seen[i] = 1'b1;
      if (out_tag[k_tag[1:0]] != 0 || (q_valid && q_tag == k_tag)) blocked[k_tag[1:0]] = 1'b1;
      if (q_valid && q_tag == k_tag) allow[k_tag[1:0]] = 1'b1;
    end
    if (rst_n && r_valid && r_ready) begin
      int unsigned index;
      index = int'(r_user);
      if (index >= MAX_QUERY || !accepted[index] || seen[index]) begin
        $display("TB_CSE_TRACE ERROR unexpected/duplicate response tag=%0d user=%0h status=%0d",
                 r_tag, r_user, r_status);
        errors++;
      end else begin
        if (r_tag != expected[index].tag ||
            (r_status != expected[index].status && !(r_status == ST_KILLED && kill_seen[index]))) begin
          if (errors < 20)
            $display("TB_CSE_TRACE ERROR index=%0d got tag=%0d status=%0d expected tag=%0d status=%0d killed=%0d",
                     index, r_tag, r_status, expected[index].tag, expected[index].status, kill_seen[index]);
          errors++;
        end
        n_killed_resp += int'(r_status == ST_KILLED);
        seen[index] = 1'b1;
        n_recv++;
        out_tag[r_tag[1:0]]--;
        if (out_tag[r_tag[1:0]] == 0 && !allow[r_tag[1:0]]) blocked[r_tag[1:0]] = 1'b0;
      end
    end
    if (rst_n && q_valid && q_ready) begin
      accepted[int'(q_user)] = 1'b1;
      out_tag[q_tag[1:0]]++;
      allow[q_tag[1:0]] = 1'b0;
    end
  end

  always @(negedge clk) begin
    if (!k_valid || k_ready) begin                     // hold K until its handshake
      if (rst_n && kill_n != 0 && !eot) begin
        k_valid = ($urandom % kill_n) == 0;
        k_tag   = TAG_W'($urandom % 4);
      end else k_valid = 1'b0;
    end
  end

  task automatic send_query(
      input int tag_i, input int user_i, input int koh_i, input int buf_i,
      input int cx_i, input int cy_i, input int cz_i, input int radius_i,
      input int hit_i
  );
    if (n_sent >= MAX_QUERY) $fatal(1, "too many trace queries");
    @(negedge clk);
    if (kill_n != 0) begin                               // next tag not blocked by an earlier kill (P2)
      int t;
      t = -1;
      while (1) begin
        for (int k = 0; k < 4 && t < 0; ++k) if (!blocked[(n_sent + k) % 4]) t = (n_sent + k) % 4;
        if (t >= 0) break;
        @(negedge clk);
      end
      tag_i = t;
    end
    expected[n_sent].tag = TAG_W'(tag_i);
    expected[n_sent].status = (hit_i != 0) ? ST_HIT : ST_MISS;
    seen[n_sent] = 1'b0;
    q_tag = TAG_W'(tag_i);
    q_user = USER_W'(n_sent) ^ (USER_W'(user_i) & '0);
    q_kill_on_hit = koh_i[0];
    q_env_buf = buf_i[0];
    q_cx = COORD_W'(cx_i);
    q_cy = COORD_W'(cy_i);
    q_cz = COORD_W'(cz_i);
    q_r = RQ_W'(radius_i);
    q_valid = 1'b1;
    do @(posedge clk); while (!q_ready);
    n_sent++;
    @(negedge clk);
    q_valid = 1'b0;
  endtask

  initial begin : p_test
    string query_file, query_file1, line;
    int fd, fd1, got;
    int tag_i, user_i, koh_i, buf_i, cx_i, cy_i, cz_i, radius_i, hit_i;
    int unsigned watchdog;

    q_valid = 1'b0;
    q_tag = '0;
    q_user = '0;
    q_kill_on_hit = 1'b0;
    q_env_buf = 1'b0;
    q_cx = '0;
    q_cy = '0;
    q_cz = '0;
    q_r = '0;
    k_valid = 1'b0;
    k_all = 1'b0;
    k_tag = '0;
    eot = 1'b0;
    n_sent = 0;
    n_recv = 0;
    errors = 0;

    repeat (4) @(posedge clk);
    rst_n = 1'b1;
    if (!$value$plusargs("kill=%d", kill_n)) kill_n = 0;
    if (!$value$plusargs("queries=%s", query_file)) $fatal(1, "+queries is required");
    fd = $fopen(query_file, "r");
    if (fd == 0) $fatal(1, "cannot open %s", query_file);
    got = $fgets(line, fd);
    while (!$feof(fd)) begin
      got = $fscanf(fd, "%d %d %d %d %d %d %d %d %d\n",
                    tag_i, user_i, koh_i, buf_i, cx_i, cy_i, cz_i, radius_i, hit_i);
      if (got == 9)
        send_query(tag_i, user_i, koh_i, buf_i, cx_i, cy_i, cz_i, radius_i, hit_i);
      else if (!$feof(fd))
        $fatal(1, "malformed query trace near entry %0d", n_sent);
    end
    $fclose(fd);
    // +queries1: a second trace (against +env1) sent with env_buf = 1 once the CSE is idle (P4: env_buf is fixed
    // while queries are in flight; the CU has one buffer select for all its reads)
    if ($value$plusargs("queries1=%s", query_file1)) begin
      watchdog = 0;
      while ((n_recv != n_sent) || !cse_idle) begin
        @(posedge clk);
        watchdog++;
        if (watchdog > 200000) $fatal(1, "drain timeout before buffer 1, sent=%0d recv=%0d", n_sent, n_recv);
      end
      fd1 = $fopen(query_file1, "r");
      if (fd1 == 0) $fatal(1, "cannot open %s", query_file1);
      got = $fgets(line, fd1);
      while (!$feof(fd1)) begin
        got = $fscanf(fd1, "%d %d %d %d %d %d %d %d %d\n",
                      tag_i, user_i, koh_i, buf_i, cx_i, cy_i, cz_i, radius_i, hit_i);
        if (got == 9)
          send_query(tag_i, user_i, koh_i, 1, cx_i, cy_i, cz_i, radius_i, hit_i);
        else if (!$feof(fd1))
          $fatal(1, "malformed second query trace near entry %0d", n_sent);
      end
      $fclose(fd1);
    end

    watchdog = 0;
    while ((n_recv != n_sent) || !cse_idle) begin
      @(posedge clk);
      watchdog++;
      if (watchdog > 200000) $fatal(1, "drain timeout sent=%0d recv=%0d", n_sent, n_recv);
    end
    @(negedge clk);
    eot = 1'b1;
    repeat (2) @(posedge clk);
    $display("TB_CSE_TRACE queries=%0d responses=%0d errors=%0d checker_errors=%0d cse_err=%0d kills=%0d killed=%0d",
             n_sent, n_recv, errors, u_checker.errors, cse_err, n_kills, n_killed_resp);
    if (errors != 0 || u_checker.errors != 0 || cse_err) $fatal(1, "trace test failed");
    $finish;
  end

  logic unused;
  assign unused = ^{got_unused};
  logic got_unused;
  assign got_unused = 1'b0;

endmodule
