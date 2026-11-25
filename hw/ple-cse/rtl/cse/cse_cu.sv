// One complete CSE compute unit: ball-to-voxel-to-cell-to-point streaming search.
// Each accepted query gets one of NCTX context entries (LUT/FF, allocated in order at Q, freed when S3 completes it);
// the stage registers and FIFOs carry only the entry id. An entry's killed flag is set at accept (birth generation
// differs from kill_gen) and by every later K that names its tag; it never clears while the entry lives.
module cse_cu
  import ple_cse_pkg::*;
  import cse_cu_pkg::*;
#(
    parameter int unsigned VOXEL_FIFO_DEPTH = 2,
    parameter int unsigned CELL_FIFO_DEPTH = 2,
    parameter int unsigned MAX_PTS_OUTSTANDING = 8,
    parameter bit KILL_ENABLE = 1'b1,
    parameter int unsigned N_BUF = 2,                 // environment buffers: 1 = buffer 0 only (env_buf = 1 answers ERROR)
    parameter int unsigned NCTX = 4                   // queries held in S0..S3
) (
    input  logic                       clk,
    input  logic                       rst_n,
    input  logic                       q_valid,
    output logic                       q_ready,
    input  cse_query_meta_t            q_payload,
    output logic                       r_valid,
    input  logic                       r_ready,
    output resp_t                      r_payload,
    // kill_gen includes this cycle's K and is read only at accept; it may change only in a kill_fire cycle (cse_top)
    input  logic [(1<<TAG_W)-1:0][KILL_GEN_W-1:0] kill_gen,
    input  logic                       kill_fire,                // K handshake this cycle
    input  kill_t                      kill_payload,
    output logic                       cse_idle,
    output logic                       cse_err,
    output logic                       mem_rd_buf,
    output logic [DIR_AW-1:0]          mem_dir_addr,
    input  logic [15:0]                mem_dir_data,
    output logic [SLOT_W-1:0]          mem_cell_addr,
    input  logic [127:0]               mem_cell_data,
    output logic [PT_AW-1:0]           mem_pt_addr,
    input  logic [PT_DATA_W-1:0]       mem_pt_data,
    input  logic [1:0]                 hdr_valid,
    input  logic [1:0]                 hdr_err,
    input  logic [1:0][PT_AW-1:0]      hdr_n_pts,
    input  logic [1:0][2:0][PT_W-1:0] hdr_glo,
    input  logic [1:0][2:0][PT_W-1:0] hdr_ghi
);

  localparam int PEND_W = (MAX_PTS_OUTSTANDING <= 1) ? 1 : $clog2(MAX_PTS_OUTSTANDING + 1);
  localparam int IDW    = (NCTX <= 1) ? 1 : $clog2(NCTX);
  localparam int CNT_W  = $clog2(NCTX + 1);
  localparam bit DBUF   = (N_BUF > 1);
  localparam logic signed [18:0] GRID_LAST = 19'(ENV_G-1);

  typedef logic [IDW-1:0] id_t;
  typedef enum logic [1:0] {S2_IDLE, S2_DIR_WAIT, S2_CELL_WAIT, S2_OUT} s2_state_e;

  typedef struct packed {
    logic [2:0][4:0] lo;
    logic [2:0][4:0] hi;
    logic            skip_search;
  } ball_t;

  typedef struct packed {
    cse_voxel_kind_e   kind;
    id_t               id;
    logic [4:0]        gx;
    logic [4:0]        gy;
    logic [4:0]        gz;
    logic [DIR_AW-1:0] dir_addr;
  } voxel_t;

  typedef struct packed {
    cse_cell_kind_e     kind;
    id_t                id;
    logic [PT_AW-1:0]   base;
    logic [COUNT_W-1:0] count;
  } cell_tok_t;

  typedef struct packed {                            // read by S2
    logic signed [18:0] bmin_x;
    logic signed [18:0] bmin_y;
    logic signed [18:0] bmin_z;
    logic signed [18:0] bmax_x;
    logic signed [18:0] bmax_y;
    logic signed [18:0] bmax_z;
  } bnd_t;

  typedef struct packed {                            // read by S3 and S4
    logic signed [COORD_W-1:0] cx;
    logic signed [COORD_W-1:0] cy;
    logic signed [COORD_W-1:0] cz;
    logic [27:0]               r2;
    logic [USER_W-1:0]         user;
    logic                      error;
  } pctx_t;

  if (MAX_PTS_OUTSTANDING < 1) begin : g_bad_max_points
    $error("cse_cu: MAX_PTS_OUTSTANDING must be at least one");
  end
  if (NCTX < 1 || N_BUF < 1 || N_BUF > 2) begin : g_bad_ctx
    $error("cse_cu: NCTX must be at least one and N_BUF 1 or 2");
  end

  logic q_fire, r_fire;

  // ---- context entries
  logic [NCTX-1:0][TAG_W-1:0]      c_tag;
  logic [NCTX-1:0]                 c_killed;
  logic [NCTX-1:0]                 c_buf;
  (* ram_style = "distributed" *) bnd_t  c_bnd [NCTX];
  (* ram_style = "distributed" *) pctx_t c_pt  [NCTX];
  logic [NCTX-1:0] canc;                             // entry i was killed after it entered the CSE
  logic [NCTX-1:0] k_hit;                            // this cycle's K names entry i's tag
  id_t wp;
  logic [CNT_W-1:0] c_cnt;
  bnd_t  bnd_new, s2_bnd;
  pctx_t pt_new, s3c;

  ball_t s0_payload, s0_new_payload;
  logic s0_valid, s0_ready;
  id_t s0_id;

  ball_t s1_ball;
  id_t s1_id;
  logic s1_active, s1_end;
  logic [4:0] s1_gx, s1_gy, s1_gz;
  logic s1_out_valid, s1_out_ready, s1_out_fire;
  voxel_t s1_out_payload;

  logic voxel_valid, voxel_ready;
  voxel_t voxel_payload;

  s2_state_e s2_state;
  voxel_t s2_voxel;
  cell_tok_t s2_out_payload;
  logic s2_out_valid, s2_out_ready;
  dir_t dir_word;
  cell_t cell_word;

  logic cell_valid, cell_ready;
  cell_tok_t cell_payload;

  logic s3_ball_active, s3_cell_active, s3_end_seen, s3_hit_seen;
  id_t s3_id;
  logic [PT_AW-1:0] s3_issue_addr;
  logic [COUNT_W-1:0] s3_issue_left;
  logic [PEND_W-1:0] s3_pending;
  logic pt_issue, pt_return_valid;
  logic d0_valid, d1_valid, d2_valid;
  cse_dist_d0_t d0_payload;
  cse_dist_d1_t d1_payload;
  cse_dist_d2_t d2_payload;
  logic [37:0] d0_sq_x, d0_sq_y, d0_sq_z;
  logic signed [37:0] d0_dx_ext, d0_dy_ext, d0_dz_ext;
  logic [39:0] d1_sum;
  logic d2_hit, s3_drain_done, s3_hit_after;

  logic s4_valid;
  resp_t s4_payload;
  logic s4_can_accept, s3_complete;

  logic [7:0] outstanding;
  logic unused_internal;

  function automatic logic [4:0] clamp_grid(input logic signed [18:0] value);
    if (value < 0) return 5'd0;
    if (value > GRID_LAST) return GRID_LAST[4:0];
    return value[4:0];
  endfunction

  function automatic id_t id_next(input id_t p);
    return (p == id_t'(NCTX - 1)) ? '0 : p + 1'b1;
  endfunction

  assign q_fire  = q_valid && q_ready;
  assign r_fire  = r_valid && r_ready;

  always_comb
    for (int i = 0; i < NCTX; ++i) begin
      k_hit[i] = KILL_ENABLE && kill_fire && (kill_payload.all || kill_payload.tag == c_tag[i]);
      canc[i]  = KILL_ENABLE && (c_killed[i] || k_hit[i]);
    end

  // S0: capture a query and compute its exact integer bounding box and voxel range.
  always_comb begin : p_s0_preprocess
    logic signed [18:0] cx, cy, cz, radius;
    logic signed [18:0] sx_lo, sy_lo, sz_lo, sx_hi, sy_hi, sz_hi;
    logic signed [18:0] gx_lo, gy_lo, gz_lo, gx_hi, gy_hi, gz_hi;
    logic signed [18:0] hglo_x, hglo_y, hglo_z, hghi_x, hghi_y, hghi_z;
    logic empty_axis, global_cull, hb, error;
    logic [27:0] radius_u;

    hb = DBUF ? q_payload.query.env_buf : 1'b0;
    cx = $signed({q_payload.query.cx[COORD_W-1], q_payload.query.cx});
    cy = $signed({q_payload.query.cy[COORD_W-1], q_payload.query.cy});
    cz = $signed({q_payload.query.cz[COORD_W-1], q_payload.query.cz});
    radius = $signed({5'b0, q_payload.query.r});
    radius_u = {14'b0, q_payload.query.r};
    sx_lo = cx - radius;
    sy_lo = cy - radius;
    sz_lo = cz - radius;
    sx_hi = cx + radius;
    sy_hi = cy + radius;
    sz_hi = cz + radius;
    gx_lo = sx_lo >>> ENV_QSHIFT;
    gy_lo = sy_lo >>> ENV_QSHIFT;
    gz_lo = sz_lo >>> ENV_QSHIFT;
    gx_hi = sx_hi >>> ENV_QSHIFT;
    gy_hi = sy_hi >>> ENV_QSHIFT;
    gz_hi = sz_hi >>> ENV_QSHIFT;
    hglo_x = $signed({3'b000, hdr_glo[hb][0]});
    hglo_y = $signed({3'b000, hdr_glo[hb][1]});
    hglo_z = $signed({3'b000, hdr_glo[hb][2]});
    hghi_x = $signed({3'b000, hdr_ghi[hb][0]});
    hghi_y = $signed({3'b000, hdr_ghi[hb][1]});
    hghi_z = $signed({3'b000, hdr_ghi[hb][2]});
    global_cull = (sx_hi < hglo_x) || (sx_lo > hghi_x) ||
                  (sy_hi < hglo_y) || (sy_lo > hghi_y) ||
                  (sz_hi < hglo_z) || (sz_lo > hghi_z);
    empty_axis = (sx_hi < 0) || (sy_hi < 0) || (sz_hi < 0) ||
                 (gx_lo > GRID_LAST) || (gy_lo > GRID_LAST) || (gz_lo > GRID_LAST);
    error = !hdr_valid[hb] || hdr_err[hb] || (!DBUF && q_payload.query.env_buf);

    bnd_new = '{bmin_x:sx_lo, bmin_y:sy_lo, bmin_z:sz_lo, bmax_x:sx_hi, bmax_y:sy_hi, bmax_z:sz_hi};
    pt_new  = '{cx:q_payload.query.cx, cy:q_payload.query.cy, cz:q_payload.query.cz, r2:radius_u * radius_u,
                user:q_payload.query.user, error:error};

    s0_new_payload.lo[0]       = clamp_grid(gx_lo);
    s0_new_payload.lo[1]       = clamp_grid(gy_lo);
    s0_new_payload.lo[2]       = clamp_grid(gz_lo);
    s0_new_payload.hi[0]       = clamp_grid(gx_hi);
    s0_new_payload.hi[1]       = clamp_grid(gy_hi);
    s0_new_payload.hi[2]       = clamp_grid(gz_hi);
    s0_new_payload.skip_search = error || (hdr_n_pts[hb] == 0) || global_cull || empty_axis;
  end

  assign q_ready = rst_n && (!s0_valid || s0_ready) && (c_cnt != CNT_W'(NCTX));

  // context entries: written once when the query is accepted, freed in order when S3 completes it
  always_ff @(posedge clk) begin
    if (q_fire) begin
      c_bnd[wp] <= bnd_new;
      c_pt[wp]  <= pt_new;
    end
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      c_tag    <= '0;
      c_killed <= '0;
      c_buf <= '0;
      wp    <= '0;
      c_cnt <= '0;
    end else begin
      c_killed <= c_killed | k_hit;
      if (q_fire) begin                                // kill_gen already counts this cycle's K
        c_tag[wp]    <= q_payload.query.tag;
        c_killed[wp] <= KILL_ENABLE && (q_payload.birth_gen != kill_gen[q_payload.query.tag]);
        c_buf[wp] <= DBUF && q_payload.query.env_buf;
        wp <= id_next(wp);
      end
      unique case ({q_fire, s3_complete})
        2'b10: c_cnt <= c_cnt + 1'b1;
        2'b01: c_cnt <= c_cnt - 1'b1;
        default: c_cnt <= c_cnt;
      endcase
    end
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      s0_valid   <= 1'b0;
      s0_payload <= '0;
      s0_id      <= '0;
    end else begin
      if (q_fire) begin
        s0_valid   <= 1'b1;
        s0_payload <= s0_new_payload;
        s0_id      <= wp;
      end else if (s0_ready) begin
        s0_valid <= 1'b0;
      end
    end
  end

  // S1: enumerate x-fastest voxels, then emit exactly one END token.
  assign s0_ready = !s1_active;
  assign s1_out_valid = s1_active;
  assign s1_out_fire  = s1_out_valid && s1_out_ready;
  always_comb begin
    s1_out_payload = '0;
    s1_out_payload.kind = s1_end ? CSE_VOXEL_END : CSE_VOXEL;
    s1_out_payload.id   = s1_id;
    s1_out_payload.gx   = s1_gx;
    s1_out_payload.gy   = s1_gy;
    s1_out_payload.gz   = s1_gz;
    s1_out_payload.dir_addr = ((DIR_AW'(s1_gz) * DIR_AW'(ENV_G)) + DIR_AW'(s1_gy)) *
                              DIR_AW'(ENV_G) + DIR_AW'(s1_gx);
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      s1_active <= 1'b0;
      s1_end    <= 1'b0;
      s1_ball   <= '0;
      s1_id     <= '0;
      s1_gx     <= '0;
      s1_gy     <= '0;
      s1_gz     <= '0;
    end else begin
      if (!s1_active && s0_valid) begin
        s1_active <= 1'b1;
        s1_ball   <= s0_payload;
        s1_id     <= s0_id;
        s1_gx     <= s0_payload.lo[0];
        s1_gy     <= s0_payload.lo[1];
        s1_gz     <= s0_payload.lo[2];
        s1_end    <= s0_payload.skip_search || canc[s0_id];
      end else if (s1_out_fire) begin
        if (s1_end) begin
          s1_active <= 1'b0;
          s1_end    <= 1'b0;
        end else if (canc[s1_id] ||
                     ((s1_gx == s1_ball.hi[0]) && (s1_gy == s1_ball.hi[1]) &&
                      (s1_gz == s1_ball.hi[2]))) begin
          s1_end <= 1'b1;
        end else if (s1_gx != s1_ball.hi[0]) begin
          s1_gx <= s1_gx + 1'b1;
        end else if (s1_gy != s1_ball.hi[1]) begin
          s1_gx <= s1_ball.lo[0];
          s1_gy <= s1_gy + 1'b1;
        end else begin
          s1_gx <= s1_ball.lo[0];
          s1_gy <= s1_ball.lo[1];
          s1_gz <= s1_gz + 1'b1;
        end
      end
    end
  end

  cse_fifo #(.PAYLOAD_T(voxel_t), .DEPTH(VOXEL_FIFO_DEPTH)) u_voxel_fifo (
      .clk, .rst_n, .in_valid(s1_out_valid), .in_ready(s1_out_ready),
      .in_payload(s1_out_payload), .out_valid(voxel_valid), .out_ready(voxel_ready),
      .out_payload(voxel_payload)
  );

  // S2: one-at-a-time DIR/CELL lookup and conservative sphere-box rejection.
  assign dir_word  = dir_t'(mem_dir_data);
  assign cell_word = cell_t'(mem_cell_data);
  assign voxel_ready = (s2_state == S2_IDLE);
  assign s2_out_valid = (s2_state == S2_OUT);
  assign s2_bnd = c_bnd[s2_voxel.id];

  always_comb begin : p_s2_memory
    mem_dir_addr  = '0;
    mem_cell_addr = '0;
    if ((s2_state == S2_IDLE) && voxel_valid && (voxel_payload.kind == CSE_VOXEL) &&
        !canc[voxel_payload.id])
      mem_dir_addr = voxel_payload.dir_addr;
    if ((s2_state == S2_DIR_WAIT) && !canc[s2_voxel.id] && dir_word.occ)
      mem_cell_addr = dir_word.slot;
  end

  always_ff @(posedge clk) begin : p_s2_fsm
    logic signed [18:0] org_x, org_y, org_z;
    logic signed [18:0] alo_x, alo_y, alo_z, ahi_x, ahi_y, ahi_z;
    logic reject;
    if (!rst_n) begin
      s2_state       <= S2_IDLE;
      s2_voxel       <= '0;
      s2_out_payload <= '0;
    end else begin
      unique case (s2_state)
        S2_IDLE: begin
          if (voxel_valid) begin
            if (voxel_payload.kind == CSE_VOXEL_END) begin
              s2_out_payload.kind  <= CSE_CELL_END;
              s2_out_payload.id    <= voxel_payload.id;
              s2_out_payload.base  <= '0;
              s2_out_payload.count <= '0;
              s2_state <= S2_OUT;
            end else if (!canc[voxel_payload.id]) begin
              s2_voxel <= voxel_payload;
              s2_state <= S2_DIR_WAIT;
            end
          end
        end
        S2_DIR_WAIT: begin
          if (canc[s2_voxel.id] || !dir_word.occ) begin
            s2_state <= S2_IDLE;
          end else begin
            s2_state <= S2_CELL_WAIT;
          end
        end
        S2_CELL_WAIT: begin
          org_x = $signed({3'b000, s2_voxel.gx, {ENV_QSHIFT{1'b0}}});
          org_y = $signed({3'b000, s2_voxel.gy, {ENV_QSHIFT{1'b0}}});
          org_z = $signed({3'b000, s2_voxel.gz, {ENV_QSHIFT{1'b0}}});
          alo_x = org_x + $signed({8'b0, cell_word.lo_x});
          alo_y = org_y + $signed({8'b0, cell_word.lo_y});
          alo_z = org_z + $signed({8'b0, cell_word.lo_z});
          ahi_x = org_x + $signed({8'b0, cell_word.hi_x});
          ahi_y = org_y + $signed({8'b0, cell_word.hi_y});
          ahi_z = org_z + $signed({8'b0, cell_word.hi_z});
          reject = (s2_bnd.bmax_x < alo_x) || (s2_bnd.bmin_x > ahi_x) ||
                   (s2_bnd.bmax_y < alo_y) || (s2_bnd.bmin_y > ahi_y) ||
                   (s2_bnd.bmax_z < alo_z) || (s2_bnd.bmin_z > ahi_z);
          if (canc[s2_voxel.id] || (cell_word.count == 0) || reject) begin
            s2_state <= S2_IDLE;
          end else begin
            s2_out_payload.kind  <= CSE_CELL;
            s2_out_payload.id    <= s2_voxel.id;
            s2_out_payload.base  <= cell_word.base;
            s2_out_payload.count <= cell_word.count;
            s2_state <= S2_OUT;
          end
        end
        default: begin
          if (s2_out_ready) s2_state <= S2_IDLE;
        end
      endcase
    end
  end

  cse_fifo #(.PAYLOAD_T(cell_tok_t), .DEPTH(CELL_FIFO_DEPTH)) u_cell_fifo (
      .clk, .rst_n, .in_valid(s2_out_valid), .in_ready(s2_out_ready),
      .in_payload(s2_out_payload), .out_valid(cell_valid), .out_ready(cell_ready),
      .out_payload(cell_payload)
  );

  // The memory port set has one shared buffer select. The interface contract keeps it fixed while busy.
  always_comb begin
    mem_rd_buf = 1'b0;
    if (DBUF) begin
      if (s3_ball_active) mem_rd_buf = c_buf[s3_id];
      else if (s2_state != S2_IDLE) mem_rd_buf = c_buf[s2_voxel.id];
      else if (voxel_valid) mem_rd_buf = c_buf[voxel_payload.id];
      else if (s1_active) mem_rd_buf = c_buf[s1_id];
    end
  end

  // S3: issue at most one point per cycle and drain a fixed D0-D3 distance pipeline.
  assign s3c = c_pt[s3_id];
  assign cell_ready = !s3_ball_active || (!s3_cell_active && !s3_end_seen);
  assign pt_issue = s3_cell_active && !canc[s3_id] && !s3_hit_seen &&
                    ((s3_pending < PEND_W'(MAX_PTS_OUTSTANDING)) || d2_valid);
  assign mem_pt_addr = s3_issue_addr;
  assign d0_dx_ext = {{19{d0_payload.dx[18]}}, d0_payload.dx};
  assign d0_dy_ext = {{19{d0_payload.dy[18]}}, d0_payload.dy};
  assign d0_dz_ext = {{19{d0_payload.dz[18]}}, d0_payload.dz};
  assign d0_sq_x = $unsigned(d0_dx_ext * d0_dx_ext);
  assign d0_sq_y = $unsigned(d0_dy_ext * d0_dy_ext);
  assign d0_sq_z = $unsigned(d0_dz_ext * d0_dz_ext);
  assign d1_sum  = {2'b0, d1_payload.sx} + {2'b0, d1_payload.sy} + {2'b0, d1_payload.sz};
  assign d2_hit  = d2_payload.d2 <= {12'b0, s3c.r2};
  assign s3_drain_done = (s3_pending == PEND_W'(d2_valid)) && !pt_issue;
  assign s3_hit_after  = s3_hit_seen || (d2_valid && d2_hit);
  assign s4_can_accept = !s4_valid || r_ready;
  assign s3_complete = s3_ball_active && s3_end_seen && !s3_cell_active &&
                       s3_drain_done && s4_can_accept;

  always_ff @(posedge clk) begin : p_s3
    logic signed [18:0] px, py, pz;
    if (!rst_n) begin
      s3_ball_active <= 1'b0;
      s3_cell_active <= 1'b0;
      s3_end_seen    <= 1'b0;
      s3_hit_seen    <= 1'b0;
      s3_id          <= '0;
      s3_issue_addr  <= '0;
      s3_issue_left  <= '0;
      s3_pending     <= '0;
      pt_return_valid <= 1'b0;
      d0_valid       <= 1'b0;
      d1_valid       <= 1'b0;
      d2_valid       <= 1'b0;
      d0_payload     <= '0;
      d1_payload     <= '0;
      d2_payload     <= '0;
    end else begin
      pt_return_valid <= pt_issue;
      d0_valid <= pt_return_valid;
      d1_valid <= d0_valid;
      d2_valid <= d1_valid;

      if (pt_return_valid) begin
        px = $signed({3'b000, mem_pt_data[15:0]});
        py = $signed({3'b000, mem_pt_data[31:16]});
        pz = $signed({3'b000, mem_pt_data[47:32]});
        d0_payload.dx <= $signed({s3c.cx[COORD_W-1], s3c.cx}) - px;
        d0_payload.dy <= $signed({s3c.cy[COORD_W-1], s3c.cy}) - py;
        d0_payload.dz <= $signed({s3c.cz[COORD_W-1], s3c.cz}) - pz;
      end
      if (d0_valid) begin
        d1_payload.sx <= d0_sq_x;
        d1_payload.sy <= d0_sq_y;
        d1_payload.sz <= d0_sq_z;
      end
      if (d1_valid) d2_payload.d2 <= d1_sum;

      unique case ({pt_issue, d2_valid})
        2'b10: s3_pending <= s3_pending + 1'b1;
        2'b01: s3_pending <= s3_pending - 1'b1;
        default: s3_pending <= s3_pending;
      endcase

      if (pt_issue) begin
        s3_issue_addr <= s3_issue_addr + 1'b1;
        s3_issue_left <= s3_issue_left - 1'b1;
        if (s3_issue_left == 1) s3_cell_active <= 1'b0;
      end

      if (d2_valid && d2_hit) begin
        s3_hit_seen    <= 1'b1;
        s3_cell_active <= 1'b0;
      end
      if (s3_ball_active && canc[s3_id]) s3_cell_active <= 1'b0;

      if (cell_valid && cell_ready) begin
        if (!s3_ball_active) begin
          s3_ball_active <= 1'b1;
          s3_id          <= cell_payload.id;
          s3_hit_seen    <= 1'b0;
          s3_end_seen    <= (cell_payload.kind == CSE_CELL_END);
          if ((cell_payload.kind == CSE_CELL) && !canc[cell_payload.id]) begin
            s3_cell_active <= 1'b1;
            s3_issue_addr  <= cell_payload.base;
            s3_issue_left  <= cell_payload.count;
          end else begin
            s3_cell_active <= 1'b0;
          end
        end else if (cell_payload.kind == CSE_CELL_END) begin
          s3_end_seen <= 1'b1;
        end else if (!canc[s3_id] && !s3_hit_after) begin
          s3_cell_active <= 1'b1;
          s3_issue_addr  <= cell_payload.base;
          s3_issue_left  <= cell_payload.count;
        end
      end

      if (s3_complete) begin
        s3_ball_active <= 1'b0;
        s3_cell_active <= 1'b0;
        s3_end_seen    <= 1'b0;
        s3_hit_seen    <= 1'b0;
      end
    end
  end

  // S4: hold a completed response stable until the receiver accepts it.
  assign r_valid   = s4_valid;
  assign r_payload = s4_payload;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      s4_valid   <= 1'b0;
      s4_payload <= '0;
      cse_err    <= 1'b0;
    end else begin
      if (r_fire) s4_valid <= 1'b0;
      if (s3_complete) begin
        s4_valid        <= 1'b1;
        s4_payload.tag  <= c_tag[s3_id];
        s4_payload.user <= s3c.user;
        if (s3c.error) begin
          s4_payload.status <= ST_ERROR;
          cse_err <= 1'b1;
        end else if (s3_hit_after) begin
          s4_payload.status <= ST_HIT;
        end else if (canc[s3_id]) begin
          s4_payload.status <= ST_KILLED;
        end else begin
          s4_payload.status <= ST_MISS;
        end
      end
    end
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      outstanding <= '0;
    end else begin
      unique case ({q_fire, r_fire})
        2'b10: outstanding <= outstanding + 1'b1;
        2'b01: outstanding <= outstanding - 1'b1;
        default: outstanding <= outstanding;
      endcase
    end
  end

  assign cse_idle = (outstanding == 0);
  assign unused_internal = ^{s1_ball.skip_search, s2_voxel.kind, s2_voxel.dir_addr,
                             dir_word.rsvd, cell_word.rsvd, q_payload.query.kill_on_hit, c_buf,
                             hdr_valid, hdr_err, hdr_n_pts, hdr_glo, hdr_ghi};

`ifndef SYNTHESIS
`ifndef CSE_NO_SHADOW
  // shadow of the generation compare this flag replaces: canc must equal birth_gen != kill_gen for every live entry
  logic [NCTX-1:0][KILL_GEN_W-1:0] sh_gen;
  logic [NCTX-1:0] sh_live;
  always_ff @(posedge clk) begin
    if (!rst_n) sh_live <= '0;
    else begin
      for (int i = 0; i < NCTX; ++i)
        if (sh_live[i] && canc[i] != (KILL_ENABLE && (sh_gen[i] != kill_gen[c_tag[i]])))
          $error("cse_cu %m: entry %0d killed flag %0d, generation compare says %0d", i, canc[i], !canc[i]);
      if (s3_complete) sh_live[s3_id] <= 1'b0;
      if (q_fire) begin sh_live[wp] <= 1'b1; sh_gen[wp] <= q_payload.birth_gen; end
    end
  end
`endif

  // entries are freed in allocation order: the completing query must be the oldest one
  id_t rp;
  always_ff @(posedge clk) begin
    if (!rst_n) rp <= '0;
    else if (s3_complete) begin
      if (s3_id != rp || c_cnt == 0) $error("cse_cu %m: S3 completed entry %0d, oldest is %0d (count %0d)", s3_id, rp, c_cnt);
      rp <= id_next(rp);
    end
  end

  longint unsigned stat_queries, stat_responses, stat_dir_reads, stat_cell_reads, stat_point_reads;
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      stat_queries <= 0;
      stat_responses <= 0;
      stat_dir_reads <= 0;
      stat_cell_reads <= 0;
      stat_point_reads <= 0;
    end else begin
      if (q_fire) stat_queries <= stat_queries + 1;
      if (r_fire) stat_responses <= stat_responses + 1;
      if ((s2_state == S2_IDLE) && voxel_valid && (voxel_payload.kind == CSE_VOXEL) &&
          !canc[voxel_payload.id]) stat_dir_reads <= stat_dir_reads + 1;
      if ((s2_state == S2_DIR_WAIT) && !canc[s2_voxel.id] && dir_word.occ)
        stat_cell_reads <= stat_cell_reads + 1;
      if (pt_issue) stat_point_reads <= stat_point_reads + 1;
    end
  end

  final $display("CSE_CU_STATS %m q=%0d r=%0d dir=%0d cell=%0d pts=%0d",
                 stat_queries, stat_responses, stat_dir_reads, stat_cell_reads, stat_point_reads);
`endif

endmodule
