// Private payloads for the single-CU CSE streaming pipeline.
package cse_cu_pkg;
  import ple_cse_pkg::*;

  localparam int KILL_GEN_W = 32;

  typedef struct packed {
    query_t                  query;
    logic [KILL_GEN_W-1:0]  birth_gen;
  } cse_query_meta_t;

  typedef enum logic { CSE_VOXEL, CSE_VOXEL_END } cse_voxel_kind_e;
  typedef enum logic { CSE_CELL, CSE_CELL_END } cse_cell_kind_e;

  typedef struct packed {
    logic signed [18:0] dx;
    logic signed [18:0] dy;
    logic signed [18:0] dz;
  } cse_dist_d0_t;

  typedef struct packed {
    logic [37:0] sx;
    logic [37:0] sy;
    logic [37:0] sz;
  } cse_dist_d1_t;

  typedef struct packed {
    logic [39:0] d2;
  } cse_dist_d2_t;

endpackage
