// DPI helpers for tb_sys_v0 (all-problem mode): the raw point cloud of a PLE-CSE problem (pc_sample, same as
// fx_ref_env_load) and the golden format-v0 build (fx_env_build) for the expected point / cell counts.
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#include <stdlib.h>
#include <string.h>
#include "env.h"
#include "problems.h"
#include "fx.h"
#include "fx_pose.h"
#include "fx_env.h"

static void *sys_mem;
static points_t sys_raw;
static fx_env_t sys_env;

int sys_n_problems(void) { return n_problems; }

// Samples problem i, builds its golden environment; returns the raw point count.
int sys_load(int i)
{
    if (!sys_mem) sys_mem = malloc(1u << 27);
    arena_init(sys_mem, 1u << 27);
    fx_pose_init(FX_DELTA_FK_MM * 1e-3, 0, 0);
    pc_sample(&problems[i], &sys_raw);
    fx_env_build(sys_raw.x, sys_raw.y, sys_raw.z, sys_raw.n, fx_w_min, &sys_env);
    return (int)sys_raw.n;
}

// Raw stream word k: point k/3, axis k%3, binary32 bits.
unsigned int sys_word(int k)
{
    const float *a = (k % 3 == 0) ? sys_raw.x : (k % 3 == 1) ? sys_raw.y : sys_raw.z;
    uint32_t u;
    memcpy(&u, &a[k / 3], 4);
    return u;
}

int sys_golden_valid(void) { return sys_env.valid && !sys_env.err; }
int sys_golden_pts(void) { return (int)sys_env.n_pts; }
int sys_golden_cells(void) { return (int)sys_env.n_cells; }
#ifdef __cplusplus
}
#endif
