// Golden vectors for the format-v0 build engine, from PLE-CSE's own reference (fx_env_build).
//   gen_env_v0 prob <problem_index> <out_dir>      raw cloud = pc_sample(problems[i])  (PLE-CSE pipeline)
//   gen_env_v0 file <points_f32.bin> <out_dir>     raw cloud = Nx3 binary32 file (finite values only)
// Writes: points_in.hex (x,y,z binary32 words per point, input order), meta.hex
//         (n_raw, err/valid, n_pts, n_cells), and env_{hdr,dir,cell,ptx,pty,ptz}.hex via fx_env_dump.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "env.h"
#include "problems.h"
#include "fx.h"
#include "fx_pose.h"
#include "fx_env.h"

void fatal(const char *msg)
{
    fprintf(stderr, "fatal: %s\n", msg);
    exit(2);
}

static fx_env_t e;

int main(int argc, char **argv)
{
    if (argc != 4) { fprintf(stderr, "usage: gen_env_v0 prob <i> <dir> | file <f32.bin> <dir>\n"); return 1; }
    points_t raw;
    float *buf = NULL;
    void *mem = malloc(1u << 27);
    arena_init(mem, 1u << 27);
    fx_pose_init(FX_DELTA_FK_MM * 1e-3, 0, 0);
    if (!strcmp(argv[1], "prob")) {
        pc_sample(&problems[atoi(argv[2])], &raw);
    } else {
        FILE *f = fopen(argv[2], "rb");
        if (!f) fatal("open input");
        fseek(f, 0, SEEK_END);
        long sz = ftell(f);
        fseek(f, 0, SEEK_SET);
        uint32_t n = (uint32_t)(sz / 12);
        buf = malloc(sz ? (size_t)sz : 12);
        if (sz && fread(buf, 12, n, f) != n) fatal("read input");
        fclose(f);
        raw.n = n;
        raw.x = malloc(4 * (n ? n : 1)); raw.y = malloc(4 * (n ? n : 1)); raw.z = malloc(4 * (n ? n : 1));
        for (uint32_t i = 0; i < n; ++i) { raw.x[i] = buf[3 * i]; raw.y[i] = buf[3 * i + 1]; raw.z[i] = buf[3 * i + 2]; }
    }
    fx_env_build(raw.x, raw.y, raw.z, raw.n, fx_w_min, &e);
    char path[1024];
    snprintf(path, sizeof path, "%s/points_in.hex", argv[3]);
    FILE *f = fopen(path, "w");
    if (!f) fatal("write points");
    for (uint32_t i = 0; i < raw.n; ++i) {
        uint32_t u[3];
        memcpy(&u[0], &raw.x[i], 4); memcpy(&u[1], &raw.y[i], 4); memcpy(&u[2], &raw.z[i], 4);
        fprintf(f, "%08x\n%08x\n%08x\n", u[0], u[1], u[2]);
    }
    fclose(f);
    snprintf(path, sizeof path, "%s/meta.hex", argv[3]);
    f = fopen(path, "w");
    fprintf(f, "%08x\n%08x\n%08x\n%08x\n", raw.n, (e.err ? 2u : 0u) | (e.valid ? 1u : 0u), e.n_pts, e.n_cells);
    fclose(f);
    if (fx_env_dump(&e, argv[3], "env")) fatal("dump");
    printf("ENVV0,%s,%s,raw=%u,valid=%d,err=%d,n_pts=%u,n_cells=%u\n", argv[1], argv[2], raw.n, e.valid, e.err, e.n_pts, e.n_cells);
    return 0;
}
