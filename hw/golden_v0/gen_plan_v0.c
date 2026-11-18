// Golden PLE run for the board test: fx_ref_solve(problem, seed) of PLE-CSE (environment = fx_env_build).
//   gen_plan_v0 <problem> <seed> <max_iter> <out.txt>
// out.txt: 14 start/goal codes, 7 Halton shifts, 8 stats (result, iters, size_start, size_goal, dd_rejects,
//          edges, nn_queries, path_len), then path_len x 7 waypoint codes. All decimal, one per line.
#include <stdio.h>
#include <stdlib.h>
int fx_ref_env_load(int i, int flat);
int fx_ref_solve(int i, int seed, int max_iter, int range_h, int *stats, short *sg, int *shift, short *path);
int main(int argc, char **argv)
{
    if (argc != 5) { fprintf(stderr, "usage: gen_plan_v0 <problem> <seed> <max_iter> <out.txt>\n"); return 1; }
    int i = atoi(argv[1]), seed = atoi(argv[2]), it = atoi(argv[3]);
    int st[8], sh[7]; short sg[14], path[448];
    if (fx_ref_env_load(i, 0) < 0) { fprintf(stderr, "env load failed\n"); return 2; }
    fx_ref_solve(i, seed, it, 1 << 24, st, sg, sh, path);
    FILE *f = fopen(argv[4], "w");
    for (int k = 0; k < 14; ++k) fprintf(f, "%d\n", sg[k]);
    for (int k = 0; k < 7; ++k) fprintf(f, "%d\n", sh[k]);
    for (int k = 0; k < 8; ++k) fprintf(f, "%d\n", st[k]);
    for (int k = 0; k < st[7] * 7; ++k) fprintf(f, "%d\n", path[k]);
    fclose(f);
    printf("PLAN,%d,seed=%d,result=%d,iters=%d,path=%d\n", i, seed, st[0], st[1], st[7]);
    return 0;
}
