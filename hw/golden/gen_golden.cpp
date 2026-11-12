// Golden-vector generator for the RTL testbenches.
// Compiles the user's unmodified integer functional model (soc_model/core.cpp)
// and dumps $readmemh-friendly files that the Verilog testbenches compare
// against bit-for-bit.
//
// usage:
//   gen_golden build <points_f32.bin> <out_dir> <hash_cap> <nq> <seed>
//   gen_golden fk    <out_dir> <ncases> <seed>
//   gen_golden plan  <points_f32.bin> <out_dir> <problem.txt>
#include "../../SoC_軟體驗證資料包/soc_model/core.cpp"
#include <cstdio>
#include <fstream>
#include <random>
#include <string>

static long long default_cfg[17] = {
    32768,  // 0 hash_capacity
    4,      // 1 selector_bits
    0,      // 2 raw_point_coverage
    4096,   // 3 rrt_max_nodes_per_tree
    5872025,// 4 floor(0.35 * 2^24)
    8,      // 5 edge_rake
    1,      // 6 ceil(fk_margin * 2^14)
    41,     // 7 ceil(point_radius * 2^14)
    4194304,// 8 sram budget
    1,      // 9 seed
    0,      // 10 trace capacity
    30000,  // 11 state evaluation budget
    300,    // 12 rrt max iterations
    0,      // 13 speculative
    19497,  // 14 ceil(1.19 * 2^14)
    32,     // 15 edge resolution
    0       // 16 goal bias interval
};

static std::vector<float> load_f32(const std::string& path) {
  std::ifstream f(path, std::ios::binary | std::ios::ate);
  if (!f) throw std::runtime_error("cannot open " + path);
  size_t n = f.tellg();
  f.seekg(0);
  std::vector<float> v(n / 4);
  f.read(reinterpret_cast<char*>(v.data()), n);
  return v;
}

static FILE* openw(const std::string& dir, const char* name) {
  FILE* f = std::fopen((dir + "/" + name).c_str(), "w");
  if (!f) throw std::runtime_error(std::string("cannot write ") + name);
  return f;
}

static unsigned rel(int v) { return unsigned(v + 32768) & 0xffff; }

static int cmd_build(const std::string& in, const std::string& out, long long cap, int nq, unsigned seed) {
  long long cfg[17];
  std::copy(default_cfg, default_cfg + 17, cfg);
  cfg[0] = cap;
  Model m(cfg);
  auto raw = load_f32(in);
  int n = int(raw.size() / 3);
  int st = m.build(raw.data(), n);

  // Input stream: 3 words per point (x, y, z raw binary32 bits).
  FILE* f = openw(out, "points_in.hex");
  for (int i = 0; i < 3 * n; i++) {
    uint32_t u;
    std::memcpy(&u, &raw[i], 4);
    std::fprintf(f, "%08x\n", u);
  }
  std::fclose(f);

  // Re-run the build to recover culled/kept when status != SUCCESS (points cleared on error).
  int kept = int(m.points.size());
  int ncells = int(m.cells.size());

  f = openw(out, "exp_points.hex");  // {z,y,x} workspace-relative u16
  for (auto& r : m.points) std::fprintf(f, "%04x%04x%04x\n", rel(r.p[2]), rel(r.p[1]), rel(r.p[0]));
  std::fclose(f);

  f = openw(out, "exp_offsets.hex");
  for (int i = 0; i < 32769; i++) std::fprintf(f, "%08x\n", st == SUCCESS ? m.offsets[i] : 0u);
  std::fclose(f);

  f = openw(out, "exp_cellid.hex");
  for (int i = 0; i < 32768; i++) std::fprintf(f, "%04x\n", (st == SUCCESS && m.lookup[i] >= 0) ? unsigned(m.lookup[i]) : 0xffffu);
  std::fclose(f);

  // cell record: start(16) count(16) lo z,y,x  hi z,y,x (rel u16) -> 128 bits
  f = openw(out, "exp_cells.hex");
  for (auto& c : m.cells)
    std::fprintf(f, "%04x%04x%04x%04x%04x%04x%04x%04x\n", c.start, c.count, rel(c.hi[2]), rel(c.hi[1]), rel(c.hi[0]),
                 rel(c.lo[2]), rel(c.lo[1]), rel(c.lo[0]));
  std::fclose(f);

  // Queries: centers in rel u16, radius u16. Mix of on-point, near-point and random.
  std::mt19937 rng(seed);
  std::vector<std::array<int, 4>> qs;
  for (int i = 0; i < nq; i++) {
    std::array<int, 4> q;
    int kind = rng() % 4;
    if (kind <= 1 && kept > 0) {
      auto& p = m.points[rng() % kept].p;
      int jit = kind == 0 ? 0 : 200;
      for (int a = 0; a < 3; a++) {
        int v = p[a] + (jit ? int(rng() % (2 * jit + 1)) - jit : 0);
        q[a] = std::max(-32768, std::min(32767, v));
      }
      q[3] = rng() % 300;
    } else if (kind == 2) {
      for (int a = 0; a < 3; a++) q[a] = int(rng() % 40000) - 20000;
      q[3] = rng() % 3000;
    } else {
      for (int a = 0; a < 3; a++) q[a] = int(rng() % 65536) - 32768;
      q[3] = rng() % 65536;  // full radius range, exercises wide enumeration
      if (rng() % 4) q[3] &= 0x1fff;
    }
    qs.push_back(q);
  }
  // Boundary probes: exactly touching distance.
  if (kept > 0) {
    for (int i = 0; i < 16 && i < kept; i++) {
      auto& p = m.points[(i * 7919) % kept].p;
      int r = 100;
      int rr = r + 41 + 2;
      std::array<int, 4> q{std::min(32767, p[0] + rr), p[1], p[2], r};
      qs.push_back(q);  // exact touch -> hit
      std::array<int, 4> q2{std::min(32767, p[0] + rr + 1), p[1], p[2], r};
      qs.push_back(q2);  // one past -> maybe miss
    }
  }
  f = openw(out, "queries.hex");
  FILE* g = openw(out, "exp_query.hex");
  int hits = 0;
  for (auto& q : qs) {
    bool hit = st == SUCCESS ? m.query({q[0], q[1], q[2]}, q[3]) : false;
    hits += hit;
    std::fprintf(f, "%04x%04x%04x%04x\n", unsigned(q[3]), rel(q[2]), rel(q[1]), rel(q[0]));
    std::fprintf(g, "%x\n", hit ? 1 : 0);
  }
  std::fclose(f);
  std::fclose(g);

  P glo = m.global_lo, ghi = m.global_hi;
  f = openw(out, "meta.hex");
  // 0 n, 1 status, 2 kept, 3 ncells, 4 nqueries, 5 culled, 6..8 glo rel, 9..11 ghi rel, 12 hash cap, 13 probes
  unsigned meta[14] = {unsigned(n), unsigned(st), unsigned(kept), unsigned(ncells), unsigned(qs.size()),
                       unsigned(m.stats[CULLED]), rel(glo[0]), rel(glo[1]), rel(glo[2]), rel(ghi[0]), rel(ghi[1]),
                       rel(ghi[2]), unsigned(cap), unsigned(m.stats[PROBES])};
  for (unsigned v : meta) std::fprintf(f, "%08x\n", v);
  std::fclose(f);
  std::printf("build %s: n=%d status=%d kept=%d cells=%d culled=%lld queries=%zu hits=%d\n", in.c_str(), n, st, kept,
              ncells, m.stats[CULLED], qs.size(), hits);
  return 0;
}

static int cmd_fk(const std::string& out, int ncases, unsigned seed) {
  std::mt19937 rng(seed);
  FILE* f = openw(out, "fk_in.hex");
  FILE* g = openw(out, "fk_out.hex");
  for (int c = 0; c < ncases; c++) {
    Q q;
    for (int j = 0; j < 7; j++) {
      long long lo = ANGLE_LO[j], hi = ANGLE_HI[j];
      if (c == 0) q[j] = 0;
      else if (c == 1) q[j] = int(lo);
      else if (c == 2) q[j] = int(hi);
      else q[j] = int(lo + (long long)(rng() % uint32_t(hi - lo + 1)));
      std::fprintf(f, "%08x\n", uint32_t(q[j]));
    }
    auto s = integer_fk(q);
    for (int n = 0; n < NS; n++) std::fprintf(g, "%04x%04x%04x\n", rel(s[n][2]), rel(s[n][1]), rel(s[n][0]));
  }
  std::fclose(f);
  std::fclose(g);
  std::printf("fk: %d cases\n", ncases);
  return 0;
}

// plan: problem.txt has 14 ints (start Q24 x7, goal Q24 x7) and optional cfg overrides "idx value" lines.
static int cmd_plan(const std::string& in, const std::string& out, const std::string& prob) {
  long long cfg[17];
  std::copy(default_cfg, default_cfg + 17, cfg);
  std::ifstream pf(prob);
  Q a, b;
  for (int j = 0; j < 7; j++) pf >> a[j];
  for (int j = 0; j < 7; j++) pf >> b[j];
  int idx;
  long long val;
  while (pf >> idx >> val) cfg[idx] = val;
  Model m(cfg);
  auto raw = load_f32(in);
  int n = int(raw.size() / 3);
  int st = m.build(raw.data(), n);
  int ps = st == SUCCESS ? m.plan(a, b) : -1;
  FILE* f = openw(out, "plan_in.hex");
  for (int j = 0; j < 7; j++) std::fprintf(f, "%08x\n", uint32_t(a[j]));
  for (int j = 0; j < 7; j++) std::fprintf(f, "%08x\n", uint32_t(b[j]));
  std::fclose(f);
  f = openw(out, "plan_cfg.hex");
  for (int i = 0; i < 17; i++) std::fprintf(f, "%08x\n", uint32_t(cfg[i]));
  std::fclose(f);
  f = openw(out, "exp_path.hex");
  for (auto& q : m.path)
    for (int j = 0; j < 7; j++) std::fprintf(f, "%08x\n", uint32_t(q[j]));
  std::fclose(f);
  f = openw(out, "plan_meta.hex");
  unsigned meta[8] = {unsigned(ps), unsigned(m.path.size()), unsigned(m.stats[ITERATIONS]), unsigned(m.stats[NODES]),
                      unsigned(m.stats[STATECALL]), unsigned(m.stats[EDGECALL]), unsigned(m.stats[QUERIES]),
                      unsigned(m.stats[FKCALL])};
  for (unsigned v : meta) std::fprintf(f, "%08x\n", v);
  std::fclose(f);
  std::printf("plan: build=%d plan=%d path=%zu iters=%lld nodes=%lld states=%lld edges=%lld queries=%lld\n", st, ps,
              m.path.size(), m.stats[ITERATIONS], m.stats[NODES], m.stats[STATECALL], m.stats[EDGECALL],
              m.stats[QUERIES]);
  return 0;
}

int main(int argc, char** argv) {
  try {
    std::string c = argc > 1 ? argv[1] : "";
    if (c == "build" && argc == 7) return cmd_build(argv[2], argv[3], std::atoll(argv[4]), std::atoi(argv[5]), std::atoi(argv[6]));
    if (c == "fk" && argc == 5) return cmd_fk(argv[2], std::atoi(argv[3]), std::atoi(argv[4]));
    if (c == "plan" && argc == 5) return cmd_plan(argv[2], argv[3], argv[4]);
    std::fprintf(stderr, "bad usage\n");
    return 2;
  } catch (std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
