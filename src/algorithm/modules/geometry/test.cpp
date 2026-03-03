#include "gpu_geo_ransac.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// ─────────────────────────────────────────────────────────────────────────────
// 计时工具（CLOCK_MONOTONIC，纳秒级）
// ─────────────────────────────────────────────────────────────────────────────

static double now_ms(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec * 1e3 + ts.tv_nsec * 1e-6;
}

// ─────────────────────────────────────────────────────────────────────────────
// 测试数据生成
// 真实单应：H = [[1.2, 0, 50], [0, 1.2, 30], [0, 0, 1]]
// 点在图像平面均匀随机分布，避免退化共线情形。
// ─────────────────────────────────────────────────────────────────────────────

static void generate_matches(Match2D* matches, int n, float noise, unsigned seed) {
  srand(seed);
  for (int i = 0; i < n; i++) {
    float x = (float)rand() / RAND_MAX * 1000.0f;
    float y = (float)rand() / RAND_MAX * 800.0f;
    float nx1 = (rand() / (float)RAND_MAX - 0.5f) * noise;
    float ny1 = (rand() / (float)RAND_MAX - 0.5f) * noise;
    float nx2 = (rand() / (float)RAND_MAX - 0.5f) * noise;
    float ny2 = (rand() / (float)RAND_MAX - 0.5f) * noise;
    matches[i].x1 = x + nx1;
    matches[i].y1 = y + ny1;
    matches[i].x2 = x * 1.2f + 50.0f + nx2;
    matches[i].y2 = y * 1.2f + 30.0f + ny2;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// 打印矩阵
// ─────────────────────────────────────────────────────────────────────────────

static void print_matrix(const char* name, float mat[9]) {
  printf("\n=== %s ===\n", name);
  printf("  %8.4f  %8.4f  %8.4f\n", mat[0], mat[1], mat[2]);
  printf("  %8.4f  %8.4f  %8.4f\n", mat[3], mat[4], mat[5]);
  printf("  %8.4f  %8.4f  %8.4f\n", mat[6], mat[7], mat[8]);
}

// ─────────────────────────────────────────────────────────────────────────────
// 效率测试：对指定求解器运行 BENCH_RUNS 次，打印 warm / min / avg / max
// ─────────────────────────────────────────────────────────────────────────────

#define BENCH_RUNS 50

typedef int (*RansacFn)(const Match2D*, int, float[9], float);

static void bench(const char* label, RansacFn fn, const Match2D* matches, int n, float thresh) {
  float mat[9];
  double times[BENCH_RUNS];

  // 热身（第一次含 GPU JIT / SSBO 首次分配开销）
  double t0 = now_ms();
  fn(matches, n, mat, thresh);
  double warm = now_ms() - t0;

  // 正式计时
  for (int i = 0; i < BENCH_RUNS; i++) {
    double ts = now_ms();
    fn(matches, n, mat, thresh);
    times[i] = now_ms() - ts;
  }

  double sum = 0, mn = times[0], mx = times[0];
  for (int i = 0; i < BENCH_RUNS; i++) {
    sum += times[i];
    if (times[i] < mn)
      mn = times[i];
    if (times[i] > mx)
      mx = times[i];
  }
  double avg = sum / BENCH_RUNS;

  printf("[Timing] %-4s  warm=%6.2fms  min=%5.2fms  avg=%5.2fms  max=%5.2fms"
         "  (%d runs, n=%d)\n",
         label, warm, mn, avg, mx, BENCH_RUNS, n);
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────

int main(void) {

  // ── 1. 初始化 ────────────────────────────────────────────────────────────
  printf("Initializing EGL/GPU context...\n");
  GeoRansacConfig cfg;
  cfg.local_size_x = 32;     // 可选：调整工作组大小以测试性能影响
  cfg.num_iterations = 1000; // 可选：调整迭代次数
  if (gpu_geo_init(&cfg) != 0) {
    fprintf(stderr, "Failed to initialize GPU context!\n");
    return -1;
  }
  gpu_geo_set_solver(1); // 可选：切换求解器（0=Jacobi, 1=IPI）
  // ── 2. 正确性验证（n=100, noise=2px）────────────────────────────────────
  printf("\n──────────────────────────────────────────\n");
  printf(" Correctness check  (n=100, noise=2px)\n");
  printf("──────────────────────────────────────────\n");

  const int N_CHECK = 100;
  Match2D pts[N_CHECK];
  generate_matches(pts, N_CHECK, 2.0f, 42);

  float H[9], F[9], E[9];

  int hi = gpu_ransac_H(pts, N_CHECK, H, 2.25f);
  print_matrix("Homography H  (expected ≈ [[1.2,0,50],[0,1.2,30],[0,0,1]])", H);
  printf("  inliers: %d/%d\n", hi, N_CHECK);

  int fi = gpu_ransac_F(pts, N_CHECK, F, 2.0f);
  print_matrix("Fundamental F", F);
  printf("  inliers: %d/%d\n", fi, N_CHECK);

  int ei = gpu_ransac_E(pts, N_CHECK, E, 2.0f);
  print_matrix("Essential E  (pixel coords – for flow check only)", E);
  printf("  inliers: %d/%d\n", ei, N_CHECK);

  // ── 2b. GPU profiling breakdown（诊断单次调用各阶段耗时） ───────────────
  printf("\n──────────────────────────────────────────\n");
  printf(" GPU profiling  (upload / dispatch / readback)\n");
  printf("──────────────────────────────────────────\n");
  gpu_geo_set_verbose(1);
  gpu_ransac_H(pts, N_CHECK, H, 2.25f); /* prints breakdown to stderr */
  gpu_ransac_F(pts, N_CHECK, F, 2.0f);
  gpu_ransac_E(pts, N_CHECK, E, 2.0f);
  gpu_geo_set_verbose(0);
  printf("  (see stderr for GPU/CPU time breakdown)\n");

  // ── 3. 效率测试：Jacobi vs IPI 对比 ─────────────────────────────────────
  printf("\n──────────────────────────────────────────\n");
  printf(" Solver comparison  (%d runs each)\n", BENCH_RUNS);
  printf(" [0] Jacobi = 9x9 symmetric Jacobi (20 sweeps)\n");
  printf(" [1] Chol  = Cholesky inverse iteration (6 solves, cubic convergence)\n");
  printf("──────────────────────────────────────────\n");

  const int SIZES[] = {100, 300, 500, 1000};
  const int NUM_SIZES = (int)(sizeof(SIZES) / sizeof(SIZES[0]));
  const int MAX_N = SIZES[NUM_SIZES - 1];

  Match2D* big = (Match2D*)malloc(MAX_N * sizeof(Match2D));
  if (!big) {
    fprintf(stderr, "malloc failed\n");
    return -1;
  }
  generate_matches(big, MAX_N, 2.0f, 42);

  // 先验证 IPI 正确性
  {
    float Hj[9], Hi[9];
    gpu_geo_set_solver(0);
    gpu_ransac_H(big, 100, Hj, 2.25f);
    gpu_geo_set_solver(1);
    gpu_ransac_H(big, 100, Hi, 2.25f);
    printf("\n  IPI correctness check (n=100 H):\n");
    printf("  Jacobi H[0..2]=[%7.4f %7.4f %7.4f]  inliers=above\n", Hj[0], Hj[1], Hj[2]);
    printf("  IPI    H[0..2]=[%7.4f %7.4f %7.4f]\n", Hi[0], Hi[1], Hi[2]);
    gpu_geo_set_solver(0);
  }

  const char* mname[] = {"H", "F", "E"};
  RansacFn mfn[] = {gpu_ransac_H, gpu_ransac_F, gpu_ransac_E};
  float mthr[] = {2.25f, 2.0f, 2.0f};

  for (int si = 0; si < NUM_SIZES; si++) {
    int n = SIZES[si];
    printf("\n  n = %d:\n", n);
    for (int mi = 0; mi < 3; mi++) {
      char lbl[16];
      snprintf(lbl, sizeof(lbl), "Jac/%s", mname[mi]);
      gpu_geo_set_solver(0);
      bench(lbl, mfn[mi], big, n, mthr[mi]);
      snprintf(lbl, sizeof(lbl), "IPI/%s", mname[mi]);
      gpu_geo_set_solver(1);
      bench(lbl, mfn[mi], big, n, mthr[mi]);
    }
  }
  gpu_geo_set_solver(0);

  free(big);

  // ── 4. 清理 ──────────────────────────────────────────────────────────────
  gpu_geo_shutdown();
  printf("\nDone.\n");
  return 0;
}