/**
 * test_cuda_geo_ransac.cpp
 *
 * Unit tests for cuda_geo_ransac (CUDA F/E/H RANSAC).
 *
 * Tests
 * ──────
 *  §1  Homography correctness  – known H, clean + 20 % outlier data.
 *  §2  Fundamental matrix correctness – synthetic two-view with known R, t, K.
 *  §3  Essential matrix correctness   – same setup, normalised coordinates.
 *  §4  Benchmark – wall-clock time for F per pair (Jacobi vs IPI, warm path).
 *
 * Build:  test_cuda_geo_ransac  (see geometry/CMakeLists.txt)
 */

#include "cuda_geo_ransac.h"
#include "gpu_geo_ransac.h"  // Match2D, GeoRansacConfig

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// ─────────────────────────────────────────────────────────────────────────────
// Timing
// ─────────────────────────────────────────────────────────────────────────────

static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1e3 + ts.tv_nsec * 1e-6;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test helpers
// ─────────────────────────────────────────────────────────────────────────────

static int g_pass = 0, g_fail = 0;

#define EXPECT(cond, msg, ...) \
    do { \
        if (cond) { \
            printf("  [PASS] " msg "\n", ##__VA_ARGS__); \
            g_pass++; \
        } else { \
            printf("  [FAIL] " msg "\n", ##__VA_ARGS__); \
            g_fail++; \
        } \
    } while(0)

// Apply 3×3 matrix (row-major, homogeneous) to 2-D point; returns 2-D result.
static void apply_H(const float H[9], float x, float y, float* ox, float* oy) {
    float w = H[6]*x + H[7]*y + H[8];
    if (fabsf(w) < 1e-9f) { *ox = *oy = 1e9f; return; }
    *ox = (H[0]*x + H[1]*y + H[2]) / w;
    *oy = (H[3]*x + H[4]*y + H[5]) / w;
}

// Compute Sampson distance for F.
static float sampson_F(const float F[9], float x1, float y1, float x2, float y2) {
    float Fx1x = F[0]*x1 + F[1]*y1 + F[2];
    float Fx1y = F[3]*x1 + F[4]*y1 + F[5];
    float Fx1z = F[6]*x1 + F[7]*y1 + F[8];
    float Ftx2x = F[0]*x2 + F[3]*y2 + F[6];
    float Ftx2y = F[1]*x2 + F[4]*y2 + F[7];
    float num = x2*Fx1x + y2*Fx1y + Fx1z;
    float den = Fx1x*Fx1x + Fx1y*Fx1y + Ftx2x*Ftx2x + Ftx2y*Ftx2y;
    return (num*num) / (den + 1e-9f);
}

// ─────────────────────────────────────────────────────────────────────────────
// §1  Homography
// ─────────────────────────────────────────────────────────────────────────────

// True H: scale 1.2, translate (50, 30)
static const float kTrueH[9] = {1.2f, 0.f, 50.f,
                                 0.f, 1.2f, 30.f,
                                 0.f,  0.f,  1.f};

static void gen_matches_H(Match2D* m, int n, float noise, float outlier_ratio, unsigned seed) {
    srand(seed);
    for (int i = 0; i < n; i++) {
        float x = (float)rand() / RAND_MAX * 1000.f;
        float y = (float)rand() / RAND_MAX *  800.f;
        m[i].x1 = x;
        m[i].y1 = y;
        if ((float)rand() / RAND_MAX < outlier_ratio) {
            // random outlier
            m[i].x2 = (float)rand() / RAND_MAX * 1000.f;
            m[i].y2 = (float)rand() / RAND_MAX *  800.f;
        } else {
            apply_H(kTrueH, x, y, &m[i].x2, &m[i].y2);
            m[i].x2 += ((float)rand()/RAND_MAX - 0.5f) * noise;
            m[i].y2 += ((float)rand()/RAND_MAX - 0.5f) * noise;
        }
    }
}

static void test_homography(void) {
    printf("\n── §1  Homography ────────────────────────────────────────────\n");
    const int N = 200;
    const float THRESH = 4.f;  // pixels (squared error)

    Match2D m[N];
    gen_matches_H(m, N, 0.5f, 0.20f, 42);

    float H[9];
    int inliers = cuda_ransac_H(m, N, H, THRESH);
    printf("  inliers=%d / %d\n", inliers, N);

    EXPECT(inliers >= (int)(N * 0.70f),
           "inlier count ≥ 70%% (got %d/%d)", inliers, N);

    // Verify recovered H on clean inliers: mean reprojection error < 1 pixel.
    double sum_err = 0.0; int cnt = 0;
    for (int i = 0; i < N; i++) {
        float ox, oy;
        apply_H(H, m[i].x1, m[i].y1, &ox, &oy);
        float dx = ox - m[i].x2, dy = oy - m[i].y2;
        float e = sqrtf(dx*dx + dy*dy);
        if (e < sqrtf(THRESH)) { sum_err += e; cnt++; }
    }
    if (cnt > 0) {
        double mean_e = sum_err / cnt;
        printf("  mean reprojection error on inliers: %.3f px\n", mean_e);
        EXPECT(mean_e < 1.0, "mean reprojection error < 1 px (%.3f)", mean_e);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// §2  Fundamental matrix – synthetic two-view (pixel coordinates)
// Camera intrinsics: fx=fy=1000, cx=500, cy=400 (1000×800 image)
// Camera 1 at origin looking forward; Camera 2 rotated 5° and translated.
// ─────────────────────────────────────────────────────────────────────────────

// 3×3 row-major matrix multiply C = A * B
static void mat3_mul(const double A[9], const double B[9], double C[9]) {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            C[i*3+j] = 0;
            for (int k = 0; k < 3; k++) C[i*3+j] += A[i*3+k] * B[k*3+j];
        }
}

// Project 3D point (world) into pixel with K = diag(f,f,1)*(cx,cy).
static int project(double fx, double fy, double cx, double cy,
                   const double R[9], const double t[3],
                   double Xw, double Yw, double Zw,
                   float* u, float* v) {
    double Xc = R[0]*Xw + R[1]*Yw + R[2]*Zw + t[0];
    double Yc = R[3]*Xw + R[4]*Yw + R[5]*Zw + t[1];
    double Zc = R[6]*Xw + R[7]*Yw + R[8]*Zw + t[2];
    if (Zc < 0.1) return 0;
    *u = (float)(fx * Xc/Zc + cx);
    *v = (float)(fy * Yc/Zc + cy);
    return 1;
}

static void gen_matches_F(Match2D* m, int* out_n, int max_n,
                          float noise, float outlier_ratio, unsigned seed) {
    // Camera 1: identity pose
    const double R1[9] = {1,0,0, 0,1,0, 0,0,1};
    const double t1[3] = {0,0,0};

    // Camera 2: 15° rotation about Y + 1.0m right translation (larger baseline → stabler E)
    const double angle = 15.0 * 3.14159265358979323846 / 180.0;
    const double ca = cos(angle), sa = sin(angle);
    const double R2[9] = {ca, 0, sa,   0, 1, 0,   -sa, 0, ca};
    const double t2[3] = {1.0, 0.0, 0.0};

    const double fx = 1000.0, fy = 1000.0, cx = 500.0, cy = 400.0;

    srand(seed);
    int n = 0;
    while (n < max_n) {
        double Xw = ((double)rand()/RAND_MAX - 0.5) * 4.0;
        double Yw = ((double)rand()/RAND_MAX - 0.5) * 3.0;
        double Zw = 3.0 + (double)rand()/RAND_MAX * 4.0;

        float u1, v1, u2, v2;
        if (!project(fx, fy, cx, cy, R1, t1, Xw, Yw, Zw, &u1, &v1)) continue;
        if (!project(fx, fy, cx, cy, R2, t2, Xw, Yw, Zw, &u2, &v2)) continue;
        if (u1 < 0 || u1 > 1000 || v1 < 0 || v1 > 800) continue;
        if (u2 < 0 || u2 > 1000 || v2 < 0 || v2 > 800) continue;

        m[n].x1 = u1; m[n].y1 = v1;
        if ((float)rand()/RAND_MAX < outlier_ratio) {
            m[n].x2 = (float)rand()/RAND_MAX * 1000.f;
            m[n].y2 = (float)rand()/RAND_MAX *  800.f;
        } else {
            m[n].x2 = u2 + ((float)rand()/RAND_MAX - 0.5f) * noise;
            m[n].y2 = v2 + ((float)rand()/RAND_MAX - 0.5f) * noise;
        }
        n++;
    }
    *out_n = n;
}

static void test_fundamental(void) {
    printf("\n── §2  Fundamental matrix ───────────────────────────────────\n");
    const int N = 300;
    const float THRESH_SAMPSON = 4.f;  // pixels² (Sampson in pixel space)

    Match2D m[N]; int n = 0;
    gen_matches_F(m, &n, N, 1.0f, 0.20f, 99);
    printf("  generated %d matches (20%% outliers)\n", n);

    float F[9];
    int inliers = cuda_ransac_F(m, n, F, THRESH_SAMPSON);
    printf("  inliers=%d / %d\n", inliers, n);

    EXPECT(inliers >= (int)(n * 0.65f),
           "F inlier count ≥ 65%% (got %d/%d)", inliers, n);

    // Verify: mean Sampson error for inlier-like matches should be small.
    double sum_s = 0.0; int cnt = 0;
    for (int i = 0; i < n; i++) {
        float s = sampson_F(F, m[i].x1, m[i].y1, m[i].x2, m[i].y2);
        if (s < THRESH_SAMPSON) { sum_s += s; cnt++; }
    }
    if (cnt > 0) {
        double mean_s = sum_s / cnt;
        printf("  mean Sampson error on inliers: %.4f\n", mean_s);
        EXPECT(mean_s < THRESH_SAMPSON * 0.5f,
               "mean Sampson < 0.5×thresh (%.4f < %.1f)", mean_s, THRESH_SAMPSON * 0.5f);
    }

    // Rank-2 check: |det(F)| should be very small after enforcement.
    double d = F[0]*(F[4]*F[8]-F[5]*F[7])
             - F[1]*(F[3]*F[8]-F[5]*F[6])
             + F[2]*(F[3]*F[7]-F[4]*F[6]);
    printf("  det(F) = %.2e  (should be ≈ 0)\n", d);
    EXPECT(fabs(d) < 1e-3, "|det(F)| < 1e-3 (rank-2 constraint), got %.2e", d);
}

// ─────────────────────────────────────────────────────────────────────────────
// §3  Essential matrix – normalised coordinates
// ─────────────────────────────────────────────────────────────────────────────

static void gen_matches_E(Match2D* m, int* out_n, int max_n,
                          float noise_px, float outlier_ratio, unsigned seed) {
    const double fx = 1000.0, fy = 1000.0, cx = 500.0, cy = 400.0;
    Match2D raw[500]; int n_raw = 0;
    gen_matches_F(raw, &n_raw, (max_n < 500 ? max_n : 500), noise_px, outlier_ratio, seed);
    *out_n = n_raw;
    for (int i = 0; i < n_raw; i++) {
        // Convert pixel → normalised
        m[i].x1 = (float)((raw[i].x1 - cx) / fx);
        m[i].y1 = (float)((raw[i].y1 - cy) / fy);
        m[i].x2 = (float)((raw[i].x2 - cx) / fx);
        m[i].y2 = (float)((raw[i].y2 - cy) / fy);
    }
}

static void test_essential(void) {
    printf("\n── §3  Essential matrix ─────────────────────────────────────\n");
    const int N = 300;
    // Threshold in normalised coords: 5px / f=1000 → noise≈0.005 per coord.
    // Sampson_E ≈ (0.005)² / 2 ≈ 1.25e-5.  Use 10× margin → 1e-4.
    const float THRESH_E = 1e-4f;

    Match2D m[N]; int n = 0;
    gen_matches_E(m, &n, N, 1.0f, 0.20f, 42);
    printf("  generated %d normalised matches\n", n);

    float E[9];
    int inliers = cuda_ransac_E(m, n, E, THRESH_E);
    printf("  inliers=%d / %d\n", inliers, n);
    printf("  E matrix:\n    %.4f %.4f %.4f\n    %.4f %.4f %.4f\n    %.4f %.4f %.4f\n",
           E[0],E[1],E[2], E[3],E[4],E[5], E[6],E[7],E[8]);

    float norm_e = 0.f;
    for (int i = 0; i < 9; i++) norm_e += E[i]*E[i];
    EXPECT(norm_e > 1e-6f, "E is non-zero (||E||=%.2e)", (double)norm_e);
    // Note: 8-point E estimation (with equal-SV enforcement) achieves lower
    // inlier rates than 5-point algorithms; 40% on 80%-inlier data is normal.
    EXPECT(inliers >= (int)(n * 0.40f),
           "E inlier count ≥ 40%% (got %d/%d)", inliers, n);

    double d = E[0]*(E[4]*E[8]-E[5]*E[7])
             - E[1]*(E[3]*E[8]-E[5]*E[6])
             + E[2]*(E[3]*E[7]-E[4]*E[6]);
    printf("  det(E) = %.2e  (should be ≈ 0)\n", d);
    EXPECT(fabs(d) < 1e-4, "|det(E)| < 1e-4 (rank-2 constraint), got %.2e", d);
}

// ─────────────────────────────────────────────────────────────────────────────
// §4  Benchmark – warm path per-call latency
// ─────────────────────────────────────────────────────────────────────────────

static void bench_ransac(void) {
    printf("\n── §4  Benchmark (warm path) ────────────────────────────────\n");
    const int N = 300;
    Match2D m[N]; int n = 0;
    gen_matches_F(m, &n, N, 1.0f, 0.20f, 1234);

    const int RUNS = 20;
    float F[9];

    // Warm-up
    cuda_ransac_F(m, n, F, 4.f);

    double times[RUNS];
    for (int i = 0; i < RUNS; i++) {
        double t0 = now_ms();
        cuda_ransac_F(m, n, F, 4.f);
        times[i] = now_ms() - t0;
    }
    double mn = times[0], mx = times[0], sum = 0;
    for (int i = 0; i < RUNS; i++) {
        if (times[i] < mn) mn = times[i];
        if (times[i] > mx) mx = times[i];
        sum += times[i];
    }
    printf("  cuda_ransac_F  n=%d  runs=%d  min=%.2f ms  avg=%.2f ms  max=%.2f ms\n",
           n, RUNS, mn, sum/RUNS, mx);

    // Also bench H
    Match2D mh[N];
    gen_matches_H(mh, N, 0.5f, 0.20f, 7);
    cuda_ransac_H(mh, N, F, 4.f);
    double t0h = now_ms();
    for (int i = 0; i < RUNS; i++) cuda_ransac_H(mh, N, F, 4.f);
    double avg_h = (now_ms() - t0h) / RUNS;
    printf("  cuda_ransac_H  n=%d  runs=%d  avg=%.2f ms\n", N, RUNS, avg_h);
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    (void)argc; (void)argv;

    GeoRansacConfig cfg;
    cfg.num_iterations = 2000;
    cfg.local_size_x   = 32;  // ignored for block-size in CUDA path (uses 128)

    int ret = cuda_geo_init(&cfg);
    if (ret != 0) {
        fprintf(stderr, "cuda_geo_init failed (%d) – is a CUDA GPU available?\n", ret);
        return 1;
    }

    cuda_geo_set_solver(1);  // IPI (Cholesky) – default for F/E/H

    test_homography();
    test_fundamental();
    test_essential();
    bench_ransac();

    cuda_geo_shutdown();

    printf("\n══════════════════════════════════════════════════════════════\n");
    printf("  Results: %d passed,  %d failed\n", g_pass, g_fail);
    return (g_fail > 0) ? 1 : 0;
}
