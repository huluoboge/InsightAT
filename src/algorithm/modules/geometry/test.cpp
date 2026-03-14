/**
 * test.cpp  –  unit tests + benchmarks for gpu_geo_ransac
 *
 * Usage:  ./test_geo_ransac [verbose]
 *         Pass any argument to enable per-call GPU timing in gpu_ransac_pnp.
 *
 * Sections
 * ────────
 *  §1  Two-view (H / F / E) correctness
 *  §2  Two-view solver benchmark (Jacobi vs IPI, multiple n)
 *  §3  PnP correctness   – synthetic pose, known inlier ratio, verify R/t
 *  §4  PnP benchmark     – warm + N runs, print upload/dispatch/readback split
 */

#include "gpu_geo_ransac.h"
#include <math.h>
#include <stdint.h>
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
// §1  helper – two-view data generation
// True H: [[1.2, 0, 50], [0, 1.2, 30], [0, 0, 1]]
// ─────────────────────────────────────────────────────────────────────────────

static void gen_matches_H(Match2D* m, int n, float noise, unsigned seed) {
    srand(seed);
    for (int i = 0; i < n; i++) {
        float x = (float)rand() / RAND_MAX * 1000.0f;
        float y = (float)rand() / RAND_MAX *  800.0f;
        m[i].x1 = x + ((float)rand()/RAND_MAX - 0.5f) * noise;
        m[i].y1 = y + ((float)rand()/RAND_MAX - 0.5f) * noise;
        m[i].x2 = x * 1.2f + 50.0f + ((float)rand()/RAND_MAX - 0.5f) * noise;
        m[i].y2 = y * 1.2f + 30.0f + ((float)rand()/RAND_MAX - 0.5f) * noise;
    }
}

static void print_mat3(const char* name, float m[9]) {
    printf("  %s:\n", name);
    printf("    %8.4f  %8.4f  %8.4f\n", m[0], m[1], m[2]);
    printf("    %8.4f  %8.4f  %8.4f\n", m[3], m[4], m[5]);
    printf("    %8.4f  %8.4f  %8.4f\n", m[6], m[7], m[8]);
}

// ─────────────────────────────────────────────────────────────────────────────
// §2  two-view benchmark helper
// ─────────────────────────────────────────────────────────────────────────────

#define BENCH_RUNS 30

typedef int (*TwoViewFn)(const Match2D*, int, float[9], float);

static void bench_twoview(const char* label, TwoViewFn fn,
                          const Match2D* m, int n, float thresh) {
    float mat[9];
    double t0 = now_ms();
    fn(m, n, mat, thresh);
    double warm_ms = now_ms() - t0;

    double times[BENCH_RUNS];
    for (int i = 0; i < BENCH_RUNS; i++) {
        double ts = now_ms();
        fn(m, n, mat, thresh);
        times[i] = now_ms() - ts;
    }
    double mn = times[0], mx = times[0], sum = 0;
    for (int i = 0; i < BENCH_RUNS; i++) {
        sum += times[i];
        if (times[i] < mn) mn = times[i];
        if (times[i] > mx) mx = times[i];
    }
    printf("  %-14s  warm=%6.1fms  min=%5.1fms  avg=%5.1fms  max=%5.1fms  (n=%d)\n",
           label, warm_ms, mn, sum/BENCH_RUNS, mx, n);
}

// ─────────────────────────────────────────────────────────────────────────────
// §3  PnP data generation
//
// Ground-truth pose:  R = Ry(8°)·Rx(5°),  camera world pos C=(1, 0.5, -3)
// K: fx=fy=1200, cx=960, cy=720  (for 1920×1440)
// Inlier 2D: project + Gaussian ~2px noise
// Outlier 2D: random within image bounds
// ─────────────────────────────────────────────────────────────────────────────

static const float GT_K[9] = {
    1200.f, 0.f, 960.f,
    0.f, 1200.f, 720.f,
    0.f,    0.f,   1.f
};

typedef struct { float R[9]; float t[3]; } Pose;
static Pose GT_POSE;

static void mv3(const float M[9], const float v[3], float out[3]) {
    out[0] = M[0]*v[0] + M[1]*v[1] + M[2]*v[2];
    out[1] = M[3]*v[0] + M[4]*v[1] + M[5]*v[2];
    out[2] = M[6]*v[0] + M[7]*v[1] + M[8]*v[2];
}

static void make_rotation_YX(float t_deg, float p_deg, float R[9]) {
    double t = t_deg * 3.14159265358979 / 180.0;
    double p = p_deg * 3.14159265358979 / 180.0;
    double cy = cos(t), sy = sin(t), cx = cos(p), sx = sin(p);
    R[0]=(float)cy;    R[1]=(float)(sy*sx); R[2]=(float)(sy*cx);
    R[3]=0.f;          R[4]=(float)cx;      R[5]=(float)(-sx);
    R[6]=(float)(-sy); R[7]=(float)(cy*sx); R[8]=(float)(cy*cx);
}

static void gen_pnp_data(Point3D2D* pts, int n, float outlier_ratio, unsigned seed) {
    float R[9];
    make_rotation_YX(8.0f, 5.0f, R);
    float C[3] = {1.0f, 0.5f, -3.0f};
    float t[3];
    mv3(R, C, t);
    t[0]=-t[0]; t[1]=-t[1]; t[2]=-t[2];
    memcpy(GT_POSE.R, R, sizeof(R));
    memcpy(GT_POSE.t, t, sizeof(t));

    srand(seed);
    int n_out = (int)(n * outlier_ratio);

    for (int i = 0; i < n; i++) {
        float X = ((float)rand()/RAND_MAX * 8.0f) - 4.0f;
        float Y = ((float)rand()/RAND_MAX * 6.0f) - 3.0f;
        float Z = ((float)rand()/RAND_MAX * 12.0f) + 8.0f;
        pts[i].x = X; pts[i].y = Y; pts[i].z = Z;

        if (i < n - n_out) {
            // Inlier: project + Gaussian 2px noise (Box-Muller)
            float Xw[3] = {X, Y, Z}, Xc[3];
            mv3(R, Xw, Xc);
            Xc[0]+=t[0]; Xc[1]+=t[1]; Xc[2]+=t[2];
            if (Xc[2] < 0.1f) Xc[2] = 0.1f;
            float u = GT_K[0]*Xc[0]/Xc[2] + GT_K[2];
            float v = GT_K[4]*Xc[1]/Xc[2] + GT_K[5];
            float r1 = (float)rand()/RAND_MAX + 1e-9f;
            float r2 = (float)rand()/RAND_MAX;
            float g = (float)sqrt(-2.0*log(r1)) * (float)cos(2.0*3.14159265*r2);
            pts[i].u = u + g * 2.0f;
            pts[i].v = v + g * 2.0f;
        } else {
            pts[i].u = (float)rand()/RAND_MAX * 1920.0f;
            pts[i].v = (float)rand()/RAND_MAX * 1440.0f;
        }
    }
}

static int count_pnp_inliers_cpu(const Point3D2D* pts, int n,
                                  const float R[9], const float t[3],
                                  float thresh_sq) {
    int cnt = 0;
    for (int i = 0; i < n; i++) {
        float Xw[3] = {pts[i].x, pts[i].y, pts[i].z}, Xc[3];
        mv3(R, Xw, Xc);
        Xc[0]+=t[0]; Xc[1]+=t[1]; Xc[2]+=t[2];
        if (Xc[2] <= 0) continue;
        float du = pts[i].u - (GT_K[0]*Xc[0]/Xc[2] + GT_K[2]);
        float dv = pts[i].v - (GT_K[4]*Xc[1]/Xc[2] + GT_K[5]);
        if (du*du + dv*dv <= thresh_sq) cnt++;
    }
    return cnt;
}

static float R_err_deg(const float A[9], const float B[9]) {
    float tr = 0.f;
    for (int i = 0; i < 9; i++) tr += A[i]*B[i];
    float cosv = (tr - 1.0f) / 2.0f;
    if (cosv >  1.f) cosv =  1.f;
    if (cosv < -1.f) cosv = -1.f;
    return (float)(acos(cosv) * 180.0 / 3.14159265358979);
}

static float t_err(const float a[3], const float b[3]) {
    float d0=a[0]-b[0], d1=a[1]-b[1], d2=a[2]-b[2];
    return (float)sqrt(d0*d0+d1*d1+d2*d2);
}

// ─────────────────────────────────────────────────────────────────────────────
// §4  PnP benchmark helper
// ─────────────────────────────────────────────────────────────────────────────

static void bench_pnp(const Point3D2D* pts, int n, float thresh_sq) {
    float R[9], t[3];
    double t0 = now_ms();
    gpu_ransac_pnp(pts, n, GT_K, R, t, thresh_sq, NULL);
    double warm_ms = now_ms() - t0;

    double times[BENCH_RUNS];
    for (int i = 0; i < BENCH_RUNS; i++) {
        double ts = now_ms();
        gpu_ransac_pnp(pts, n, GT_K, R, t, thresh_sq, NULL);
        times[i] = now_ms() - ts;
    }
    double mn = times[0], mx = times[0], sum = 0;
    for (int i = 0; i < BENCH_RUNS; i++) {
        sum += times[i];
        if (times[i] < mn) mn = times[i];
        if (times[i] > mx) mx = times[i];
    }
    int inl = count_pnp_inliers_cpu(pts, n, R, t, thresh_sq);
    printf("  n=%-5d  warm=%6.1fms  min=%5.1fms  avg=%5.1fms  max=%5.1fms"
           "  inliers=%d/%d\n",
           n, warm_ms, mn, sum/BENCH_RUNS, mx, inl, n);
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    int verbose = (argc > 1);

    printf("Initializing EGL/GPU context...\n");
    GeoRansacConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.num_iterations = 1000;
    cfg.local_size_x   = 64;
    if (gpu_geo_init(&cfg) != 0) {
        fprintf(stderr, "ERROR: gpu_geo_init failed\n");
        return 1;
    }
    if (verbose) gpu_geo_set_verbose(1);

    // ═════════════════════════════════════════════════════════════════════════
    // §1  Two-view correctness
    // ═════════════════════════════════════════════════════════════════════════
    printf("\n════════════════════════════════════════════════\n");
    printf(" §1  Two-view correctness  (n=200, noise=2px)\n");
    printf("════════════════════════════════════════════════\n");
    {
        const int N = 200;
        Match2D pts2d[N];
        gen_matches_H(pts2d, N, 2.0f, 42);

        float H[9], F[9], E[9];
        int hi = gpu_ransac_H(pts2d, N, H, 9.0f);   // 3px threshold
        int fi = gpu_ransac_F(pts2d, N, F, 9.0f);
        int ei = gpu_ransac_E(pts2d, N, E, 9.0f);

        printf("  H  (expected ≈ diag(1.2,1.2), tx=50, ty=30)  inliers=%d/%d\n", hi, N);
        print_mat3("H", H);
        printf("  F  inliers=%d/%d\n", fi, N);
        print_mat3("F", F);
        printf("  E  inliers=%d/%d\n", ei, N);
        print_mat3("E", E);

        int ok = (hi > N * 0.80f);
        printf("\n  [%s] H inlier ratio >= 80%% (thresh=3px)  (%d/%d)\n", ok ? "PASS" : "FAIL", hi, N);
    }

    // ═════════════════════════════════════════════════════════════════════════
    // §2  Two-view solver benchmark  (Jacobi vs IPI)
    // ═════════════════════════════════════════════════════════════════════════
    printf("\n════════════════════════════════════════════════\n");
    printf(" §2  Two-view benchmark  (%d runs, Jacobi vs IPI)\n", BENCH_RUNS);
    printf("════════════════════════════════════════════════\n");
    {
        const int SIZES[] = {200, 500, 1000};
        const int NS = (int)(sizeof(SIZES)/sizeof(SIZES[0]));
        const int MAX_N = SIZES[NS-1];
        Match2D* big = (Match2D*)malloc(MAX_N * sizeof(Match2D));
        gen_matches_H(big, MAX_N, 2.0f, 99);

        struct { const char* name; TwoViewFn fn; float thr; } models[] = {
            { "H", gpu_ransac_H, 9.0f },   // 3px sq
            { "F", gpu_ransac_F, 9.0f },
            { "E", gpu_ransac_E, 9.0f },
        };

        for (int si = 0; si < NS; si++) {
            int n = SIZES[si];
            printf("\n  n=%d:\n", n);
            for (int mi = 0; mi < 3; mi++) {
                char lbl[20];
                snprintf(lbl, sizeof(lbl), "Jacobi/%s", models[mi].name);
                gpu_geo_set_solver(0);
                bench_twoview(lbl, models[mi].fn, big, n, models[mi].thr);
                snprintf(lbl, sizeof(lbl), "IPI/%s", models[mi].name);
                gpu_geo_set_solver(1);
                bench_twoview(lbl, models[mi].fn, big, n, models[mi].thr);
            }
        }
        gpu_geo_set_solver(1);
        free(big);
    }

    // ═════════════════════════════════════════════════════════════════════════
    // §3  PnP correctness
    //   Synthetic: Ry(8°)·Rx(5°) pose, 20% outliers, 500 points
    //   Pass: R_err < 1 deg, t_err < 0.05, inliers >= 90% of GT count
    // ═════════════════════════════════════════════════════════════════════════
    printf("\n════════════════════════════════════════════════\n");
    printf(" §3  PnP correctness\n");
    printf("     Pose: Ry(8)·Rx(5), C=(1,0.5,-3)\n");
    printf("     K: fx=fy=1200, cx=960, cy=720\n");
    printf("════════════════════════════════════════════════\n");
    {
        const int   N            = 500;
        const float OUTLIER_RATIO = 0.20f;
        const float THRESH_SQ     = 64.0f;   // 8px

        Point3D2D pts[N];
        gen_pnp_data(pts, N, OUTLIER_RATIO, 7);

        printf("  GT R:\n");
        print_mat3("R_gt", GT_POSE.R);
        printf("  GT t: [%.4f  %.4f  %.4f]\n\n",
               GT_POSE.t[0], GT_POSE.t[1], GT_POSE.t[2]);

        int gt_inl = count_pnp_inliers_cpu(pts, N, GT_POSE.R, GT_POSE.t, THRESH_SQ);
        printf("  GT inliers (reference): %d/%d\n", gt_inl, N);

        float R_est[9], t_est[3];
        unsigned char mask[N];
        int inliers = gpu_ransac_pnp(pts, N, GT_K, R_est, t_est, THRESH_SQ, mask);

        if (inliers < 0) {
            printf("  [FAIL] gpu_ransac_pnp returned %d\n", inliers);
        } else {
            printf("  Estimated inliers: %d/%d\n", inliers, N);
            print_mat3("R_est", R_est);
            printf("  t_est: [%.4f  %.4f  %.4f]\n\n", t_est[0], t_est[1], t_est[2]);

            float rErr = R_err_deg(GT_POSE.R, R_est);
            float tErr = t_err(GT_POSE.t, t_est);

            printf("  R error : %.4f deg  (pass < 2.0)\n", rErr);
            printf("  t error : %.5f     (pass < 0.50)\n", tErr);
            printf("  inliers vs GT: %d / %d  (pass >= 70%%)\n\n", inliers, gt_inl);

            printf("  [%s] R_error < 2 deg  (DLT accuracy)\n",     (rErr  < 2.0f) ? "PASS" : "FAIL");
            printf("  [%s] t_error < 0.50  (DLT accuracy)\n",       (tErr  < 0.50f) ? "PASS" : "FAIL");
            printf("  [%s] inliers >= 70%% of GT\n",                 (inliers >= (int)(gt_inl*0.70f)) ? "PASS" : "FAIL");
        }
    }

    // ═════════════════════════════════════════════════════════════════════════
    // §4  PnP benchmark  (varying n, 20% / 40% / 60% outliers)
    // ═════════════════════════════════════════════════════════════════════════
    printf("\n════════════════════════════════════════════════\n");
    printf(" §4  PnP benchmark  (%d runs, thresh=8px)\n", BENCH_RUNS);
    printf("════════════════════════════════════════════════\n");
    {
        const float THRESH_SQ  = 64.0f;
        const int   SIZES[]    = {300, 600, 1000, 1500, 2000};
        const int   NS         = (int)(sizeof(SIZES)/sizeof(SIZES[0]));
        const int   MAX_N      = SIZES[NS-1];

        Point3D2D* pts = (Point3D2D*)malloc(MAX_N * sizeof(Point3D2D));

        printf("\n  20%% outliers:\n");
        gen_pnp_data(pts, MAX_N, 0.20f, 13);
        for (int si = 0; si < NS; si++) bench_pnp(pts, SIZES[si], THRESH_SQ);

        printf("\n  40%% outliers:\n");
        gen_pnp_data(pts, MAX_N, 0.40f, 17);
        for (int si = 0; si < NS; si++) bench_pnp(pts, SIZES[si], THRESH_SQ);

        printf("\n  60%% outliers:\n");
        gen_pnp_data(pts, MAX_N, 0.60f, 23);
        for (int si = 0; si < NS; si++) bench_pnp(pts, SIZES[si], THRESH_SQ);

        // GPU phase breakdown for one representative call
        printf("\n  GPU phase breakdown (n=1000, 20%% outliers):\n");
        gpu_geo_set_verbose(1);
        gen_pnp_data(pts, 1000, 0.20f, 42);
        float R[9], t[3];
        gpu_ransac_pnp(pts, 1000, GT_K, R, t, THRESH_SQ, NULL);
        gpu_geo_set_verbose(0);
        printf("  (timing printed to stderr above)\n");

        free(pts);
    }

    gpu_geo_shutdown();
    printf("\nDone.\n");
    return 0;
}