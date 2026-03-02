/**
 * @file  gpu_geo_ransac.h
 * @brief GPU-accelerated two-view geometry solver (H / F / E) via
 *        OpenGL 4.3 Compute Shaders + EGL headless context.
 *
 * Architecture
 * ─────────────
 *  CPU side
 *    ├─ Hartley normalization of input matches
 *    ├─ Generate NUM_ITER random minimal-sample index sets
 *    ├─ Upload matches + indices to GPU SSBOs
 *    └─ Read back best model, then denormalize
 *
 *  GPU side (Compute Shader, one thread per RANSAC iteration)
 *    ├─ Build 8×9 DLT matrix per minimal sample
 *    ├─ Compute Aᵀ A  (9×9 symmetric)
 *    ├─ Jacobi eigendecomposition → null vector (last right singular vec)
 *    ├─ Rank / singular-value enforcement  (F rank-2 ; E σ₁=σ₂, σ₃=0)
 *    └─ Sampson / symmetric-transfer inlier count across all matches
 *
 * Usage
 * ─────
 *   gpu_geo_init(NULL);                              // once per process
 *
 *   Match2D pts[N];  // fill with correspondences
 *   float H[9];
 *   int inl = gpu_ransac_H(pts, N, H, 1.5f*1.5f);  // squared px threshold
 *
 *   float F[9];
 *   int fi  = gpu_ransac_F(pts, N, F, 2.0f);        // Sampson sq-threshold
 *
 *   float E[9];                                      // needs K⁻¹ coords
 *   int ei  = gpu_ransac_E(pts, N, E, 1e-6f);
 *
 *   gpu_geo_shutdown();
 *
 * Notes
 * ─────
 *  • For E, supply K⁻¹-multiplied coordinates and a matching threshold
 *    in normalised (unitless) squared units.
 *  • NOT thread-safe – protect concurrent calls with an external mutex.
 */

#pragma once
#ifndef GPU_GEO_RANSAC_H
#define GPU_GEO_RANSAC_H

#ifdef __cplusplus
extern "C" {
#endif

/* ── Match type ─────────────────────────────────────────────────────────── */

/** A 2-D point correspondence between two images. */
typedef struct {
    float x1, y1;   /**< Feature location in image 1 (pixels or normalised). */
    float x2, y2;   /**< Feature location in image 2. */
} Match2D;

/* ── Configuration ──────────────────────────────────────────────────────── */

/** Optional runtime tuning – zero-initialise for built-in defaults. */
typedef struct {
    int num_iterations; /**< RANSAC iterations (0 → default 3000). */
    int local_size_x;   /**< Compute-shader workgroup X size (0 → default 64). */
} GeoRansacConfig;

/* ── Life-cycle ─────────────────────────────────────────────────────────── */

/**
 * Initialise EGL surfaceless context + GLEW.
 * @param cfg  NULL for built-in defaults.
 * @return     0 on success, negative error code on failure.
 */
int  gpu_geo_init(const GeoRansacConfig* cfg);

/** Release all GPU resources and terminate the EGL context. */
void gpu_geo_shutdown(void);

/**
 * Enable per-call GPU profiling via GL_TIME_ELAPSED query.
 * When v != 0, each solver call prints a breakdown to stderr.
 */
void gpu_geo_set_verbose(int v);

/**
 * Select null-vector solver used inside the compute shader.
 * @param s  0 = full 9×9 symmetric Jacobi (default, ~44ms on GTX 1060)
 *           1 = Cholesky inverse iteration (fast, ~1ms, same correctness)
 *               B_μ=B+μI factored once; 6 rounds of (L·Lᵀ)\x, cubic convergence.
 * Takes effect on the next gpu_ransac_* call (no re-compile needed).
 */
void gpu_geo_set_solver(int s);

/* ── Solvers ────────────────────────────────────────────────────────────── */

/**
 * Estimate a Homography matrix (3×3, normalised so H[8]=1).
 *
 * Minimal sample : 4 correspondences (8×9 DLT).
 * Error metric   : squared symmetric forward-transfer distance.
 * Normalisation  : Hartley isotropic scaling applied internally.
 *
 * @param matches  Array of 2-D correspondences.
 * @param n        Number of correspondences (≥ 4).
 * @param mat      Output row-major 3×3 matrix (9 floats).
 * @param thresh   Inlier threshold – **squared** pixel distance (e.g. 2.25f for 1.5 px).
 * @return         Inlier count of the best model, or –1 on error.
 */
int gpu_ransac_H(const Match2D* matches, int n, float mat[9], float thresh);

/**
 * Estimate a Fundamental matrix (3×3, rank 2).
 *
 * Minimal sample : 8 correspondences (normalised 8-point algorithm).
 * Error metric   : squared Sampson distance.
 *
 * @param thresh  Sampson threshold (squared pixels, e.g. 1.0–4.0).
 * @return        Inlier count, or –1 on error.
 */
int gpu_ransac_F(const Match2D* matches, int n, float mat[9], float thresh);

/**
 * Estimate an Essential matrix (3×3, σ₁=σ₂, σ₃=0).
 *
 * Minimal sample : 8 normalised correspondences (8-point form).
 * Error metric   : squared Sampson distance in normalised coordinates.
 *
 * The caller must supply coordinates pre-multiplied by K⁻¹ and choose
 * a threshold in normalised squared units (e.g. 1e-6 for typical imagery).
 *
 * @return Inlier count, or –1 on error.
 */
int gpu_ransac_E(const Match2D* matches, int n, float mat[9], float thresh);

/* ── PnP (Resection) ───────────────────────────────────────────────────────── */

/** 3D point (world/camera) + 2D observation (pixel) for PnP. */
typedef struct {
    float x, y, z;   /**< 3D point (world frame). */
    float u, v;      /**< 2D observation (pixels). */
} Point3D2D;

/**
 * PnP RANSAC: estimate camera pose (R, t) from 3D–2D correspondences.
 *
 * Minimal sample : 6 points (DLT).
 * Error metric   : squared reprojection error in pixels.
 *
 * @param pts     Array of n 3D–2D correspondences (world frame, pixel coords).
 * @param n       Number of correspondences (≥ 6).
 * @param K       Row-major 3×3 camera matrix (fx, 0, cx; 0, fy, cy; 0, 0, 1).
 * @param R_out   Output row-major 3×3 rotation (world to camera).
 * @param t_out   Output translation (3 floats, world to camera).
 * @param thresh  Inlier threshold – **squared** reprojection distance (e.g. 64 for 8 px).
 * @param inlier_mask  Optional: if non-NULL, must be n bytes; set to 1 for inliers, 0 otherwise.
 * @return        Inlier count of the best model, or –1 on error.
 */
int gpu_ransac_pnp(const Point3D2D* pts, int n, const float K[9],
                  float R_out[9], float t_out[3],
                  float thresh, unsigned char* inlier_mask);

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif /* GPU_GEO_RANSAC_H */