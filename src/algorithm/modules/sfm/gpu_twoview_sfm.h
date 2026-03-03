/**
 * @file  gpu_twoview_sfm.h
 * @brief GPU-accelerated two-view SfM primitives via OpenGL 4.3 Compute
 *        Shaders + EGL headless context.
 *
 * Architecture (same pattern as gpu_geo_ransac)
 * ─────────────────────────────────────────────
 *  Two compute-shader programs:
 *
 *  1. triangulate_shader
 *     Each GPU thread processes ONE correspondence (x1n,y1n)↔(x2n,y2n).
 *     Builds the 4×4 homogeneous DLT system from P1=[I|0] and P2=[R|t],
 *     finds null vector via 4×4 Cholesky inverse-power iteration,
 *     dehomogenizes to get (X, Y, Z).
 *     Invalid points (behind camera or |W|<ε) are written as (NaN, NaN, NaN).
 *
 *  2. residuals_shader
 *     Each GPU thread processes ONE 3D point.
 *     Projects X through cam1 (identity) and cam2 (R,t) using focal f and
 *     principal point (cx, cy), computes 4 reprojection residuals, and applies
 *     Huber soft-weighting.  Per-thread results are written to an SSBO.
 *     CPU reduces the SSBO to the aggregate statistics.
 *
 * Both shaders operate on SSBOs and uniforms; no textures or FBOs needed.
 *
 * Usage
 * ─────
 *   gpu_twoview_init();                // once per process
 *
 *   float X[3*N];
 *   int valid = gpu_triangulate(pts_n, N, R, t, X);
 *
 *   float residuals[4*N];
 *   float wrss;  int ninl;
 *   gpu_ba_residuals(pts_px, X, N, R, t, f, cx, cy, 4.0f,
 *                    residuals, &wrss, &ninl);
 *
 *   gpu_twoview_shutdown();
 *
 * Threading
 * ─────────
 *   NOT thread-safe.  All calls must come from the same thread that called
 *   gpu_twoview_init() (the EGL context owner).  In isat_twoview the
 *   reconstruction stage runs as StageCurrent (main thread).
 */

#pragma once
#ifndef GPU_TWOVIEW_SFM_H
#define GPU_TWOVIEW_SFM_H

#ifdef __cplusplus
extern "C" {
#endif

/* ── Life-cycle ─────────────────────────────────────────────────────────── */

/**
 * Initialise EGL surfaceless context + GLEW + compile both compute shaders.
 * @return 0 on success, negative on failure.
 */
int gpu_twoview_init(void);

/** Release all GPU resources and terminate the EGL context. */
void gpu_twoview_shutdown(void);

/* ── Triangulation ──────────────────────────────────────────────────────── */

/**
 * Triangulate @p n correspondences in parallel on GPU (linear DLT per point).
 *
 * @param pts_n  Normalised image coordinates, row-major float[4n]:
 *               [x1n, y1n, x2n, y2n, x1n, y1n, ...].
 *               x1n = (u1 - cx) / f,  y1n = (v1 - cy) / f
 * @param n      Number of correspondences (≥ 1).
 * @param R      Camera-2 rotation matrix, row-major float[9].
 * @param t      Camera-2 translation vector, float[3].
 * @param X_out  Output 3-D points, float[3n] = [X, Y, Z, X, Y, Z, ...].
 *               Points with negative depth in either camera are written as NaN.
 * @return       Number of valid (finite, positive-depth) points.
 */
int gpu_triangulate(const float* pts_n, int n, const float R[9], const float t[3], float* X_out);

/* ── BA residuals ───────────────────────────────────────────────────────── */

/**
 * Compute Huber-weighted reprojection residuals for @p n 3-D points.
 *
 * Camera model:
 *   cam1: X_c = X_world  (reference camera at origin)
 *   cam2: X_c = R * X_world + t
 *   proj: u = f * X_c[0] / X_c[2] + cx,  v = f * X_c[1] / X_c[2] + cy
 *
 * Huber weighting: w = 1 if |r| ≤ huber_k, else huber_k / |r|.
 *   Each of the 4 residuals per point is independently weighted.
 *
 * @param pts_px  Pixel observations, float[4n] = [u1, v1, u2, v2, ...].
 * @param X       Current 3-D points, float[3n].  NaN points are skipped.
 * @param n       Number of points.
 * @param R       Current cam2 rotation, float[9].
 * @param t       Current cam2 translation, float[3].
 * @param f       Current focal length (pixels).
 * @param cx, cy  Principal point (pixels).
 * @param huber_k Huber threshold (pixels).
 * @param residuals_out  Output Huber-weighted residuals, float[4n].
 *                       May be NULL (stats are still computed).
 * @param wrss_out   Output sum of (Huber residual)²  (divided by n_valid).
 * @param valid_out  Output number of non-NaN, positive-depth points.
 */
void gpu_ba_residuals(const float* pts_px, const float* X, int n, const float R[9],
                      const float t[3], float f, float cx, float cy, float huber_k,
                      float* residuals_out, float* wrss_out, int* valid_out);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* GPU_TWOVIEW_SFM_H */
