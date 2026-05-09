/**
 * @file  cuda_resection.cuh
 * @brief GPU-accelerated PnP pose refinement and inlier classification for
 *        single-image resection in incremental SfM.
 *
 * Design philosophy
 * ─────────────────────────────────────────────────────────────────────────────
 *  The existing CPU pipeline uses PoseLib (CPU RANSAC P3P) to find an initial
 *  pose, then runs a Gauss-Newton (GN) / LM refinement.  For large n (>1000
 *  correspondences), the refinement dominates.
 *
 *  This module provides:
 *
 *  1. GPU GN/LM pose refinement (kernel_pose_refine_lm)
 *       Replaces the CPU pose_refine_gn() for large n:
 *       • Parallel Jacobian assembly across n observations
 *       • Shared-memory reduction to form the 6×6 JᵀJ and 6×1 Jᵀr systems
 *       • Single-thread Cholesky solve of the 6×6 system (tiny, on device)
 *       • Rotation updated via Rodrigues (so3 exponential map)
 *       One LM iteration = one kernel launch.  The outer LM loop runs
 *       from the host (checking cost convergence each iteration).
 *
 *  2. GPU inlier/outlier classification (kernel_p3p_inliers)
 *       After PoseLib P3P gives the best pose candidate, classify all n
 *       correspondences in parallel (one thread each) using the full
 *       Brown-Conrady distortion model.  Returns inlier count and mask.
 *
 * Coordinate conventions
 * ─────────────────────────────────────────────────────────────────────────────
 *  R[9] row-major, t[3] = camera-frame translation (Xc = R·X + t).
 *  pts3d[n*3] world coordinates, pts2d[n*2] distorted pixel observations.
 *  K[9] = {fx,fy,cx,cy,k1,k2,k3,p1,p2}, Bentley convention.
 *
 * Thread safety
 * ─────────────────────────────────────────────────────────────────────────────
 *  Not thread-safe.  Use one GpuResectionContext per thread.
 */

#pragma once
#ifndef INSIGHT_CUDA_RESECTION_CUH
#define INSIGHT_CUDA_RESECTION_CUH

#include <cuda_runtime.h>
#include <stdint.h>

namespace insight {
namespace cuda {

// ─────────────────────────────────────────────────────────────────────────────
// GpuResectionContext — persistent device buffers for one camera at a time
// ─────────────────────────────────────────────────────────────────────────────

struct GpuResectionContext {
    float*   d_pts3d       = nullptr; ///< [max_pts*3]  world 3-D points
    float*   d_pts2d_u     = nullptr; ///< [max_pts]    observed distorted pixel u
    float*   d_pts2d_v     = nullptr; ///< [max_pts]    observed distorted pixel v

    // Current pose (kept on device so the LM loop avoids host round-trips)
    float*   d_R           = nullptr; ///< [9]  rotation matrix (row-major, float)
    float*   d_t           = nullptr; ///< [3]  translation vector

    // Per-observation buffers updated each LM iteration
    float*   d_residuals   = nullptr; ///< [max_pts*2]    (ru, rv) per obs
    float*   d_valid       = nullptr; ///< [max_pts]      1.0 if depth > 0, else 0.0

    // JtJ (6×6 symmetric) and Jtr (6×1) accumulated across all observations.
    // Stored as flat arrays: JtJ has 36 entries, Jtr has 6.
    float*   d_JtJ         = nullptr; ///< [36]  6×6 row-major, reset each iteration
    float*   d_Jtr         = nullptr; ///< [6]   reset each iteration
    float*   d_cost        = nullptr; ///< [1]   sum of squared residuals

    // Inlier classification output
    uint8_t* d_inlier_flags = nullptr; ///< [max_pts]
    float*   d_errors_sq    = nullptr; ///< [max_pts] squared reproj errors

    int max_pts = 0;
};

// ─────────────────────────────────────────────────────────────────────────────
// Lifecycle
// ─────────────────────────────────────────────────────────────────────────────

GpuResectionContext* gpu_resection_ctx_create(int max_pts);
void                 gpu_resection_ctx_free(GpuResectionContext* ctx);

// ─────────────────────────────────────────────────────────────────────────────
// Upload data to device
// ─────────────────────────────────────────────────────────────────────────────

/// Upload 3D-2D correspondences and an initial pose to the device context.
/// @param n_pts    number of correspondences (<= max_pts)
/// @param pts3d   [n*3] world points (float)
/// @param pts2d_u [n]   distorted pixel u (observed)
/// @param pts2d_v [n]   distorted pixel v (observed)
/// @param R_init  [9]   initial rotation (row-major)
/// @param t_init  [3]   initial translation
void gpu_resection_upload(GpuResectionContext* ctx,
                          int n_pts,
                          const float* pts3d,
                          const float* pts2d_u,
                          const float* pts2d_v,
                          const float* R_init,
                          const float* t_init);

// ─────────────────────────────────────────────────────────────────────────────
// LM pose refinement (full Brown-Conrady Jacobian)
// ─────────────────────────────────────────────────────────────────────────────

/// GPU Levenberg-Marquardt pose refinement.
///
/// Runs @p max_iters LM iterations.  Each iteration:
///   1. kernel_gn_residuals  — parallel residuals + cost (all points)
///   2. kernel_gn_jtj        — parallel Jacobian accumulation (shared-memory reduction)
///   3. host solve of 6×6 LM system (tiny, done on CPU to avoid cuSOLVER dependency)
///   4. kernel_gn_update     — apply delta (so3 rotation + t perturbation)
///
/// Convergence: stops early when ||delta||₂ < 1e-8 or lambda > 1e8.
///
/// @param K[9]        intrinsics {fx,fy,cx,cy,k1,k2,k3,p1,p2}
/// @param n_pts       number of correspondences (must match upload)
/// @param max_iters   maximum LM iterations (default 10)
/// @param R_out[9]    output refined rotation (host, row-major)
/// @param t_out[3]    output refined translation (host)
/// @param rmse_px_out output RMSE in pixels (can be nullptr)
/// @return true on success
bool gpu_resection_refine(GpuResectionContext* ctx,
                          const float* K,
                          int n_pts,
                          int max_iters,
                          float* R_out,
                          float* t_out,
                          float* rmse_px_out = nullptr);

// ─────────────────────────────────────────────────────────────────────────────
// Inlier classification (post-RANSAC)
// ─────────────────────────────────────────────────────────────────────────────

/// Classify observations as inlier/outlier for a given pose.
/// Uses the full Brown-Conrady distortion model.
///
/// @param K[9]           intrinsics
/// @param n_pts          number of correspondences
/// @param threshold_px   inlier threshold in pixels
/// @param R[9]           pose rotation (host, row-major)
/// @param t[3]           pose translation (host)
/// @param out_flags[n]   host output: 1=inlier, 0=outlier
/// @return               inlier count
int gpu_resection_inliers(GpuResectionContext* ctx,
                          const float* K,
                          int n_pts,
                          float threshold_px,
                          const float* R,
                          const float* t,
                          uint8_t* out_flags);

}  // namespace cuda
}  // namespace insight

#endif  // INSIGHT_CUDA_RESECTION_CUH
