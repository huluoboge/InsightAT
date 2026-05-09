/**
 * @file  cuda_reproj.cuh
 * @brief GPU-accelerated reprojection error computation and outlier flagging.
 *
 * Purpose
 * ─────────────────────────────────────────────────────────────────────────────
 *  After triangulation (or bundle adjustment), every observation must be
 *  evaluated for outlier status.  This is an embarrassingly parallel operation:
 *  each observation is independent, making it ideal for GPU acceleration.
 *
 *  Two use cases:
 *
 *  1. Post-triangulation outlier filter
 *       For each triangulated track, compute reprojection error at every
 *       observation.  Mark observations above threshold as outliers.
 *       Called from run_batch_triangulation / run_retriangulation.
 *
 *  2. Post-resection inlier classification
 *       Given a newly registered camera pose, compute reprojection errors for
 *       all observations belonging to that image.  Used to filter bad matches
 *       before writing the pose to the scene (prune_resection_observations).
 *
 * API
 * ─────────────────────────────────────────────────────────────────────────────
 *  gpu_reproj_ctx_create / gpu_reproj_ctx_free  — lifecycle
 *  gpu_reproj_compute  — compute per-observation squared errors (device output)
 *  gpu_reproj_filter   — classify inliers / outliers and optionally read back
 */

#pragma once
#ifndef INSIGHT_CUDA_REPROJ_CUH
#define INSIGHT_CUDA_REPROJ_CUH

#include <cuda_runtime.h>
#include <stdint.h>

namespace insight {
namespace cuda {

// ─────────────────────────────────────────────────────────────────────────────
// GpuReprojContext  — persistent buffers for reprojection error computation
// ─────────────────────────────────────────────────────────────────────────────

struct GpuReprojContext {
    // Per-observation input arrays (device)
    float*   d_track_xyz    = nullptr; ///< [max_tracks*3]  3D point XYZ
    int*     d_obs_track_id = nullptr; ///< [max_obs]       which track this obs belongs to
    float*   d_obs_u        = nullptr; ///< [max_obs]       distorted pixel u (observed)
    float*   d_obs_v        = nullptr; ///< [max_obs]       distorted pixel v (observed)
    float*   d_obs_R        = nullptr; ///< [max_obs*9]     rotation matrix (row-major)
    float*   d_obs_C        = nullptr; ///< [max_obs*3]     camera centre (world)
    float*   d_obs_K        = nullptr; ///< [max_obs*9]     intrinsics {fx,fy,cx,cy,k1,k2,k3,p1,p2}

    // Output arrays (device)
    float*   d_errors_sq    = nullptr; ///< [max_obs]       squared reprojection errors (pixels²)
    uint8_t* d_inlier_flags = nullptr; ///< [max_obs]       1=inlier, 0=outlier (after filter)

    // Capacity
    int max_obs    = 0;
    int max_tracks = 0;
};

// ─────────────────────────────────────────────────────────────────────────────
// Lifecycle
// ─────────────────────────────────────────────────────────────────────────────

GpuReprojContext* gpu_reproj_ctx_create(int max_obs, int max_tracks);
void              gpu_reproj_ctx_free(GpuReprojContext* ctx);

// ─────────────────────────────────────────────────────────────────────────────
// Batch reprojection errors
// ─────────────────────────────────────────────────────────────────────────────

/// Upload data and compute per-observation squared reprojection errors (px²).
///
/// On return, ctx->d_errors_sq[i] holds the squared reprojection error for
/// observation i.  Results remain on device; call gpu_reproj_filter or
/// copy d_errors_sq manually.
///
/// @param n_obs        total observations
/// @param n_tracks     total tracks (XYZ array size)
/// @param track_xyz    [n_tracks*3] 3D point coordinates (world, float)
/// @param obs_track_id [n_obs] index into track_xyz for each observation
/// @param obs_u/v      [n_obs] distorted pixel observations
/// @param obs_R/C/K    [n_obs*9/3/9] per-observation camera data
void gpu_reproj_compute(GpuReprojContext* ctx,
                        int n_obs, int n_tracks,
                        const float* track_xyz,
                        const int*   obs_track_id,
                        const float* obs_u,   const float* obs_v,
                        const float* obs_R,   const float* obs_C,
                        const float* obs_K);

// ─────────────────────────────────────────────────────────────────────────────
// Outlier classification
// ─────────────────────────────────────────────────────────────────────────────

/// Classify observations as inlier/outlier based on a pixel threshold.
/// Reads ctx->d_errors_sq (must call gpu_reproj_compute first), writes
/// ctx->d_inlier_flags.  Optionally reads back flags to host.
///
/// @param threshold_px   inlier threshold in pixels
/// @param n_obs          number of observations
/// @param out_flags_host [n_obs] host buffer to receive flags; pass nullptr to skip readback
/// @return               number of inliers (only if out_flags_host != nullptr, else 0)
int gpu_reproj_filter(GpuReprojContext* ctx,
                      float threshold_px,
                      int n_obs,
                      uint8_t* out_flags_host = nullptr);

// ─────────────────────────────────────────────────────────────────────────────
// Single-image reprojection filter (for prune_resection_observations)
// ─────────────────────────────────────────────────────────────────────────────

/// Compute reprojection errors for all observations of a single registered image
/// and write per-observation inlier flags to out_flags_host.
///
/// This is a convenience wrapper for the two-step compute+filter API when
/// operating on a single camera pose (e.g. after PoseLib resection).
///
/// @param R[9]           rotation matrix (row-major, float)
/// @param C[3]           camera centre (world, float)
/// @param K[9]           intrinsics {fx,fy,cx,cy,k1,k2,k3,p1,p2}
/// @param pts3d          [n_pts*3] 3D points (world)
/// @param pts2d_u/v      [n_pts]   distorted pixel observations
/// @param n_pts          number of point-observation pairs
/// @param threshold_px   inlier threshold in pixels
/// @param out_flags_host [n_pts] host buffer; 1=inlier, 0=outlier
void gpu_reproj_single_image(GpuReprojContext* ctx,
                             const float* R, const float* C, const float* K,
                             const float* pts3d,
                             const float* pts2d_u, const float* pts2d_v,
                             int n_pts,
                             float threshold_px,
                             uint8_t* out_flags_host);

}  // namespace cuda
}  // namespace insight

#endif  // INSIGHT_CUDA_REPROJ_CUH
