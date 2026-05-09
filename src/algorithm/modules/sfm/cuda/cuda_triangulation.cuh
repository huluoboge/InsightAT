/**
 * @file  cuda_triangulation.cuh
 * @brief GPU-accelerated batch DLT triangulation for incremental SfM.
 *
 * Architecture
 * ─────────────────────────────────────────────────────────────────────────────
 *  Mirrors the CPU robust_triangulate_point_multiview() logic but parallelises
 *  at two levels:
 *
 *  1. Track level  — up to max_pairs_per_track view-pair hypotheses per track
 *                    are evaluated simultaneously across all tracks in the batch.
 *  2. Pair level   — one CUDA thread per (track, pair) hypothesis.
 *
 *  CPU side (before launch):
 *    • Build flat PairTask arrays from track CSR (up to max_pairs_per_track pairs
 *      per track, enumerated in lexicographic order to match CPU determinism).
 *
 *  GPU kernels:
 *    kernel_tri_pairs   — one thread per PairTask.
 *                         DLT 2-view → depth check → angle check →
 *                         count inliers (all N obs of that track) →
 *                         atomicMax best into per-track slot.
 *    kernel_tri_collect — one thread per track.
 *                         Read best slot → copy winner XYZ to output.
 *
 * Data layout
 * ─────────────────────────────────────────────────────────────────────────────
 *  All observation arrays are flat (SoA) over N total observations of the batch.
 *  Track-to-obs mapping via CSR (csr_ptr[t] .. csr_ptr[t+1]).
 *  Intrinsics packed as float[9] = {fx,fy,cx,cy,k1,k2,k3,p1,p2}.
 *  The best-slot encoding uses a uint64:
 *      high 32 bits = inlier count  (maximized via atomicMax)
 *      low  32 bits = pair task index (stored together with the count)
 *
 * Thread safety
 * ─────────────────────────────────────────────────────────────────────────────
 *  Not thread-safe.  One GpuTriContext should be used serially per scene.
 *  Multiple contexts may coexist in process memory (one per CUDA stream).
 */

#pragma once
#ifndef INSIGHT_CUDA_TRIANGULATION_CUH
#define INSIGHT_CUDA_TRIANGULATION_CUH

#include <cuda_runtime.h>
#include <stdint.h>

namespace insight {
namespace cuda {

// ─────────────────────────────────────────────────────────────────────────────
// Options (mirrors RobustTriangulationOptions defaults)
// ─────────────────────────────────────────────────────────────────────────────

struct GpuTriOptions {
    float ransac_inlier_px    = 16.0f;  ///< inlier reprojection threshold (pixels)
    float min_tri_angle_deg   =  0.5f;  ///< minimum triangulation angle between seed pair
    float max_tri_angle_deg   = 170.0f; ///< maximum triangulation angle (reject nearly-parallel)
    int   max_pairs_per_track = 500;    ///< cap on view-pair hypotheses per track
    int   min_inlier_views    = 2;      ///< minimum inlier view count to accept a track
};

// ─────────────────────────────────────────────────────────────────────────────
// GpuTriContext — persistent GPU allocations for a full SfM run
// ─────────────────────────────────────────────────────────────────────────────

struct GpuTriContext {
    // Per-observation arrays (device, SoA)
    float*   d_obs_xn   = nullptr; ///< [max_obs]    undistorted normalised x
    float*   d_obs_yn   = nullptr; ///< [max_obs]    undistorted normalised y
    float*   d_obs_u    = nullptr; ///< [max_obs]    original distorted pixel u
    float*   d_obs_v    = nullptr; ///< [max_obs]    original distorted pixel v
    float*   d_obs_R    = nullptr; ///< [max_obs*9]  rotation matrices (row-major)
    float*   d_obs_C    = nullptr; ///< [max_obs*3]  camera centres (world)
    float*   d_obs_K    = nullptr; ///< [max_obs*9]  intrinsics {fx,fy,cx,cy,k1,k2,k3,p1,p2}

    // Track CSR (device)
    int*     d_csr_ptr  = nullptr; ///< [max_tracks+1]  CSR row-pointer

    // Pair-task arrays (device) — rebuilt each batch
    int*     d_task_track = nullptr; ///< [max_pairs]  which track
    int*     d_task_a     = nullptr; ///< [max_pairs]  absolute obs index a
    int*     d_task_b     = nullptr; ///< [max_pairs]  absolute obs index b

    // Best-hypothesis slots (device)
    // Encoding: (inlier_count << 32) | pair_task_index.
    // Initialised to 0 each batch; any slot with high32 >= min_inlier_views is valid.
    unsigned long long* d_best_slot = nullptr; ///< [max_tracks]

    // Per-pair XYZ scratch (device) — sized to actual n_pairs at run time
    float*   d_pair_xyz  = nullptr; ///< [current_n_pairs*3], allocated inside gpu_tri_run
    int      pair_xyz_capacity = 0; ///< current allocation size

    // Output arrays (device)
    float*   d_out_xyz   = nullptr; ///< [max_tracks*3]
    uint8_t* d_out_valid = nullptr; ///< [max_tracks]

    // Allocation limits
    int max_obs    = 0;
    int max_tracks = 0;
    int max_pairs  = 0;
};

// ─────────────────────────────────────────────────────────────────────────────
// Lifecycle
// ─────────────────────────────────────────────────────────────────────────────

/// Allocate a GPU triangulation context.
/// @param max_obs     Maximum total observations across all tracks in a single batch
/// @param max_tracks  Maximum number of tracks in a single batch
/// @param max_pairs   Maximum total pair tasks in a single batch
GpuTriContext* gpu_tri_ctx_create(int max_obs, int max_tracks, int max_pairs);

/// Free all device memory and delete the context object.
void gpu_tri_ctx_free(GpuTriContext* ctx);

// ─────────────────────────────────────────────────────────────────────────────
// Upload observation data (host → device)
// ─────────────────────────────────────────────────────────────────────────────

/// Upload all per-observation arrays and the CSR pointer for a batch of tracks.
/// Must be called before gpu_tri_run.
///
/// @param n_tracks   Number of tracks in this batch
/// @param n_obs      Total observations across all tracks (sum of CSR interval sizes)
/// @param csr_ptr    [n_tracks+1] host CSR pointers (0-based into obs arrays)
/// @param obs_xn/yn  [n_obs] undistorted normalised coords
/// @param obs_u/v    [n_obs] original distorted pixel coords
/// @param obs_R      [n_obs*9] rotation matrices (row-major, float)
/// @param obs_C      [n_obs*3] camera centres (world, float)
/// @param obs_K      [n_obs*9] intrinsics {fx,fy,cx,cy,k1,k2,k3,p1,p2}
void gpu_tri_upload(GpuTriContext* ctx,
                    int n_tracks, int n_obs,
                    const int*   csr_ptr,
                    const float* obs_xn,  const float* obs_yn,
                    const float* obs_u,   const float* obs_v,
                    const float* obs_R,   const float* obs_C,
                    const float* obs_K);

// ─────────────────────────────────────────────────────────────────────────────
// Run batch triangulation
// ─────────────────────────────────────────────────────────────────────────────

/// Enumerate pair tasks, launch triangulation kernels, read back results.
///
/// Must be called after gpu_tri_upload for the same batch.
///
/// @param n_tracks   Number of tracks (matches the upload call)
/// @param opt        Triangulation options
/// @param out_xyz    [n_tracks*3] host buffer for output 3D points (pre-allocated)
/// @param out_valid  [n_tracks]   host buffer for validity flags (pre-allocated)
/// @return           Number of successfully triangulated tracks
int gpu_tri_run(GpuTriContext* ctx,
                int n_tracks,
                const GpuTriOptions& opt,
                float*   out_xyz,
                uint8_t* out_valid);

}  // namespace cuda
}  // namespace insight

#endif  // INSIGHT_CUDA_TRIANGULATION_CUH
