/**
 * @file  cuda_pnp_ransac.cuh
 * @brief CUDA batch PnP RANSAC for incremental SfM resection.
 *
 * Design principles
 * ─────────────────────────────────────────────────────────────────────────────
 *  • GPU-to-GPU data flow: all observation and track arrays live on device.
 *    The caller provides pointers into CudaSfMState or equivalent device
 *    arrays; no host round-trips during the RANSAC loop.
 *
 *  • Undistorted coordinates: the kernel operates entirely on
 *    pre-undistorted normalised image coordinates (xn, yn) — equivalent to
 *    the bearing directions (xn, yn, 1). The caller is responsible for
 *    providing up-to-date undistorted coordinates (e.g. CudaSfMState
 *    d_obs_xn / d_obs_yn maintained by gpu_sfm_state_undistort_all whenever intrinsics change).
 *
 *  • Batch processing: K images are resectioned simultaneously.
 *    Grid layout: grid = (N_iter, K),  block = (BLOCK_N, 1).
 *    Each (hyp, image) block generates a P3P hypothesis, evaluates all
 *    observations of that image in parallel, and records the best of the
 *    4 sub-hypotheses in a results buffer.
 *
 *  • Fixed-iteration RANSAC (no adaptive branching):
 *    - All N_iter hypotheses always execute — no warp-divergent early exit.
 *    - On typical aerial data (≤80% outliers) 2000 iterations gives >99%
 *      confidence for a 3-point minimal sample.
 *    - N_iter is a runtime parameter so callers can tune without recompile.
 *
 *  • Stateless in-kernel pseudo-random sampling:
 *    LCG hash of (hyp_id, sample_slot, per_image_seed) — no cuRAND state,
 *    no extra register pressure, no host-side sample generation.
 *
 *  • Pose refinement is handled separately by gpu_resection_refine()
 *    (cuda_resection.cuh), which accepts undistorted normalised obs and
 *    performs pinhole-only Gauss-Newton / LM.
 *
 * Coordinate conventions
 * ─────────────────────────────────────────────────────────────────────────────
 *  Rotation R[9] row-major float, translation t[3] camera-frame (Xc = R·X + t).
 *  Bearing  b[3] = normalize(xn, yn, 1) — unit vectors.
 *  All 3D points in world frame (float[3]).
 *
 * Thread safety
 * ─────────────────────────────────────────────────────────────────────────────
 *  GpuPnpBatchContext is NOT thread-safe. Create one per pipeline.
 */

#pragma once
#ifndef INSIGHT_CUDA_PNP_RANSAC_CUH
#define INSIGHT_CUDA_PNP_RANSAC_CUH

#include <cuda_runtime.h>
#include <stdint.h>
#include <vector>

namespace insight {
namespace cuda {

// ─────────────────────────────────────────────────────────────────────────────
// GpuPnpBatchContext — persistent device scratch for batch resection
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Pre-allocated device buffers reused across all batch resection calls.
 * Sized at creation for max_images × max_iter × per-result storage.
 */
struct GpuPnpBatchContext {
    // Per-(image, hypothesis) results written by kernel_pnp_ransac_batch.
    // Layout: [K × N_iter]  stride = N_iter
    float*   d_hyp_R         = nullptr; ///< [K × N_iter × 9]   best R from this hyp
    float*   d_hyp_t         = nullptr; ///< [K × N_iter × 3]   best t
    int*     d_hyp_inliers   = nullptr; ///< [K × N_iter]        inlier count

    // Per-image best result (after host-side argmax).
    float*   d_best_R        = nullptr; ///< [K × 9]
    float*   d_best_t        = nullptr; ///< [K × 3]
    int*     d_best_inliers  = nullptr; ///< [K]

    // Per-obs inlier flag — reused for classify after RANSAC selects best pose.
    uint8_t* d_inlier_flags  = nullptr; ///< [max_obs_total]

    /// Staging for packed per-image correspondences (xn, yn, XYZ); total length max_obs_total.
    float* d_pack_xn  = nullptr;
    float* d_pack_yn  = nullptr;
    float* d_pack_xyz = nullptr;

    int max_images    = 0;
    int max_iter      = 0;
    int max_obs_total = 0; ///< upper bound on sum of packed obs in one batch (use n_obs of scene)
};

// ─────────────────────────────────────────────────────────────────────────────
// Lifecycle
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Allocate device scratch for up to max_images simultaneous resections,
 * max_iter RANSAC iterations per image, and max_obs_total observations
 * across all images in one batch.
 */
GpuPnpBatchContext* gpu_pnp_batch_ctx_create(int max_images, int max_iter, int max_obs_total);
void                gpu_pnp_batch_ctx_free(GpuPnpBatchContext* ctx);

// ─────────────────────────────────────────────────────────────────────────────
// GpuPnpImageDesc — per-image descriptor (host-side, uploaded before kernel)
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Describes one image's 3D–2D correspondences.
 * All device pointer fields point into a single flat allocation managed by
 * the caller (e.g. slices of CudaSfMState arrays).
 *
 * obs_xn[i], obs_yn[i]  — undistorted normalised image coords (already on device).
 * pts3d[i*3..i*3+2]     — world 3-D point (already on device, from d_track_xyz).
 * n_obs                  — number of valid 3D–2D correspondences for this image.
 * seed                   — per-image salt for in-kernel LCG sampling.
 */
struct GpuPnpImageDesc {
    const float*   d_obs_xn = nullptr; ///< [n_obs]    undistorted xn
    const float*   d_obs_yn = nullptr; ///< [n_obs]    undistorted yn
    const float*   d_pts3d  = nullptr; ///< [n_obs×3]  world 3-D points (X,Y,Z)
    int            n_obs    = 0;
    uint32_t       seed     = 0;
};

// ─────────────────────────────────────────────────────────────────────────────
// Batch RANSAC P3P
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Run RANSAC with Lambda-twist P3P for K images simultaneously.
 *
 * For each image k:
 *   - Runs n_iter hypotheses. Each hypothesis draws 3 random correspondences
 *     (via stateless LCG hash of hyp_id + image seed), solves P3P (up to 4
 *     solutions), evaluates all n_obs correspondences in parallel, keeps the
 *     best sub-solution.
 *   - Returns best R[9], t[3], and inlier count for each image.
 *
 * Reprojection error metric: squared pinhole error in normalised image plane
 *   ||(xn_proj - xn_obs, yn_proj - yn_obs)||²  < threshold_n²
 * where threshold_n = threshold_px / focal_px  (caller converts).
 *
 * @param ctx             Pre-allocated device scratch.
 * @param descs           Host array of K image descriptors. Device pointers
 *                        inside each desc must already be valid.
 * @param K               Number of images (must be ≤ ctx->max_images).
 * @param n_iter          RANSAC iterations (must be ≤ ctx->max_iter).
 * @param threshold_n_sq  Inlier threshold in squared NORMALISED coords.
 *                        Typically (threshold_px / fx)².
 * @param out_R           Host output: K×9 rotation matrices (row-major).
 * @param out_t           Host output: K×3 translation vectors.
 * @param out_inliers     Host output: K inlier counts.
 * @return                Number of images with inlier count > 0.
 */
int gpu_pnp_ransac_batch(GpuPnpBatchContext*       ctx,
                         const GpuPnpImageDesc*    descs,
                         int                       K,
                         int                       n_iter,
                         float                     threshold_n_sq,
                         float*                    out_R,       // [K×9]
                         float*                    out_t,       // [K×3]
                         int*                      out_inliers); // [K]

// ─────────────────────────────────────────────────────────────────────────────
// Inlier classification (post-RANSAC, with best pose)
// ─────────────────────────────────────────────────────────────────────────────

/**
 * For one image, classify all n_obs correspondences as inlier/outlier using
 * the pinhole model with the given R, t.
 *
 * Writes d_flags[i] = 1 if inlier, 0 otherwise.
 * Returns inlier count.
 *
 * @param d_obs_xn      [n_obs]   undistorted normalised xn (device)
 * @param d_obs_yn      [n_obs]   undistorted normalised yn (device)
 * @param d_pts3d       [n_obs×3] world points (device)
 * @param n_obs         number of correspondences
 * @param R             [9] rotation (host, row-major)
 * @param t             [3] translation (host)
 * @param threshold_n_sq  squared normalised threshold
 * @param d_flags       [n_obs]   output flags (device, caller allocates)
 */
int gpu_pnp_classify_inliers(const float*   d_obs_xn,
                              const float*   d_obs_yn,
                              const float*   d_pts3d,
                              int            n_obs,
                              const float    R[9],
                              const float    t[3],
                              float          threshold_n_sq,
                              uint8_t*       d_flags);

} // namespace cuda
} // namespace insight

#endif // INSIGHT_CUDA_PNP_RANSAC_CUH
