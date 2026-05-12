/**
 * @file  cuda_sfm_state.h
 * @brief Persistent GPU state for the full incremental SfM CUDA pipeline.
 *
 * CudaSfMState holds device-side copies of all data that must survive across
 * SfM iterations: poses, track XYZ, observation pixel coords, undistorted
 * normalised coords, intrinsics, and validity flags.
 *
 * Observation normalised coords (d_obs_xn/yn) are **derived** from d_obs_u/v and
 * d_intrinsics via gpu_sfm_state_undistort_all().  Call after the initial upload,
 * and **whenever intrinsics used for geometry are updated** (e.g. after BA that
 * refines distortion): recompute on GPU so xn/yn match the current K.  Do not
 * assume a single undistort at track upload is sufficient for the whole run.
 *
 * Conventions:
 *   R[9]    row-major 3×3 rotation (world → camera)
 *   C[3]    camera centre in world coords  (t_cam = -R·C)
 *   K[9]    {fx,fy,cx,cy,k1,k2,k3,p1,p2}  Brown-Conrady (Bentley convention)
 *   xn,yn   undistorted normalised image coords
 */

#pragma once
#ifndef INSIGHT_CUDA_SFM_STATE_H
#define INSIGHT_CUDA_SFM_STATE_H

#include <stdint.h>

namespace insight {
namespace cuda {

// ─────────────────────────────────────────────────────────────────────────────
// CudaSfMState — persistent GPU buffers for one SfM scene
// ─────────────────────────────────────────────────────────────────────────────

struct CudaSfMState {
    // ── Pose arrays (updated per-image on registration and after BA) ──────
    float*   d_poses_R    = nullptr; ///< [n_images * 9]   row-major rotation
    float*   d_poses_C    = nullptr; ///< [n_images * 3]   camera centre (world)
    uint8_t* d_registered = nullptr; ///< [n_images]       1 = registered

    // ── Track arrays (updated after triangulation and after BA) ──────────
    float*   d_track_xyz   = nullptr; ///< [n_tracks * 3]  X,Y,Z (only valid when d_track_valid=1)
    uint8_t* d_track_valid = nullptr; ///< [n_tracks]      1 = alive AND has triangulated XYZ

    // ── Observation arrays ────────────────────────────────────────────────
    float*   d_obs_u         = nullptr; ///< [n_obs]  distorted pixel u — permanent, never changes
    float*   d_obs_v         = nullptr; ///< [n_obs]  distorted pixel v — permanent, never changes
    float*   d_obs_xn        = nullptr; ///< [n_obs]  undistorted normalised x — recomputed after intrinsics change
    float*   d_obs_yn        = nullptr; ///< [n_obs]  undistorted normalised y — recomputed after intrinsics change
    int*     d_obs_image_idx = nullptr; ///< [n_obs]  index into d_poses_R/C
    int*     d_obs_track_idx = nullptr; ///< [n_obs]  index into d_track_xyz/valid
    int*     d_obs_cam_idx   = nullptr; ///< [n_obs]  index into d_intrinsics (camera model)
    uint8_t* d_obs_valid     = nullptr; ///< [n_obs]  1 = observation alive (deleted = 0)

    // ── Per-image observation CSR (read-only after upload; static TrackStore) ─
    int* d_img_obs_ptr = nullptr; ///< [n_images + 1] CSR row pointers
    int* d_img_obs_idx = nullptr; ///< [n_obs] global obs id per image row

    // ── Camera intrinsics: {fx,fy,cx,cy,k1,k2,k3,p1,p2} ─────────────────
    float*   d_intrinsics    = nullptr; ///< [n_cameras * 9]

    // ── Outlier kernels scratch (sizes fixed at gpu_sfm_state_create from n_obs / n_tracks) ─
    uint8_t* d_reproj_reject_mark = nullptr; ///< [n_obs]  kernel_reject_reproj: 0 / 1 restorable / 2 permanent
    int*     d_angle_csr_ptr      = nullptr; ///< [n_tracks + 1]  per-track obs range (angle pass)
    int*     d_angle_csr_obs      = nullptr; ///< [n_obs]       compacted global obs ids (nnz ≤ n_obs)
    uint8_t* d_angle_per_obs_flag = nullptr; ///< [n_obs]       kernel_reject_angle marks obs to delete

    // ── BA subset selection flags ─────────────────────────────────────────
    uint8_t* d_track_flags_ba    = nullptr; ///< [n_tracks]  1 = skip this track from Ceres BA (set by select_ba_subset)
    uint8_t* d_track_needs_retri = nullptr; ///< [n_tracks]  1 = track needs retriangulation (set by outlier/BA)

    // ── Capacities ────────────────────────────────────────────────────────
    int n_images  = 0;
    int n_tracks  = 0;
    int n_obs     = 0;
    int n_cameras = 0;
};

// ─────────────────────────────────────────────────────────────────────────────
// Lifecycle
// ─────────────────────────────────────────────────────────────────────────────

/// Allocate a CudaSfMState and zeroise all device buffers.
CudaSfMState* gpu_sfm_state_create(int n_images, int n_tracks, int n_obs, int n_cameras);

/// Free all device memory and delete the state object.
void gpu_sfm_state_free(CudaSfMState* state);

// ─────────────────────────────────────────────────────────────────────────────
// Upload (CPU → device)
// ─────────────────────────────────────────────────────────────────────────────

/// Upload intrinsics for all cameras.
/// @param K_flat  host array of size n_cameras * 9; layout: K_flat[c*9 .. c*9+8]
void gpu_sfm_upload_intrinsics(CudaSfMState* state,
                               const float* K_flat, int n_cameras);

/// Upload all observation pixel coords, index mappings, and initial valid flags.
/// Called once at startup before the first undistort pass.
void gpu_sfm_upload_observations(CudaSfMState* state,
                                 const float*   obs_u,
                                 const float*   obs_v,
                                 const int*     obs_image_idx,
                                 const int*     obs_track_idx,
                                 const int*     obs_cam_idx,
                                 const uint8_t* obs_valid,
                                 int n_obs);

/// Upload per-image observation CSR (row = image, entries = global obs indices).
/// h_img_obs_ptr has length n_images + 1; h_img_obs_idx has length n_obs (same nnz as flat obs).
void gpu_sfm_upload_img_obs_csr(CudaSfMState* state,
                                const int* h_img_obs_ptr,
                                const int* h_img_obs_idx,
                                int n_images,
                                int n_idx_values);

/// Upload (or refresh) all poses and registered flags.
/// @param poses_R    host [n_images * 9]
/// @param poses_C    host [n_images * 3]
/// @param registered host [n_images]
void gpu_sfm_upload_poses(CudaSfMState* state,
                          const float*   poses_R,
                          const float*   poses_C,
                          const uint8_t* registered,
                          int n_images);

/// Upload (or refresh) track XYZ and valid flags.
void gpu_sfm_upload_tracks(CudaSfMState* state,
                           const float*   track_xyz,
                           const uint8_t* track_valid,
                           int n_tracks);

/// Upload a single image's pose after resection.  O(1) cudaMemcpy.
void gpu_sfm_upload_one_pose(CudaSfMState* state,
                              int image_idx,
                              const float* R9,
                              const float* C3);

/// Mark one image as registered (or unregistered) on device.
void gpu_sfm_set_registered(CudaSfMState* state, int image_idx, uint8_t value);

/// Scatter-update track XYZ for a batch of track ids.
/// @param tid_list  host [n]  track indices
/// @param xyz_data  host [n*3]  X,Y,Z per track
/// Internally allocates and frees temporary device buffers.
void gpu_sfm_update_tracks_batch(CudaSfMState* state,
                                 const int*   tid_list,
                                 const float* xyz_data,
                                 int n);

/// Same as gpu_sfm_update_tracks_batch but uses caller-provided pre-allocated
/// device buffers (d_tid_scratch ≥ n ints, d_xyz_scratch ≥ n*3 floats).
/// Avoids cudaMalloc/cudaFree overhead when called in a hot loop.
void gpu_sfm_update_tracks_batch_prealloc(CudaSfMState* state,
                                          const int*   h_tid_list,
                                          const float* h_xyz_data,
                                          int          n,
                                          int*         d_tid_scratch,
                                          float*       d_xyz_scratch);

/// Set d_track_valid[track_idx] = value.
void gpu_sfm_set_track_valid(CudaSfMState* state, int track_idx, uint8_t value);

/// Set d_obs_valid[obs_idx] = value.
void gpu_sfm_set_obs_valid(CudaSfMState* state, int obs_idx, uint8_t value);

/// Set d_track_needs_retri[tid] = 1 for all track ids in the batch.
/// Safe to call with n == 0.
void gpu_sfm_mark_tracks_needs_retri(CudaSfMState* state, const int* tid_list, int n);

/// Set d_track_needs_retri[t] = 0 for all tracks.
void gpu_sfm_clear_tracks_needs_retri(CudaSfMState* state);

/// Batch-update d_obs_valid[obs_ids[i]] = 0 for all i in [0, n).
/// Used after GPU outlier rejection downloads the deleted-flag array.
void gpu_sfm_invalidate_obs_batch(CudaSfMState* state,
                                  const int* obs_ids, int n);

// ─────────────────────────────────────────────────────────────────────────────
// Download (device → CPU)
// ─────────────────────────────────────────────────────────────────────────────

/// Download poses for all images (call before feeding data to Ceres BA).
void gpu_sfm_download_poses(const CudaSfMState* state,
                             float* poses_R,   ///< host [n_images * 9]
                             float* poses_C,   ///< host [n_images * 3]
                             int n_images);

/// Download track XYZ for all tracks.
void gpu_sfm_download_tracks(const CudaSfMState* state,
                              float* track_xyz, ///< host [n_tracks * 3]
                              int n_tracks);

/// Download undistorted normalised coords for all observations (to use as CPU cache).
void gpu_sfm_download_obs_norm(const CudaSfMState* state,
                                float* obs_xn, ///< host [n_obs]
                                float* obs_yn, ///< host [n_obs]
                                int n_obs);

// ─────────────────────────────────────────────────────────────────────────────
// Undistort pass
// ─────────────────────────────────────────────────────────────────────────────

/// Recompute d_obs_xn/yn from d_obs_u/v using d_intrinsics[d_obs_cam_idx[i]].
/// Must be called:
///   1. Once at startup (after obs upload and intrinsics upload).
///   2. After any BA pass that changes camera intrinsics.
/// Cost: one GPU kernel over n_obs threads with 8 fixed-point iterations each.
/// For 10M observations at 500MHz SM throughput: ≈ 1-2 ms.
void gpu_sfm_state_undistort_all(CudaSfMState* state);

// ─────────────────────────────────────────────────────────────────────────────
// Candidate selection helpers (M5 GPU pre-filter)
// ─────────────────────────────────────────────────────────────────────────────

/// Count the number of valid triangulated observations per image (3D-2D matches).
/// @param state            Current GPU state.
/// @param n_tri_per_image  Output host array of size n_images, filled with counts.
///                         Caller must ensure it has size >= n_images.
/// Complexity: one GPU kernel over n_obs threads.
void gpu_sfm_count_tri_per_image(const CudaSfMState* state, int* n_tri_per_image);

} // namespace cuda
} // namespace insight

#endif // INSIGHT_CUDA_SFM_STATE_H
