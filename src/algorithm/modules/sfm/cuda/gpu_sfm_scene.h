/**
 * @file  gpu_sfm_scene.h
 * @brief C++ RAII wrapper around `CudaSfMState` — the GPU-side SfM working set.
 *
 * CUDA-first incremental SfM keeps a dense device mirror of poses, tracks, and
 * observations for kernels (resection, triangulation, undistort, outlier passes).
 * `TrackStore` remains the host source of truth for Ceres BA, retriangulation, and
 * persisted graph edits; push/pull at explicit sync boundaries.
 *
 * Prefer `GpuSfMScene` over bare `CudaSfMState*` in pipeline code so ownership and
 * intent stay obvious; existing C APIs still take `CudaSfMState*` via `cuda_state()`.
 *
 * Host-side methods below mirror *operations* you would conceptually apply to scene
 * data (tracks, obs validity, poses, intrinsics). Kernels such as
 * `gpu_reject_reproj_multiview` read/write the same `CudaSfMState` buffers; call the
 * wrappers here so call sites stay scene-centric.
 */

#pragma once

#include "../../camera/camera_types.h"
#include "../track_store.h"
#include "cuda_ba_subset.cuh"
#include "cuda_sfm_state.h"

#include <Eigen/Core>
#include <cstdint>
#include <memory>
#include <vector>

namespace insight {
namespace cuda {

// ─────────────────────────────────────────────────────────────────────────────
// LocalBAWorkset — compact host-side bundle-adjustment input/output packet
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Compact subset of scene data passed to Ceres BA (and written back after BA).
 *
 * All indices are *local* within this workset (0 .. n_local_images-1, etc.).
 * `image_ids` and `track_ids` map local indices back to global CudaSfMState indices.
 *
 * Filled by `GpuSfMScene::pack_for_ba()`; updated in-place by Ceres then written
 * back to device via `GpuSfMScene::upload_ba_result()`.
 */
struct LocalBAWorkset {
    std::vector<int>    image_ids;        ///< [n_local_images]  global image idx
    std::vector<double> poses_R;          ///< [n_local_images * 9] row-major R (double for Ceres)
    std::vector<double> poses_C;          ///< [n_local_images * 3]
    std::vector<uint8_t> fixed_pose;      ///< [n_local_images]  1 = anchor (fixed in BA)

    std::vector<int>    track_ids;        ///< [n_local_tracks]  global track idx
    std::vector<double> track_xyz;        ///< [n_local_tracks * 3]

    std::vector<int>    obs_image_local;  ///< [n_local_obs]  index into image_ids
    std::vector<int>    obs_track_local;  ///< [n_local_obs]  index into track_ids
    std::vector<double> obs_u;            ///< [n_local_obs]  distorted pixel u
    std::vector<double> obs_v;            ///< [n_local_obs]  distorted pixel v
    std::vector<int>    obs_cam_idx;      ///< [n_local_obs]  camera model index (global)

    int n_local_images = 0;
    int n_local_tracks = 0;
    int n_local_obs    = 0;
};

/// GPU resident scene buffers for incremental SfM (parallel role to `TrackStore` for geometry).
class GpuSfMScene {
public:
    /**
     * Allocate device state and upload a full snapshot from `TrackStore` + poses + intrinsics.
     * Runs the first `gpu_sfm_state_undistort_all` pass.
     * @return nullptr on failure (logs with LOG(ERROR)).
     */
    static std::shared_ptr<GpuSfMScene> try_create_from_track_store(
            const sfm::TrackStore&                     store,
            const std::vector<camera::Intrinsics>&     cameras,
            const std::vector<int>&                    image_to_camera_index,
            const std::vector<Eigen::Matrix3d>&        poses_R,
            const std::vector<Eigen::Vector3d>&        poses_C,
            const std::vector<bool>&                   registered);

    [[nodiscard]] CudaSfMState*       cuda_state() { return state_.get(); }
    [[nodiscard]] const CudaSfMState* cuda_state() const { return state_.get(); }

    [[nodiscard]] int num_images() const { return state_ ? state_->n_images : 0; }
    [[nodiscard]] int num_tracks() const { return state_ ? state_->n_tracks : 0; }
    [[nodiscard]] int num_observations() const { return state_ ? state_->n_obs : 0; }
    [[nodiscard]] int num_cameras() const { return state_ ? state_->n_cameras : 0; }

    // ── Host ↔ device: align with TrackStore / pose vectors (explicit sync boundaries) ──

    /// Full re-upload of track XYZ + triangulated-valid mask from host `TrackStore`.
    void upload_tracks_from_host(const sfm::TrackStore& store);

    /// Full re-upload of per-observation alive flags from host `TrackStore` (`is_obs_valid`).
    void upload_obs_valid_from_host(const sfm::TrackStore& store);

    void upload_poses_from_host(const std::vector<Eigen::Matrix3d>& poses_R,
                                const std::vector<Eigen::Vector3d>& poses_C,
                                const std::vector<bool>&             registered);

    void upload_one_pose_register(int image_index,
                                  const Eigen::Matrix3d& R,
                                  const Eigen::Vector3d& C);

    void upload_intrinsics_from_host(const std::vector<camera::Intrinsics>& cameras);

    void undistort_all_observations();

    void download_obs_norm_to_host(std::vector<float>* obs_xn, std::vector<float>* obs_yn) const;

    /**
     * Count per-image number of observations with a valid triangulated track (3D-2D count).
     * Used by the GPU-accelerated resection candidate pre-filter (M5).
     *
     * @param n_tri_out  Output vector of size `num_images()`, filled with per-image counts.
     */
    void count_tri_per_image(std::vector<int>* n_tri_out) const;

    /// Upload poses + intrinsics; rerun undistort if any intrinsics component changed.
    /// @return true if `undistort_all_observations()` was run.
    bool sync_after_bundle_adjust(const std::vector<Eigen::Matrix3d>& poses_R,
                                  const std::vector<Eigen::Vector3d>& poses_C,
                                  const std::vector<bool>&             registered,
                                  const std::vector<camera::Intrinsics>& cameras,
                                  const std::vector<camera::Intrinsics>& cameras_before_ba);

    void set_track_valid_on_device(int track_id, uint8_t value);

    // ── GPU retriangulation support (M4 Phase 1) ─────────────────────────

    /**
     * Mark a batch of tracks as needing retriangulation (sets d_track_needs_retri = 1).
     * Called by the pipeline when outlier rejection deletes restorable observations
     * for tracks that still have ≥ 2 alive observations.
     */
    void mark_tracks_need_retri(const std::vector<int>& track_ids);

    /**
     * Collect track ids with d_track_needs_retri == 1, then reset all flags to 0.
     * The caller is expected to call `run_batch_triangulation_gpu` on the returned ids.
     *
     * @param track_ids_out  Filled with global track ids that need retriangulation.
     */
    void drain_retri_pending(std::vector<int>* track_ids_out);

    // ── GPU BA subset selection ───────────────────────────────────────────

    /**
     * Run 3-pass GPU grid-NMS BA subset selection.
     * Writes `d_track_flags_ba` (1 = skip from Ceres BA) on device.
     * Pass `cuda_state()->d_track_flags_ba` as `d_skip_ba` argument to `pack_for_ba`.
     */
    void select_ba_subset(const BaSubsetOptions& opts);

    /// Read-only access to the BA skip flags on device.
    [[nodiscard]] const uint8_t* track_flags_ba() const
    {
        return state_ ? state_->d_track_flags_ba : nullptr;
    }

    // ── BA workset pack / unpack (GPU → host → Ceres → host → GPU) ────────

    /**
     * Build a `LocalBAWorkset` from the current GPU state for a selected subset of images.
     *
     * @param d_image_mask  Device uint8 array [n_images]: 1 = include, 0 = skip.
     *                      Pass nullptr to include ALL registered images.
     * @param anchor_image  Global image index to mark as fixed (fixed_pose=1).
     *                      Pass -1 for none (all variable).
     * @param workset_out   Output; filled on success.
     * @param d_skip_ba     Optional device uint8 [n_tracks]: 1 = exclude this track.
     *                      Pass nullptr to include all valid triangulated tracks.
     * @return true on success.
     */
    bool pack_for_ba(const uint8_t* d_image_mask,
                     int            anchor_image,
                     LocalBAWorkset* workset_out,
                     const uint8_t* d_skip_ba = nullptr) const;

    /**
     * Scatter Ceres BA track XYZ results back to device.
     *
     * Only updates `d_track_xyz` / `d_track_valid` for tracks in the workset.
     * Poses are NOT uploaded here — call `sync_after_bundle_adjust()` immediately
     * after to push optimised poses (and re-run undistort if intrinsics changed).
     * This avoids N small per-image cudaMemcpy calls; the single full-scene upload
     * in sync_after_bundle_adjust is cheaper than scattering individually.
     */
    void upload_ba_result(const LocalBAWorkset& workset);

    // ── Algorithms that mutate / read this scene on device (kernel entry points) ──

    /// Reprojection-threshold outlier pass; uses `CudaSfMState::d_reproj_reject_mark` [n_obs].
    int reject_reproj_multiview(float             threshold_px,
                                std::vector<int>* deleted_restorable_out,
                                std::vector<int>* deleted_permanent_out);

    void outlier_angle_build_csr() const;

    int reject_angle_multiview(float             min_angle_deg,
                               float             max_angle_deg,
                               std::vector<int>* deleted_obs_out);

private:
    explicit GpuSfMScene(std::shared_ptr<CudaSfMState> impl) : state_(std::move(impl)) {}

    std::shared_ptr<CudaSfMState> state_;
};

} // namespace cuda
} // namespace insight
