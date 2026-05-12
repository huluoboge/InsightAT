/**
 * @file  gpu_sfm_scene.cu
 * @brief Factory + helpers for `GpuSfMScene` (host→device upload from `TrackStore`).
 */

#include "gpu_sfm_scene.h"

#include "cuda_ba_subset.cuh"
#include "cuda_sfm_math.cuh"
#include "cuda_sfm_outlier_reject.cuh"

#include <cuda_runtime.h>

#include <cmath>
#include <glog/logging.h>

namespace insight {
namespace cuda {

namespace {

using insight::cuda::eigen_R_to_float9;
using insight::cuda::eigen_C_to_float3;
using insight::cuda::intrinsics_to_float9;

/// Returns owning raw pointer; nullptr on failure.
static CudaSfMState* build_cuda_sfm_state_from_store(
        const sfm::TrackStore&                     store,
        const std::vector<camera::Intrinsics>&     cameras,
        const std::vector<int>&                    image_to_camera_index,
        const std::vector<Eigen::Matrix3d>&        poses_R,
        const std::vector<Eigen::Vector3d>&        poses_C,
        const std::vector<bool>&                   registered)
{
    const int n_images  = store.num_images();
    const int n_tracks  = static_cast<int>(store.num_tracks());
    const int n_obs     = static_cast<int>(store.num_observations());
    const int n_cameras = static_cast<int>(cameras.size());

    CudaSfMState* gs = gpu_sfm_state_create(n_images, n_tracks, n_obs, n_cameras);
    if (!gs) {
        LOG(ERROR) << "GpuSfMScene: gpu_sfm_state_create failed";
        return nullptr;
    }

    std::vector<float> K_flat(static_cast<size_t>(n_cameras) * 9);
    for (int c = 0; c < n_cameras; ++c)
        intrinsics_to_float9(cameras[static_cast<size_t>(c)], K_flat.data() + c * 9);
    gpu_sfm_upload_intrinsics(gs, K_flat.data(), n_cameras);

    std::vector<float>   poses_R_flat(static_cast<size_t>(n_images) * 9, 0.0f);
    std::vector<float>   poses_C_flat(static_cast<size_t>(n_images) * 3, 0.0f);
    std::vector<uint8_t> reg_flags(static_cast<size_t>(n_images), 0);
    for (int i = 0; i < n_images; ++i) {
        if (registered[static_cast<size_t>(i)]) {
            eigen_R_to_float9(poses_R[static_cast<size_t>(i)], poses_R_flat.data() + i * 9);
            eigen_C_to_float3(poses_C[static_cast<size_t>(i)], poses_C_flat.data() + i * 3);
            reg_flags[static_cast<size_t>(i)] = 1;
        }
    }
    gpu_sfm_upload_poses(gs, poses_R_flat.data(), poses_C_flat.data(), reg_flags.data(), n_images);

    std::vector<float>   track_xyz_flat(static_cast<size_t>(n_tracks) * 3, 0.0f);
    std::vector<uint8_t> track_valid_flat(static_cast<size_t>(n_tracks), 0);
    for (int t = 0; t < n_tracks; ++t) {
        if (store.is_track_valid(t) && store.track_has_triangulated_xyz(t)) {
            float x, y, z;
            store.get_track_xyz(t, &x, &y, &z);
            track_xyz_flat[static_cast<size_t>(t) * 3 + 0] = x;
            track_xyz_flat[static_cast<size_t>(t) * 3 + 1] = y;
            track_xyz_flat[static_cast<size_t>(t) * 3 + 2] = z;
            track_valid_flat[static_cast<size_t>(t)] = 1;
        }
    }
    gpu_sfm_upload_tracks(gs, track_xyz_flat.data(), track_valid_flat.data(), n_tracks);

    std::vector<float>   obs_u(static_cast<size_t>(n_obs));
    std::vector<float>   obs_v(static_cast<size_t>(n_obs));
    std::vector<int>     obs_image_idx(static_cast<size_t>(n_obs));
    std::vector<int>     obs_track_idx(static_cast<size_t>(n_obs));
    std::vector<int>     obs_cam_idx(static_cast<size_t>(n_obs));
    std::vector<uint8_t> obs_valid(static_cast<size_t>(n_obs));

    sfm::Observation obs_tmp;
    for (int i = 0; i < n_obs; ++i) {
        store.get_obs(i, &obs_tmp);
        obs_u[static_cast<size_t>(i)]         = static_cast<float>(obs_tmp.u);
        obs_v[static_cast<size_t>(i)]         = static_cast<float>(obs_tmp.v);
        obs_image_idx[static_cast<size_t>(i)] = static_cast<int>(obs_tmp.image_index);
        obs_track_idx[static_cast<size_t>(i)] = store.obs_track_id(i);
        const int im = static_cast<int>(obs_tmp.image_index);
        obs_cam_idx[static_cast<size_t>(i)]   = (im >= 0 && im < static_cast<int>(image_to_camera_index.size()))
                                               ? image_to_camera_index[static_cast<size_t>(im)]
                                               : 0;
        obs_valid[static_cast<size_t>(i)]     = store.is_obs_valid(i) ? 1u : 0u;
    }
    gpu_sfm_upload_observations(gs, obs_u.data(), obs_v.data(),
                                 obs_image_idx.data(), obs_track_idx.data(),
                                 obs_cam_idx.data(), obs_valid.data(), n_obs);

    {
        std::vector<int> h_img_obs_ptr(static_cast<size_t>(n_images + 1));
        std::vector<int> h_img_obs_idx;
        h_img_obs_idx.reserve(static_cast<size_t>(n_obs));
        for (int im = 0; im < n_images; ++im) {
            h_img_obs_ptr[static_cast<size_t>(im)] = static_cast<int>(h_img_obs_idx.size());
            std::vector<int> ids;
            store.get_image_all_obs_ids(im, &ids);
            for (int oid : ids)
                h_img_obs_idx.push_back(oid);
        }
        h_img_obs_ptr[static_cast<size_t>(n_images)] = static_cast<int>(h_img_obs_idx.size());
        gpu_sfm_upload_img_obs_csr(gs, h_img_obs_ptr.data(), h_img_obs_idx.data(), n_images,
                                   static_cast<int>(h_img_obs_idx.size()));
    }

    gpu_sfm_state_undistort_all(gs);
    return gs;
}

} // namespace

std::shared_ptr<GpuSfMScene> GpuSfMScene::try_create_from_track_store(
        const sfm::TrackStore&                     store,
        const std::vector<camera::Intrinsics>&     cameras,
        const std::vector<int>&                    image_to_camera_index,
        const std::vector<Eigen::Matrix3d>&        poses_R,
        const std::vector<Eigen::Vector3d>&        poses_C,
        const std::vector<bool>&                   registered)
{
    CudaSfMState* raw = build_cuda_sfm_state_from_store(store, cameras, image_to_camera_index,
                                                        poses_R, poses_C, registered);
    if (!raw)
        return nullptr;
    auto impl = std::shared_ptr<CudaSfMState>(raw, [](CudaSfMState* p) { gpu_sfm_state_free(p); });
    return std::shared_ptr<GpuSfMScene>(new GpuSfMScene(std::move(impl)));
}


void GpuSfMScene::upload_tracks_from_host(const sfm::TrackStore& store)
{
    CudaSfMState* gs = state_.get();
    if (!gs) return;
    const int n_tracks = gs->n_tracks;
    std::vector<float>   xyz_flat(static_cast<size_t>(n_tracks) * 3, 0.0f);
    std::vector<uint8_t> valid_flat(static_cast<size_t>(n_tracks), 0u);
    for (int t = 0; t < n_tracks; ++t) {
        if (store.is_track_valid(t) && store.track_has_triangulated_xyz(t)) {
            float x, y, z;
            store.get_track_xyz(t, &x, &y, &z);
            xyz_flat[static_cast<size_t>(t) * 3 + 0] = x;
            xyz_flat[static_cast<size_t>(t) * 3 + 1] = y;
            xyz_flat[static_cast<size_t>(t) * 3 + 2] = z;
            valid_flat[static_cast<size_t>(t)] = 1u;
        }
    }
    gpu_sfm_upload_tracks(gs, xyz_flat.data(), valid_flat.data(), n_tracks);
}

void GpuSfMScene::upload_obs_valid_from_host(const sfm::TrackStore& store)
{
    CudaSfMState* gs = state_.get();
    if (!gs) return;
    const int n_obs = gs->n_obs;
    std::vector<uint8_t> valid(static_cast<size_t>(n_obs));
    for (int i = 0; i < n_obs; ++i)
        valid[static_cast<size_t>(i)] = store.is_obs_valid(i) ? 1u : 0u;
    cudaMemcpy(gs->d_obs_valid, valid.data(),
               static_cast<size_t>(n_obs) * sizeof(uint8_t), cudaMemcpyHostToDevice);
}

void GpuSfMScene::upload_poses_from_host(const std::vector<Eigen::Matrix3d>& poses_R,
                                         const std::vector<Eigen::Vector3d>& poses_C,
                                         const std::vector<bool>&             registered)
{
    CudaSfMState* gs = state_.get();
    if (!gs) return;
    const int n_images = gs->n_images;
    std::vector<float>   R_flat(static_cast<size_t>(n_images) * 9, 0.0f);
    std::vector<float>   C_flat(static_cast<size_t>(n_images) * 3, 0.0f);
    std::vector<uint8_t> reg_flags(static_cast<size_t>(n_images), 0);
    for (int i = 0; i < n_images; ++i) {
        if (registered[static_cast<size_t>(i)]) {
            eigen_R_to_float9(poses_R[static_cast<size_t>(i)], R_flat.data() + i * 9);
            eigen_C_to_float3(poses_C[static_cast<size_t>(i)], C_flat.data() + i * 3);
            reg_flags[static_cast<size_t>(i)] = 1;
        }
    }
    gpu_sfm_upload_poses(gs, R_flat.data(), C_flat.data(), reg_flags.data(), n_images);
}

void GpuSfMScene::upload_one_pose_register(int image_index,
                                           const Eigen::Matrix3d& R,
                                           const Eigen::Vector3d& C)
{
    CudaSfMState* gs = state_.get();
    if (!gs) return;
    float R9[9], C3[3];
    eigen_R_to_float9(R, R9);
    eigen_C_to_float3(C, C3);
    gpu_sfm_upload_one_pose(gs, image_index, R9, C3);
    gpu_sfm_set_registered(gs, image_index, 1u);
}

void GpuSfMScene::upload_intrinsics_from_host(const std::vector<camera::Intrinsics>& cameras)
{
    CudaSfMState* gs = state_.get();
    if (!gs) return;
    const int n_cameras = static_cast<int>(cameras.size());
    std::vector<float> K_flat(static_cast<size_t>(n_cameras) * 9);
    for (int c = 0; c < n_cameras; ++c)
        intrinsics_to_float9(cameras[static_cast<size_t>(c)], K_flat.data() + c * 9);
    gpu_sfm_upload_intrinsics(gs, K_flat.data(), n_cameras);
}

void GpuSfMScene::undistort_all_observations()
{
    if (state_)
        gpu_sfm_state_undistort_all(state_.get());
}

void GpuSfMScene::count_tri_per_image(std::vector<int>* n_tri_out) const
{
    if (!state_ || !n_tri_out) return;
    const int n_images = state_->n_images;
    n_tri_out->resize(static_cast<size_t>(n_images), 0);
    gpu_sfm_count_tri_per_image(state_.get(), n_tri_out->data());
}

void GpuSfMScene::download_obs_norm_to_host(std::vector<float>* obs_xn,
                                            std::vector<float>* obs_yn) const
{
    if (!state_ || !obs_xn || !obs_yn) return;
    const int n_obs = state_->n_obs;
    obs_xn->resize(static_cast<size_t>(n_obs));
    obs_yn->resize(static_cast<size_t>(n_obs));
    gpu_sfm_download_obs_norm(state_.get(),
                              obs_xn->data(),
                              obs_yn->data(),
                              n_obs);
}

bool GpuSfMScene::sync_after_bundle_adjust(const std::vector<Eigen::Matrix3d>& poses_R,
                                           const std::vector<Eigen::Vector3d>& poses_C,
                                           const std::vector<bool>&             registered,
                                           const std::vector<camera::Intrinsics>& cameras,
                                           const std::vector<camera::Intrinsics>& cameras_before_ba)
{
    upload_poses_from_host(poses_R, poses_C, registered);
    upload_intrinsics_from_host(cameras);

    const int n_cameras = static_cast<int>(cameras.size());
    bool intrinsics_changed = false;
    if (cameras_before_ba.empty()) {
        intrinsics_changed = true;
    } else {
        const int n_check = std::min(n_cameras, static_cast<int>(cameras_before_ba.size()));
        constexpr double eps = 1e-7;
        for (int c = 0; c < n_check && !intrinsics_changed; ++c) {
            const camera::Intrinsics& A = cameras_before_ba[static_cast<size_t>(c)];
            const camera::Intrinsics& B = cameras[static_cast<size_t>(c)];
            if (std::abs(A.fx - B.fx) > eps || std::abs(A.fy - B.fy) > eps ||
                std::abs(A.cx - B.cx) > eps || std::abs(A.cy - B.cy) > eps ||
                std::abs(A.k1 - B.k1) > eps || std::abs(A.k2 - B.k2) > eps ||
                std::abs(A.k3 - B.k3) > eps || std::abs(A.p1 - B.p1) > eps ||
                std::abs(A.p2 - B.p2) > eps)
                intrinsics_changed = true;
        }
    }

    if (intrinsics_changed) {
        undistort_all_observations();
        return true;
    }
    return false;
}

void GpuSfMScene::set_track_valid_on_device(int track_id, uint8_t value)
{
    if (state_)
        gpu_sfm_set_track_valid(state_.get(), track_id, value);
}

void GpuSfMScene::select_ba_subset(const BaSubsetOptions& opts)
{
    if (state_)
        gpu_select_ba_subset(state_.get(), opts);
}

void GpuSfMScene::mark_tracks_need_retri(const std::vector<int>& track_ids)
{
    if (!state_ || track_ids.empty()) return;
    gpu_sfm_mark_tracks_needs_retri(state_.get(),
                                    track_ids.data(),
                                    static_cast<int>(track_ids.size()));
}

void GpuSfMScene::drain_retri_pending(std::vector<int>* track_ids_out)
{
    if (!state_ || !track_ids_out) return;
    const int n_tracks = state_->n_tracks;
    track_ids_out->clear();

    std::vector<uint8_t> h_flags(static_cast<size_t>(n_tracks));
    cudaMemcpy(h_flags.data(), state_->d_track_needs_retri,
               static_cast<size_t>(n_tracks) * sizeof(uint8_t), cudaMemcpyDeviceToHost);

    for (int t = 0; t < n_tracks; ++t) {
        if (h_flags[static_cast<size_t>(t)])
            track_ids_out->push_back(t);
    }

    if (!track_ids_out->empty())
        gpu_sfm_clear_tracks_needs_retri(state_.get());
}

int GpuSfMScene::reject_reproj_multiview(float             threshold_px,
                                         std::vector<int>* deleted_restorable_out,
                                         std::vector<int>* deleted_permanent_out)
{
    if (!state_) return 0;
    return gpu_reject_reproj_multiview(state_.get(), threshold_px,
                                       deleted_restorable_out, deleted_permanent_out);
}

void GpuSfMScene::outlier_angle_build_csr() const
{
    if (state_)
        gpu_outlier_angle_build_csr(state_.get());
}

int GpuSfMScene::reject_angle_multiview(float             min_angle_deg,
                                        float             max_angle_deg,
                                        std::vector<int>* deleted_obs_out)
{
    if (!state_) return 0;
    return gpu_reject_angle_multiview(state_.get(), min_angle_deg, max_angle_deg, deleted_obs_out);
}

// ─────────────────────────────────────────────────────────────────────────────
// BA workset pack / unpack
// ─────────────────────────────────────────────────────────────────────────────

bool GpuSfMScene::pack_for_ba(const uint8_t* d_image_mask,
                               int            anchor_image,
                               LocalBAWorkset* workset_out,
                               const uint8_t* d_skip_ba) const
{
    if (!state_ || !workset_out) return false;
    const CudaSfMState* gs = state_.get();
    const int n_images  = gs->n_images;
    const int n_tracks  = gs->n_tracks;
    const int n_obs     = gs->n_obs;

    // ── Download GPU arrays needed for compaction ─────────────────────────
    std::vector<uint8_t> h_registered(static_cast<size_t>(n_images));
    std::vector<float>   h_poses_R(static_cast<size_t>(n_images) * 9);
    std::vector<float>   h_poses_C(static_cast<size_t>(n_images) * 3);
    cudaMemcpy(h_registered.data(), gs->d_registered,
               static_cast<size_t>(n_images) * sizeof(uint8_t), cudaMemcpyDeviceToHost);
    cudaMemcpy(h_poses_R.data(), gs->d_poses_R,
               static_cast<size_t>(n_images) * 9 * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(h_poses_C.data(), gs->d_poses_C,
               static_cast<size_t>(n_images) * 3 * sizeof(float), cudaMemcpyDeviceToHost);

    std::vector<uint8_t> h_image_mask_host(static_cast<size_t>(n_images), 0);
    if (d_image_mask) {
        cudaMemcpy(h_image_mask_host.data(), d_image_mask,
                   static_cast<size_t>(n_images) * sizeof(uint8_t), cudaMemcpyDeviceToHost);
    } else {
        // Default: include all registered images.
        for (int i = 0; i < n_images; ++i)
            h_image_mask_host[static_cast<size_t>(i)] = h_registered[static_cast<size_t>(i)];
    }

    std::vector<uint8_t> h_track_valid(static_cast<size_t>(n_tracks));
    std::vector<float>   h_track_xyz(static_cast<size_t>(n_tracks) * 3);
    cudaMemcpy(h_track_valid.data(), gs->d_track_valid,
               static_cast<size_t>(n_tracks) * sizeof(uint8_t), cudaMemcpyDeviceToHost);
    cudaMemcpy(h_track_xyz.data(), gs->d_track_xyz,
               static_cast<size_t>(n_tracks) * 3 * sizeof(float), cudaMemcpyDeviceToHost);

    std::vector<uint8_t> h_skip_ba(static_cast<size_t>(n_tracks), 0);
    if (d_skip_ba) {
        cudaMemcpy(h_skip_ba.data(), d_skip_ba,
                   static_cast<size_t>(n_tracks) * sizeof(uint8_t), cudaMemcpyDeviceToHost);
    }

    std::vector<int>     h_obs_image(static_cast<size_t>(n_obs));
    std::vector<int>     h_obs_track(static_cast<size_t>(n_obs));
    std::vector<int>     h_obs_cam(static_cast<size_t>(n_obs));
    std::vector<uint8_t> h_obs_valid(static_cast<size_t>(n_obs));
    std::vector<float>   h_obs_u(static_cast<size_t>(n_obs));
    std::vector<float>   h_obs_v(static_cast<size_t>(n_obs));
    std::vector<int>     h_img_obs_ptr(static_cast<size_t>(n_images + 1));
    std::vector<int>     h_img_obs_idx(static_cast<size_t>(n_obs));
    cudaMemcpy(h_obs_image.data(), gs->d_obs_image_idx,
               static_cast<size_t>(n_obs) * sizeof(int),     cudaMemcpyDeviceToHost);
    cudaMemcpy(h_obs_track.data(), gs->d_obs_track_idx,
               static_cast<size_t>(n_obs) * sizeof(int),     cudaMemcpyDeviceToHost);
    cudaMemcpy(h_obs_cam.data(),   gs->d_obs_cam_idx,
               static_cast<size_t>(n_obs) * sizeof(int),     cudaMemcpyDeviceToHost);
    cudaMemcpy(h_obs_valid.data(), gs->d_obs_valid,
               static_cast<size_t>(n_obs) * sizeof(uint8_t), cudaMemcpyDeviceToHost);
    cudaMemcpy(h_obs_u.data(),     gs->d_obs_u,
               static_cast<size_t>(n_obs) * sizeof(float),   cudaMemcpyDeviceToHost);
    cudaMemcpy(h_obs_v.data(),     gs->d_obs_v,
               static_cast<size_t>(n_obs) * sizeof(float),   cudaMemcpyDeviceToHost);
    cudaMemcpy(h_img_obs_ptr.data(), gs->d_img_obs_ptr,
               static_cast<size_t>(n_images + 1) * sizeof(int), cudaMemcpyDeviceToHost);
    cudaMemcpy(h_img_obs_idx.data(), gs->d_img_obs_idx,
               static_cast<size_t>(n_obs) * sizeof(int),        cudaMemcpyDeviceToHost);

    // ── Build local image index map ───────────────────────────────────────
    std::vector<int> global_to_local_image(static_cast<size_t>(n_images), -1);
    LocalBAWorkset& ws = *workset_out;
    ws.image_ids.clear();
    ws.poses_R.clear();
    ws.poses_C.clear();
    ws.fixed_pose.clear();

    for (int i = 0; i < n_images; ++i) {
        if (!h_image_mask_host[static_cast<size_t>(i)]) continue;
        const int local_i = static_cast<int>(ws.image_ids.size());
        global_to_local_image[static_cast<size_t>(i)] = local_i;
        ws.image_ids.push_back(i);
        const float* R = h_poses_R.data() + i * 9;
        const float* C = h_poses_C.data() + i * 3;
        for (int k = 0; k < 9; ++k) ws.poses_R.push_back(static_cast<double>(R[k]));
        for (int k = 0; k < 3; ++k) ws.poses_C.push_back(static_cast<double>(C[k]));
        ws.fixed_pose.push_back(i == anchor_image ? 1u : 0u);
    }
    ws.n_local_images = static_cast<int>(ws.image_ids.size());
    if (ws.n_local_images == 0) return false;

    // ── Determine which tracks are included ───────────────────────────────
    // A track is included if: valid, has XYZ, not skip_ba, and has >= 2 obs in selected images.
    // Use image CSR to build per-track alive-selected-obs count.
    std::vector<int> track_selected_obs_cnt(static_cast<size_t>(n_tracks), 0);
    for (int i = 0; i < n_images; ++i) {
        if (global_to_local_image[static_cast<size_t>(i)] < 0) continue;
        const int ptr0 = h_img_obs_ptr[static_cast<size_t>(i)];
        const int ptr1 = h_img_obs_ptr[static_cast<size_t>(i + 1)];
        for (int p = ptr0; p < ptr1; ++p) {
            const int oi = h_img_obs_idx[static_cast<size_t>(p)];
            if (oi < 0 || oi >= n_obs) continue;
            if (!h_obs_valid[static_cast<size_t>(oi)]) continue;
            const int tid = h_obs_track[static_cast<size_t>(oi)];
            if (tid >= 0 && tid < n_tracks)
                ++track_selected_obs_cnt[static_cast<size_t>(tid)];
        }
    }

    std::vector<int> global_to_local_track(static_cast<size_t>(n_tracks), -1);
    ws.track_ids.clear();
    ws.track_xyz.clear();

    for (int t = 0; t < n_tracks; ++t) {
        if (!h_track_valid[static_cast<size_t>(t)]) continue;
        if (h_skip_ba[static_cast<size_t>(t)]) continue;
        if (track_selected_obs_cnt[static_cast<size_t>(t)] < 2) continue;
        const int local_t = static_cast<int>(ws.track_ids.size());
        global_to_local_track[static_cast<size_t>(t)] = local_t;
        ws.track_ids.push_back(t);
        const float* xyz = h_track_xyz.data() + t * 3;
        ws.track_xyz.push_back(static_cast<double>(xyz[0]));
        ws.track_xyz.push_back(static_cast<double>(xyz[1]));
        ws.track_xyz.push_back(static_cast<double>(xyz[2]));
    }
    ws.n_local_tracks = static_cast<int>(ws.track_ids.size());

    // ── Build observation list ────────────────────────────────────────────
    ws.obs_image_local.clear();
    ws.obs_track_local.clear();
    ws.obs_u.clear();
    ws.obs_v.clear();
    ws.obs_cam_idx.clear();

    for (int i = 0; i < n_images; ++i) {
        const int local_i = global_to_local_image[static_cast<size_t>(i)];
        if (local_i < 0) continue;
        const int ptr0 = h_img_obs_ptr[static_cast<size_t>(i)];
        const int ptr1 = h_img_obs_ptr[static_cast<size_t>(i + 1)];
        for (int p = ptr0; p < ptr1; ++p) {
            const int oi = h_img_obs_idx[static_cast<size_t>(p)];
            if (oi < 0 || oi >= n_obs) continue;
            if (!h_obs_valid[static_cast<size_t>(oi)]) continue;
            const int tid = h_obs_track[static_cast<size_t>(oi)];
            if (tid < 0 || tid >= n_tracks) continue;
            const int local_t = global_to_local_track[static_cast<size_t>(tid)];
            if (local_t < 0) continue;
            ws.obs_image_local.push_back(local_i);
            ws.obs_track_local.push_back(local_t);
            ws.obs_u.push_back(static_cast<double>(h_obs_u[static_cast<size_t>(oi)]));
            ws.obs_v.push_back(static_cast<double>(h_obs_v[static_cast<size_t>(oi)]));
            ws.obs_cam_idx.push_back(h_obs_cam[static_cast<size_t>(oi)]);
        }
    }
    ws.n_local_obs = static_cast<int>(ws.obs_image_local.size());

    return true;
}

void GpuSfMScene::upload_ba_result(const LocalBAWorkset& workset)
{
    if (!state_) return;
    CudaSfMState* gs = state_.get();

    // Poses are intentionally NOT uploaded here.  The caller must follow this
    // call with sync_after_bundle_adjust() (or upload_poses_from_host()), which
    // performs a single full-scene pose upload and handles intrinsics/undistort.
    // Scattering poses here would require one cudaMemcpy per local image (many
    // small transfers), and sync_after_bundle_adjust already does a contiguous
    // bulk upload — doing both would be redundant.

    // ── Scatter track XYZ back to device ─────────────────────────────────
    const int n_local_tracks = workset.n_local_tracks;
    if (n_local_tracks > 0 && !workset.track_xyz.empty()) {
        std::vector<int>   tid_list(static_cast<size_t>(n_local_tracks));
        std::vector<float> xyz_list(static_cast<size_t>(n_local_tracks) * 3);
        for (int lt = 0; lt < n_local_tracks; ++lt) {
            tid_list[static_cast<size_t>(lt)] = workset.track_ids[static_cast<size_t>(lt)];
            const double* xyz = workset.track_xyz.data() + lt * 3;
            xyz_list[static_cast<size_t>(lt) * 3 + 0] = static_cast<float>(xyz[0]);
            xyz_list[static_cast<size_t>(lt) * 3 + 1] = static_cast<float>(xyz[1]);
            xyz_list[static_cast<size_t>(lt) * 3 + 2] = static_cast<float>(xyz[2]);
        }
        gpu_sfm_update_tracks_batch(gs, tid_list.data(), xyz_list.data(), n_local_tracks);
    }
}

} // namespace cuda
} // namespace insight
