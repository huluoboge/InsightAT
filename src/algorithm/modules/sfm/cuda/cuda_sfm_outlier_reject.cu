/**
 * @file  cuda_sfm_outlier_reject.cu
 * @brief GPU multi-view outlier rejection kernels and host wrappers.
 */

#include "cuda_sfm_outlier_reject.cuh"
#include "cuda_sfm_common.cuh"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cuda_runtime.h>

#define CUDA_CHECK(call)                                                    \
    do {                                                                    \
        cudaError_t _e = (call);                                           \
        if (_e != cudaSuccess) {                                            \
            fprintf(stderr, "CUDA error %s:%d  %s: %s\n",                 \
                    __FILE__, __LINE__, #call,                             \
                    cudaGetErrorString(_e));                                \
            std::abort();                                                   \
        }                                                                   \
    } while (0)

namespace insight {
namespace cuda {

// ─────────────────────────────────────────────────────────────────────────────
// Reprojection outlier kernel
// ─────────────────────────────────────────────────────────────────────────────

/// One thread per observation.
/// Flags d_deleted[i] = 1 if the obs should be marked deleted.
__global__
void kernel_reject_reproj(
        const float*   d_obs_u,
        const float*   d_obs_v,
        const int*     d_obs_image_idx,
        const int*     d_obs_track_idx,
        const int*     d_obs_cam_idx,
        const uint8_t* d_obs_valid,
        const float*   d_poses_R,       // [n_images * 9]
        const float*   d_poses_C,       // [n_images * 3]
        const float*   d_track_xyz,     // [n_tracks * 3]
        const uint8_t* d_track_valid,
        const uint8_t* d_registered,
        const float*   d_intrinsics,    // [n_cameras * 9]
        float          thresh_sq,       // threshold_px²
        uint8_t*       d_deleted,
        int            n_obs)
{
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n_obs) return;

    d_deleted[i] = 0;

    if (!d_obs_valid[i])                      return;

    const int im    = d_obs_image_idx[i];
    const int track = d_obs_track_idx[i];

    if (!d_registered[im])                    return;
    if (!d_track_valid[track])                return;

    const float* R   = d_poses_R  + im * 9;
    const float* C   = d_poses_C  + im * 3;
    const float* K   = d_intrinsics + d_obs_cam_idx[i] * 9;
    const float* XYZ = d_track_xyz + track * 3;

    GpuIntrinsics intr = GpuIntrinsics::from_array(K);

    float u_proj, v_proj;
    const bool visible = dev_project_f(R, C, intr, XYZ[0], XYZ[1], XYZ[2], &u_proj, &v_proj);
    if (!visible) {
        d_deleted[i] = 2; // permanent (cheirality)
        return;
    }

    const float du = d_obs_u[i] - u_proj;
    const float dv = d_obs_v[i] - v_proj;
    if (du * du + dv * dv > thresh_sq)
        d_deleted[i] = 1; // restorable reproj outlier
}

int gpu_reject_reproj_multiview(CudaSfMState*     state,
                                float             threshold_px,
                                std::vector<int>* deleted_restorable_out,
                                std::vector<int>* deleted_permanent_out)
{
    if (!state || !deleted_restorable_out || !deleted_permanent_out) return 0;
    const int n_obs = state->n_obs;
    if (n_obs <= 0 || !state->d_reproj_reject_mark) return 0;

    CUDA_CHECK(cudaMemset(state->d_reproj_reject_mark, 0,
                          static_cast<size_t>(n_obs) * sizeof(uint8_t)));

    const int block = 256;
    const int grid  = (n_obs + block - 1) / block;
    kernel_reject_reproj<<<grid, block>>>(
            state->d_obs_u,
            state->d_obs_v,
            state->d_obs_image_idx,
            state->d_obs_track_idx,
            state->d_obs_cam_idx,
            state->d_obs_valid,
            state->d_poses_R,
            state->d_poses_C,
            state->d_track_xyz,
            state->d_track_valid,
            state->d_registered,
            state->d_intrinsics,
            threshold_px * threshold_px,
            state->d_reproj_reject_mark,
            n_obs);
    CUDA_CHECK(cudaGetLastError());
    CUDA_CHECK(cudaDeviceSynchronize());

    std::vector<uint8_t> h_deleted(static_cast<size_t>(n_obs));
    CUDA_CHECK(cudaMemcpy(h_deleted.data(), state->d_reproj_reject_mark,
                          static_cast<size_t>(n_obs) * sizeof(uint8_t),
                          cudaMemcpyDeviceToHost));

    deleted_restorable_out->clear();
    deleted_permanent_out->clear();
    for (int i = 0; i < n_obs; ++i) {
        if (h_deleted[i] == 1)
            deleted_restorable_out->push_back(i);
        else if (h_deleted[i] == 2)
            deleted_permanent_out->push_back(i);
    }

    auto batch_invalidate = [&](const std::vector<int>& ids) {
        if (ids.empty()) return;
        gpu_sfm_invalidate_obs_batch(state, ids.data(), static_cast<int>(ids.size()));
    };
    batch_invalidate(*deleted_restorable_out);
    batch_invalidate(*deleted_permanent_out);

    return static_cast<int>(deleted_restorable_out->size() + deleted_permanent_out->size());
}

// ─────────────────────────────────────────────────────────────────────────────
// Per-track angle check
// ─────────────────────────────────────────────────────────────────────────────

void gpu_outlier_angle_build_csr(CudaSfMState* state)
{
    if (!state || !state->d_angle_csr_ptr || !state->d_angle_csr_obs) return;
    const int n_obs    = state->n_obs;
    const int n_tracks = state->n_tracks;

    std::vector<uint8_t> h_obs_valid(static_cast<size_t>(n_obs));
    std::vector<int>     h_obs_track(static_cast<size_t>(n_obs));
    std::vector<uint8_t> h_track_valid(static_cast<size_t>(n_tracks));

    CUDA_CHECK(cudaMemcpy(h_obs_valid.data(),  state->d_obs_valid,
                          static_cast<size_t>(n_obs) * sizeof(uint8_t), cudaMemcpyDeviceToHost));
    CUDA_CHECK(cudaMemcpy(h_obs_track.data(),  state->d_obs_track_idx,
                          static_cast<size_t>(n_obs) * sizeof(int),     cudaMemcpyDeviceToHost));
    CUDA_CHECK(cudaMemcpy(h_track_valid.data(),state->d_track_valid,
                          static_cast<size_t>(n_tracks)  * sizeof(uint8_t), cudaMemcpyDeviceToHost));

    std::vector<int> cnt(static_cast<size_t>(n_tracks), 0);
    for (int i = 0; i < n_obs; ++i) {
        if (!h_obs_valid[i]) continue;
        const int t = h_obs_track[i];
        if (t >= 0 && t < n_tracks && h_track_valid[t])
            cnt[static_cast<size_t>(t)]++;
    }
    std::vector<int> csr_ptr(static_cast<size_t>(n_tracks + 1), 0);
    for (int t = 0; t < n_tracks; ++t)
        csr_ptr[static_cast<size_t>(t + 1)] = csr_ptr[static_cast<size_t>(t)] + cnt[static_cast<size_t>(t)];
    const int n_alive = csr_ptr[static_cast<size_t>(n_tracks)];

    std::vector<int> csr_obs(static_cast<size_t>(n_alive));
    std::vector<int> pos(static_cast<size_t>(n_tracks), 0);
    for (int i = 0; i < n_obs; ++i) {
        if (!h_obs_valid[i]) continue;
        const int t = h_obs_track[i];
        if (t >= 0 && t < n_tracks && h_track_valid[t]) {
            const int slot = csr_ptr[static_cast<size_t>(t)] + pos[static_cast<size_t>(t)];
            csr_obs[static_cast<size_t>(slot)] = i;
            pos[static_cast<size_t>(t)]++;
        }
    }

    CUDA_CHECK(cudaMemcpy(state->d_angle_csr_ptr, csr_ptr.data(),
                          static_cast<size_t>(n_tracks + 1) * sizeof(int), cudaMemcpyHostToDevice));
    if (n_alive > 0) {
        CUDA_CHECK(cudaMemcpy(state->d_angle_csr_obs, csr_obs.data(),
                              static_cast<size_t>(n_alive) * sizeof(int), cudaMemcpyHostToDevice));
    }
}

__global__
void kernel_reject_angle(
        const int*     d_csr_ptr,
        const int*     d_csr_obs,
        const int*     d_obs_image_idx,
        const uint8_t* d_registered,
        const float*   d_poses_C,
        const float*   d_track_xyz,
        const uint8_t* d_track_valid,
        float          min_angle_rad,
        float          max_angle_rad,
        uint8_t*       d_per_obs_flag,
        int            n_tracks)
{
    const int t = blockIdx.x * blockDim.x + threadIdx.x;
    if (t >= n_tracks) return;
    if (!d_track_valid[t]) return;

    const int obs_start = d_csr_ptr[t];
    const int obs_end   = d_csr_ptr[t + 1];
    const int deg       = obs_end - obs_start;
    if (deg < 2) return;

    const float Xw = d_track_xyz[t * 3 + 0];
    const float Yw = d_track_xyz[t * 3 + 1];
    const float Zw = d_track_xyz[t * 3 + 2];

    const int max_inner = min(deg, 16);

    float rays_x[16], rays_y[16], rays_z[16];
    int   ray_obs[16];
    int   n_rays = 0;
    for (int k = obs_start; k < obs_end && n_rays < max_inner; ++k) {
        const int oi = d_csr_obs[k];
        const int im = d_obs_image_idx[oi];
        if (!d_registered[im]) continue;
        const float* C = d_poses_C + im * 3;
        float rx = Xw - C[0], ry = Yw - C[1], rz = Zw - C[2];
        const float len = sqrtf(rx * rx + ry * ry + rz * rz);
        if (len < 1e-8f) continue;
        const float inv_len = 1.0f / len;
        rays_x[n_rays] = rx * inv_len;
        rays_y[n_rays] = ry * inv_len;
        rays_z[n_rays] = rz * inv_len;
        ray_obs[n_rays] = oi;
        n_rays++;
    }
    if (n_rays < 2) return;

    for (int a = 0; a < n_rays; ++a) {
        float max_angle = 0.0f;
        for (int b = 0; b < n_rays; ++b) {
            if (b == a) continue;
            float cos_ab = rays_x[a]*rays_x[b] + rays_y[a]*rays_y[b] + rays_z[a]*rays_z[b];
            if (cos_ab >  1.0f) cos_ab =  1.0f;
            if (cos_ab < -1.0f) cos_ab = -1.0f;
            const float angle = acosf(cos_ab);
            if (angle > max_angle) max_angle = angle;
        }
        if (max_angle < min_angle_rad || max_angle > max_angle_rad)
            d_per_obs_flag[ray_obs[a]] = 1;
    }
}

int gpu_reject_angle_multiview(CudaSfMState*     state,
                               float             min_angle_deg,
                               float             max_angle_deg,
                               std::vector<int>* deleted_obs_out)
{
    if (!state || !deleted_obs_out || !state->d_angle_per_obs_flag) return 0;
    if (min_angle_deg <= 0.0f) return 0;

    const float min_rad = min_angle_deg * (3.14159265f / 180.0f);
    const float max_rad = max_angle_deg * (3.14159265f / 180.0f);
    const int n_obs    = state->n_obs;
    const int n_tracks = state->n_tracks;

    CUDA_CHECK(cudaMemset(state->d_angle_per_obs_flag, 0,
                          static_cast<size_t>(n_obs) * sizeof(uint8_t)));

    const int block = 256;
    const int grid  = (n_tracks + block - 1) / block;
    kernel_reject_angle<<<grid, block>>>(
            state->d_angle_csr_ptr,
            state->d_angle_csr_obs,
            state->d_obs_image_idx,
            state->d_registered,
            state->d_poses_C,
            state->d_track_xyz,
            state->d_track_valid,
            min_rad, max_rad,
            state->d_angle_per_obs_flag,
            n_tracks);
    CUDA_CHECK(cudaGetLastError());
    CUDA_CHECK(cudaDeviceSynchronize());

    std::vector<uint8_t> h_flag(static_cast<size_t>(n_obs));
    CUDA_CHECK(cudaMemcpy(h_flag.data(), state->d_angle_per_obs_flag,
                          static_cast<size_t>(n_obs) * sizeof(uint8_t),
                          cudaMemcpyDeviceToHost));

    deleted_obs_out->clear();
    for (int i = 0; i < n_obs; ++i) {
        if (h_flag[i])
            deleted_obs_out->push_back(i);
    }
    if (!deleted_obs_out->empty()) {
        gpu_sfm_invalidate_obs_batch(state, deleted_obs_out->data(),
                                     static_cast<int>(deleted_obs_out->size()));
    }
    return static_cast<int>(deleted_obs_out->size());
}

} // namespace cuda
} // namespace insight
