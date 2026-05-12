/**
 * @file  cuda_sfm_state.cu
 * @brief Persistent GPU state lifecycle and undistort kernel for incremental
 *        SfM CUDA pipeline.
 */

#include "cuda_sfm_state.h"
#include "cuda_sfm_common.cuh"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cuda_runtime.h>

// ─────────────────────────────────────────────────────────────────────────────
// Utility macro
// ─────────────────────────────────────────────────────────────────────────────

#define CUDA_CHECK(call)                                                    \
    do {                                                                    \
        cudaError_t _e = (call);                                           \
        if (_e != cudaSuccess) {                                            \
            fprintf(stderr, "CUDA error %s:%d  %s: %s\n",                 \
                    __FILE__, __LINE__, #call,                              \
                    cudaGetErrorString(_e));                                \
            std::abort();                                                   \
        }                                                                   \
    } while (0)

namespace insight {
namespace cuda {

// ─────────────────────────────────────────────────────────────────────────────
// CUDA kernels
// ─────────────────────────────────────────────────────────────────────────────

/// Recompute undistorted normalised coords for ALL observations.
/// One thread per observation.
__global__
void kernel_undistort_all(
        const float*   d_obs_u,
        const float*   d_obs_v,
        const int*     d_obs_cam_idx,
        const float*   d_intrinsics,   // [n_cameras * 9]
        const uint8_t* d_obs_valid,
        float*         d_obs_xn,
        float*         d_obs_yn,
        int            n_obs)
{
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n_obs) return;

    // Skip deleted observations — leave stale xn/yn in place (harmless).
    if (!d_obs_valid[i]) {
        d_obs_xn[i] = 0.0f;
        d_obs_yn[i] = 0.0f;
        return;
    }

    const int cam = d_obs_cam_idx[i];
    const float* K = d_intrinsics + cam * 9;
    const float fx = K[0], fy = K[1], cx = K[2], cy = K[3];
    const float k1 = K[4], k2 = K[5], k3 = K[6], p1 = K[7], p2 = K[8];

    float xn, yn;
    dev_undistort_nrm_f(d_obs_u[i], d_obs_v[i], fx, fy, cx, cy,
                         k1, k2, k3, p1, p2, &xn, &yn, 8);
    d_obs_xn[i] = xn;
    d_obs_yn[i] = yn;
}

/// Scatter-update track XYZ for a batch.
/// Each thread handles one track from the id list.
__global__
void kernel_scatter_track_xyz(
        const int*   d_tid_list,
        const float* d_xyz_data,  // [n * 3]
        float*       d_track_xyz, // [n_tracks * 3]
        uint8_t*     d_track_valid,
        int          n)
{
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    const int tid = d_tid_list[i];
    d_track_xyz[tid * 3 + 0] = d_xyz_data[i * 3 + 0];
    d_track_xyz[tid * 3 + 1] = d_xyz_data[i * 3 + 1];
    d_track_xyz[tid * 3 + 2] = d_xyz_data[i * 3 + 2];
    d_track_valid[tid] = 1;
}

/// Batch-invalidate observations.
__global__
void kernel_invalidate_obs(
        const int*  d_obs_ids,
        uint8_t*    d_obs_valid,
        int         n)
{
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    d_obs_valid[d_obs_ids[i]] = 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// Lifecycle
// ─────────────────────────────────────────────────────────────────────────────

CudaSfMState* gpu_sfm_state_create(int n_images, int n_tracks, int n_obs, int n_cameras)
{
    auto* s = new CudaSfMState;
    s->n_images  = n_images;
    s->n_tracks  = n_tracks;
    s->n_obs     = n_obs;
    s->n_cameras = n_cameras;

    CUDA_CHECK(cudaMalloc(&s->d_poses_R,    static_cast<size_t>(n_images)  * 9 * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&s->d_poses_C,    static_cast<size_t>(n_images)  * 3 * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&s->d_registered, static_cast<size_t>(n_images)  * sizeof(uint8_t)));

    CUDA_CHECK(cudaMalloc(&s->d_track_xyz,   static_cast<size_t>(n_tracks) * 3 * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&s->d_track_valid, static_cast<size_t>(n_tracks) * sizeof(uint8_t)));

    CUDA_CHECK(cudaMalloc(&s->d_obs_u,         static_cast<size_t>(n_obs) * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&s->d_obs_v,         static_cast<size_t>(n_obs) * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&s->d_obs_xn,        static_cast<size_t>(n_obs) * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&s->d_obs_yn,        static_cast<size_t>(n_obs) * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&s->d_obs_image_idx, static_cast<size_t>(n_obs) * sizeof(int)));
    CUDA_CHECK(cudaMalloc(&s->d_obs_track_idx, static_cast<size_t>(n_obs) * sizeof(int)));
    CUDA_CHECK(cudaMalloc(&s->d_obs_cam_idx,   static_cast<size_t>(n_obs) * sizeof(int)));
    CUDA_CHECK(cudaMalloc(&s->d_obs_valid,     static_cast<size_t>(n_obs) * sizeof(uint8_t)));

    CUDA_CHECK(cudaMalloc(&s->d_img_obs_ptr, static_cast<size_t>(n_images + 1) * sizeof(int)));
    CUDA_CHECK(cudaMalloc(&s->d_img_obs_idx, static_cast<size_t>(n_obs) * sizeof(int)));
    CUDA_CHECK(cudaMemset(s->d_img_obs_ptr, 0, static_cast<size_t>(n_images + 1) * sizeof(int)));
    CUDA_CHECK(cudaMemset(s->d_img_obs_idx, 0, static_cast<size_t>(n_obs) * sizeof(int)));

    CUDA_CHECK(cudaMalloc(&s->d_intrinsics, static_cast<size_t>(n_cameras) * 9 * sizeof(float)));

    CUDA_CHECK(cudaMalloc(&s->d_reproj_reject_mark, static_cast<size_t>(n_obs) * sizeof(uint8_t)));
    CUDA_CHECK(cudaMalloc(&s->d_angle_csr_ptr, static_cast<size_t>(n_tracks + 1) * sizeof(int)));
    CUDA_CHECK(cudaMalloc(&s->d_angle_csr_obs, static_cast<size_t>(n_obs) * sizeof(int)));
    CUDA_CHECK(cudaMalloc(&s->d_angle_per_obs_flag, static_cast<size_t>(n_obs) * sizeof(uint8_t)));

    CUDA_CHECK(cudaMalloc(&s->d_track_flags_ba,    static_cast<size_t>(n_tracks) * sizeof(uint8_t)));
    CUDA_CHECK(cudaMalloc(&s->d_track_needs_retri, static_cast<size_t>(n_tracks) * sizeof(uint8_t)));

    // Zero all buffers so uninitialised data is detectably 0.
    CUDA_CHECK(cudaMemset(s->d_poses_R,    0, static_cast<size_t>(n_images)  * 9 * sizeof(float)));
    CUDA_CHECK(cudaMemset(s->d_poses_C,    0, static_cast<size_t>(n_images)  * 3 * sizeof(float)));
    CUDA_CHECK(cudaMemset(s->d_registered, 0, static_cast<size_t>(n_images)  * sizeof(uint8_t)));
    CUDA_CHECK(cudaMemset(s->d_track_xyz,  0, static_cast<size_t>(n_tracks)  * 3 * sizeof(float)));
    CUDA_CHECK(cudaMemset(s->d_track_valid,0, static_cast<size_t>(n_tracks)  * sizeof(uint8_t)));
    CUDA_CHECK(cudaMemset(s->d_obs_xn,     0, static_cast<size_t>(n_obs)     * sizeof(float)));
    CUDA_CHECK(cudaMemset(s->d_obs_yn,     0, static_cast<size_t>(n_obs)     * sizeof(float)));
    CUDA_CHECK(cudaMemset(s->d_obs_valid,  0, static_cast<size_t>(n_obs)     * sizeof(uint8_t)));
    CUDA_CHECK(cudaMemset(s->d_intrinsics, 0, static_cast<size_t>(n_cameras) * 9 * sizeof(float)));
    CUDA_CHECK(cudaMemset(s->d_reproj_reject_mark, 0, static_cast<size_t>(n_obs) * sizeof(uint8_t)));
    CUDA_CHECK(cudaMemset(s->d_angle_csr_ptr, 0, static_cast<size_t>(n_tracks + 1) * sizeof(int)));
    CUDA_CHECK(cudaMemset(s->d_angle_csr_obs, 0, static_cast<size_t>(n_obs) * sizeof(int)));
    CUDA_CHECK(cudaMemset(s->d_angle_per_obs_flag, 0, static_cast<size_t>(n_obs) * sizeof(uint8_t)));
    CUDA_CHECK(cudaMemset(s->d_track_flags_ba,    0, static_cast<size_t>(n_tracks) * sizeof(uint8_t)));
    CUDA_CHECK(cudaMemset(s->d_track_needs_retri, 0, static_cast<size_t>(n_tracks) * sizeof(uint8_t)));

    return s;
}

void gpu_sfm_state_free(CudaSfMState* state)
{
    if (!state) return;
    cudaFree(state->d_poses_R);
    cudaFree(state->d_poses_C);
    cudaFree(state->d_registered);
    cudaFree(state->d_track_xyz);
    cudaFree(state->d_track_valid);
    cudaFree(state->d_obs_u);
    cudaFree(state->d_obs_v);
    cudaFree(state->d_obs_xn);
    cudaFree(state->d_obs_yn);
    cudaFree(state->d_obs_image_idx);
    cudaFree(state->d_obs_track_idx);
    cudaFree(state->d_obs_cam_idx);
    cudaFree(state->d_obs_valid);
    cudaFree(state->d_img_obs_ptr);
    cudaFree(state->d_img_obs_idx);
    cudaFree(state->d_intrinsics);
    cudaFree(state->d_reproj_reject_mark);
    cudaFree(state->d_angle_csr_ptr);
    cudaFree(state->d_angle_csr_obs);
    cudaFree(state->d_angle_per_obs_flag);
    cudaFree(state->d_track_flags_ba);
    cudaFree(state->d_track_needs_retri);
    delete state;
}

// ─────────────────────────────────────────────────────────────────────────────
// Upload helpers
// ─────────────────────────────────────────────────────────────────────────────

void gpu_sfm_upload_intrinsics(CudaSfMState* state,
                               const float* K_flat, int n_cameras)
{
    CUDA_CHECK(cudaMemcpy(state->d_intrinsics, K_flat,
                          static_cast<size_t>(n_cameras) * 9 * sizeof(float),
                          cudaMemcpyHostToDevice));
}

void gpu_sfm_upload_img_obs_csr(CudaSfMState* state, const int* h_img_obs_ptr, const int* h_img_obs_idx,
                                int n_images, int n_idx_values)
{
    if (!state || !h_img_obs_ptr || !h_img_obs_idx || n_images <= 0 || n_idx_values < 0)
        return;
    CUDA_CHECK(cudaMemcpy(state->d_img_obs_ptr, h_img_obs_ptr,
                          static_cast<size_t>(n_images + 1) * sizeof(int), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(state->d_img_obs_idx, h_img_obs_idx,
                          static_cast<size_t>(n_idx_values) * sizeof(int), cudaMemcpyHostToDevice));
}

void gpu_sfm_upload_observations(CudaSfMState* state,
                                 const float*   obs_u,
                                 const float*   obs_v,
                                 const int*     obs_image_idx,
                                 const int*     obs_track_idx,
                                 const int*     obs_cam_idx,
                                 const uint8_t* obs_valid,
                                 int n_obs)
{
    CUDA_CHECK(cudaMemcpy(state->d_obs_u,         obs_u,
                          static_cast<size_t>(n_obs) * sizeof(float),   cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(state->d_obs_v,         obs_v,
                          static_cast<size_t>(n_obs) * sizeof(float),   cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(state->d_obs_image_idx, obs_image_idx,
                          static_cast<size_t>(n_obs) * sizeof(int),     cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(state->d_obs_track_idx, obs_track_idx,
                          static_cast<size_t>(n_obs) * sizeof(int),     cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(state->d_obs_cam_idx,   obs_cam_idx,
                          static_cast<size_t>(n_obs) * sizeof(int),     cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(state->d_obs_valid,     obs_valid,
                          static_cast<size_t>(n_obs) * sizeof(uint8_t), cudaMemcpyHostToDevice));
}

void gpu_sfm_upload_poses(CudaSfMState* state,
                          const float*   poses_R,
                          const float*   poses_C,
                          const uint8_t* registered,
                          int n_images)
{
    CUDA_CHECK(cudaMemcpy(state->d_poses_R,    poses_R,
                          static_cast<size_t>(n_images) * 9 * sizeof(float),   cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(state->d_poses_C,    poses_C,
                          static_cast<size_t>(n_images) * 3 * sizeof(float),   cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(state->d_registered, registered,
                          static_cast<size_t>(n_images) * sizeof(uint8_t),     cudaMemcpyHostToDevice));
}

void gpu_sfm_upload_tracks(CudaSfMState* state,
                           const float*   track_xyz,
                           const uint8_t* track_valid,
                           int n_tracks)
{
    CUDA_CHECK(cudaMemcpy(state->d_track_xyz,   track_xyz,
                          static_cast<size_t>(n_tracks) * 3 * sizeof(float),   cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(state->d_track_valid, track_valid,
                          static_cast<size_t>(n_tracks) * sizeof(uint8_t),     cudaMemcpyHostToDevice));
}

void gpu_sfm_upload_one_pose(CudaSfMState* state,
                              int image_idx,
                              const float* R9,
                              const float* C3)
{
    CUDA_CHECK(cudaMemcpy(state->d_poses_R + image_idx * 9, R9,
                          9 * sizeof(float), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(state->d_poses_C + image_idx * 3, C3,
                          3 * sizeof(float), cudaMemcpyHostToDevice));
}

void gpu_sfm_set_registered(CudaSfMState* state, int image_idx, uint8_t value)
{
    CUDA_CHECK(cudaMemcpy(state->d_registered + image_idx, &value,
                          sizeof(uint8_t), cudaMemcpyHostToDevice));
}

void gpu_sfm_update_tracks_batch(CudaSfMState* state,
                                 const int*   tid_list,
                                 const float* xyz_data,
                                 int n)
{
    if (n <= 0) return;

    // Allocate temporary device buffers for tid_list and xyz_data.
    int*   d_tid  = nullptr;
    float* d_xyz  = nullptr;
    CUDA_CHECK(cudaMalloc(&d_tid, static_cast<size_t>(n) * sizeof(int)));
    CUDA_CHECK(cudaMalloc(&d_xyz, static_cast<size_t>(n) * 3 * sizeof(float)));

    CUDA_CHECK(cudaMemcpy(d_tid, tid_list,
                          static_cast<size_t>(n) * sizeof(int),       cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_xyz, xyz_data,
                          static_cast<size_t>(n) * 3 * sizeof(float), cudaMemcpyHostToDevice));

    const int block = 256;
    const int grid  = (n + block - 1) / block;
    kernel_scatter_track_xyz<<<grid, block>>>(
            d_tid, d_xyz, state->d_track_xyz, state->d_track_valid, n);
    CUDA_CHECK(cudaGetLastError());
    CUDA_CHECK(cudaDeviceSynchronize());

    cudaFree(d_tid);
    cudaFree(d_xyz);
}

void gpu_sfm_update_tracks_batch_prealloc(CudaSfMState* state,
                                          const int*   h_tid_list,
                                          const float* h_xyz_data,
                                          int          n,
                                          int*         d_tid_scratch,
                                          float*       d_xyz_scratch)
{
    if (n <= 0) return;
    CUDA_CHECK(cudaMemcpy(d_tid_scratch, h_tid_list,
                          static_cast<size_t>(n) * sizeof(int),       cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_xyz_scratch, h_xyz_data,
                          static_cast<size_t>(n) * 3 * sizeof(float), cudaMemcpyHostToDevice));
    const int block = 256;
    const int grid  = (n + block - 1) / block;
    kernel_scatter_track_xyz<<<grid, block>>>(
            d_tid_scratch, d_xyz_scratch, state->d_track_xyz, state->d_track_valid, n);
    CUDA_CHECK(cudaGetLastError());
    CUDA_CHECK(cudaDeviceSynchronize());
}

void gpu_sfm_set_track_valid(CudaSfMState* state, int track_idx, uint8_t value)
{
    CUDA_CHECK(cudaMemcpy(state->d_track_valid + track_idx, &value,
                          sizeof(uint8_t), cudaMemcpyHostToDevice));
}

void gpu_sfm_set_obs_valid(CudaSfMState* state, int obs_idx, uint8_t value)
{
    CUDA_CHECK(cudaMemcpy(state->d_obs_valid + obs_idx, &value,
                          sizeof(uint8_t), cudaMemcpyHostToDevice));
}

void gpu_sfm_invalidate_obs_batch(CudaSfMState* state,
                                  const int* obs_ids, int n)
{
    if (n <= 0) return;
    int* d_ids = nullptr;
    CUDA_CHECK(cudaMalloc(&d_ids, static_cast<size_t>(n) * sizeof(int)));
    CUDA_CHECK(cudaMemcpy(d_ids, obs_ids, static_cast<size_t>(n) * sizeof(int),
                          cudaMemcpyHostToDevice));
    const int block = 256;
    const int grid  = (n + block - 1) / block;
    kernel_invalidate_obs<<<grid, block>>>(d_ids, state->d_obs_valid, n);
    CUDA_CHECK(cudaGetLastError());
    CUDA_CHECK(cudaDeviceSynchronize());
    cudaFree(d_ids);
}

/// Batch-set d_track_needs_retri = 1 for the given track id list.
__global__
void kernel_mark_tracks_needs_retri(
        const int* d_tid_list,
        uint8_t*   d_track_needs_retri,
        int        n)
{
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    d_track_needs_retri[d_tid_list[i]] = 1u;
}

void gpu_sfm_mark_tracks_needs_retri(CudaSfMState* state, const int* tid_list, int n)
{
    if (!state || n <= 0) return;
    int* d_ids = nullptr;
    CUDA_CHECK(cudaMalloc(&d_ids, static_cast<size_t>(n) * sizeof(int)));
    CUDA_CHECK(cudaMemcpy(d_ids, tid_list, static_cast<size_t>(n) * sizeof(int),
                          cudaMemcpyHostToDevice));
    const int block = 256;
    const int grid  = (n + block - 1) / block;
    kernel_mark_tracks_needs_retri<<<grid, block>>>(d_ids, state->d_track_needs_retri, n);
    CUDA_CHECK(cudaGetLastError());
    CUDA_CHECK(cudaDeviceSynchronize());
    cudaFree(d_ids);
}

void gpu_sfm_clear_tracks_needs_retri(CudaSfMState* state)
{
    if (!state || state->n_tracks <= 0) return;
    CUDA_CHECK(cudaMemset(state->d_track_needs_retri, 0,
                          static_cast<size_t>(state->n_tracks) * sizeof(uint8_t)));
}

// ─────────────────────────────────────────────────────────────────────────────
// Download helpers
// ─────────────────────────────────────────────────────────────────────────────

void gpu_sfm_download_poses(const CudaSfMState* state,
                             float* poses_R,
                             float* poses_C,
                             int n_images)
{
    CUDA_CHECK(cudaMemcpy(poses_R, state->d_poses_R,
                          static_cast<size_t>(n_images) * 9 * sizeof(float), cudaMemcpyDeviceToHost));
    CUDA_CHECK(cudaMemcpy(poses_C, state->d_poses_C,
                          static_cast<size_t>(n_images) * 3 * sizeof(float), cudaMemcpyDeviceToHost));
}

void gpu_sfm_download_tracks(const CudaSfMState* state,
                              float* track_xyz,
                              int n_tracks)
{
    CUDA_CHECK(cudaMemcpy(track_xyz, state->d_track_xyz,
                          static_cast<size_t>(n_tracks) * 3 * sizeof(float), cudaMemcpyDeviceToHost));
}

void gpu_sfm_download_obs_norm(const CudaSfMState* state,
                                float* obs_xn,
                                float* obs_yn,
                                int n_obs)
{
    CUDA_CHECK(cudaMemcpy(obs_xn, state->d_obs_xn,
                          static_cast<size_t>(n_obs) * sizeof(float), cudaMemcpyDeviceToHost));
    CUDA_CHECK(cudaMemcpy(obs_yn, state->d_obs_yn,
                          static_cast<size_t>(n_obs) * sizeof(float), cudaMemcpyDeviceToHost));
}

// ─────────────────────────────────────────────────────────────────────────────
// Undistort pass
// ─────────────────────────────────────────────────────────────────────────────

void gpu_sfm_state_undistort_all(CudaSfMState* state)
{
    if (!state || state->n_obs <= 0) return;
    const int block = 256;
    const int grid  = (state->n_obs + block - 1) / block;
    kernel_undistort_all<<<grid, block>>>(
            state->d_obs_u,
            state->d_obs_v,
            state->d_obs_cam_idx,
            state->d_intrinsics,
            state->d_obs_valid,
            state->d_obs_xn,
            state->d_obs_yn,
            state->n_obs);
    CUDA_CHECK(cudaGetLastError());
    CUDA_CHECK(cudaDeviceSynchronize());
}

// ─────────────────────────────────────────────────────────────────────────────
// Per-image triangulated-count kernel (M5 GPU candidate pre-filter)
// ─────────────────────────────────────────────────────────────────────────────

__global__
void kernel_count_tri_per_image(
        const int*     d_obs_image_idx,
        const int*     d_obs_track_idx,
        const uint8_t* d_obs_valid,
        const uint8_t* d_track_valid,
        int*           d_count,  ///< [n_images] out: triangulated obs count per image
        int n_obs)
{
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n_obs) return;
    if (!d_obs_valid[i]) return;
    const int tid = d_obs_track_idx[i];
    if (tid < 0 || !d_track_valid[tid]) return;
    atomicAdd(&d_count[d_obs_image_idx[i]], 1);
}

void gpu_sfm_count_tri_per_image(const CudaSfMState* state, int* n_tri_per_image)
{
    if (!state || !n_tri_per_image || state->n_obs <= 0) return;
    const int n_obs    = state->n_obs;
    const int n_images = state->n_images;

    int* d_count = nullptr;
    CUDA_CHECK(cudaMalloc(&d_count, static_cast<size_t>(n_images) * sizeof(int)));
    CUDA_CHECK(cudaMemset(d_count, 0, static_cast<size_t>(n_images) * sizeof(int)));

    const int block = 256;
    const int grid  = (n_obs + block - 1) / block;
    kernel_count_tri_per_image<<<grid, block>>>(
            state->d_obs_image_idx, state->d_obs_track_idx,
            state->d_obs_valid,     state->d_track_valid,
            d_count, n_obs);
    CUDA_CHECK(cudaGetLastError());
    CUDA_CHECK(cudaDeviceSynchronize());

    CUDA_CHECK(cudaMemcpy(n_tri_per_image, d_count,
                          static_cast<size_t>(n_images) * sizeof(int), cudaMemcpyDeviceToHost));
    cudaFree(d_count);
}

} // namespace cuda
} // namespace insight
