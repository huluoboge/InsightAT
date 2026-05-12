/**
 * @file  cuda_ba_subset.cu
 * @brief GPU 3-pass BA subset selection kernels.
 */

#include "cuda_ba_subset.cuh"
#include "cuda_sfm_state.h"

#include <cuda_runtime.h>
#include <float.h>
#include <limits.h>

#include <cstdio>
#include <cstdlib>
#include <vector>

// ─────────────────────────────────────────────────────────────────────────────
// Internal helpers
// ─────────────────────────────────────────────────────────────────────────────

#define CUDA_CHECK_BA(call)                                                      \
    do {                                                                         \
        cudaError_t _e = (call);                                                 \
        if (_e != cudaSuccess) {                                                 \
            fprintf(stderr, "CUDA error %s:%d  %s: %s\n",                       \
                    __FILE__, __LINE__, #call, cudaGetErrorString(_e));          \
            std::abort();                                                        \
        }                                                                        \
    } while (0)

// Maximum number of grid cells per image that fit in shared memory.
// 4096 × 8 bytes (uint64) = 32 KB < 48 KB typical per-block shared memory.
static constexpr int kMaxCells = 4096;

namespace insight {
namespace cuda {

// ─────────────────────────────────────────────────────────────────────────────
// Pass 1a: per-observation — accumulate track registered degree,
//          record first/last registered image index per track (for sin²θ)
// ─────────────────────────────────────────────────────────────────────────────

__global__
void kernel_ba_pass1a_degree(
        const int*     d_obs_image_idx,
        const int*     d_obs_track_idx,
        const uint8_t* d_obs_valid,
        const uint8_t* d_registered,
        int*           d_track_degree,   ///< [n_tracks] out: registered obs count
        int*           d_track_first_im, ///< [n_tracks] out: min registered image id
        int*           d_track_last_im,  ///< [n_tracks] out: max registered image id
        int n_obs)
{
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n_obs) return;
    if (!d_obs_valid[i]) return;

    const int im  = d_obs_image_idx[i];
    if (!d_registered[im]) return;

    const int tid = d_obs_track_idx[i];
    if (tid < 0) return;

    atomicAdd(&d_track_degree[tid], 1);
    atomicMin(&d_track_first_im[tid], im);
    atomicMax(&d_track_last_im[tid], im);
}

// ─────────────────────────────────────────────────────────────────────────────
// Pass 1b: per-track — compute sin²(parallax) for degree-2 tracks
//          using first/last registered image pair
// ─────────────────────────────────────────────────────────────────────────────

__global__
void kernel_ba_pass1b_sin2(
        const int*     d_track_degree,
        const int*     d_track_first_im,
        const int*     d_track_last_im,
        const uint8_t* d_track_valid,
        const float*   d_track_xyz,   ///< [n_tracks * 3]
        const float*   d_poses_C,     ///< [n_images * 3]
        float*         d_track_sin2,  ///< [n_tracks] out; pre-filled with 1.0
        int n_tracks)
{
    const int t = blockIdx.x * blockDim.x + threadIdx.x;
    if (t >= n_tracks) return;
    if (!d_track_valid[t]) return;
    if (d_track_degree[t] != 2) return;

    const int im0 = d_track_first_im[t];
    const int im1 = d_track_last_im[t];
    if (im0 < 0 || im1 < 0 || im0 == im1) return; // degenerate

    const float Xx = d_track_xyz[t * 3 + 0];
    const float Xy = d_track_xyz[t * 3 + 1];
    const float Xz = d_track_xyz[t * 3 + 2];

    const float C0x = d_poses_C[im0 * 3 + 0];
    const float C0y = d_poses_C[im0 * 3 + 1];
    const float C0z = d_poses_C[im0 * 3 + 2];

    const float C1x = d_poses_C[im1 * 3 + 0];
    const float C1y = d_poses_C[im1 * 3 + 1];
    const float C1z = d_poses_C[im1 * 3 + 2];

    float r0x = Xx - C0x, r0y = Xy - C0y, r0z = Xz - C0z;
    float r1x = Xx - C1x, r1y = Xy - C1y, r1z = Xz - C1z;

    const float l0 = sqrtf(r0x*r0x + r0y*r0y + r0z*r0z);
    const float l1 = sqrtf(r1x*r1x + r1y*r1y + r1z*r1z);
    if (l0 < 1e-8f || l1 < 1e-8f) return;

    r0x /= l0; r0y /= l0; r0z /= l0;
    r1x /= l1; r1y /= l1; r1z /= l1;

    const float cos_a = fmaxf(-1.0f, fminf(1.0f, r0x*r1x + r0y*r1y + r0z*r1z));
    d_track_sin2[t] = 1.0f - cos_a * cos_a;
}

// ─────────────────────────────────────────────────────────────────────────────
// Pass 2: per-image block — grid-NMS selects best-scoring track per cell
// ─────────────────────────────────────────────────────────────────────────────
//
// One CUDA block per image. Shared memory holds the cell winner array.
// Each cell stores (score_uint32 << 32) | track_id as uint64, so atomicMax
// naturally selects the highest-scoring track.
//
// Shared memory size = kMaxCells * sizeof(uint64_t) = 32 KB (passed at launch).
// ─────────────────────────────────────────────────────────────────────────────

__global__
void kernel_ba_pass2_grid_nms(
        const int*     d_img_obs_ptr,
        const int*     d_img_obs_idx,
        const uint8_t* d_registered,
        const uint8_t* d_obs_valid,
        const int*     d_obs_track_idx,
        const float*   d_obs_u,
        const float*   d_obs_v,
        const int*     d_obs_cam_idx,
        const float*   d_intrinsics,      ///< [n_cameras * 9]: {fx,fy,cx,cy,...}
        const uint8_t* d_track_valid,
        const int*     d_track_degree,
        const float*   d_track_sin2,
        uint8_t*       d_track_selected,  ///< [n_tracks] out: 1 = selected
        int n_images,
        int target_per_image,
        float w_degree,
        float w_score,
        int degree_cap)
{
    extern __shared__ unsigned long long s_cell[]; // [kMaxCells]

    const int im = blockIdx.x;
    if (im >= n_images || !d_registered[im]) return;

    const int ptr0 = d_img_obs_ptr[im];
    const int ptr1 = d_img_obs_ptr[im + 1];
    if (ptr0 >= ptr1) return;

    // Derive approximate image size from first obs's camera intrinsics.
    // d_intrinsics layout: {fx, fy, cx, cy, k1, k2, k3, p1, p2} per camera.
    const int first_obs = d_img_obs_idx[ptr0];
    const int cam       = d_obs_cam_idx[first_obs];
    const float cx      = d_intrinsics[cam * 9 + 2];
    const float cy      = d_intrinsics[cam * 9 + 3];
    const int   W       = (int)(cx * 2.0f);
    const int   H       = (int)(cy * 2.0f);
    if (W <= 0 || H <= 0) return;

    // Adaptive cell size.
    const float area      = (float)W * (float)H;
    const float cell_f    = sqrtf(area / (float)max(1, target_per_image));
    const int   gridsize  = max(1, (int)ceilf(cell_f));
    const int   gw        = (W + gridsize - 1) / gridsize;
    const int   gh        = (H + gridsize - 1) / gridsize;
    const int   n_cells   = gw * gh;

    if (n_cells > kMaxCells) return; // safety guard; should not happen for typical settings

    // Initialise shared cell array (0 = empty; valid entries have score > 0).
    for (int c = threadIdx.x; c < n_cells; c += blockDim.x)
        s_cell[c] = 0ULL;
    __syncthreads();

    // Each thread processes a strided slice of the image's observations.
    for (int p = ptr0 + (int)threadIdx.x; p < ptr1; p += (int)blockDim.x) {
        const int oi = d_img_obs_idx[p];
        if (!d_obs_valid[oi]) continue;

        const int tid = d_obs_track_idx[oi];
        if (tid < 0 || !d_track_valid[tid]) continue;

        const int deg = d_track_degree[tid];
        if (deg < 2) continue;

        // Compute score.
        const float deg_contrib = fminf(1.0f, (float)deg / (float)max(1, degree_cap));
        const float score = w_degree * deg_contrib + w_score * d_track_sin2[tid];
        if (score <= 0.0f) continue;

        // Map observation pixel to grid cell.
        const float u    = d_obs_u[oi];
        const float v    = d_obs_v[oi];
        const int cx_c   = min((int)(u / (float)gridsize), gw - 1);
        const int cy_c   = min((int)(v / (float)gridsize), gh - 1);
        const int cell   = cy_c * gw + cx_c;
        if (cell < 0 || cell >= n_cells) continue;

        // Encode (score_as_uint, track_id) in 64 bits and atomicMax.
        // IEEE-754 non-negative floats compare correctly via integer representation.
        const unsigned int score_uint = __float_as_uint(score);
        const unsigned long long encoded =
            ((unsigned long long)score_uint << 32) | (unsigned long long)(unsigned int)tid;

        atomicMax(&s_cell[cell], encoded);
    }
    __syncthreads();

    // Write winning track ids to global d_track_selected.
    for (int c = threadIdx.x; c < n_cells; c += blockDim.x) {
        const unsigned long long v = s_cell[c];
        if (v == 0ULL) continue;
        const int winner = (int)(unsigned int)(v & 0xFFFFFFFFULL);
        if (winner >= 0)
            d_track_selected[winner] = 1u;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Pass 3: per-track — write d_track_flags_ba (skip = 1 if not selected)
// ─────────────────────────────────────────────────────────────────────────────

__global__
void kernel_ba_pass3_write_skip(
        const uint8_t* d_track_valid,
        const int*     d_track_degree,
        const uint8_t* d_track_selected,
        uint8_t*       d_track_flags_ba,
        int n_tracks)
{
    const int t = blockIdx.x * blockDim.x + threadIdx.x;
    if (t >= n_tracks) return;

    if (!d_track_valid[t] || d_track_degree[t] < 2) {
        // Tracks with < 2 registered obs are neither selected nor skipped.
        d_track_flags_ba[t] = 0u;
        return;
    }
    d_track_flags_ba[t] = d_track_selected[t] ? 0u : 1u;
}

// ─────────────────────────────────────────────────────────────────────────────
// Host entry point
// ─────────────────────────────────────────────────────────────────────────────

void gpu_select_ba_subset(CudaSfMState* state, const BaSubsetOptions& opts)
{
    if (!state) return;
    const int n_obs     = state->n_obs;
    const int n_tracks  = state->n_tracks;
    const int n_images  = state->n_images;
    if (n_obs <= 0 || n_tracks <= 0 || n_images <= 0) return;

    // ── Allocate per-track scratch ─────────────────────────────────────────
    int*     d_track_degree   = nullptr;
    int*     d_track_first_im = nullptr;
    int*     d_track_last_im  = nullptr;
    float*   d_track_sin2     = nullptr;
    uint8_t* d_track_selected = nullptr;

    CUDA_CHECK_BA(cudaMalloc(&d_track_degree,   static_cast<size_t>(n_tracks) * sizeof(int)));
    CUDA_CHECK_BA(cudaMalloc(&d_track_first_im, static_cast<size_t>(n_tracks) * sizeof(int)));
    CUDA_CHECK_BA(cudaMalloc(&d_track_last_im,  static_cast<size_t>(n_tracks) * sizeof(int)));
    CUDA_CHECK_BA(cudaMalloc(&d_track_sin2,     static_cast<size_t>(n_tracks) * sizeof(float)));
    CUDA_CHECK_BA(cudaMalloc(&d_track_selected, static_cast<size_t>(n_tracks) * sizeof(uint8_t)));

    CUDA_CHECK_BA(cudaMemset(d_track_degree,   0,          static_cast<size_t>(n_tracks) * sizeof(int)));
    CUDA_CHECK_BA(cudaMemset(d_track_selected, 0,          static_cast<size_t>(n_tracks) * sizeof(uint8_t)));

    // d_track_first_im: init to INT_MAX so atomicMin gives correct minimum.
    CUDA_CHECK_BA(cudaMemset(d_track_first_im, 0x7f,       static_cast<size_t>(n_tracks) * sizeof(int)));
    // d_track_last_im: init to -1 so atomicMax gives correct maximum.
    CUDA_CHECK_BA(cudaMemset(d_track_last_im,  0xff,       static_cast<size_t>(n_tracks) * sizeof(int)));
    // d_track_sin2: init to 1.0 (default for degree >= 3; overwritten for degree-2 in pass 1b).
    {
        std::vector<float> host_ones_ba(static_cast<size_t>(n_tracks), 1.0f);
        CUDA_CHECK_BA(cudaMemcpy(d_track_sin2, host_ones_ba.data(),
                                 static_cast<size_t>(n_tracks) * sizeof(float),
                                 cudaMemcpyHostToDevice));
    }

    // ── Pass 1a: per-obs — count degree, record first/last image ──────────
    {
        const int block = 256;
        const int grid  = (n_obs + block - 1) / block;
        kernel_ba_pass1a_degree<<<grid, block>>>(
                state->d_obs_image_idx, state->d_obs_track_idx,
                state->d_obs_valid,     state->d_registered,
                d_track_degree, d_track_first_im, d_track_last_im,
                n_obs);
        CUDA_CHECK_BA(cudaGetLastError());
        CUDA_CHECK_BA(cudaDeviceSynchronize());
    }

    // ── Pass 1b: per-track — sin²(parallax) for degree-2 tracks ──────────
    {
        const int block = 256;
        const int grid  = (n_tracks + block - 1) / block;
        kernel_ba_pass1b_sin2<<<grid, block>>>(
                d_track_degree, d_track_first_im, d_track_last_im,
                state->d_track_valid, state->d_track_xyz, state->d_poses_C,
                d_track_sin2, n_tracks);
        CUDA_CHECK_BA(cudaGetLastError());
        CUDA_CHECK_BA(cudaDeviceSynchronize());
    }

    // ── Pass 2: per-image block — grid-NMS ────────────────────────────────
    {
        const int block         = 256;
        const size_t smem_bytes = static_cast<size_t>(kMaxCells) * sizeof(unsigned long long);
        kernel_ba_pass2_grid_nms<<<n_images, block, smem_bytes>>>(
                state->d_img_obs_ptr, state->d_img_obs_idx,
                state->d_registered,
                state->d_obs_valid,   state->d_obs_track_idx,
                state->d_obs_u,       state->d_obs_v,
                state->d_obs_cam_idx, state->d_intrinsics,
                state->d_track_valid, d_track_degree, d_track_sin2,
                d_track_selected,
                n_images,
                opts.target_per_image,
                opts.w_degree, opts.w_score, opts.degree_cap);
        CUDA_CHECK_BA(cudaGetLastError());
        CUDA_CHECK_BA(cudaDeviceSynchronize());
    }

    // ── Pass 3: per-track — write skip flags ──────────────────────────────
    {
        const int block = 256;
        const int grid  = (n_tracks + block - 1) / block;
        kernel_ba_pass3_write_skip<<<grid, block>>>(
                state->d_track_valid, d_track_degree,
                d_track_selected, state->d_track_flags_ba,
                n_tracks);
        CUDA_CHECK_BA(cudaGetLastError());
        CUDA_CHECK_BA(cudaDeviceSynchronize());
    }

    // ── Free scratch ──────────────────────────────────────────────────────
    cudaFree(d_track_degree);
    cudaFree(d_track_first_im);
    cudaFree(d_track_last_im);
    cudaFree(d_track_sin2);
    cudaFree(d_track_selected);
}

} // namespace cuda
} // namespace insight
