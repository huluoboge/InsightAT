/**
 * @file  cuda_triangulation.cu
 * @brief CUDA kernel implementations for batch DLT triangulation.
 *
 * Kernel design
 * ─────────────────────────────────────────────────────────────────────────────
 *  kernel_tri_pairs
 *      Grid:  ceil(n_pair_tasks / BLOCK) × 1
 *      Block: BLOCK (256) threads
 *      Each thread handles one PairTask (track_id, obs_a, obs_b):
 *        1. dev_triangulate_2view  → X[3]
 *        2. Depth check on both seed views
 *        3. Ray angle check between seed views
 *        4. Count inliers across all N observations of the track
 *        5. Encode (inlier_count | task_idx) and atomicMax into d_best_slot[track_id]
 *        6. Write X to d_pair_xyz[task_idx*3]
 *
 *  kernel_tri_collect
 *      Grid:  ceil(n_tracks / BLOCK) × 1
 *      Block: BLOCK (256) threads
 *      Each thread handles one track: read d_best_slot, copy winner XYZ.
 *
 * Atomics encoding
 * ─────────────────────────────────────────────────────────────────────────────
 *      slot = (inlier_count << 32) | task_index
 *  Initial value = 0 (inlier_count=0). Any pair whose inlier_count >= min_inlier_views
 *  wins over the initial sentinel.  Among ties, higher task_index wins (arbitrary but
 *  deterministic).  kernel_tri_collect checks inlier_count >= min_inlier_views.
 */

#include "cuda_triangulation.cuh"
#include "cuda_sfm_common.cuh"

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <glog/logging.h>

#include <algorithm>
#include <cstring>
#include <vector>

namespace insight {
namespace cuda {

// ─────────────────────────────────────────────────────────────────────────────
// kernel_tri_pairs
// ─────────────────────────────────────────────────────────────────────────────

static constexpr int kTriBlock = 256;

__global__
void kernel_tri_pairs(
    const int*   __restrict__ d_task_track,    // [P]
    const int*   __restrict__ d_task_a,        // [P] absolute obs index of view a
    const int*   __restrict__ d_task_b,        // [P] absolute obs index of view b
    int          n_pair_tasks,
    const int*   __restrict__ d_csr_ptr,       // [T+1]
    const float* __restrict__ d_obs_xn,        // [N]
    const float* __restrict__ d_obs_yn,        // [N]
    const float* __restrict__ d_obs_u,         // [N]
    const float* __restrict__ d_obs_v,         // [N]
    const float* __restrict__ d_obs_R,         // [N*9]
    const float* __restrict__ d_obs_C,         // [N*3]
    const float* __restrict__ d_obs_K,         // [N*9]
    float        inlier_thresh_sq,             // (ransac_inlier_px)²
    float        min_angle_deg,
    float        max_angle_deg,
    float*       d_pair_xyz,                   // [P*3] output: triangulated XYZ per task
    unsigned long long* d_best_slot            // [T]  best (inlier_count << 32 | task_idx)
)
{
    const int tid = blockIdx.x * kTriBlock + threadIdx.x;
    if (tid >= n_pair_tasks) return;

    const int track_id = d_task_track[tid];
    const int oa = d_task_a[tid];
    const int ob = d_task_b[tid];

    const float* Ra = d_obs_R + oa*9,  *Ca = d_obs_C + oa*3;
    const float* Rb = d_obs_R + ob*9,  *Cb = d_obs_C + ob*3;
    const float xna = d_obs_xn[oa], yna = d_obs_yn[oa];
    const float xnb = d_obs_xn[ob], ynb = d_obs_yn[ob];

    // ── 2-view DLT ───────────────────────────────────────────────────────────
    float X[3];
    if (!dev_triangulate_2view(Ra, Ca, xna, yna, Rb, Cb, xnb, ynb, X)) return;

    // ── Depth checks for the seed pair ───────────────────────────────────────
    if (dev_depth(Ra, Ca, X[0], X[1], X[2]) <= 0.0f) return;
    if (dev_depth(Rb, Cb, X[0], X[1], X[2]) <= 0.0f) return;

    // ── Ray angle check ───────────────────────────────────────────────────────
    const float angle = dev_ray_angle_deg(Ca, Cb, X[0], X[1], X[2]);
    if (angle < min_angle_deg || angle > max_angle_deg) return;

    // ── Store XYZ for this pair task ──────────────────────────────────────────
    d_pair_xyz[tid*3+0] = X[0];
    d_pair_xyz[tid*3+1] = X[1];
    d_pair_xyz[tid*3+2] = X[2];

    // ── Count inliers across all observations of this track ───────────────────
    const int obs_begin = d_csr_ptr[track_id];
    const int obs_end   = d_csr_ptr[track_id + 1];
    int inlier_count = 0;

    for (int i = obs_begin; i < obs_end; ++i) {
        if (dev_depth(d_obs_R + i*9, d_obs_C + i*3, X[0], X[1], X[2]) <= 0.0f) continue;
        const GpuIntrinsics K = GpuIntrinsics::from_array(d_obs_K + i*9);
        const float err_sq = dev_reproj_sq_f(d_obs_R + i*9, d_obs_C + i*3, K,
                                              X[0], X[1], X[2],
                                              d_obs_u[i], d_obs_v[i]);
        if (err_sq <= inlier_thresh_sq) ++inlier_count;
    }

    // ── Update per-track best slot (atomicMax on encoded uint64) ─────────────
    // Encoding: high32 = inlier_count (maximized), low32 = task_index (tiebreak)
    const unsigned long long encoded =
        (static_cast<unsigned long long>(inlier_count) << 32) |
         static_cast<unsigned long long>(tid);
    atomicMax(&d_best_slot[track_id], encoded);
}

// ─────────────────────────────────────────────────────────────────────────────
// kernel_tri_collect
// ─────────────────────────────────────────────────────────────────────────────

__global__
void kernel_tri_collect(
    int          n_tracks,
    const unsigned long long* __restrict__ d_best_slot, // [T]
    const float* __restrict__ d_pair_xyz,               // [P*3]
    int          min_inlier_views,
    float*       d_out_xyz,                             // [T*3]
    uint8_t*     d_out_valid                            // [T]
)
{
    const int tid = blockIdx.x * kTriBlock + threadIdx.x;
    if (tid >= n_tracks) return;

    const unsigned long long slot = d_best_slot[tid];
    const int inlier_count = static_cast<int>(slot >> 32);
    const int task_idx     = static_cast<int>(slot & 0xFFFFFFFFULL);

    if (inlier_count < min_inlier_views) {
        d_out_valid[tid] = 0;
        d_out_xyz[tid*3+0] = d_out_xyz[tid*3+1] = d_out_xyz[tid*3+2] = 0.0f;
        return;
    }

    d_out_xyz[tid*3+0] = d_pair_xyz[task_idx*3+0];
    d_out_xyz[tid*3+1] = d_pair_xyz[task_idx*3+1];
    d_out_xyz[tid*3+2] = d_pair_xyz[task_idx*3+2];
    d_out_valid[tid] = 1;
}

// ─────────────────────────────────────────────────────────────────────────────
// GpuTriContext lifecycle
// ─────────────────────────────────────────────────────────────────────────────

GpuTriContext* gpu_tri_ctx_create(int max_obs, int max_tracks, int max_pairs) {
    auto* ctx = new GpuTriContext();
    ctx->max_obs    = max_obs;
    ctx->max_tracks = max_tracks;
    ctx->max_pairs  = max_pairs;

    cudaMalloc(&ctx->d_obs_xn,    static_cast<size_t>(max_obs)       * sizeof(float));
    cudaMalloc(&ctx->d_obs_yn,    static_cast<size_t>(max_obs)       * sizeof(float));
    cudaMalloc(&ctx->d_obs_u,     static_cast<size_t>(max_obs)       * sizeof(float));
    cudaMalloc(&ctx->d_obs_v,     static_cast<size_t>(max_obs)       * sizeof(float));
    cudaMalloc(&ctx->d_obs_R,     static_cast<size_t>(max_obs) * 9   * sizeof(float));
    cudaMalloc(&ctx->d_obs_C,     static_cast<size_t>(max_obs) * 3   * sizeof(float));
    cudaMalloc(&ctx->d_obs_K,     static_cast<size_t>(max_obs) * 9   * sizeof(float));
    cudaMalloc(&ctx->d_csr_ptr,   static_cast<size_t>(max_tracks + 1) * sizeof(int));
    cudaMalloc(&ctx->d_task_track, static_cast<size_t>(max_pairs)    * sizeof(int));
    cudaMalloc(&ctx->d_task_a,     static_cast<size_t>(max_pairs)    * sizeof(int));
    cudaMalloc(&ctx->d_task_b,     static_cast<size_t>(max_pairs)    * sizeof(int));
    cudaMalloc(&ctx->d_best_slot, static_cast<size_t>(max_tracks)
                                                      * sizeof(unsigned long long));
    cudaMalloc(&ctx->d_out_xyz,   static_cast<size_t>(max_tracks) * 3 * sizeof(float));
    cudaMalloc(&ctx->d_out_valid, static_cast<size_t>(max_tracks)     * sizeof(uint8_t));

    return ctx;
}

void gpu_tri_ctx_free(GpuTriContext* ctx) {
    if (!ctx) return;
    cudaFree(ctx->d_obs_xn);
    cudaFree(ctx->d_obs_yn);
    cudaFree(ctx->d_obs_u);
    cudaFree(ctx->d_obs_v);
    cudaFree(ctx->d_obs_R);
    cudaFree(ctx->d_obs_C);
    cudaFree(ctx->d_obs_K);
    cudaFree(ctx->d_csr_ptr);
    cudaFree(ctx->d_task_track);
    cudaFree(ctx->d_task_a);
    cudaFree(ctx->d_task_b);
    cudaFree(ctx->d_best_slot);
    cudaFree(ctx->d_pair_xyz);
    cudaFree(ctx->d_out_xyz);
    cudaFree(ctx->d_out_valid);
    delete ctx;
}

// ─────────────────────────────────────────────────────────────────────────────
// gpu_tri_upload
// ─────────────────────────────────────────────────────────────────────────────

void gpu_tri_upload(GpuTriContext* ctx,
                    int n_tracks, int n_obs,
                    const int*   csr_ptr,
                    const float* obs_xn,  const float* obs_yn,
                    const float* obs_u,   const float* obs_v,
                    const float* obs_R,   const float* obs_C,
                    const float* obs_K)
{
    DCHECK(ctx);
    DCHECK_LE(n_obs,    ctx->max_obs);
    DCHECK_LE(n_tracks, ctx->max_tracks);

    const size_t n = static_cast<size_t>(n_obs);
    cudaMemcpy(ctx->d_obs_xn,  obs_xn,  n         * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(ctx->d_obs_yn,  obs_yn,  n         * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(ctx->d_obs_u,   obs_u,   n         * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(ctx->d_obs_v,   obs_v,   n         * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(ctx->d_obs_R,   obs_R,   n * 9     * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(ctx->d_obs_C,   obs_C,   n * 3     * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(ctx->d_obs_K,   obs_K,   n * 9     * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(ctx->d_csr_ptr, csr_ptr, static_cast<size_t>(n_tracks + 1) * sizeof(int),
                                         cudaMemcpyHostToDevice);
}

// ─────────────────────────────────────────────────────────────────────────────
// gpu_tri_run  — build pair tasks + launch kernels + read back results
// ─────────────────────────────────────────────────────────────────────────────

int gpu_tri_run(GpuTriContext* ctx,
                int n_tracks,
                const GpuTriOptions& opt,
                float*   out_xyz,
                uint8_t* out_valid)
{
    DCHECK(ctx);
    if (n_tracks <= 0) return 0;

    // ── Read CSR back to host to enumerate pairs ──────────────────────────────
    std::vector<int> csr_host(static_cast<size_t>(n_tracks + 1));
    cudaMemcpy(csr_host.data(), ctx->d_csr_ptr,
               static_cast<size_t>(n_tracks + 1) * sizeof(int), cudaMemcpyDeviceToHost);

    // ── Build pair tasks on CPU ───────────────────────────────────────────────
    std::vector<int> h_task_track, h_task_a, h_task_b;
    const size_t reserve_cap = static_cast<size_t>(n_tracks) *
                                static_cast<size_t>(opt.max_pairs_per_track);
    h_task_track.reserve(reserve_cap);
    h_task_a.reserve(reserve_cap);
    h_task_b.reserve(reserve_cap);

    for (int t = 0; t < n_tracks; ++t) {
        const int obs_begin = csr_host[static_cast<size_t>(t)];
        const int obs_end   = csr_host[static_cast<size_t>(t + 1)];
        const int n_obs_t   = obs_end - obs_begin;
        if (n_obs_t < 2) continue;

        int n_added = 0;
        for (int a = obs_begin; a < obs_end && n_added < opt.max_pairs_per_track; ++a) {
            for (int b = a + 1; b < obs_end && n_added < opt.max_pairs_per_track; ++b) {
                h_task_track.push_back(t);
                h_task_a.push_back(a);
                h_task_b.push_back(b);
                ++n_added;
            }
        }
    }

    const int n_pairs = static_cast<int>(h_task_track.size());
    if (n_pairs == 0) {
        std::memset(out_valid, 0, static_cast<size_t>(n_tracks));
        std::memset(out_xyz,   0, static_cast<size_t>(n_tracks) * 3 * sizeof(float));
        return 0;
    }

    DCHECK_LE(n_pairs, ctx->max_pairs);

    // ── Upload pair tasks ─────────────────────────────────────────────────────
    const size_t np = static_cast<size_t>(n_pairs);
    cudaMemcpy(ctx->d_task_track, h_task_track.data(), np * sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(ctx->d_task_a,     h_task_a.data(),     np * sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(ctx->d_task_b,     h_task_b.data(),     np * sizeof(int), cudaMemcpyHostToDevice);

    // ── Grow per-pair XYZ scratch buffer if needed ───────────────────────────
    if (n_pairs > ctx->pair_xyz_capacity) {
        cudaFree(ctx->d_pair_xyz);
        cudaMalloc(&ctx->d_pair_xyz, np * 3 * sizeof(float));
        ctx->pair_xyz_capacity = n_pairs;
    }
    cudaMemset(ctx->d_pair_xyz, 0, np * 3 * sizeof(float));

    // ── Initialise best-slot array to 0 (inlier_count=0 sentinel) ────────────
    cudaMemset(ctx->d_best_slot, 0,
               static_cast<size_t>(n_tracks) * sizeof(unsigned long long));

    // ── Launch kernel_tri_pairs ───────────────────────────────────────────────
    const float inlier_thresh_sq = opt.ransac_inlier_px * opt.ransac_inlier_px;
    const int grid_pairs = (n_pairs + kTriBlock - 1) / kTriBlock;

    kernel_tri_pairs<<<grid_pairs, kTriBlock>>>(
        ctx->d_task_track, ctx->d_task_a, ctx->d_task_b, n_pairs,
        ctx->d_csr_ptr,
        ctx->d_obs_xn, ctx->d_obs_yn,
        ctx->d_obs_u,  ctx->d_obs_v,
        ctx->d_obs_R,  ctx->d_obs_C, ctx->d_obs_K,
        inlier_thresh_sq,
        opt.min_tri_angle_deg, opt.max_tri_angle_deg,
        ctx->d_pair_xyz,
        ctx->d_best_slot
    );
    {
        const cudaError_t err = cudaGetLastError();
        if (err != cudaSuccess) {
            LOG(ERROR) << "[gpu_tri] kernel_tri_pairs launch error: "
                       << cudaGetErrorString(err);
            return 0;
        }
    }

    // ── Launch kernel_tri_collect ─────────────────────────────────────────────
    const int grid_collect = (n_tracks + kTriBlock - 1) / kTriBlock;
    kernel_tri_collect<<<grid_collect, kTriBlock>>>(
        n_tracks,
        ctx->d_best_slot, ctx->d_pair_xyz,
        opt.min_inlier_views,
        ctx->d_out_xyz, ctx->d_out_valid
    );
    {
        const cudaError_t err = cudaDeviceSynchronize();
        if (err != cudaSuccess) {
            LOG(ERROR) << "[gpu_tri] kernel_tri_collect sync error: "
                       << cudaGetErrorString(err);
            return 0;
        }
    }

    // ── Copy results to host ──────────────────────────────────────────────────
    cudaMemcpy(out_xyz,   ctx->d_out_xyz,
               static_cast<size_t>(n_tracks) * 3 * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(out_valid, ctx->d_out_valid,
               static_cast<size_t>(n_tracks) * sizeof(uint8_t),   cudaMemcpyDeviceToHost);

    // ── Count and return successes ────────────────────────────────────────────
    int n_success = 0;
    for (int i = 0; i < n_tracks; ++i) n_success += out_valid[i];
    return n_success;
}

}  // namespace cuda
}  // namespace insight
