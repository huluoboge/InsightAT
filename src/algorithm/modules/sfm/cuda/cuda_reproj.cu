/**
 * @file  cuda_reproj.cu
 * @brief GPU reprojection error computation and outlier classification kernels.
 *
 * Kernels
 * ─────────────────────────────────────────────────────────────────────────────
 *  kernel_reproj_errors
 *      Grid: ceil(n_obs / BLOCK) × 1
 *      One thread per observation.
 *      Looks up the triangulated XYZ for its track, projects through the
 *      Brown-Conrady model, returns squared pixel error.
 *
 *  kernel_reproj_filter
 *      Grid: ceil(n_obs / BLOCK) × 1
 *      One thread per observation.
 *      Compares d_errors_sq[i] against threshold²; writes d_inlier_flags[i].
 *
 *  kernel_reproj_single_image
 *      Grid: ceil(n_pts / BLOCK) × 1
 *      One thread per 3D–2D correspondence.
 *      Broadcasts the same R, C, K (in constant/shared memory) across all points.
 *      Projects each point against single camera, writes error and flag in one pass.
 */

#include "cuda_reproj.cuh"
#include "cuda_sfm_common.cuh"

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <glog/logging.h>

#include <cstring>

namespace insight {
namespace cuda {

static constexpr int kReprojBlock = 256;

// ─────────────────────────────────────────────────────────────────────────────
// kernel_reproj_errors
// ─────────────────────────────────────────────────────────────────────────────

__global__
void kernel_reproj_errors(
    int          n_obs,
    const float* __restrict__ d_track_xyz,     // [n_tracks*3]
    const int*   __restrict__ d_obs_track_id,  // [n_obs]
    const float* __restrict__ d_obs_u,         // [n_obs]
    const float* __restrict__ d_obs_v,         // [n_obs]
    const float* __restrict__ d_obs_R,         // [n_obs*9]
    const float* __restrict__ d_obs_C,         // [n_obs*3]
    const float* __restrict__ d_obs_K,         // [n_obs*9]
    float*       d_errors_sq                   // [n_obs] output
)
{
    const int i = blockIdx.x * kReprojBlock + threadIdx.x;
    if (i >= n_obs) return;

    const int   tid = d_obs_track_id[i];
    const float Xw  = d_track_xyz[tid*3+0];
    const float Yw  = d_track_xyz[tid*3+1];
    const float Zw  = d_track_xyz[tid*3+2];

    const GpuIntrinsics K = GpuIntrinsics::from_array(d_obs_K + i*9);
    d_errors_sq[i] = dev_reproj_sq_f(d_obs_R + i*9, d_obs_C + i*3, K,
                                      Xw, Yw, Zw,
                                      d_obs_u[i], d_obs_v[i]);
}

// ─────────────────────────────────────────────────────────────────────────────
// kernel_reproj_filter
// ─────────────────────────────────────────────────────────────────────────────

__global__
void kernel_reproj_filter(
    int          n_obs,
    const float* __restrict__ d_errors_sq,
    float        threshold_sq,
    uint8_t*     d_inlier_flags
)
{
    const int i = blockIdx.x * kReprojBlock + threadIdx.x;
    if (i >= n_obs) return;
    d_inlier_flags[i] = (d_errors_sq[i] <= threshold_sq) ? 1u : 0u;
}

// ─────────────────────────────────────────────────────────────────────────────
// kernel_reproj_single  — broadcast a single pose to all points
// ─────────────────────────────────────────────────────────────────────────────

__global__
void kernel_reproj_single(
    int          n_pts,
    const float* __restrict__ d_pts3d,    // [n_pts*3]
    const float* __restrict__ d_pts2d_u,  // [n_pts]
    const float* __restrict__ d_pts2d_v,  // [n_pts]
    const float* __restrict__ d_R,        // [9]  broadcast rotation
    const float* __restrict__ d_C,        // [3]  broadcast centre
    const float* __restrict__ d_K,        // [9]  broadcast intrinsics
    float        threshold_sq,
    float*       d_errors_sq,             // [n_pts] output
    uint8_t*     d_inlier_flags           // [n_pts] output
)
{
    // Load shared R, C, K — small broadcast: store in registers for this block
    // (compiler will cache in L1 due to __restrict__ aliasing hint)
    const int i = blockIdx.x * kReprojBlock + threadIdx.x;
    if (i >= n_pts) return;

    const GpuIntrinsics K = GpuIntrinsics::from_array(d_K);
    const float Xw = d_pts3d[i*3+0], Yw = d_pts3d[i*3+1], Zw = d_pts3d[i*3+2];
    const float e_sq = dev_reproj_sq_f(d_R, d_C, K,
                                        Xw, Yw, Zw,
                                        d_pts2d_u[i], d_pts2d_v[i]);
    d_errors_sq[i]    = e_sq;
    d_inlier_flags[i] = (e_sq <= threshold_sq) ? 1u : 0u;
}

// ─────────────────────────────────────────────────────────────────────────────
// GpuReprojContext lifecycle
// ─────────────────────────────────────────────────────────────────────────────

GpuReprojContext* gpu_reproj_ctx_create(int max_obs, int max_tracks) {
    auto* ctx = new GpuReprojContext();
    ctx->max_obs    = max_obs;
    ctx->max_tracks = max_tracks;

    cudaMalloc(&ctx->d_track_xyz,    static_cast<size_t>(max_tracks) * 3 * sizeof(float));
    cudaMalloc(&ctx->d_obs_track_id, static_cast<size_t>(max_obs)       * sizeof(int));
    cudaMalloc(&ctx->d_obs_u,        static_cast<size_t>(max_obs)       * sizeof(float));
    cudaMalloc(&ctx->d_obs_v,        static_cast<size_t>(max_obs)       * sizeof(float));
    cudaMalloc(&ctx->d_obs_R,        static_cast<size_t>(max_obs) * 9   * sizeof(float));
    cudaMalloc(&ctx->d_obs_C,        static_cast<size_t>(max_obs) * 3   * sizeof(float));
    cudaMalloc(&ctx->d_obs_K,        static_cast<size_t>(max_obs) * 9   * sizeof(float));
    cudaMalloc(&ctx->d_errors_sq,    static_cast<size_t>(max_obs)       * sizeof(float));
    cudaMalloc(&ctx->d_inlier_flags, static_cast<size_t>(max_obs)       * sizeof(uint8_t));

    return ctx;
}

void gpu_reproj_ctx_free(GpuReprojContext* ctx) {
    if (!ctx) return;
    cudaFree(ctx->d_track_xyz);
    cudaFree(ctx->d_obs_track_id);
    cudaFree(ctx->d_obs_u);
    cudaFree(ctx->d_obs_v);
    cudaFree(ctx->d_obs_R);
    cudaFree(ctx->d_obs_C);
    cudaFree(ctx->d_obs_K);
    cudaFree(ctx->d_errors_sq);
    cudaFree(ctx->d_inlier_flags);
    delete ctx;
}

// ─────────────────────────────────────────────────────────────────────────────
// gpu_reproj_compute
// ─────────────────────────────────────────────────────────────────────────────

void gpu_reproj_compute(GpuReprojContext* ctx,
                        int n_obs, int n_tracks,
                        const float* track_xyz,
                        const int*   obs_track_id,
                        const float* obs_u,   const float* obs_v,
                        const float* obs_R,   const float* obs_C,
                        const float* obs_K)
{
    DCHECK(ctx);
    DCHECK_LE(n_obs,    ctx->max_obs);
    DCHECK_LE(n_tracks, ctx->max_tracks);

    const size_t no = static_cast<size_t>(n_obs);
    cudaMemcpy(ctx->d_track_xyz,    track_xyz,    static_cast<size_t>(n_tracks) * 3 * sizeof(float),
                                                   cudaMemcpyHostToDevice);
    cudaMemcpy(ctx->d_obs_track_id, obs_track_id, no       * sizeof(int),   cudaMemcpyHostToDevice);
    cudaMemcpy(ctx->d_obs_u,        obs_u,         no       * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(ctx->d_obs_v,        obs_v,         no       * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(ctx->d_obs_R,        obs_R,         no * 9   * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(ctx->d_obs_C,        obs_C,         no * 3   * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(ctx->d_obs_K,        obs_K,         no * 9   * sizeof(float), cudaMemcpyHostToDevice);

    const int grid = (n_obs + kReprojBlock - 1) / kReprojBlock;
    kernel_reproj_errors<<<grid, kReprojBlock>>>(
        n_obs,
        ctx->d_track_xyz, ctx->d_obs_track_id,
        ctx->d_obs_u, ctx->d_obs_v,
        ctx->d_obs_R, ctx->d_obs_C, ctx->d_obs_K,
        ctx->d_errors_sq
    );
    const cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess)
        LOG(ERROR) << "[gpu_reproj] kernel_reproj_errors: " << cudaGetErrorString(err);
}

// ─────────────────────────────────────────────────────────────────────────────
// gpu_reproj_filter
// ─────────────────────────────────────────────────────────────────────────────

int gpu_reproj_filter(GpuReprojContext* ctx,
                      float threshold_px,
                      int n_obs,
                      uint8_t* out_flags_host)
{
    DCHECK(ctx);
    const float thr_sq = threshold_px * threshold_px;
    const int   grid   = (n_obs + kReprojBlock - 1) / kReprojBlock;

    kernel_reproj_filter<<<grid, kReprojBlock>>>(
        n_obs, ctx->d_errors_sq, thr_sq, ctx->d_inlier_flags);

    if (out_flags_host) {
        cudaDeviceSynchronize();
        cudaMemcpy(out_flags_host, ctx->d_inlier_flags,
                   static_cast<size_t>(n_obs) * sizeof(uint8_t), cudaMemcpyDeviceToHost);
        int n_inliers = 0;
        for (int i = 0; i < n_obs; ++i) n_inliers += out_flags_host[i];
        return n_inliers;
    }
    return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// gpu_reproj_single_image
// ─────────────────────────────────────────────────────────────────────────────

void gpu_reproj_single_image(GpuReprojContext* ctx,
                             const float* R, const float* C, const float* K,
                             const float* pts3d,
                             const float* pts2d_u, const float* pts2d_v,
                             int n_pts,
                             float threshold_px,
                             uint8_t* out_flags_host)
{
    DCHECK(ctx);
    DCHECK_LE(n_pts, ctx->max_obs);

    const size_t np = static_cast<size_t>(n_pts);

    // Reuse obs_R/C/K slots for pts3d and the single pose
    cudaMemcpy(ctx->d_obs_u,       pts2d_u, np       * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(ctx->d_obs_v,       pts2d_v, np       * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(ctx->d_track_xyz,   pts3d,   np * 3   * sizeof(float), cudaMemcpyHostToDevice);

    // Upload single pose into the first slots of R/C/K (broadcast pattern)
    float* d_R = nullptr; float* d_C = nullptr; float* d_K_ptr = nullptr;
    cudaMalloc(&d_R,     9 * sizeof(float));
    cudaMalloc(&d_C,     3 * sizeof(float));
    cudaMalloc(&d_K_ptr, 9 * sizeof(float));
    cudaMemcpy(d_R,     R, 9 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_C,     C, 3 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_K_ptr, K, 9 * sizeof(float), cudaMemcpyHostToDevice);

    const float thr_sq = threshold_px * threshold_px;
    const int   grid   = (n_pts + kReprojBlock - 1) / kReprojBlock;

    kernel_reproj_single<<<grid, kReprojBlock>>>(
        n_pts,
        ctx->d_track_xyz,
        ctx->d_obs_u, ctx->d_obs_v,
        d_R, d_C, d_K_ptr,
        thr_sq,
        ctx->d_errors_sq,
        ctx->d_inlier_flags
    );
    cudaDeviceSynchronize();
    cudaMemcpy(out_flags_host, ctx->d_inlier_flags, np * sizeof(uint8_t), cudaMemcpyDeviceToHost);

    cudaFree(d_R);
    cudaFree(d_C);
    cudaFree(d_K_ptr);
}

}  // namespace cuda
}  // namespace insight
