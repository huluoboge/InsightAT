/**
 * @file cuda_pnp_ransac.cu
 * @brief Batch P3P RANSAC (Lambda Twist) on undistorted normalised coords + host argmax.
 */

#include "cuda_pnp_ransac.cuh"
#include "cuda_pnp_lambdatwist_device.cuh"

#include <cuda_runtime.h>
#include <algorithm>
#include <cstring>
#include <cstdlib>
#include <cstdio>

#define CUDA_CHECK(call)                                                                 \
    do {                                                                                 \
        cudaError_t _e = (call);                                                        \
        if (_e != cudaSuccess) {                                                        \
            fprintf(stderr, "CUDA error %s:%d %s\n", __FILE__, __LINE__, cudaGetErrorString(_e)); \
            std::abort();                                                               \
        }                                                                                \
    } while (0)

namespace insight {
namespace cuda {

// ─────────────────────────────────────────────────────────────────────────────
// Hash sampling (3 distinct indices in [0, n))
// ─────────────────────────────────────────────────────────────────────────────

__device__ __forceinline__ uint32_t dev_hash_u32(uint32_t x)
{
    x ^= x >> 16;
    x *= 0x7feb352dU;
    x ^= x >> 15;
    x *= 0x846ca68bU;
    x ^= x >> 16;
    return x;
}

__device__ void dev_sample3_indices(int n, int hyp, uint32_t seed, int out_i[3])
{
    uint32_t base = seed ^ (uint32_t)(hyp + 1) * 0x9e3779b9U;
    for (int guard = 0; guard < 64; ++guard) {
        for (int t = 0; t < 3; ++t) {
            uint32_t h = dev_hash_u32(base + (uint32_t)(t * 7919U + guard * 104729U));
            out_i[t] = (n > 0) ? (int)(h % (unsigned int)n) : 0;
        }
        if (out_i[0] != out_i[1] && out_i[0] != out_i[2] && out_i[1] != out_i[2])
            return;
        base = dev_hash_u32(base + 2654435761U);
    }
}

__device__ __forceinline__ int dev_count_inliers_norm(int n, const float* xn, const float* yn, const float* xyz3,
                                                       const float* R9, const float* t3, float thr_sq)
{
    int cnt = 0;
    for (int i = 0; i < n; ++i) {
        const float Xw = xyz3[i * 3 + 0], Yw = xyz3[i * 3 + 1], Zw = xyz3[i * 3 + 2];
        const float xc = R9[0] * Xw + R9[1] * Yw + R9[2] * Zw + t3[0];
        const float yc = R9[3] * Xw + R9[4] * Yw + R9[5] * Zw + t3[1];
        const float zc = R9[6] * Xw + R9[7] * Yw + R9[8] * Zw + t3[2];
        if (zc <= 1e-5f)
            continue;
        const float invz = 1.0f / zc;
        const float xnp = xc * invz;
        const float ynp = yc * invz;
        const float du = xnp - xn[i];
        const float dv = ynp - yn[i];
        if (du * du + dv * dv <= thr_sq)
            ++cnt;
    }
    return cnt;
}

// Device copy of per-image descriptor (pointers + scalars only).
struct GpuPnpImageDescDev {
    const float* d_obs_xn = nullptr;
    const float* d_obs_yn = nullptr;
    const float* d_pts3d  = nullptr;
    int            n_obs  = 0;
    uint32_t       seed   = 0;
};

__global__ void kernel_pnp_ransac_hypotheses(const GpuPnpImageDescDev* __restrict__ d_desc,
                                             int K,
                                             int n_iter,
                                             float threshold_n_sq,
                                             float* __restrict__ d_hyp_R,
                                             float* __restrict__ d_hyp_t,
                                             int* __restrict__ d_hyp_inliers,
                                             int max_iter)
{
    const int hyp = blockIdx.x;
    const int k   = blockIdx.y;
    if (k >= K || hyp >= n_iter)
        return;
    if (threadIdx.x != 0)
        return;

    const GpuPnpImageDescDev& d = d_desc[k];
    const int n                 = d.n_obs;
    const int out_stride_R = max_iter * 9;
    const int out_stride_t = max_iter * 3;
    float* outR            = d_hyp_R + k * out_stride_R + hyp * 9;
    float* outT            = d_hyp_t + k * out_stride_t + hyp * 3;

    if (n < 6) {
        for (int j = 0; j < 9; ++j) outR[j] = 0.f;
        for (int j = 0; j < 3; ++j) outT[j] = 0.f;
        d_hyp_inliers[k * max_iter + hyp] = 0;
        return;
    }

    int id0[3];
    dev_sample3_indices(n, hyp, d.seed, id0);

    const float xn0 = d.d_obs_xn[id0[0]], yn0 = d.d_obs_yn[id0[0]];
    const float xn1 = d.d_obs_xn[id0[1]], yn1 = d.d_obs_yn[id0[1]];
    const float xn2 = d.d_obs_xn[id0[2]], yn2 = d.d_obs_yn[id0[2]];
    auto unit_bearing = [](float xn, float yn, double* bx, double* by, double* bz) {
        double nx = (double)xn, ny = (double)yn, nz = 1.0;
        double invl = 1.0 / sqrt(nx * nx + ny * ny + nz * nz);
        *bx = nx * invl;
        *by = ny * invl;
        *bz = nz * invl;
    };
    double bx0, by0, bz0, bx1, by1, bz1, bx2, by2, bz2;
    unit_bearing(xn0, yn0, &bx0, &by0, &bz0);
    unit_bearing(xn1, yn1, &bx1, &by1, &bz1);
    unit_bearing(xn2, yn2, &bx2, &by2, &bz2);

    p3p_dev::Vec3 x0(bx0, by0, bz0), x1(bx1, by1, bz1), x2(bx2, by2, bz2);

    const float* P = d.d_pts3d;
    p3p_dev::Vec3 X0(P[id0[0] * 3 + 0], P[id0[0] * 3 + 1], P[id0[0] * 3 + 2]);
    p3p_dev::Vec3 X1(P[id0[1] * 3 + 0], P[id0[1] * 3 + 1], P[id0[1] * 3 + 2]);
    p3p_dev::Vec3 X2(P[id0[2] * 3 + 0], P[id0[2] * 3 + 1], P[id0[2] * 3 + 2]);

    float Rs[4 * 9];
    float ts[4 * 3];
    int n_sol = p3p_dev::dev_p3p_lambdatwist(x0, x1, x2, X0, X1, X2, Rs, ts, 4);

    int best_inliers = 0;
    int best_sol     = 0;
    for (int s = 0; s < n_sol; ++s) {
        const float* R9 = Rs + s * 9;
        const float* t3 = ts + s * 3;
        int inliers     = dev_count_inliers_norm(n, d.d_obs_xn, d.d_obs_yn, d.d_pts3d, R9, t3, threshold_n_sq);
        if (inliers > best_inliers) {
            best_inliers = inliers;
            best_sol     = s;
        }
    }

    if (best_inliers <= 0 || n_sol <= 0) {
        for (int j = 0; j < 9; ++j) outR[j] = 0.f;
        for (int j = 0; j < 3; ++j) outT[j] = 0.f;
        d_hyp_inliers[k * max_iter + hyp] = 0;
        return;
    }

    const float* Rb = Rs + best_sol * 9;
    const float* tb = ts + best_sol * 3;
    for (int j = 0; j < 9; ++j) outR[j] = Rb[j];
    for (int j = 0; j < 3; ++j) outT[j] = tb[j];
    d_hyp_inliers[k * max_iter + hyp] = best_inliers;
}

__global__ void kernel_classify_inliers(const float* d_xn, const float* d_yn, const float* d_xyz, int n,
                                        const float* __restrict__ R9, const float* __restrict__ t3, float thr_sq,
                                        uint8_t* d_flags, int* d_inlier_count)
{
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) {
        const float Xw = d_xyz[i * 3 + 0], Yw = d_xyz[i * 3 + 1], Zw = d_xyz[i * 3 + 2];
        const float xc = R9[0] * Xw + R9[1] * Yw + R9[2] * Zw + t3[0];
        const float yc = R9[3] * Xw + R9[4] * Yw + R9[5] * Zw + t3[1];
        const float zc = R9[6] * Xw + R9[7] * Yw + R9[8] * Zw + t3[2];
        uint8_t f = 0;
        if (zc > 1e-5f) {
            const float invz = 1.0f / zc;
            const float xnp = xc * invz;
            const float ynp = yc * invz;
            const float du = xnp - d_xn[i];
            const float dv = ynp - d_yn[i];
            if (du * du + dv * dv <= thr_sq)
                f = 1;
        }
        d_flags[i] = f;
        if (f && d_inlier_count)
            atomicAdd(d_inlier_count, 1);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// GpuPnpBatchContext
// ─────────────────────────────────────────────────────────────────────────────

GpuPnpBatchContext* gpu_pnp_batch_ctx_create(int max_images, int max_iter, int max_obs_total)
{
    auto* ctx = new GpuPnpBatchContext;
    ctx->max_images    = max_images;
    ctx->max_iter      = max_iter;
    ctx->max_obs_total = max_obs_total;

    const size_t sz_hyp_R = static_cast<size_t>(max_images) * static_cast<size_t>(max_iter) * 9u * sizeof(float);
    const size_t sz_hyp_t = static_cast<size_t>(max_images) * static_cast<size_t>(max_iter) * 3u * sizeof(float);
    const size_t sz_hyp_i = static_cast<size_t>(max_images) * static_cast<size_t>(max_iter) * sizeof(int);
    const size_t sz_bestR = static_cast<size_t>(max_images) * 9u * sizeof(float);
    const size_t sz_bestt = static_cast<size_t>(max_images) * 3u * sizeof(float);
    const size_t sz_besti = static_cast<size_t>(max_images) * sizeof(int);
    const size_t sz_flags = static_cast<size_t>(max_obs_total) * sizeof(uint8_t);
    const size_t sz_pack  = static_cast<size_t>(max_obs_total) * sizeof(float);

    CUDA_CHECK(cudaMalloc(&ctx->d_hyp_R, sz_hyp_R));
    CUDA_CHECK(cudaMalloc(&ctx->d_hyp_t, sz_hyp_t));
    CUDA_CHECK(cudaMalloc(&ctx->d_hyp_inliers, sz_hyp_i));
    CUDA_CHECK(cudaMalloc(&ctx->d_best_R, sz_bestR));
    CUDA_CHECK(cudaMalloc(&ctx->d_best_t, sz_bestt));
    CUDA_CHECK(cudaMalloc(&ctx->d_best_inliers, sz_besti));
    CUDA_CHECK(cudaMalloc(&ctx->d_inlier_flags, sz_flags));
    CUDA_CHECK(cudaMalloc(&ctx->d_pack_xn, sz_pack));
    CUDA_CHECK(cudaMalloc(&ctx->d_pack_yn, sz_pack));
    CUDA_CHECK(cudaMalloc(&ctx->d_pack_xyz, sz_pack * 3u));
    CUDA_CHECK(cudaMemset(ctx->d_hyp_R, 0, sz_hyp_R));
    CUDA_CHECK(cudaMemset(ctx->d_hyp_t, 0, sz_hyp_t));
    CUDA_CHECK(cudaMemset(ctx->d_hyp_inliers, 0, sz_hyp_i));
    return ctx;
}

void gpu_pnp_batch_ctx_free(GpuPnpBatchContext* ctx)
{
    if (!ctx) return;
    cudaFree(ctx->d_hyp_R);
    cudaFree(ctx->d_hyp_t);
    cudaFree(ctx->d_hyp_inliers);
    cudaFree(ctx->d_best_R);
    cudaFree(ctx->d_best_t);
    cudaFree(ctx->d_best_inliers);
    cudaFree(ctx->d_inlier_flags);
    cudaFree(ctx->d_pack_xn);
    cudaFree(ctx->d_pack_yn);
    cudaFree(ctx->d_pack_xyz);
    delete ctx;
}

int gpu_pnp_ransac_batch(GpuPnpBatchContext* ctx, const GpuPnpImageDesc* descs, int K, int n_iter,
                         float threshold_n_sq, float* out_R, float* out_t, int* out_inliers)
{
    if (!ctx || !descs || K <= 0 || n_iter <= 0 || !out_R || !out_t || !out_inliers)
        return 0;
    if (K > ctx->max_images || n_iter > ctx->max_iter)
        return 0;

    std::vector<GpuPnpImageDescDev> hdev(static_cast<size_t>(K));
    for (int k = 0; k < K; ++k) {
        hdev[static_cast<size_t>(k)].d_obs_xn = descs[k].d_obs_xn;
        hdev[static_cast<size_t>(k)].d_obs_yn = descs[k].d_obs_yn;
        hdev[static_cast<size_t>(k)].d_pts3d  = descs[k].d_pts3d;
        hdev[static_cast<size_t>(k)].n_obs    = descs[k].n_obs;
        hdev[static_cast<size_t>(k)].seed     = descs[k].seed;
    }

    GpuPnpImageDescDev* d_desc = nullptr;
    CUDA_CHECK(cudaMalloc(&d_desc, static_cast<size_t>(K) * sizeof(GpuPnpImageDescDev)));
    CUDA_CHECK(cudaMemcpy(d_desc, hdev.data(), static_cast<size_t>(K) * sizeof(GpuPnpImageDescDev),
                          cudaMemcpyHostToDevice));

    dim3 grid(n_iter, K, 1);
    dim3 block(1, 1, 1);
    kernel_pnp_ransac_hypotheses<<<grid, block>>>(d_desc, K, n_iter, threshold_n_sq, ctx->d_hyp_R, ctx->d_hyp_t,
                                                    ctx->d_hyp_inliers, ctx->max_iter);
    CUDA_CHECK(cudaGetLastError());
    CUDA_CHECK(cudaDeviceSynchronize());

    const int M = ctx->max_iter;
    std::vector<int> hyp_inliers(static_cast<size_t>(K) * static_cast<size_t>(M));
    CUDA_CHECK(cudaMemcpy(hyp_inliers.data(), ctx->d_hyp_inliers,
                          static_cast<size_t>(K) * static_cast<size_t>(M) * sizeof(int), cudaMemcpyDeviceToHost));

    std::vector<float> hyp_R(static_cast<size_t>(K) * static_cast<size_t>(M) * 9u);
    std::vector<float> hyp_t(static_cast<size_t>(K) * static_cast<size_t>(M) * 3u);
    CUDA_CHECK(cudaMemcpy(hyp_R.data(), ctx->d_hyp_R,
                          static_cast<size_t>(K) * static_cast<size_t>(M) * 9u * sizeof(float),
                          cudaMemcpyDeviceToHost));
    CUDA_CHECK(cudaMemcpy(hyp_t.data(), ctx->d_hyp_t,
                          static_cast<size_t>(K) * static_cast<size_t>(M) * 3u * sizeof(float),
                          cudaMemcpyDeviceToHost));

    int n_ok = 0;
    for (int k = 0; k < K; ++k) {
        int best_h = 0;
        int best_n = hyp_inliers[static_cast<size_t>(k) * static_cast<size_t>(M) + 0];
        for (int h = 1; h < n_iter; ++h) {
            const int v = hyp_inliers[static_cast<size_t>(k) * static_cast<size_t>(M) + static_cast<size_t>(h)];
            if (v > best_n) {
                best_n = v;
                best_h = h;
            }
        }
        out_inliers[static_cast<size_t>(k)] = best_n;
        const size_t off = (static_cast<size_t>(k) * static_cast<size_t>(M) + static_cast<size_t>(best_h));
        const float* Rsrc = hyp_R.data() + off * 9u;
        const float* tsrc = hyp_t.data() + off * 3u;
        for (int j = 0; j < 9; ++j) out_R[static_cast<size_t>(k) * 9u + static_cast<size_t>(j)] = Rsrc[j];
        for (int j = 0; j < 3; ++j) out_t[static_cast<size_t>(k) * 3u + static_cast<size_t>(j)] = tsrc[j];
        if (best_n > 0)
            ++n_ok;
    }

    cudaFree(d_desc);
    return n_ok;
}

int gpu_pnp_classify_inliers(const float* d_obs_xn, const float* d_obs_yn, const float* d_pts3d, int n_obs,
                             const float R[9], const float t[3], float threshold_n_sq, uint8_t* d_flags)
{
    if (!d_obs_xn || !d_obs_yn || !d_pts3d || !R || !t || !d_flags || n_obs <= 0)
        return 0;
    int* d_cnt = nullptr;
    CUDA_CHECK(cudaMalloc(&d_cnt, sizeof(int)));
    CUDA_CHECK(cudaMemset(d_cnt, 0, sizeof(int)));
    float* d_R = nullptr;
    float* d_t = nullptr;
    CUDA_CHECK(cudaMalloc(&d_R, 9 * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_t, 3 * sizeof(float)));
    CUDA_CHECK(cudaMemcpy(d_R, R, 9 * sizeof(float), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_t, t, 3 * sizeof(float), cudaMemcpyHostToDevice));
    const int block = 256;
    const int grid  = (n_obs + block - 1) / block;
    kernel_classify_inliers<<<grid, block>>>(d_obs_xn, d_obs_yn, d_pts3d, n_obs, d_R, d_t, threshold_n_sq, d_flags,
                                              d_cnt);
    CUDA_CHECK(cudaGetLastError());
    int hcnt = 0;
    CUDA_CHECK(cudaMemcpy(&hcnt, d_cnt, sizeof(int), cudaMemcpyDeviceToHost));
    cudaFree(d_cnt);
    cudaFree(d_R);
    cudaFree(d_t);
    return hcnt;
}

} // namespace cuda
} // namespace insight
