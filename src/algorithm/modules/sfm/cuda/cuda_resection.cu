/**
 * @file  cuda_resection.cu
 * @brief GPU LM pose refinement and inlier classification for image resection.
 *
 * Kernel design
 * ─────────────────────────────────────────────────────────────────────────────
 *  kernel_gn_residuals_cost
 *      Grid: ceil(n / BLOCK) × 1
 *      One thread per 3D–2D correspondence.
 *      Projects X through R,t,K; writes (ru, rv) to d_residuals, depth flag
 *      to d_valid, and accumulates squared cost via atomicAdd.
 *
 *  kernel_gn_jtj_accumulate
 *      Grid: ceil(n / BLOCK) × 1
 *      One thread per observation.
 *      Computes the full 2×6 Jacobian (chain rule: proj distortion ∘ norm ∘ cam):
 *          J = d(u,v)/d(ω,t)
 *        = diag(fx,fy) · J_dist · J_norm · J_cam
 *      where J_dist is the 2×2 Brown-Conrady Jacobian, J_norm the 2×3 division
 *      model Jacobian, J_cam = [-[Xc]×, I] (so3 + translation).
 *      Accumulates JᵀJ (6×6) and Jᵀr (6×1) into shared memory, then atomicAdd
 *      block sums into global d_JtJ / d_Jtr.
 *
 *  kernel_gn_apply_update
 *      Single-thread (block=1, thread=1).
 *      Reads delta[6] from host, applies:
 *          R_new = exp(ω) · R,    t_new = t + δt
 *
 *  kernel_inlier_classify
 *      Grid: ceil(n / BLOCK) × 1
 *      One thread per correspondence; computes squared reprojection error and
 *      writes binary inlier flag.
 *
 * Host LM loop (gpu_resection_refine)
 * ─────────────────────────────────────────────────────────────────────────────
 *  For each LM iteration:
 *    1. Launch kernel_gn_residuals_cost     → d_cost
 *    2. Launch kernel_gn_jtj_accumulate     → d_JtJ, d_Jtr
 *    3. cudaMemcpy d_cost, d_JtJ, d_Jtr → host (3 small copies, ~160 bytes total)
 *    4. Add LM damping to diagonal of JtJ on host
 *    5. Solve 6×6 system (LDLT on host, <1 µs)
 *    6. If cost improved: launch kernel_gn_apply_update, reduce λ
 *       Else: increase λ (no update kernel needed)
 *    7. Check convergence in host
 *  On convergence, copy d_R, d_t back to host.
 */

#include "cuda_resection.cuh"
#include "cuda_sfm_common.cuh"

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <Eigen/Dense>
#include <glog/logging.h>

#include <cmath>
#include <cstring>

namespace insight {
namespace cuda {

static constexpr int kReBlock = 256;

// ─────────────────────────────────────────────────────────────────────────────
// kernel_gn_residuals_cost
// ─────────────────────────────────────────────────────────────────────────────

__global__
void kernel_gn_residuals_cost(
    int          n,
    const float* __restrict__ d_pts3d,   // [n*3]
    const float* __restrict__ d_pts2d_u, // [n]
    const float* __restrict__ d_pts2d_v, // [n]
    const float* __restrict__ d_R,       // [9]
    const float* __restrict__ d_t,       // [3]
    const float* __restrict__ d_K,       // [9]
    float*       d_residuals,            // [n*2]  (ru, rv) per obs
    float*       d_valid,                // [n]    1.0 if depth > 0
    float*       d_cost                  // [1]    accumulated sum-of-squares
)
{
    const int i = blockIdx.x * kReBlock + threadIdx.x;
    if (i >= n) return;

    const GpuIntrinsics K = GpuIntrinsics::from_array(d_K);
    const float Xw = d_pts3d[i*3+0], Yw = d_pts3d[i*3+1], Zw = d_pts3d[i*3+2];

    // Xc = R * X + t   (note: t = -R*C here, not C)
    const float xc = d_R[0]*Xw + d_R[1]*Yw + d_R[2]*Zw + d_t[0];
    const float yc = d_R[3]*Xw + d_R[4]*Yw + d_R[5]*Zw + d_t[1];
    const float zc = d_R[6]*Xw + d_R[7]*Yw + d_R[8]*Zw + d_t[2];

    if (zc <= 1e-6f) {
        d_residuals[i*2+0] = 0.0f;
        d_residuals[i*2+1] = 0.0f;
        d_valid[i]         = 0.0f;
        return;
    }
    d_valid[i] = 1.0f;

    const float inv_z = 1.0f / zc;
    const float xn = xc * inv_z, yn = yc * inv_z;

    float xd, yd;
    dev_distort_f(xn, yn, K.k1, K.k2, K.k3, K.p1, K.p2, &xd, &yd);

    const float ru = K.fx * xd + K.cx - d_pts2d_u[i];
    const float rv = K.fy * yd + K.cy - d_pts2d_v[i];
    d_residuals[i*2+0] = ru;
    d_residuals[i*2+1] = rv;

    atomicAdd(d_cost, ru*ru + rv*rv);
}

// ─────────────────────────────────────────────────────────────────────────────
// kernel_gn_jtj_accumulate
// ─────────────────────────────────────────────────────────────────────────────
// Computes the full 2×6 Jacobian per observation (matching resection.cpp logic
// exactly), accumulates 6×6 JᵀJ and 6×1 Jᵀr with shared-memory block reduction.

__global__
void kernel_gn_jtj_accumulate(
    int          n,
    const float* __restrict__ d_pts3d,    // [n*3]
    const float* __restrict__ d_pts2d_u,  // [n]   (unused here but kept for symmetry)
    const float* __restrict__ d_pts2d_v,  // [n]
    const float* __restrict__ d_residuals,// [n*2]
    const float* __restrict__ d_valid,    // [n]
    const float* __restrict__ d_R,        // [9]
    const float* __restrict__ d_t,        // [3]
    const float* __restrict__ d_K,        // [9]
    float*       d_JtJ,                   // [36]  global accumulator (atomicAdd)
    float*       d_Jtr                    // [6]   global accumulator (atomicAdd)
)
{
    // Shared memory: 36 + 6 = 42 floats for block-level JtJ / Jtr
    __shared__ float s_JtJ[36];
    __shared__ float s_Jtr[6];

    // Initialise shared memory (thread 0 of block)
    if (threadIdx.x < 36) s_JtJ[threadIdx.x] = 0.0f;
    if (threadIdx.x < 6)  s_Jtr[threadIdx.x] = 0.0f;
    __syncthreads();

    const int i = blockIdx.x * kReBlock + threadIdx.x;
    if (i < n && d_valid[i] > 0.5f) {
        const GpuIntrinsics K = GpuIntrinsics::from_array(d_K);
        const float Xw = d_pts3d[i*3+0], Yw = d_pts3d[i*3+1], Zw = d_pts3d[i*3+2];

        const float xc = d_R[0]*Xw + d_R[1]*Yw + d_R[2]*Zw + d_t[0];
        const float yc = d_R[3]*Xw + d_R[4]*Yw + d_R[5]*Zw + d_t[1];
        const float zc = d_R[6]*Xw + d_R[7]*Yw + d_R[8]*Zw + d_t[2];

        const float inv_z  = 1.0f / zc;
        const float inv_z2 = inv_z * inv_z;
        const float xn = xc * inv_z, yn = yc * inv_z;
        const float r2 = xn*xn + yn*yn, r4 = r2*r2, r6 = r4*r2;

        // ── Brown-Conrady distortion Jacobian d(xd,yd)/d(xn,yn) ─────────────
        const float drad = K.k1 + 2.0f*K.k2*r2 + 3.0f*K.k3*r4;
        const float rad  = 1.0f + K.k1*r2 + K.k2*r4 + K.k3*r6;

        // matchs resection.cpp: Bentley p1/p2 convention
        const float dxd_dxn = rad + 2.0f*xn*xn*drad + 2.0f*K.p2*yn + 6.0f*K.p1*xn;
        const float dxd_dyn = 2.0f*xn*yn*drad + 2.0f*K.p2*xn + 2.0f*K.p1*yn;
        const float dyd_dxn = 2.0f*xn*yn*drad + 2.0f*K.p1*yn + 2.0f*K.p2*xn;
        const float dyd_dyn = rad + 2.0f*yn*yn*drad + 2.0f*K.p1*xn + 6.0f*K.p2*yn;

        // d(u,v)/d(xn,yn) = diag(fx,fy) * J_dist
        const float A00 = K.fx * dxd_dxn, A01 = K.fx * dxd_dyn;
        const float A10 = K.fy * dyd_dxn, A11 = K.fy * dyd_dyn;

        // d(xn,yn)/d(Xc): xn = px/pz, yn = py/pz
        // [[ inv_z,    0, -xc/zc²],
        //  [    0, inv_z, -yc/zc²]]
        const float Bn00 =  inv_z,  Bn02 = -xc * inv_z2;
        const float Bn11 =  inv_z,  Bn12 = -yc * inv_z2;

        // d(u,v)/d(Xc) = A * Bn   (2×3)
        const float Jpc00 = A00*Bn00 /*+ A01*0*/;
        const float Jpc01 = /*A00*0 +*/ A01*Bn11;
        const float Jpc02 = A00*Bn02 + A01*Bn12;
        const float Jpc10 = A10*Bn00 /*+ A11*0*/;
        const float Jpc11 = /*A10*0 +*/ A11*Bn11;
        const float Jpc12 = A10*Bn02 + A11*Bn12;

        // d(Xc)/d(ω,t): Xc = R*X + t
        //   d(Xc)/dt = I
        //   d(Xc)/dω = − [Xc]×  (so3 perturbation: Xc ≈ (I + [ω]×)*(R*X+t) )
        // [Xc]× = [[0, -zc, yc], [zc, 0, -xc], [-yc, xc, 0]]
        // J_cam = [-[Xc]×, I]  (2×6)
        // Left 3 cols: Jpc * (-[p]×)
        // Right 3 cols: Jpc

        // row 0 of J (for u):
        // J[0,0] = Jpc0 * (-[p]×)[:,0] = Jpc00*0 + Jpc01*(-zc) + Jpc02*yc
        //        = -Jpc01*zc + Jpc02*yc
        const float J00 = -Jpc01*zc + Jpc02*yc;
        const float J01 =  Jpc00*zc - Jpc02*xc;
        const float J02 = -Jpc00*yc + Jpc01*xc;
        // right 3: = Jpc0
        const float J03 = Jpc00, J04 = Jpc01, J05 = Jpc02;

        // row 1 of J (for v):
        const float J10 = -Jpc11*zc + Jpc12*yc;
        const float J11 =  Jpc10*zc - Jpc12*xc;
        const float J12 = -Jpc10*yc + Jpc11*xc;
        const float J13 = Jpc10, J14 = Jpc11, J15 = Jpc12;

        // residuals
        const float ru = d_residuals[i*2+0];
        const float rv = d_residuals[i*2+1];

        // Accumulate JᵀJ (6×6 symmetric, only upper triangle needed for solve)
        // and Jᵀr (6) into shared memory
        // JᵀJ += J[0,:]ᵀ * J[0,:] + J[1,:]ᵀ * J[1,:]
        // Jᵀr += J[0,:]ᵀ * ru + J[1,:]ᵀ * rv

        const float row0[6] = {J00, J01, J02, J03, J04, J05};
        const float row1[6] = {J10, J11, J12, J13, J14, J15};

#pragma unroll
        for (int r = 0; r < 6; ++r) {
#pragma unroll
            for (int c = 0; c < 6; ++c) {
                atomicAdd(&s_JtJ[r*6+c], row0[r]*row0[c] + row1[r]*row1[c]);
            }
            atomicAdd(&s_Jtr[r], row0[r]*ru + row1[r]*rv);
        }
    }

    __syncthreads();

    // Flush shared → global (one atomic per entry, done by first 42 threads)
    if (threadIdx.x < 36) atomicAdd(&d_JtJ[threadIdx.x], s_JtJ[threadIdx.x]);
    if (threadIdx.x < 6)  atomicAdd(&d_Jtr[threadIdx.x], s_Jtr[threadIdx.x]);
}

// ─────────────────────────────────────────────────────────────────────────────
// kernel_gn_apply_update
// ─────────────────────────────────────────────────────────────────────────────

__global__
void kernel_gn_apply_update(
    const float* __restrict__ d_delta,  // [6] — ω (3) + δt (3) from host
    float*       d_R,                   // [9] in/out
    float*       d_t                    // [3] in/out
)
{
    // Single-thread kernel — runs on thread 0 only
    if (threadIdx.x != 0 || blockIdx.x != 0) return;

    float omega[3] = {d_delta[0], d_delta[1], d_delta[2]};
    dev_so3_update(d_R, omega);

    d_t[0] += d_delta[3];
    d_t[1] += d_delta[4];
    d_t[2] += d_delta[5];
}

// ─────────────────────────────────────────────────────────────────────────────
// kernel_inlier_classify
// ─────────────────────────────────────────────────────────────────────────────

__global__
void kernel_inlier_classify(
    int          n,
    const float* __restrict__ d_pts3d,
    const float* __restrict__ d_pts2d_u,
    const float* __restrict__ d_pts2d_v,
    const float* __restrict__ d_R,
    const float* __restrict__ d_t,
    const float* __restrict__ d_K,
    float        threshold_sq,
    uint8_t*     d_inlier_flags,
    float*       d_errors_sq
)
{
    const int i = blockIdx.x * kReBlock + threadIdx.x;
    if (i >= n) return;

    const GpuIntrinsics K = GpuIntrinsics::from_array(d_K);
    const float Xw = d_pts3d[i*3+0], Yw = d_pts3d[i*3+1], Zw = d_pts3d[i*3+2];

    const float xc = d_R[0]*Xw + d_R[1]*Yw + d_R[2]*Zw + d_t[0];
    const float yc = d_R[3]*Xw + d_R[4]*Yw + d_R[5]*Zw + d_t[1];
    const float zc = d_R[6]*Xw + d_R[7]*Yw + d_R[8]*Zw + d_t[2];

    if (zc <= 1e-6f) {
        d_inlier_flags[i] = 0;
        d_errors_sq[i]    = 1e20f;
        return;
    }
    const float inv_z = 1.0f / zc;
    const float xn = xc * inv_z, yn = yc * inv_z;
    float xd, yd;
    dev_distort_f(xn, yn, K.k1, K.k2, K.k3, K.p1, K.p2, &xd, &yd);

    const float du = K.fx*xd + K.cx - d_pts2d_u[i];
    const float dv = K.fy*yd + K.cy - d_pts2d_v[i];
    const float e_sq = du*du + dv*dv;
    d_errors_sq[i]    = e_sq;
    d_inlier_flags[i] = (e_sq <= threshold_sq) ? 1u : 0u;
}

// ─────────────────────────────────────────────────────────────────────────────
// GpuResectionContext lifecycle
// ─────────────────────────────────────────────────────────────────────────────

GpuResectionContext* gpu_resection_ctx_create(int max_pts) {
    auto* ctx = new GpuResectionContext();
    ctx->max_pts = max_pts;

    cudaMalloc(&ctx->d_pts3d,        static_cast<size_t>(max_pts) * 3 * sizeof(float));
    cudaMalloc(&ctx->d_pts2d_u,      static_cast<size_t>(max_pts)     * sizeof(float));
    cudaMalloc(&ctx->d_pts2d_v,      static_cast<size_t>(max_pts)     * sizeof(float));
    cudaMalloc(&ctx->d_R,            9  * sizeof(float));
    cudaMalloc(&ctx->d_t,            3  * sizeof(float));
    cudaMalloc(&ctx->d_residuals,    static_cast<size_t>(max_pts) * 2 * sizeof(float));
    cudaMalloc(&ctx->d_valid,        static_cast<size_t>(max_pts)     * sizeof(float));
    cudaMalloc(&ctx->d_JtJ,          36 * sizeof(float));
    cudaMalloc(&ctx->d_Jtr,          6  * sizeof(float));
    cudaMalloc(&ctx->d_cost,         1  * sizeof(float));
    cudaMalloc(&ctx->d_inlier_flags, static_cast<size_t>(max_pts)     * sizeof(uint8_t));
    cudaMalloc(&ctx->d_errors_sq,    static_cast<size_t>(max_pts)     * sizeof(float));

    return ctx;
}

void gpu_resection_ctx_free(GpuResectionContext* ctx) {
    if (!ctx) return;
    cudaFree(ctx->d_pts3d);
    cudaFree(ctx->d_pts2d_u);
    cudaFree(ctx->d_pts2d_v);
    cudaFree(ctx->d_R);
    cudaFree(ctx->d_t);
    cudaFree(ctx->d_residuals);
    cudaFree(ctx->d_valid);
    cudaFree(ctx->d_JtJ);
    cudaFree(ctx->d_Jtr);
    cudaFree(ctx->d_cost);
    cudaFree(ctx->d_inlier_flags);
    cudaFree(ctx->d_errors_sq);
    delete ctx;
}

// ─────────────────────────────────────────────────────────────────────────────
// gpu_resection_upload
// ─────────────────────────────────────────────────────────────────────────────

void gpu_resection_upload(GpuResectionContext* ctx,
                          int n_pts,
                          const float* pts3d,
                          const float* pts2d_u,
                          const float* pts2d_v,
                          const float* R_init,
                          const float* t_init)
{
    DCHECK(ctx);
    DCHECK_LE(n_pts, ctx->max_pts);
    const size_t np = static_cast<size_t>(n_pts);
    cudaMemcpy(ctx->d_pts3d,   pts3d,   np * 3 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(ctx->d_pts2d_u, pts2d_u, np     * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(ctx->d_pts2d_v, pts2d_v, np     * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(ctx->d_R,       R_init,  9      * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(ctx->d_t,       t_init,  3      * sizeof(float), cudaMemcpyHostToDevice);
}

// ─────────────────────────────────────────────────────────────────────────────
// gpu_resection_refine  — host-side LM loop
// ─────────────────────────────────────────────────────────────────────────────

bool gpu_resection_refine(GpuResectionContext* ctx,
                          const float* K,         // host K[9]
                          int n_pts,
                          int max_iters,
                          float* R_out,
                          float* t_out,
                          float* rmse_px_out)
{
    DCHECK(ctx);
    if (!R_out || !t_out || n_pts <= 0) return false;

    // Upload K to a temporary device buffer (broadcast across all obs)
    float* d_K = nullptr;
    cudaMalloc(&d_K, 9 * sizeof(float));
    cudaMemcpy(d_K, K, 9 * sizeof(float), cudaMemcpyHostToDevice);

    // Delta buffer on device for applying update
    float* d_delta = nullptr;
    cudaMalloc(&d_delta, 6 * sizeof(float));

    const int grid = (n_pts + kReBlock - 1) / kReBlock;

    // ── Evaluate initial cost ────────────────────────────────────────────────
    cudaMemset(ctx->d_cost, 0, sizeof(float));
    kernel_gn_residuals_cost<<<grid, kReBlock>>>(
        n_pts, ctx->d_pts3d, ctx->d_pts2d_u, ctx->d_pts2d_v,
        ctx->d_R, ctx->d_t, d_K,
        ctx->d_residuals, ctx->d_valid, ctx->d_cost);
    cudaDeviceSynchronize();

    float prev_cost = 0.0f;
    cudaMemcpy(&prev_cost, ctx->d_cost, sizeof(float), cudaMemcpyDeviceToHost);

    float lambda = 1e-3f;
    bool  success = true;

    for (int iter = 0; iter < max_iters; ++iter) {
        // ── Accumulate JᵀJ and Jᵀr ───────────────────────────────────────────
        cudaMemset(ctx->d_JtJ,  0, 36 * sizeof(float));
        cudaMemset(ctx->d_Jtr,  0,  6 * sizeof(float));

        kernel_gn_jtj_accumulate<<<grid, kReBlock>>>(
            n_pts,
            ctx->d_pts3d, ctx->d_pts2d_u, ctx->d_pts2d_v,
            ctx->d_residuals, ctx->d_valid,
            ctx->d_R, ctx->d_t, d_K,
            ctx->d_JtJ, ctx->d_Jtr);
        cudaDeviceSynchronize();

        // ── Copy JᵀJ and Jᵀr to host ─────────────────────────────────────────
        float h_JtJ[36], h_Jtr[6];
        cudaMemcpy(h_JtJ, ctx->d_JtJ, 36 * sizeof(float), cudaMemcpyDeviceToHost);
        cudaMemcpy(h_Jtr, ctx->d_Jtr,  6 * sizeof(float), cudaMemcpyDeviceToHost);

        // ── LM damping ────────────────────────────────────────────────────────
        for (int d = 0; d < 6; ++d) h_JtJ[d*6+d] *= (1.0f + lambda);

        // ── Solve 6×6 system via Eigen LDLT on host ──────────────────────────
        Eigen::Matrix<float, 6, 6> JtJ_e;
        Eigen::Matrix<float, 6, 1> Jtr_e;
        for (int r = 0; r < 6; ++r) {
            Jtr_e(r) = h_Jtr[r];
            for (int c = 0; c < 6; ++c)
                JtJ_e(r, c) = h_JtJ[r*6+c];
        }
        const Eigen::Matrix<float, 6, 1> delta_e = JtJ_e.ldlt().solve(-Jtr_e);
        if (!delta_e.allFinite()) break;

        // ── Apply update on device ────────────────────────────────────────────
        float h_delta[6];
        for (int d = 0; d < 6; ++d) h_delta[d] = delta_e(d);
        cudaMemcpy(d_delta, h_delta, 6 * sizeof(float), cudaMemcpyHostToDevice);

        kernel_gn_apply_update<<<1, 1>>>(d_delta, ctx->d_R, ctx->d_t);

        // ── Evaluate new cost ─────────────────────────────────────────────────
        cudaMemset(ctx->d_cost, 0, sizeof(float));
        kernel_gn_residuals_cost<<<grid, kReBlock>>>(
            n_pts, ctx->d_pts3d, ctx->d_pts2d_u, ctx->d_pts2d_v,
            ctx->d_R, ctx->d_t, d_K,
            ctx->d_residuals, ctx->d_valid, ctx->d_cost);
        cudaDeviceSynchronize();

        float new_cost = 0.0f;
        cudaMemcpy(&new_cost, ctx->d_cost, sizeof(float), cudaMemcpyDeviceToHost);

        if (new_cost < prev_cost) {
            prev_cost = new_cost;
            lambda   *= 0.5f;
        } else {
            // Revert: re-apply -delta
            float h_neg_delta[6];
            for (int d = 0; d < 6; ++d) h_neg_delta[d] = -h_delta[d];
            cudaMemcpy(d_delta, h_neg_delta, 6 * sizeof(float), cudaMemcpyHostToDevice);
            kernel_gn_apply_update<<<1, 1>>>(d_delta, ctx->d_R, ctx->d_t);
            // Re-compute residuals at original pose for next iteration JtJ
            cudaMemset(ctx->d_cost, 0, sizeof(float));
            kernel_gn_residuals_cost<<<grid, kReBlock>>>(
                n_pts, ctx->d_pts3d, ctx->d_pts2d_u, ctx->d_pts2d_v,
                ctx->d_R, ctx->d_t, d_K,
                ctx->d_residuals, ctx->d_valid, ctx->d_cost);
            cudaDeviceSynchronize();
            lambda *= 2.0f;
        }

        if (lambda > 1e8f || delta_e.norm() < 1e-8f) break;
    }

    // ── Read back final pose ──────────────────────────────────────────────────
    cudaMemcpy(R_out, ctx->d_R, 9 * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(t_out, ctx->d_t, 3 * sizeof(float), cudaMemcpyDeviceToHost);

    if (rmse_px_out) {
        float cost_final = 0.0f;
        cudaMemcpy(&cost_final, ctx->d_cost, sizeof(float), cudaMemcpyDeviceToHost);
        *rmse_px_out = sqrtf(cost_final / static_cast<float>(std::max(n_pts, 1)));
    }

    cudaFree(d_K);
    cudaFree(d_delta);
    return success;
}

// ─────────────────────────────────────────────────────────────────────────────
// gpu_resection_inliers
// ─────────────────────────────────────────────────────────────────────────────

int gpu_resection_inliers(GpuResectionContext* ctx,
                          const float* K,
                          int n_pts,
                          float threshold_px,
                          const float* R,
                          const float* t,
                          uint8_t* out_flags)
{
    DCHECK(ctx);
    DCHECK_LE(n_pts, ctx->max_pts);

    // Upload pose and K
    cudaMemcpy(ctx->d_R, R, 9 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(ctx->d_t, t, 3 * sizeof(float), cudaMemcpyHostToDevice);

    float* d_K = nullptr;
    cudaMalloc(&d_K, 9 * sizeof(float));
    cudaMemcpy(d_K, K, 9 * sizeof(float), cudaMemcpyHostToDevice);

    const float thr_sq = threshold_px * threshold_px;
    const int grid     = (n_pts + kReBlock - 1) / kReBlock;

    kernel_inlier_classify<<<grid, kReBlock>>>(
        n_pts,
        ctx->d_pts3d, ctx->d_pts2d_u, ctx->d_pts2d_v,
        ctx->d_R, ctx->d_t, d_K,
        thr_sq,
        ctx->d_inlier_flags, ctx->d_errors_sq);
    cudaDeviceSynchronize();

    cudaMemcpy(out_flags, ctx->d_inlier_flags,
               static_cast<size_t>(n_pts) * sizeof(uint8_t), cudaMemcpyDeviceToHost);
    cudaFree(d_K);

    int n_inliers = 0;
    for (int i = 0; i < n_pts; ++i) n_inliers += out_flags[i];
    return n_inliers;
}

}  // namespace cuda
}  // namespace insight
