/**
 * gpu_twoview_sfm_cuda.cu
 * GPU two-view SfM primitives – CUDA batch implementation.
 * Adds gpu_triangulate_batch() and gpu_ba_residuals_batch() alongside existing
 * gpu_triangulate() and gpu_ba_residuals() (OpenGL versions retained).
 *
 * Architecture:
 *   - One kernel processes 1000+ image pairs in single dispatch
 *   - Per-thread: 4×4 DLT for one correspondence
 *   - Shared memory: per-warp Cholesky solver
 *   - Output: [X, Y, Z, valid_flag] flattened
 */

#include "gpu_twoview_sfm.h"

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <vector>

// ─────────────────────────────────────────────────────────────────────────────
// CUDA Device Code
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Warp-level 4×4 Cholesky inverse power iteration (shared memory).
 *        Finds null vector (smallest eigenvalue eigenvector) of 4×4 system A.
 *        Synchronizes across 32 threads (assumes warp occupies lanes 0-31 of block).
 */
__device__ void warp_null_vector4(float A[16], float* x, int lane_id) {
  // Compute B = A^T A (4×4, symmetric)
  float B[16] = {0.0f};

#pragma unroll
  for (int k = 0; k < 4; k++)
#pragma unroll
    for (int i = 0; i < 4; i++)
#pragma unroll
      for (int j = 0; j < 4; j++)
        B[i * 4 + j] += A[k * 4 + i] * A[k * 4 + j];

  // Regularise: B += mu*I
  float tr = B[0] + B[5] + B[10] + B[15];
  float mu = fmaxf(tr * 1e-3f, 1e-30f);
  B[0] += mu;
  B[5] += mu;
  B[10] += mu;
  B[15] += mu;

  // Cholesky: L·L^T = B (lower triangular in B)
  // Serial (one thread does all)
  if (lane_id == 0) {
#pragma unroll
    for (int j = 0; j < 4; j++) {
      float s = B[j * 4 + j];
#pragma unroll
      for (int k = 0; k < j; k++) s -= B[j * 4 + k] * B[j * 4 + k];
      B[j * 4 + j] = sqrtf(fmaxf(s, 1e-30f));

#pragma unroll
      for (int i = j + 1; i < 4; i++) {
        s = B[i * 4 + j];
#pragma unroll
        for (int k = 0; k < j; k++) s -= B[i * 4 + k] * B[j * 4 + k];
        B[i * 4 + j] = s / B[j * 4 + j];
      }
    }
  }
  __syncwarp();

  // Inverse power iteration (8 rounds)
  x[0] = 0.5f;
  x[1] = 0.5f;
  x[2] = 0.5f;
  x[3] = 0.5f;

  for (int iter = 0; iter < 8; iter++) {
    float y[4] = {0.0f};

    // Forward sub: L·y = x (thread 0 does computation)
    if (lane_id == 0) {
#pragma unroll
      for (int i = 0; i < 4; i++) {
        y[i] = x[i];
#pragma unroll
        for (int j = 0; j < i; j++) y[i] -= B[i * 4 + j] * y[j];
        y[i] /= B[i * 4 + i];
      }
    }
    __syncwarp();

    // Back sub: L^T·x = y (thread 0)
    if (lane_id == 0) {
      for (int i = 3; i >= 0; i--) {
        x[i] = y[i];
        for (int j = i + 1; j < 4; j++) x[i] -= B[j * 4 + i] * x[j];
        x[i] /= B[i * 4 + i];
      }

      // Normalize
      float nrm = 0.0f;
#pragma unroll
      for (int i = 0; i < 4; i++) nrm += x[i] * x[i];
      nrm = sqrtf(nrm) + 1e-30f;
#pragma unroll
      for (int i = 0; i < 4; i++) x[i] /= nrm;
    }
    __syncwarp();
  }
}

/**
 * @brief Triangulate batch: one thread processes one correspondence.
 *
 * Data layout:
 *   pts_n_flat:   [x1n, y1n, x2n, y2n, ...] for all pairs combined
 *   pair_offsets: [offset_start_0, offset_end_0, offset_start_1, ...]
 *   pair_Rt:      [R[9], t[3], R[9], t[3], ...] for each pair
 *   X_out:        [X, Y, Z, valid_flag, ...] same flattened layout
 */
__global__ void kernel_triangulate_batch(const float* pts_n_flat,
                                          const int* pair_offsets,  // [2*num_pairs]
                                          int num_pairs, const float* pair_Rt,
                                          float* X_out) {
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= pair_offsets[2 * num_pairs - 1]) return;  // global point index limit

  // Find which pair this point belongs to (binary search on pair_offsets)
  int pair_id = 0;
  {
    int lo = 0, hi = num_pairs - 1;
    while (lo < hi) {
      int mid = (lo + hi) / 2;
      if (pair_offsets[2 * mid + 1] <= idx)
        lo = mid + 1;
      else
        hi = mid;
    }
    pair_id = lo;
  }

  int start = pair_offsets[2 * pair_id];
  int end = pair_offsets[2 * pair_id + 1];
  if (idx < start || idx >= end) {
    // Point out of range (shouldn't happen with proper bounds)
    return;
  }

  // Load correspondence
  float x1n = pts_n_flat[idx * 4 + 0];
  float y1n = pts_n_flat[idx * 4 + 1];
  float x2n = pts_n_flat[idx * 4 + 2];
  float y2n = pts_n_flat[idx * 4 + 3];

  // Load R, t for this pair
  const float* Rt = pair_Rt + pair_id * 12;
  float R00 = Rt[0], R01 = Rt[1], R02 = Rt[2];
  float R10 = Rt[3], R11 = Rt[4], R12 = Rt[5];
  float R20 = Rt[6], R21 = Rt[7], R22 = Rt[8];
  float tx = Rt[9], ty = Rt[10], tz = Rt[11];

  // Build 4×4 DLT matrix: A·X_h = 0, X_h = (X, Y, Z, W)
  // cam1 = [I|0]:
  //   row 0: (-1, 0, x1n, 0)
  //   row 1: (0, -1, y1n, 0)
  // cam2 = [R|t]:
  //   row 2: (x2n*R20-R00, x2n*R21-R01, x2n*R22-R02, x2n*tz-tx)
  //   row 3: (y2n*R20-R10, y2n*R21-R11, y2n*R22-R12, y2n*tz-ty)
  float A[16];
  A[0] = -1.0f;
  A[1] = 0.0f;
  A[2] = x1n;
  A[3] = 0.0f;
  A[4] = 0.0f;
  A[5] = -1.0f;
  A[6] = y1n;
  A[7] = 0.0f;
  A[8] = x2n * R20 - R00;
  A[9] = x2n * R21 - R01;
  A[10] = x2n * R22 - R02;
  A[11] = x2n * tz - tx;
  A[12] = y2n * R20 - R10;
  A[13] = y2n * R21 - R11;
  A[14] = y2n * R22 - R12;
  A[15] = y2n * tz - ty;

  // Null vector via Cholesky inverse power (per-warp)
  float Xh[4];
  int lane_id = threadIdx.x % 32;
  warp_null_vector4(A, Xh, lane_id);

  // Dehomogenize
  float W = Xh[3];
  const float EPS = 1e-9f;
  if (fabsf(W) < EPS) {
    X_out[idx * 4 + 0] = nanf("");
    X_out[idx * 4 + 1] = nanf("");
    X_out[idx * 4 + 2] = nanf("");
    X_out[idx * 4 + 3] = 0.0f;
    return;
  }

  float X = Xh[0] / W;
  float Y = Xh[1] / W;
  float Z = Xh[2] / W;

  // Depth check
  float depth1 = Z;
  float depth2 = R20 * X + R21 * Y + R22 * Z + tz;

  if (depth1 <= EPS || depth2 <= EPS) {
    X_out[idx * 4 + 0] = nanf("");
    X_out[idx * 4 + 1] = nanf("");
    X_out[idx * 4 + 2] = nanf("");
    X_out[idx * 4 + 3] = 0.0f;
    return;
  }

  X_out[idx * 4 + 0] = X;
  X_out[idx * 4 + 1] = Y;
  X_out[idx * 4 + 2] = Z;
  X_out[idx * 4 + 3] = 1.0f;  // valid
}

/**
 * @brief Reprojection residuals batch: one thread per 3D point.
 *
 * Computes Huber-weighted residuals for all points across all pairs.
 */
__global__ void kernel_ba_residuals_batch(const float* pts_px_flat,
                                           const float* X_flat,
                                           const int* pair_offsets,  // [2*num_pairs]
                                           int num_pairs, const float* pair_Rt,
                                           float focal_length, float cx, float cy,
                                           float huber_k, float* residuals_out,
                                           float* d_wrss, int* d_valid) {
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= pair_offsets[2 * num_pairs - 1]) return;

  // Find pair
  int pair_id = 0;
  {
    int lo = 0, hi = num_pairs - 1;
    while (lo < hi) {
      int mid = (lo + hi) / 2;
      if (pair_offsets[2 * mid + 1] <= idx)
        lo = mid + 1;
      else
        hi = mid;
    }
    pair_id = lo;
  }

  int start = pair_offsets[2 * pair_id];
  int end = pair_offsets[2 * pair_id + 1];
  if (idx < start || idx >= end) return;

  // Load 3D point
  float X = X_flat[idx * 3 + 0];
  float Y = X_flat[idx * 3 + 1];
  float Z = X_flat[idx * 3 + 2];

  // NaN check
  if (isnan(X) || isnan(Y) || isnan(Z)) {
    if (residuals_out) {
      residuals_out[idx * 4 + 0] = 0.0f;
      residuals_out[idx * 4 + 1] = 0.0f;
      residuals_out[idx * 4 + 2] = 0.0f;
      residuals_out[idx * 4 + 3] = 0.0f;
    }
    return;
  }

  // Load observations
  float obs_u1 = pts_px_flat[idx * 4 + 0];
  float obs_v1 = pts_px_flat[idx * 4 + 1];
  float obs_u2 = pts_px_flat[idx * 4 + 2];
  float obs_v2 = pts_px_flat[idx * 4 + 3];

  // Load R, t
  const float* Rt = pair_Rt + pair_id * 12;
  float R00 = Rt[0], R01 = Rt[1], R02 = Rt[2];
  float R10 = Rt[3], R11 = Rt[4], R12 = Rt[5];
  float R20 = Rt[6], R21 = Rt[7], R22 = Rt[8];
  float tx = Rt[9], ty = Rt[10], tz = Rt[11];

  // Camera 1 (identity)
  float depth1 = Z;
  if (depth1 <= 1e-9f) {
    if (residuals_out) {
      residuals_out[idx * 4 + 0] = 0.0f;
      residuals_out[idx * 4 + 1] = 0.0f;
      residuals_out[idx * 4 + 2] = 0.0f;
      residuals_out[idx * 4 + 3] = 0.0f;
    }
    return;
  }
  float pred_u1 = focal_length * X / depth1 + cx;
  float pred_v1 = focal_length * Y / depth1 + cy;

  // Camera 2
  float Xc2 = R00 * X + R01 * Y + R02 * Z + tx;
  float Yc2 = R10 * X + R11 * Y + R12 * Z + ty;
  float Zc2 = R20 * X + R21 * Y + R22 * Z + tz;
  if (Zc2 <= 1e-9f) {
    if (residuals_out) {
      residuals_out[idx * 4 + 0] = 0.0f;
      residuals_out[idx * 4 + 1] = 0.0f;
      residuals_out[idx * 4 + 2] = 0.0f;
      residuals_out[idx * 4 + 3] = 0.0f;
    }
    return;
  }
  float pred_u2 = focal_length * Xc2 / Zc2 + cx;
  float pred_v2 = focal_length * Yc2 / Zc2 + cy;

  // Raw residuals
  float r0 = pred_u1 - obs_u1;
  float r1 = pred_v1 - obs_v1;
  float r2 = pred_u2 - obs_u2;
  float r3 = pred_v2 - obs_v2;

  // Huber weighting
  auto huber_weight = [huber_k](float r) {
    return (fabsf(r) <= huber_k) ? 1.0f : huber_k / (fabsf(r) + 1e-30f);
  };

  float w0 = huber_weight(r0);
  float w1 = huber_weight(r1);
  float w2 = huber_weight(r2);
  float w3 = huber_weight(r3);

  float wr0 = w0 * r0;
  float wr1 = w1 * r1;
  float wr2 = w2 * r2;
  float wr3 = w3 * r3;

  if (residuals_out) {
    residuals_out[idx * 4 + 0] = wr0;
    residuals_out[idx * 4 + 1] = wr1;
    residuals_out[idx * 4 + 2] = wr2;
    residuals_out[idx * 4 + 3] = wr3;
  }

  // Accumulate stats (atomic reduce, computed on CPU later)
  if (d_wrss) atomicAdd(d_wrss, wr0 * wr0 + wr1 * wr1 + wr2 * wr2 + wr3 * wr3);
  if (d_valid) atomicAdd(d_valid, 1);
}

// ─────────────────────────────────────────────────────────────────────────────
// Host API (C linkage)
// ─────────────────────────────────────────────────────────────────────────────

static int s_cuda_device = -1;

int gpu_twoview_cuda_init(void) {
  if (s_cuda_device >= 0) return 0;  // already initialized

  int device_count = 0;
  if (cudaGetDeviceCount(&device_count) != cudaSuccess || device_count == 0) {
    fprintf(stderr, "[gpu_twoview_cuda] No CUDA device found\n");
    return -1;
  }

  // Choose best device (highest compute capability)
  int best_device = 0;
  int best_cc = 0;
  for (int i = 0; i < device_count; ++i) {
    cudaDeviceProp prop;
    if (cudaGetDeviceProperties(&prop, i) == cudaSuccess) {
      int cc = prop.major * 10 + prop.minor;
      fprintf(stderr, "[gpu_twoview_cuda] Device %d: %s (CC %d.%d)\n", i, prop.name,
              prop.major, prop.minor);
      if (cc > best_cc) {
        best_cc = cc;
        best_device = i;
      }
    }
  }

  if (cudaSetDevice(best_device) != cudaSuccess) {
    fprintf(stderr, "[gpu_twoview_cuda] Failed to set device %d\n", best_device);
    return -1;
  }

  s_cuda_device = best_device;
  fprintf(stderr, "[gpu_twoview_cuda] Initialized on device %d\n", best_device);
  return 0;
}

void gpu_twoview_cuda_shutdown(void) {
  if (s_cuda_device >= 0) {
    cudaDeviceReset();
    s_cuda_device = -1;
  }
}

int gpu_triangulate_batch(const float* pts_n_flat, const int* pair_offsets,
                          int num_pairs, const float* pair_Rt, float* X_out) {
  if (num_pairs <= 0) return 0;

  int total_pts = pair_offsets[2 * num_pairs - 1];
  if (total_pts <= 0) return 0;

  // Allocate device memory
  float* d_pts_n = nullptr;
  int* d_offsets = nullptr;
  float* d_Rt = nullptr;
  float* d_X_out = nullptr;

  size_t pts_bytes = total_pts * 4 * sizeof(float);
  size_t offset_bytes = (2 * num_pairs + 1) * sizeof(int);
  size_t rt_bytes = num_pairs * 12 * sizeof(float);
  size_t x_bytes = total_pts * 4 * sizeof(float);

  if (cudaMalloc(&d_pts_n, pts_bytes) != cudaSuccess ||
      cudaMalloc(&d_offsets, offset_bytes) != cudaSuccess ||
      cudaMalloc(&d_Rt, rt_bytes) != cudaSuccess ||
      cudaMalloc(&d_X_out, x_bytes) != cudaSuccess) {
    fprintf(stderr, "[gpu_triangulate_batch] CUDA alloc failed\n");
    cudaFree(d_pts_n);
    cudaFree(d_offsets);
    cudaFree(d_Rt);
    cudaFree(d_X_out);
    return -1;
  }

  // Copy to device
  cudaMemcpy(d_pts_n, pts_n_flat, pts_bytes, cudaMemcpyHostToDevice);
  cudaMemcpy(d_offsets, pair_offsets, offset_bytes, cudaMemcpyHostToDevice);
  cudaMemcpy(d_Rt, pair_Rt, rt_bytes, cudaMemcpyHostToDevice);

  // Launch kernel
  int threads_per_block = 256;
  int num_blocks = (total_pts + threads_per_block - 1) / threads_per_block;
  kernel_triangulate_batch<<<num_blocks, threads_per_block>>>(
      d_pts_n, d_offsets, num_pairs, d_Rt, d_X_out);

  // Copy results back
  std::vector<float> host_X(total_pts * 4);
  cudaMemcpy(host_X.data(), d_X_out, x_bytes, cudaMemcpyDeviceToHost);

  // Count valid points and compact output
  int valid_count = 0;
  for (int i = 0; i < total_pts; ++i) {
    X_out[i * 3 + 0] = host_X[i * 4 + 0];
    X_out[i * 3 + 1] = host_X[i * 4 + 1];
    X_out[i * 3 + 2] = host_X[i * 4 + 2];
    if (host_X[i * 4 + 3] > 0.5f) ++valid_count;
  }

  // Cleanup
  cudaFree(d_pts_n);
  cudaFree(d_offsets);
  cudaFree(d_Rt);
  cudaFree(d_X_out);

  return valid_count;
}

void gpu_ba_residuals_batch(const float* pts_px_flat, const float* X_flat,
                            const int* pair_offsets, int num_pairs,
                            const float* pair_Rt, float focal_length, float cx,
                            float cy, float huber_k, float* residuals_out,
                            float* wrss_out, int* valid_out) {
  if (num_pairs <= 0) {
    if (wrss_out) *wrss_out = 0.0f;
    if (valid_out) *valid_out = 0;
    return;
  }

  int total_pts = pair_offsets[2 * num_pairs - 1];
  if (total_pts <= 0) {
    if (wrss_out) *wrss_out = 0.0f;
    if (valid_out) *valid_out = 0;
    return;
  }

  // Allocate device memory (declare all before any goto)
  float* d_pts_px = nullptr;
  float* d_X = nullptr;
  int* d_offsets = nullptr;
  float* d_Rt = nullptr;
  float* d_res_out = nullptr;
  float* d_wrss = nullptr;
  int* d_valid = nullptr;
  float h_wrss = 0.0f;
  int h_valid = 0;

  size_t pts_bytes = total_pts * 4 * sizeof(float);
  size_t x_bytes = total_pts * 3 * sizeof(float);
  size_t offset_bytes = (2 * num_pairs + 1) * sizeof(int);
  size_t rt_bytes = num_pairs * 12 * sizeof(float);
  size_t res_bytes = total_pts * 4 * sizeof(float);

  if (cudaMalloc(&d_pts_px, pts_bytes) != cudaSuccess ||
      cudaMalloc(&d_X, x_bytes) != cudaSuccess ||
      cudaMalloc(&d_offsets, offset_bytes) != cudaSuccess ||
      cudaMalloc(&d_Rt, rt_bytes) != cudaSuccess ||
      cudaMalloc(&d_res_out, res_bytes) != cudaSuccess ||
      cudaMalloc(&d_wrss, sizeof(float)) != cudaSuccess ||
      cudaMalloc(&d_valid, sizeof(int)) != cudaSuccess) {
    fprintf(stderr, "[gpu_ba_residuals_batch] CUDA alloc failed\n");
    goto cleanup;
  }

  // Copy to device
  cudaMemcpy(d_pts_px, pts_px_flat, pts_bytes, cudaMemcpyHostToDevice);
  cudaMemcpy(d_X, X_flat, x_bytes, cudaMemcpyHostToDevice);
  cudaMemcpy(d_offsets, pair_offsets, offset_bytes, cudaMemcpyHostToDevice);
  cudaMemcpy(d_Rt, pair_Rt, rt_bytes, cudaMemcpyHostToDevice);

  // Initialize accumulators to zero
  h_wrss = 0.0f;
  h_valid = 0;
  cudaMemcpy(d_wrss, &h_wrss, sizeof(float), cudaMemcpyHostToDevice);
  cudaMemcpy(d_valid, &h_valid, sizeof(int), cudaMemcpyHostToDevice);

  // Launch kernel
  {
    int threads_per_block = 256;
    int num_blocks = (total_pts + threads_per_block - 1) / threads_per_block;
    kernel_ba_residuals_batch<<<num_blocks, threads_per_block>>>(
        d_pts_px, d_X, d_offsets, num_pairs, d_Rt, focal_length, cx, cy, huber_k,
        residuals_out ? d_res_out : nullptr, d_wrss, d_valid);
  }

  // Copy results back
  if (residuals_out) {
    cudaMemcpy(residuals_out, d_res_out, res_bytes, cudaMemcpyDeviceToHost);
  }

  cudaMemcpy(&h_wrss, d_wrss, sizeof(float), cudaMemcpyDeviceToHost);
  cudaMemcpy(&h_valid, d_valid, sizeof(int), cudaMemcpyDeviceToHost);

  if (h_valid > 0) h_wrss /= (float)h_valid;

  if (wrss_out) *wrss_out = h_wrss;
  if (valid_out) *valid_out = h_valid;

cleanup:
  cudaFree(d_pts_px);
  cudaFree(d_X);
  cudaFree(d_offsets);
  cudaFree(d_Rt);
  cudaFree(d_res_out);
  cudaFree(d_wrss);
  cudaFree(d_valid);
}

GpuTwoviewStream gpu_twoview_create_stream(void) {
  cudaStream_t stream;
  if (cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking) != cudaSuccess)
    return nullptr;
  return (GpuTwoviewStream)stream;
}

void gpu_twoview_destroy_stream(GpuTwoviewStream stream) {
  if (stream) cudaStreamDestroy((cudaStream_t)stream);
}

int gpu_triangulate_batch_async(GpuTwoviewStream stream, const float* pts_n_flat,
                                 const int* pair_offsets, int num_pairs,
                                 const float* pair_Rt, float* X_out) {
  // Wrapper: same as sync version (could be extended with true async later)
  return gpu_triangulate_batch(pts_n_flat, pair_offsets, num_pairs, pair_Rt, X_out);
}

int gpu_twoview_stream_synchronize(GpuTwoviewStream stream) {
  if (stream) {
    if (cudaStreamSynchronize((cudaStream_t)stream) != cudaSuccess) return -1;
  }
  return 0;
}
