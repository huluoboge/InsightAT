/**
 * @file pca_whitening_cuda.cu
 * @brief GPU PCA training: covariance via cuBLAS, eigendecomposition via cuSOLVER.
 */

#include "pca_whitening_cuda.h"
#include <cublas_v2.h>
#include <cuda_runtime.h>
#include <cusolverDn.h>
#include <glog/logging.h>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace insight {
namespace algorithm {
namespace retrieval {

#define CUDA_CHECK(call)                                                                 \
  do {                                                                                   \
    cudaError_t err = (call);                                                            \
    if (err != cudaSuccess) {                                                            \
      LOG(ERROR) << "CUDA error " << (int)err << " " << cudaGetErrorString(err);         \
      return false;                                                                     \
    }                                                                                    \
  } while (0)

#define CUBLAS_CHECK(call)                                                               \
  do {                                                                                   \
    cublasStatus_t st = (call);                                                          \
    if (st != CUBLAS_STATUS_SUCCESS) {                                                   \
      LOG(ERROR) << "cuBLAS error " << (int)st;                                         \
      return false;                                                                     \
    }                                                                                    \
  } while (0)

#define CUSOLVER_CHECK(call)                                                             \
  do {                                                                                   \
    cusolverStatus_t st = (call);                                                         \
    if (st != CUSOLVER_STATUS_SUCCESS) {                                                \
      LOG(ERROR) << "cuSOLVER error " << (int)st;                                        \
      return false;                                                                     \
    }                                                                                    \
  } while (0)

__global__ void kernel_subtract_mean(float* __restrict__ X, const float* __restrict__ mean,
                                     int num_rows, int num_cols) {
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  int total = num_rows * num_cols;
  if (idx >= total) return;
  int row = idx / num_cols;
  int col = idx % num_cols;
  X[idx] -= mean[col];
}

__global__ void kernel_compute_mean(const float* __restrict__ X, float* __restrict__ mean,
                                    int num_rows, int num_cols) {
  int col = blockIdx.x * blockDim.x + threadIdx.x;
  if (col >= num_cols) return;
  float sum = 0.0f;
  for (int row = 0; row < num_rows; ++row) {
    sum += X[row * num_cols + col];
  }
  mean[col] = sum / static_cast<float>(num_rows);
}

__global__ void kernel_transpose_rowmajor_to_colmajor(const float* __restrict__ src,
                                                      float* __restrict__ dst, int n, int d) {
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  int total = n * d;
  if (idx >= total) return;
  int row = idx / d;
  int col = idx % d;
  dst[col * n + row] = src[idx];
}

bool trainPCAOnGPU(const float* vlad_vectors, int num_samples, int input_dim, int n_components,
                   bool whiten, std::vector<float>& mean_out,
                   std::vector<float>& components_out,
                   std::vector<float>& explained_variance_out) {
  (void)whiten;
  if (vlad_vectors == nullptr || num_samples <= 0 || input_dim <= 0 || n_components <= 0 ||
      n_components > input_dim) {
    return false;
  }

  const size_t n = static_cast<size_t>(num_samples);
  const size_t d = static_cast<size_t>(input_dim);
  const size_t n_d = n * d;
  const size_t d_d = d * d;

  float* d_X = nullptr;
  float* d_mean = nullptr;
  float* d_X_cm = nullptr;  // column-major [d x n] for gemm
  float* d_cov = nullptr;

  CUDA_CHECK(cudaMalloc(&d_X, n_d * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&d_mean, d * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&d_X_cm, n_d * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&d_cov, d_d * sizeof(float)));

  CUDA_CHECK(cudaMemcpy(d_X, vlad_vectors, n_d * sizeof(float), cudaMemcpyHostToDevice));

  // 1. Compute mean (one thread per column)
  kernel_compute_mean<<<(d + 255) / 256, 256>>>(d_X, d_mean, num_samples, input_dim);
  CUDA_CHECK(cudaGetLastError());
  CUDA_CHECK(cudaDeviceSynchronize());

  // 2. Center: X -= mean
  kernel_subtract_mean<<<(n_d + 255) / 256, 256>>>(d_X, d_mean, num_samples, input_dim);
  CUDA_CHECK(cudaGetLastError());
  CUDA_CHECK(cudaDeviceSynchronize());

  // 3. Transpose to column-major [d x n] for cov = X_cm * X_cm^T
  kernel_transpose_rowmajor_to_colmajor<<<(n_d + 255) / 256, 256>>>(d_X, d_X_cm,
                                                                    num_samples, input_dim);
  CUDA_CHECK(cudaGetLastError());
  CUDA_CHECK(cudaDeviceSynchronize());

  cublasHandle_t cublas_handle = nullptr;
  CUBLAS_CHECK(cublasCreate(&cublas_handle));
  float alpha = 1.0f / static_cast<float>(num_samples - 1);
  float beta = 0.0f;
  // cov = alpha * X_cm * X_cm^T  -> [d x d], column-major
  CUBLAS_CHECK(cublasSgemm(cublas_handle, CUBLAS_OP_N, CUBLAS_OP_T, input_dim, input_dim,
                           num_samples, &alpha, d_X_cm, input_dim, d_X_cm, input_dim, &beta,
                           d_cov, input_dim));
  cublasDestroy(cublas_handle);

  // 4. Symmetric eigenvalue decomposition (cuSOLVER overwrites d_cov with eigenvectors)
  cusolverDnHandle_t cusolver_handle = nullptr;
  CUSOLVER_CHECK(cusolverDnCreate(&cusolver_handle));

  int lwork = 0;
  CUSOLVER_CHECK(cusolverDnSsyevd_bufferSize(cusolver_handle, CUSOLVER_EIG_MODE_VECTOR,
                                             CUBLAS_FILL_MODE_LOWER, input_dim, d_cov,
                                             input_dim, nullptr, &lwork));
  float* d_work = nullptr;
  float* d_eigenvalues = nullptr;
  int* d_info = nullptr;
  CUDA_CHECK(cudaMalloc(&d_work, lwork * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&d_eigenvalues, input_dim * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&d_info, sizeof(int)));

  CUSOLVER_CHECK(cusolverDnSsyevd(cusolver_handle, CUSOLVER_EIG_MODE_VECTOR,
                                   CUBLAS_FILL_MODE_LOWER, input_dim, d_cov, input_dim,
                                   d_eigenvalues, d_work, lwork, d_info));
  int info_host = 0;
  CUDA_CHECK(cudaMemcpy(&info_host, d_info, sizeof(int), cudaMemcpyDeviceToHost));
  cudaFree(d_work);
  cudaFree(d_info);
  cusolverDnDestroy(cusolver_handle);
  if (info_host != 0) {
    LOG(ERROR) << "cusolverDnSsyevd failed with info=" << info_host;
    cudaFree(d_X);
    cudaFree(d_mean);
    cudaFree(d_X_cm);
    cudaFree(d_cov);
    cudaFree(d_eigenvalues);
    return false;
  }

  std::vector<float> eigenvalues_host(input_dim);
  CUDA_CHECK(cudaMemcpy(eigenvalues_host.data(), d_eigenvalues, input_dim * sizeof(float),
                        cudaMemcpyDeviceToHost));
  cudaFree(d_eigenvalues);

  // Eigenvalues are ascending: [0]=smallest, [input_dim-1]=largest.
  // We want top n_components (largest) -> indices input_dim-1, input_dim-2, ...
  // Eigenvectors in d_cov are column-major: column j = eigenvector for eigenvalue j.
  mean_out.resize(input_dim);
  components_out.resize(n_components * input_dim);
  explained_variance_out.resize(n_components);

  CUDA_CHECK(cudaMemcpy(mean_out.data(), d_mean, d * sizeof(float), cudaMemcpyDeviceToHost));

  for (int i = 0; i < n_components; ++i) {
    int idx = input_dim - 1 - i;
    explained_variance_out[i] = eigenvalues_host[idx];
    // Copy eigenvector (column idx of d_cov) to row i of components_out
    CUDA_CHECK(cudaMemcpy(components_out.data() + i * input_dim, d_cov + idx * input_dim,
                          input_dim * sizeof(float), cudaMemcpyDeviceToHost));
  }

  CUDA_CHECK(cudaFree(d_X));
  CUDA_CHECK(cudaFree(d_mean));
  CUDA_CHECK(cudaFree(d_X_cm));
  CUDA_CHECK(cudaFree(d_cov));

  float total_var = 0.0f;
  for (float v : eigenvalues_host) total_var += v;
  float retained_var = 0.0f;
  for (float v : explained_variance_out) retained_var += v;
  LOG(INFO) << "PCA (GPU) training complete: variance retained = "
            << (total_var > 1e-20f ? (retained_var / total_var * 100.0f) : 0.0f) << "%";

  return true;
}

}  // namespace retrieval
}  // namespace algorithm
}  // namespace insight
