/**
 * @file  pca_whitening_cuda.h
 * @brief GPU (CUDA) PCA 训练：cuBLAS + cuSOLVER；由 train_pca() 在 INSIGHTAT_USE_CUDA_PCA 时调用。
 */

#pragma once

#include <vector>

namespace insight {
namespace algorithm {
namespace retrieval {

bool train_pca_on_gpu(const float* vlad_vectors, int num_samples, int input_dim, int n_components,
                     bool whiten, std::vector<float>& mean_out, std::vector<float>& components_out,
                     std::vector<float>& explained_variance_out);

}  // namespace retrieval
}  // namespace algorithm
}  // namespace insight
