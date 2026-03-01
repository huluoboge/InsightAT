#pragma once

#include <vector>

namespace insight {
namespace algorithm {
namespace retrieval {

/**
 * @brief GPU (CUDA) PCA training using cuBLAS + cuSOLVER.
 * Called from trainPCA() when INSIGHTAT_USE_CUDA_PCA is defined.
 *
 * @param vlad_vectors  Input VLAD vectors [N x D] row-major flat
 * @param num_samples   N
 * @param input_dim     D
 * @param n_components  Target PCA dimensions
 * @param whiten        Whether to store whitening flag (variance not applied here)
 * @param mean_out      Output mean [input_dim]
 * @param components_out Output components [n_components x input_dim] row-major flat
 * @param explained_variance_out Output variance [n_components]
 * @return true if GPU path succeeded, false to fall back to CPU
 */
bool trainPCAOnGPU(
    const float* vlad_vectors,
    int num_samples,
    int input_dim,
    int n_components,
    bool whiten,
    std::vector<float>& mean_out,
    std::vector<float>& components_out,
    std::vector<float>& explained_variance_out
);

}  // namespace retrieval
}  // namespace algorithm
}  // namespace insight
