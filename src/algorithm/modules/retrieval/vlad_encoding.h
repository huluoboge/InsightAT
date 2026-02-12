#pragma once

#include <vector>
#include <string>
#include <Eigen/Core>

namespace insight {
namespace algorithm {
namespace retrieval {

// ============================================================================
// VLAD Encoding Functions (Pure, Functional)
// ============================================================================

/**
 * @brief k-means cluster assignment
 * @param descriptors Input descriptors [N x D] (D=128 for SIFT)
 * @param centroids Cluster centroids [K x D]
 * @return Cluster assignment for each descriptor [N]
 */
std::vector<int> assignToClusters(
    const std::vector<float>& descriptors,
    const std::vector<float>& centroids,
    int descriptor_dim = 128
);

/**
 * @brief Train k-means codebook from descriptors
 * @param descriptors Input descriptors [N x 128]
 * @param num_clusters Number of clusters (typical: 64 or 128)
 * @param max_iterations Maximum k-means iterations (default: 100)
 * @param convergence_threshold Early stop threshold (default: 1e-4)
 * @return Centroids [K x 128] as flat vector
 */
std::vector<float> trainKMeans(
    const std::vector<float>& descriptors,
    int num_clusters,
    int max_iterations = 100,
    float convergence_threshold = 1e-4
);

/**
 * @brief Encode image features as VLAD vector
 * @param descriptors Input descriptors [N x 128]
 * @param centroids Cluster centroids [K x 128]
 * @param num_clusters Number of clusters K
 * @return VLAD vector [K x 128] as flat vector, L2-normalized
 */
std::vector<float> encodeVLAD(
    const std::vector<float>& descriptors,
    const std::vector<float>& centroids,
    int num_clusters
);

/**
 * @brief L2 normalize a vector in-place
 * @param vec Vector to normalize
 */
void normalizeL2(std::vector<float>& vec);

/**
 * @brief Compute L2 distance between two vectors
 * @param vec1 First vector
 * @param vec2 Second vector
 * @return L2 distance
 */
float computeL2Distance(
    const std::vector<float>& vec1,
    const std::vector<float>& vec2
);

/**
 * @brief Load VLAD vector from cache file
 * @param cache_path Path to .isat_vlad file
 * @return VLAD vector, empty if not found
 */
std::vector<float> loadVLADCache(const std::string& cache_path);

/**
 * @brief Save VLAD vector to cache file
 * @param cache_path Path to .isat_vlad file
 * @param vlad_vector VLAD vector to save
 * @return true on success
 */
bool saveVLADCache(
    const std::string& cache_path,
    const std::vector<float>& vlad_vector
);

/**
 * @brief Load or compute VLAD vector with caching
 * @param feature_file Path to .isat_feat file
 * @param cache_file Path to .isat_vlad cache file
 * @param centroids k-means centroids [K x 128]
 * @param num_clusters Number of clusters K
 * @param force_recompute Force recomputation even if cache exists
 * @return VLAD vector [K x 128]
 */
std::vector<float> loadOrComputeVLAD(
    const std::string& feature_file,
    const std::string& cache_file,
    const std::vector<float>& centroids,
    int num_clusters,
    bool force_recompute = false
);

}  // namespace retrieval
}  // namespace algorithm
}  // namespace insight
