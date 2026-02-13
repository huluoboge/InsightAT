#pragma once

#include "retrieval_types.h"
#include "pca_whitening.h"
#include <string>
#include <vector>

namespace insight::algorithm::retrieval {

/**
 * @brief Retrieve image pairs using VLAD visual similarity
 * 
 * Pure function: Computes VLAD vectors for all images, finds top-k similar pairs
 * based on L2 distance, and returns sorted pairs.
 * 
 * @param images Image list with feature files
 * @param options Retrieval configuration (vlad_clusters, top_k)
 * @param centroids Pre-trained k-means centroids [K x 128]
 * @param cache_dir Directory for .isat_vlad cache files (optional)
 * @param pca_model Optional PCA model for dimensionality reduction
 * @param scale_weighted Enable scale-weighted VLAD encoding
 * @param target_scale Target scale for weighting (default: 4.0)
 * @param scale_sigma Gaussian sigma for scale weighting (default: 2.0)
 * @return Image pairs sorted by visual similarity (lower distance = higher score)
 */
std::vector<ImagePair> retrieveByVLAD(
    const std::vector<ImageInfo>& images,
    const RetrievalOptions& options,
    const std::vector<float>& centroids,
    const std::string& cache_dir = "",
    const PCAModel* pca_model = nullptr,
    bool scale_weighted = false,
    float target_scale = 4.0f,
    float scale_sigma = 2.0f
);

/**
 * @brief Convert L2 distance to similarity score [0, 1]
 * 
 * Uses exponential decay: score = exp(-distance / sigma)
 * 
 * @param distance L2 distance between VLAD vectors
 * @param sigma Decay parameter (default: 1.0)
 * @return Similarity score in [0, 1]
 */
double computeVLADScore(double distance, double sigma = 1.0);

/**
 * @brief Find top-k most similar images for each query image
 * 
 * @param vlad_vectors VLAD vectors for all images [N x (K*128)]
 * @param top_k Number of top similar images per query
 * @return Pairs of (query_idx, similar_idx, distance)
 */
std::vector<std::tuple<int, int, float>> findTopKSimilar(
    const std::vector<std::vector<float>>& vlad_vectors,
    int top_k
);

}  // namespace insight::algorithm::retrieval
