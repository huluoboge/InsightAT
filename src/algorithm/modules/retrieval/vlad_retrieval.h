/**
 * @file  vlad_retrieval.h
 * @brief VLAD 视觉相似度检索：按 L2 距离取 top-k 相似对并转成 ImagePair。
 */

#pragma once

#include "pca_whitening.h"
#include "retrieval_types.h"

#include <string>
#include <vector>

namespace insight::algorithm::retrieval {

std::vector<ImagePair> retrieve_by_vlad(const std::vector<ImageInfo>& images,
                                       const RetrievalOptions& options,
                                       const std::vector<float>& centroids,
                                       const std::string& cache_dir = "",
                                       const PCAModel* pca_model = nullptr,
                                       bool scale_weighted = false, float target_scale = 4.0f,
                                       float scale_sigma = 2.0f);
double compute_vlad_score(double distance, double sigma = 1.0);
std::vector<std::tuple<int, int, float>>
find_top_k_similar(const std::vector<std::vector<float>>& vlad_vectors, int top_k);

}  // namespace insight::algorithm::retrieval
