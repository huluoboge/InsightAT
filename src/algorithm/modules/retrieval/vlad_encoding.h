/**
 * @file  vlad_encoding.h
 * @brief VLAD 编码：k-means 训练、聚类分配、VLAD 向量计算与缓存接口。
 */

#pragma once

#include <string>
#include <vector>

namespace insight {
namespace algorithm {
namespace retrieval {

std::vector<int> assign_to_clusters(const std::vector<float>& descriptors,
                                    const std::vector<float>& centroids,
                                    int descriptor_dim = 128);
std::vector<float> train_k_means(const std::vector<float>& descriptors, int num_clusters,
                                int max_iterations = 100, float convergence_threshold = 1e-4);
std::vector<float> encode_vlad(const std::vector<float>& descriptors,
                               const std::vector<float>& centroids, int num_clusters);
float compute_scale_weight(float scale, float target_scale = 4.0f, float sigma = 2.0f);
std::vector<float> extract_scales(const std::vector<float>& keypoints);
std::vector<float> encode_vlad_scale_weighted(const std::vector<float>& descriptors,
                                              const std::vector<float>& scales,
                                              const std::vector<float>& centroids, int num_clusters,
                                              float target_scale = 4.0f, float sigma = 2.0f);
void normalize_l2(std::vector<float>& vec);
float compute_l2_distance(const std::vector<float>& vec1, const std::vector<float>& vec2);
std::vector<float> load_vlad_cache(const std::string& cache_path);
bool save_vlad_cache(const std::string& cache_path, const std::vector<float>& vlad_vector);
std::vector<float> load_or_compute_vlad(const std::string& feature_file,
                                        const std::string& cache_file,
                                        const std::vector<float>& centroids, int num_clusters,
                                        bool force_recompute = false, bool scale_weighted = false,
                                        float target_scale = 4.0f, float scale_sigma = 2.0f);

}  // namespace retrieval
}  // namespace algorithm
}  // namespace insight
