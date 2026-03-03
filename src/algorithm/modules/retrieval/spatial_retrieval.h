/**
 * @file  spatial_retrieval.h
 * @brief 基于 GPS 的空间检索：k-d 树半径搜索、GNSS/IMU 过滤。
 */

#pragma once

#include "retrieval_types.h"

#include <Eigen/Core>

namespace insight::algorithm::retrieval {

inline double euclidean_distance(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
  return (a - b).norm();
}
double angle_difference(const Eigen::Vector3d& att1, const Eigen::Vector3d& att2);
double compute_spatial_score(double distance, double threshold);
std::vector<ImageInfo> filter_images_with_gnss(const std::vector<ImageInfo>& images);
std::vector<ImagePair> retrieve_by_gps(const std::vector<ImageInfo>& images,
                                      const RetrievalOptions& options);
std::vector<std::vector<size_t>> radius_search_batch(const std::vector<Eigen::Vector3d>& positions,
                                                     double radius, int max_neighbors = -1);

}  // namespace insight::algorithm::retrieval
