/**
 * @file  retrieval_types.h
 * @brief 检索模块核心类型：GNSS/IMU/ImageInfo/ImagePair、RetrievalOptions 及工具函数声明。
 */

#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace insight::algorithm::retrieval {

// ============================================================================
// Core Data Structures
// ============================================================================

/**
 * @brief GNSS positioning data
 */
struct GNSSData {
  double x = 0.0, y = 0.0, z = 0.0;                ///< Position in project CRS
  double cov_xx = 1.0, cov_yy = 1.0, cov_zz = 1.0; ///< Diagonal covariance
  double cov_xy = 0.0, cov_xz = 0.0, cov_yz = 0.0; ///< Off-diagonal covariance
  uint8_t num_satellites = 0;
  double hdop = 0.0, vdop = 0.0;

  Eigen::Vector3d position() const { return {x, y, z}; }

  bool is_valid() const { return num_satellites > 4 && hdop > 0 && hdop < 10.0; }
};

/**
 * @brief IMU orientation data
 */
struct IMUData {
  double roll = 0.0, pitch = 0.0, yaw = 0.0; ///< Attitude in radians
  double cov_att_xx = 0.1, cov_att_yy = 0.1, cov_att_zz = 0.1;

  Eigen::Vector3d attitude() const { return {roll, pitch, yaw}; }

  bool is_valid() const {
    return std::isfinite(roll) && std::isfinite(pitch) && std::isfinite(yaw);
  }
};

/**
 * @brief Image metadata for retrieval
 *
 * image_id is a numeric identifier (e.g. from project export). Feature/match
 * filenames use it as string: {image_id}.isat_feat, {id1}_{id2}.isat_match, etc.
 */
struct ImageInfo {
  uint32_t image_id = 0;
  std::string image_path;
  std::string feature_file;
  int camera_id = 1;

  // Optional sensor data
  std::optional<GNSSData> gnss;
  std::optional<IMUData> imu;

  bool has_gnss() const { return gnss.has_value() && gnss->is_valid(); }
  bool has_imu() const { return imu.has_value() && imu->is_valid(); }
};

/**
 * @brief Image pair with retrieval metadata
 */
struct ImagePair {
  int image1_idx = -1;
  int image2_idx = -1;
  double score = 0.0; ///< Similarity score [0, 1], higher = more similar
  std::string method; ///< "gps" | "vlad" | "vocab_tree" | "sequential" | "exhaustive"

  // Optional metadata
  std::optional<double> spatial_distance;  ///< Euclidean distance in meters
  std::optional<double> visual_similarity; ///< Visual descriptor similarity
  std::optional<double> angle_difference;  ///< Orientation difference in degrees

  bool is_valid() const {
    return image1_idx >= 0 && image2_idx >= 0 && image1_idx != image2_idx && score >= 0.0;
  }
};

/**
 * @brief Retrieval configuration options
 */
struct RetrievalOptions {
  // GPS spatial retrieval
  double distance_threshold = 200.0; ///< Max distance in meters
  double angle_threshold = 45.0;     ///< Max angle difference in degrees (0 = disable)
  int max_neighbors = 50;            ///< Max neighbors per image
  bool use_imu_filter = false;       ///< Filter by IMU orientation

  // VLAD encoding
  int vlad_clusters = 64;
  int top_k = 20;

  // Vocabulary tree
  std::string vocab_file;

  // Sequential matching
  int window_size = 10;

  // General
  int max_pairs = -1; ///< -1 = unlimited
  bool verbose = false;
  double min_score = 0.01; ///< Minimum score threshold
};

// ============================================================================
// Function Types
// ============================================================================

/**
 * @brief Retrieval strategy function signature
 *
 * Pure function: takes images + options, returns pairs
 */
using RetrievalFunction =
    std::function<std::vector<ImagePair>(const std::vector<ImageInfo>&, const RetrievalOptions&)>;

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Filter pairs using predicate (generic)
 */
template <typename Pred>
std::vector<ImagePair> filter_pairs(const std::vector<ImagePair>& pairs, Pred predicate) {
  std::vector<ImagePair> result;
  result.reserve(pairs.size() / 2);
  std::copy_if(pairs.begin(), pairs.end(), std::back_inserter(result), predicate);
  return result;
}

/** @brief Sort pairs by score (descending). */
inline std::vector<ImagePair> sort_by_score(std::vector<ImagePair> pairs) {
  std::sort(pairs.begin(), pairs.end(), [](const ImagePair& a, const ImagePair& b) {
    return a.score > b.score;
  });
  return pairs;
}

/** Remove duplicate pairs and merge scores; (i,j) and (j,i) count as duplicate. */
std::vector<ImagePair> deduplicate_and_merge(std::vector<ImagePair> pairs);

/** Combine pairs from multiple strategies; optional deduplication. */
std::vector<ImagePair> combine_pairs(const std::vector<std::vector<ImagePair>>& all_pairs,
                                     bool deduplicate = true);

}  // namespace insight::algorithm::retrieval
