/**
 * @file  feature_distribution.h
 * @brief 特征点分布策略：网格 NMS、四叉树（ORB-SLAM 风格）等。
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include <opencv2/core.hpp>

#include "SiftGPU/SiftGPU.h"

namespace insight {
namespace modules {

/**
 * Distribution strategy type
 */
enum class DistributionStrategy {
  NONE,    // No distribution, keep all features
  GRID,    // Grid-based uniform distribution (fast O(n))
  QUADTREE // Quadtree adaptive distribution (ORB-SLAM style)
};

/**
 * Grid-based feature distribution
 *
 * Complexity: O(n) where n = number of keypoints
 * Memory: O(grid_cells)
 *
 * Strategy:
 * - Divide image into grid cells
 * - Keep top-k strongest features per cell
 * - Supports multi-orientation (same location, different angle)
 *
 * Pros:
 * - Very fast O(n)
 * - Simple implementation
 * - Predictable memory usage
 * - Good for most SfM/SLAM scenarios
 *
 * Cons:
 * - Fixed density (not adaptive to image content)
 * - May waste capacity in empty regions
 */
struct GridDistributionParams {
  int grid_size = 32;                      // Grid cell size in pixels
  int max_per_cell = 2;                    // Maximum features per grid cell
  bool keep_orientation = true;            // Keep multiple orientations at same location
  float orientation_threshold_deg = 30.0f; // Angle threshold for same orientation (degrees)
};

/**
 * Quadtree-based feature distribution (ORB-SLAM style)
 *
 * Complexity: O(n log n) due to recursive subdivision
 * Memory: O(n) for tree nodes
 *
 * Strategy:
 * - Recursively divide regions with too many features
 * - Keep strongest feature per leaf node
 * - Stops when target feature count or min cell size reached
 *
 * Pros:
 * - Adaptive to feature density
 * - Better coverage in sparse regions
 * - Good for pyramid-level uniform distribution
 *
 * Cons:
 * - Slower than grid (O(n log n) vs O(n))
 * - More complex implementation
 * - Higher memory overhead
 */
struct QuadtreeDistributionParams {
  int max_num_features = 1000;       // Target number of features
  float min_cell_size_factor = 1.0f; // Stop subdivision when cell size < this * avg_cell_size
};

/**
 * Apply grid-based distribution to SIFT keypoints
 *
 * @param keypoints Input keypoints (SiftGPU format)
 * @param image_width Image width
 * @param image_height Image height
 * @param params Grid distribution parameters
 * @return Indices of selected keypoints
 */
std::vector<size_t>
distribute_keypoints_grid(const std::vector<SiftGPU::SiftKeypoint>& keypoints, int image_width,
                          int image_height,
                          const GridDistributionParams& params = GridDistributionParams());

std::vector<size_t>
distribute_keypoints_grid(const std::vector<cv::KeyPoint>& keypoints, int image_width,
                          int image_height,
                          const GridDistributionParams& params = GridDistributionParams());

std::vector<size_t> distribute_keypoints_quadtree(
    const std::vector<cv::KeyPoint>& keypoints, int image_width, int image_height,
    const QuadtreeDistributionParams& params = QuadtreeDistributionParams());

std::vector<cv::KeyPoint>
sift_gpu_to_opencv(const std::vector<SiftGPU::SiftKeypoint>& sift_keypoints);

std::vector<size_t> get_keypoint_indices(const std::vector<cv::KeyPoint>& original,
                                         const std::vector<cv::KeyPoint>& distributed);

} // namespace modules
} // namespace insight
