#pragma once

#include "retrieval_types.h"
#include <Eigen/Core>

namespace insight::algorithm::retrieval {

/**
 * @brief GPS-based spatial retrieval using k-d tree
 * 
 * Functional interface - all functions are pure (no side effects)
 */

/**
 * @brief Compute Euclidean distance between two positions
 */
inline double euclideanDistance(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    return (a - b).norm();
}

/**
 * @brief Compute angle difference between two orientations (degrees)
 */
double angleDifference(const Eigen::Vector3d& att1, const Eigen::Vector3d& att2);

/**
 * @brief Compute spatial score from distance (0-1, higher = closer)
 * 
 * Uses exponential decay: score = exp(-distance / threshold)
 */
double computeSpatialScore(double distance, double threshold);

/**
 * @brief Filter images that have valid GNSS data
 */
std::vector<ImageInfo> filterImagesWithGNSS(const std::vector<ImageInfo>& images);

/**
 * @brief GPS-based spatial retrieval (main function)
 * 
 * @param images All images (some may not have GNSS)
 * @param options Retrieval configuration
 * @return Image pairs within distance threshold
 * 
 * Algorithm:
 * 1. Filter images with valid GNSS
 * 2. Build k-d tree from positions
 * 3. Radius search for each image
 * 4. Optional: filter by IMU orientation
 * 5. Return pairs with spatial metadata
 */
std::vector<ImagePair> retrieveByGPS(
    const std::vector<ImageInfo>& images,
    const RetrievalOptions& options
);

/**
 * @brief Batch radius search using k-d tree
 * 
 * @param positions Query positions (Nx3)
 * @param radius Search radius
 * @param max_neighbors Maximum neighbors per query
 * @return neighbors[i] = indices of neighbors for position i
 */
std::vector<std::vector<size_t>> radiusSearchBatch(
    const std::vector<Eigen::Vector3d>& positions,
    double radius,
    int max_neighbors = -1
);

}  // namespace insight::algorithm::retrieval
