#include "spatial_retrieval.h"
#include <glog/logging.h>
#include <cmath>
#include <algorithm>
#include "nanoflann/nanoflann.hpp"

namespace insight::algorithm::retrieval {

// ============================================================================
// Point Cloud Adaptor for nanoflann
// ============================================================================

/**
 * @brief Adaptor for Eigen::Vector3d point cloud
 */
struct PointCloud {
    std::vector<Eigen::Vector3d> pts;
    
    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.size(); }
    
    // Returns the dim'th component of the idx'th point
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
        return pts[idx](dim);
    }
    
    // Optional bounding-box computation (return false to default to a standard bbox)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
};

// Define k-d tree type
using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<double, PointCloud>,
    PointCloud,
    3  // 3D space
>;

// ============================================================================
// Utility Functions
// ============================================================================

double angleDifference(const Eigen::Vector3d& att1, const Eigen::Vector3d& att2) {
    // Compute angular difference for roll, pitch, yaw
    // Handle wrap-around for yaw (0-360 degrees)
    
    double diff_roll = std::abs(att1(0) - att2(0));
    double diff_pitch = std::abs(att1(1) - att2(1));
    double diff_yaw = std::abs(att1(2) - att2(2));
    
    // Normalize yaw to [-pi, pi]
    if (diff_yaw > M_PI) {
        diff_yaw = 2.0 * M_PI - diff_yaw;
    }
    
    // Return max difference (most conservative)
    double max_diff = std::max({diff_roll, diff_pitch, diff_yaw});
    return max_diff * 180.0 / M_PI;  // Convert to degrees
}

double computeSpatialScore(double distance, double threshold) {
    if (threshold <= 0) return 1.0;
    // Exponential decay: score = exp(-distance / threshold)
    // At distance = threshold, score = exp(-1) ≈ 0.37
    // At distance = 0, score = 1.0
    return std::exp(-distance / threshold);
}

std::vector<ImageInfo> filterImagesWithGNSS(const std::vector<ImageInfo>& images) {
    std::vector<ImageInfo> result;
    result.reserve(images.size());
    
    for (const auto& img : images) {
        if (img.hasGNSS()) {
            result.push_back(img);
        }
    }
    
    LOG(INFO) << "Filtered " << result.size() << "/" << images.size() 
              << " images with valid GNSS";
    
    return result;
}

// ============================================================================
// K-D Tree Search
// ============================================================================

std::vector<std::vector<size_t>> radiusSearchBatch(
    const std::vector<Eigen::Vector3d>& positions,
    double radius,
    int max_neighbors
) {
    if (positions.empty()) {
        return {};
    }
    
    // Build point cloud
    PointCloud cloud;
    cloud.pts = positions;
    
    // Build k-d tree (10 leaf size is default)
    KDTree index(3, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    index.buildIndex();
    
    // Radius search for each position
    std::vector<std::vector<size_t>> all_neighbors;
    all_neighbors.reserve(positions.size());
    
    nanoflann::SearchParameters params;
    params.sorted = true;  // Sort by distance
    
    for (size_t i = 0; i < positions.size(); ++i) {
        std::vector<nanoflann::ResultItem<uint32_t, double>> matches;
        
        double query_pt[3] = {positions[i](0), positions[i](1), positions[i](2)};
        double search_radius_sq = radius * radius;
        
        size_t n_matches = index.radiusSearch(query_pt, search_radius_sq, matches, params);
        
        // Extract indices (skip self-match)
        std::vector<size_t> neighbors;
        neighbors.reserve(n_matches);
        
        for (const auto& match : matches) {
            if (match.first != static_cast<uint32_t>(i)) {  // Skip self
                neighbors.push_back(static_cast<size_t>(match.first));
                
                if (max_neighbors > 0 && neighbors.size() >= static_cast<size_t>(max_neighbors)) {
                    break;
                }
            }
        }
        
        all_neighbors.push_back(std::move(neighbors));
    }
    
    return all_neighbors;
}

// ============================================================================
// Main Retrieval Function
// ============================================================================

std::vector<ImagePair> retrieveByGPS(
    const std::vector<ImageInfo>& images,
    const RetrievalOptions& options
) {
    LOG(INFO) << "GPS spatial retrieval: distance_threshold=" << options.distance_threshold
              << "m, angle_threshold=" << options.angle_threshold << "°";
    
    // 1. Filter images with GNSS
    auto valid_images = filterImagesWithGNSS(images);
    
    if (valid_images.empty()) {
        LOG(WARNING) << "No images with valid GNSS data";
        return {};
    }
    
    // 2. Extract positions
    std::vector<Eigen::Vector3d> positions;
    positions.reserve(valid_images.size());
    for (const auto& img : valid_images) {
        positions.push_back(img.gnss->position());
    }
    
    // 3. Batch radius search
    auto all_neighbors = radiusSearchBatch(
        positions, 
        options.distance_threshold,
        options.max_neighbors
    );
    
    // 4. Build image pairs
    std::vector<ImagePair> pairs;
    
    for (size_t i = 0; i < valid_images.size(); ++i) {
        const auto& img1 = valid_images[i];
        const auto& neighbors = all_neighbors[i];
        
        for (size_t j_idx : neighbors) {
            // Skip if pair already added (maintain i < j)
            if (i >= j_idx) continue;
            
            const auto& img2 = valid_images[j_idx];
            
            // Compute distance
            double distance = euclideanDistance(
                img1.gnss->position(),
                img2.gnss->position()
            );
            
            // Optional: IMU orientation filter
            if (options.use_imu_filter && 
                img1.hasIMU() && img2.hasIMU() && 
                options.angle_threshold > 0) {
                
                double angle_diff = angleDifference(
                    img1.imu->attitude(),
                    img2.imu->attitude()
                );
                
                if (angle_diff > options.angle_threshold) {
                    VLOG(2) << "Filtered pair (" << i << "," << j_idx 
                            << ") by angle: " << angle_diff << "°";
                    continue;
                }
            }
            
            // Create pair
            ImagePair pair;
            pair.image1_idx = static_cast<int>(i);
            pair.image2_idx = static_cast<int>(j_idx);
            pair.score = computeSpatialScore(distance, options.distance_threshold);
            pair.method = "gps";
            pair.spatial_distance = distance;
            
            // Add angle difference if IMU available
            if (options.use_imu_filter && img1.hasIMU() && img2.hasIMU()) {
                pair.angle_difference = angleDifference(
                    img1.imu->attitude(),
                    img2.imu->attitude()
                );
            }
            
            pairs.push_back(pair);
        }
    }
    
    LOG(INFO) << "GPS retrieval: generated " << pairs.size() << " pairs from "
              << valid_images.size() << " images";
    
    return pairs;
}

}  // namespace insight::algorithm::retrieval
