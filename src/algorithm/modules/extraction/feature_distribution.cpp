/**
 * @file feature_distribution.cpp
 * @brief Implementation of feature distribution strategies
 */

#include "feature_distribution.h"
#include "KeyPointsNode.h"
#include <glog/logging.h>
#include <unordered_map>
#include <cmath>

namespace insight {
namespace modules {

// ============================================================================
// Grid-based distribution (O(n) complexity)
// ============================================================================

std::vector<size_t> DistributeKeypointsGrid(
    const std::vector<SiftGPU::SiftKeypoint>& keypoints,
    int image_width,
    int image_height,
    const GridDistributionParams& params)
{
    if (keypoints.empty()) return {};
    
    // Calculate grid dimensions
    const int num_cols = (image_width + params.grid_size - 1) / params.grid_size;
    const int num_rows = (image_height + params.grid_size - 1) / params.grid_size;
    const int num_cells = num_cols * num_rows;
    
    // Group keypoints by grid cell
    std::vector<std::vector<size_t>> grid_cells(num_cells);
    
    for (size_t i = 0; i < keypoints.size(); ++i) {
        const auto& kp = keypoints[i];
        int col = static_cast<int>(kp.x) / params.grid_size;
        int row = static_cast<int>(kp.y) / params.grid_size;
        
        // Boundary check
        col = std::clamp(col, 0, num_cols - 1);
        row = std::clamp(row, 0, num_rows - 1);
        
        int cell_idx = row * num_cols + col;
        grid_cells[cell_idx].push_back(i);
    }
    
    // For each cell, keep top features
    std::vector<size_t> kept_indices;
    kept_indices.reserve(keypoints.size() / 2);  // Rough estimate
    
    const float angle_threshold_rad = params.orientation_threshold_deg * M_PI / 180.0f;
    
    for (auto& cell : grid_cells) {
        if (cell.empty()) continue;
        
        // Sort by scale (larger scale = stronger response in SIFT)
        std::sort(cell.begin(), cell.end(),
            [&keypoints](size_t a, size_t b) {
                return keypoints[a].s > keypoints[b].s;
            });
        
        if (!params.keep_orientation) {
            // Simple: keep top max_per_cell features
            int to_keep = std::min(params.max_per_cell, static_cast<int>(cell.size()));
            for (int i = 0; i < to_keep; ++i) {
                kept_indices.push_back(cell[i]);
            }
        } else {
            // Advanced: keep multiple orientations at same location
            std::vector<size_t> cell_kept;
            cell_kept.reserve(params.max_per_cell * 2);
            
            for (size_t idx : cell) {
                const auto& kp = keypoints[idx];
                bool is_duplicate = false;
                
                // Check if this is same location but different orientation
                for (size_t kept_idx : cell_kept) {
                    const auto& kept_kp = keypoints[kept_idx];
                    
                    // Same location check (< 2 pixel distance)
                    const float dx = kp.x - kept_kp.x;
                    const float dy = kp.y - kept_kp.y;
                    if (dx * dx + dy * dy < 4.0f) {
                        // Check orientation difference
                        float angle_diff = std::fabs(kp.o - kept_kp.o);
                        // Wrap to [0, π]
                        if (angle_diff > M_PI) {
                            angle_diff = 2.0f * M_PI - angle_diff;
                        }
                        
                        // If orientations are similar, it's a duplicate
                        if (angle_diff < angle_threshold_rad) {
                            is_duplicate = true;
                            break;
                        }
                        // Different orientation at same location - keep both!
                    }
                }
                
                if (!is_duplicate && static_cast<int>(cell_kept.size()) < params.max_per_cell * 2) {
                    cell_kept.push_back(idx);
                }
            }
            
            // Add to final list
            kept_indices.insert(kept_indices.end(), cell_kept.begin(), cell_kept.end());
        }
    }
    
    LOG(INFO) << "Grid distribution: " << kept_indices.size() << "/" << keypoints.size()
              << " features kept (grid=" << params.grid_size << "px, max/cell=" << params.max_per_cell << ")";
    
    return kept_indices;
}

std::vector<size_t> DistributeKeypointsGrid(
    const std::vector<cv::KeyPoint>& keypoints,
    int image_width,
    int image_height,
    const GridDistributionParams& params)
{
    if (keypoints.empty()) return {};
    
    // Calculate grid dimensions
    const int num_cols = (image_width + params.grid_size - 1) / params.grid_size;
    const int num_rows = (image_height + params.grid_size - 1) / params.grid_size;
    const int num_cells = num_cols * num_rows;
    
    // Group keypoints by grid cell
    std::vector<std::vector<size_t>> grid_cells(num_cells);
    
    for (size_t i = 0; i < keypoints.size(); ++i) {
        const auto& kp = keypoints[i];
        int col = static_cast<int>(kp.pt.x) / params.grid_size;
        int row = static_cast<int>(kp.pt.y) / params.grid_size;
        
        // Boundary check
        col = std::clamp(col, 0, num_cols - 1);
        row = std::clamp(row, 0, num_rows - 1);
        
        int cell_idx = row * num_cols + col;
        grid_cells[cell_idx].push_back(i);
    }
    
    // For each cell, keep top features
    std::vector<size_t> kept_indices;
    kept_indices.reserve(keypoints.size() / 2);
    
    const float angle_threshold_rad = params.orientation_threshold_deg * M_PI / 180.0f;
    
    for (auto& cell : grid_cells) {
        if (cell.empty()) continue;
        
        // Sort by response (higher = stronger)
        std::sort(cell.begin(), cell.end(),
            [&keypoints](size_t a, size_t b) {
                return keypoints[a].response > keypoints[b].response;
            });
        
        if (!params.keep_orientation) {
            int to_keep = std::min(params.max_per_cell, static_cast<int>(cell.size()));
            for (int i = 0; i < to_keep; ++i) {
                kept_indices.push_back(cell[i]);
            }
        } else {
            std::vector<size_t> cell_kept;
            cell_kept.reserve(params.max_per_cell * 2);
            
            for (size_t idx : cell) {
                const auto& kp = keypoints[idx];
                bool is_duplicate = false;
                
                for (size_t kept_idx : cell_kept) {
                    const auto& kept_kp = keypoints[kept_idx];
                    
                    const float dx = kp.pt.x - kept_kp.pt.x;
                    const float dy = kp.pt.y - kept_kp.pt.y;
                    if (dx * dx + dy * dy < 4.0f) {
                        float angle_diff = std::fabs(kp.angle - kept_kp.angle);
                        if (angle_diff > 180.0f) {
                            angle_diff = 360.0f - angle_diff;
                        }
                        angle_diff = angle_diff * M_PI / 180.0f;
                        
                        if (angle_diff < angle_threshold_rad) {
                            is_duplicate = true;
                            break;
                        }
                    }
                }
                
                if (!is_duplicate && static_cast<int>(cell_kept.size()) < params.max_per_cell * 2) {
                    cell_kept.push_back(idx);
                }
            }
            
            kept_indices.insert(kept_indices.end(), cell_kept.begin(), cell_kept.end());
        }
    }
    
    LOG(INFO) << "Grid distribution: " << kept_indices.size() << "/" << keypoints.size()
              << " features kept";
    
    return kept_indices;
}

// ============================================================================
// Quadtree-based distribution (ORB-SLAM style, wrapper)
// ============================================================================

std::vector<cv::KeyPoint> SiftGPUToOpenCV(
    const std::vector<SiftGPU::SiftKeypoint>& sift_keypoints)
{
    std::vector<cv::KeyPoint> cv_keypoints;
    cv_keypoints.reserve(sift_keypoints.size());
    
    for (const auto& kp : sift_keypoints) {
        cv::KeyPoint cv_kp;
        cv_kp.pt.x = kp.x;
        cv_kp.pt.y = kp.y;
        cv_kp.size = kp.s * 2.0f;  // SIFT scale to diameter
        cv_kp.angle = kp.o * 180.0f / M_PI;  // Radians to degrees
        cv_kp.response = kp.s;  // Use scale as response
        cv_keypoints.push_back(cv_kp);
    }
    
    return cv_keypoints;
}

std::vector<size_t> GetKeypointIndices(
    const std::vector<cv::KeyPoint>& original,
    const std::vector<cv::KeyPoint>& distributed)
{
    std::vector<size_t> indices;
    indices.reserve(distributed.size());
    
    // Build hash map for fast lookup (x,y -> index)
    std::unordered_map<int64_t, size_t> position_map;
    for (size_t i = 0; i < original.size(); ++i) {
        int64_t key = (static_cast<int64_t>(original[i].pt.x * 10) << 32) | 
                      static_cast<int64_t>(original[i].pt.y * 10);
        position_map[key] = i;
    }
    
    // Find indices
    for (const auto& kp : distributed) {
        int64_t key = (static_cast<int64_t>(kp.pt.x * 10) << 32) | 
                      static_cast<int64_t>(kp.pt.y * 10);
        auto it = position_map.find(key);
        if (it != position_map.end()) {
            indices.push_back(it->second);
        }
    }
    
    return indices;
}

std::vector<size_t> DistributeKeypointsQuadtree(
    const std::vector<cv::KeyPoint>& keypoints,
    int image_width,
    int image_height,
    const QuadtreeDistributionParams& params)
{
    if (keypoints.empty()) return {};
    
    // Call ORB-SLAM style quadtree distribution
    auto distributed = distribute_keypoints_via_tree(
        keypoints,
        0, image_width,
        0, image_height,
        1.0f,  // scale_factor (not used for SIFT, set to 1)
        params.max_num_features
    );
    
    LOG(INFO) << "Quadtree distribution: " << distributed.size() << "/" << keypoints.size()
              << " features kept (target=" << params.max_num_features << ")";
    
    // Get indices of distributed keypoints
    return GetKeypointIndices(keypoints, distributed);
}

} // namespace modules
} // namespace insight
