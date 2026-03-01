/**
 * @file KeyPointsNode.h
 * @brief Quadtree-based keypoint distribution (ORB-SLAM style)
 * 
 * Implements adaptive spatial distribution of keypoints using recursive
 * quadtree subdivision. Ensures uniform feature coverage across the image.
 */

#pragma once

#include <array>
#include <list>
#include <vector>
#include <opencv2/core/types.hpp>

namespace insight {
namespace modules {

/**
 * @class KeypointsNode
 * @brief Quadtree node for keypoint spatial distribution
 * 
 * Each node represents a rectangular region of the image.
 * Nodes with too many keypoints are recursively subdivided into 4 children.
 */
class KeypointsNode {
public:
    KeypointsNode() = default;

    /**
     * Divide this node into four child nodes (quadtree subdivision)
     * @return Array of 4 child nodes (top-left, top-right, bottom-left, bottom-right)
     */
    std::array<KeypointsNode, 4> divide_node();

    /**
     * Get the area size of this node in pixels
     * @return Width × Height of the node's region
     */
    unsigned int size() const {
        return (pt_end_.x - pt_begin_.x) * (pt_end_.y - pt_begin_.y);
    }

    //! Keypoints distributed into this node's region
    std::vector<cv::KeyPoint> keypts_;

    //! Top-left corner of the node's region
    cv::Point2i pt_begin_;
    
    //! Bottom-right corner of the node's region (exclusive)
    cv::Point2i pt_end_;

    //! Iterator pointing to self in the node list (for efficient removal)
    std::list<KeypointsNode>::iterator iter_;
};

/**
 * Distribute keypoints uniformly using adaptive quadtree subdivision
 * (ORB-SLAM style algorithm)
 * 
 * @param keypts_to_distribute Input keypoints to distribute
 * @param min_x Left boundary of the image region
 * @param max_x Right boundary of the image region
 * @param min_y Top boundary of the image region
 * @param max_y Bottom boundary of the image region
 * @param scale_factor Scale factor for pyramid level (used to determine min cell size)
 * @param max_num_keypts Target number of keypoints to keep
 * @return Uniformly distributed subset of keypoints
 * 
 * Algorithm:
 * 1. Initialize grid-based nodes covering the image
 * 2. Recursively subdivide nodes with multiple keypoints
 * 3. Stop when target count reached or cells too small
 * 4. Keep strongest keypoint per leaf node
 * 
 * Complexity: O(n log n) where n = number of keypoints
 */
std::vector<cv::KeyPoint> distribute_keypoints_via_tree(
    const std::vector<cv::KeyPoint>& keypts_to_distribute,
    int min_x, int max_x,
    int min_y, int max_y,
    float scale_factor,
    int max_num_keypts);

}  // namespace modules
}  // namespace insight
