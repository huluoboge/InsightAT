/**
 * @file  key_points_node.h
 * @brief 四叉树关键点分布（ORB-SLAM 风格）：递归四分保证空间均匀覆盖。
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
 * @brief 四叉树节点，表示图像上一块矩形区域；关键点过多时递归四分。
 */
class KeypointsNode {
public:
  KeypointsNode() = default;

  /** 将本节点四分，返回 4 个子节点（左上、右上、左下、右下）。 */
  std::array<KeypointsNode, 4> divide_node();

  /** 节点区域像素面积（宽×高）。 */
  unsigned int size() const { return (pt_end_.x - pt_begin_.x) * (pt_end_.y - pt_begin_.y); }

  std::vector<cv::KeyPoint> keypts_;        ///< 落入本节点区域的关键点
  cv::Point2i pt_begin_;                    ///< 区域左上角
  cv::Point2i pt_end_;                      ///< 区域右下角（不包含）
  std::list<KeypointsNode>::iterator iter_; ///< 自身在节点链表中的迭代器（便于删除）
};

/**
 * 使用自适应四叉树将关键点均匀分布（ORB-SLAM 风格）。
 *
 * @param keypts_to_distribute 待分布的关键点
 * @param min_x, max_x, min_y, max_y 图像区域边界
 * @param scale_factor 金字塔尺度因子（参与最小格大小）
 * @param max_num_keypts 目标保留关键点数量
 * @return 均匀分布后的关键点子集（每叶节点取响应最大者）
 */
std::vector<cv::KeyPoint>
distribute_keypoints_via_tree(const std::vector<cv::KeyPoint>& keypts_to_distribute, int min_x,
                              int max_x, int min_y, int max_y, float scale_factor,
                              int max_num_keypts);

} // namespace modules
} // namespace insight
