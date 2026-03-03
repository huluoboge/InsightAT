/**
 * @file  key_points_node.cpp
 * @brief 四叉树关键点分布节点实现：divide_node、initialize_nodes、assign_child_nodes 等。
 */

#include "key_points_node.h"

#include <algorithm>
#include <cmath>

namespace insight {
namespace modules {

std::array<KeypointsNode, 4> KeypointsNode::divide_node() {
  const unsigned int half_x = cvCeil((pt_end_.x - pt_begin_.x) / 2.0);
  const unsigned int half_y = cvCeil((pt_end_.y - pt_begin_.y) / 2.0);

  std::array<KeypointsNode, 4> child_nodes;

  const auto pt_top = cv::Point2i(pt_begin_.x + half_x, pt_begin_.y);
  const auto pt_left = cv::Point2i(pt_begin_.x, pt_begin_.y + half_y);
  const auto pt_center = cv::Point2i(pt_begin_.x + half_x, pt_begin_.y + half_y);
  const auto pt_right = cv::Point2i(pt_end_.x, pt_begin_.y + half_y);
  const auto pt_bottom = cv::Point2i(pt_begin_.x + half_x, pt_end_.y);

  child_nodes.at(0).pt_begin_ = pt_begin_;
  child_nodes.at(0).pt_end_ = pt_center;
  child_nodes.at(1).pt_begin_ = pt_top;
  child_nodes.at(1).pt_end_ = pt_right;
  child_nodes.at(2).pt_begin_ = pt_left;
  child_nodes.at(2).pt_end_ = pt_bottom;
  child_nodes.at(3).pt_begin_ = pt_center;
  child_nodes.at(3).pt_end_ = pt_end_;

  for (auto& node : child_nodes) {
    node.keypts_.reserve(keypts_.size());
  }

  for (const auto& keypt : keypts_) {
    unsigned int idx = 0;
    if (pt_begin_.x + half_x <= keypt.pt.x)
      idx += 1;
    if (pt_begin_.y + half_y <= keypt.pt.y)
      idx += 2;
    child_nodes.at(idx).keypts_.push_back(keypt);
  }

  return child_nodes;
}

static std::list<KeypointsNode>
initialize_nodes(const std::vector<cv::KeyPoint>& keypts_to_distribute, int min_x, int max_x,
                 int min_y, int max_y) {
  const double ratio = static_cast<double>(max_x - min_x) / (max_y - min_y);

  double delta_x, delta_y;
  unsigned int num_x_grid, num_y_grid;

  if (ratio > 1) {
    num_x_grid = std::round(ratio);
    num_y_grid = 1;
    delta_x = static_cast<double>(max_x - min_x) / num_x_grid;
    delta_y = max_y - min_y;
  } else {
    num_x_grid = 1;
    num_y_grid = std::round(1 / ratio);
    delta_x = static_cast<double>(max_x - min_x);
    delta_y = static_cast<double>(max_y - min_y) / num_y_grid;
  }

  const unsigned int num_initial_nodes = num_x_grid * num_y_grid;
  std::list<KeypointsNode> nodes;
  std::vector<KeypointsNode*> initial_nodes;
  initial_nodes.resize(num_initial_nodes);

  for (unsigned int i = 0; i < num_initial_nodes; ++i) {
    KeypointsNode node;
    const unsigned int ix = i % num_x_grid;
    const unsigned int iy = i / num_x_grid;

    node.pt_begin_ =
        cv::Point2i(static_cast<int>(min_x + delta_x * ix), static_cast<int>(min_y + delta_y * iy));
    node.pt_end_ = cv::Point2i(static_cast<int>(min_x + delta_x * (ix + 1)),
                               static_cast<int>(min_y + delta_y * (iy + 1)));
    node.keypts_.reserve(keypts_to_distribute.size());

    nodes.push_back(node);
    initial_nodes.at(i) = &nodes.back();
  }

  for (const auto& keypt : keypts_to_distribute) {
    if (keypt.pt.x < min_x || keypt.pt.x >= max_x || keypt.pt.y < min_y || keypt.pt.y >= max_y)
      continue;
    const unsigned int ix = static_cast<unsigned int>((keypt.pt.x - min_x) / delta_x);
    const unsigned int iy = static_cast<unsigned int>((keypt.pt.y - min_y) / delta_y);
    const unsigned int node_idx = ix + iy * num_x_grid;
    if (node_idx < initial_nodes.size())
      initial_nodes.at(node_idx)->keypts_.push_back(keypt);
  }

  auto iter = nodes.begin();
  while (iter != nodes.end()) {
    if (iter->keypts_.empty()) {
      iter = nodes.erase(iter);
      continue;
    }
    iter++;
  }

  return nodes;
}

static void assign_child_nodes(const std::array<KeypointsNode, 4>& child_nodes,
                               std::list<KeypointsNode>& nodes,
                               std::vector<std::pair<int, KeypointsNode*>>& leaf_nodes) {
  for (const auto& child_node : child_nodes) {
    if (child_node.keypts_.empty())
      continue;
    nodes.push_front(child_node);
    if (child_node.keypts_.size() == 1)
      continue;
    leaf_nodes.emplace_back(
        std::make_pair(static_cast<int>(child_node.keypts_.size()), &nodes.front()));
    nodes.front().iter_ = nodes.begin();
  }
}

static std::vector<cv::KeyPoint> find_keypoints_with_max_response(std::list<KeypointsNode>& nodes) {
  std::vector<cv::KeyPoint> result_keypts;
  result_keypts.reserve(nodes.size());

  for (auto& node : nodes) {
    auto& node_keypts = node.keypts_;
    if (node_keypts.empty())
      continue;
    auto max_it = std::max_element(
        node_keypts.begin(), node_keypts.end(),
        [](const cv::KeyPoint& a, const cv::KeyPoint& b) { return a.response < b.response; });
    result_keypts.push_back(*max_it);
  }

  return result_keypts;
}

std::vector<cv::KeyPoint>
distribute_keypoints_via_tree(const std::vector<cv::KeyPoint>& keypts_to_distribute, int min_x,
                              int max_x, int min_y, int max_y, float scale_factor,
                              int max_num_keypts) {
  const float min_cell_size = static_cast<float>(max_num_keypts);

  auto nodes = initialize_nodes(keypts_to_distribute, min_x, max_x, min_y, max_y);
  std::vector<std::pair<int, KeypointsNode*>> leaf_nodes_pool;
  leaf_nodes_pool.reserve(nodes.size() * 10);

  while (true) {
    const unsigned int prev_size = nodes.size();
    auto iter = nodes.begin();
    leaf_nodes_pool.clear();

    while (iter != nodes.end()) {
      if (iter->keypts_.size() == 1 ||
          iter->size() * scale_factor * scale_factor <= min_cell_size) {
        iter++;
        continue;
      }
      const auto child_nodes = iter->divide_node();
      assign_child_nodes(child_nodes, nodes, leaf_nodes_pool);
      iter = nodes.erase(iter);
    }

    if (nodes.size() == prev_size)
      break;
  }

  return find_keypoints_with_max_response(nodes);
}

} // namespace modules
} // namespace insight
