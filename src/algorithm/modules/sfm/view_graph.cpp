/**
 * @file  view_graph.cpp
 * @brief ViewGraph implementation: pair scoring and initial pair selection.
 */

#include "view_graph.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace insight {
namespace sfm {

void ViewGraph::add_pair(const PairGeoInfo& info) {
  pairs_.push_back(info);
  degree_dirty_ = true;
}

void ViewGraph::reserve(size_t n_pairs) { pairs_.reserve(n_pairs); }

void ViewGraph::ensure_degree_computed() const {
  if (!degree_dirty_)
    return;
  std::unordered_map<uint32_t, int> deg;
  for (const auto& p : pairs_) {
    deg[p.image1_index]++;
    deg[p.image2_index]++;
  }
  if (deg.empty()) {
    image_degree_.clear();
    degree_dirty_ = false;
    return;
  }
  uint32_t max_id = 0;
  for (const auto& kv : deg)
    if (kv.first > max_id)
      max_id = kv.first;
  image_degree_.assign(static_cast<size_t>(max_id) + 1, 0);
  for (const auto& kv : deg)
    image_degree_[static_cast<size_t>(kv.first)] = kv.second;
  degree_dirty_ = false;
}

double ViewGraph::get_pair_score(size_t pair_index, double w_f_inliers, double w_connect,
                                 double w_stable) const {
  if (pair_index >= pairs_.size())
    return -1e9;
  ensure_degree_computed();
  const auto& p = pairs_[pair_index];
  double s = 0.0;
  // Primary: F inliers (log scale to avoid dominance by large counts)
  s += w_f_inliers * std::log(1.0 + static_cast<double>(std::max(0, p.F_inliers)));
  // Connectivity bonus
  int d1 = 0, d2 = 0;
  if (static_cast<size_t>(p.image1_index) < image_degree_.size())
    d1 = image_degree_[static_cast<size_t>(p.image1_index)];
  if (static_cast<size_t>(p.image2_index) < image_degree_.size())
    d2 = image_degree_[static_cast<size_t>(p.image2_index)];
  s += w_connect * static_cast<double>(d1 + d2);
  // Supplementary bonus if twoview result is stable
  if (p.twoview_ok && p.stable)
    s += w_stable;
  // Degeneracy penalty
  if (p.is_degenerate)
    s -= 5.0;
  return s;
}

std::optional<std::pair<uint32_t, uint32_t>>
ViewGraph::choose_initial_pair(const std::set<uint32_t>& registered) const {
  if (pairs_.empty())
    return std::nullopt;
  ensure_degree_computed();
  size_t best = pairs_.size();
  double best_score = -1e9;
  for (size_t i = 0; i < pairs_.size(); ++i) {
    const auto& p = pairs_[i];
    if (!p.F_ok || p.is_degenerate)
      continue;
    if (registered.count(p.image1_index) || registered.count(p.image2_index))
      continue;
    const double sc = get_pair_score(i);
    if (sc > best_score) {
      best_score = sc;
      best = i;
    }
  }
  if (best >= pairs_.size())
    return std::nullopt;
  return std::make_pair(pairs_[best].image1_index, pairs_[best].image2_index);
}

std::vector<size_t>
ViewGraph::get_candidate_pair_indices_sorted(const std::set<uint32_t>& registered) const {
  std::vector<size_t> indices;
  for (size_t i = 0; i < pairs_.size(); ++i) {
    const auto& p = pairs_[i];
    // F_ok is the minimum requirement; degenerate pairs are skipped.
    if (!p.F_ok || p.is_degenerate)
      continue;
    if (registered.count(p.image1_index) || registered.count(p.image2_index))
      continue;
    indices.push_back(i);
  }
  std::sort(indices.begin(), indices.end(),
            [this](size_t a, size_t b) { return get_pair_score(a) > get_pair_score(b); });
  return indices;
}

int ViewGraph::get_F_inliers(uint32_t im_a, uint32_t im_b) const {
  const uint32_t lo = std::min(im_a, im_b);
  const uint32_t hi = std::max(im_a, im_b);
  for (const auto& p : pairs_) {
    if (p.image1_index == lo && p.image2_index == hi)
      return p.F_inliers;
  }
  return -1;
}

} // namespace sfm
} // namespace insight
