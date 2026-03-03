/**
 * @file  view_graph.h
 * @brief View graph for incremental SfM: pair geometry info and initial pair selection.
 *
 * Build from geo results (E_ok, twoview_ok, stable, num_valid_points). Supports
 * scoring and choosing the best initial pair (both images unregistered, stable two-view).
 * No Qt / IDC dependency; loaders fill PairGeoInfo from .isat_geo elsewhere.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <set>
#include <utility>
#include <vector>

namespace insight {
namespace sfm {

/// Per-pair geometry summary from isat_geo (or equivalent).
struct PairGeoInfo {
  uint32_t image1_id = 0;
  uint32_t image2_id = 0;
  bool E_ok = false;
  bool twoview_ok = false;
  bool stable = false;
  int num_valid_points = 0;
  int E_inliers = 0; ///< For connectivity / quality weighting
};

/**
 * View graph: list of pairs with geometry, used to score and choose initial pair.
 * Connectivity is derived from how many pairs each image appears in.
 */
class ViewGraph {
public:
  ViewGraph() = default;

  void add_pair(const PairGeoInfo& info);
  void reserve(size_t n_pairs);

  size_t num_pairs() const { return pairs_.size(); }
  const PairGeoInfo& pair_at(size_t i) const { return pairs_.at(i); }

  /// Score for pair index: stability + log(1 + num_valid_points) + connectivity.
  /// Weights: w_stable=1, w_points=0.5, w_connect=0.3 (tunable).
  double get_pair_score(size_t pair_index, double w_stable = 1.0, double w_points = 0.5,
                        double w_connect = 0.3) const;

  /// Best initial pair: both images not in \p registered, twoview_ok && stable, max score.
  /// Returns (image1_id, image2_id) or nullopt if none.
  std::optional<std::pair<uint32_t, uint32_t>>
  choose_initial_pair(const std::set<uint32_t>& registered) const;

private:
  std::vector<PairGeoInfo> pairs_;
  /// Per-image degree (number of pairs containing this image). Rebuilt when needed.
  mutable std::vector<int> image_degree_;
  mutable bool degree_dirty_ = true;

  void ensure_degree_computed() const;
};

} // namespace sfm
} // namespace insight
