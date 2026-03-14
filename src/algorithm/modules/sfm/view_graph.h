/**
 * @file  view_graph.h
 * @brief View graph for incremental SfM: pair geometry info and initial pair selection.
 *
 * F matrix is always estimated (no K needed) and drives connectivity and initial-pair
 * selection. E / twoview results are recorded as optional supplementary info.
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

/// Per-pair geometry summary from isat_geo (or equivalent). Index-only (0..n_images-1).
/// F is the primary quality signal (always available). E/twoview are supplementary.
struct PairGeoInfo {
  uint32_t image1_index = 0;
  uint32_t image2_index = 0;

  // ── F (Fundamental) — primary, always computed ────────────────────────────
  bool F_ok = false;          ///< F RANSAC succeeded
  int F_inliers = 0;          ///< Number of F inliers (main connectivity weight)
  float F_inlier_ratio = 0.f; ///< F inlier ratio

  // ── H (Homography) — degeneracy signal ───────────────────────────────────
  bool H_ok = false;
  int H_inliers = 0;

  // ── Degeneracy ────────────────────────────────────────────────────────────
  bool is_degenerate = false; ///< Scene degenerate (pure rotation / planar)

  // ── E (Essential) — supplementary, requires known K ──────────────────────
  bool E_ok = false;
  int E_inliers = 0;

  // ── Two-view reconstruction — supplementary, requires --twoview ──────────
  bool twoview_ok = false;
  bool stable = false;
  int num_valid_points = 0;
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

  /// Score for pair: log(1 + F_inliers) + connectivity bonus.
  /// Degeneracy applies a penalty. twoview stable adds a bonus when available.
  /// Weights: w_f_inliers=1.0, w_connect=0.3, w_stable=0.5 (tunable).
  double get_pair_score(size_t pair_index, double w_f_inliers = 1.0, double w_connect = 0.3,
                        double w_stable = 0.5) const;

  /// Best initial pair: both images not in \p registered, F_ok && !is_degenerate, max score.
  /// Returns (image1_index, image2_index) or nullopt if none.
  std::optional<std::pair<uint32_t, uint32_t>>
  choose_initial_pair(const std::set<uint32_t>& registered) const;

  /// Candidate pair indices (F_ok && !is_degenerate, both not in registered), sorted by score descending.
  /// Use for the initial-pair loop: try each in order until one succeeds.
  std::vector<size_t> get_candidate_pair_indices_sorted(const std::set<uint32_t>& registered) const;

  /// F inlier count for the pair (im_a, im_b). Returns -1 if not found.
  int get_F_inliers(uint32_t im_a, uint32_t im_b) const;

private:
  std::vector<PairGeoInfo> pairs_;
  /// Per-image degree (number of pairs containing this image). Rebuilt when needed.
  mutable std::vector<int> image_degree_;
  mutable bool degree_dirty_ = true;

  void ensure_degree_computed() const;
};

} // namespace sfm
} // namespace insight
