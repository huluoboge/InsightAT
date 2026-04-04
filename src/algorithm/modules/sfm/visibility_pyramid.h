/**
 * @file visibility_pyramid.h
 * @brief Multi-resolution occupancy pyramid for 2D point distribution (COLMAP-compatible).
 *
 * Ported from COLMAP src/colmap/scene/visibility_pyramid.{h,cc} (BSD license).
 * Used for resection candidate ranking: higher normalized score ⇒ better spread in image space.
 */

#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>

namespace insight {
namespace sfm {

/// Captures how uniformly 2D points cover the image using a multi-level grid pyramid.
/// Score increases when points occupy finer cells; clustered points add little extra score.
class VisibilityPyramid {
 public:
  VisibilityPyramid();
  VisibilityPyramid(size_t num_levels, size_t width, size_t height);

  void SetPoint(double x, double y);
  void ResetPoint(double x, double y);

  /// Clear all occupancy counts and reset score to 0 without reallocating pyramid matrices.
  /// Requires that width/height/num_levels are already set (i.e. not the default constructor).
  void Reset();

  size_t NumLevels() const { return pyramid_.size(); }
  size_t Width() const { return width_; }
  size_t Height() const { return height_; }
  size_t Score() const { return score_; }
  size_t MaxScore() const { return max_score_; }

 private:
  void CellForPoint(double x, double y, size_t* cx, size_t* cy) const;

  size_t width_ = 0;
  size_t height_ = 0;
  size_t score_ = 0;
  size_t max_score_ = 0;
  std::vector<Eigen::MatrixXi> pyramid_;
};

} // namespace sfm
} // namespace insight
