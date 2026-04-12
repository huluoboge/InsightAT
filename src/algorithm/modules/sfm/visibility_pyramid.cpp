/**
 * @file visibility_pyramid.cpp
 * @brief COLMAP-compatible VisibilityPyramid (see visibility_pyramid.h).
 */

#include "visibility_pyramid.h"

#include <algorithm>
#include <cmath>

#include <glog/logging.h>

namespace insight {
namespace sfm {

namespace {

template <typename T>
static T clamp_value(T v, T lo, T hi) {
  return std::max(lo, std::min(v, hi));
}

} // namespace

VisibilityPyramid::VisibilityPyramid() : VisibilityPyramid(0, 0, 0) {}

VisibilityPyramid::VisibilityPyramid(const size_t num_levels, const size_t width, const size_t height)
    : width_(width), height_(height), score_(0), max_score_(0) {
  pyramid_.resize(num_levels);
  for (size_t level = 0; level < num_levels; ++level) {
    const size_t level_plus_one = level + 1;
    const int dim = 1 << level_plus_one;
    pyramid_[level].setZero(dim, dim);
    max_score_ += static_cast<size_t>(dim) * static_cast<size_t>(dim) * static_cast<size_t>(dim) *
                  static_cast<size_t>(dim);
  }
}

void VisibilityPyramid::Reset() {
  score_ = 0;
  for (auto& level : pyramid_)
    level.setZero();
}

void VisibilityPyramid::SetPoint(const double x, const double y) {
  CHECK_GT(pyramid_.size(), 0u);

  size_t cx = 0;
  size_t cy = 0;
  CellForPoint(x, y, &cx, &cy);

  for (int i = static_cast<int>(pyramid_.size() - 1); i >= 0; --i) {
    auto& level = pyramid_[static_cast<size_t>(i)];

    level(static_cast<Eigen::Index>(cy), static_cast<Eigen::Index>(cx)) += 1;
    if (level(static_cast<Eigen::Index>(cy), static_cast<Eigen::Index>(cx)) == 1) {
      score_ += static_cast<size_t>(level.size());
    }

    cx >>= 1u;
    cy >>= 1u;
  }

  CHECK_LE(score_, max_score_);
}

void VisibilityPyramid::ResetPoint(const double x, const double y) {
  CHECK_GT(pyramid_.size(), 0u);

  size_t cx = 0;
  size_t cy = 0;
  CellForPoint(x, y, &cx, &cy);

  for (int i = static_cast<int>(pyramid_.size() - 1); i >= 0; --i) {
    auto& level = pyramid_[static_cast<size_t>(i)];

    level(static_cast<Eigen::Index>(cy), static_cast<Eigen::Index>(cx)) -= 1;
    if (level(static_cast<Eigen::Index>(cy), static_cast<Eigen::Index>(cx)) == 0) {
      score_ -= static_cast<size_t>(level.size());
    }

    cx >>= 1u;
    cy >>= 1u;
  }

  CHECK_LE(score_, max_score_);
}

void VisibilityPyramid::CellForPoint(const double x, const double y, size_t* cx, size_t* cy) const {
  CHECK_GT(width_, 0u);
  CHECK_GT(height_, 0u);
  const size_t max_dim = static_cast<size_t>(1) << pyramid_.size();
  *cx = clamp_value(static_cast<size_t>(static_cast<double>(max_dim) * x / static_cast<double>(width_)),
                    static_cast<size_t>(0), max_dim - 1);
  *cy = clamp_value(static_cast<size_t>(static_cast<double>(max_dim) * y / static_cast<double>(height_)),
                    static_cast<size_t>(0), max_dim - 1);
}

} // namespace sfm
} // namespace insight
