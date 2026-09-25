/**
 * focal_from_view_graph.h
 *
 * Estimate camera focal length(s) from a view graph of Fundamental matrices.
 *
 * Motivation (GLOMAP ViewGraphCalibrator / COLMAP-style self-calibration):
 *   When EXIF / sensor-DB prior is missing, a single fallback (35mm-equiv or
 *   image width) is often badly wrong.  Instead, recover shared or per-camera
 *   focals from many F constraints, then optionally refine with Ceres.
 *
 * Pipeline:
 *   1. Same-camera pairs: Hartley 1-D search → robust weighted median per camera
 *      (search range / soft prior from that camera's image size).
 *   2. Optional joint Ceres on all pairs (same- and cross-camera):
 *        E = K₂ᵀ F K₁, residual |σ₁(E)−σ₂(E)|, one f parameter per camera.
 */
#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace insight {
namespace sfm {

/// One geometrically verified image pair contributing an F constraint.
struct FocalPairConstraint {
  Eigen::Matrix3d F = Eigen::Matrix3d::Zero();
  double cx1 = 0.0;
  double cy1 = 0.0;
  double cx2 = 0.0;
  double cy2 = 0.0;
  int camera_id1 = 0;
  int camera_id2 = 0;
  int weight = 1; ///< Robust aggregate weight (inliers × quality).
  bool is_degenerate = false;
};

/// Per-camera size / search prior (index = camera_id in options.camera_priors).
struct CameraFocalPrior {
  double width = 0.0;
  double height = 0.0;
  /// 0 ⇒ derive via focal_bounds_from_image_size(width,height).
  double f_min = 0.0;
  double f_max = 0.0;
  double soft_prior = 0.0;
};

/// Derive Hartley / Ceres search range from image size (pixels).
/// Soft prior ≈ 0.7·max(w,h) (COLMAP-ish). Bounds stay within ~[0.5, 2]× that,
/// so we do not accept ultra-wide / tele disasters far from a plausible pinhole.
inline void focal_bounds_from_image_size(double width, double height, double* f_min,
                                         double* f_max, double* soft_prior = nullptr) {
  const double mx = std::max(width, height);
  const double soft = 0.7 * mx;
  if (soft_prior)
    *soft_prior = soft;
  if (f_min)
    *f_min = std::max(100.0, 0.5 * soft); // ≥ ~0.35·max(w,h)
  if (f_max)
    *f_max = std::max((*f_min) * 1.01, 2.0 * soft); // ≤ ~1.4·max(w,h)
}

struct FocalFromViewGraphOptions {
  /// Global fallback bounds when camera_priors is empty / incomplete.
  double f_min = 100.0;
  double f_max = 50000.0;
  double prior_focal = 0.0;
  double soft_prior_focal = 0.0;
  /// Reject Hartley samples outside [lo,hi] × soft prior (tighter than old 0.25–4).
  double prior_ratio_lo = 0.5;
  double prior_ratio_hi = 2.0;
  int min_inliers = 30;
  int min_pairs_per_camera = 3;
  bool refine_with_ceres = true;
  /// Include cross-camera pairs in the joint Ceres stage (K2^T F K1).
  bool use_cross_camera_pairs = true;
  /// Relative σ for soft prior residual in Ceres: (f−prior)/(σ·prior).
  /// Default 0: disabled — image soft prior is only used for Hartley sample
  /// gating / bounds; a strong Ceres prior toward 0.7·max(w,h) fights good F data.
  double ceres_prior_sigma_frac = 0.0;
  /// If Ceres moves more than this fraction from the robust median, keep the median.
  double ceres_max_median_rel_delta = 0.20;
  double ceres_cauchy_scale = 1e-2;
  int max_ceres_iterations = 50;
  int ceres_num_threads = 1;
  int hartley_num_threads = 1;
  /// Optional per-camera priors (size > camera_id). Empty ⇒ global bounds only.
  std::vector<CameraFocalPrior> camera_priors;
};

struct CameraFocalEstimate {
  int camera_id = 0;
  double focal = 0.0;
  double median_focal = 0.0;
  int num_pairs = 0;        ///< Same-camera Hartley samples.
  int num_cross_pairs = 0;  ///< Cross-camera pairs touching this camera in Ceres.
  int num_rejected = 0;
  bool ok = false;
};

struct FocalFromViewGraphResult {
  bool ok = false;
  std::vector<CameraFocalEstimate> cameras;
  double shared_focal = 0.0;
  int num_pairs_used = 0;
  int num_pairs_skipped = 0;
  int num_cross_pairs_used = 0;
  std::string method;
};

FocalFromViewGraphResult estimate_focals_from_view_graph(
    const std::vector<FocalPairConstraint>& pairs,
    const FocalFromViewGraphOptions& opts = {});

} // namespace sfm
} // namespace insight
