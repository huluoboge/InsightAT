/**
 * focal_from_view_graph.h
 *
 * Estimate camera focal length(s) from a view graph of Fundamental matrices.
 *
 * Motivation (GLOMAP ViewGraphCalibrator / COLMAP-style self-calibration):
 *   When EXIF / sensor-DB prior is missing, a single fallback (35mm-equiv or
 *   image width) is often badly wrong.  Instead, recover a shared (or
 *   per-camera) focal from many F constraints, then optionally refine with
 *   Ceres before handing the result to incremental SfM / BA.
 *
 * Pipeline for the common single-camera case:
 *   1. Per-pair Hartley 1-D search: minimise |σ₁(E)−σ₂(E)| with E = Kᵀ F K
 *      (reuses focal_from_fundamental).
 *   2. Robust weighted-median aggregate (weight = F inliers).
 *   3. Optional Ceres joint refine of one shared f against all pairs
 *      (Cauchy loss), analogous to GLOMAP's Fetzer residual stage.
 *
 * Different-camera pairs (camera_id1 != camera_id2) are skipped in v1;
 * they need a two-focal (Bougnoux / iterative) path.
 */
#pragma once

#include <Eigen/Core>
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
  int weight = 1; ///< Typically F-RANSAC inlier count.
  bool is_degenerate = false;
};

struct FocalFromViewGraphOptions {
  double f_min = 100.0;
  double f_max = 50000.0;
  /// Soft prior used only to reject crazy ratios (GLOMAP thres_lower/higher).
  /// 0 means "use image-size-derived soft prior if provided via soft_prior_focal".
  double prior_focal = 0.0;
  double soft_prior_focal = 0.0; ///< e.g. 0.7 * max(w,h); used when prior_focal==0
  double prior_ratio_lo = 0.1;
  double prior_ratio_hi = 10.0;
  int min_inliers = 30;
  int min_pairs_per_camera = 3;
  bool refine_with_ceres = true;
  double ceres_cauchy_scale = 1e-2;
  int max_ceres_iterations = 50;
  int ceres_num_threads = 1;
};

struct CameraFocalEstimate {
  int camera_id = 0;
  double focal = 0.0;        ///< Final estimate (after optional Ceres).
  double median_focal = 0.0; ///< Robust median before Ceres.
  int num_pairs = 0;
  int num_rejected = 0;
  bool ok = false;
};

struct FocalFromViewGraphResult {
  bool ok = false;
  std::vector<CameraFocalEstimate> cameras;
  double shared_focal = 0.0; ///< Convenience: cameras[0].focal when single camera.
  int num_pairs_used = 0;
  int num_pairs_skipped = 0;
  std::string method; ///< "robust_median" or "robust_median+ceres"
};

/**
 * Estimate focals from many F constraints.
 *
 * Primary target: shared-camera collections without EXIF.
 * Returns ok=false if no camera gathers enough valid pair estimates.
 */
FocalFromViewGraphResult estimate_focals_from_view_graph(
    const std::vector<FocalPairConstraint>& pairs,
    const FocalFromViewGraphOptions& opts = {});

} // namespace sfm
} // namespace insight
