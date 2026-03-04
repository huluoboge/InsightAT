/**
 * two_view_reconstruction.h
 * InsightAT – Two-View Geometric Reconstruction
 *
 * Provides (pure CPU / Eigen) algorithms used by isat_twoview and isat_calibrate:
 *
 *   1. FocalFromFundamental  – Hartley-like 1-D optimisation: find f so that
 *                              E = K^T F K satisfies the essential-matrix constraint
 *                              (σ₁ = σ₂, σ₃ = 0).
 *
 *   2. EnforceEssential      – Project arbitrary 3×3 matrix onto the essential-
 *                              matrix manifold (σ₁=σ₂=1, σ₃=0).
 *
 *   3. DecomposeEssential    – Return the unique (R, t) from the four E-decomposition
 *                              candidates using a cheirality test on triangulated points.
 *
 *   4. TriangulateLinear     – DLT-based linear triangulation in normalised coordinates.
 *
 * Bundle adjustment (TwoViewBA) has been removed.  All BA is GPU-only via
 * gpu_twoview_sfm.h (gpu_ba_residuals).
 *
 * All functions are in namespace insight::sfm.
 * No GPU / Qt dependency.
 */
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace insight {
namespace sfm {

// ─────────────────────────────────────────────────────────────────────────────
// Structs
// ─────────────────────────────────────────────────────────────────────────────

/// Pinhole intrinsics (principal point at (cx, cy), no distortion)
struct Intrinsics {
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;

  bool valid() const { return fx > 0.0 && fy > 0.0; }

  /// Build camera matrix K
  Eigen::Matrix3d K() const {
    Eigen::Matrix3d m;
    m << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
    return m;
  }
  /// Build K⁻¹ exactly
  Eigen::Matrix3d Kinv() const {
    Eigen::Matrix3d m;
    m << 1.0 / fx, 0.0, -cx / fx, 0.0, 1.0 / fy, -cy / fy, 0.0, 0.0, 1.0;
    return m;
  }
};

/// Result of a single two-view reconstruction
struct TwoViewResult {
  bool success = false;

  Eigen::Matrix3d R; ///< Rotation: X_cam2 = R * X_cam1 frame shift
  Eigen::Vector3d t; ///< Translation (unit norm)

  std::vector<Eigen::Vector3d> points3d; ///< Triangulated 3-D points (in cam1 frame)
  std::vector<int> inlier_ids;           ///< Original inlier match indices

  double focal_refined = 0.0;     ///< Refined focal length (px) after BA; 0 = not refined
  double reprojection_rmse = 0.0; ///< Final RMSE after BA (px)
  int num_valid_points = 0;
  std::string quality; ///< "good" / "poor" / "degenerate"
};

/// Degeneracy detection result (F vs H model selection, COLMAP-style)
struct DegeneracyResult {
  bool is_degenerate = false;  ///< true if scene is planar/pure-rotation, F unreliable
  int model_preferred = 0;     ///< 0=general (F), 1=planar (H)
  double h_over_f_ratio = 0.0; ///< H_inliers / F_inliers (when H estimated)
};

/// Stability metrics for two-view triangulation
struct StabilityMetrics {
  double median_parallax_deg = 0.0;   ///< Median parallax angle (degrees), >2° typically OK
  double median_depth_baseline = 0.0; ///< Median depth/baseline ratio, 2-500 typically OK
  int num_valid = 0;                  ///< Points used for metrics (positive depth both cams)
  bool is_stable = false;             ///< true if parallax and depth/baseline in acceptable range
};

// ─────────────────────────────────────────────────────────────────────────────
// 1. Focal estimation from F
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Estimate the common focal length f (in pixels) from a Fundamental matrix.
 *
 * Strategy: grid + golden-section search over f to minimise the "essential
 * matrix residual"  r(f) = |σ₁(E) - σ₂(E)|  where E = K^T F K and
 * K = diag(f, f, 1) with pp at (cx, cy).
 *
 * @param F      Row-major 3×3 Fundamental matrix (double).
 * @param cx, cy Principal point (pixels).
 * @param f_min, f_max  Search range (pixels). Defaults cover typical aerial cameras.
 * @return Estimated focal length (> 0) or -1 on failure.
 */
double focal_from_fundamental(const Eigen::Matrix3d& F, double cx, double cy, double f_min = 100.0,
                              double f_max = 50000.0);

// ─────────────────────────────────────────────────────────────────────────────
// 2. Enforce essential matrix constraint
// ─────────────────────────────────────────────────────────────────────────────

/// Project M (unnormalised 3×3) onto the essential-matrix manifold:
/// set singular values to (1, 1, 0).
Eigen::Matrix3d enforce_essential(const Eigen::Matrix3d& M);

// ─────────────────────────────────────────────────────────────────────────────
// 3. E decomposition + cheirality
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Decompose an essential matrix into (R, t) using a cheirality test.
 *
 * Given a set of normalised image correspondences, from the four candidate
 * (R, ±t) pairs linear triangulation is run and the solution where the
 * highest number of points are positive-depth in both cameras is returned.
 *
 * @param E         Normalised Essential matrix (K₂⁻ᵀ E K₁⁻¹ in pixel space).
 * @param pts1_n    Normalised coords in image 1  (x = (px - cx)/fx, etc.).
 * @param pts2_n    Normalised coords in image 2.
 * @param[out] R    Best rotation (3×3).
 * @param[out] t    Best translation (unit norm).
 * @return number of cheirality-consistent points for the chosen solution.
 */
int decompose_essential(const Eigen::Matrix3d& E, const std::vector<Eigen::Vector2d>& pts1_n,
                        const std::vector<Eigen::Vector2d>& pts2_n, Eigen::Matrix3d& R,
                        Eigen::Vector3d& t);

// ─────────────────────────────────────────────────────────────────────────────
// 4. Linear triangulation (DLT)
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Triangulate a single point from two normalised image rays.
 *
 * P1 = [I | 0],   P2 = [R | t]  (all in normalised / camera coords)
 *
 * @param x1  Normalised coord in cam1.
 * @param x2  Normalised coord in cam2.
 * @param R, t  Relative pose (cam2 w.r.t. cam1).
 * @return 3-D point in cam1 frame (homogeneous component w is 1), or
 *         (0,0,0) if the system is degenerate.
 */
Eigen::Vector3d triangulate_point(const Eigen::Vector2d& x1, const Eigen::Vector2d& x2,
                                  const Eigen::Matrix3d& R, const Eigen::Vector3d& t);

/**
 * Triangulate a set of normalised correspondences.
 * Points with negative depth in either camera are marked as (NaN, NaN, NaN).
 *
 * @param pts1_n, pts2_n  Normalised, same-length correspondences.
 * @param R, t            Relative pose.
 * @return vector of 3-D points in cam1 frame (same length as pts1_n).
 */
std::vector<Eigen::Vector3d> triangulate_points(const std::vector<Eigen::Vector2d>& pts1_n,
                                                const std::vector<Eigen::Vector2d>& pts2_n,
                                                const Eigen::Matrix3d& R, const Eigen::Vector3d& t);

// ─────────────────────────────────────────────────────────────────────────────
// 5. Degeneracy detection (F vs H, COLMAP-style)
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Detect whether two-view geometry is degenerate (planar / pure rotation).
 *
 * When H is estimated (--estimate-h): compare F vs H inliers. If H explains
 * the data as well as or better than F (by inlier ratio and AIC-like score),
 * the scene is likely planar → F/E unreliable.
 *
 * @param F_inliers   Inlier count for Fundamental matrix.
 * @param H_inliers   Inlier count for Homography (0 if H not estimated).
 * @param num_matches Total number of matches.
 * @return DegeneracyResult with is_degenerate, model_preferred.
 */
DegeneracyResult detect_degeneracy(int F_inliers, int H_inliers, int num_matches);

// ─────────────────────────────────────────────────────────────────────────────
// 6. Stability metrics (parallax, depth/baseline)
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Compute stability metrics for triangulated two-view geometry.
 *
 * Uses median parallax angle (ray夹角) and median depth/baseline ratio.
 * Typical stable range: parallax 2–10°, depth/baseline 2–500.
 *
 * @param points3d    Triangulated 3-D points in cam1 frame (same order as pts).
 * @param pts1_n      Normalised coords in cam1.
 * @param pts2_n      Normalised coords in cam2.
 * @param R, t        Relative pose (cam2 w.r.t. cam1).
 * @param min_parallax_deg     Min parallax (deg) for stability. Default 2.0.
 * @param min_depth_baseline   Min depth/B ratio. Default 2.0.
 * @param max_depth_baseline   Max depth/B ratio. Default 500.0.
 */
StabilityMetrics compute_stability_metrics(const std::vector<Eigen::Vector3d>& points3d,
                                           const std::vector<Eigen::Vector2d>& pts1_n,
                                           const std::vector<Eigen::Vector2d>& pts2_n,
                                           const Eigen::Matrix3d& R, const Eigen::Vector3d& t,
                                           double min_parallax_deg = 2.0,
                                           double min_depth_baseline = 2.0,
                                           double max_depth_baseline = 500.0);

// ─────────────────────────────────────────────────────────────────────────────
// 7. Convenience: convert row-major float[9] ↔ Eigen::Matrix3d
// ─────────────────────────────────────────────────────────────────────────────

inline Eigen::Matrix3d float_array_to_matrix3d(const float m[9]) {
  Eigen::Matrix3d R;
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      R(i, j) = static_cast<double>(m[i * 3 + j]);
  return R;
}

inline void matrix3d_to_float_array(const Eigen::Matrix3d& M, float out[9]) {
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      out[i * 3 + j] = static_cast<float>(M(i, j));
}

} // namespace sfm
} // namespace insight
