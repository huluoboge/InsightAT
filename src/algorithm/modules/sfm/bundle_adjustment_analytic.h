/**
 * @file  bundle_adjustment_analytic.h
 * @brief Analytic-Jacobian BA with quaternion rotation and sigma focal-ratio.
 *
 * Camera model
 * ─────────────
 *  Intrinsics: fx, sigma (fy = sigma * fx), cx, cy,
 *              Brown-Conrady 5-parameter distortion (k1, k2, k3, p1, p2).
 *
 *  Tangential distortion follows the Bentley convention (camera_utils.cpp):
 *    tang_x = 2·p2·xu·yu + p1·(r² + 2·xu²)
 *    tang_y = 2·p1·xu·yu + p2·(r² + 2·yu²)
 *
 *  Full projection (world → pixel):
 *    Xw  = X − C
 *    Xc  = R(q) · Xw          (q = unit quaternion [qx,qy,qz,qw])
 *    xu  = xc / zc ,  yu = yc / zc
 *    r²  = xu² + yu²
 *    dx  = xu·(1+k1·r²+k2·r⁴+k3·r⁶) + tang_x
 *    dy  = yu·(1+k1·r²+k2·r⁴+k3·r⁶) + tang_y
 *    u   = fx · dx + cx
 *    v   = sigma · fx · dy + cy
 *
 * Parameter blocks per residual
 * ──────────────────────────────
 *  intr[9] = [fx, sigma, cx, cy, k1, k2, k3, p1, p2]
 *  pose[7] = [qx, qy, qz, qw, Cx, Cy, Cz]
 *  pt[3]   = [X, Y, Z]
 *
 * When sigma is fixed to 1, the model degenerates to a single focal length.
 */

#pragma once

#include "../camera/camera_types.h"
#include <Eigen/Core>
#include <ceres/ceres.h>
#include <cstdint>
#include <vector>

namespace insight {
namespace sfm {

// ─── BA types (compact input/result for analytic BA) ────────────────────────

/// Soft prior on distance between two camera centres (BA image indices).
/// Residual = sqrt(weight) · (‖C_a − C_b‖ − d₀) / d₀  (dimensionless). d₀ may be 1.0 for a unit
/// baseline (gauge) when the anchor pose is fixed.
/// With one pose fixed via fix_pose (anchor), this adds a weak constraint on metric scale drift
/// along the baseline; it does not replace a full similarity gauge fix by itself.
struct BACameraDistancePrior {
  int image_index_a = 0; ///< BA image index (same convention as BAObservation.image_index).
  int image_index_b = 0;
  double distance_m = 0.0; ///< Target ‖C_a − C_b‖ (metres or same unit as poses_C).
  double weight = 0.0;   ///< If ≤ 0, ignored. Quadratic cost uses weight as σ⁻² scale (see impl).
};

/// Single observation for BA: image index, point index, 2D pixel, optional scale for weighting.
/// Residual weight = 1/scale (scale > 0); scale <= 0 or omitted is treated as 1.0 (unit weight).
struct BAObservation {
  int image_index = 0;
  int point_index = 0;
  double u = 0.0;
  double v = 0.0;
  double scale = 1.0;  ///< Observation scale; weight in cost = 1/scale (e.g. SIFT sigma or feature scale).
};

/// Bitmask for per-parameter fix of intrinsics (matches analytic layout: fx, sigma, cx, cy, k1, k2, k3, p1, p2).
enum FixIntrinsicsMask : uint32_t {
  kFixIntrFx    = 1u << 0,
  kFixIntrSigma = 1u << 1,
  kFixIntrCx    = 1u << 2,
  kFixIntrCy    = 1u << 3,
  kFixIntrK1    = 1u << 4,
  kFixIntrK2    = 1u << 5,
  kFixIntrK3    = 1u << 6,
  kFixIntrP1    = 1u << 7,
  kFixIntrP2    = 1u << 8,
  kFixIntrAll   = (1u << 9) - 1u
};

/// BA input: N images (poses), M points, multi-camera intrinsics. Compact layout.
/// Gauge: set fix_pose[i]=true on exactly one (or more for local BA constants) camera(s).
/// Optional camera_distance_priors (see BACameraDistancePrior) can stabilise scale with a fixed anchor.
/// image_camera_index[i] = camera index for image i.
struct BAInput {
  std::vector<Eigen::Matrix3d> poses_R;
  std::vector<Eigen::Vector3d> poses_C; ///< Camera centres in world coords (C = -Rᵀ·t)
  std::vector<Eigen::Vector3d> points3d;
  std::vector<BAObservation> observations;

  std::vector<int> image_camera_index;
  std::vector<camera::Intrinsics> cameras;
  bool optimize_intrinsics = false;

  std::vector<bool> fix_pose;
  std::vector<uint32_t> fix_intrinsics_flags;
  std::vector<bool> fix_point;
  double focal_prior_weight = 0.0; ///< If > 0, adds a Tikhonov prior on fx: residual = sqrt(w)·(fx−fx₀)/fx₀ per camera.

  /// Optional: soft priors ‖C_a−C_b‖ = distance_m (see BACameraDistancePrior). Empty = none.
  std::vector<BACameraDistancePrior> camera_distance_priors;

  // ── Optional Ceres solver overrides (0 / 0.0 = use built-in defaults) ──────
  double huber_loss_delta              = 4.0; ///< Huber loss δ (px). Applied to all residual blocks via ceres::HuberLoss(δ).
  int    solver_max_num_iterations     = 0;   ///< Override max_num_iterations; 0 = use the parameter passed to global_bundle_analytic.
  double solver_gradient_tolerance     = 0.0; ///< Ceres gradient_tolerance.  0 = Ceres default.
  double solver_function_tolerance     = 0.0; ///< Ceres function_tolerance.  0 = Ceres default.
  double solver_parameter_tolerance    = 0.0; ///< Ceres parameter_tolerance. 0 = Ceres default.
  int    solver_dense_schur_max_variable_cams = 0; ///< DENSE↔SPARSE Schur threshold. 0 = built-in default (30).
  int num_threads = 0; ///< Ceres num_threads. 0 = auto-select based on hardware_concurrency.
  /// Tikhonov regularization lambda. 0 = disabled (default, backward-compatible).
  /// When > 0, adds L2 penalty residuals sqrt(lambda)*(x-x0) for each non-fixed pose and
  /// intrinsics parameter block, ensuring Hessian positive-definiteness for CHOLMOD.
  /// Passed from BASolverOverrides::tikhonov_lambda via run_global_ba.
  double tikhonov_lambda = 0.0;
};

struct BAResult {
  bool success = false;
  std::vector<Eigen::Matrix3d> poses_R;
  std::vector<Eigen::Vector3d> poses_C;
  std::vector<Eigen::Vector3d> points3d;
  std::vector<camera::Intrinsics> cameras;
  double rmse_px = 0.0;
  int num_residuals = 0;
};

// ─── Analytic cost and solver ───────────────────────────────────────────────

inline constexpr int kAnalyticIntrCount = 9;

enum AnalyticIntrIdx : int {
  kFx    = 0,
  kSigma = 1,
  kCx    = 2,
  kCy    = 3,
  kK1    = 4,
  kK2    = 5,
  kK3    = 6,
  kP1    = 7,
  kP2    = 8,
};

/**
 * Analytic reprojection cost: 2 residuals, parameter blocks intr(9) / pose(7) / pt(3).
 *
 * All Jacobians (intrinsics, pose, point) are hand-derived from the projection chain.
 * The rotation matrix is computed with the q²-form so that the Sola quaternion
 * Jacobian is consistent with the forward pass even for non-unit perturbations.
 */
class ReprojectionCostAnalytic
    : public ceres::SizedCostFunction<2, kAnalyticIntrCount, 7, 3> {
public:
  ReprojectionCostAnalytic(double u_obs, double v_obs)
      : u_obs_(u_obs), v_obs_(v_obs) {}

  bool Evaluate(double const* const* params, double* residuals,
                double** jacobians) const override;

private:
  double u_obs_;
  double v_obs_;
};

/**
 * Same as ReprojectionCostAnalytic but residuals and Jacobians are scaled by 1/scale
 * (observation weight). scale must be > 0; typically feature scale or sigma (weight = 1/scale).
 */
class ReprojectionCostAnalyticWeighted
    : public ceres::SizedCostFunction<2, kAnalyticIntrCount, 7, 3> {
public:
  ReprojectionCostAnalyticWeighted(double u_obs, double v_obs, double scale)
      : u_obs_(u_obs), v_obs_(v_obs), weight_(scale > 1e-12 ? 1.0 / scale : 1.0) {}

  bool Evaluate(double const* const* params, double* residuals,
                double** jacobians) const override;

private:
  double u_obs_;
  double v_obs_;
  double weight_;
};

/**
 * BA with analytic Jacobians and quaternion rotation parameterization.
 *
 * Multi-camera path only: requires image_camera_index + cameras to be set.
 *
 * Intrinsics mapping:
 *   camera::Intrinsics{fx, fy, …} → intr[kFx] = fx, intr[kSigma] = fy/fx.
 *   After BA: result.cameras[c].fx = optimised fx, .fy = sigma * fx.
 *
 * Per-parameter fix: use input.fix_intrinsics_flags[c] (FixIntrinsicsMask bitmask).
 * Bit set = that parameter fixed. Empty fix_intrinsics_flags = optimise all when optimize_intrinsics.
 *
 * @param input          BA inputs (poses, points, observations, cameras, fix_*).
 * @param result         Output: optimised poses, points, cameras, RMSE.
 * @param max_iterations Ceres solver iteration limit.
 * @return true if Ceres reports a usable solution.
 */
bool global_bundle_analytic(const BAInput& input, BAResult* result, int max_iterations = 50);

// ─── Tikhonov regularization cost functions (exposed for unit testing) ───────

/**
 * Diagonal L2 regularization for a 7-DOF pose block.
 *
 *   residual[i] = sqrt(lambda) * (pose[i] - pose0[i]),  i = 0..6
 *
 * Jacobian: 7×7 diagonal matrix with sqrt(lambda) on the diagonal.
 * Ensures the Hessian diagonal is positive-definite for CHOLMOD when the scene
 * geometry is near-degenerate (collinear cameras, planar scenes, small baseline).
 */
class TikhonovPoseCost : public ceres::SizedCostFunction<7, 7> {
public:
  TikhonovPoseCost(const double* pose_snapshot, double lambda);
  bool Evaluate(double const* const* params, double* residuals,
                double** jacobians) const override;
private:
  double pose0_[7];
  double sqrt_lambda_;
};

/**
 * Diagonal L2 regularization for a kAnalyticIntrCount-DOF intrinsics block.
 *
 *   residual[i] = sqrt(lambda) * (intr[i] - intr0[i]),  i = 0..8
 *
 * Jacobian: 9×9 diagonal matrix with sqrt(lambda) on the diagonal.
 */
class TikhonovIntrCost : public ceres::SizedCostFunction<kAnalyticIntrCount, kAnalyticIntrCount> {
public:
  TikhonovIntrCost(const double* intr_snapshot, double lambda);
  bool Evaluate(double const* const* params, double* residuals,
                double** jacobians) const override;
private:
  double intr0_[kAnalyticIntrCount];
  double sqrt_lambda_;
};

} // namespace sfm
} // namespace insight
