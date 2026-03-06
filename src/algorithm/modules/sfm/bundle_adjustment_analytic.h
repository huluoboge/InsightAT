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

#include "bundle_adjustment.h" // GlobalBAInput, GlobalBAResult

#include <ceres/ceres.h>

namespace insight {
namespace sfm {

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
 * Global BA with analytic Jacobians and quaternion rotation parameterization.
 *
 * Accepts the same GlobalBAInput / GlobalBAResult as global_bundle().
 * Multi-camera path only: requires image_camera_index + cameras to be set.
 *
 * Intrinsics mapping:
 *   camera::Intrinsics{fx, fy, …} → intr[kFx] = fx, intr[kSigma] = fy/fx.
 *   After BA: result.cameras[c].fx = optimised fx, .fy = sigma * fx.
 *
 * @param input          BA inputs (poses, points, observations, cameras).
 * @param result         Output: optimised poses, points, cameras, RMSE.
 * @param max_iterations Ceres solver iteration limit.
 * @param fix_sigma      If true, sigma is held constant (fy/fx ratio fixed).
 * @return true if Ceres reports a usable solution.
 */
bool global_bundle_analytic(const GlobalBAInput& input, GlobalBAResult* result,
                            int max_iterations = 50, bool fix_sigma = true);

} // namespace sfm
} // namespace insight
