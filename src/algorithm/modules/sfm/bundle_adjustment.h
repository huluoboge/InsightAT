/**
 * @file  bundle_adjustment.h
 * @brief Ceres-based bundle adjustment for two-view and resection (pose-only).
 *
 * When INSIGHTAT_USE_CERES is defined, uses Ceres to optimize; otherwise stubs return false.
 * Pinhole model only (no distortion in this first layer).
 */

#pragma once

#include <Eigen/Core>
#include <vector>

namespace insight {
namespace sfm {

/// Single observation: point index, image index (0 or 1 for two-view), 2D pixel.
struct Observation2d {
  int point_index = 0;
  int image_index = 0;
  double u = 0.0;
  double v = 0.0;
};

/// Two-view BA input: 2 poses (cam1 = identity), N 3D points, 2N observations.
struct TwoViewBAInput {
  Eigen::Matrix3d R;   ///< Rotation of cam2 w.r.t. cam1
  Eigen::Vector3d t;  ///< Translation of cam2 w.r.t. cam1 (unit norm)
  std::vector<Eigen::Vector3d> points3d;
  std::vector<Observation2d> observations;  ///< size = sum over points of (2 obs per point)
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
};

/// Two-view BA result.
struct TwoViewBAResult {
  bool success = false;
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  std::vector<Eigen::Vector3d> points3d;
  double rmse_px = 0.0;
  int num_valid_observations = 0;
};

/**
 * Run two-view bundle adjustment (2 poses + 3D points).
 * Cam1 is fixed at identity; cam2 and all points are optimized.
 * @return true if optimization converged and result is filled.
 */
bool two_view_bundle(const TwoViewBAInput& input,
                     TwoViewBAResult* result,
                     int max_iterations = 50);

/**
 * Pose-only BA: fix 3D points, optimize single camera pose (for resection).
 * @param pts3d  Fixed 3D points (world frame = cam1 frame in two-view convention).
 * @param pts2d  Corresponding 2D observations (pixel).
 * @param K      fx, fy, cx, cy
 * @param R_in   Initial rotation (world to camera).
 * @param t_in   Initial translation (world to camera).
 * @param R_out  Output rotation.
 * @param t_out  Output translation.
 * @return true if converged.
 */
bool pose_only_bundle(const std::vector<Eigen::Vector3d>& pts3d,
                      const std::vector<Eigen::Vector2d>& pts2d,
                      double fx, double fy, double cx, double cy,
                      const Eigen::Matrix3d& R_in,
                      const Eigen::Vector3d& t_in,
                      Eigen::Matrix3d* R_out,
                      Eigen::Vector3d* t_out,
                      double* rmse_px = nullptr,
                      int max_iterations = 30);

}  // namespace sfm
}  // namespace insight
