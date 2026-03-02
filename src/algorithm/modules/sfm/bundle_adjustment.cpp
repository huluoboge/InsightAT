/**
 * @file  bundle_adjustment.cpp
 * @brief Two-view and pose-only BA via Ceres (when INSIGHTAT_USE_CERES).
 */

#include "bundle_adjustment.h"
#include <cmath>

#if defined(INSIGHTAT_USE_CERES) && INSIGHTAT_USE_CERES
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#endif

namespace insight {
namespace sfm {

#if defined(INSIGHTAT_USE_CERES) && INSIGHTAT_USE_CERES

namespace {

// Reprojection cost for cam1 (identity): residual = project(X) - (u,v)
struct CostCam1 {
  double u, v, fx, fy, cx, cy;
  template <typename T>
  bool operator()(const T* const point, T* residuals) const {
    T x = point[0], y = point[1], z = point[2];
    if (z <= T(1e-12)) { residuals[0] = T(1e6); residuals[1] = T(1e6); return true; }
    T u_pred = fx * x / z + cx;
    T v_pred = fy * y / z + cy;
    residuals[0] = u_pred - T(u);
    residuals[1] = v_pred - T(v);
    return true;
  }
};

// Reprojection cost for cam2 (R, t): residual = project(R*X + t) - (u,v)
struct CostCam2 {
  double u, v, fx, fy, cx, cy;
  template <typename T>
  bool operator()(const T* const cam_aa, const T* const cam_t,
                  const T* const point, T* residuals) const {
    T p[3];
    ceres::AngleAxisRotatePoint(cam_aa, point, p);
    p[0] += cam_t[0]; p[1] += cam_t[1]; p[2] += cam_t[2];
    if (p[2] <= T(1e-12)) { residuals[0] = T(1e6); residuals[1] = T(1e6); return true; }
    T u_pred = T(fx) * p[0] / p[2] + T(cx);
    T v_pred = T(fy) * p[1] / p[2] + T(cy);
    residuals[0] = u_pred - T(u);
    residuals[1] = v_pred - T(v);
    return true;
  }
};

void rotationMatrixToAngleAxis(const Eigen::Matrix3d& R, double* aa) {
  Eigen::AngleAxisd aa_val(R);
  aa[0] = aa_val.axis()(0) * aa_val.angle();
  aa[1] = aa_val.axis()(1) * aa_val.angle();
  aa[2] = aa_val.axis()(2) * aa_val.angle();
}

void angleAxisToRotationMatrix(const double* aa, Eigen::Matrix3d* R) {
  Eigen::AngleAxisd aa_val(
      Eigen::Vector3d(aa[0], aa[1], aa[2]).norm(),
      Eigen::Vector3d(aa[0], aa[1], aa[2]).normalized());
  *R = aa_val.toRotationMatrix();
}

}  // namespace

bool two_view_bundle(const TwoViewBAInput& input,
                     TwoViewBAResult* result,
                     int max_iterations) {
  if (!result || input.points3d.empty() || input.observations.empty() ||
      !input.fx || !input.fy) return false;
  const size_t n_points = input.points3d.size();
  double cam2_aa[3], cam2_t[3];
  rotationMatrixToAngleAxis(input.R, cam2_aa);
  cam2_t[0] = input.t(0); cam2_t[1] = input.t(1); cam2_t[2] = input.t(2);

  std::vector<double> points_flat(n_points * 3);
  for (size_t i = 0; i < n_points; ++i) {
    points_flat[i*3+0] = input.points3d[i](0);
    points_flat[i*3+1] = input.points3d[i](1);
    points_flat[i*3+2] = input.points3d[i](2);
  }

  ceres::Problem problem;
  const double fx = input.fx, fy = input.fy, cx = input.cx, cy = input.cy;
  ceres::LossFunction* loss = new ceres::HuberLoss(1.0);

  for (const auto& obs : input.observations) {
    if (obs.point_index < 0 || static_cast<size_t>(obs.point_index) >= n_points)
      continue;
    double* point_ptr = points_flat.data() + static_cast<size_t>(obs.point_index) * 3;
    if (obs.image_index == 0) {
      ceres::CostFunction* cost = new ceres::AutoDiffCostFunction<CostCam1, 2, 3>(
          new CostCam1{obs.u, obs.v, fx, fy, cx, cy});
      problem.AddResidualBlock(cost, loss, point_ptr);
    } else {
      ceres::CostFunction* cost = new ceres::AutoDiffCostFunction<CostCam2, 2, 3, 3, 3>(
          new CostCam2{obs.u, obs.v, fx, fy, cx, cy});
      problem.AddResidualBlock(cost, loss, cam2_aa, cam2_t, point_ptr);
    }
  }

  ceres::Solver::Options options;
  options.max_num_iterations = max_iterations;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  if (!summary.IsSolutionUsable()) return false;

  angleAxisToRotationMatrix(cam2_aa, &result->R);
  result->t = Eigen::Vector3d(cam2_t[0], cam2_t[1], cam2_t[2]);
  result->points3d.resize(n_points);
  for (size_t i = 0; i < n_points; ++i) {
    result->points3d[i] = Eigen::Vector3d(
        points_flat[i*3+0], points_flat[i*3+1], points_flat[i*3+2]);
  }
  result->success = true;
  result->rmse_px = std::sqrt(summary.final_cost * 2.0 / summary.num_residuals);
  result->num_valid_observations = static_cast<int>(summary.num_residuals / 2);
  return true;
}

bool pose_only_bundle(const std::vector<Eigen::Vector3d>& pts3d,
                      const std::vector<Eigen::Vector2d>& pts2d,
                      double fx, double fy, double cx, double cy,
                      const Eigen::Matrix3d& R_in,
                      const Eigen::Vector3d& t_in,
                      Eigen::Matrix3d* R_out,
                      Eigen::Vector3d* t_out,
                      double* rmse_px,
                      int max_iterations) {
  if (!R_out || !t_out || pts3d.size() != pts2d.size() || pts3d.empty()) return false;
  double cam_aa[3], cam_t[3];
  rotationMatrixToAngleAxis(R_in, cam_aa);
  cam_t[0] = t_in(0); cam_t[1] = t_in(1); cam_t[2] = t_in(2);

  ceres::Problem problem;
  ceres::LossFunction* loss = new ceres::HuberLoss(1.0);
  const size_t n = pts3d.size();
  for (size_t i = 0; i < n; ++i) {
    ceres::CostFunction* cost = new ceres::AutoDiffCostFunction<CostCam2, 2, 3, 3, 3>(
        new CostCam2{pts2d[i](0), pts2d[i](1), fx, fy, cx, cy});
    double pt[3] = {pts3d[i](0), pts3d[i](1), pts3d[i](2)};
    problem.AddResidualBlock(cost, loss, cam_aa, cam_t, pt);
    problem.SetParameterBlockConstant(pt);
  }

  ceres::Solver::Options options;
  options.max_num_iterations = max_iterations;
  options.linear_solver_type = ceres::DENSE_QR;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  if (!summary.IsSolutionUsable()) return false;

  angleAxisToRotationMatrix(cam_aa, R_out);
  *t_out = Eigen::Vector3d(cam_t[0], cam_t[1], cam_t[2]);
  if (rmse_px) *rmse_px = std::sqrt(summary.final_cost * 2.0 / summary.num_residuals);
  return true;
}

#else

bool two_view_bundle(const TwoViewBAInput&, TwoViewBAResult*, int) { return false; }

bool pose_only_bundle(const std::vector<Eigen::Vector3d>&,
                      const std::vector<Eigen::Vector2d>&,
                      double, double, double, double,
                      const Eigen::Matrix3d&, const Eigen::Vector3d&,
                      Eigen::Matrix3d*, Eigen::Vector3d*, double*, int) { return false; }

#endif

}  // namespace sfm
}  // namespace insight
