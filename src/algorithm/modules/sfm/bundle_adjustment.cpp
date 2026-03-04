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
  template <typename T> bool operator()(const T* const point, T* residuals) const {
    T x = point[0], y = point[1], z = point[2];
    if (z <= T(1e-12)) {
      residuals[0] = T(1e6);
      residuals[1] = T(1e6);
      return true;
    }
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
  bool operator()(const T* const cam_aa, const T* const cam_t, const T* const point,
                  T* residuals) const {
    T p[3];
    ceres::AngleAxisRotatePoint(cam_aa, point, p);
    p[0] += cam_t[0];
    p[1] += cam_t[1];
    p[2] += cam_t[2];
    if (p[2] <= T(1e-12)) {
      residuals[0] = T(1e6);
      residuals[1] = T(1e6);
      return true;
    }
    T u_pred = T(fx) * p[0] / p[2] + T(cx);
    T v_pred = T(fy) * p[1] / p[2] + T(cy);
    residuals[0] = u_pred - T(u);
    residuals[1] = v_pred - T(v);
    return true;
  }
};

// Global BA: reprojection for any camera (angle-axis + t + point). Legacy, no distortion.
struct CostGlobal {
  double u, v, fx, fy, cx, cy;
  template <typename T>
  bool operator()(const T* const cam_aa, const T* const cam_t, const T* const point,
                  T* residuals) const {
    T p[3];
    ceres::AngleAxisRotatePoint(cam_aa, point, p);
    p[0] += cam_t[0];
    p[1] += cam_t[1];
    p[2] += cam_t[2];
    if (p[2] <= T(1e-12)) {
      residuals[0] = T(1e6);
      residuals[1] = T(1e6);
      return true;
    }
    T u_pred = T(fx) * p[0] / p[2] + T(cx);
    T v_pred = T(fy) * p[1] / p[2] + T(cy);
    residuals[0] = u_pred - T(u);
    residuals[1] = v_pred - T(v);
    return true;
  }
};

// Multi-camera global BA: shared intrinsics+distortion block per camera.
// intr[9] = [fx, fy, cx, cy, k1, k2, k3, p1, p2]  (Brown-Conrady)
// Images with the same camera share the same intr pointer → shared Ceres parameter block.
struct CostGlobalMultiCam {
  double u, v;
  template <typename T>
  bool operator()(const T* const cam_aa, const T* const cam_t, const T* const intr,
                  const T* const point, T* residuals) const {
    T p[3];
    ceres::AngleAxisRotatePoint(cam_aa, point, p);
    p[0] += cam_t[0];
    p[1] += cam_t[1];
    p[2] += cam_t[2];
    if (p[2] <= T(1e-12)) {
      residuals[0] = T(1e6);
      residuals[1] = T(1e6);
      return true;
    }
    T xn = p[0] / p[2];
    T yn = p[1] / p[2];
    T r2 = xn * xn + yn * yn;
    T r4 = r2 * r2;
    T r6 = r4 * r2;
    // Brown-Conrady: radial + tangential
    T radial = T(1) + intr[4] * r2 + intr[5] * r4 + intr[6] * r6;
    T xd = xn * radial + T(2) * intr[7] * xn * yn + intr[8] * (r2 + T(2) * xn * xn);
    T yd = yn * radial + intr[7] * (r2 + T(2) * yn * yn) + T(2) * intr[8] * xn * yn;
    residuals[0] = intr[0] * xd + intr[2] - T(u);
    residuals[1] = intr[1] * yd + intr[3] - T(v);
    return true;
  }
};

// Distortion-only: pose and point fixed, optimize k1,k2. p = R*X+t, then radial.
struct CostDistortion {
  double u, v, fx, fy, cx, cy;
  double pwx, pwy, pwz; // point in camera frame (fixed)
  template <typename T> bool operator()(const T* const k1, const T* const k2, T* residuals) const {
    if (pwz <= 1e-12) {
      residuals[0] = T(1e6);
      residuals[1] = T(1e6);
      return true;
    }
    T xn = (T(pwx) / T(pwz) - T(cx)) / T(fx);
    T yn = (T(pwy) / T(pwz) - T(cy)) / T(fy);
    T r2 = xn * xn + yn * yn;
    T factor = T(1) + (*k1) * r2 + (*k2) * r2 * r2;
    T u_pred = T(cx) + T(fx) * xn * factor;
    T v_pred = T(cy) + T(fy) * yn * factor;
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
  Eigen::AngleAxisd aa_val(Eigen::Vector3d(aa[0], aa[1], aa[2]).norm(),
                           Eigen::Vector3d(aa[0], aa[1], aa[2]).normalized());
  *R = aa_val.toRotationMatrix();
}

} // namespace

bool two_view_bundle(const TwoViewBAInput& input, TwoViewBAResult* result, int max_iterations) {
  if (!result || input.points3d.empty() || input.observations.empty() || !input.fx || !input.fy)
    return false;
  const size_t n_points = input.points3d.size();
  double cam2_aa[3], cam2_t[3];
  rotationMatrixToAngleAxis(input.R, cam2_aa);
  cam2_t[0] = input.t(0);
  cam2_t[1] = input.t(1);
  cam2_t[2] = input.t(2);

  std::vector<double> points_flat(n_points * 3);
  for (size_t i = 0; i < n_points; ++i) {
    points_flat[i * 3 + 0] = input.points3d[i](0);
    points_flat[i * 3 + 1] = input.points3d[i](1);
    points_flat[i * 3 + 2] = input.points3d[i](2);
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
  if (!summary.IsSolutionUsable())
    return false;

  angleAxisToRotationMatrix(cam2_aa, &result->R);
  result->t = Eigen::Vector3d(cam2_t[0], cam2_t[1], cam2_t[2]);
  result->points3d.resize(n_points);
  for (size_t i = 0; i < n_points; ++i) {
    result->points3d[i] =
        Eigen::Vector3d(points_flat[i * 3 + 0], points_flat[i * 3 + 1], points_flat[i * 3 + 2]);
  }
  result->success = true;
  result->rmse_px = std::sqrt(summary.final_cost * 2.0 / summary.num_residuals);
  result->num_valid_observations = static_cast<int>(summary.num_residuals / 2);
  return true;
}

bool pose_only_bundle(const std::vector<Eigen::Vector3d>& pts3d,
                      const std::vector<Eigen::Vector2d>& pts2d, double fx, double fy, double cx,
                      double cy, const Eigen::Matrix3d& R_in, const Eigen::Vector3d& t_in,
                      Eigen::Matrix3d* R_out, Eigen::Vector3d* t_out, double* rmse_px,
                      int max_iterations) {
  if (!R_out || !t_out || pts3d.size() != pts2d.size() || pts3d.empty())
    return false;
  double cam_aa[3], cam_t[3];
  rotationMatrixToAngleAxis(R_in, cam_aa);
  cam_t[0] = t_in(0);
  cam_t[1] = t_in(1);
  cam_t[2] = t_in(2);

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
  if (!summary.IsSolutionUsable())
    return false;

  angleAxisToRotationMatrix(cam_aa, R_out);
  *t_out = Eigen::Vector3d(cam_t[0], cam_t[1], cam_t[2]);
  if (rmse_px)
    *rmse_px = std::sqrt(summary.final_cost * 2.0 / summary.num_residuals);
  return true;
}

bool global_bundle(const GlobalBAInput& input, GlobalBAResult* result, int max_iterations) {
  if (!result || input.points3d.empty() || input.observations.empty() ||
      input.poses_R.size() != input.poses_t.size() || input.poses_R.empty())
    return false;

  const int n_cams = static_cast<int>(input.poses_R.size());
  const size_t n_points = input.points3d.size();

  const bool use_multicam =
      !input.image_camera_index.empty() && !input.cameras.empty() &&
      input.image_camera_index.size() == static_cast<size_t>(n_cams);

  // Legacy mode requires at least valid shared intrinsics
  if (!use_multicam && !input.fx && !input.fy)
    return false;

  // ── Camera pose arrays (angle-axis + t) ───────────────────────────────────
  std::vector<double> cam_aa(static_cast<size_t>(n_cams) * 3);
  std::vector<double> cam_t(static_cast<size_t>(n_cams) * 3);
  for (int i = 0; i < n_cams; ++i) {
    rotationMatrixToAngleAxis(input.poses_R[static_cast<size_t>(i)],
                              &cam_aa[static_cast<size_t>(i) * 3]);
    cam_t[static_cast<size_t>(i) * 3 + 0] = input.poses_t[static_cast<size_t>(i)](0);
    cam_t[static_cast<size_t>(i) * 3 + 1] = input.poses_t[static_cast<size_t>(i)](1);
    cam_t[static_cast<size_t>(i) * 3 + 2] = input.poses_t[static_cast<size_t>(i)](2);
  }

  // ── 3D points ─────────────────────────────────────────────────────────────
  std::vector<double> points_flat(n_points * 3);
  for (size_t i = 0; i < n_points; ++i) {
    points_flat[i * 3 + 0] = input.points3d[i](0);
    points_flat[i * 3 + 1] = input.points3d[i](1);
    points_flat[i * 3 + 2] = input.points3d[i](2);
  }

  ceres::Problem problem;
  ceres::LossFunction* loss = new ceres::HuberLoss(1.0);

  if (use_multicam) {
    // ── Multi-camera path: each distinct camera has ONE shared intrinsics block ──
    // Layout: intr[9] = [fx, fy, cx, cy, k1, k2, k3, p1, p2]
    const int n_distinct = static_cast<int>(input.cameras.size());
    std::vector<double> intr_params(static_cast<size_t>(n_distinct) * 9);
    for (int c = 0; c < n_distinct; ++c) {
      const auto& K = input.cameras[static_cast<size_t>(c)];
      double* p = intr_params.data() + static_cast<size_t>(c) * 9;
      p[0] = K.fx; p[1] = K.fy; p[2] = K.cx; p[3] = K.cy;
      p[4] = K.k1; p[5] = K.k2; p[6] = K.k3; p[7] = K.p1; p[8] = K.p2;
    }

    for (const auto& obs : input.observations) {
      if (obs.image_index < 0 || obs.image_index >= n_cams || obs.point_index < 0 ||
          static_cast<size_t>(obs.point_index) >= n_points)
        continue;
      const int cam_idx = input.image_camera_index[static_cast<size_t>(obs.image_index)];
      if (cam_idx < 0 || cam_idx >= n_distinct)
        continue;
      double* aa_ptr = cam_aa.data() + static_cast<size_t>(obs.image_index) * 3;
      double* t_ptr  = cam_t.data()  + static_cast<size_t>(obs.image_index) * 3;
      double* intr_ptr = intr_params.data() + static_cast<size_t>(cam_idx) * 9;
      double* pt_ptr = points_flat.data() + static_cast<size_t>(obs.point_index) * 3;
      ceres::CostFunction* cost =
          new ceres::AutoDiffCostFunction<CostGlobalMultiCam, 2, 3, 3, 9, 3>(
              new CostGlobalMultiCam{obs.u, obs.v});
      problem.AddResidualBlock(cost, loss, aa_ptr, t_ptr, intr_ptr, pt_ptr);
    }

    // Fix cam0 (world origin)
    problem.SetParameterBlockConstant(cam_aa.data());
    problem.SetParameterBlockConstant(cam_t.data());

    // Fix intrinsics when not optimizing them
    if (!input.optimize_intrinsics) {
      for (int c = 0; c < n_distinct; ++c)
        problem.SetParameterBlockConstant(intr_params.data() + static_cast<size_t>(c) * 9);
    }

    ceres::Solver::Options options;
    options.max_num_iterations = max_iterations;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    if (!summary.IsSolutionUsable())
      return false;

    result->poses_R.resize(static_cast<size_t>(n_cams));
    result->poses_t.resize(static_cast<size_t>(n_cams));
    for (int i = 0; i < n_cams; ++i) {
      angleAxisToRotationMatrix(&cam_aa[static_cast<size_t>(i) * 3],
                                &result->poses_R[static_cast<size_t>(i)]);
      result->poses_t[static_cast<size_t>(i)] = Eigen::Vector3d(
          cam_t[static_cast<size_t>(i) * 3 + 0], cam_t[static_cast<size_t>(i) * 3 + 1],
          cam_t[static_cast<size_t>(i) * 3 + 2]);
    }
    result->points3d.resize(n_points);
    for (size_t i = 0; i < n_points; ++i)
      result->points3d[i] = Eigen::Vector3d(points_flat[i * 3 + 0], points_flat[i * 3 + 1],
                                            points_flat[i * 3 + 2]);

    // Read back (possibly optimized) camera intrinsics
    result->cameras.resize(static_cast<size_t>(n_distinct));
    for (int c = 0; c < n_distinct; ++c) {
      const double* p = intr_params.data() + static_cast<size_t>(c) * 9;
      auto& K = result->cameras[static_cast<size_t>(c)];
      K.fx = p[0]; K.fy = p[1]; K.cx = p[2]; K.cy = p[3];
      K.k1 = p[4]; K.k2 = p[5]; K.k3 = p[6]; K.p1 = p[7]; K.p2 = p[8];
    }

    result->success = true;
    result->num_residuals = static_cast<int>(summary.num_residuals);
    result->rmse_px = (summary.num_residuals > 0)
                          ? std::sqrt(summary.final_cost * 2.0 / summary.num_residuals)
                          : 0.0;
    return true;
  }

  // ── Legacy path: shared or per-image scalar intrinsics, no distortion ─────
  const bool use_per_camera =
      input.fx_per_camera.size() == static_cast<size_t>(n_cams) &&
      input.fy_per_camera.size() == static_cast<size_t>(n_cams) &&
      input.cx_per_camera.size() == static_cast<size_t>(n_cams) &&
      input.cy_per_camera.size() == static_cast<size_t>(n_cams);

  for (const auto& obs : input.observations) {
    if (obs.image_index < 0 || obs.image_index >= n_cams || obs.point_index < 0 ||
        static_cast<size_t>(obs.point_index) >= n_points)
      continue;
    double fx = input.fx, fy = input.fy, cx = input.cx, cy = input.cy;
    if (use_per_camera) {
      const size_t c = static_cast<size_t>(obs.image_index);
      fx = input.fx_per_camera[c];
      fy = input.fy_per_camera[c];
      cx = input.cx_per_camera[c];
      cy = input.cy_per_camera[c];
    }
    double* cam_aa_ptr = cam_aa.data() + static_cast<size_t>(obs.image_index) * 3;
    double* cam_t_ptr  = cam_t.data()  + static_cast<size_t>(obs.image_index) * 3;
    double* point_ptr  = points_flat.data() + static_cast<size_t>(obs.point_index) * 3;
    ceres::CostFunction* cost = new ceres::AutoDiffCostFunction<CostGlobal, 2, 3, 3, 3>(
        new CostGlobal{obs.u, obs.v, fx, fy, cx, cy});
    problem.AddResidualBlock(cost, loss, cam_aa_ptr, cam_t_ptr, point_ptr);
  }
  problem.SetParameterBlockConstant(cam_aa.data());
  problem.SetParameterBlockConstant(cam_t.data());

  ceres::Solver::Options options;
  options.max_num_iterations = max_iterations;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  if (!summary.IsSolutionUsable())
    return false;

  result->poses_R.resize(static_cast<size_t>(n_cams));
  result->poses_t.resize(static_cast<size_t>(n_cams));
  for (int i = 0; i < n_cams; ++i) {
    angleAxisToRotationMatrix(&cam_aa[static_cast<size_t>(i) * 3],
                              &result->poses_R[static_cast<size_t>(i)]);
    result->poses_t[static_cast<size_t>(i)] = Eigen::Vector3d(
        cam_t[static_cast<size_t>(i) * 3 + 0], cam_t[static_cast<size_t>(i) * 3 + 1],
        cam_t[static_cast<size_t>(i) * 3 + 2]);
  }
  result->points3d.resize(n_points);
  for (size_t i = 0; i < n_points; ++i)
    result->points3d[i] =
        Eigen::Vector3d(points_flat[i * 3 + 0], points_flat[i * 3 + 1], points_flat[i * 3 + 2]);
  result->success = true;
  result->num_residuals = static_cast<int>(summary.num_residuals);
  result->rmse_px = (summary.num_residuals > 0)
                        ? std::sqrt(summary.final_cost * 2.0 / summary.num_residuals)
                        : 0.0;
  return true;
}

bool distortion_only_bundle(const std::vector<Eigen::Matrix3d>& poses_R,
                            const std::vector<Eigen::Vector3d>& poses_t,
                            const std::vector<Eigen::Vector3d>& points3d,
                            const std::vector<GlobalObservation>& observations, double fx,
                            double fy, double cx, double cy, std::vector<double>* k1_per_camera,
                            std::vector<double>* k2_per_camera, double* rmse_px,
                            int max_iterations) {
  if (!k1_per_camera || !k2_per_camera || poses_R.size() != poses_t.size() ||
      observations.empty() || points3d.empty())
    return false;
  const size_t n_cams = poses_R.size();
  k1_per_camera->resize(n_cams, 0.0);
  k2_per_camera->resize(n_cams, 0.0);

  std::vector<double> p_cam_x(observations.size()), p_cam_y(observations.size()),
      p_cam_z(observations.size());
  for (size_t i = 0; i < observations.size(); ++i) {
    const auto& o = observations[i];
    if (o.image_index < 0 || static_cast<size_t>(o.image_index) >= n_cams || o.point_index < 0 ||
        static_cast<size_t>(o.point_index) >= points3d.size())
      continue;
    const Eigen::Vector3d P =
        poses_R[static_cast<size_t>(o.image_index)] * points3d[static_cast<size_t>(o.point_index)] +
        poses_t[static_cast<size_t>(o.image_index)];
    p_cam_x[i] = P(0);
    p_cam_y[i] = P(1);
    p_cam_z[i] = P(2);
  }

  ceres::Problem problem;
  ceres::LossFunction* loss = new ceres::HuberLoss(1.0);
  for (size_t i = 0; i < observations.size(); ++i) {
    const auto& o = observations[i];
    if (o.image_index < 0 || static_cast<size_t>(o.image_index) >= n_cams || o.point_index < 0 ||
        static_cast<size_t>(o.point_index) >= points3d.size())
      continue;
    const int cam = o.image_index;
    double* k1 = k1_per_camera->data() + cam;
    double* k2 = k2_per_camera->data() + cam;
    ceres::CostFunction* cost = new ceres::AutoDiffCostFunction<CostDistortion, 2, 1, 1>(
        new CostDistortion{o.u, o.v, fx, fy, cx, cy, p_cam_x[i], p_cam_y[i], p_cam_z[i]});
    problem.AddResidualBlock(cost, loss, k1, k2);
  }

  ceres::Solver::Options options;
  options.max_num_iterations = max_iterations;
  options.linear_solver_type = ceres::DENSE_QR;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  if (!summary.IsSolutionUsable())
    return false;
  if (rmse_px && summary.num_residuals > 0)
    *rmse_px = std::sqrt(summary.final_cost * 2.0 / summary.num_residuals);
  return true;
}

#else

bool two_view_bundle(const TwoViewBAInput&, TwoViewBAResult*, int) { return false; }

bool pose_only_bundle(const std::vector<Eigen::Vector3d>&, const std::vector<Eigen::Vector2d>&,
                      double, double, double, double, const Eigen::Matrix3d&,
                      const Eigen::Vector3d&, Eigen::Matrix3d*, Eigen::Vector3d*, double*, int) {
  return false;
}

bool global_bundle(const GlobalBAInput&, GlobalBAResult*, int) { return false; }

bool distortion_only_bundle(const std::vector<Eigen::Matrix3d>&,
                            const std::vector<Eigen::Vector3d>&,
                            const std::vector<Eigen::Vector3d>&,
                            const std::vector<GlobalObservation>&, double, double, double, double,
                            std::vector<double>*, std::vector<double>*, double*, int) {
  return false;
}

#endif

} // namespace sfm
} // namespace insight
