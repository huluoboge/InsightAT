/**
 * @file  bundle_adjustment_analytic_test.cpp
 * @brief Unit tests for ReprojectionCostAnalytic, ReprojectionCostAnalyticWeighted, global_bundle_analytic.
 *
 * Tests
 * ──────
 *  1. Zero-noise residual check (distorted camera, exact observations → residual ≈ 0).
 *  2. Jacobian gradient check (analytic vs. numeric-diff via ceres::GradientChecker).
 *  3. Convergence on noisy single-camera data (4 views, 100 points).
 *  4. Multi-camera convergence (2 cameras, 6 views, 200 pts).
 *  5. Zero-noise residual with scale (weighted cost 1/scale).
 *  6. Jacobian gradient check for weighted cost.
 *  7. Two-view one camera (2 images, same intrinsics via image_to_camera).
 *  8. Two-view two cameras (different intrinsics).
 *  9. Fix pose and fix point (constant parameter blocks).
 *
 * Build: test_ba_analytic (see sfm/CMakeLists.txt).
 */

#include "bundle_adjustment_analytic.h"
#include "bundle_adjustment_analytic.h"
#include "../camera/camera_types.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ceres/ceres.h>
#include <ceres/gradient_checker.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

using insight::camera::Intrinsics;
using namespace insight::sfm;

// ─────────────────────────────────────────────────────────────────────────────
// Projection helper (q²-form R, Bentley distortion) – same model as cost func
// ─────────────────────────────────────────────────────────────────────────────

static bool project(const double* intr, const double* pose, const double* pt,
                    double uv[2]) {
  const double fx    = intr[kFx];
  const double sigma = intr[kSigma];
  const double cx    = intr[kCx], cy = intr[kCy];
  const double k1    = intr[kK1], k2 = intr[kK2], k3 = intr[kK3];
  const double p1    = intr[kP1], p2 = intr[kP2];
  const double fy    = sigma * fx;

  const double qx = pose[0], qy = pose[1], qz = pose[2], qw = pose[3];
  const double xx = qx*qx, yy = qy*qy, zz = qz*qz, ww = qw*qw;
  const double _xy = qx*qy, _xz = qx*qz, _xw = qx*qw;
  const double _yz = qy*qz, _yw = qy*qw, _zw = qz*qw;

  const double R00 = ww+xx-yy-zz, R01 = 2*(_xy-_zw), R02 = 2*(_xz+_yw);
  const double R10 = 2*(_xy+_zw), R11 = ww-xx+yy-zz, R12 = 2*(_yz-_xw);
  const double R20 = 2*(_xz-_yw), R21 = 2*(_yz+_xw), R22 = ww-xx-yy+zz;

  const double Xwx = pt[0] - pose[4], Xwy = pt[1] - pose[5], Xwz = pt[2] - pose[6];
  const double xc = R00*Xwx + R01*Xwy + R02*Xwz;
  const double yc = R10*Xwx + R11*Xwy + R12*Xwz;
  const double zc = R20*Xwx + R21*Xwy + R22*Xwz;
  if (zc < 1e-12) return false;

  const double xu = xc / zc, yu = yc / zc;
  const double xu2 = xu*xu, yu2 = yu*yu;
  const double r2 = xu2 + yu2, r4 = r2*r2, r6 = r4*r2;
  const double rad = 1.0 + k1*r2 + k2*r4 + k3*r6;
  const double dx  = xu*rad + 2.0*p2*xu*yu + p1*(r2 + 2.0*xu2);
  const double dy  = yu*rad + 2.0*p1*xu*yu + p2*(r2 + 2.0*yu2);
  uv[0] = fx*dx + cx;
  uv[1] = fy*dy + cy;
  return true;
}

// Build (R, t) for a camera at position C looking toward the origin.
static void look_at_origin(const Eigen::Vector3d& C,
                           Eigen::Matrix3d* R_out, Eigen::Vector3d* t_out) {
  const Eigen::Vector3d z = (Eigen::Vector3d::Zero() - C).normalized();
  Eigen::Vector3d up(0, 1, 0);
  if (std::abs(z.dot(up)) > 0.95) up = Eigen::Vector3d(0, 0, 1);
  const Eigen::Vector3d x = up.cross(z).normalized();
  const Eigen::Vector3d y = z.cross(x);
  R_out->row(0) = x.transpose();
  R_out->row(1) = y.transpose();
  R_out->row(2) = z.transpose();
  *t_out = -(*R_out) * C;
}

// (R, t) → pose[7] = [qx, qy, qz, qw, Cx, Cy, Cz]
static void Rt_to_pose(const Eigen::Matrix3d& R, const Eigen::Vector3d& t,
                       double pose[7]) {
  const Eigen::Vector3d C = -R.transpose() * t;
  const Eigen::Quaterniond q(R);
  pose[0] = q.x(); pose[1] = q.y(); pose[2] = q.z(); pose[3] = q.w();
  pose[4] = C.x(); pose[5] = C.y(); pose[6] = C.z();
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 1 – Zero-noise residual
// ─────────────────────────────────────────────────────────────────────────────

static int test_zero_noise_residual() {
  std::cout << "[Test 1] Zero-noise residual check\n";

  double intr[kAnalyticIntrCount] = {800.0, 1.0, 320.0, 240.0,
                                      0.01, -0.002, 0.0005, 0.001, 0.001};
  double pose[7] = {0,0,0,1,  0,0,0}; // identity, centre at origin
  double pt[3]   = {0.5, -0.3, 5.0};

  double uv[2];
  if (!project(intr, pose, pt, uv)) { std::cerr << "  projection failed\n"; return 1; }

  double res[2];
  const double* params[3] = {intr, pose, pt};
  ReprojectionCostAnalytic cost(uv[0], uv[1]);
  cost.Evaluate(params, res, nullptr);

  if (std::abs(res[0]) > 1e-8 || std::abs(res[1]) > 1e-8) {
    std::cerr << "  FAIL: residuals = (" << res[0] << ", " << res[1] << ")\n";
    return 1;
  }
  std::cout << "  PASS\n";
  return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 2 – Gradient (Jacobian) check
// ─────────────────────────────────────────────────────────────────────────────

static int test_jacobian_gradient_check() {
  std::cout << "[Test 2] Jacobian gradient check (analytic vs. numeric)\n";

  double intr[kAnalyticIntrCount] = {600.0, 1.02, 400.0, 300.0,
                                      0.05, -0.01, 0.001, 0.002, -0.001};
  const Eigen::AngleAxisd aa(0.26, Eigen::Vector3d(0.3, 1.0, 0.1).normalized());
  const Eigen::Quaterniond q_gt(aa);
  const Eigen::Vector3d    C_gt(1.0, -0.5, 0.2);
  double pose[7] = {q_gt.x(), q_gt.y(), q_gt.z(), q_gt.w(),
                    C_gt.x(), C_gt.y(), C_gt.z()};
  double pt[3] = {0.8, 0.4, 6.0};

  double uv[2];
  if (!project(intr, pose, pt, uv)) { std::cerr << "  projection failed\n"; return 1; }

  ceres::CostFunction* cost = new ReprojectionCostAnalytic(uv[0], uv[1]);

  ceres::NumericDiffOptions ndiff;
  ceres::GradientChecker checker(cost, nullptr, ndiff);
  std::vector<double*> params_vec = {intr, pose, pt};

  ceres::GradientChecker::ProbeResults probe;
  if (!checker.Probe(params_vec.data(), 1e-4, &probe)) {
    std::cerr << "  FAIL: max relative error = " << probe.maximum_relative_error << "\n";
    std::cerr << "  " << probe.error_log << "\n";
    return 1;
  }
  std::cout << "  PASS: max relative error = " << probe.maximum_relative_error << "\n";
  return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 3 – Convergence (single camera, 4 views, 100 points)
// ─────────────────────────────────────────────────────────────────────────────

static int test_convergence_single_camera() {
  std::cout << "[Test 3] Single-camera convergence (4 views, 100 pts)\n";

  const double f_gt  = 800.0;
  const double sigma_gt = 1.0;
  const double cx_gt = 320.0, cy_gt = 240.0;
  const double k1_gt = 0.02, k2_gt = -0.005, k3_gt = 0.0;
  const double p1_gt = 0.001, p2_gt = -0.001;

  double intr_gt[kAnalyticIntrCount] = {f_gt, sigma_gt, cx_gt, cy_gt,
                                         k1_gt, k2_gt, k3_gt, p1_gt, p2_gt};

  const int n_cams = 4, n_pts = 100;
  const Eigen::Vector3d cam_pos[4] = {
      {0, 0, -10}, {5, 0, -10}, {-3, 2, -10}, {1, -3, -9}};

  std::vector<Eigen::Matrix3d> R_gt(n_cams);
  std::vector<Eigen::Vector3d> t_gt(n_cams);
  for (int i = 0; i < n_cams; ++i) look_at_origin(cam_pos[i], &R_gt[i], &t_gt[i]);

  std::mt19937 rng(1234);
  std::uniform_real_distribution<double> dxy(-2.0, 2.0);
  std::uniform_real_distribution<double> dz(2.0, 6.0);
  std::normal_distribution<double> noise(0.0, 0.5);

  std::vector<Eigen::Vector3d> pts_gt(n_pts);
  for (auto& p : pts_gt) p = {dxy(rng), dxy(rng), dz(rng)};

  BAInput input;
  input.poses_R = R_gt;
  // Convert t → C: C_i = -R_iᵀ · t_i
  input.poses_C.resize(t_gt.size());
  for (size_t i = 0; i < t_gt.size(); ++i)
    input.poses_C[i] = -R_gt[i].transpose() * t_gt[i];
  input.points3d = pts_gt;
  input.optimize_intrinsics = true;
  input.fix_intrinsics_flags.resize(1);
  input.fix_intrinsics_flags[0] = kFixIntrSigma;

  Intrinsics K;
  K.fx = f_gt; K.fy = sigma_gt * f_gt; K.cx = cx_gt; K.cy = cy_gt;
  K.k1 = k1_gt; K.k2 = k2_gt; K.k3 = k3_gt; K.p1 = p1_gt; K.p2 = p2_gt;
  input.cameras.push_back(K);
  input.image_camera_index.assign(n_cams, 0);

  for (int i = 0; i < n_cams; ++i) {
    double pose_d[7];
    Rt_to_pose(R_gt[i], t_gt[i], pose_d);
    for (int j = 0; j < n_pts; ++j) {
      double uv[2], pt[3] = {pts_gt[j].x(), pts_gt[j].y(), pts_gt[j].z()};
      if (!project(intr_gt, pose_d, pt, uv)) continue;
      uv[0] += noise(rng); uv[1] += noise(rng);
      BAObservation obs;
      obs.image_index = i; obs.point_index = j; obs.u = uv[0]; obs.v = uv[1];
      input.observations.push_back(obs);
    }
  }

  auto t0 = std::chrono::high_resolution_clock::now();
  BAResult result;
  const bool ok = global_bundle_analytic(input, &result, 100);
  auto t1 = std::chrono::high_resolution_clock::now();
  const double elapsed = std::chrono::duration<double>(t1 - t0).count();

  if (!ok || !result.success) {
    std::cerr << "  FAIL: solver returned false\n";
    return 1;
  }
  std::cout << "  Time: " << elapsed << " s, RMSE: " << result.rmse_px << " px\n";
  if (result.rmse_px > 0.6) {
    std::cerr << "  FAIL: RMSE too high\n";
    return 1;
  }
  std::cout << "  PASS\n";
  return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 4 – Multi-camera convergence
// ─────────────────────────────────────────────────────────────────────────────

static int test_convergence_multi_camera_timing() {
  std::cout << "[Test 4] Multi-camera convergence + timing (2 cameras, 6 views, 200 pts)\n";

  const int n_cams = 6, n_pts = 200;

  Intrinsics K0, K1;
  K0.fx = 900.0; K0.fy = 900.0; K0.cx = 400.0; K0.cy = 300.0;
  K0.k1 = 0.03;  K0.k2 = -0.008; K0.k3 = 0.0; K0.p1 = 0.0015; K0.p2 = -0.001;
  K1.fx = 600.0; K1.fy = 600.0; K1.cx = 320.0; K1.cy = 240.0;
  K1.k1 = 0.01;  K1.k2 = 0.0;    K1.k3 = 0.0; K1.p1 = 0.0;    K1.p2 = 0.0;

  double intr0[kAnalyticIntrCount] = {K0.fx, 1.0, K0.cx, K0.cy,
                                       K0.k1, K0.k2, K0.k3, K0.p1, K0.p2};
  double intr1[kAnalyticIntrCount] = {K1.fx, 1.0, K1.cx, K1.cy,
                                       K1.k1, K1.k2, K1.k3, K1.p1, K1.p2};

  std::vector<Eigen::Matrix3d> R_gt(n_cams);
  std::vector<Eigen::Vector3d> t_gt(n_cams);
  for (int i = 0; i < n_cams; ++i) {
    const double angle = i * (2.0 * M_PI / n_cams);
    const Eigen::Vector3d C(8.0*std::cos(angle), 0.0, 8.0*std::sin(angle));
    look_at_origin(C, &R_gt[i], &t_gt[i]);
  }

  std::mt19937 rng(42);
  std::uniform_real_distribution<double> dxy(-3.0, 3.0);
  std::uniform_real_distribution<double> dz(2.0, 6.0);
  std::normal_distribution<double> noise(0.0, 0.3);

  std::vector<Eigen::Vector3d> pts_gt(n_pts);
  for (auto& p : pts_gt) p = {dxy(rng), dxy(rng), dz(rng)};

  std::vector<int> cam_assign(n_cams);
  for (int i = 0; i < n_cams; ++i) cam_assign[i] = (i % 2 == 0) ? 0 : 1;

  BAInput input;
  input.poses_R = R_gt;
  // Convert t → C: C_i = -R_iᵀ · t_i
  input.poses_C.resize(t_gt.size());
  for (size_t i = 0; i < t_gt.size(); ++i)
    input.poses_C[i] = -R_gt[i].transpose() * t_gt[i];
  input.points3d = pts_gt;
  input.cameras.push_back(K0);
  input.cameras.push_back(K1);
  input.image_camera_index = cam_assign;
  input.optimize_intrinsics = false;

  for (int i = 0; i < n_cams; ++i) {
    double pose_d[7];
    Rt_to_pose(R_gt[i], t_gt[i], pose_d);
    const double* intr = (cam_assign[i] == 0) ? intr0 : intr1;
    for (int j = 0; j < n_pts; ++j) {
      double uv[2], pt[3] = {pts_gt[j].x(), pts_gt[j].y(), pts_gt[j].z()};
      if (!project(intr, pose_d, pt, uv)) continue;
      uv[0] += noise(rng); uv[1] += noise(rng);
      BAObservation obs;
      obs.image_index = i; obs.point_index = j; obs.u = uv[0]; obs.v = uv[1];
      input.observations.push_back(obs);
    }
  }

  auto ta0 = std::chrono::high_resolution_clock::now();
  BAResult res_a;
  const bool ok_a = global_bundle_analytic(input, &res_a, 100);
  auto ta1 = std::chrono::high_resolution_clock::now();
  const double time_a = std::chrono::duration<double>(ta1 - ta0).count();

  std::cout << "  Analytic BA: ok=" << ok_a << "  RMSE=" << res_a.rmse_px
            << " px  time=" << time_a << " s\n";

  if (!ok_a || !res_a.success) {
    std::cerr << "  FAIL: analytic BA failed\n";
    return 1;
  }
  if (res_a.rmse_px > 0.35) {
    std::cerr << "  FAIL: analytic RMSE too high\n";
    return 1;
  }
  std::cout << "  PASS\n";
  return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 5 – Zero-noise residual with scale (weighted cost)
// ─────────────────────────────────────────────────────────────────────────────

static int test_zero_noise_residual_with_scale() {
  std::cout << "[Test 5] Zero-noise residual with scale (weighted cost)\n";

  double intr[kAnalyticIntrCount] = {800.0, 1.0, 320.0, 240.0,
                                      0.01, -0.002, 0.0005, 0.001, 0.001};
  double pose[7] = {0, 0, 0, 1,  0, 0, 0};
  double pt[3]  = {0.5, -0.3, 5.0};

  double uv[2];
  if (!project(intr, pose, pt, uv)) { std::cerr << "  projection failed\n"; return 1; }

  const double scale = 2.0;
  double res[2];
  const double* params[3] = {intr, pose, pt};
  ReprojectionCostAnalyticWeighted cost(uv[0], uv[1], scale);
  cost.Evaluate(params, res, nullptr);

  if (std::abs(res[0]) > 1e-8 || std::abs(res[1]) > 1e-8) {
    std::cerr << "  FAIL: weighted residuals = (" << res[0] << ", " << res[1] << ")\n";
    return 1;
  }
  std::cout << "  PASS\n";
  return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 6 – Jacobian gradient check (weighted cost)
// ─────────────────────────────────────────────────────────────────────────────

static int test_jacobian_weighted() {
  std::cout << "[Test 6] Jacobian gradient check (weighted cost)\n";

  double intr[kAnalyticIntrCount] = {600.0, 1.02, 400.0, 300.0,
                                      0.05, -0.01, 0.001, 0.002, -0.001};
  const Eigen::AngleAxisd aa(0.26, Eigen::Vector3d(0.3, 1.0, 0.1).normalized());
  const Eigen::Quaterniond q_gt(aa);
  const Eigen::Vector3d    C_gt(1.0, -0.5, 0.2);
  double pose[7] = {q_gt.x(), q_gt.y(), q_gt.z(), q_gt.w(),
                    C_gt.x(), C_gt.y(), C_gt.z()};
  double pt[3] = {0.8, 0.4, 6.0};

  double uv[2];
  if (!project(intr, pose, pt, uv)) { std::cerr << "  projection failed\n"; return 1; }

  const double scale = 1.5;
  ceres::CostFunction* cost = new ReprojectionCostAnalyticWeighted(uv[0], uv[1], scale);
  ceres::NumericDiffOptions ndiff;
  ceres::GradientChecker checker(cost, nullptr, ndiff);
  std::vector<double*> params_vec = {intr, pose, pt};

  ceres::GradientChecker::ProbeResults probe;
  if (!checker.Probe(params_vec.data(), 1e-4, &probe)) {
    std::cerr << "  FAIL: max relative error = " << probe.maximum_relative_error << "\n";
    std::cerr << "  " << probe.error_log << "\n";
    return 1;
  }
  std::cout << "  PASS: max relative error = " << probe.maximum_relative_error << "\n";
  return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 7 – Two-view, one camera (2 images, same intrinsics via image_to_camera)
// ─────────────────────────────────────────────────────────────────────────────

static int test_two_view_one_camera() {
  std::cout << "[Test 7] Two-view one camera (2 images, N points)\n";

  const double f = 700.0, cx = 320.0, cy = 240.0;
  Intrinsics K;
  K.fx = f; K.fy = f; K.cx = cx; K.cy = cy;
  K.k1 = 0.02; K.k2 = -0.005; K.k3 = 0.0; K.p1 = 0.001; K.p2 = -0.001;

  Eigen::Matrix3d R0, R1;
  Eigen::Vector3d t0, t1;
  look_at_origin(Eigen::Vector3d(0, 0, -8), &R0, &t0);
  look_at_origin(Eigen::Vector3d(1.5, 0, -8), &R1, &t1);

  const int n_pts = 50;
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> dxy(-2.0, 2.0);
  std::uniform_real_distribution<double> dz(3.0, 6.0);
  std::normal_distribution<double> noise(0.0, 0.4);

  std::vector<Eigen::Vector3d> pts(n_pts);
  for (auto& p : pts) p = {dxy(rng), dxy(rng), dz(rng)};

  BAInput input;
  input.poses_R = {R0, R1};
  input.poses_C.resize(2);
  input.poses_C[0] = -R0.transpose() * t0;
  input.poses_C[1] = -R1.transpose() * t1;
  input.points3d = pts;
  input.cameras = {K};
  input.image_camera_index = {0, 0};
  input.optimize_intrinsics = false;

  double intr[9] = {K.fx, 1.0, K.cx, K.cy, K.k1, K.k2, K.k3, K.p1, K.p2};
  double pose0[7], pose1[7];
  Rt_to_pose(R0, t0, pose0);
  Rt_to_pose(R1, t1, pose1);
  for (int j = 0; j < n_pts; ++j) {
    double uv[2], pt[3] = {pts[j].x(), pts[j].y(), pts[j].z()};
    if (!project(intr, pose0, pt, uv)) continue;
    uv[0] += noise(rng); uv[1] += noise(rng);
    BAObservation o;
    o.image_index = 0; o.point_index = j; o.u = uv[0]; o.v = uv[1]; o.scale = 1.2;
    input.observations.push_back(o);
    if (!project(intr, pose1, pt, uv)) continue;
    uv[0] += noise(rng); uv[1] += noise(rng);
    o.image_index = 1; o.point_index = j; o.u = uv[0]; o.v = uv[1]; o.scale = 1.0;
    input.observations.push_back(o);
  }

  BAResult result;
  if (!global_bundle_analytic(input, &result, 80)) {
    std::cerr << "  FAIL: solver failed\n";
    return 1;
  }
  if (!result.success || result.rmse_px > 0.55) {
    std::cerr << "  FAIL: RMSE = " << result.rmse_px << "\n";
    return 1;
  }
  std::cout << "  PASS: RMSE = " << result.rmse_px << " px\n";
  return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 8 – Two-view, two cameras (different intrinsics)
// ─────────────────────────────────────────────────────────────────────────────

static int test_two_view_two_cameras() {
  std::cout << "[Test 8] Two-view two cameras (different intrinsics)\n";

  Intrinsics K0, K1;
  K0.fx = 800.0; K0.fy = 800.0; K0.cx = 400.0; K0.cy = 300.0;
  K0.k1 = 0.02;  K0.k2 = -0.005; K0.k3 = 0.0; K0.p1 = 0.0; K0.p2 = 0.0;
  K1.fx = 500.0; K1.fy = 500.0; K1.cx = 250.0; K1.cy = 200.0;
  K1.k1 = 0.01;  K1.k2 = 0.0;    K1.k3 = 0.0; K1.p1 = 0.001; K1.p2 = -0.001;

  Eigen::Matrix3d R0, R1;
  Eigen::Vector3d t0, t1;
  look_at_origin(Eigen::Vector3d(0, 0, -10), &R0, &t0);
  look_at_origin(Eigen::Vector3d(2, 0, -10), &R1, &t1);

  const int n_pts = 40;
  std::mt19937 rng(8);
  std::uniform_real_distribution<double> dxy(-2.0, 2.0);
  std::uniform_real_distribution<double> dz(4.0, 8.0);
  std::normal_distribution<double> noise(0.0, 0.35);

  std::vector<Eigen::Vector3d> pts(n_pts);
  for (auto& p : pts) p = {dxy(rng), dxy(rng), dz(rng)};

  BAInput input;
  input.poses_R = {R0, R1};
  input.poses_C.resize(2);
  input.poses_C[0] = -R0.transpose() * t0;
  input.poses_C[1] = -R1.transpose() * t1;
  input.points3d = pts;
  input.cameras = {K0, K1};
  input.image_camera_index = {0, 1};
  input.optimize_intrinsics = false;

  double intr0[9] = {K0.fx, 1.0, K0.cx, K0.cy, K0.k1, K0.k2, K0.k3, K0.p1, K0.p2};
  double intr1[9] = {K1.fx, 1.0, K1.cx, K1.cy, K1.k1, K1.k2, K1.k3, K1.p1, K1.p2};
  double pose0[7], pose1[7];
  Rt_to_pose(R0, t0, pose0);
  Rt_to_pose(R1, t1, pose1);
  for (int j = 0; j < n_pts; ++j) {
    double uv[2], pt[3] = {pts[j].x(), pts[j].y(), pts[j].z()};
    if (!project(intr0, pose0, pt, uv)) continue;
    uv[0] += noise(rng); uv[1] += noise(rng);
    BAObservation o;
    o.image_index = 0; o.point_index = j; o.u = uv[0]; o.v = uv[1]; o.scale = 1.0;
    input.observations.push_back(o);
    if (!project(intr1, pose1, pt, uv)) continue;
    uv[0] += noise(rng); uv[1] += noise(rng);
    o.image_index = 1; o.point_index = j; o.u = uv[0]; o.v = uv[1]; o.scale = 1.5;
    input.observations.push_back(o);
  }

  BAResult result;
  if (!global_bundle_analytic(input, &result, 80)) {
    std::cerr << "  FAIL: solver failed\n";
    return 1;
  }
  if (!result.success || result.rmse_px > 0.5) {
    std::cerr << "  FAIL: RMSE = " << result.rmse_px << "\n";
    return 1;
  }
  std::cout << "  PASS: RMSE = " << result.rmse_px << " px\n";
  return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 9 – Fix pose and fix point (constant blocks)
// ─────────────────────────────────────────────────────────────────────────────

static int test_fix_pose_and_point() {
  std::cout << "[Test 9] Fix pose and fix point\n";

  const double f = 750.0, cx = 320.0, cy = 240.0;
  Intrinsics K;
  K.fx = f; K.fy = f; K.cx = cx; K.cy = cy;
  K.k1 = 0.015; K.k2 = -0.003; K.k3 = 0.0; K.p1 = 0.0; K.p2 = 0.0;

  Eigen::Matrix3d R0, R1, R2;
  Eigen::Vector3d t0, t1, t2;
  look_at_origin(Eigen::Vector3d(0, 0, -9), &R0, &t0);
  look_at_origin(Eigen::Vector3d(1.2, 0, -9), &R1, &t1);
  look_at_origin(Eigen::Vector3d(-0.8, 1.0, -9), &R2, &t2);

  const int n_pts = 30;
  std::mt19937 rng(9);
  std::uniform_real_distribution<double> dxy(-1.5, 1.5);
  std::uniform_real_distribution<double> dz(2.0, 5.0);
  std::normal_distribution<double> noise(0.0, 0.3);

  std::vector<Eigen::Vector3d> pts(n_pts);
  for (auto& p : pts) p = {dxy(rng), dxy(rng), dz(rng)};

  BAInput input;
  input.poses_R = {R0, R1, R2};
  input.poses_C.resize(3);
  input.poses_C[0] = -R0.transpose() * t0;
  input.poses_C[1] = -R1.transpose() * t1;
  input.poses_C[2] = -R2.transpose() * t2;
  input.points3d = pts;
  input.cameras = {K};
  input.image_camera_index = {0, 0, 0};
  input.optimize_intrinsics = false;

  input.fix_pose.resize(3);
  input.fix_pose[0] = true;
  input.fix_pose[1] = true;
  input.fix_pose[2] = false;

  input.fix_point.resize(static_cast<size_t>(n_pts));
  for (int p = 0; p < n_pts; ++p)
    input.fix_point[static_cast<size_t>(p)] = (p < n_pts / 2);

  double intr[9] = {K.fx, 1.0, K.cx, K.cy, K.k1, K.k2, K.k3, K.p1, K.p2};
  double pose_buf[3][7];
  Rt_to_pose(R0, t0, pose_buf[0]);
  Rt_to_pose(R1, t1, pose_buf[1]);
  Rt_to_pose(R2, t2, pose_buf[2]);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < n_pts; ++j) {
      double uv[2], pt[3] = {pts[j].x(), pts[j].y(), pts[j].z()};
      if (!project(intr, pose_buf[i], pt, uv)) continue;
      uv[0] += noise(rng); uv[1] += noise(rng);
      BAObservation o;
      o.image_index = i; o.point_index = j; o.u = uv[0]; o.v = uv[1];
      input.observations.push_back(o);
    }
  }

  BAResult result;
  if (!global_bundle_analytic(input, &result, 60)) {
    std::cerr << "  FAIL: solver failed\n";
    return 1;
  }
  if (!result.success) {
    std::cerr << "  FAIL: no success\n";
    return 1;
  }

  const Eigen::Matrix3d& R1_in = input.poses_R[1];
  const Eigen::Matrix3d& R1_out = result.poses_R[1];
  const Eigen::Vector3d& C1_in = input.poses_C[1];
  const Eigen::Vector3d& C1_out = result.poses_C[1];
  double pose_err = (R1_out - R1_in).norm() + (C1_out - C1_in).norm();
  if (pose_err > 1e-10) {
    std::cerr << "  FAIL: fixed pose[1] changed, diff = " << pose_err << "\n";
    return 1;
  }
  for (int p = 0; p < n_pts / 2; ++p) {
    const auto& pin = input.points3d[static_cast<size_t>(p)];
    const auto& pout = result.points3d[static_cast<size_t>(p)];
    if ((pout - pin).norm() > 1e-10) {
      std::cerr << "  FAIL: fixed point " << p << " changed\n";
      return 1;
    }
  }
  std::cout << "  PASS: fixed pose and points unchanged, RMSE = " << result.rmse_px << " px\n";
  return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────

int main() {
  google::InitGoogleLogging("test_ba_analytic");
  FLAGS_logtostderr   = 1;
  FLAGS_minloglevel   = 2; // suppress INFO/WARNING in test output

  int failures = 0;
  failures += test_zero_noise_residual();
  failures += test_jacobian_gradient_check();
  failures += test_convergence_single_camera();
  failures += test_convergence_multi_camera_timing();
  failures += test_zero_noise_residual_with_scale();
  failures += test_jacobian_weighted();
  failures += test_two_view_one_camera();
  failures += test_two_view_two_cameras();
  failures += test_fix_pose_and_point();

  if (failures == 0) {
    std::cout << "\nAll tests PASSED.\n";
    return 0;
  }
  std::cerr << "\n" << failures << " test(s) FAILED.\n";
  return 1;
}
