/**
 * @file  bundle_adjustment_analytic.cpp
 * @brief Analytic-Jacobian BA implementation.
 *
 * Camera model: fx + sigma (fy = sigma·fx), Brown-Conrady k1/k2/k3 + p1/p2 (Bentley).
 * Rotation: quaternion [qx,qy,qz,qw] + camera center C.
 * EigenQuaternionParameterization × IdentityParameterization(3) applied to pose[7].
 *
 * Performance
 * ───────────
 *  · All Jacobian code is pure scalar arithmetic; Evaluate() creates no Eigen
 *    temporaries — the rotation matrix is computed inline from the quaternion
 *    in the q²-form (|q|² terms, NOT 1−2q² form) to stay consistent with the
 *    Sola closed-form dR(q)·p / dq.
 *  · Points are stored in result->points3d and Ceres optimises them in-place
 *    via Eigen::Vector3d::data(), eliminating the final readback copy.
 *  · The 6-scalar "jp" products (d(res)/d(R·col)) are shared between the
 *    camera-center and 3D-point Jacobians (negation relation).
 */

#include "bundle_adjustment_analytic.h"

#include "../camera/camera_utils.h"
#include <chrono>
#include <cmath>
#include <fstream>
#include <glog/logging.h>
#include <sstream>
#include <thread>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ceres/ceres.h>

namespace insight {
namespace sfm {

// ─────────────────────────────────────────────────────────────────────────────
// Sola closed-form: J[3][4] = d(R(q)·p) / d(qx,qy,qz,qw)
//
// R is the q²-form:
//   R(0,0) = w²+x²−y²−z²,  R(0,1) = 2(xy−zw),  R(0,2) = 2(xz+yw),  …
// ─────────────────────────────────────────────────────────────────────────────

namespace {

inline void sola_dRp_dq(double px, double py, double pz, double qx, double qy, double qz, double qw,
                        double J[3][4]) {
  const double qxpx = qx * px, qxpy = qx * py, qxpz = qx * pz;
  const double qypx = qy * px, qypy = qy * py, qypz = qy * pz;
  const double qzpx = qz * px, qzpy = qz * py, qzpz = qz * pz;
  const double qwpx = qw * px, qwpy = qw * py, qwpz = qw * pz;

  const double d = 2.0 * (qxpx + qypy + qzpz);  // diagonal
  const double a = 2.0 * (qypx - qxpy - qwpz);  // (1,0)
  const double b = 2.0 * (-qzpx + qxpz - qwpy); // (2,0) negated below
  const double c = 2.0 * (qzpy - qypz - qwpx);  // (2,1)

  J[0][0] = d;
  J[0][1] = -a;
  J[0][2] = b;
  J[0][3] = 2.0 * (qwpx + qypz - qzpy);
  J[1][0] = a;
  J[1][1] = d;
  J[1][2] = -c;
  J[1][3] = 2.0 * (qwpy - qxpz + qzpx);
  J[2][0] = -b;
  J[2][1] = c;
  J[2][2] = d;
  J[2][3] = 2.0 * (qwpz + qxpy - qypx);
}

} // namespace

// #region agent log
namespace {
void agent_debug_ndjson(const char* hypothesis_id, const char* location, const char* message,
                        const std::string& data_json) {
  std::ofstream f("/home/jones/Git/01jones/InsightAT/InsightAT/.cursor/debug-2ba246.log",
                  std::ios::app);
  if (!f)
    return;
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::system_clock::now().time_since_epoch())
                      .count();
  f << "{\"sessionId\":\"2ba246\",\"hypothesisId\":\"" << hypothesis_id << "\",\"location\":\""
    << location << "\",\"message\":\"" << message << "\",\"data\":" << data_json
    << ",\"timestamp\":" << ms << "}\n";
}

/// Same geometry as ReprojectionCostAnalytic + camera_utils distortion (matches pipeline).
double reproj_px_pipeline(const Eigen::Matrix3d& R, const Eigen::Vector3d& C,
                          const camera::Intrinsics& K, const Eigen::Vector3d& X, double u_obs,
                          double v_obs, bool* zc_ok) {
  Eigen::Vector3d p = R * (X - C);
  if (p(2) <= 1e-12) {
    *zc_ok = false;
    return 1e6;
  }
  *zc_ok = true;
  const double xn = p(0) / p(2), yn = p(1) / p(2);
  double xd, yd;
  camera::apply_distortion(xn, yn, K, &xd, &yd);
  const double u_pred = K.fx * xd + K.cx;
  const double v_pred = K.fy * yd + K.cy;
  const double du = u_obs - u_pred, dv = v_obs - v_pred;
  return std::sqrt(du * du + dv * dv);
}

void diagnose_ba_input_pre_solve(const BAInput& input, const std::vector<Eigen::Vector3d>& points3d,
                                 int n_cams, int n_distinct) {
  int n_obs = 0, n_behind = 0, n_huge = 0;
  double sum_e = 0.0, max_e = 0.0;
  std::vector<double> sum_per_cam(static_cast<size_t>(n_cams), 0.0);
  std::vector<int> cnt_per_cam(static_cast<size_t>(n_cams), 0);
  std::vector<double> max_per_cam(static_cast<size_t>(n_cams), 0.0);
  int fix_pose_true = 0;
  for (size_t i = 0; i < input.fix_pose.size(); ++i)
    if (input.fix_pose[i])
      ++fix_pose_true;

  for (const auto& obs : input.observations) {
    if (obs.image_index < 0 || obs.image_index >= n_cams || obs.point_index < 0 ||
        obs.point_index >= static_cast<int>(points3d.size()))
      continue;
    const int cam_idx = input.image_camera_index[static_cast<size_t>(obs.image_index)];
    if (cam_idx < 0 || cam_idx >= n_distinct)
      continue;
    const Eigen::Matrix3d& R = input.poses_R[static_cast<size_t>(obs.image_index)];
    const Eigen::Vector3d& C = input.poses_C[static_cast<size_t>(obs.image_index)];
    const Eigen::Vector3d& X = points3d[static_cast<size_t>(obs.point_index)];
    const camera::Intrinsics& K = input.cameras[static_cast<size_t>(cam_idx)];
    bool zc_ok = false;
    const double e = reproj_px_pipeline(R, C, K, X, obs.u, obs.v, &zc_ok);
    ++n_obs;
    if (!zc_ok)
      ++n_behind;
    if (e >= 1e5)
      ++n_huge;
    sum_e += e;
    if (e > max_e)
      max_e = e;
    const int bi = obs.image_index;
    sum_per_cam[static_cast<size_t>(bi)] += e;
    ++cnt_per_cam[static_cast<size_t>(bi)];
    if (e > max_per_cam[static_cast<size_t>(bi)])
      max_per_cam[static_cast<size_t>(bi)] = e;
  }

  int worst_bi = -1;
  double worst_mean = 0.0;
  for (int i = 0; i < n_cams; ++i) {
    if (cnt_per_cam[static_cast<size_t>(i)] == 0)
      continue;
    const double m = sum_per_cam[static_cast<size_t>(i)] /
                     static_cast<double>(cnt_per_cam[static_cast<size_t>(i)]);
    if (m > worst_mean) {
      worst_mean = m;
      worst_bi = i;
    }
  }

  const double mean_e = (n_obs > 0) ? (sum_e / static_cast<double>(n_obs)) : 0.0;
  const double frac_behind =
      (n_obs > 0) ? (static_cast<double>(n_behind) / static_cast<double>(n_obs)) : 0.0;

  std::ostringstream d1;
  d1 << "{\"n_obs\":" << n_obs << ",\"n_zc_nonpositive\":" << n_behind
     << ",\"frac_zc_bad\":" << frac_behind << ",\"n_residual_clamped_1e6\":" << n_huge
     << ",\"mean_reproj_px\":" << mean_e << ",\"max_reproj_px\":" << max_e
     << ",\"worst_mean_ba_image_index\":" << worst_bi << ",\"worst_mean_px\":" << worst_mean << "}";
  agent_debug_ndjson("H10", "bundle_adjustment_analytic.cpp:diagnose", "pre_solve_reproj_stats",
                     d1.str());

  std::ostringstream d2;
  d2 << "{\"n_ba_cameras\":" << n_cams << ",\"fix_pose_flags_set\":" << fix_pose_true
     << ",\"huber_delta\":" << (input.huber_loss_delta > 0.0 ? input.huber_loss_delta : 4.0) << "}";
  agent_debug_ndjson("H_fix", "bundle_adjustment_analytic.cpp:diagnose", "gauge_and_loss",
                     d2.str());

  if (worst_bi >= 0) {
    std::ostringstream d3;
    d3 << "{\"worst_ba_im\":" << worst_bi << ",\"mean_on_worst\":" << worst_mean
       << ",\"max_on_worst\":" << max_per_cam[static_cast<size_t>(worst_bi)]
       << ",\"obs_count_on_worst\":" << cnt_per_cam[static_cast<size_t>(worst_bi)] << "}";
    agent_debug_ndjson("H12", "bundle_adjustment_analytic.cpp:diagnose", "worst_image_detail",
                       d3.str());
  }
}
} // namespace
// #endregion

// ─────────────────────────────────────────────────────────────────────────────
// ReprojectionCostAnalytic::Evaluate
// ─────────────────────────────────────────────────────────────────────────────

bool ReprojectionCostAnalytic::Evaluate(double const* const* params, double* residuals,
                                        double** jacobians) const {
  const double* intr = params[0];
  const double* pose = params[1];
  const double* pt = params[2];

  // ── Intrinsics ────────────────────────────────────────────────────────────
  const double fx = intr[kFx];
  const double sigma = intr[kSigma];
  const double cx = intr[kCx];
  const double cy = intr[kCy];
  const double k1 = intr[kK1];
  const double k2 = intr[kK2];
  const double k3 = intr[kK3];
  const double p1 = intr[kP1];
  const double p2 = intr[kP2];
  const double fy = sigma * fx;

  // ── Quaternion + camera centre ────────────────────────────────────────────
  const double qx = pose[0], qy = pose[1], qz = pose[2], qw = pose[3];

  // Rotation matrix – q²-form (consistent with Sola Jacobian)
  const double xx = qx * qx, yy = qy * qy, zz = qz * qz, ww = qw * qw;
  const double xy = qx * qy, xz = qx * qz, xw = qx * qw;
  const double yz = qy * qz, yw = qy * qw, zw = qz * qw;

  const double R00 = ww + xx - yy - zz, R01 = 2.0 * (xy - zw), R02 = 2.0 * (xz + yw);
  const double R10 = 2.0 * (xy + zw), R11 = ww - xx + yy - zz, R12 = 2.0 * (yz - xw);
  const double R20 = 2.0 * (xz - yw), R21 = 2.0 * (yz + xw), R22 = ww - xx - yy + zz;

  // Xw = X − C
  const double Xwx = pt[0] - pose[4];
  const double Xwy = pt[1] - pose[5];
  const double Xwz = pt[2] - pose[6];

  // Xc = R · Xw
  const double xc = R00 * Xwx + R01 * Xwy + R02 * Xwz;
  const double yc = R10 * Xwx + R11 * Xwy + R12 * Xwz;
  const double zc = R20 * Xwx + R21 * Xwy + R22 * Xwz;

  if (zc < 1e-12) {
    residuals[0] = residuals[1] = 1e6;
    if (jacobians) {
      if (jacobians[0])
        std::fill_n(jacobians[0], 2 * kAnalyticIntrCount, 0.0);
      if (jacobians[1])
        std::fill_n(jacobians[1], 14, 0.0);
      if (jacobians[2])
        std::fill_n(jacobians[2], 6, 0.0);
    }
    return true;
  }

  // ── Normalised image coordinates ──────────────────────────────────────────
  const double inv_z = 1.0 / zc;
  const double inv_z2 = inv_z * inv_z;
  const double xu = xc * inv_z;
  const double yu = yc * inv_z;
  const double xu2 = xu * xu;
  const double yu2 = yu * yu;
  const double r2 = xu2 + yu2;
  const double r4 = r2 * r2;
  const double r6 = r4 * r2;

  const double rad = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;

  // Tangential distortion (Bentley convention – matches camera_utils.cpp)
  const double tx = 2.0 * p2 * xu * yu + p1 * (r2 + 2.0 * xu2);
  const double ty = 2.0 * p1 * xu * yu + p2 * (r2 + 2.0 * yu2);

  const double dx = xu * rad + tx;
  const double dy = yu * rad + ty;

  // ── Residuals ─────────────────────────────────────────────────────────────
  residuals[0] = fx * dx + cx - u_obs_;
  residuals[1] = fy * dy + cy - v_obs_;

  if (!jacobians)
    return true;

  // ── Intermediate derivatives ──────────────────────────────────────────────
  const double drad_c = 2.0 * (k1 + 2.0 * k2 * r2 + 3.0 * k3 * r4);
  const double drad_dxu = drad_c * xu;
  const double drad_dyu = drad_c * yu;

  // d(tx)/d(xu, yu)
  const double dtx_dxu = 2.0 * p2 * yu + 6.0 * p1 * xu;
  const double dtx_dyu = 2.0 * p2 * xu + 2.0 * p1 * yu;
  // d(ty)/d(xu, yu)
  const double dty_dxu = 2.0 * p1 * yu + 2.0 * p2 * xu; // = dtx_dyu with p1↔p2
  const double dty_dyu = 2.0 * p1 * xu + 6.0 * p2 * yu;

  const double ddx_dxu = rad + xu * drad_dxu + dtx_dxu;
  const double ddx_dyu = xu * drad_dyu + dtx_dyu;
  const double ddy_dxu = yu * drad_dxu + dty_dxu;
  const double ddy_dyu = rad + yu * drad_dyu + dty_dyu;

  // d(res)/d(xc, yc, zc)
  const double dr0_dxc = fx * ddx_dxu * inv_z;
  const double dr0_dyc = fx * ddx_dyu * inv_z;
  const double dr0_dzc = -fx * (ddx_dxu * xu + ddx_dyu * yu) * inv_z;

  const double dr1_dxc = fy * ddy_dxu * inv_z;
  const double dr1_dyc = fy * ddy_dyu * inv_z;
  const double dr1_dzc = -fy * (ddy_dxu * xu + ddy_dyu * yu) * inv_z;

  // Shared R·column products  (point Jacobian = +jp, centre Jacobian = −jp)
  const double jp00 = dr0_dxc * R00 + dr0_dyc * R10 + dr0_dzc * R20;
  const double jp01 = dr0_dxc * R01 + dr0_dyc * R11 + dr0_dzc * R21;
  const double jp02 = dr0_dxc * R02 + dr0_dyc * R12 + dr0_dzc * R22;
  const double jp10 = dr1_dxc * R00 + dr1_dyc * R10 + dr1_dzc * R20;
  const double jp11 = dr1_dxc * R01 + dr1_dyc * R11 + dr1_dzc * R21;
  const double jp12 = dr1_dxc * R02 + dr1_dyc * R12 + dr1_dzc * R22;

  // ── J wrt intrinsics [fx, sigma, cx, cy, k1, k2, k3, p1, p2] ────────────
  if (jacobians[0]) {
    double* J = jacobians[0]; // row-major 2×9
    // row 0
    J[kFx] = dx;
    J[kSigma] = 0.0;
    J[kCx] = 1.0;
    J[kCy] = 0.0;
    J[kK1] = fx * xu * r2;
    J[kK2] = fx * xu * r4;
    J[kK3] = fx * xu * r6;
    J[kP1] = fx * (r2 + 2.0 * xu2); // d(tx)/d(p1)
    J[kP2] = fx * 2.0 * xu * yu;    // d(tx)/d(p2)
    // row 1
    J += kAnalyticIntrCount;
    J[kFx] = sigma * dy;
    J[kSigma] = fx * dy;
    J[kCx] = 0.0;
    J[kCy] = 1.0;
    J[kK1] = fy * yu * r2;
    J[kK2] = fy * yu * r4;
    J[kK3] = fy * yu * r6;
    J[kP1] = fy * 2.0 * xu * yu;    // d(ty)/d(p1)
    J[kP2] = fy * (r2 + 2.0 * yu2); // d(ty)/d(p2)
  }

  // ── J wrt pose [qx, qy, qz, qw, Cx, Cy, Cz] ────────────────────────────
  if (jacobians[1]) {
    double* J = jacobians[1]; // row-major 2×7

    // Quaternion columns 0-3 via Sola d(R·Xw)/dq
    double dXc_dq[3][4];
    sola_dRp_dq(Xwx, Xwy, Xwz, qx, qy, qz, qw, dXc_dq);

    for (int i = 0; i < 4; ++i) {
      J[i] = dr0_dxc * dXc_dq[0][i] + dr0_dyc * dXc_dq[1][i] + dr0_dzc * dXc_dq[2][i];
      J[7 + i] = dr1_dxc * dXc_dq[0][i] + dr1_dyc * dXc_dq[1][i] + dr1_dzc * dXc_dq[2][i];
    }

    // Camera-centre columns 4-6: −jp
    J[4] = -jp00;
    J[5] = -jp01;
    J[6] = -jp02;
    J[11] = -jp10;
    J[12] = -jp11;
    J[13] = -jp12;
  }

  // ── J wrt 3D point [X, Y, Z] ─────────────────────────────────────────────
  if (jacobians[2]) {
    double* J = jacobians[2]; // row-major 2×3
    J[0] = jp00;
    J[1] = jp01;
    J[2] = jp02;
    J[3] = jp10;
    J[4] = jp11;
    J[5] = jp12;
  }

  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// ReprojectionCostAnalyticWeighted::Evaluate (same as above, then scale by 1/scale)
// ─────────────────────────────────────────────────────────────────────────────

bool ReprojectionCostAnalyticWeighted::Evaluate(double const* const* params, double* residuals,
                                                double** jacobians) const {
  const double* intr = params[0];
  const double* pose = params[1];
  const double* pt = params[2];

  const double fx = intr[kFx];
  const double sigma = intr[kSigma];
  const double cx = intr[kCx];
  const double cy = intr[kCy];
  const double k1 = intr[kK1];
  const double k2 = intr[kK2];
  const double k3 = intr[kK3];
  const double p1 = intr[kP1];
  const double p2 = intr[kP2];
  const double fy = sigma * fx;

  const double qx = pose[0], qy = pose[1], qz = pose[2], qw = pose[3];
  const double xx = qx * qx, yy = qy * qy, zz = qz * qz, ww = qw * qw;
  const double xy = qx * qy, xz = qx * qz, xw = qx * qw;
  const double yz = qy * qz, yw = qy * qw, zw = qz * qw;

  const double R00 = ww + xx - yy - zz, R01 = 2.0 * (xy - zw), R02 = 2.0 * (xz + yw);
  const double R10 = 2.0 * (xy + zw), R11 = ww - xx + yy - zz, R12 = 2.0 * (yz - xw);
  const double R20 = 2.0 * (xz - yw), R21 = 2.0 * (yz + xw), R22 = ww - xx - yy + zz;

  const double Xwx = pt[0] - pose[4];
  const double Xwy = pt[1] - pose[5];
  const double Xwz = pt[2] - pose[6];

  const double xc = R00 * Xwx + R01 * Xwy + R02 * Xwz;
  const double yc = R10 * Xwx + R11 * Xwy + R12 * Xwz;
  const double zc = R20 * Xwx + R21 * Xwy + R22 * Xwz;

  if (zc < 1e-12) {
    residuals[0] = residuals[1] = 1e6 * weight_;
    if (jacobians) {
      if (jacobians[0])
        std::fill_n(jacobians[0], 2 * kAnalyticIntrCount, 0.0);
      if (jacobians[1])
        std::fill_n(jacobians[1], 14, 0.0);
      if (jacobians[2])
        std::fill_n(jacobians[2], 6, 0.0);
    }
    return true;
  }

  const double inv_z = 1.0 / zc;
  const double inv_z2 = inv_z * inv_z;
  const double xu = xc * inv_z;
  const double yu = yc * inv_z;
  const double xu2 = xu * xu;
  const double yu2 = yu * yu;
  const double r2 = xu2 + yu2;
  const double r4 = r2 * r2;
  const double r6 = r4 * r2;
  const double rad = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
  const double tx = 2.0 * p2 * xu * yu + p1 * (r2 + 2.0 * xu2);
  const double ty = 2.0 * p1 * xu * yu + p2 * (r2 + 2.0 * yu2);
  const double dx = xu * rad + tx;
  const double dy = yu * rad + ty;

  residuals[0] = (fx * dx + cx - u_obs_) * weight_;
  residuals[1] = (fy * dy + cy - v_obs_) * weight_;

  if (!jacobians)
    return true;

  const double drad_c = 2.0 * (k1 + 2.0 * k2 * r2 + 3.0 * k3 * r4);
  const double drad_dxu = drad_c * xu;
  const double drad_dyu = drad_c * yu;
  const double dtx_dxu = 2.0 * p2 * yu + 6.0 * p1 * xu;
  const double dtx_dyu = 2.0 * p2 * xu + 2.0 * p1 * yu;
  const double dty_dxu = 2.0 * p1 * yu + 2.0 * p2 * xu;
  const double dty_dyu = 2.0 * p1 * xu + 6.0 * p2 * yu;
  const double ddx_dxu = rad + xu * drad_dxu + dtx_dxu;
  const double ddx_dyu = xu * drad_dyu + dtx_dyu;
  const double ddy_dxu = yu * drad_dxu + dty_dxu;
  const double ddy_dyu = rad + yu * drad_dyu + dty_dyu;
  const double dr0_dxc = fx * ddx_dxu * inv_z;
  const double dr0_dyc = fx * ddx_dyu * inv_z;
  const double dr0_dzc = -fx * (ddx_dxu * xu + ddx_dyu * yu) * inv_z;
  const double dr1_dxc = fy * ddy_dxu * inv_z;
  const double dr1_dyc = fy * ddy_dyu * inv_z;
  const double dr1_dzc = -fy * (ddy_dxu * xu + ddy_dyu * yu) * inv_z;
  const double jp00 = dr0_dxc * R00 + dr0_dyc * R10 + dr0_dzc * R20;
  const double jp01 = dr0_dxc * R01 + dr0_dyc * R11 + dr0_dzc * R21;
  const double jp02 = dr0_dxc * R02 + dr0_dyc * R12 + dr0_dzc * R22;
  const double jp10 = dr1_dxc * R00 + dr1_dyc * R10 + dr1_dzc * R20;
  const double jp11 = dr1_dxc * R01 + dr1_dyc * R11 + dr1_dzc * R21;
  const double jp12 = dr1_dxc * R02 + dr1_dyc * R12 + dr1_dzc * R22;

  if (jacobians[0]) {
    double* J = jacobians[0];
    J[kFx] = dx * weight_;
    J[kSigma] = 0.0;
    J[kCx] = weight_;
    J[kCy] = 0.0;
    J[kK1] = fx * xu * r2 * weight_;
    J[kK2] = fx * xu * r4 * weight_;
    J[kK3] = fx * xu * r6 * weight_;
    J[kP1] = fx * (r2 + 2.0 * xu2) * weight_;
    J[kP2] = fx * 2.0 * xu * yu * weight_;
    J += kAnalyticIntrCount;
    J[kFx] = sigma * dy * weight_;
    J[kSigma] = fx * dy * weight_;
    J[kCx] = 0.0;
    J[kCy] = weight_;
    J[kK1] = fy * yu * r2 * weight_;
    J[kK2] = fy * yu * r4 * weight_;
    J[kK3] = fy * yu * r6 * weight_;
    J[kP1] = fy * 2.0 * xu * yu * weight_;
    J[kP2] = fy * (r2 + 2.0 * yu2) * weight_;
  }
  if (jacobians[1]) {
    double* J = jacobians[1];
    double dXc_dq[3][4];
    sola_dRp_dq(Xwx, Xwy, Xwz, qx, qy, qz, qw, dXc_dq);
    for (int i = 0; i < 4; ++i) {
      J[i] = (dr0_dxc * dXc_dq[0][i] + dr0_dyc * dXc_dq[1][i] + dr0_dzc * dXc_dq[2][i]) * weight_;
      J[7 + i] =
          (dr1_dxc * dXc_dq[0][i] + dr1_dyc * dXc_dq[1][i] + dr1_dzc * dXc_dq[2][i]) * weight_;
    }
    J[4] = -jp00 * weight_;
    J[5] = -jp01 * weight_;
    J[6] = -jp02 * weight_;
    J[11] = -jp10 * weight_;
    J[12] = -jp11 * weight_;
    J[13] = -jp12 * weight_;
  }
  if (jacobians[2]) {
    double* J = jacobians[2];
    J[0] = jp00 * weight_;
    J[1] = jp01 * weight_;
    J[2] = jp02 * weight_;
    J[3] = jp10 * weight_;
    J[4] = jp11 * weight_;
    J[5] = jp12 * weight_;
  }
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Focal-length soft prior (Tikhonov) — single residual, intr[9] only.
//
//   residual = sqrt(w) · (fx − fx₀) / fx₀   (relative focal deviation)
//
// Jacobian: nonzero only at kFx slot = sqrt(w)/fx₀; all other slots zero.
// This adds exactly w/fx₀² to the (fx,fx) diagonal of the Hessian —
// no off-diagonal entries, Schur/Cholesky sparsity is completely unchanged.
// ─────────────────────────────────────────────────────────────────────────────
class FocalPriorCostAnalytic : public ceres::SizedCostFunction<1, kAnalyticIntrCount> {
public:
  FocalPriorCostAnalytic(double fx0, double weight) : fx0_(fx0), sqrt_w_(std::sqrt(weight)) {}

  bool Evaluate(double const* const* params, double* residuals, double** jacobians) const override {
    residuals[0] = sqrt_w_ * (params[0][kFx] - fx0_) / fx0_;
    if (jacobians && jacobians[0]) {
      std::fill_n(jacobians[0], kAnalyticIntrCount, 0.0);
      jacobians[0][kFx] = sqrt_w_ / fx0_;
    }
    return true;
  }

private:
  double fx0_;
  double sqrt_w_;
};

// ─────────────────────────────────────────────────────────────────────────────
// Camera-centre distance prior — two pose[7] blocks; only C (indices 4–6) contribute.
//
//   residual = sqrt(w) · (‖C_a − C_b‖ − d₀) / d₀
//
// Adds a small positive diagonal on the Schur complement along the baseline direction when
// paired with a fixed anchor, helping metric scale / drift without a full pose-graph.
// ─────────────────────────────────────────────────────────────────────────────
class CameraDistanceCostAnalytic : public ceres::SizedCostFunction<1, 7, 7> {
public:
  CameraDistanceCostAnalytic(double d0, double weight) : d0_(d0), sqrt_w_(std::sqrt(weight)) {}

  bool Evaluate(double const* const* params, double* residuals, double** jacobians) const override {
    const double ax = params[0][4], ay = params[0][5], az = params[0][6];
    const double bx = params[1][4], by = params[1][5], bz = params[1][6];
    const double dx = ax - bx, dy = ay - by, dz = az - bz;
    const double dist_sq = dx * dx + dy * dy + dz * dz;
    const double dist = std::sqrt(dist_sq);
    constexpr double kEps = 1e-12;
    if (dist < kEps) {
      residuals[0] = 0.0;
      if (jacobians) {
        if (jacobians[0])
          std::fill_n(jacobians[0], 7, 0.0);
        if (jacobians[1])
          std::fill_n(jacobians[1], 7, 0.0);
      }
      return true;
    }
    const double inv_d = 1.0 / dist;
    const double nx = dx * inv_d, ny = dy * inv_d, nz = dz * inv_d;
    const double scale = sqrt_w_ / d0_;
    residuals[0] = scale * (dist - d0_);
    if (jacobians) {
      if (jacobians[0]) {
        std::fill_n(jacobians[0], 7, 0.0);
        jacobians[0][4] = scale * nx;
        jacobians[0][5] = scale * ny;
        jacobians[0][6] = scale * nz;
      }
      if (jacobians[1]) {
        std::fill_n(jacobians[1], 7, 0.0);
        jacobians[1][4] = -scale * nx;
        jacobians[1][5] = -scale * ny;
        jacobians[1][6] = -scale * nz;
      }
    }
    return true;
  }

private:
  double d0_;
  double sqrt_w_;
};

// ─────────────────────────────────────────────────────────────────────────────
// TikhonovPoseCost — diagonal L2 regularization for a 7-DOF pose block.
// (class declared in bundle_adjustment_analytic.h)
// ─────────────────────────────────────────────────────────────────────────────
TikhonovPoseCost::TikhonovPoseCost(const double* pose_snapshot, double lambda)
    : sqrt_lambda_(std::sqrt(lambda)) {
  std::copy(pose_snapshot, pose_snapshot + 7, pose0_);
}

bool TikhonovPoseCost::Evaluate(double const* const* params, double* residuals,
                                double** jacobians) const {
  const double* pose = params[0];
  for (int i = 0; i < 7; ++i)
    residuals[i] = sqrt_lambda_ * (pose[i] - pose0_[i]);
  if (jacobians && jacobians[0]) {
    std::fill_n(jacobians[0], 7 * 7, 0.0);
    for (int i = 0; i < 7; ++i)
      jacobians[0][i * 7 + i] = sqrt_lambda_;
  }
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// TikhonovIntrCost — diagonal L2 regularization for a kAnalyticIntrCount-DOF intrinsics block.
// (class declared in bundle_adjustment_analytic.h)
// ─────────────────────────────────────────────────────────────────────────────
TikhonovIntrCost::TikhonovIntrCost(const double* intr_snapshot, double lambda)
    : sqrt_lambda_(std::sqrt(lambda)) {
  std::copy(intr_snapshot, intr_snapshot + kAnalyticIntrCount, intr0_);
}

bool TikhonovIntrCost::Evaluate(double const* const* params, double* residuals,
                                double** jacobians) const {
  const double* intr = params[0];
  for (int i = 0; i < kAnalyticIntrCount; ++i)
    residuals[i] = sqrt_lambda_ * (intr[i] - intr0_[i]);
  if (jacobians && jacobians[0]) {
    std::fill_n(jacobians[0], kAnalyticIntrCount * kAnalyticIntrCount, 0.0);
    for (int i = 0; i < kAnalyticIntrCount; ++i)
      jacobians[0][i * kAnalyticIntrCount + i] = sqrt_lambda_;
  }
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// global_bundle_analytic
// ─────────────────────────────────────────────────────────────────────────────

bool global_bundle_analytic(const BAInput& input, BAResult* result, int max_iterations) {
  if (!result)
    return false;
  if (input.poses_R.size() != input.poses_C.size() || input.poses_R.empty() ||
      input.points3d.empty() || input.observations.empty())
    return false;

  const int n_cams = static_cast<int>(input.poses_R.size());
  const int n_pts = static_cast<int>(input.points3d.size());
  const bool use_multicam = !input.image_camera_index.empty() && !input.cameras.empty() &&
                            static_cast<int>(input.image_camera_index.size()) == n_cams;

  if (!use_multicam) {
    LOG(WARNING) << "global_bundle_analytic: image_camera_index and cameras required";
    return false;
  }

  const int n_distinct = static_cast<int>(input.cameras.size());

  // ── Intrinsics: Intrinsics → analytic block [fx, sigma, cx, cy, k…, p…] ──
  std::vector<double> intr_params(static_cast<size_t>(n_distinct) * kAnalyticIntrCount);
  for (int c = 0; c < n_distinct; ++c) {
    const auto& K = input.cameras[static_cast<size_t>(c)];
    double* ip = intr_params.data() + static_cast<size_t>(c) * kAnalyticIntrCount;
    ip[kFx] = K.fx;
    ip[kSigma] = (K.fx != 0.0) ? K.fy / K.fx : 1.0;
    ip[kCx] = K.cx;
    ip[kCy] = K.cy;
    ip[kK1] = K.k1;
    ip[kK2] = K.k2;
    ip[kK3] = K.k3;
    ip[kP1] = K.p1;
    ip[kP2] = K.p2;
  }

  // ── Poses: R → q,  C taken directly ──────────────────────────────────────
  std::vector<double> poses_data(static_cast<size_t>(n_cams) * 7);
  for (int i = 0; i < n_cams; ++i) {
    const Eigen::Matrix3d& Ri = input.poses_R[static_cast<size_t>(i)];
    const Eigen::Vector3d& C = input.poses_C[static_cast<size_t>(i)];
    const Eigen::Quaterniond q(Ri);
    double* pd = poses_data.data() + static_cast<size_t>(i) * 7;
    pd[0] = q.x();
    pd[1] = q.y();
    pd[2] = q.z();
    pd[3] = q.w();
    pd[4] = C.x();
    pd[5] = C.y();
    pd[6] = C.z();
  }

  // ── Points: write into result for in-place optimisation ───────────────────
  result->points3d = input.points3d;

  // ── Build Ceres problem ───────────────────────────────────────────────────
  ceres::Problem problem;
  ceres::LossFunction* loss =
      new ceres::HuberLoss(input.huber_loss_delta > 0.0 ? input.huber_loss_delta : 4.0);

  for (const auto& obs : input.observations) {
    if (obs.image_index < 0 || obs.image_index >= n_cams || obs.point_index < 0 ||
        obs.point_index >= n_pts)
      continue;
    const int cam_idx = input.image_camera_index[static_cast<size_t>(obs.image_index)];
    if (cam_idx < 0 || cam_idx >= n_distinct)
      continue;

    double* ip = intr_params.data() + static_cast<size_t>(cam_idx) * kAnalyticIntrCount;
    double* pp = poses_data.data() + static_cast<size_t>(obs.image_index) * 7;
    double* xp = result->points3d[static_cast<size_t>(obs.point_index)].data();

    const double scale = (obs.scale > 1e-12) ? obs.scale : 1.0;
    ceres::CostFunction* cost =
        (scale == 1.0)
            ? static_cast<ceres::CostFunction*>(new ReprojectionCostAnalytic(obs.u, obs.v))
            : static_cast<ceres::CostFunction*>(
                  new ReprojectionCostAnalyticWeighted(obs.u, obs.v, scale));
    problem.AddResidualBlock(cost, loss, ip, pp, xp);
  }

  // ── Parameterisation for quaternion+centre (only blocks in the problem) ───
  for (int i = 0; i < n_cams; ++i) {
    double* pp = poses_data.data() + static_cast<size_t>(i) * 7;
    if (!problem.HasParameterBlock(pp))
      continue;
    problem.SetParameterization(
        pp, new ceres::ProductParameterization(new ceres::EigenQuaternionParameterization(),
                                               new ceres::IdentityParameterization(3)));
  }

  // Fix poses: input.fix_pose[i] (gauge anchor, COLMAP constants, local-BA frozen cams).
  // If nothing is marked fixed (e.g. COLMAP local with empty constant_set), fix ba index 0
  // for gauge — but only in that case; do not also fix 0 when anchor_ba_idx≠0 is already set.
  bool any_pose_fixed = false;
  for (int i = 0; i < n_cams; ++i) {
    if (input.fix_pose.size() > static_cast<size_t>(i) && input.fix_pose[static_cast<size_t>(i)])
      any_pose_fixed = true;
  }
  for (int i = 0; i < n_cams; ++i) {
    double* pp = poses_data.data() + static_cast<size_t>(i) * 7;
    if (!problem.HasParameterBlock(pp))
      continue;
    const bool fix_from_input =
        (input.fix_pose.size() > static_cast<size_t>(i) && input.fix_pose[static_cast<size_t>(i)]);
    const bool fix = fix_from_input || (!any_pose_fixed && i == 0);
    if (fix)
      problem.SetParameterBlockConstant(pp);
  }

  // Intrinsics: constant vs optimise (fix_intrinsics_flags per camera, bitmask per parameter)
  for (int c = 0; c < n_distinct; ++c) {
    double* ip = intr_params.data() + static_cast<size_t>(c) * kAnalyticIntrCount;
    if (!problem.HasParameterBlock(ip))
      continue;
    if (!input.optimize_intrinsics) {
      problem.SetParameterBlockConstant(ip);
      continue;
    }
    uint32_t flags = (input.fix_intrinsics_flags.size() > static_cast<size_t>(c))
                         ? static_cast<uint32_t>(input.fix_intrinsics_flags[static_cast<size_t>(c)])
                         : 0u;
    std::vector<int> fixed;
    for (int i = 0; i < kAnalyticIntrCount; ++i)
      if ((flags >> i) & 1u)
        fixed.push_back(i);
    if (fixed.size() >= static_cast<size_t>(kAnalyticIntrCount))
      problem.SetParameterBlockConstant(ip);
    else if (!fixed.empty())
      problem.SetParameterization(ip, new ceres::SubsetParameterization(kAnalyticIntrCount, fixed));
  }

  // ── Intrinsics parameter bounds (prevent radial-polynomial degeneracy) ────────────────
  //
  // In aerial nadir imagery, feature points are concentrated at r/r_max ≈ 0.4–0.7.
  // In this range, r², r⁴, r⁶ are nearly collinear → the optimizer can trade k1/k2/k3
  // against each other and against focal length, creating a "bowl" or "banana" artifact
  // even when reprojection RMSE is low.
  //
  // Tight bounds on distortion prevent k2/k3 from blowing up (observed k2=0.594, k3=-0.919)
  // and constrain sigma (fy/fx) to physically plausible values for a metric camera lens.
  //
  // Bounds are NOT applied to fully-constant blocks (frozen cameras) — those blocks are
  // already handled by SetParameterBlockConstant and never touch the optimizer.
  if (input.optimize_intrinsics) {
    for (int c = 0; c < n_distinct; ++c) {
      double* ip = intr_params.data() + static_cast<size_t>(c) * kAnalyticIntrCount;
      if (!problem.HasParameterBlock(ip))
        continue;
      // Skip fully-frozen cameras (all bits set → block is SetParameterBlockConstant).
      const uint32_t flags =
          (input.fix_intrinsics_flags.size() > static_cast<size_t>(c))
              ? static_cast<uint32_t>(input.fix_intrinsics_flags[static_cast<size_t>(c)])
              : 0u;
      int n_fixed = 0;
      for (int i = 0; i < kAnalyticIntrCount; ++i)
        n_fixed += static_cast<int>((flags >> i) & 1u);
      if (n_fixed >= kAnalyticIntrCount)
        continue;

      // sigma = fy/fx: modern camera lenses are very close to square pixels.
      problem.SetParameterLowerBound(ip, kSigma, 0.95);
      problem.SetParameterUpperBound(ip, kSigma, 1.05);
      // Radial distortion: tighter bounds for aerial/drone lenses where distortion is small.
      // k1/k2/k3 are nearly collinear in the r range of aerial nadir imagery (r/r_max ≈ 0.4-0.7),
      // so loose bounds allow the optimizer to trade them against each other and focal length.
      // Tighter bounds prevent over-fitting and keep the Schur complement well-conditioned.
      problem.SetParameterLowerBound(ip, kK1, -0.3);
      problem.SetParameterUpperBound(ip, kK1, 0.3);
      problem.SetParameterLowerBound(ip, kK2, -0.15);
      problem.SetParameterUpperBound(ip, kK2, 0.15);
      problem.SetParameterLowerBound(ip, kK3, -0.05);
      problem.SetParameterUpperBound(ip, kK3, 0.05);
      // Tangential distortion: typically very small for modern drone lenses.
      problem.SetParameterLowerBound(ip, kP1, -0.05);
      problem.SetParameterUpperBound(ip, kP1, 0.05);
      problem.SetParameterLowerBound(ip, kP2, -0.05);
      problem.SetParameterUpperBound(ip, kP2, 0.05);
    }
  }

  // ── Focal soft prior ─────────────────────────────────────────────────────
  // Residual = sqrt(w)·(fx − fx₀)/fx₀, where fx₀ is the pre-BA input focal.
  // Only active for cameras where fx is not already fixed.
  // Does NOT add off-diagonal Hessian entries — sparsity is unchanged.
  if (input.focal_prior_weight > 0.0) {
    for (int c = 0; c < n_distinct; ++c) {
      double* ip = intr_params.data() + static_cast<size_t>(c) * kAnalyticIntrCount;
      if (!problem.HasParameterBlock(ip))
        continue;
      const uint32_t flags = (input.fix_intrinsics_flags.size() > static_cast<size_t>(c))
                                 ? input.fix_intrinsics_flags[static_cast<size_t>(c)]
                                 : 0u;
      // Skip if fx is fixed (either via kFixIntrFx or kFixIntrAll).
      if (flags & static_cast<uint32_t>(FixIntrinsicsMask::kFixIntrFx))
        continue;
      const double fx0 = ip[kFx];
      if (fx0 <= 0.0)
        continue;
      problem.AddResidualBlock(new FocalPriorCostAnalytic(fx0, input.focal_prior_weight), nullptr,
                               ip); // nullptr: no robust loss on the prior term
    }
  }

  // ── Optional camera-centre distance priors (gauge / scale assist with fixed anchor) ──
  for (const auto& dp : input.camera_distance_priors) {
    if (dp.weight <= 0.0 || dp.distance_m <= 1e-12)
      continue;
    if (dp.image_index_a < 0 || dp.image_index_a >= n_cams || dp.image_index_b < 0 ||
        dp.image_index_b >= n_cams || dp.image_index_a == dp.image_index_b)
      continue;
    double* pa = poses_data.data() + static_cast<size_t>(dp.image_index_a) * 7;
    double* pb = poses_data.data() + static_cast<size_t>(dp.image_index_b) * 7;
    if (!problem.HasParameterBlock(pa) || !problem.HasParameterBlock(pb))
      continue;
    problem.AddResidualBlock(new CameraDistanceCostAnalytic(dp.distance_m, dp.weight), nullptr, pa,
                             pb);
  }

  // Fix 3D points: optional fix_point[p]
  for (int p = 0; p < n_pts; ++p) {
    double* xp = result->points3d[static_cast<size_t>(p)].data();
    if (!problem.HasParameterBlock(xp))
      continue;
    if (input.fix_point.size() > static_cast<size_t>(p) && input.fix_point[static_cast<size_t>(p)])
      problem.SetParameterBlockConstant(xp);
  }

  // ── Tikhonov diagonal regularization (optional) ──────────────────────────
  if (input.tikhonov_lambda > 0.0) {
    if (input.tikhonov_lambda > 0.1)
      LOG(WARNING) << "tikhonov_lambda=" << input.tikhonov_lambda
                   << " is very high, may cause pose drift";

    // Pose blocks: only non-fixed blocks
    for (int i = 0; i < n_cams; ++i) {
      double* pp = poses_data.data() + i * 7;
      if (!problem.HasParameterBlock(pp))
        continue;
      if (problem.IsParameterBlockConstant(pp))
        continue;
      problem.AddResidualBlock(new TikhonovPoseCost(pp, input.tikhonov_lambda), nullptr, pp);
    }

    // Intrinsics blocks: only non-fixed blocks
    for (int c = 0; c < n_distinct; ++c) {
      double* ip = intr_params.data() + c * kAnalyticIntrCount;
      if (!problem.HasParameterBlock(ip))
        continue;
      if (problem.IsParameterBlockConstant(ip))
        continue;
      problem.AddResidualBlock(new TikhonovIntrCost(ip, input.tikhonov_lambda), nullptr, ip);
    }

    // Do NOT add regularization to 3D point blocks
  }

  // ── Solve ─────────────────────────────────────────────────────────────────
  ceres::Solver::Options options;
  options.max_num_iterations =
      (input.solver_max_num_iterations > 0) ? input.solver_max_num_iterations : max_iterations;
  options.minimizer_progress_to_stdout = false;

  // Auto-select linear solver based on variable camera count.
  //
  // DENSE_SCHUR (n_cams ≤ threshold):
  //   Schur complement is dense, solved by LAPACK. LAPACK's pivot threshold is more
  //   permissive than CHOLMOD's strict PD check, so it survives near-degenerate
  //   local configurations. Very fast for small Schur matrices (≤ 180×180 at n=30).
  //
  // ITERATIVE_SCHUR (n_cams > threshold):
  //   Solves the Schur complement system with PCG – never calls Cholesky, so it does
  //   NOT require the matrix to be strictly positive-definite. This is critical for
  //   large-scene aerial BA, regardless of camera configuration:
  //
  //   · Oblique 5-camera systems: each "station" contributes 5 cameras with highly
  //     correlated residuals (they share the same rigid platform). The resulting Schur
  //     complement has block structure that CHOLMOD can mis-order, yielding extreme
  //     condition numbers even though the scene itself is well-conditioned (oblique
  //     cameras observe facades and have good depth variation). PCG + CLUSTER_JACOBI
  //     exploits this cluster structure directly as the preconditioner, so convergence
  //     is much faster than JACOBI and no Cholesky factorisation is needed.
  //
  //   · Nadir-only imagery: quasi-planar ground scenes; depth direction poorly
  //     constrained; Schur condition number easily exceeds 10^12 → CHOLMOD fails.
  //
  //   · Any large graph (>300 cams): near-degenerate sub-graphs appear naturally
  //     (thin corridors, weakly connected image groups) that break CHOLMOD's strict PD.
  //
  // (SPARSE_SCHUR / CHOLMOD removed from the large-scene path.)
  const int kDenseSchurMaxVariableCams = (input.solver_dense_schur_max_variable_cams > 0)
                                             ? input.solver_dense_schur_max_variable_cams
                                             : 300;
  int n_variable_cams = 0;
  for (int i = 0; i < n_cams; ++i) {
    const bool pose_fixed =
        (input.fix_pose.size() > static_cast<size_t>(i) && input.fix_pose[static_cast<size_t>(i)]);
    if (!pose_fixed)
      ++n_variable_cams;
  }
  options.dense_linear_algebra_library_type = ceres::EIGEN;
  if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::SUITE_SPARSE)) {
    options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
    LOG(INFO) << "Using Ceres with SuiteSparse for sparse linear algebra.";
  }
  if (n_cams < kDenseSchurMaxVariableCams) {
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.preconditioner_type = ceres::JACOBI;
  } else {
    // ITERATIVE_SCHUR + JACOBI: the only combination that is unconditionally robust
    // for large aerial BA.
    //
    // CLUSTER_JACOBI was tried here but fails for oblique multi-camera systems:
    // cameras at the same station (5-camera rig) are within centimetres of each other
    // while flying at hundreds of metres, so the baseline inside each cluster is
    // effectively zero. The cluster-level Schur complement is nearly singular, and
    // CLUSTER_JACOBI's internal Cholesky factorisation fails with "Preconditioner
    // update failed" – the same PD problem that kills SPARSE_SCHUR.
    //
    // Plain JACOBI uses only the diagonal of the Schur complement (= sum of squared
    // Jacobian columns per camera DOF), which is always ≥ 0 as long as each camera
    // has at least one observation. No factorisation → no PD requirement → always works.
    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    options.preconditioner_type = ceres::JACOBI;
  }
  options.num_threads = input.num_threads > 0
                            ? input.num_threads
                            : static_cast<int>(std::thread::hardware_concurrency());
  if (options.num_threads > static_cast<int>(std::thread::hardware_concurrency())) {
    LOG(WARNING) << "Requested num_threads=" << options.num_threads
                 << " exceeds hardware concurrency; using max available threads instead.";
    options.num_threads = static_cast<int>(std::thread::hardware_concurrency());
  }
  if (input.solver_gradient_tolerance > 0.0)
    options.gradient_tolerance = input.solver_gradient_tolerance;
  if (input.solver_function_tolerance > 0.0)
    options.function_tolerance = input.solver_function_tolerance;
  if (input.solver_parameter_tolerance > 0.0)
    options.parameter_tolerance = input.solver_parameter_tolerance;

  // #region agent log
  if (VLOG_IS_ON(1)) {
    diagnose_ba_input_pre_solve(input, result->points3d, n_cams, n_distinct);
  }
  // #endregion

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Fallback: if DENSE_SCHUR failed (very rare – LAPACK pivot failure on a tiny local
  // BA with extreme near-degeneracy), retry with ITERATIVE_SCHUR + JACOBI which is
  // unconditionally robust (no Cholesky required).
  // For the large-scene path (ITERATIVE_SCHUR + JACOBI), there is no further fallback
  // since JACOBI is already the most conservative choice.
  if (!summary.IsSolutionUsable() && options.linear_solver_type == ceres::DENSE_SCHUR) {
    LOG(WARNING) << "global_bundle_analytic: DENSE_SCHUR failed (" << summary.message
                 << "), retrying with ITERATIVE_SCHUR + JACOBI";
    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    options.preconditioner_type = ceres::JACOBI;
    ceres::Solve(options, &problem, &summary);
  }

  if (!summary.IsSolutionUsable()) {
    LOG(INFO) << summary.FullReport();
    LOG(WARNING) << "global_bundle_analytic: " << summary.message;
    return false;
  }
  LOG(INFO) << summary.FullReport();

  const double rmse_before = (summary.num_residuals > 0)
                                 ? std::sqrt(summary.initial_cost * 2.0 / summary.num_residuals)
                                 : 0.0;
  const double rmse_after = (summary.num_residuals > 0)
                                ? std::sqrt(summary.final_cost * 2.0 / summary.num_residuals)
                                : 0.0;
  LOG(INFO) << "global_bundle_analytic: RMSE  before=" << rmse_before << " px  after=" << rmse_after
            << " px"
            << "  iters=" << summary.iterations.size() << "  " << summary.BriefReport();
  // ── Extract poses: q → R,  C stored directly ─────────────────────────────
  result->poses_R.resize(static_cast<size_t>(n_cams));
  result->poses_C.resize(static_cast<size_t>(n_cams));
  for (int i = 0; i < n_cams; ++i) {
    const double* pd = poses_data.data() + static_cast<size_t>(i) * 7;
    const Eigen::Quaterniond q(pd[3], pd[0], pd[1], pd[2]); // (w,x,y,z)
    result->poses_R[static_cast<size_t>(i)] = q.normalized().toRotationMatrix();
    result->poses_C[static_cast<size_t>(i)] = Eigen::Vector3d(pd[4], pd[5], pd[6]);
  }

  // ── Extract cameras ───────────────────────────────────────────────────────
  result->cameras.resize(static_cast<size_t>(n_distinct));
  for (int c = 0; c < n_distinct; ++c) {
    const double* ip = intr_params.data() + static_cast<size_t>(c) * kAnalyticIntrCount;
    auto& K = result->cameras[static_cast<size_t>(c)];
    K.fx = ip[kFx];
    K.fy = ip[kSigma] * ip[kFx];
    K.cx = ip[kCx];
    K.cy = ip[kCy];
    K.k1 = ip[kK1];
    K.k2 = ip[kK2];
    K.k3 = ip[kK3];
    K.p1 = ip[kP1];
    K.p2 = ip[kP2];
  }

  // points are already in result->points3d (optimised in-place)
  result->success = true;
  result->num_residuals = static_cast<int>(summary.num_residuals);
  result->rmse_px = rmse_after;
  return true;
}

} // namespace sfm
} // namespace insight
