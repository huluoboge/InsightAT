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

#include <cmath>
#include <glog/logging.h>

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

inline void sola_dRp_dq(double px, double py, double pz,
                         double qx, double qy, double qz, double qw,
                         double J[3][4]) {
  const double qxpx = qx*px, qxpy = qx*py, qxpz = qx*pz;
  const double qypx = qy*px, qypy = qy*py, qypz = qy*pz;
  const double qzpx = qz*px, qzpy = qz*py, qzpz = qz*pz;
  const double qwpx = qw*px, qwpy = qw*py, qwpz = qw*pz;

  const double d  = 2.0 * (qxpx + qypy + qzpz);            // diagonal
  const double a  = 2.0 * (qypx - qxpy - qwpz);            // (1,0)
  const double b  = 2.0 * (-qzpx + qxpz - qwpy);           // (2,0) negated below
  const double c  = 2.0 * (qzpy - qypz - qwpx);            // (2,1)

  J[0][0] =  d;  J[0][1] = -a;  J[0][2] =  b;  J[0][3] = 2.0*(qwpx + qypz - qzpy);
  J[1][0] =  a;  J[1][1] =  d;  J[1][2] = -c;  J[1][3] = 2.0*(qwpy - qxpz + qzpx);
  J[2][0] = -b;  J[2][1] =  c;  J[2][2] =  d;  J[2][3] = 2.0*(qwpz + qxpy - qypx);
}

} // namespace

// ─────────────────────────────────────────────────────────────────────────────
// ReprojectionCostAnalytic::Evaluate
// ─────────────────────────────────────────────────────────────────────────────

bool ReprojectionCostAnalytic::Evaluate(double const* const* params,
                                        double* residuals,
                                        double** jacobians) const {
  const double* intr = params[0];
  const double* pose = params[1];
  const double* pt   = params[2];

  // ── Intrinsics ────────────────────────────────────────────────────────────
  const double fx    = intr[kFx];
  const double sigma = intr[kSigma];
  const double cx    = intr[kCx];
  const double cy    = intr[kCy];
  const double k1    = intr[kK1];
  const double k2    = intr[kK2];
  const double k3    = intr[kK3];
  const double p1    = intr[kP1];
  const double p2    = intr[kP2];
  const double fy    = sigma * fx;

  // ── Quaternion + camera centre ────────────────────────────────────────────
  const double qx = pose[0], qy = pose[1], qz = pose[2], qw = pose[3];

  // Rotation matrix – q²-form (consistent with Sola Jacobian)
  const double xx = qx*qx, yy = qy*qy, zz = qz*qz, ww = qw*qw;
  const double xy = qx*qy, xz = qx*qz, xw = qx*qw;
  const double yz = qy*qz, yw = qy*qw, zw = qz*qw;

  const double R00 = ww+xx-yy-zz, R01 = 2.0*(xy-zw),   R02 = 2.0*(xz+yw);
  const double R10 = 2.0*(xy+zw),  R11 = ww-xx+yy-zz,  R12 = 2.0*(yz-xw);
  const double R20 = 2.0*(xz-yw),  R21 = 2.0*(yz+xw),  R22 = ww-xx-yy+zz;

  // Xw = X − C
  const double Xwx = pt[0] - pose[4];
  const double Xwy = pt[1] - pose[5];
  const double Xwz = pt[2] - pose[6];

  // Xc = R · Xw
  const double xc = R00*Xwx + R01*Xwy + R02*Xwz;
  const double yc = R10*Xwx + R11*Xwy + R12*Xwz;
  const double zc = R20*Xwx + R21*Xwy + R22*Xwz;

  if (zc < 1e-12) {
    residuals[0] = residuals[1] = 1e6;
    if (jacobians) {
      if (jacobians[0]) std::fill_n(jacobians[0], 2 * kAnalyticIntrCount, 0.0);
      if (jacobians[1]) std::fill_n(jacobians[1], 14, 0.0);
      if (jacobians[2]) std::fill_n(jacobians[2], 6, 0.0);
    }
    return true;
  }

  // ── Normalised image coordinates ──────────────────────────────────────────
  const double inv_z  = 1.0 / zc;
  const double inv_z2 = inv_z * inv_z;
  const double xu  = xc * inv_z;
  const double yu  = yc * inv_z;
  const double xu2 = xu * xu;
  const double yu2 = yu * yu;
  const double r2  = xu2 + yu2;
  const double r4  = r2 * r2;
  const double r6  = r4 * r2;

  const double rad = 1.0 + k1*r2 + k2*r4 + k3*r6;

  // Tangential distortion (Bentley convention – matches camera_utils.cpp)
  const double tx = 2.0*p2*xu*yu + p1*(r2 + 2.0*xu2);
  const double ty = 2.0*p1*xu*yu + p2*(r2 + 2.0*yu2);

  const double dx = xu * rad + tx;
  const double dy = yu * rad + ty;

  // ── Residuals ─────────────────────────────────────────────────────────────
  residuals[0] = fx * dx + cx - u_obs_;
  residuals[1] = fy * dy + cy - v_obs_;

  if (!jacobians) return true;

  // ── Intermediate derivatives ──────────────────────────────────────────────
  const double drad_c   = 2.0 * (k1 + 2.0*k2*r2 + 3.0*k3*r4);
  const double drad_dxu = drad_c * xu;
  const double drad_dyu = drad_c * yu;

  // d(tx)/d(xu, yu)
  const double dtx_dxu = 2.0*p2*yu + 6.0*p1*xu;
  const double dtx_dyu = 2.0*p2*xu + 2.0*p1*yu;
  // d(ty)/d(xu, yu)
  const double dty_dxu = 2.0*p1*yu + 2.0*p2*xu;   // = dtx_dyu with p1↔p2
  const double dty_dyu = 2.0*p1*xu + 6.0*p2*yu;

  const double ddx_dxu = rad + xu*drad_dxu + dtx_dxu;
  const double ddx_dyu =       xu*drad_dyu + dtx_dyu;
  const double ddy_dxu =       yu*drad_dxu + dty_dxu;
  const double ddy_dyu = rad + yu*drad_dyu + dty_dyu;

  // d(res)/d(xc, yc, zc)
  const double dr0_dxc = fx * ddx_dxu * inv_z;
  const double dr0_dyc = fx * ddx_dyu * inv_z;
  const double dr0_dzc = -fx * (ddx_dxu * xu + ddx_dyu * yu) * inv_z;

  const double dr1_dxc = fy * ddy_dxu * inv_z;
  const double dr1_dyc = fy * ddy_dyu * inv_z;
  const double dr1_dzc = -fy * (ddy_dxu * xu + ddy_dyu * yu) * inv_z;

  // Shared R·column products  (point Jacobian = +jp, centre Jacobian = −jp)
  const double jp00 = dr0_dxc*R00 + dr0_dyc*R10 + dr0_dzc*R20;
  const double jp01 = dr0_dxc*R01 + dr0_dyc*R11 + dr0_dzc*R21;
  const double jp02 = dr0_dxc*R02 + dr0_dyc*R12 + dr0_dzc*R22;
  const double jp10 = dr1_dxc*R00 + dr1_dyc*R10 + dr1_dzc*R20;
  const double jp11 = dr1_dxc*R01 + dr1_dyc*R11 + dr1_dzc*R21;
  const double jp12 = dr1_dxc*R02 + dr1_dyc*R12 + dr1_dzc*R22;

  // ── J wrt intrinsics [fx, sigma, cx, cy, k1, k2, k3, p1, p2] ────────────
  if (jacobians[0]) {
    double* J = jacobians[0]; // row-major 2×9
    // row 0
    J[kFx]    = dx;
    J[kSigma] = 0.0;
    J[kCx]    = 1.0;
    J[kCy]    = 0.0;
    J[kK1]    = fx * xu * r2;
    J[kK2]    = fx * xu * r4;
    J[kK3]    = fx * xu * r6;
    J[kP1]    = fx * (r2 + 2.0*xu2);   // d(tx)/d(p1)
    J[kP2]    = fx * 2.0*xu*yu;        // d(tx)/d(p2)
    // row 1
    J += kAnalyticIntrCount;
    J[kFx]    = sigma * dy;
    J[kSigma] = fx * dy;
    J[kCx]    = 0.0;
    J[kCy]    = 1.0;
    J[kK1]    = fy * yu * r2;
    J[kK2]    = fy * yu * r4;
    J[kK3]    = fy * yu * r6;
    J[kP1]    = fy * 2.0*xu*yu;        // d(ty)/d(p1)
    J[kP2]    = fy * (r2 + 2.0*yu2);   // d(ty)/d(p2)
  }

  // ── J wrt pose [qx, qy, qz, qw, Cx, Cy, Cz] ────────────────────────────
  if (jacobians[1]) {
    double* J = jacobians[1]; // row-major 2×7

    // Quaternion columns 0-3 via Sola d(R·Xw)/dq
    double dXc_dq[3][4];
    sola_dRp_dq(Xwx, Xwy, Xwz, qx, qy, qz, qw, dXc_dq);

    for (int i = 0; i < 4; ++i) {
      J[    i] = dr0_dxc*dXc_dq[0][i] + dr0_dyc*dXc_dq[1][i] + dr0_dzc*dXc_dq[2][i];
      J[7 + i] = dr1_dxc*dXc_dq[0][i] + dr1_dyc*dXc_dq[1][i] + dr1_dzc*dXc_dq[2][i];
    }

    // Camera-centre columns 4-6: −jp
    J[4] = -jp00;  J[5] = -jp01;  J[6] = -jp02;
    J[11] = -jp10; J[12] = -jp11; J[13] = -jp12;
  }

  // ── J wrt 3D point [X, Y, Z] ─────────────────────────────────────────────
  if (jacobians[2]) {
    double* J = jacobians[2]; // row-major 2×3
    J[0] = jp00;  J[1] = jp01;  J[2] = jp02;
    J[3] = jp10;  J[4] = jp11;  J[5] = jp12;
  }

  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// ReprojectionCostAnalyticWeighted::Evaluate (same as above, then scale by 1/scale)
// ─────────────────────────────────────────────────────────────────────────────

bool ReprojectionCostAnalyticWeighted::Evaluate(double const* const* params,
                                                double* residuals,
                                                double** jacobians) const {
  const double* intr = params[0];
  const double* pose = params[1];
  const double* pt   = params[2];

  const double fx    = intr[kFx];
  const double sigma = intr[kSigma];
  const double cx    = intr[kCx];
  const double cy    = intr[kCy];
  const double k1    = intr[kK1];
  const double k2    = intr[kK2];
  const double k3    = intr[kK3];
  const double p1    = intr[kP1];
  const double p2    = intr[kP2];
  const double fy    = sigma * fx;

  const double qx = pose[0], qy = pose[1], qz = pose[2], qw = pose[3];
  const double xx = qx*qx, yy = qy*qy, zz = qz*qz, ww = qw*qw;
  const double xy = qx*qy, xz = qx*qz, xw = qx*qw;
  const double yz = qy*qz, yw = qy*qw, zw = qz*qw;

  const double R00 = ww+xx-yy-zz, R01 = 2.0*(xy-zw),   R02 = 2.0*(xz+yw);
  const double R10 = 2.0*(xy+zw),  R11 = ww-xx+yy-zz,  R12 = 2.0*(yz-xw);
  const double R20 = 2.0*(xz-yw),  R21 = 2.0*(yz+xw),  R22 = ww-xx-yy+zz;

  const double Xwx = pt[0] - pose[4];
  const double Xwy = pt[1] - pose[5];
  const double Xwz = pt[2] - pose[6];

  const double xc = R00*Xwx + R01*Xwy + R02*Xwz;
  const double yc = R10*Xwx + R11*Xwy + R12*Xwz;
  const double zc = R20*Xwx + R21*Xwy + R22*Xwz;

  if (zc < 1e-12) {
    residuals[0] = residuals[1] = 1e6 * weight_;
    if (jacobians) {
      if (jacobians[0]) std::fill_n(jacobians[0], 2 * kAnalyticIntrCount, 0.0);
      if (jacobians[1]) std::fill_n(jacobians[1], 14, 0.0);
      if (jacobians[2]) std::fill_n(jacobians[2], 6, 0.0);
    }
    return true;
  }

  const double inv_z  = 1.0 / zc;
  const double inv_z2 = inv_z * inv_z;
  const double xu  = xc * inv_z;
  const double yu  = yc * inv_z;
  const double xu2 = xu * xu;
  const double yu2 = yu * yu;
  const double r2  = xu2 + yu2;
  const double r4  = r2 * r2;
  const double r6  = r4 * r2;
  const double rad = 1.0 + k1*r2 + k2*r4 + k3*r6;
  const double tx  = 2.0*p2*xu*yu + p1*(r2 + 2.0*xu2);
  const double ty  = 2.0*p1*xu*yu + p2*(r2 + 2.0*yu2);
  const double dx  = xu * rad + tx;
  const double dy  = yu * rad + ty;

  residuals[0] = (fx * dx + cx - u_obs_) * weight_;
  residuals[1] = (fy * dy + cy - v_obs_) * weight_;

  if (!jacobians) return true;

  const double drad_c   = 2.0 * (k1 + 2.0*k2*r2 + 3.0*k3*r4);
  const double drad_dxu = drad_c * xu;
  const double drad_dyu = drad_c * yu;
  const double dtx_dxu = 2.0*p2*yu + 6.0*p1*xu;
  const double dtx_dyu = 2.0*p2*xu + 2.0*p1*yu;
  const double dty_dxu = 2.0*p1*yu + 2.0*p2*xu;
  const double dty_dyu = 2.0*p1*xu + 6.0*p2*yu;
  const double ddx_dxu = rad + xu*drad_dxu + dtx_dxu;
  const double ddx_dyu =       xu*drad_dyu + dtx_dyu;
  const double ddy_dxu =       yu*drad_dxu + dty_dxu;
  const double ddy_dyu = rad + yu*drad_dyu + dty_dyu;
  const double dr0_dxc = fx * ddx_dxu * inv_z;
  const double dr0_dyc = fx * ddx_dyu * inv_z;
  const double dr0_dzc = -fx * (ddx_dxu * xu + ddx_dyu * yu) * inv_z;
  const double dr1_dxc = fy * ddy_dxu * inv_z;
  const double dr1_dyc = fy * ddy_dyu * inv_z;
  const double dr1_dzc = -fy * (ddy_dxu * xu + ddy_dyu * yu) * inv_z;
  const double jp00 = dr0_dxc*R00 + dr0_dyc*R10 + dr0_dzc*R20;
  const double jp01 = dr0_dxc*R01 + dr0_dyc*R11 + dr0_dzc*R21;
  const double jp02 = dr0_dxc*R02 + dr0_dyc*R12 + dr0_dzc*R22;
  const double jp10 = dr1_dxc*R00 + dr1_dyc*R10 + dr1_dzc*R20;
  const double jp11 = dr1_dxc*R01 + dr1_dyc*R11 + dr1_dzc*R21;
  const double jp12 = dr1_dxc*R02 + dr1_dyc*R12 + dr1_dzc*R22;

  if (jacobians[0]) {
    double* J = jacobians[0];
    J[kFx]    = dx * weight_;
    J[kSigma] = 0.0;
    J[kCx]    = weight_;
    J[kCy]    = 0.0;
    J[kK1]    = fx * xu * r2 * weight_;
    J[kK2]    = fx * xu * r4 * weight_;
    J[kK3]    = fx * xu * r6 * weight_;
    J[kP1]    = fx * (r2 + 2.0*xu2) * weight_;
    J[kP2]    = fx * 2.0*xu*yu * weight_;
    J += kAnalyticIntrCount;
    J[kFx]    = sigma * dy * weight_;
    J[kSigma] = fx * dy * weight_;
    J[kCx]    = 0.0;
    J[kCy]    = weight_;
    J[kK1]    = fy * yu * r2 * weight_;
    J[kK2]    = fy * yu * r4 * weight_;
    J[kK3]    = fy * yu * r6 * weight_;
    J[kP1]    = fy * 2.0*xu*yu * weight_;
    J[kP2]    = fy * (r2 + 2.0*yu2) * weight_;
  }
  if (jacobians[1]) {
    double* J = jacobians[1];
    double dXc_dq[3][4];
    sola_dRp_dq(Xwx, Xwy, Xwz, qx, qy, qz, qw, dXc_dq);
    for (int i = 0; i < 4; ++i) {
      J[    i] = (dr0_dxc*dXc_dq[0][i] + dr0_dyc*dXc_dq[1][i] + dr0_dzc*dXc_dq[2][i]) * weight_;
      J[7 + i] = (dr1_dxc*dXc_dq[0][i] + dr1_dyc*dXc_dq[1][i] + dr1_dzc*dXc_dq[2][i]) * weight_;
    }
    J[4] = -jp00*weight_;  J[5] = -jp01*weight_;  J[6] = -jp02*weight_;
    J[11] = -jp10*weight_; J[12] = -jp11*weight_; J[13] = -jp12*weight_;
  }
  if (jacobians[2]) {
    double* J = jacobians[2];
    J[0] = jp00*weight_;  J[1] = jp01*weight_;  J[2] = jp02*weight_;
    J[3] = jp10*weight_;  J[4] = jp11*weight_;  J[5] = jp12*weight_;
  }
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// global_bundle_analytic
// ─────────────────────────────────────────────────────────────────────────────

bool global_bundle_analytic(const BAInput& input, BAResult* result, int max_iterations) {
  if (!result) return false;
  if (input.poses_R.size() != input.poses_C.size() || input.poses_R.empty() ||
      input.points3d.empty() || input.observations.empty())
    return false;

  const int n_cams = static_cast<int>(input.poses_R.size());
  const int n_pts  = static_cast<int>(input.points3d.size());
  const bool use_multicam =
      !input.image_camera_index.empty() && !input.cameras.empty() &&
      static_cast<int>(input.image_camera_index.size()) == n_cams;

  if (!use_multicam) {
    LOG(WARNING) << "global_bundle_analytic: image_camera_index and cameras required";
    return false;
  }

  const int n_distinct = static_cast<int>(input.cameras.size());

  // ── Intrinsics: Intrinsics → analytic block [fx, sigma, cx, cy, k…, p…] ──
  std::vector<double> intr_params(static_cast<size_t>(n_distinct) * kAnalyticIntrCount);
  for (int c = 0; c < n_distinct; ++c) {
    const auto& K  = input.cameras[static_cast<size_t>(c)];
    double*     ip = intr_params.data() + static_cast<size_t>(c) * kAnalyticIntrCount;
    ip[kFx]    = K.fx;
    ip[kSigma] = (K.fx != 0.0) ? K.fy / K.fx : 1.0;
    ip[kCx]    = K.cx;
    ip[kCy]    = K.cy;
    ip[kK1]    = K.k1;
    ip[kK2]    = K.k2;
    ip[kK3]    = K.k3;
    ip[kP1]    = K.p1;
    ip[kP2]    = K.p2;
  }

  // ── Poses: R → q,  C taken directly ──────────────────────────────────────
  std::vector<double> poses_data(static_cast<size_t>(n_cams) * 7);
  for (int i = 0; i < n_cams; ++i) {
    const Eigen::Matrix3d& Ri = input.poses_R[static_cast<size_t>(i)];
    const Eigen::Vector3d& C  = input.poses_C[static_cast<size_t>(i)];
    const Eigen::Quaterniond q(Ri);
    double* pd = poses_data.data() + static_cast<size_t>(i) * 7;
    pd[0] = q.x(); pd[1] = q.y(); pd[2] = q.z(); pd[3] = q.w();
    pd[4] = C.x(); pd[5] = C.y(); pd[6] = C.z();
  }

  // ── Points: write into result for in-place optimisation ───────────────────
  result->points3d = input.points3d;

  // ── Build Ceres problem ───────────────────────────────────────────────────
  ceres::Problem problem;
  ceres::LossFunction* loss = new ceres::HuberLoss(10.0);

  for (const auto& obs : input.observations) {
    if (obs.image_index < 0 || obs.image_index >= n_cams ||
        obs.point_index < 0 || obs.point_index >= n_pts)
      continue;
    const int cam_idx = input.image_camera_index[static_cast<size_t>(obs.image_index)];
    if (cam_idx < 0 || cam_idx >= n_distinct) continue;

    double* ip = intr_params.data() + static_cast<size_t>(cam_idx) * kAnalyticIntrCount;
    double* pp = poses_data.data()  + static_cast<size_t>(obs.image_index) * 7;
    double* xp = result->points3d[static_cast<size_t>(obs.point_index)].data();

    const double scale = (obs.scale > 1e-12) ? obs.scale : 1.0;
    ceres::CostFunction* cost = (scale == 1.0)
        ? static_cast<ceres::CostFunction*>(new ReprojectionCostAnalytic(obs.u, obs.v))
        : static_cast<ceres::CostFunction*>(new ReprojectionCostAnalyticWeighted(obs.u, obs.v, scale));
    problem.AddResidualBlock(cost, loss, ip, pp, xp);
  }

  // ── Parameterisation for quaternion+centre (only blocks in the problem) ───
  for (int i = 0; i < n_cams; ++i) {
    double* pp = poses_data.data() + static_cast<size_t>(i) * 7;
    if (!problem.HasParameterBlock(pp)) continue;
    problem.SetParameterization(
        pp,
        new ceres::ProductParameterization(
            new ceres::EigenQuaternionParameterization(),
            new ceres::IdentityParameterization(3)));
  }

  // Fix poses: only honour the explicit fix_pose mask set by the caller.
  // The caller (run_global_ba / run_local_ba) is responsible for fixing the coordinate-system
  // anchor.  We MUST NOT unconditionally fix i==0 here because BA image index 0 is just the
  // lowest-registered-image-index, which is a PnP-estimated camera that may have drift.
  // Fixing a drifted camera alongside the true anchor (fix_pose[anchor_idx]=true) creates two
  // conflicting rigid constraints → Ceres diverges → RMSE explodes into hundreds of thousands.
  for (int i = 0; i < n_cams; ++i) {
    double* pp = poses_data.data() + static_cast<size_t>(i) * 7;
    if (!problem.HasParameterBlock(pp)) continue;
    const bool fix =
        (input.fix_pose.size() > static_cast<size_t>(i) && input.fix_pose[static_cast<size_t>(i)]);
    if (fix)
      problem.SetParameterBlockConstant(pp);
  }

  // Intrinsics: constant vs optimise (fix_intrinsics_flags per camera, bitmask per parameter)
  for (int c = 0; c < n_distinct; ++c) {
    double* ip = intr_params.data() + static_cast<size_t>(c) * kAnalyticIntrCount;
    if (!problem.HasParameterBlock(ip)) continue;
    if (!input.optimize_intrinsics) {
      problem.SetParameterBlockConstant(ip);
      continue;
    }
    uint32_t flags = (input.fix_intrinsics_flags.size() > static_cast<size_t>(c))
                         ? static_cast<uint32_t>(input.fix_intrinsics_flags[static_cast<size_t>(c)])
                         : 0u;
    std::vector<int> fixed;
    for (int i = 0; i < kAnalyticIntrCount; ++i)
      if ((flags >> i) & 1u) fixed.push_back(i);
    if (fixed.size() >= static_cast<size_t>(kAnalyticIntrCount))
      problem.SetParameterBlockConstant(ip);
    else if (!fixed.empty())
      problem.SetParameterization(ip, new ceres::SubsetParameterization(kAnalyticIntrCount, fixed));
  }

  // Fix 3D points: optional fix_point[p]
  for (int p = 0; p < n_pts; ++p) {
    double* xp = result->points3d[static_cast<size_t>(p)].data();
    if (!problem.HasParameterBlock(xp)) continue;
    if (input.fix_point.size() > static_cast<size_t>(p) && input.fix_point[static_cast<size_t>(p)])
      problem.SetParameterBlockConstant(xp);
  }

  // ── Solve ─────────────────────────────────────────────────────────────────
  ceres::Solver::Options options;
  options.max_num_iterations           = max_iterations;
  options.minimizer_progress_to_stdout = false;

  // Solver strategy: SPARSE_SCHUR (CHOLMOD, fast for large scenes) first,
  // fall back to DENSE_SCHUR (LAPACK) on failure.
  // DENSE_SCHUR is used as fallback for all cases:
  //  - near-planar scenes make Schur complement ill-conditioned → CHOLMOD fails
  //  - free intrinsics coupling can also violate positive-definiteness
  // LAPACK's pivot threshold is more permissive than CHOLMOD's, and for
  // typical BA windows (≤70 cams → ≤630×630 Schur) it is sub-second.
  // Avoids ITERATIVE_SCHUR which is slow for small-scale high-condition problems.
  options.linear_solver_type = ceres::SPARSE_SCHUR;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  if (!summary.IsSolutionUsable()) {
    LOG(WARNING) << "global_bundle_analytic: SPARSE_SCHUR failed ("
                 << summary.message << "), retrying with DENSE_SCHUR";
    options.linear_solver_type  = ceres::DENSE_SCHUR;
    options.preconditioner_type = ceres::JACOBI;
    ceres::Solve(options, &problem, &summary);
  }

  if (!summary.IsSolutionUsable()) {
    LOG(WARNING) << "global_bundle_analytic: " << summary.message;
    return false;
  }

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
    auto& K  = result->cameras[static_cast<size_t>(c)];
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
  result->success       = true;
  result->num_residuals = static_cast<int>(summary.num_residuals);

  // ── True reprojection RMSE (unrobustified, pixel units) ──────────────────
  // summary.final_cost is the Huber-modified cost, not the true squared-pixel sum,
  // so we recompute RMSE manually using the optimised poses, points, and cameras.
  {
    double sq_sum = 0.0;
    int    n_valid = 0;
    for (const auto& obs : input.observations) {
      if (obs.image_index < 0 || obs.image_index >= n_cams ||
          obs.point_index < 0 || obs.point_index >= n_pts)
        continue;
      const int cam_idx = input.image_camera_index[static_cast<size_t>(obs.image_index)];
      if (cam_idx < 0 || cam_idx >= n_distinct)
        continue;
      const auto& K  = result->cameras[static_cast<size_t>(cam_idx)];
      const Eigen::Vector3d& X = result->points3d[static_cast<size_t>(obs.point_index)];
      const Eigen::Matrix3d& R = result->poses_R[static_cast<size_t>(obs.image_index)];
      const Eigen::Vector3d& C = result->poses_C[static_cast<size_t>(obs.image_index)];
      const Eigen::Vector3d Xc = R * (X - C);
      double eu, ev;
      if (Xc(2) <= 1e-12) {
        eu = ev = 1e6; // mirrors cost-function sentinel for behind-camera points
      } else {
        const double inv_z = 1.0 / Xc(2);
        const double xu = Xc(0) * inv_z, yu = Xc(1) * inv_z;
        const double r2 = xu*xu + yu*yu, r4 = r2*r2, r6 = r4*r2;
        const double rad = 1.0 + K.k1*r2 + K.k2*r4 + K.k3*r6;
        const double tx = 2.0*K.p2*xu*yu + K.p1*(r2 + 2.0*xu*xu);
        const double ty = 2.0*K.p1*xu*yu + K.p2*(r2 + 2.0*yu*yu);
        eu = K.fx*(xu*rad + tx) + K.cx - obs.u;
        ev = K.fy*(yu*rad + ty) + K.cy - obs.v;
      }
      sq_sum += eu*eu + ev*ev;
      ++n_valid;
    }
    // RMSE = sqrt( sum(eu²+ev²) / num_observations )
    // The denominator counts observations (each contributing a 2D error vector),
    // so this matches the standard "reprojection error per observation" metric.
    result->rmse_px = (n_valid > 0) ? std::sqrt(sq_sum / n_valid) : 0.0;
  }
  return true;
}

} // namespace sfm
} // namespace insight
