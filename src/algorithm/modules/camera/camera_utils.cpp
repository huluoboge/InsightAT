/**
 * @file  camera_utils.cpp
 * @brief Camera distortion utilities – Bentley-compatible 5-parameter Brown-Conrady model.
 *
 * Implements apply_distortion, remove_distortion, undistort_point, undistort_points.
 * See camera_utils.h for the mathematical description.
 */

#include "camera_utils.h"

#include <cmath>
#include <cstring>

namespace insight {
namespace camera {

// ─────────────────────────────────────────────────────────────────────────────
// Internal helper: evaluate distortion residual at normalised point (u, v).
// Returns (u_dist, v_dist) per the Bentley model.
// ─────────────────────────────────────────────────────────────────────────────

namespace {

void distort_normalised(double u, double v,
                        double k1, double k2, double k3,
                        double p1, double p2,
                        double* u_d, double* v_d) {
  const double r2 = u * u + v * v;
  const double r4 = r2 * r2;
  const double r6 = r4 * r2;
  const double radial = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
  *u_d = radial * u + 2.0 * p2 * u * v + p1 * (r2 + 2.0 * u * u);
  *v_d = radial * v + 2.0 * p1 * u * v + p2 * (r2 + 2.0 * v * v);
}

} // namespace

// ─────────────────────────────────────────────────────────────────────────────
// apply_distortion
// ─────────────────────────────────────────────────────────────────────────────

void apply_distortion(double u_n, double v_n, const Intrinsics& K,
                     double* u_d, double* v_d) {
  if (!u_d || !v_d)
    return;
  if (!K.has_distortion()) {
    *u_d = u_n;
    *v_d = v_n;
    return;
  }
  distort_normalised(u_n, v_n, K.k1, K.k2, K.k3, K.p1, K.p2, u_d, v_d);
}

// ─────────────────────────────────────────────────────────────────────────────
// remove_distortion  (fixed-point iteration with early termination)
// ─────────────────────────────────────────────────────────────────────────────

void remove_distortion(double u_d, double v_d, const Intrinsics& K,
                      double* u_n, double* v_n, int max_iter, double tol) {
  if (!u_n || !v_n)
    return;
  if (!K.has_distortion()) {
    *u_n = u_d;
    *v_n = v_d;
    return;
  }

  const double k1 = K.k1, k2 = K.k2, k3 = K.k3;
  const double p1 = K.p1, p2 = K.p2;

  double u = u_d;
  double v = v_d;
  for (int i = 0; i < max_iter; ++i) {
    double ud, vd;
    distort_normalised(u, v, k1, k2, k3, p1, p2, &ud, &vd);
    const double du = ud - u_d;
    const double dv = vd - v_d;
    if (std::abs(du) < tol && std::abs(dv) < tol)
      break;
    u = u_d - du;
    v = v_d - dv;
  }
  *u_n = u;
  *v_n = v;
}

// ─────────────────────────────────────────────────────────────────────────────
// undistort_point  (distorted pixel → undistorted pixel)
// ─────────────────────────────────────────────────────────────────────────────

void undistort_point(const Intrinsics& K, double u_px_in, double v_px_in,
                    double* u_px_out, double* v_px_out) {
  if (!u_px_out || !v_px_out)
    return;
  if (!K.has_distortion()) {
    *u_px_out = u_px_in;
    *v_px_out = v_px_in;
    return;
  }

  const double fx = K.fx, fy = K.fy, cx = K.cx, cy = K.cy;
  const double u_d = (u_px_in - cx) / fx;
  const double v_d = (v_px_in - cy) / fy;

  double u_n, v_n;
  remove_distortion(u_d, v_d, K, &u_n, &v_n);

  // undistorted normalised → pixel
  *u_px_out = fx * u_n + cx;
  *v_px_out = fy * v_n + cy;
}

// ─────────────────────────────────────────────────────────────────────────────
// undistort_points  (batch)
// ─────────────────────────────────────────────────────────────────────────────

void undistort_points(const Intrinsics& K,
                     const float* u_in, const float* v_in,
                     float* u_out, float* v_out, int n) {
  if (n <= 0 || !u_in || !v_in || !u_out || !v_out)
    return;

  if (!K.has_distortion()) {
    if (u_out != u_in)
      std::memcpy(u_out, u_in, static_cast<size_t>(n) * sizeof(float));
    if (v_out != v_in)
      std::memcpy(v_out, v_in, static_cast<size_t>(n) * sizeof(float));
    return;
  }

  const double fx = K.fx, fy = K.fy, cx = K.cx, cy = K.cy;
  const double k1 = K.k1, k2 = K.k2, k3 = K.k3, p1 = K.p1, p2 = K.p2;

  constexpr double tol = 1e-8;
  constexpr int max_iter = 20;

  for (int i = 0; i < n; ++i) {
    const double u_d = (static_cast<double>(u_in[i]) - cx) / fx;
    const double v_d = (static_cast<double>(v_in[i]) - cy) / fy;

    double u = u_d, v = v_d;
    for (int it = 0; it < max_iter; ++it) {
      double ud, vd;
      distort_normalised(u, v, k1, k2, k3, p1, p2, &ud, &vd);
      const double du = ud - u_d;
      const double dv = vd - v_d;
      if (std::abs(du) < tol && std::abs(dv) < tol)
        break;
      u = u_d - du;
      v = v_d - dv;
    }

    u_out[i] = static_cast<float>(fx * u + cx);
    v_out[i] = static_cast<float>(fy * v + cy);
  }
}

}  // namespace camera
}  // namespace insight
