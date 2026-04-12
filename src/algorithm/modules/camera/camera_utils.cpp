/**
 * @file  camera_utils.cpp
 * @brief Camera distortion utilities – Bentley-compatible 5-parameter Brown-Conrady model.
 *
 * Implements apply_distortion, undistort_point, undistort_points.
 * Undistortion uses pixel-space fixed-point iteration; see camera_utils.h for details.
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
// undistort_point  (distorted pixel → undistorted normalised coords, returned as pixel)
//
// Normalised-space fixed-point iteration:
//   xn_{n+1} = xn_n − ( distort_n(xn_n) − xn_observed )
// Converges in ~5 iterations for typical lens distortion.
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

  // Observed normalised coords (distorted).
  const double xn_obs = (u_px_in - K.cx) / K.fx;
  const double yn_obs = (v_px_in - K.cy) / K.fy;

  // Iterate in normalised space: xn_{n+1} = xn_n - (distort(xn_n) - xn_obs)
  constexpr double kTol = 1e-6;   // normalised units (~1e-6 ≈ 0.004 px for f≈4000)
  constexpr int    kMaxIt = 20;
  double xn = xn_obs, yn = yn_obs;
  for (int i = 0; i < kMaxIt; ++i) {
    double xd, yd;
    distort_normalised(xn, yn, K.k1, K.k2, K.k3, K.p1, K.p2, &xd, &yd);
    const double dxn = xd - xn_obs;
    const double dyn = yd - yn_obs;
    xn -= dxn;
    yn -= dyn;
    if (std::abs(dxn) < kTol && std::abs(dyn) < kTol)
      break;
  }
  *u_px_out = K.fx * xn + K.cx;
  *v_px_out = K.fy * yn + K.cy;
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

  constexpr double kTol = 1e-6;   // normalised units
  constexpr int max_iter = 20;

  for (int i = 0; i < n; ++i) {
    const double xn_obs = (static_cast<double>(u_in[i]) - cx) / fx;
    const double yn_obs = (static_cast<double>(v_in[i]) - cy) / fy;

    double xn = xn_obs, yn = yn_obs;
    for (int it = 0; it < max_iter; ++it) {
      double xd, yd;
      distort_normalised(xn, yn, K.k1, K.k2, K.k3, K.p1, K.p2, &xd, &yd);
      const double dxn = xd - xn_obs;
      const double dyn = yd - yn_obs;
      xn -= dxn;
      yn -= dyn;
      if (std::abs(dxn) < kTol && std::abs(dyn) < kTol)
        break;
    }

    u_out[i] = static_cast<float>(fx * xn + cx);
    v_out[i] = static_cast<float>(fy * yn + cy);
  }
}

}  // namespace camera
}  // namespace insight
