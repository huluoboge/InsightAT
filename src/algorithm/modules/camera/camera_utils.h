/**
 * @file  camera_utils.h
 * @brief Camera distortion utilities – apply / remove Bentley-compatible 5-parameter
 *        Brown-Conrady distortion model and batch undistort pixel observations.
 *
 * Uses algorithm-only Intrinsics (no database dependency). See camera_types.h.
 *
 * Distortion model (identical to Bentley ContextCapture perspective camera model):
 *   https://docs.bentley.com/LiveContent/web/ContextCapture%20Help-v17/en/GUID-2D452A8A-A4FE-450D-A0CA-9336DCF1238A.html
 *
 *   Inputs: normalised coordinates (u, v) = K⁻¹ * [px, py, 1]ᵀ,  r² = u² + v²
 *
 *   u_dist = (1 + k1·r² + k2·r⁴ + k3·r⁶)·u  +  2·p2·u·v + p1·(r² + 2·u²)
 *   v_dist = (1 + k1·r² + k2·r⁴ + k3·r⁶)·v  +  2·p1·u·v + p2·(r² + 2·v²)
 *
 * Undistortion is solved by fixed-point iteration directly in pixel space:
 *   u_{n+1} = u_n − (distort_px(u_n) − u_observed)
 * Iterating in pixel space ensures the convergence tolerance is physically
 * meaningful (pixels) and avoids the ill-conditioning that occurs when k3
 * drives large normalised-space residuals near the sensor boundary.
 *
 * Usage
 * ─────
 *   insight::camera::Intrinsics K = ...;
 *   insight::camera::undistort_point(K, u_px, v_px, &u_out, &v_out);
 *   insight::camera::undistort_points(K, u_in, v_in, u_out, v_out, n);
 */

#pragma once
#ifndef INSIGHT_CAMERA_UTILS_H
#define INSIGHT_CAMERA_UTILS_H

#include "camera_types.h"

namespace insight {
namespace camera {

// ─────────────────────────────────────────────────────────────────────────────
// Forward distortion: normalised → distorted normalised
// ─────────────────────────────────────────────────────────────────────────────

void apply_distortion(double u_n, double v_n, const Intrinsics& K,
                      double* u_d, double* v_d);

// ─────────────────────────────────────────────────────────────────────────────
// Undistortion: distorted pixel → undistorted pixel (single / batch)
//
// Iterates in pixel space until |du|,|dv| < tol_px (default 1e-3 px).
// Typical convergence in 5–15 iterations; max_iter=100 is a safety cap only.
// ─────────────────────────────────────────────────────────────────────────────

void undistort_point(const Intrinsics& K, double u_px_in, double v_px_in,
                     double* u_px_out, double* v_px_out);

void undistort_points(const Intrinsics& K,
                      const float* u_in, const float* v_in,
                      float* u_out, float* v_out, int n);

}  // namespace camera
}  // namespace insight

#endif  // INSIGHT_CAMERA_UTILS_H
