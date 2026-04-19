/**
 * @file  camera_types.h
 * @brief Minimal camera intrinsics for algorithm layer (no database dependency).
 *
 * Database stores full CameraModel (UI/export); algorithms use this struct only.
 * Align with JSON / Bentley 5-parameter model: k1, k2, k3, p1, p2.
 *
 * Reference: Bentley ContextCapture perspective camera model.
 */

#pragma once
#ifndef INSIGHT_CAMERA_TYPES_H
#define INSIGHT_CAMERA_TYPES_H

#include <cmath>
#include <Eigen/Core>

namespace insight {
namespace camera {

/**
 * Convergence delta between two Intrinsics snapshots, with heterogeneous fields
 * decomposed into three independently-thresholded groups.
 *
 *  focal_rel  = (|Δfx|/fx + |Δfy|/fy) / 2   — dimensionless relative focal change
 *  pp_px      = |Δcx| + |Δcy|               — principal-point shift in pixels
 *  distortion = |Δk1|+|Δk2|+|Δk3|+|Δp1|+|Δp2| — dimensionless distortion delta
 */
struct IntrinsicsDelta {
  double focal_rel  = 0.0;
  double pp_px      = 0.0;
  double distortion = 0.0;

  /// Returns true when every component is below its respective threshold.
  bool stable(double thr_focal, double thr_pp, double thr_dist) const {
    return focal_rel < thr_focal && pp_px < thr_pp && distortion < thr_dist;
  }
};

/**
 * Minimal intrinsics for distortion/undistortion and PnP/BA.
 *
 * No dependency on database. Call sites (tools, UI) fill this from
 * JSON or from database::CameraModel when crossing the boundary.
 */
struct Intrinsics {
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  /// Image size in pixels (from EXIF / project export). Used for absolute 2D coverage (e.g.
  /// VisibilityPyramid in resection ordering). When zero, algorithms may infer size from cx/cy.
  int width = 0;
  int height = 0;
  double k1 = 0.0, k2 = 0.0, k3 = 0.0;
  /// Tangential coeffs: Bentley / ContextCapture order (camera_utils.h). COLMAP export uses
  /// OpenCV tangential order (swap): written OpenCV p1 = p2 here, OpenCV p2 = p1 here.
  /// Radial k3: COLMAP ``OPENCV`` has no k3 — use ``FULL_OPENCV`` in exporters when k3 ≠ 0.
  double p1 = 0.0, p2 = 0.0;

  bool has_image_size() const { return width > 0 && height > 0; }

  bool has_distortion() const {
    return k1 != 0.0 || k2 != 0.0 || k3 != 0.0 || p1 != 0.0 || p2 != 0.0;
  }

  /// Compute heterogeneous convergence delta against a previous snapshot.
  IntrinsicsDelta delta(const Intrinsics& prev) const {
    IntrinsicsDelta d;
    double rf = 0.0;
    if (prev.fx > 0.0) rf += std::abs(fx - prev.fx) / prev.fx;
    if (prev.fy > 0.0) rf += std::abs(fy - prev.fy) / prev.fy;
    d.focal_rel  = rf * 0.5;
    d.pp_px      = std::abs(cx - prev.cx) + std::abs(cy - prev.cy);
    d.distortion = std::abs(k1 - prev.k1) + std::abs(k2 - prev.k2) +
                   std::abs(k3 - prev.k3) + std::abs(p1 - prev.p1) +
                   std::abs(p2 - prev.p2);
    return d;
  }

  /// Build camera matrix K
  Eigen::Matrix3d K() const {
    Eigen::Matrix3d m;
    m << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
    return m;
  }
  /// Build K⁻¹ exactly
  Eigen::Matrix3d Kinv() const {
    Eigen::Matrix3d m;
    m << 1.0 / fx, 0.0, -cx / fx, 0.0, 1.0 / fy, -cy / fy, 0.0, 0.0, 1.0;
    return m;
  }
};

} // namespace camera
} // namespace insight

#endif // INSIGHT_CAMERA_TYPES_H
