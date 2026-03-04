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

namespace insight {
namespace camera {

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
  double k1 = 0.0, k2 = 0.0, k3 = 0.0;
  double p1 = 0.0, p2 = 0.0;

  bool has_distortion() const {
    return k1 != 0.0 || k2 != 0.0 || k3 != 0.0 || p1 != 0.0 || p2 != 0.0;
  }
};

}  // namespace camera
}  // namespace insight

#endif  // INSIGHT_CAMERA_TYPES_H
