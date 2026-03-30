#include "bundle_adjustment_pba.h"

#include <glog/logging.h>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

// Distortion removal (Brown-Conrady 5-parameter) for pre-undistorting observations
#include "../camera/camera_utils.h"

// PBA headers (CPU-only build; PBA_NO_GPU is defined by the pba_cpu target).
// GetMatrixRotation uses m[0][i] for i=0..8 to read all 9 floats contiguously
// via C layout – technically UB but intentional; suppress the compiler warning.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waggressive-loop-optimizations"
#include "pba/pba.h"
#include "pba/DataInterface.h"
#pragma GCC diagnostic pop

namespace insight {
namespace sfm {

// ─── helpers ─────────────────────────────────────────────────────────────────

static int safe_cam_idx(const BAInput& input, int image_index) {
  const int n = static_cast<int>(input.cameras.size());
  if (n == 0) return 0;
  if (static_cast<int>(input.image_camera_index.size()) <= image_index) return 0;
  return std::min(input.image_camera_index[image_index], n - 1);
}

// ─── main ────────────────────────────────────────────────────────────────────

bool global_bundle_pba(const BAInput& input, BAResult* result,
                       int max_iterations) {
  if (!result) return false;

  const int n_images   = static_cast<int>(input.poses_R.size());
  const int n_distinct = static_cast<int>(input.cameras.size());
  const int n_pts      = static_cast<int>(input.points3d.size());

  if (n_images == 0 || n_pts == 0 || input.observations.empty() ||
      n_distinct == 0)
    return false;

  // ── Build PBA camera data ──────────────────────────────────────────────────
  // We run PBA in the *undistorted pixel* measurement space so the objective matches
  // pixel reprojection error:
  //   1) Undistort (u,v) -> (u_und, v_und) in pixels using Brown-Conrady (our model)
  //   2) Subtract principal point: (u_c, v_c) = (u_und-cx, v_und-cy)
  //   3) Feed (u_c, v_c) as the measurement to PBA with f=fx and NO distortion.
  //
  // PBA assumes square pixels (fx==fy). When fy!=fx we scale v by s=fx/fy so that
  // residuals are in consistent \"x-pixel\" units: v' = s * v_c.
  std::vector<CameraT> camera_data(n_images);

  for (int i = 0; i < n_images; ++i) {
    CameraT& cam             = camera_data[i];
    const Eigen::Matrix3d& R = input.poses_R[i];
    const Eigen::Vector3d& C = input.poses_C[i];

    // Rotation matrix: PBA row-major layout r[i*3+j] = R(i,j)
    float r9[9];
    for (int row = 0; row < 3; ++row)
      for (int col = 0; col < 3; ++col)
        r9[row * 3 + col] = static_cast<float>(R(row, col));
    cam.SetMatrixRotation(r9);

    // Translation T = −R·C
    const double c3[3] = {C.x(), C.y(), C.z()};
    cam.SetCameraCenterAfterRotation(c3);

    // Fixed intrinsics: f = fx (pixels), no radial.
    const camera::Intrinsics& intr = input.cameras[safe_cam_idx(input, i)];
    cam.SetFocalLength(static_cast<float>(intr.fx));
    cam.SetFixedIntrinsic();  // freeze f and radial

    // Gauge anchor: fully fixed camera
    if (static_cast<int>(input.fix_pose.size()) > i && input.fix_pose[i])
      cam.SetConstantCamera();  // constant_camera = 1.0
    // All other cameras: variable pose (constant_camera = 0)
  }

  // ── Build PBA 3-D point data ───────────────────────────────────────────────
  std::vector<Point3D> point_data(n_pts);
  for (int i = 0; i < n_pts; ++i) {
    const float p[3] = {
        static_cast<float>(input.points3d[i].x()),
        static_cast<float>(input.points3d[i].y()),
        static_cast<float>(input.points3d[i].z())};
    point_data[i].SetPoint(p);
  }

  // ── Build PBA observations (undistorted pixels, centred) ───────────────────
  std::vector<Point2D> measurements;
  std::vector<int>     cam_idx_list;
  std::vector<int>     pt_idx_list;
  measurements.reserve(input.observations.size());
  cam_idx_list.reserve(input.observations.size());
  pt_idx_list.reserve(input.observations.size());

  for (const auto& obs : input.observations) {
    if (obs.image_index < 0 || obs.image_index >= n_images) continue;
    if (obs.point_index < 0 || obs.point_index >= n_pts) continue;

    const camera::Intrinsics& intr =
        input.cameras[safe_cam_idx(input, obs.image_index)];

    // Undistort in pixel space using our full Brown-Conrady model.
    double u_und = obs.u, v_und = obs.v;
    insight::camera::undistort_point(intr, obs.u, obs.v, &u_und, &v_und);
    if (!std::isfinite(u_und) || !std::isfinite(v_und)) continue;

    // Centre and scale to match PBA's square-pixel assumption.
    const double u_c = u_und - intr.cx;
    const double v_c = v_und - intr.cy;
    const double s   = (intr.fy > 0.0) ? (intr.fx / intr.fy) : 1.0;

    Point2D m;
    m.SetPoint2D(static_cast<float>(u_c), static_cast<float>(s * v_c));
    measurements.push_back(m);
    cam_idx_list.push_back(obs.image_index);
    pt_idx_list.push_back(obs.point_index);
  }

  if (measurements.empty()) return false;

  LOG(INFO) << "global_bundle_pba: ncam=" << n_images << "  npts=" << n_pts
            << "  nobs=" << measurements.size() << "  n_distinct=" << n_distinct
            << "  max_iter=" << max_iterations;

  // ── Run PBA (with one numeric-stability retry) ─────────────────────────────
  const std::vector<CameraT> camera_init = camera_data;
  const std::vector<Point3D> point_init  = point_data;

  int   n_successful   = 0;
  float final_mse      = std::numeric_limits<float>::infinity();
  float initial_mse    = std::numeric_limits<float>::infinity();
  int   return_code    = 'E';
  char  return_code_ch = 'E';

  for (int attempt = 0; attempt < 2; ++attempt) {
    camera_data = camera_init;
    point_data  = point_init;

    // Use CPU double backend for better numerical stability on large aerial scenes.
    // (Camera input is float in DataInterface, but solver internals run in double.)
    ParallelBA pba(ParallelBA::PBA_CPU_DOUBLE);
    ConfigBA* cfg = pba.GetInternalConfig();
    cfg->__lm_max_iteration      = max_iterations;
    cfg->__verbose_level         = 1;
    cfg->__lm_damping_auto_switch = 2.0f;  // same robust setting as upstream interface.h
    if (attempt == 1) {
      // Retry path: stronger initial damping to avoid CG numeric breakdown.
      cfg->__lm_initial_damp = 1e-2f;
    }
    pba.SetFixedIntrinsics(true);
    pba.EnableRadialDistortion(ParallelBA::PBA_NO_DISTORTION);

    pba.SetCameraData(camera_data.size(), camera_data.data());
    pba.SetPointData(point_data.size(), point_data.data());
    pba.SetProjection(measurements.size(), measurements.data(),
                      pt_idx_list.data(), cam_idx_list.data());

    n_successful   = pba.RunBundleAdjustment();
    final_mse      = pba.GetMeanSquaredError();
    initial_mse    = cfg->GetInitialMSE();
    return_code    = cfg->GetBundleReturnCode();
    return_code_ch = static_cast<char>(return_code);

    if (return_code_ch != 'E') break;
    if (attempt == 0) {
      LOG(WARNING) << "global_bundle_pba: numeric error code 'E' on attempt 1, "
                   << "retrying with stronger damping";
    }
  }

  LOG(INFO) << "global_bundle_pba: " << n_successful
            << " successful LM iters"
            << "  return_code=" << return_code_ch
            << "  initial_MSE=" << initial_mse
            << "  final_MSE=" << final_mse
            << "  final_RMSE=" << std::sqrt(static_cast<double>(final_mse))
            << " px";

  // Guard: PBA uses float internally; a diverged optimisation can produce NaN
  // or Inf in camera/point data, which would later cause CHECK failures in
  // triangulation (sqrt(NaN) = -nan).  Reject the result outright if any
  // value is non-finite.
  if (!std::isfinite(final_mse) || final_mse < 0.0f) {
    LOG(WARNING) << "global_bundle_pba: final_mse=" << final_mse << " is non-finite; rejecting";
    return false;
  }
  // Important: n_successful can be 0 at convergence (no accepted LM step).
  // In PBA, GetBundleReturnCode() is NOT StatusT; it is a char code:
  //   'S','G','M','T' => acceptable termination,  'E' => numeric error.
  if (return_code_ch == 'E') {
    LOG(WARNING) << "global_bundle_pba: backend numeric error code 'E'; rejecting";
    return false;
  }
  for (int i = 0; i < n_images; ++i) {
    const CameraT& cam = camera_data[i];
    float r9[9];
    const_cast<CameraT&>(cam).GetMatrixRotation(r9);
    for (int k = 0; k < 9; ++k) {
      if (!std::isfinite(r9[k])) {
        LOG(WARNING) << "global_bundle_pba: camera " << i << " R[" << k
                     << "]=" << r9[k] << " non-finite; rejecting";
        return false;
      }
    }
    if (!std::isfinite(cam.t[0]) || !std::isfinite(cam.t[1]) || !std::isfinite(cam.t[2])) {
      LOG(WARNING) << "global_bundle_pba: camera " << i << " t non-finite; rejecting";
      return false;
    }
    if (!std::isfinite(cam.f) || cam.f <= 0.0f) {
      LOG(WARNING) << "global_bundle_pba: camera " << i << " f=" << cam.f << " invalid; rejecting";
      return false;
    }
  }
  for (int i = 0; i < n_pts; ++i) {
    float xyz[3];
    point_data[i].GetPoint(xyz);
    if (!std::isfinite(xyz[0]) || !std::isfinite(xyz[1]) || !std::isfinite(xyz[2])) {
      LOG(WARNING) << "global_bundle_pba: point " << i << " non-finite; rejecting";
      return false;
    }
  }

  // ── Write back results ─────────────────────────────────────────────────────
  result->poses_R.resize(n_images);
  result->poses_C.resize(n_images);
  result->points3d.resize(n_pts);
  result->cameras = input.cameras;  // intrinsics unchanged (fixed)

  for (int i = 0; i < n_images; ++i) {
    CameraT& cam = camera_data[i];

    // R (row-major → Eigen), then re-orthogonalise via SVD.
    // PBA uses float; accumulated rounding errors can make det(R) drift from ±1,
    // causing near-zero Z values in projection and ultimately NaN reprojections.
    float r9[9];
    cam.GetMatrixRotation(r9);
    Eigen::Matrix3d R_raw;
    for (int row = 0; row < 3; ++row)
      for (int col = 0; col < 3; ++col)
        R_raw(row, col) = static_cast<double>(r9[row * 3 + col]);
    {
      Eigen::JacobiSVD<Eigen::Matrix3d> svd(R_raw,
                                            Eigen::ComputeFullU | Eigen::ComputeFullV);
      // Enforce det = +1 (proper rotation, not reflection)
      Eigen::Matrix3d diag = Eigen::Matrix3d::Identity();
      diag(2, 2) = (svd.matrixU() * svd.matrixV().transpose()).determinant();
      result->poses_R[i] = svd.matrixU() * diag * svd.matrixV().transpose();
    }

    // C = −R^T · T
    float c3[3];
    cam.GetCameraCenter(c3);
    result->poses_C[i] = Eigen::Vector3d(
        static_cast<double>(c3[0]),
        static_cast<double>(c3[1]),
        static_cast<double>(c3[2]));
  }

  for (int i = 0; i < n_pts; ++i) {
    float xyz[3];
    point_data[i].GetPoint(xyz);
    result->points3d[i] = Eigen::Vector3d(static_cast<double>(xyz[0]),
                                          static_cast<double>(xyz[1]),
                                          static_cast<double>(xyz[2]));
  }

  result->rmse_px       = std::sqrt(static_cast<double>(final_mse));
  result->num_residuals = static_cast<int>(measurements.size()) * 2;
  result->success       = true;
  return true;
}

}  // namespace sfm
}  // namespace insight
