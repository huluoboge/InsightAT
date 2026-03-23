/**
 * @file  resection.cpp
 * @brief Resection via GPU PnP RANSAC + lightweight Gauss-Newton pose refinement (6-DOF).
 */

#include "resection.h"

#include "../camera/camera_utils.h"
#include "../geometry/gpu_geo_ransac.h"

#include <PoseLib/robust.h>

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <glog/logging.h>
#include <limits>
#include <unordered_set>
#include <vector>

namespace insight {
namespace sfm {

namespace {

ResectionBackend g_resection_backend = ResectionBackend::kGpuRansac;

/// Ensure GPU RANSAC (EGL) is initialised once for resection PnP.
static void ensure_gpu_geo_init() {
  static bool inited = false;
  if (inited)
    return;
  if (gpu_geo_init(nullptr) == 0)
    inited = true;
}

void float_rt_to_eigen(const float R[9], const float t[3], Eigen::Matrix3d* R_out,
                       Eigen::Vector3d* t_out) {
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      (*R_out)(i, j) = static_cast<double>(R[i * 3 + j]);
  (*t_out)(0) = static_cast<double>(t[0]);
  (*t_out)(1) = static_cast<double>(t[1]);
  (*t_out)(2) = static_cast<double>(t[2]);
}

/// Lightweight pose-only refinement: Gauss-Newton, 6-DOF (so3 + t), fixed 3D points.
/// Replaces the previous Ceres-based BA call; ~10x faster for single-image refinement.
static bool pose_refine_gn(const std::vector<Eigen::Vector3d>& pts3d,
                           const std::vector<Eigen::Vector2d>& pts2d, double fx, double fy,
                           double cx, double cy, const Eigen::Matrix3d& R_in,
                           const Eigen::Vector3d& t_in, Eigen::Matrix3d* R_out,
                           Eigen::Vector3d* t_out, double* rmse_px, int max_iterations) {
  if (!R_out || !t_out || pts3d.size() != pts2d.size() || pts3d.empty())
    return false;

  Eigen::Matrix3d R = R_in;
  Eigen::Vector3d t = t_in;
  const int n = static_cast<int>(pts3d.size());
  const double lambda_init = 1e-3; // LM damping start
  double lambda = lambda_init;

  auto evaluate_cost = [&](const Eigen::Matrix3d& R_eval, const Eigen::Vector3d& t_eval,
                           int* valid_count_out) -> double {
    double cost = 0.0;
    int valid_count = 0;
    for (int i = 0; i < n; ++i) {
      const Eigen::Vector3d p = R_eval * pts3d[i] + t_eval;
      if (p(2) < 1e-6)
        continue;
      const double inv_z = 1.0 / p(2);
      const double u_proj = fx * p(0) * inv_z + cx;
      const double v_proj = fy * p(1) * inv_z + cy;
      const double ru = u_proj - pts2d[i](0);
      const double rv = v_proj - pts2d[i](1);
      cost += ru * ru + rv * rv;
      ++valid_count;
    }
    if (valid_count_out)
      *valid_count_out = valid_count;
    return cost;
  };

  int valid_count = 0;
  double prev_cost = evaluate_cost(R, t, &valid_count);
  for (int iter = 0; iter < max_iterations; ++iter) {
    Eigen::Matrix<double, 6, 6> JtJ = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> Jtr = Eigen::Matrix<double, 6, 1>::Zero();
    double cost = 0.0;
    int used_obs = 0;

    for (int i = 0; i < n; ++i) {
      const Eigen::Vector3d p = R * pts3d[i] + t; // camera-space
      if (p(2) < 1e-6)
        continue;
      const double inv_z = 1.0 / p(2);
      const double inv_z2 = inv_z * inv_z;
      const double u_proj = fx * p(0) * inv_z + cx;
      const double v_proj = fy * p(1) * inv_z + cy;
      const double ru = u_proj - pts2d[i](0);
      const double rv = v_proj - pts2d[i](1);
      cost += ru * ru + rv * rv;
      ++used_obs;

      // Jacobian of (u_proj, v_proj) w.r.t. [omega (so3), t]
      // du/dp = [fx/z, 0, -fx*px/z^2]
      // dv/dp = [0, fy/z, -fy*py/z^2]
      // dp/domega = -[p]_x (skew), dp/dt = I
      Eigen::Matrix<double, 2, 3> Jproj;
      Jproj(0, 0) = fx * inv_z;
      Jproj(0, 1) = 0.0;
      Jproj(0, 2) = -fx * p(0) * inv_z2;
      Jproj(1, 0) = 0.0;
      Jproj(1, 1) = fy * inv_z;
      Jproj(1, 2) = -fy * p(1) * inv_z2;

      // dp/domega = -[p]_x
      Eigen::Matrix3d skew_p;
      skew_p << 0.0, -p(2), p(1), p(2), 0.0, -p(0), -p(1), p(0), 0.0;
      // Jw = Jproj * (-skew_p)
      Eigen::Matrix<double, 2, 3> Jw = Jproj * (-skew_p);
      // Jt = Jproj
      // Full J row: [Jw | Jproj]  (6 cols)
      Eigen::Matrix<double, 2, 6> J;
      J.leftCols<3>() = Jw;
      J.rightCols<3>() = Jproj;

      Eigen::Vector2d r(ru, rv);
      JtJ += J.transpose() * J;
      Jtr += J.transpose() * r;
    }

    if (used_obs <= 0)
      break;

    // LM damping
    for (int d = 0; d < 6; ++d)
      JtJ(d, d) *= (1.0 + lambda);

    // Solve
    Eigen::Matrix<double, 6, 1> delta = JtJ.ldlt().solve(-Jtr);

    // Update pose: so3 perturbation for rotation, direct for translation
    Eigen::Vector3d omega = delta.head<3>();
    double angle = omega.norm();
    Eigen::Matrix3d R_new = R;
    if (angle > 1e-10) {
      Eigen::AngleAxisd aa(angle, omega / angle);
      R_new = aa.toRotationMatrix() * R;
    }
    Eigen::Vector3d t_new = t + delta.tail<3>();

    int new_valid_count = 0;
    const double new_cost = evaluate_cost(R_new, t_new, &new_valid_count);

    if (new_valid_count > 0 && new_cost < prev_cost) {
      R = R_new;
      t = t_new;
      lambda *= 0.5;
      prev_cost = new_cost;
      valid_count = new_valid_count;
    } else {
      lambda *= 2.0;
    }
    if (lambda > 1e8 || delta.norm() < 1e-10)
      break;
  }

  *R_out = R;
  *t_out = t;
  if (rmse_px)
    *rmse_px = std::sqrt(prev_cost / std::max(valid_count, 1));
  return true;
}

static double compute_pose_rmse_pinhole(const std::vector<Eigen::Vector3d>& pts3d,
                                        const std::vector<Eigen::Vector2d>& pts2d, double fx,
                                        double fy, double cx, double cy, const Eigen::Matrix3d& R,
                                        const Eigen::Vector3d& t) {
  if (pts3d.empty() || pts3d.size() != pts2d.size())
    return std::numeric_limits<double>::infinity();

  double cost = 0.0;
  int valid_count = 0;
  for (size_t i = 0; i < pts3d.size(); ++i) {
    const Eigen::Vector3d p = R * pts3d[i] + t;
    if (p(2) < 1e-6)
      continue;
    const double inv_z = 1.0 / p(2);
    const double u_proj = fx * p(0) * inv_z + cx;
    const double v_proj = fy * p(1) * inv_z + cy;
    const double du = u_proj - pts2d[i](0);
    const double dv = v_proj - pts2d[i](1);
    cost += du * du + dv * dv;
    ++valid_count;
  }
  if (valid_count <= 0)
    return std::numeric_limits<double>::infinity();
  return std::sqrt(cost / static_cast<double>(valid_count));
}

static void mark_pnp_outliers_deleted(TrackStore& store, const std::vector<int>& pnp_obs_ids,
                                      const std::vector<char>& inlier_mask) {
  if (pnp_obs_ids.size() != inlier_mask.size())
    return;
  for (size_t i = 0; i < pnp_obs_ids.size(); ++i) {
    if (!inlier_mask[i])
      store.mark_observation_deleted(pnp_obs_ids[static_cast<size_t>(i)]);
  }
}

static bool resection_poselib_pinhole(const std::vector<Eigen::Vector3d>& pts3d,
                                      const std::vector<Eigen::Vector2d>& pts2d, double fx,
                                      double fy, double cx, double cy, int min_inliers,
                                      double ransac_thresh_px, Eigen::Matrix3d* R_out,
                                      Eigen::Vector3d* t_out, int* inliers_out, double* rmse_px_out,
                                      std::vector<char>* inlier_mask_full_out) {
  if (!R_out || !t_out || pts3d.size() != pts2d.size() || pts3d.empty())
    return false;

  using Clock = std::chrono::steady_clock;
  auto t0 = Clock::now();

  std::vector<poselib::Point2D> points2d;
  std::vector<poselib::Point3D> points3d;
  points2d.reserve(pts2d.size());
  points3d.reserve(pts3d.size());
  for (size_t i = 0; i < pts2d.size(); ++i) {
    points2d.push_back(pts2d[i]);
    points3d.push_back(pts3d[i]);
  }

  poselib::AbsolutePoseOptions opt;
  opt.max_error = ransac_thresh_px;
  opt.ransac.max_iterations = 10000;
  opt.ransac.min_iterations = 1000;
  opt.bundle.max_iterations = 25;
  opt.bundle.loss_type = poselib::BundleOptions::CAUCHY;
  opt.bundle.loss_scale = std::max(1.0, ransac_thresh_px);

  poselib::Image image;
  image.camera = poselib::Camera(poselib::CameraModelId::PINHOLE, {fx, fy, cx, cy});
  std::vector<char> inlier_mask;
  const poselib::RansacStats stats =
      poselib::estimate_absolute_pose(points2d, points3d, opt, &image, &inlier_mask);

  auto t1 = Clock::now();
  const int n_inliers = static_cast<int>(stats.num_inliers);
  if (inliers_out)
    *inliers_out = n_inliers;
  if (n_inliers < min_inliers) {
    VLOG(1) << "[PERF] resection_single_image(poselib)"
            << "  pts=" << pts3d.size() << "  poselib="
            << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() << "ms"
            << "  FAILED inliers=" << n_inliers;
    return false;
  }

  Eigen::Matrix3d R = image.pose.R();
  Eigen::Vector3d t = image.pose.t;

  std::vector<Eigen::Vector3d> inlier_pts3d;
  std::vector<Eigen::Vector2d> inlier_pts2d;
  inlier_pts3d.reserve(static_cast<size_t>(n_inliers));
  inlier_pts2d.reserve(static_cast<size_t>(n_inliers));
  for (size_t i = 0; i < inlier_mask.size() && i < pts3d.size(); ++i) {
    if (!inlier_mask[i])
      continue;
    inlier_pts3d.push_back(pts3d[i]);
    inlier_pts2d.push_back(pts2d[i]);
  }

  Eigen::Matrix3d R_refined = R;
  Eigen::Vector3d t_refined = t;
  double gn_rmse = 0.0;
  if (!inlier_pts3d.empty()) {
    pose_refine_gn(inlier_pts3d, inlier_pts2d, fx, fy, cx, cy, R, t, &R_refined, &t_refined,
                   &gn_rmse, 10);
  }
  auto t2 = Clock::now();

  const double rmse =
      compute_pose_rmse_pinhole(inlier_pts3d, inlier_pts2d, fx, fy, cx, cy, R_refined, t_refined);
  VLOG(1) << "[PERF] resection_single_image(poselib)"
          << "  pts=" << pts3d.size() << "  inliers=" << n_inliers
          << "  poselib=" << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
          << "ms"
          << "  gn_refine="
          << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << "ms"
          << "  rmse=" << rmse;
  if (rmse_px_out)
    *rmse_px_out = rmse;
  if (!is_resection_stable(n_inliers, static_cast<int>(pts3d.size()), rmse,
                           /*min_inlier_ratio=*/0.02,
                           /*max_rmse_px=*/std::max(6.0, 1.5 * ransac_thresh_px))) {
    VLOG(1) << "[PERF] resection_single_image(poselib)"
            << "  rejected_by_stability: inlier_ratio="
            << (pts3d.empty() ? 0.0
                              : static_cast<double>(n_inliers) / static_cast<double>(pts3d.size()))
            << "  rmse=" << rmse;
    return false;
  }

  if (inlier_mask_full_out)
    *inlier_mask_full_out = inlier_mask;

  *R_out = R_refined;
  *t_out = t_refined;
  return true;
}

} // namespace

int prune_resection_observations_reprojection(TrackStore* store, int image_index,
                                              const Eigen::Matrix3d& R, const Eigen::Vector3d& C,
                                              const camera::Intrinsics& K, double threshold_px) {
  if (!store || threshold_px <= 0.0)
    return 0;
  const int n_images = store->num_images();
  if (image_index < 0 || image_index >= n_images)
    return 0;

  const double thresh_sq = threshold_px * threshold_px;
  int marked = 0;
  std::unordered_set<int> dirty_tracks;
  std::vector<int> obs_ids;
  store->get_image_observation_indices(image_index, &obs_ids);

  for (int obs_id : obs_ids) {
    if (!store->is_obs_valid(obs_id))
      continue;
    const int tid = store->obs_track_id(obs_id);
    if (!store->is_track_valid(tid) || !store->track_has_triangulated_xyz(tid))
      continue;

    float tx, ty, tz;
    store->get_track_xyz(tid, &tx, &ty, &tz);
    const Eigen::Vector3d X(static_cast<double>(tx), static_cast<double>(ty),
                            static_cast<double>(tz));
    const Eigen::Vector3d p = R * (X - C);
    if (p(2) <= 1e-12) {
      store->mark_observation_deleted(obs_id);
      dirty_tracks.insert(tid);
      ++marked;
      continue;
    }

    const double xn = p(0) / p(2), yn = p(1) / p(2);
    double xd, yd;
    camera::apply_distortion(xn, yn, K, &xd, &yd);
    const double u_pred = K.fx * xd + K.cx;
    const double v_pred = K.fy * yd + K.cy;

    Observation obs;
    store->get_obs(obs_id, &obs);
    const double du = static_cast<double>(obs.u) - u_pred;
    const double dv = static_cast<double>(obs.v) - v_pred;
    if (du * du + dv * dv > thresh_sq) {
      store->mark_observation_deleted(obs_id);
      dirty_tracks.insert(tid);
      ++marked;
    }
  }

  std::vector<int> track_obs_ids;
  for (int tid : dirty_tracks) {
    if (!store->is_track_valid(tid) || !store->track_has_triangulated_xyz(tid))
      continue;
    track_obs_ids.clear();
    store->get_track_obs_ids(tid, &track_obs_ids);
    if (track_obs_ids.size() < 2)
      store->clear_track_xyz(tid);
  }
  return marked;
}

void resection_init_gpu() {
  if (resection_backend_uses_gpu(g_resection_backend))
    ensure_gpu_geo_init();
}

void set_resection_backend(ResectionBackend backend) { g_resection_backend = backend; }

ResectionBackend get_resection_backend() { return g_resection_backend; }

const char* resection_backend_name(ResectionBackend backend) {
  switch (backend) {
  case ResectionBackend::kGpuRansac:
    return "gpu";
  case ResectionBackend::kPoseLib:
    return "poselib";
  }
  return "unknown";
}

bool resection_backend_uses_gpu(ResectionBackend backend) {
  return backend == ResectionBackend::kGpuRansac;
}

bool is_resection_stable(int inlier_count, int total_correspondences, double rmse_px,
                         double min_inlier_ratio, double max_rmse_px) {
  if (total_correspondences <= 0)
    return false;
  double ratio = static_cast<double>(inlier_count) / static_cast<double>(total_correspondences);
  return ratio >= min_inlier_ratio && rmse_px <= max_rmse_px;
}

bool resection_single_image(TrackStore& store, int image_index, double fx, double fy, double cx,
                            double cy, Eigen::Matrix3d* R_out, Eigen::Vector3d* t_out,
                            int min_inliers, double ransac_thresh_px, int* inliers_out,
                            double* rmse_px_out) {
  if (!R_out || !t_out)
    return false;

  using Clock = std::chrono::steady_clock;
  auto t0 = Clock::now();

  std::vector<int> obs_ids_all;
  const int n_obs = store.get_image_observation_indices(image_index, &obs_ids_all);
  if (n_obs < static_cast<int>(min_inliers))
    return false;

  std::vector<int> pnp_obs_ids;
  std::vector<Point3D2D> pts_pnp;
  pnp_obs_ids.reserve(static_cast<size_t>(n_obs));
  pts_pnp.reserve(static_cast<size_t>(n_obs));
  for (int obs_id : obs_ids_all) {
    const int tid = store.obs_track_id(obs_id);
    if (!store.track_has_triangulated_xyz(tid))
      continue;
    Observation o;
    store.get_obs(obs_id, &o);
    float x, y, z;
    store.get_track_xyz(tid, &x, &y, &z);
    Point3D2D pt;
    pt.x = x;
    pt.y = y;
    pt.z = z;
    pt.u = o.u;
    pt.v = o.v;
    pnp_obs_ids.push_back(obs_id);
    pts_pnp.push_back(pt);
  }
  if (static_cast<int>(pts_pnp.size()) < min_inliers)
    return false;

  if (g_resection_backend == ResectionBackend::kPoseLib) {
    std::vector<Eigen::Vector3d> pts3d;
    std::vector<Eigen::Vector2d> pts2d;
    pts3d.reserve(pts_pnp.size());
    pts2d.reserve(pts_pnp.size());
    for (const Point3D2D& pt : pts_pnp) {
      pts3d.emplace_back(static_cast<double>(pt.x), static_cast<double>(pt.y),
                         static_cast<double>(pt.z));
      pts2d.emplace_back(static_cast<double>(pt.u), static_cast<double>(pt.v));
    }
    std::vector<char> inlier_full;
    if (!resection_poselib_pinhole(pts3d, pts2d, fx, fy, cx, cy, min_inliers, ransac_thresh_px,
                                   R_out, t_out, inliers_out, rmse_px_out, &inlier_full))
      return false;
    mark_pnp_outliers_deleted(store, pnp_obs_ids, inlier_full);
    return true;
  }

  auto t1 = Clock::now(); // after data prep
  const int num_pts = static_cast<int>(pts_pnp.size());
  float K[9] = {static_cast<float>(fx),
                0.f,
                static_cast<float>(cx),
                0.f,
                static_cast<float>(fy),
                static_cast<float>(cy),
                0.f,
                0.f,
                1.f};
  const float thresh_sq = static_cast<float>(ransac_thresh_px * ransac_thresh_px);
  float R_init[9], t_init[3];
  std::vector<unsigned char> inlier_mask(static_cast<size_t>(num_pts), 0);

  ensure_gpu_geo_init();
  const int n_inliers =
      gpu_ransac_pnp(pts_pnp.data(), num_pts, K, R_init, t_init, thresh_sq, inlier_mask.data());

  auto t2 = Clock::now(); // after gpu_ransac_pnp
  if (n_inliers < min_inliers) {
    if (inliers_out)
      *inliers_out = n_inliers >= 0 ? n_inliers : 0;
    VLOG(1) << "[PERF] resection_single_image im=" << image_index << "  pts=" << num_pts
            << "  data_prep="
            << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() << "ms"
            << "  gpu_ransac="
            << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << "ms"
            << "  FAILED inliers=" << n_inliers;
    return false;
  }

  Eigen::Matrix3d R_eig;
  Eigen::Vector3d t_eig;
  float_rt_to_eigen(R_init, t_init, &R_eig, &t_eig);

  std::vector<Eigen::Vector3d> inlier_pts3d;
  std::vector<Eigen::Vector2d> inlier_pts2d;
  inlier_pts3d.reserve(static_cast<size_t>(n_inliers));
  inlier_pts2d.reserve(static_cast<size_t>(n_inliers));
  for (int i = 0; i < num_pts; ++i) {
    if (!inlier_mask[static_cast<size_t>(i)])
      continue;
    inlier_pts3d.push_back(Eigen::Vector3d(static_cast<double>(pts_pnp[static_cast<size_t>(i)].x),
                                           static_cast<double>(pts_pnp[static_cast<size_t>(i)].y),
                                           static_cast<double>(pts_pnp[static_cast<size_t>(i)].z)));
    inlier_pts2d.push_back(Eigen::Vector2d(static_cast<double>(pts_pnp[static_cast<size_t>(i)].u),
                                           static_cast<double>(pts_pnp[static_cast<size_t>(i)].v)));
  }

  Eigen::Matrix3d R_refined = R_eig;
  Eigen::Vector3d t_refined = t_eig;
  double rmse = 0.0;
  pose_refine_gn(inlier_pts3d, inlier_pts2d, fx, fy, cx, cy, R_eig, t_eig, &R_refined, &t_refined,
                 &rmse, 10);
  auto t3 = Clock::now(); // after GN refinement
  VLOG(1) << "[PERF] resection_single_image im=" << image_index << "  pts=" << num_pts
          << "  inliers=" << n_inliers << "  data_prep="
          << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() << "ms"
          << "  gpu_ransac="
          << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << "ms"
          << "  gn_refine="
          << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count() << "ms"
          << "  rmse=" << rmse;
  if (rmse_px_out)
    *rmse_px_out = rmse;
  if (!is_resection_stable(n_inliers, num_pts, rmse,
                           /*min_inlier_ratio=*/0.02,
                           /*max_rmse_px=*/std::max(6.0, 1.5 * ransac_thresh_px))) {
    VLOG(1) << "[PERF] resection_single_image im=" << image_index
            << "  rejected_by_stability: inlier_ratio="
            << (num_pts > 0 ? static_cast<double>(n_inliers) / static_cast<double>(num_pts) : 0.0)
            << "  rmse=" << rmse;
    return false;
  }
  if (inliers_out)
    *inliers_out = n_inliers;
  {
    std::vector<char> mask_char(static_cast<size_t>(num_pts));
    for (int i = 0; i < num_pts; ++i)
      mask_char[static_cast<size_t>(i)] = inlier_mask[static_cast<size_t>(i)] ? 1 : 0;
    mark_pnp_outliers_deleted(store, pnp_obs_ids, mask_char);
  }
  *R_out = R_refined;
  *t_out = t_refined;
  return true;
}

bool resection_single_image(const camera::Intrinsics& K, TrackStore& store, int image_index,
                            Eigen::Matrix3d* R_out, Eigen::Vector3d* t_out, int min_inliers,
                            double ransac_thresh_px, int* inliers_out, double* rmse_px_out) {
  if (!K.has_distortion()) {
    return resection_single_image(store, image_index, K.fx, K.fy, K.cx, K.cy, R_out, t_out,
                                  min_inliers, ransac_thresh_px, inliers_out, rmse_px_out);
  }

  std::vector<int> obs_ids_all;
  const int n = store.get_image_observation_indices(image_index, &obs_ids_all);
  if (n < static_cast<int>(min_inliers))
    return false;

  std::vector<float> u_raw(static_cast<size_t>(n)), v_raw(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    Observation o;
    store.get_obs(obs_ids_all[static_cast<size_t>(i)], &o);
    u_raw[static_cast<size_t>(i)] = o.u;
    v_raw[static_cast<size_t>(i)] = o.v;
  }
  std::vector<float> u_und(static_cast<size_t>(n)), v_und(static_cast<size_t>(n));
  camera::undistort_points(K, u_raw.data(), v_raw.data(), u_und.data(), v_und.data(), n);

  std::vector<int> pnp_obs_ids;
  std::vector<Point3D2D> pts_pnp;
  pnp_obs_ids.reserve(static_cast<size_t>(n));
  pts_pnp.reserve(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    const int tid = store.obs_track_id(obs_ids_all[static_cast<size_t>(i)]);
    if (!store.track_has_triangulated_xyz(tid))
      continue;
    float x, y, z;
    store.get_track_xyz(tid, &x, &y, &z);
    Point3D2D pt;
    pt.x = x;
    pt.y = y;
    pt.z = z;
    pt.u = u_und[static_cast<size_t>(i)];
    pt.v = v_und[static_cast<size_t>(i)];
    pnp_obs_ids.push_back(obs_ids_all[static_cast<size_t>(i)]);
    pts_pnp.push_back(pt);
  }
  if (static_cast<int>(pts_pnp.size()) < min_inliers)
    return false;

  if (g_resection_backend == ResectionBackend::kPoseLib) {
    std::vector<Eigen::Vector3d> pts3d;
    std::vector<Eigen::Vector2d> pts2d;
    pts3d.reserve(pts_pnp.size());
    pts2d.reserve(pts_pnp.size());
    for (const Point3D2D& pt : pts_pnp) {
      pts3d.emplace_back(static_cast<double>(pt.x), static_cast<double>(pt.y),
                         static_cast<double>(pt.z));
      pts2d.emplace_back(static_cast<double>(pt.u), static_cast<double>(pt.v));
    }
    std::vector<char> inlier_full;
    if (!resection_poselib_pinhole(pts3d, pts2d, K.fx, K.fy, K.cx, K.cy, min_inliers,
                                   ransac_thresh_px, R_out, t_out, inliers_out, rmse_px_out,
                                   &inlier_full))
      return false;
    mark_pnp_outliers_deleted(store, pnp_obs_ids, inlier_full);
    return true;
  }

  const int num_pts = static_cast<int>(pts_pnp.size());
  const float fx = static_cast<float>(K.fx), fy = static_cast<float>(K.fy);
  const float cx = static_cast<float>(K.cx), cy = static_cast<float>(K.cy);
  float K_mat[9] = {fx, 0.f, cx, 0.f, fy, cy, 0.f, 0.f, 1.f};
  const float thresh_sq = static_cast<float>(ransac_thresh_px * ransac_thresh_px);
  float R_init[9], t_init[3];
  std::vector<unsigned char> inlier_mask(static_cast<size_t>(num_pts), 0);

  ensure_gpu_geo_init();

  const int n_inliers =
      gpu_ransac_pnp(pts_pnp.data(), num_pts, K_mat, R_init, t_init, thresh_sq, inlier_mask.data());
  if (n_inliers < min_inliers) {
    if (inliers_out)
      *inliers_out = n_inliers >= 0 ? n_inliers : 0;
    return false;
  }

  Eigen::Matrix3d R_eig;
  Eigen::Vector3d t_eig;
  float_rt_to_eigen(R_init, t_init, &R_eig, &t_eig);

  std::vector<Eigen::Vector3d> inlier_pts3d;
  std::vector<Eigen::Vector2d> inlier_pts2d;
  inlier_pts3d.reserve(static_cast<size_t>(n_inliers));
  inlier_pts2d.reserve(static_cast<size_t>(n_inliers));
  for (int i = 0; i < num_pts; ++i) {
    if (!inlier_mask[static_cast<size_t>(i)])
      continue;
    inlier_pts3d.push_back(Eigen::Vector3d(static_cast<double>(pts_pnp[static_cast<size_t>(i)].x),
                                           static_cast<double>(pts_pnp[static_cast<size_t>(i)].y),
                                           static_cast<double>(pts_pnp[static_cast<size_t>(i)].z)));
    inlier_pts2d.push_back(Eigen::Vector2d(static_cast<double>(pts_pnp[static_cast<size_t>(i)].u),
                                           static_cast<double>(pts_pnp[static_cast<size_t>(i)].v)));
  }

  Eigen::Matrix3d R_refined = R_eig;
  Eigen::Vector3d t_refined = t_eig;
  double rmse = 0.0;
  pose_refine_gn(inlier_pts3d, inlier_pts2d, K.fx, K.fy, K.cx, K.cy, R_eig, t_eig, &R_refined,
                 &t_refined, &rmse, 10);
  if (rmse_px_out)
    *rmse_px_out = rmse;
  if (!is_resection_stable(n_inliers, num_pts, rmse,
                           /*min_inlier_ratio=*/0.02,
                           /*max_rmse_px=*/std::max(6.0, 1.5 * ransac_thresh_px)))
    return false;
  if (inliers_out)
    *inliers_out = n_inliers;
  {
    std::vector<char> mask_char(static_cast<size_t>(num_pts));
    for (int i = 0; i < num_pts; ++i)
      mask_char[static_cast<size_t>(i)] = inlier_mask[static_cast<size_t>(i)] ? 1 : 0;
    mark_pnp_outliers_deleted(store, pnp_obs_ids, mask_char);
  }
  *R_out = R_refined;
  *t_out = t_refined;
  return true;
}

int resection_image_grid_coverage(const TrackStore& store, int image_index, int grid_cols,
                                  int grid_rows) {
  std::vector<int> track_ids;
  std::vector<Observation> obs_list;
  const int n = store.get_image_track_observations(image_index, &track_ids, &obs_list);
  if (n == 0 || grid_cols <= 0 || grid_rows <= 0)
    return 0;
  std::vector<float> uu, vv;
  for (size_t i = 0; i < static_cast<size_t>(n); ++i) {
    if (!store.track_has_triangulated_xyz(track_ids[i]))
      continue;
    uu.push_back(obs_list[i].u);
    vv.push_back(obs_list[i].v);
  }
  if (uu.size() < 2u)
    return uu.empty() ? 0 : 1;
  float umin = uu[0], umax = uu[0], vmin = vv[0], vmax = vv[0];
  for (size_t i = 1; i < uu.size(); ++i) {
    if (uu[i] < umin)
      umin = uu[i];
    if (uu[i] > umax)
      umax = uu[i];
    if (vv[i] < vmin)
      vmin = vv[i];
    if (vv[i] > vmax)
      vmax = vv[i];
  }
  float ur = umax - umin, vr = vmax - vmin;
  if (ur < 1e-6f)
    ur = 1e-6f;
  if (vr < 1e-6f)
    vr = 1e-6f;
  std::vector<std::vector<bool>> cell_used(
      static_cast<size_t>(grid_rows), std::vector<bool>(static_cast<size_t>(grid_cols), false));
  for (size_t i = 0; i < uu.size(); ++i) {
    int c = static_cast<int>((uu[i] - umin) / ur * grid_cols);
    int r = static_cast<int>((vv[i] - vmin) / vr * grid_rows);
    if (c >= grid_cols)
      c = grid_cols - 1;
    if (r >= grid_rows)
      r = grid_rows - 1;
    if (c < 0)
      c = 0;
    if (r < 0)
      r = 0;
    cell_used[static_cast<size_t>(r)][static_cast<size_t>(c)] = true;
  }
  int cells = 0;
  for (int r = 0; r < grid_rows; ++r)
    for (int c = 0; c < grid_cols; ++c)
      if (cell_used[static_cast<size_t>(r)][static_cast<size_t>(c)])
        ++cells;
  return cells;
}

int choose_next_resection_image(const TrackStore& store,
                                const std::vector<bool>& registered_indices,
                                const std::vector<bool>* skip_indices, int min_grid_cells) {
  const int n_images = store.num_images();
  if (n_images == 0 || static_cast<int>(registered_indices.size()) != n_images)
    return -1;
  if (skip_indices && static_cast<int>(skip_indices->size()) != n_images)
    return -1;

  int best_image = -1;
  int best_count = 0;
  for (int im = 0; im < n_images; ++im) {
    if (registered_indices[static_cast<size_t>(im)])
      continue;
    if (skip_indices && (*skip_indices)[static_cast<size_t>(im)])
      continue;
    if (min_grid_cells > 0 && resection_image_grid_coverage(store, im, 4, 4) < min_grid_cells)
      continue;
    std::vector<int> track_ids;
    store.get_image_track_observations(im, &track_ids, nullptr);
    int count = 0;
    for (int tid : track_ids) {
      if (store.track_has_triangulated_xyz(tid))
        ++count;
    }
    if (count > best_count) {
      best_count = count;
      best_image = im;
    }
  }
  return best_image;
}

} // namespace sfm
} // namespace insight
