/**
 * @file  resection.cpp
 * @brief Resection via GPU PnP RANSAC + lightweight Gauss-Newton pose refinement (6-DOF).
 */

#include "resection.h"

#include "../camera/camera_utils.h"
#include "../geometry/gpu_geo_ransac.h"

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <glog/logging.h>
#include <vector>

namespace insight {
namespace sfm {

namespace {

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

  double prev_cost = 1e18;
  for (int iter = 0; iter < max_iterations; ++iter) {
    Eigen::Matrix<double, 6, 6> JtJ = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> Jtr = Eigen::Matrix<double, 6, 1>::Zero();
    double cost = 0.0;

    for (int i = 0; i < n; ++i) {
      const Eigen::Vector3d p = R * pts3d[i] + t;  // camera-space
      if (p(2) < 1e-6)
        continue;
      const double inv_z = 1.0 / p(2);
      const double inv_z2 = inv_z * inv_z;
      const double u_proj = fx * p(0) * inv_z + cx;
      const double v_proj = fy * p(1) * inv_z + cy;
      const double ru = u_proj - pts2d[i](0);
      const double rv = v_proj - pts2d[i](1);
      cost += ru * ru + rv * rv;

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
      skew_p <<      0.0, -p(2),  p(1),
                  p(2),     0.0, -p(0),
                 -p(1),  p(0),     0.0;
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

    // LM damping
    for (int d = 0; d < 6; ++d)
      JtJ(d, d) *= (1.0 + lambda);

    // Solve
    Eigen::Matrix<double, 6, 1> delta =
        JtJ.ldlt().solve(-Jtr);

    // Update pose: so3 perturbation for rotation, direct for translation
    Eigen::Vector3d omega = delta.head<3>();
    double angle = omega.norm();
    Eigen::Matrix3d R_new = R;
    if (angle > 1e-10) {
      Eigen::AngleAxisd aa(angle, omega / angle);
      R_new = aa.toRotationMatrix() * R;
    }
    Eigen::Vector3d t_new = t + delta.tail<3>();

    if (cost < prev_cost) {
      R = R_new;
      t = t_new;
      lambda *= 0.5;
      prev_cost = cost;
    } else {
      lambda *= 2.0;
    }
    if (lambda > 1e8 || delta.norm() < 1e-10)
      break;
  }

  *R_out = R;
  *t_out = t;
  if (rmse_px)
    *rmse_px = std::sqrt(prev_cost / std::max(n, 1));
  return true;
}

} // namespace

void resection_init_gpu() { ensure_gpu_geo_init(); }

bool is_resection_stable(int inlier_count, int total_correspondences, double rmse_px,
                         double min_inlier_ratio, double max_rmse_px) {
  if (total_correspondences <= 0)
    return false;
  double ratio = static_cast<double>(inlier_count) / static_cast<double>(total_correspondences);
  return ratio >= min_inlier_ratio && rmse_px <= max_rmse_px;
}

bool resection_single_image(const TrackStore& store, int image_index, double fx, double fy,
                            double cx, double cy, Eigen::Matrix3d* R_out, Eigen::Vector3d* t_out,
                            int min_inliers, double ransac_thresh_px, int* inliers_out) {
  if (!R_out || !t_out)
    return false;

  using Clock = std::chrono::steady_clock;
  auto t0 = Clock::now();

  std::vector<int> track_ids;
  std::vector<Observation> obs_list;
  const int n = store.get_image_track_observations(image_index, &track_ids, &obs_list);
  if (n < static_cast<int>(min_inliers))
    return false;

  std::vector<Point3D2D> pts_pnp;
  pts_pnp.reserve(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    const int tid = track_ids[static_cast<size_t>(i)];
    if (!store.track_has_triangulated_xyz(tid))
      continue;
    float x, y, z;
    store.get_track_xyz(tid, &x, &y, &z);
    Point3D2D pt;
    pt.x = x;
    pt.y = y;
    pt.z = z;
    pt.u = obs_list[static_cast<size_t>(i)].u;
    pt.v = obs_list[static_cast<size_t>(i)].v;
    pts_pnp.push_back(pt);
  }
  if (static_cast<int>(pts_pnp.size()) < min_inliers)
    return false;

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
    LOG(INFO) << "[PERF] resection_single_image im=" << image_index
              << "  pts=" << num_pts
              << "  data_prep=" << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() << "ms"
              << "  gpu_ransac=" << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << "ms"
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
  pose_refine_gn(inlier_pts3d, inlier_pts2d, fx, fy, cx, cy, R_eig, t_eig, &R_refined,
                 &t_refined, &rmse, 10);
  auto t3 = Clock::now(); // after GN refinement
  LOG(INFO) << "[PERF] resection_single_image im=" << image_index
            << "  pts=" << num_pts << "  inliers=" << n_inliers
            << "  data_prep=" << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() << "ms"
            << "  gpu_ransac=" << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << "ms"
            << "  gn_refine=" << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count() << "ms"
            << "  rmse=" << rmse;
  if (inliers_out)
    *inliers_out = n_inliers;
  *R_out = R_refined;
  *t_out = t_refined;
  return true;
}

bool resection_single_image(const camera::Intrinsics& K, const TrackStore& store, int image_index,
                            Eigen::Matrix3d* R_out, Eigen::Vector3d* t_out,
                            int min_inliers, double ransac_thresh_px,
                            int* inliers_out) {
  if (!K.has_distortion()) {
    return resection_single_image(store, image_index, K.fx, K.fy, K.cx, K.cy, R_out, t_out,
                                  min_inliers, ransac_thresh_px, inliers_out);
  }

  // Collect raw observations (distorted pixels) for this image.
  std::vector<int> track_ids;
  std::vector<Observation> obs_list;
  const int n = store.get_image_track_observations(image_index, &track_ids, &obs_list);
  if (n < static_cast<int>(min_inliers))
    return false;

  // Batch-undistort 2D observations → undistorted pixel coordinates.
  std::vector<float> u_raw(static_cast<size_t>(n)), v_raw(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    u_raw[static_cast<size_t>(i)] = obs_list[static_cast<size_t>(i)].u;
    v_raw[static_cast<size_t>(i)] = obs_list[static_cast<size_t>(i)].v;
  }
  std::vector<float> u_und(static_cast<size_t>(n)), v_und(static_cast<size_t>(n));
  camera::undistort_points(K, u_raw.data(), v_raw.data(), u_und.data(), v_und.data(), n);

  // Build 3D–2D correspondences using undistorted 2D points.
  std::vector<Point3D2D> pts_pnp;
  pts_pnp.reserve(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    const int tid = track_ids[static_cast<size_t>(i)];
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
    pts_pnp.push_back(pt);
  }
  if (static_cast<int>(pts_pnp.size()) < min_inliers)
    return false;

  const int num_pts = static_cast<int>(pts_pnp.size());
  const float fx = static_cast<float>(K.fx), fy = static_cast<float>(K.fy);
  const float cx = static_cast<float>(K.cx), cy = static_cast<float>(K.cy);
  float K_mat[9] = {fx, 0.f, cx, 0.f, fy, cy, 0.f, 0.f, 1.f};
  const float thresh_sq = static_cast<float>(ransac_thresh_px * ransac_thresh_px);
  float R_init[9], t_init[3];
  std::vector<unsigned char> inlier_mask(static_cast<size_t>(num_pts), 0);

  ensure_gpu_geo_init();
  gpu_geo_set_solver(1);
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
    inlier_pts3d.push_back(
        Eigen::Vector3d(static_cast<double>(pts_pnp[static_cast<size_t>(i)].x),
                        static_cast<double>(pts_pnp[static_cast<size_t>(i)].y),
                        static_cast<double>(pts_pnp[static_cast<size_t>(i)].z)));
    inlier_pts2d.push_back(
        Eigen::Vector2d(static_cast<double>(pts_pnp[static_cast<size_t>(i)].u),
                        static_cast<double>(pts_pnp[static_cast<size_t>(i)].v)));
  }

  Eigen::Matrix3d R_refined = R_eig;
  Eigen::Vector3d t_refined = t_eig;
  double rmse = 0.0;
  pose_refine_gn(inlier_pts3d, inlier_pts2d, K.fx, K.fy, K.cx, K.cy, R_eig, t_eig, &R_refined,
                 &t_refined, &rmse, 10);
  if (inliers_out)
    *inliers_out = n_inliers;
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
