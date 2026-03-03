/**
 * @file  resection.cpp
 * @brief Resection via GPU PnP RANSAC (gpu_ransac_pnp) + pose_only_bundle.
 */

#include "resection.h"

#include "../geometry/gpu_geo_ransac.h"
#include "bundle_adjustment.h"

#include <cmath>
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

} // namespace

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

#if defined(INSIGHTAT_USE_CERES) && INSIGHTAT_USE_CERES
  Eigen::Matrix3d R_refined = R_eig;
  Eigen::Vector3d t_refined = t_eig;
  double rmse = 0.0;
  if (!pose_only_bundle(inlier_pts3d, inlier_pts2d, fx, fy, cx, cy, R_eig, t_eig, &R_refined,
                        &t_refined, &rmse, 30)) {
    if (inliers_out)
      *inliers_out = n_inliers;
    *R_out = R_eig;
    *t_out = t_eig;
    return true;
  }
  *R_out = R_refined;
  *t_out = t_refined;
#else
  *R_out = R_eig;
  *t_out = t_eig;
#endif
  if (inliers_out)
    *inliers_out = n_inliers;
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
