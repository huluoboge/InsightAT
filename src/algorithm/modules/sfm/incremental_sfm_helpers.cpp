/**
 * @file  incremental_sfm_helpers.cpp
 * @brief Outlier rejection, track filtering, re-triangulation for two-view.
 */

#include "incremental_sfm_helpers.h"
#include "two_view_reconstruction.h"
#include <Eigen/SVD>
#include <cmath>
#include <vector>

namespace insight {
namespace sfm {

int reject_outliers_two_view(TrackStore* store,
                             const Eigen::Matrix3d& R,
                             const Eigen::Vector3d& t,
                             double fx, double fy, double cx, double cy,
                             double threshold_px) {
  if (!store) return 0;
  const double thresh_sq = threshold_px * threshold_px;
  int marked = 0;
  const size_t n_obs = store->num_observations();
  for (size_t i = 0; i < n_obs; ++i) {
    const int obs_id = static_cast<int>(i);
    if (!store->is_obs_valid(obs_id)) continue;
    Observation obs;
    store->get_obs(obs_id, &obs);
    if (obs.image_index > 1u) continue;
    const int tid = store->obs_track_id(obs_id);
    float tx, ty, tz;
    store->get_track_xyz(tid, &tx, &ty, &tz);
    double x = static_cast<double>(tx), y = static_cast<double>(ty), z = static_cast<double>(tz);
    double u_pred, v_pred;
    if (obs.image_index == 0) {
      if (z <= 1e-12) { store->mark_observation_deleted(obs_id); ++marked; continue; }
      u_pred = fx * x / z + cx;
      v_pred = fy * y / z + cy;
    } else {
      Eigen::Vector3d p = R * Eigen::Vector3d(x, y, z) + t;
      if (p(2) <= 1e-12) { store->mark_observation_deleted(obs_id); ++marked; continue; }
      u_pred = fx * p(0) / p(2) + cx;
      v_pred = fy * p(1) / p(2) + cy;
    }
    double du = static_cast<double>(obs.u) - u_pred;
    double dv = static_cast<double>(obs.v) - v_pred;
    if (du * du + dv * dv > thresh_sq) {
      store->mark_observation_deleted(obs_id);
      ++marked;
    }
  }
  return marked;
}

int filter_tracks_two_view(TrackStore* store,
                           const Eigen::Matrix3d& R,
                           const Eigen::Vector3d& t,
                           int min_observations,
                           double min_angle_deg) {
  if (!store) return 0;
  const double min_angle_rad = min_angle_deg * (3.141592653589793 / 180.0);
  const size_t n_tracks = store->num_tracks();
  int marked = 0;
  std::vector<Observation> obs_buf;
  Eigen::Vector3d c1 = -R.transpose() * t;
  for (size_t ti = 0; ti < n_tracks; ++ti) {
    const int track_id = static_cast<int>(ti);
    if (!store->is_track_valid(track_id)) continue;
    obs_buf.clear();
    const int n_obs = store->get_track_observations(track_id, &obs_buf);
    if (n_obs < min_observations) {
      store->mark_track_deleted(track_id);
      ++marked;
      continue;
    }
    if (n_obs >= 2 && min_angle_deg > 0) {
      float tx, ty, tz;
      store->get_track_xyz(track_id, &tx, &ty, &tz);
      Eigen::Vector3d X(tx, ty, tz);
      Eigen::Vector3d r0 = X.normalized();
      Eigen::Vector3d r1 = (X - c1).normalized();
      double cos_a = r0.dot(r1);
      if (cos_a > 1.0) cos_a = 1.0;
      if (cos_a < -1.0) cos_a = -1.0;
      double angle = std::acos(cos_a);
      if (angle < min_angle_rad) {
        store->mark_track_deleted(track_id);
        ++marked;
      }
    }
  }
  return marked;
}

int retriangulate_two_view_tracks(TrackStore* store,
                                  const Eigen::Matrix3d& R,
                                  const Eigen::Vector3d& t,
                                  double fx, double fy, double cx, double cy) {
  if (!store) return 0;
  std::vector<Observation> obs_buf;
  const size_t n_tracks = store->num_tracks();
  int updated = 0;
  for (size_t ti = 0; ti < n_tracks; ++ti) {
    const int track_id = static_cast<int>(ti);
    if (!store->is_track_valid(track_id)) continue;
    obs_buf.clear();
    const int n_obs = store->get_track_observations(track_id, &obs_buf);
    if (n_obs != 2) continue;
    uint32_t img0 = obs_buf[0].image_index, img1 = obs_buf[1].image_index;
    if (img0 > 1u || img1 > 1u) continue;
    if (img0 == img1) continue;
    Eigen::Vector2d p0(static_cast<double>(obs_buf[0].u), static_cast<double>(obs_buf[0].v));
    Eigen::Vector2d p1(static_cast<double>(obs_buf[1].u), static_cast<double>(obs_buf[1].v));
    Eigen::Vector2d cam1_n, cam2_n;
    cam1_n(0) = (p0(0) - cx) / fx; cam1_n(1) = (p0(1) - cy) / fy;
    cam2_n(0) = (p1(0) - cx) / fx; cam2_n(1) = (p1(1) - cy) / fy;
    if (img0 == 1u) { cam1_n.swap(cam2_n); }
    Eigen::Vector3d X = TriangulatePoint(cam1_n, cam2_n, R, t);
    if (X(2) <= 1e-9) continue;
    store->set_track_xyz(track_id, static_cast<float>(X(0)), static_cast<float>(X(1)), static_cast<float>(X(2)));
    ++updated;
  }
  return updated;
}

namespace {

// Multi-view DLT: one 3D point from N views. P_i = [R_i | t_i] world-to-camera.
// rays_n: normalized (nx, ny) per view, i.e. (u-cx)/fx, (v-cy)/fy.
Eigen::Vector3d TriangulatePointMultiview(
    const std::vector<Eigen::Matrix3d>& R_list,
    const std::vector<Eigen::Vector3d>& t_list,
    const std::vector<Eigen::Vector2d>& rays_n) {
  const int N = static_cast<int>(R_list.size());
  if (N < 2 || static_cast<int>(t_list.size()) != N || static_cast<int>(rays_n.size()) != N)
    return Eigen::Vector3d(0, 0, 0);
  Eigen::MatrixXd A(2 * N, 4);
  for (int i = 0; i < N; ++i) {
    const Eigen::Matrix3d& R = R_list[i];
    const Eigen::Vector3d& t = t_list[i];
    const double nx = rays_n[i](0), ny = rays_n[i](1);
    A.row(2 * i)     << nx * R.row(1) - ny * R.row(0), nx * t(1) - ny * t(0);
    A.row(2 * i + 1) << nx * R.row(2) - R.row(0),      nx * t(2) - t(0);
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
  const Eigen::Vector4d v = svd.matrixV().col(3);
  if (std::fabs(v(3)) < 1e-12) return Eigen::Vector3d(0, 0, 0);
  return v.head<3>() / v(3);
}

}  // namespace

int triangulate_tracks_for_new_image(TrackStore* store,
                                     int new_image_index,
                                     const std::vector<Eigen::Matrix3d>& poses_R,
                                     const std::vector<Eigen::Vector3d>& poses_t,
                                     const std::vector<bool>& registered_indices,
                                     double fx, double fy, double cx, double cy) {
  if (!store) return 0;
  const int n_images = store->num_images();
  if (new_image_index < 0 || new_image_index >= n_images) return 0;
  if (static_cast<int>(poses_R.size()) != n_images ||
      static_cast<int>(poses_t.size()) != n_images ||
      static_cast<int>(registered_indices.size()) != n_images)
    return 0;
  if (!registered_indices[static_cast<size_t>(new_image_index)]) return 0;

  int updated = 0;
  std::vector<Observation> obs_buf;
  const size_t n_tracks = store->num_tracks();
  for (size_t ti = 0; ti < n_tracks; ++ti) {
    const int track_id = static_cast<int>(ti);
    if (!store->is_track_valid(track_id)) continue;
    if (store->track_has_triangulated_xyz(track_id)) continue;
    obs_buf.clear();
    store->get_track_observations(track_id, &obs_buf);
    bool has_new = false;
    std::vector<int> reg_inds;
    std::vector<Eigen::Vector2d> rays_n;
    for (const Observation& o : obs_buf) {
      const int im = static_cast<int>(o.image_index);
      if (im == new_image_index) has_new = true;
      if (!registered_indices[static_cast<size_t>(im)]) continue;
      const double nx = (static_cast<double>(o.u) - cx) / fx;
      const double ny = (static_cast<double>(o.v) - cy) / fy;
      reg_inds.push_back(im);
      rays_n.push_back(Eigen::Vector2d(nx, ny));
    }
    if (!has_new || static_cast<int>(reg_inds.size()) < 2) continue;
    std::vector<Eigen::Matrix3d> R_list;
    std::vector<Eigen::Vector3d> t_list;
    R_list.reserve(reg_inds.size());
    t_list.reserve(reg_inds.size());
    for (int im : reg_inds) {
      R_list.push_back(poses_R[static_cast<size_t>(im)]);
      t_list.push_back(poses_t[static_cast<size_t>(im)]);
    }
    Eigen::Vector3d X = TriangulatePointMultiview(R_list, t_list, rays_n);
    if (X(2) <= 1e-9) continue;
    store->set_track_xyz(track_id, static_cast<float>(X(0)), static_cast<float>(X(1)), static_cast<float>(X(2)));
    ++updated;
  }
  return updated;
}

}  // namespace sfm
}  // namespace insight
