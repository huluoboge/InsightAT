/**
 * @file  incremental_sfm_helpers.cpp
 * @brief Outlier rejection, track filtering, re-triangulation for two-view.
 */

#include "incremental_sfm_helpers.h"
#include "../camera/camera_utils.h"
#include "two_view_reconstruction.h"
#include <Eigen/SVD>
#include <cmath>
#include <vector>

namespace insight {
namespace sfm {

int reject_outliers_two_view(TrackStore* store, const Eigen::Matrix3d& R,
                             const Eigen::Vector3d& C,
                             double fx, double fy, double cx, double cy, double threshold_px) {
  if (!store)
    return 0;
  const double thresh_sq = threshold_px * threshold_px;
  int marked = 0;
  const size_t n_obs = store->num_observations();
  for (size_t i = 0; i < n_obs; ++i) {
    const int obs_id = static_cast<int>(i);
    if (!store->is_obs_valid(obs_id))
      continue;
    Observation obs;
    store->get_obs(obs_id, &obs);
    if (obs.image_index > 1u)
      continue;
    const int tid = store->obs_track_id(obs_id);
    float tx, ty, tz;
    store->get_track_xyz(tid, &tx, &ty, &tz);
    const Eigen::Vector3d X(static_cast<double>(tx), static_cast<double>(ty),
                            static_cast<double>(tz));
    double u_pred, v_pred;
    if (obs.image_index == 0) {
      if (X(2) <= 1e-12) {
        store->mark_observation_deleted(obs_id);
        ++marked;
        continue;
      }
      u_pred = fx * X(0) / X(2) + cx;
      v_pred = fy * X(1) / X(2) + cy;
    } else {
      Eigen::Vector3d p = R * (X - C);
      if (p(2) <= 1e-12) {
        store->mark_observation_deleted(obs_id);
        ++marked;
        continue;
      }
      u_pred = fx * p(0) / p(2) + cx;
      v_pred = fy * p(1) / p(2) + cy;
    }
    const double du = static_cast<double>(obs.u) - u_pred;
    const double dv = static_cast<double>(obs.v) - v_pred;
    if (du * du + dv * dv > thresh_sq) {
      store->mark_observation_deleted(obs_id);
      ++marked;
    }
  }
  return marked;
}

int reject_outliers_two_view(TrackStore* store, const Eigen::Matrix3d& R,
                             const Eigen::Vector3d& C,
                             const camera::Intrinsics& K, double threshold_px) {
  if (!store)
    return 0;
  if (!K.has_distortion()) {
    return reject_outliers_two_view(store, R, C, K.fx, K.fy, K.cx, K.cy, threshold_px);
  }

  const double fx = K.fx, fy = K.fy, cx = K.cx, cy = K.cy;
  const double thresh_sq = threshold_px * threshold_px;
  int marked = 0;
  const size_t n_obs = store->num_observations();
  for (size_t i = 0; i < n_obs; ++i) {
    const int obs_id = static_cast<int>(i);
    if (!store->is_obs_valid(obs_id))
      continue;
    Observation obs;
    store->get_obs(obs_id, &obs);
    if (obs.image_index > 1u)
      continue;
    const int tid = store->obs_track_id(obs_id);
    float tx, ty, tz;
    store->get_track_xyz(tid, &tx, &ty, &tz);
    const Eigen::Vector3d X(static_cast<double>(tx), static_cast<double>(ty),
                            static_cast<double>(tz));
    double u_pred, v_pred;
    if (obs.image_index == 0) {
      if (X(2) <= 1e-12) {
        store->mark_observation_deleted(obs_id);
        ++marked;
        continue;
      }
      u_pred = fx * X(0) / X(2) + cx;
      v_pred = fy * X(1) / X(2) + cy;
    } else {
      Eigen::Vector3d p = R * (X - C);
      if (p(2) <= 1e-12) {
        store->mark_observation_deleted(obs_id);
        ++marked;
        continue;
      }
      u_pred = fx * p(0) / p(2) + cx;
      v_pred = fy * p(1) / p(2) + cy;
    }

    double u_und, v_und;
    camera::undistort_point(K, static_cast<double>(obs.u), static_cast<double>(obs.v),
                            &u_und, &v_und);

    const double du = u_und - u_pred;
    const double dv = v_und - v_pred;
    if (du * du + dv * dv > thresh_sq) {
      store->mark_observation_deleted(obs_id);
      ++marked;
    }
  }
  return marked;
}

int reject_outliers_two_view(TrackStore* store, const Eigen::Matrix3d& R,
                             const Eigen::Vector3d& C,
                             const camera::Intrinsics& K0, const camera::Intrinsics& K1,
                             double threshold_px) {
  if (!store)
    return 0;
  const double thresh_sq = threshold_px * threshold_px;
  int marked = 0;
  const size_t n_obs = store->num_observations();
  for (size_t i = 0; i < n_obs; ++i) {
    const int obs_id = static_cast<int>(i);
    if (!store->is_obs_valid(obs_id))
      continue;
    Observation obs;
    store->get_obs(obs_id, &obs);
    if (obs.image_index > 1u)
      continue;
    const camera::Intrinsics& K = (obs.image_index == 0) ? K0 : K1;
    const int tid = store->obs_track_id(obs_id);
    float tx, ty, tz;
    store->get_track_xyz(tid, &tx, &ty, &tz);
    const Eigen::Vector3d X(static_cast<double>(tx), static_cast<double>(ty),
                            static_cast<double>(tz));
    double u_pred, v_pred;
    if (obs.image_index == 0) {
      if (X(2) <= 1e-12) {
        store->mark_observation_deleted(obs_id);
        ++marked;
        continue;
      }
      u_pred = K.fx * X(0) / X(2) + K.cx;
      v_pred = K.fy * X(1) / X(2) + K.cy;
    } else {
      Eigen::Vector3d p = R * (X - C);
      if (p(2) <= 1e-12) {
        store->mark_observation_deleted(obs_id);
        ++marked;
        continue;
      }
      u_pred = K.fx * p(0) / p(2) + K.cx;
      v_pred = K.fy * p(1) / p(2) + K.cy;
    }
    double u_obs = static_cast<double>(obs.u);
    double v_obs = static_cast<double>(obs.v);
    if (K.has_distortion())
      camera::undistort_point(K, u_obs, v_obs, &u_obs, &v_obs);
    const double du = u_obs - u_pred;
    const double dv = v_obs - v_pred;
    if (du * du + dv * dv > thresh_sq) {
      store->mark_observation_deleted(obs_id);
      ++marked;
    }
  }
  return marked;
}

int filter_tracks_two_view(TrackStore* store, const Eigen::Matrix3d& R,
                           const Eigen::Vector3d& C,
                           int min_observations, double min_angle_deg) {
  if (!store)
    return 0;
  const double min_angle_rad = min_angle_deg * (3.141592653589793 / 180.0);
  const size_t n_tracks = store->num_tracks();
  int marked = 0;
  std::vector<Observation> obs_buf;
  const Eigen::Vector3d& c1 = C;
  for (size_t ti = 0; ti < n_tracks; ++ti) {
    const int track_id = static_cast<int>(ti);
    if (!store->is_track_valid(track_id))
      continue;
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
      if (cos_a > 1.0)
        cos_a = 1.0;
      if (cos_a < -1.0)
        cos_a = -1.0;
      double angle = std::acos(cos_a);
      if (angle < min_angle_rad) {
        store->mark_track_deleted(track_id);
        ++marked;
      }
    }
  }
  return marked;
}

int retriangulate_two_view_tracks(TrackStore* store, const Eigen::Matrix3d& R,
                                  const Eigen::Vector3d& C, double fx, double fy, double cx,
                                  double cy) {
  if (!store)
    return 0;
  std::vector<Observation> obs_buf;
  const size_t n_tracks = store->num_tracks();
  int updated = 0;
  for (size_t ti = 0; ti < n_tracks; ++ti) {
    const int track_id = static_cast<int>(ti);
    if (!store->is_track_valid(track_id))
      continue;
    obs_buf.clear();
    const int n_obs = store->get_track_observations(track_id, &obs_buf);
    if (n_obs != 2)
      continue;
    uint32_t img0 = obs_buf[0].image_index, img1 = obs_buf[1].image_index;
    if (img0 > 1u || img1 > 1u)
      continue;
    if (img0 == img1)
      continue;
    Eigen::Vector2d p0(static_cast<double>(obs_buf[0].u), static_cast<double>(obs_buf[0].v));
    Eigen::Vector2d p1(static_cast<double>(obs_buf[1].u), static_cast<double>(obs_buf[1].v));
    Eigen::Vector2d cam1_n, cam2_n;
    cam1_n(0) = (p0(0) - cx) / fx;
    cam1_n(1) = (p0(1) - cy) / fy;
    cam2_n(0) = (p1(0) - cx) / fx;
    cam2_n(1) = (p1(1) - cy) / fy;
    if (img0 == 1u) {
      cam1_n.swap(cam2_n);
    }
    const Eigen::Vector3d t_cam = -R * C; // world-to-camera translation for DLT
    Eigen::Vector3d X = insight::sfm::triangulate_point(cam1_n, cam2_n, R, t_cam);
    if (X(2) <= 1e-9)
      continue;
    store->set_track_xyz(track_id, static_cast<float>(X(0)), static_cast<float>(X(1)),
                         static_cast<float>(X(2)));
    ++updated;
  }
  return updated;
}

int retriangulate_two_view_tracks(TrackStore* store, const Eigen::Matrix3d& R,
                                  const Eigen::Vector3d& C,
                                  const camera::Intrinsics& K0, const camera::Intrinsics& K1) {
  if (!store)
    return 0;
  std::vector<Observation> obs_buf;
  const size_t n_tracks = store->num_tracks();
  int updated = 0;
  for (size_t ti = 0; ti < n_tracks; ++ti) {
    const int track_id = static_cast<int>(ti);
    if (!store->is_track_valid(track_id))
      continue;
    obs_buf.clear();
    const int n_obs = store->get_track_observations(track_id, &obs_buf);
    if (n_obs != 2)
      continue;
    uint32_t img0 = obs_buf[0].image_index, img1 = obs_buf[1].image_index;
    if (img0 > 1u || img1 > 1u)
      continue;
    if (img0 == img1)
      continue;
    double u0 = static_cast<double>(obs_buf[0].u), v0 = static_cast<double>(obs_buf[0].v);
    double u1 = static_cast<double>(obs_buf[1].u), v1 = static_cast<double>(obs_buf[1].v);
    const camera::Intrinsics& Ka = (obs_buf[0].image_index == 0) ? K0 : K1;
    const camera::Intrinsics& Kb = (obs_buf[1].image_index == 0) ? K0 : K1;
    if (Ka.has_distortion())
      camera::undistort_point(Ka, u0, v0, &u0, &v0);
    if (Kb.has_distortion())
      camera::undistort_point(Kb, u1, v1, &u1, &v1);
    Eigen::Vector2d cam1_n((u0 - Ka.cx) / Ka.fx, (v0 - Ka.cy) / Ka.fy);
    Eigen::Vector2d cam2_n((u1 - Kb.cx) / Kb.fx, (v1 - Kb.cy) / Kb.fy);
    if (obs_buf[0].image_index == 1u)
      cam1_n.swap(cam2_n);
    const Eigen::Vector3d t_cam = -R * C;
    Eigen::Vector3d X = insight::sfm::triangulate_point(cam1_n, cam2_n, R, t_cam);
    if (X(2) <= 1e-9)
      continue;
    store->set_track_xyz(track_id, static_cast<float>(X(0)), static_cast<float>(X(1)),
                         static_cast<float>(X(2)));
    ++updated;
  }
  return updated;
}

int retriangulate_two_view_tracks(TrackStore* store, int image0_index, int image1_index,
                                  const Eigen::Matrix3d& R, const Eigen::Vector3d& C,
                                  const camera::Intrinsics& K0, const camera::Intrinsics& K1) {
  if (!store || image0_index == image1_index)
    return 0;
  const uint32_t im0 = static_cast<uint32_t>(image0_index);
  const uint32_t im1 = static_cast<uint32_t>(image1_index);
  std::vector<Observation> obs_buf;
  const size_t n_tracks = store->num_tracks();
  int updated = 0;
  for (size_t ti = 0; ti < n_tracks; ++ti) {
    const int track_id = static_cast<int>(ti);
    if (!store->is_track_valid(track_id))
      continue;
    obs_buf.clear();
    store->get_track_observations(track_id, &obs_buf);
    double u0 = 0, v0 = 0, u1 = 0, v1 = 0;
    bool has0 = false, has1 = false;
    for (const auto& o : obs_buf) {
      if (o.image_index == im0) {
        u0 = static_cast<double>(o.u);
        v0 = static_cast<double>(o.v);
        has0 = true;
        break;
      }
    }
    for (const auto& o : obs_buf) {
      if (o.image_index == im1) {
        u1 = static_cast<double>(o.u);
        v1 = static_cast<double>(o.v);
        has1 = true;
        break;
      }
    }
    if (!has0 || !has1)
      continue;
    if (K0.has_distortion())
      camera::undistort_point(K0, u0, v0, &u0, &v0);
    if (K1.has_distortion())
      camera::undistort_point(K1, u1, v1, &u1, &v1);
    Eigen::Vector2d cam0_n((u0 - K0.cx) / K0.fx, (v0 - K0.cy) / K0.fy);
    Eigen::Vector2d cam1_n((u1 - K1.cx) / K1.fx, (v1 - K1.cy) / K1.fy);
    const Eigen::Vector3d t_cam = -R * C;
    Eigen::Vector3d X = insight::sfm::triangulate_point(cam0_n, cam1_n, R, t_cam);
    if (X(2) <= 1e-9)
      continue;
    store->set_track_xyz(track_id, static_cast<float>(X(0)), static_cast<float>(X(1)),
                        static_cast<float>(X(2)));
    ++updated;
  }
  return updated;
}

int reject_outliers_two_view(TrackStore* store, const Eigen::Matrix3d& R,
                             const Eigen::Vector3d& C, int image0_index, int image1_index,
                             const camera::Intrinsics& K0, const camera::Intrinsics& K1,
                             double threshold_px) {
  if (!store)
    return 0;
  const uint32_t im0 = static_cast<uint32_t>(image0_index);
  const uint32_t im1 = static_cast<uint32_t>(image1_index);
  const double thresh_sq = threshold_px * threshold_px;
  int marked = 0;
  const size_t n_obs = store->num_observations();
  for (size_t i = 0; i < n_obs; ++i) {
    const int obs_id = static_cast<int>(i);
    if (!store->is_obs_valid(obs_id))
      continue;
    Observation obs;
    store->get_obs(obs_id, &obs);
    if (obs.image_index != im0 && obs.image_index != im1)
      continue;
    const camera::Intrinsics& K = (obs.image_index == im0) ? K0 : K1;
    const int tid = store->obs_track_id(obs_id);
    float tx, ty, tz;
    store->get_track_xyz(tid, &tx, &ty, &tz);
    const Eigen::Vector3d X(static_cast<double>(tx), static_cast<double>(ty),
                            static_cast<double>(tz));
    double u_pred, v_pred;
    if (obs.image_index == im0) {
      if (X(2) <= 1e-12) {
        store->mark_observation_deleted(obs_id);
        ++marked;
        continue;
      }
      u_pred = K.fx * X(0) / X(2) + K.cx;
      v_pred = K.fy * X(1) / X(2) + K.cy;
    } else {
      Eigen::Vector3d p = R * (X - C);
      if (p(2) <= 1e-12) {
        store->mark_observation_deleted(obs_id);
        ++marked;
        continue;
      }
      u_pred = K.fx * p(0) / p(2) + K.cx;
      v_pred = K.fy * p(1) / p(2) + K.cy;
    }
    double u_obs = static_cast<double>(obs.u);
    double v_obs = static_cast<double>(obs.v);
    if (K.has_distortion())
      camera::undistort_point(K, u_obs, v_obs, &u_obs, &v_obs);
    const double du = u_obs - u_pred;
    const double dv = v_obs - v_pred;
    if (du * du + dv * dv > thresh_sq) {
      store->mark_observation_deleted(obs_id);
      ++marked;
    }
  }
  return marked;
}

int filter_tracks_two_view(TrackStore* store, const Eigen::Matrix3d& R,
                           const Eigen::Vector3d& C, int image0_index, int image1_index,
                           int min_observations, double min_angle_deg) {
  if (!store || image0_index == image1_index)
    return 0;
  const uint32_t im0 = static_cast<uint32_t>(image0_index);
  const uint32_t im1 = static_cast<uint32_t>(image1_index);
  const double min_angle_rad = min_angle_deg * (3.141592653589793 / 180.0);
  const size_t n_tracks = store->num_tracks();
  int marked = 0;
  std::vector<Observation> obs_buf;
  const Eigen::Vector3d& c1 = C;
  for (size_t ti = 0; ti < n_tracks; ++ti) {
    const int track_id = static_cast<int>(ti);
    if (!store->is_track_valid(track_id))
      continue;
    obs_buf.clear();
    store->get_track_observations(track_id, &obs_buf);
    int n_in_pair = 0;
    for (const auto& o : obs_buf) {
      if (o.image_index == im0 || o.image_index == im1)
        ++n_in_pair;
    }
    if (n_in_pair < min_observations) {
      store->mark_track_deleted(track_id);
      ++marked;
      continue;
    }
    if (n_in_pair >= 2 && min_angle_deg > 0 && store->track_has_triangulated_xyz(track_id)) {
      float tx, ty, tz;
      store->get_track_xyz(track_id, &tx, &ty, &tz);
      Eigen::Vector3d X(tx, ty, tz);
      Eigen::Vector3d r0 = X.normalized();
      Eigen::Vector3d r1 = (X - c1).normalized();
      double cos_a = r0.dot(r1);
      if (cos_a > 1.0)
        cos_a = 1.0;
      if (cos_a < -1.0)
        cos_a = -1.0;
      double angle = std::acos(cos_a);
      if (angle < min_angle_rad) {
        store->mark_track_deleted(track_id);
        ++marked;
      }
    }
  }
  return marked;
}

namespace {

// Multi-view DLT: one 3D point from N views. Pose: R_i (world-to-camera), C_i (camera centre).
// t_i = -R_i * C_i used internally for the DLT formulation.
// rays_n: normalized (nx, ny) per view, i.e. (u-cx)/fx, (v-cy)/fy.
Eigen::Vector3d triangulate_point_multiview(const std::vector<Eigen::Matrix3d>& R_list,
                                            const std::vector<Eigen::Vector3d>& C_list,
                                            const std::vector<Eigen::Vector2d>& rays_n) {
  const int N = static_cast<int>(R_list.size());
  if (N < 2 || static_cast<int>(C_list.size()) != N || static_cast<int>(rays_n.size()) != N)
    return Eigen::Vector3d(0, 0, 0);
  Eigen::MatrixXd A(2 * N, 4);
  for (int i = 0; i < N; ++i) {
    const Eigen::Matrix3d& R = R_list[i];
    const Eigen::Vector3d  t = -R * C_list[i];
    const double nx = rays_n[i](0), ny = rays_n[i](1);
    A.row(2 * i)     << nx * R.row(1) - ny * R.row(0), nx * t(1) - ny * t(0);
    A.row(2 * i + 1) << nx * R.row(2) - R.row(0),      nx * t(2) - t(0);
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
  const Eigen::Vector4d v = svd.matrixV().col(3);
  if (std::fabs(v(3)) < 1e-12)
    return Eigen::Vector3d(0, 0, 0);
  return v.head<3>() / v(3);
}

} // namespace

int triangulate_tracks_for_new_image(TrackStore* store, int new_image_index,
                                     const std::vector<Eigen::Matrix3d>& poses_R,
                                     const std::vector<Eigen::Vector3d>& poses_C,
                                     const std::vector<bool>& registered_indices, double fx,
                                     double fy, double cx, double cy) {
  if (!store)
    return 0;
  const int n_images = store->num_images();
  if (new_image_index < 0 || new_image_index >= n_images)
    return 0;
  if (static_cast<int>(poses_R.size()) != n_images ||
      static_cast<int>(poses_C.size()) != n_images ||
      static_cast<int>(registered_indices.size()) != n_images)
    return 0;
  if (!registered_indices[static_cast<size_t>(new_image_index)])
    return 0;

  int updated = 0;
  std::vector<Observation> obs_buf;
  const size_t n_tracks = store->num_tracks();
  for (size_t ti = 0; ti < n_tracks; ++ti) {
    const int track_id = static_cast<int>(ti);
    if (!store->is_track_valid(track_id))
      continue;
    if (store->track_has_triangulated_xyz(track_id))
      continue;
    obs_buf.clear();
    store->get_track_observations(track_id, &obs_buf);
    bool has_new = false;
    std::vector<int> reg_inds;
    std::vector<Eigen::Vector2d> rays_n;
    for (const Observation& o : obs_buf) {
      const int im = static_cast<int>(o.image_index);
      if (im == new_image_index)
        has_new = true;
      if (!registered_indices[static_cast<size_t>(im)])
        continue;
      const double nx = (static_cast<double>(o.u) - cx) / fx;
      const double ny = (static_cast<double>(o.v) - cy) / fy;
      reg_inds.push_back(im);
      rays_n.push_back(Eigen::Vector2d(nx, ny));
    }
    if (!has_new || static_cast<int>(reg_inds.size()) < 2)
      continue;
    std::vector<Eigen::Matrix3d> R_list;
    std::vector<Eigen::Vector3d> C_list;
    R_list.reserve(reg_inds.size());
    C_list.reserve(reg_inds.size());
    for (int im : reg_inds) {
      R_list.push_back(poses_R[static_cast<size_t>(im)]);
      C_list.push_back(poses_C[static_cast<size_t>(im)]);
    }
    Eigen::Vector3d X = triangulate_point_multiview(R_list, C_list, rays_n);
    if (X(2) <= 1e-9)
      continue;
    store->set_track_xyz(track_id, static_cast<float>(X(0)), static_cast<float>(X(1)),
                         static_cast<float>(X(2)));
    ++updated;
  }
  return updated;
}

int triangulate_tracks_for_new_image(TrackStore* store, int new_image_index,
                                     const std::vector<Eigen::Matrix3d>& poses_R,
                                     const std::vector<Eigen::Vector3d>& poses_C,
                                     const std::vector<bool>& registered_indices,
                                     const std::vector<camera::Intrinsics>& intrinsics_per_image) {
  if (!store)
    return 0;
  const int n_images = store->num_images();
  if (new_image_index < 0 || new_image_index >= n_images)
    return 0;
  if (static_cast<int>(poses_R.size()) != n_images ||
      static_cast<int>(poses_C.size()) != n_images ||
      static_cast<int>(registered_indices.size()) != n_images ||
      static_cast<int>(intrinsics_per_image.size()) != n_images)
    return 0;
  if (!registered_indices[static_cast<size_t>(new_image_index)])
    return 0;

  int updated = 0;
  std::vector<Observation> obs_buf;
  const size_t n_tracks = store->num_tracks();
  for (size_t ti = 0; ti < n_tracks; ++ti) {
    const int track_id = static_cast<int>(ti);
    if (!store->is_track_valid(track_id))
      continue;
    if (store->track_has_triangulated_xyz(track_id))
      continue;
    obs_buf.clear();
    store->get_track_observations(track_id, &obs_buf);
    bool has_new = false;
    std::vector<int> reg_inds;
    std::vector<Eigen::Vector2d> rays_n;
    for (const Observation& o : obs_buf) {
      const int im = static_cast<int>(o.image_index);
      if (im == new_image_index)
        has_new = true;
      if (!registered_indices[static_cast<size_t>(im)])
        continue;
      const camera::Intrinsics& K = intrinsics_per_image[static_cast<size_t>(im)];
      double u = static_cast<double>(o.u), v = static_cast<double>(o.v);
      if (K.has_distortion())
        camera::undistort_point(K, u, v, &u, &v);
      const double nx = (u - K.cx) / K.fx;
      const double ny = (v - K.cy) / K.fy;
      reg_inds.push_back(im);
      rays_n.push_back(Eigen::Vector2d(nx, ny));
    }
    if (!has_new || static_cast<int>(reg_inds.size()) < 2)
      continue;
    std::vector<Eigen::Matrix3d> R_list;
    std::vector<Eigen::Vector3d> C_list;
    R_list.reserve(reg_inds.size());
    C_list.reserve(reg_inds.size());
    for (int im : reg_inds) {
      R_list.push_back(poses_R[static_cast<size_t>(im)]);
      C_list.push_back(poses_C[static_cast<size_t>(im)]);
    }
    Eigen::Vector3d X = triangulate_point_multiview(R_list, C_list, rays_n);
    if (X(2) <= 1e-9)
      continue;
    store->set_track_xyz(track_id, static_cast<float>(X(0)), static_cast<float>(X(1)),
                         static_cast<float>(X(2)));
    ++updated;
  }
  return updated;
}

// ─────────────────────────────────────────────────────────────────────────────
// Multiview versions
// ─────────────────────────────────────────────────────────────────────────────

int reject_outliers_multiview(TrackStore* store,
                              const std::vector<Eigen::Matrix3d>& poses_R,
                              const std::vector<Eigen::Vector3d>& poses_C,
                              const std::vector<bool>& registered, double fx, double fy,
                              double cx, double cy, double threshold_px) {
  if (!store || poses_R.size() != poses_C.size() || poses_R.size() != registered.size())
    return 0;
  const double thresh_sq = threshold_px * threshold_px;
  int marked = 0;
  const size_t n_obs = store->num_observations();
  const size_t n_images = poses_R.size();
  for (size_t i = 0; i < n_obs; ++i) {
    const int obs_id = static_cast<int>(i);
    if (!store->is_obs_valid(obs_id))
      continue;
    Observation obs;
    store->get_obs(obs_id, &obs);
    if (obs.image_index >= n_images || !registered[obs.image_index])
      continue;
    const int tid = store->obs_track_id(obs_id);
    if (!store->track_has_triangulated_xyz(tid))
      continue;
    float tx, ty, tz;
    store->get_track_xyz(tid, &tx, &ty, &tz);
    const Eigen::Vector3d X(tx, ty, tz);
    const Eigen::Vector3d p =
        poses_R[obs.image_index] * (X - poses_C[obs.image_index]);
    if (p(2) <= 1e-12) {
      store->mark_observation_deleted(obs_id);
      ++marked;
      continue;
    }
    const double u_pred = fx * p(0) / p(2) + cx;
    const double v_pred = fy * p(1) / p(2) + cy;
    const double du = static_cast<double>(obs.u) - u_pred;
    const double dv = static_cast<double>(obs.v) - v_pred;
    if (du * du + dv * dv > thresh_sq) {
      store->mark_observation_deleted(obs_id);
      ++marked;
    }
  }
  return marked;
}

int reject_outliers_multiview(TrackStore* store,
                              const std::vector<Eigen::Matrix3d>& poses_R,
                              const std::vector<Eigen::Vector3d>& poses_C,
                              const std::vector<bool>& registered,
                              const camera::Intrinsics& K, double threshold_px) {
  if (!store || !K.has_distortion()) {
    return reject_outliers_multiview(store, poses_R, poses_C, registered, K.fx,
                                     K.fy, K.cx, K.cy, threshold_px);
  }
  if (poses_R.size() != poses_C.size() || poses_R.size() != registered.size())
    return 0;
  const double fx = K.fx, fy = K.fy, cx = K.cx, cy = K.cy;
  const double thresh_sq = threshold_px * threshold_px;
  int marked = 0;
  const size_t n_obs = store->num_observations();
  const size_t n_images = poses_R.size();
  for (size_t i = 0; i < n_obs; ++i) {
    const int obs_id = static_cast<int>(i);
    if (!store->is_obs_valid(obs_id))
      continue;
    Observation obs;
    store->get_obs(obs_id, &obs);
    if (obs.image_index >= n_images || !registered[obs.image_index])
      continue;
    const int tid = store->obs_track_id(obs_id);
    if (!store->track_has_triangulated_xyz(tid))
      continue;
    float tx, ty, tz;
    store->get_track_xyz(tid, &tx, &ty, &tz);
    const Eigen::Vector3d X(tx, ty, tz);
    const Eigen::Vector3d p =
        poses_R[obs.image_index] * (X - poses_C[obs.image_index]);
    if (p(2) <= 1e-12) {
      store->mark_observation_deleted(obs_id);
      ++marked;
      continue;
    }
    const double u_pred = fx * p(0) / p(2) + cx;
    const double v_pred = fy * p(1) / p(2) + cy;
    double u_und, v_und;
    camera::undistort_point(K, static_cast<double>(obs.u), static_cast<double>(obs.v),
                            &u_und, &v_und);
    const double du = u_und - u_pred;
    const double dv = v_und - v_pred;
    if (du * du + dv * dv > thresh_sq) {
      store->mark_observation_deleted(obs_id);
      ++marked;
    }
  }
  return marked;
}

int reject_outliers_multiview(TrackStore* store,
                              const std::vector<Eigen::Matrix3d>& poses_R,
                              const std::vector<Eigen::Vector3d>& poses_C,
                              const std::vector<bool>& registered,
                              const std::vector<camera::Intrinsics>& intrinsics_per_image,
                              double threshold_px) {
  if (!store || poses_R.size() != poses_C.size() || poses_R.size() != registered.size() ||
      intrinsics_per_image.size() != poses_R.size())
    return 0;
  const double thresh_sq = threshold_px * threshold_px;
  int marked = 0;
  const size_t n_obs = store->num_observations();
  const size_t n_images = poses_R.size();
  for (size_t i = 0; i < n_obs; ++i) {
    const int obs_id = static_cast<int>(i);
    if (!store->is_obs_valid(obs_id))
      continue;
    Observation obs;
    store->get_obs(obs_id, &obs);
    if (obs.image_index >= n_images || !registered[obs.image_index])
      continue;
    const int tid = store->obs_track_id(obs_id);
    if (!store->track_has_triangulated_xyz(tid))
      continue;
    const camera::Intrinsics& K = intrinsics_per_image[obs.image_index];
    float tx, ty, tz;
    store->get_track_xyz(tid, &tx, &ty, &tz);
    const Eigen::Vector3d X(tx, ty, tz);
    const Eigen::Vector3d p =
        poses_R[obs.image_index] * (X - poses_C[obs.image_index]);
    if (p(2) <= 1e-12) {
      store->mark_observation_deleted(obs_id);
      ++marked;
      continue;
    }
    const double u_pred = K.fx * p(0) / p(2) + K.cx;
    const double v_pred = K.fy * p(1) / p(2) + K.cy;
    double u_obs = static_cast<double>(obs.u);
    double v_obs = static_cast<double>(obs.v);
    if (K.has_distortion()) {
      camera::undistort_point(K, u_obs, v_obs, &u_obs, &v_obs);
    }
    const double du = u_obs - u_pred;
    const double dv = v_obs - v_pred;
    if (du * du + dv * dv > thresh_sq) {
      store->mark_observation_deleted(obs_id);
      ++marked;
    }
  }
  return marked;
}

int filter_tracks_multiview(TrackStore* store,
                            const std::vector<Eigen::Matrix3d>& poses_R,
                            const std::vector<Eigen::Vector3d>& poses_C,
                            const std::vector<bool>& registered, int min_observations,
                            double min_angle_deg) {
  if (!store || poses_R.size() != poses_C.size() || poses_R.size() != registered.size())
    return 0;
  const double min_angle_rad = min_angle_deg * (3.141592653589793 / 180.0);
  const size_t n_tracks = store->num_tracks();
  int marked = 0;
  std::vector<Observation> obs_buf;
  for (size_t ti = 0; ti < n_tracks; ++ti) {
    const int track_id = static_cast<int>(ti);
    if (!store->is_track_valid(track_id))
      continue;
    obs_buf.clear();
    const int n_obs = store->get_track_observations(track_id, &obs_buf);
    if (n_obs < min_observations) {
      store->mark_track_deleted(track_id);
      ++marked;
      continue;
    }
    if (n_obs >= 2 && min_angle_deg > 0 && store->track_has_triangulated_xyz(track_id)) {
      float tx, ty, tz;
      store->get_track_xyz(track_id, &tx, &ty, &tz);
      const Eigen::Vector3d X(tx, ty, tz);
      double max_cos = -1.0;
      for (size_t i = 0; i < static_cast<size_t>(n_obs); ++i) {
        const int im = static_cast<int>(obs_buf[i].image_index);
        if (im < 0 || im >= static_cast<int>(poses_R.size()) ||
            !registered[static_cast<size_t>(im)])
          continue;
        const Eigen::Vector3d r = (X - poses_C[static_cast<size_t>(im)]).normalized();
        for (size_t j = i + 1; j < static_cast<size_t>(n_obs); ++j) {
          const int jm = static_cast<int>(obs_buf[j].image_index);
          if (jm < 0 || jm >= static_cast<int>(poses_R.size()) ||
              !registered[static_cast<size_t>(jm)])
            continue;
          const Eigen::Vector3d rj = (X - poses_C[static_cast<size_t>(jm)]).normalized();
          double cos_a = r.dot(rj);
          if (cos_a > 1.0)  cos_a = 1.0;
          if (cos_a < -1.0) cos_a = -1.0;
          if (cos_a > max_cos)
            max_cos = cos_a;
        }
      }
      const double min_angle = std::acos(max_cos);
      if (min_angle < min_angle_rad) {
        store->mark_track_deleted(track_id);
        ++marked;
      }
    }
  }
  return marked;
}

} // namespace sfm
} // namespace insight
