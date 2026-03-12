/**
 * @file  incremental_sfm_pipeline.cpp
 * @brief Initial pair loop and (later) full incremental SfM pipeline.
 */

#include "incremental_sfm_pipeline.h"

#include "../../io/idc_reader.h"
#include "../../io/track_store_idc.h"
#include "../camera/camera_utils.h"
#include "bundle_adjustment_analytic.h"
#include "resection.h"
#include "track_store.h"
#include "two_view_reconstruction.h"
#include "view_graph.h"
#include "view_graph_loader.h"

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <glog/logging.h>
#include <limits>
#include <set>
#include <string>
#include <unordered_set>
#include <unordered_map>
#include <utility>
#include <vector>

namespace insight {
namespace sfm {

namespace {

bool load_pair_geo(const std::string& geo_path, Eigen::Matrix3d* R, Eigen::Vector3d* t) {
  if (!R || !t)
    return false;
  io::IDCReader reader(geo_path);
  if (!reader.is_valid())
    return false;
  const auto& meta = reader.get_metadata();
  if (!meta.contains("twoview"))
    return false;
  auto R_blob = reader.read_blob<float>("R_matrix");
  auto t_blob = reader.read_blob<float>("t_vector");
  if (R_blob.size() != 9u || t_blob.size() != 3u)
    return false;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      (*R)(i, j) = static_cast<double>(R_blob[static_cast<size_t>(i * 3 + j)]);
  (*t)(0) = static_cast<double>(t_blob[0]);
  (*t)(1) = static_cast<double>(t_blob[1]);
  (*t)(2) = static_cast<double>(t_blob[2]);
  return true;
}

// NOTE: triangulate_two_view_tracks removed — replaced by local triangulation in
// try_initial_pair_candidate() which avoids mutating the TrackStore during trials.

int reject_outliers_two_view(TrackStore* store, const Eigen::Matrix3d& R, const Eigen::Vector3d& C,
                             int im0, int im1, const std::vector<camera::Intrinsics>& cameras,
                             const std::vector<int>& image_to_camera_index, double threshold_px) {
  if (!store)
    return 0;
  const camera::Intrinsics& K0 = cameras[static_cast<size_t>(image_to_camera_index[im0])];
  const camera::Intrinsics& K1 = cameras[static_cast<size_t>(image_to_camera_index[im1])];
  const uint32_t uim0 = static_cast<uint32_t>(im0), uim1 = static_cast<uint32_t>(im1);
  const double thresh_sq = threshold_px * threshold_px;
  int marked = 0;
  for (size_t i = 0; i < store->num_observations(); ++i) {
    const int obs_id = static_cast<int>(i);
    if (!store->is_obs_valid(obs_id))
      continue;
    Observation obs;
    store->get_obs(obs_id, &obs);
    if (obs.image_index != uim0 && obs.image_index != uim1)
      continue;
    const camera::Intrinsics& K = (obs.image_index == uim0) ? K0 : K1;
    const int tid = store->obs_track_id(obs_id);
    float tx, ty, tz;
    store->get_track_xyz(tid, &tx, &ty, &tz);
    const Eigen::Vector3d X(static_cast<double>(tx), static_cast<double>(ty),
                            static_cast<double>(tz));
    double u_pred, v_pred;
    if (obs.image_index == uim0) {
      if (X(2) <= 1e-12) {
        store->mark_observation_deleted(obs_id);
        ++marked;
        continue;
      }
      {
        const double xn = X(0) / X(2), yn = X(1) / X(2);
        double xd, yd;
        camera::apply_distortion(xn, yn, K, &xd, &yd);
        u_pred = K.fx * xd + K.cx;
        v_pred = K.fy * yd + K.cy;
      }
    } else {
      Eigen::Vector3d p = R * (X - C);
      if (p(2) <= 1e-12) {
        store->mark_observation_deleted(obs_id);
        ++marked;
        continue;
      }
      {
        const double xn = p(0) / p(2), yn = p(1) / p(2);
        double xd, yd;
        camera::apply_distortion(xn, yn, K, &xd, &yd);
        u_pred = K.fx * xd + K.cx;
        v_pred = K.fy * yd + K.cy;
      }
    }
    const double u_obs = static_cast<double>(obs.u);
    const double v_obs = static_cast<double>(obs.v);
    if ((u_obs - u_pred) * (u_obs - u_pred) + (v_obs - v_pred) * (v_obs - v_pred) > thresh_sq) {
      store->mark_observation_deleted(obs_id);
      ++marked;
    }
  }
  return marked;
}

/// Multi-view: mark observations with reprojection error > threshold_px (in undistorted pixel).
int reject_outliers_multiview(TrackStore* store, const std::vector<Eigen::Matrix3d>& poses_R,
                              const std::vector<Eigen::Vector3d>& poses_C,
                              const std::vector<bool>& registered,
                              const std::vector<camera::Intrinsics>& cameras,
                              const std::vector<int>& image_to_camera_index, double threshold_px) {
  if (!store)
    return 0;
  const int n_images = store->num_images();
  if (static_cast<int>(poses_R.size()) != n_images ||
      static_cast<int>(poses_C.size()) != n_images ||
      static_cast<int>(registered.size()) != n_images ||
      static_cast<int>(image_to_camera_index.size()) != n_images)
    return 0;
  const double thresh_sq = threshold_px * threshold_px;
  int marked = 0;
  for (size_t i = 0; i < store->num_observations(); ++i) {
    const int obs_id = static_cast<int>(i);
    if (!store->is_obs_valid(obs_id))
      continue;
    Observation obs;
    store->get_obs(obs_id, &obs);
    const int im = static_cast<int>(obs.image_index);
    if (im < 0 || im >= n_images || !registered[static_cast<size_t>(im)])
      continue;
    const int tid = store->obs_track_id(obs_id);
    if (!store->is_track_valid(tid) || !store->track_has_triangulated_xyz(tid))
      continue;
    float tx, ty, tz;
    store->get_track_xyz(tid, &tx, &ty, &tz);
    const Eigen::Vector3d X(static_cast<double>(tx), static_cast<double>(ty),
                            static_cast<double>(tz));
    const Eigen::Matrix3d& R = poses_R[static_cast<size_t>(im)];
    const Eigen::Vector3d& C = poses_C[static_cast<size_t>(im)];
    Eigen::Vector3d p = R * (X - C);
    if (p(2) <= 1e-12) {
      store->mark_observation_deleted(obs_id);
      ++marked;
      continue;
    }
    const camera::Intrinsics& K =
        cameras[static_cast<size_t>(image_to_camera_index[static_cast<size_t>(im)])];
    const double xn = p(0) / p(2), yn = p(1) / p(2);
    double xd, yd;
    camera::apply_distortion(xn, yn, K, &xd, &yd);
    const double u_pred = K.fx * xd + K.cx;
    const double v_pred = K.fy * yd + K.cy;
    const double u_obs = static_cast<double>(obs.u);
    const double v_obs = static_cast<double>(obs.v);
    if ((u_obs - u_pred) * (u_obs - u_pred) + (v_obs - v_pred) * (v_obs - v_pred) > thresh_sq) {
      store->mark_observation_deleted(obs_id);
      ++marked;
    }
  }
  return marked;
}

/// Mark observations whose max parallax angle (with any other view in the same track) is <
/// min_angle_deg.
int reject_outliers_angle_multiview(TrackStore* store, const std::vector<Eigen::Matrix3d>& poses_R,
                                    const std::vector<Eigen::Vector3d>& poses_C,
                                    const std::vector<bool>& registered, double min_angle_deg) {
  if (!store || min_angle_deg <= 0)
    return 0;
  const int n_images = store->num_images();
  if (static_cast<int>(poses_R.size()) != n_images ||
      static_cast<int>(poses_C.size()) != n_images ||
      static_cast<int>(registered.size()) != n_images)
    return 0;
  const double min_angle_rad = min_angle_deg * (3.141592653589793 / 180.0);
  int marked = 0;
  std::vector<Observation> obs_buf;
  for (size_t i = 0; i < store->num_observations(); ++i) {
    const int obs_id = static_cast<int>(i);
    if (!store->is_obs_valid(obs_id))
      continue;
    Observation obs;
    store->get_obs(obs_id, &obs);
    const int im = static_cast<int>(obs.image_index);
    if (im < 0 || im >= n_images || !registered[static_cast<size_t>(im)])
      continue;
    const int tid = store->obs_track_id(obs_id);
    if (!store->is_track_valid(tid) || !store->track_has_triangulated_xyz(tid))
      continue;
    float tx, ty, tz;
    store->get_track_xyz(tid, &tx, &ty, &tz);
    const Eigen::Vector3d X(static_cast<double>(tx), static_cast<double>(ty),
                            static_cast<double>(tz));
    const Eigen::Vector3d C_this = poses_C[static_cast<size_t>(im)];
    const Eigen::Vector3d ray_this = (X - C_this).normalized();
    double max_angle = 0.0;
    obs_buf.clear();
    store->get_track_observations(tid, &obs_buf);
    for (const auto& o : obs_buf) {
      const int im_other = static_cast<int>(o.image_index);
      if (im_other == im || im_other < 0 || im_other >= n_images ||
          !registered[static_cast<size_t>(im_other)])
        continue;
      const Eigen::Vector3d C_other = poses_C[static_cast<size_t>(im_other)];
      const Eigen::Vector3d ray_other = (X - C_other).normalized();
      double cos_a = std::max(-1.0, std::min(1.0, ray_this.dot(ray_other)));
      double angle = std::acos(cos_a);
      if (angle > max_angle)
        max_angle = angle;
    }
    if (max_angle < min_angle_rad) {
      store->mark_observation_deleted(obs_id);
      ++marked;
    }
  }
  return marked;
}

/// Select active and fixed camera indices for local BA.
///
/// ACTIVE cameras (active_out): new_registered images + the most-connected neighbours, up to
/// local_ba_window total.  Their poses will be optimised.
///
/// FIXED cameras (fixed_out): the next local_ba_fixed_window cameras by shared-track count.
/// Their poses are held constant (constraint-only), allowing the BA to see cross-window structure
/// without the cost of optimising them.
///
/// If total registered <= local_ba_window, all registered images go into active_out and
/// fixed_out is empty.
void choose_local_ba_indices_by_connectivity(const TrackStore& store,
                                             const std::vector<bool>& registered,
                                             const std::vector<int>& new_registered_indices,
                                             int local_ba_window, int local_ba_fixed_window,
                                             std::vector<int>* active_out,
                                             std::vector<int>* fixed_out) {
  if (active_out)
    active_out->clear();
  if (fixed_out)
    fixed_out->clear();
  const int n_images = store.num_images();
  if (n_images == 0 || static_cast<int>(registered.size()) != n_images || local_ba_window <= 0)
    return;

  // Seed the active set with the newly registered images
  std::set<int> active_set;
  for (int im : new_registered_indices)
    if (im >= 0 && im < n_images && registered[static_cast<size_t>(im)])
      active_set.insert(im);

  std::vector<int> reg_list;
  for (int i = 0; i < n_images; ++i)
    if (registered[static_cast<size_t>(i)])
      reg_list.push_back(i);

  // If few enough cameras registered, all go into active
  if (static_cast<int>(reg_list.size()) <= local_ba_window) {
    if (active_out)
      *active_out = reg_list;
    return;
  }

  // Compute shared triangulated-track count between each candidate and the current active set
  std::unordered_map<int, int> shared_count;
  std::vector<Observation> obs_buf_inner;
  for (int im : reg_list) {
    if (active_set.count(im))
      continue;
    std::vector<int> track_ids;
    store.get_image_track_observations(im, &track_ids, nullptr);
    int count = 0;
    for (int tid : track_ids) {
      if (!store.track_has_triangulated_xyz(tid))
        continue;
      obs_buf_inner.clear();
      store.get_track_observations(tid, &obs_buf_inner);
      for (const auto& o : obs_buf_inner) {
        if (active_set.count(static_cast<int>(o.image_index))) {
          ++count;
          break;
        }
      }
    }
    shared_count[im] = count;
  }

  // Sort remaining cameras by descending connectivity score
  std::vector<std::pair<int, int>> by_count(shared_count.begin(), shared_count.end());
  std::sort(by_count.begin(), by_count.end(),
            [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
              return a.second > b.second;
            });

  // Fill ACTIVE up to local_ba_window
  size_t rank = 0;
  while (static_cast<int>(active_set.size()) < local_ba_window && rank < by_count.size()) {
    active_set.insert(by_count[rank].first);
    ++rank;
  }

  // Fill FIXED with the next local_ba_fixed_window cameras (if requested)
  std::vector<int> fixed_vec;
  if (local_ba_fixed_window > 0) {
    while (static_cast<int>(fixed_vec.size()) < local_ba_fixed_window && rank < by_count.size()) {
      fixed_vec.push_back(by_count[rank].first);
      ++rank;
    }
  }

  if (active_out)
    *active_out = std::vector<int>(active_set.begin(), active_set.end());
  if (fixed_out)
    *fixed_out = fixed_vec;
}

int filter_tracks_two_view(TrackStore* store, int im0, int im1, const Eigen::Vector3d& C,
                           int min_observations, double min_angle_deg) {
  if (!store)
    return 0;
  const uint32_t uim0 = static_cast<uint32_t>(im0), uim1 = static_cast<uint32_t>(im1);
  const double min_angle_rad = min_angle_deg * (3.141592653589793 / 180.0);
  int marked = 0;
  std::vector<Observation> obs_buf;
  for (size_t ti = 0; ti < store->num_tracks(); ++ti) {
    const int track_id = static_cast<int>(ti);
    if (!store->is_track_valid(track_id))
      continue;
    obs_buf.clear();
    store->get_track_observations(track_id, &obs_buf);
    int n_in_pair = 0;
    for (const auto& o : obs_buf) {
      if (o.image_index == uim0 || o.image_index == uim1)
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
      Eigen::Vector3d r1 = (X - C).normalized();
      double cos_a = std::max(-1.0, std::min(1.0, r0.dot(r1)));
      if (std::acos(cos_a) < min_angle_rad) {
        store->mark_track_deleted(track_id);
        ++marked;
      }
    }
  }
  return marked;
}

int count_two_view_valid_tracks(const TrackStore& store, int im0, int im1) {
  const uint32_t uim0 = static_cast<uint32_t>(im0), uim1 = static_cast<uint32_t>(im1);
  int n = 0;
  std::vector<Observation> obs_buf;
  for (size_t ti = 0; ti < store.num_tracks(); ++ti) {
    const int tid = static_cast<int>(ti);
    if (!store.is_track_valid(tid))
      continue;
    obs_buf.clear();
    store.get_track_observations(tid, &obs_buf);
    bool has0 = false, has1 = false;
    for (const auto& o : obs_buf) {
      if (o.image_index == uim0)
        has0 = true;
      if (o.image_index == uim1)
        has1 = true;
    }
    if (has0 && has1)
      ++n;
  }
  return n;
}

// ─────────────────────────────────────────────────────────────────────────────
// COLMAP-style initial pair selection helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Load geo for pair (im0, im1); geo files are always stored as min_max.isat_geo.
/// If im0 > im1 the stored pose is inverted so that im0 is always the reference camera.
bool load_pair_geo_flexible(const std::string& dir, int im0, int im1, Eigen::Matrix3d* R,
                            Eigen::Vector3d* t) {
  const int lo = std::min(im0, im1);
  const int hi = std::max(im0, im1);
  const std::string path = dir + std::to_string(lo) + "_" + std::to_string(hi) + ".isat_geo";
  Eigen::Matrix3d R_file;
  Eigen::Vector3d t_file;
  if (!load_pair_geo(path, &R_file, &t_file))
    return false;
  if (im0 == lo) {
    // File stores lo→hi; caller wants im0→im1, same direction
    *R = R_file;
    *t = t_file;
  } else {
    // File stores lo→hi (= im1→im0); invert so im0 is reference
    *R = R_file.transpose();
    *t = -(R_file.transpose() * t_file);
  }
  return true;
}

/// Rank all images by total correspondence count (sum over tracks of (track_len - 1)).
/// More correspondences = better hub image for initialization.
/// Returns {image_index, correspondence_count} sorted descending.
std::vector<std::pair<int, int>> find_initial_first_images(const TrackStore& store) {
  const int n = store.num_images();
  std::vector<std::pair<int, int>> result;
  result.reserve(static_cast<size_t>(n));
  for (int im = 0; im < n; ++im) {
    std::vector<int> track_ids;
    store.get_image_track_observations(im, &track_ids, nullptr);
    int corr_count = 0;
    for (int tid : track_ids) {
      if (!store.is_track_valid(tid))
        continue;
      std::vector<Observation> obs;
      store.get_track_observations(tid, &obs);
      corr_count += std::max(0, static_cast<int>(obs.size()) - 1);
    }
    if (corr_count > 0)
      result.push_back({im, corr_count});
  }
  std::sort(result.begin(), result.end(),
            [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
              return a.second > b.second;
            });
  return result;
}

/// For a given first image, find connected second images via shared tracks.
/// Returns {image_index, shared_track_count} sorted descending, filtered by min_shared.
std::vector<std::pair<int, int>> find_initial_second_images(const TrackStore& store, int im_first,
                                                            int min_shared) {
  std::unordered_map<int, int> shared_count;
  std::vector<int> track_ids;
  store.get_image_track_observations(im_first, &track_ids, nullptr);
  for (int tid : track_ids) {
    if (!store.is_track_valid(tid))
      continue;
    std::vector<Observation> obs;
    store.get_track_observations(tid, &obs);
    for (const auto& o : obs) {
      int im2 = static_cast<int>(o.image_index);
      if (im2 != im_first)
        shared_count[im2]++;
    }
  }
  std::vector<std::pair<int, int>> result;
  for (const auto& p : shared_count) {
    if (p.second >= min_shared)
      result.push_back(p);
  }
  std::sort(result.begin(), result.end(),
            [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
              return a.second > b.second;
            });
  return result;
}

/// Trial result for an initial pair candidate (no store mutation).
struct InitPairTrialResult {
  bool success = false;
  Eigen::Matrix3d R1 = Eigen::Matrix3d::Identity();
  Eigen::Vector3d C1 = Eigen::Vector3d::Zero();
  double rmse_px = 1e9;
  int n_inlier_tracks = 0;
  int n_triangulated = 0;
  std::vector<std::pair<int, Eigen::Vector3d>> track_xyz; ///< track_id -> refined XYZ
};

/// Try an initial pair WITHOUT mutating the store. Triangulates into a local buffer,
/// runs BA locally, checks quality (reprojection + angle + RMSE). Returns trial result.
InitPairTrialResult try_initial_pair_candidate(const TrackStore& store, int im0, int im1,
                                               const std::string& dir,
                                               const std::vector<camera::Intrinsics>& cameras,
                                               const std::vector<int>& image_to_camera_index,
                                               int min_tracks_after, double ba_rmse_max,
                                               double outlier_thresh_px, double min_angle_deg) {
  InitPairTrialResult result;

  // 1. Load geo (try both orderings)
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  if (!load_pair_geo_flexible(dir, im0, im1, &R, &t)) {
    LOG(INFO) << "    pair (" << im0 << "," << im1 << "): no geo file found";
    return result;
  }
  Eigen::Vector3d C1 = -R.transpose() * t;

  // 2. Triangulate shared tracks into LOCAL buffer (const store, no mutation)
  const camera::Intrinsics& K0 = cameras[static_cast<size_t>(image_to_camera_index[im0])];
  const camera::Intrinsics& K1 = cameras[static_cast<size_t>(image_to_camera_index[im1])];
  const uint32_t uim0 = static_cast<uint32_t>(im0);
  const uint32_t uim1 = static_cast<uint32_t>(im1);

  std::vector<int> track_ids;
  std::vector<Eigen::Vector3d> points3d;
  std::vector<Observation> obs_buf;

  for (size_t ti = 0; ti < store.num_tracks(); ++ti) {
    const int tid = static_cast<int>(ti);
    if (!store.is_track_valid(tid))
      continue;
    obs_buf.clear();
    store.get_track_observations(tid, &obs_buf);
    double u0 = 0, v0 = 0, u1 = 0, v1 = 0;
    bool has0 = false, has1 = false;
    for (const auto& o : obs_buf) {
      if (o.image_index == uim0) {
        u0 = static_cast<double>(o.u);
        v0 = static_cast<double>(o.v);
        has0 = true;
      }
      if (o.image_index == uim1) {
        u1 = static_cast<double>(o.u);
        v1 = static_cast<double>(o.v);
        has1 = true;
      }
    }
    if (!has0 || !has1)
      continue;
    if (K0.has_distortion())
      camera::undistort_point(K0, u0, v0, &u0, &v0);
    if (K1.has_distortion())
      camera::undistort_point(K1, u1, v1, &u1, &v1);
    Eigen::Vector2d n0((u0 - K0.cx) / K0.fx, (v0 - K0.cy) / K0.fy);
    Eigen::Vector2d n1((u1 - K1.cx) / K1.fx, (v1 - K1.cy) / K1.fy);
    Eigen::Vector3d t_cam = -R * C1;
    Eigen::Vector3d X = triangulate_point(n0, n1, R, t_cam);
    if (X(2) <= 1e-9)
      continue;
    track_ids.push_back(tid);
    points3d.push_back(X);
  }
  result.n_triangulated = static_cast<int>(track_ids.size());

  if (track_ids.size() < 8u) {
    LOG(INFO) << "    pair (" << im0 << "," << im1 << "): only " << track_ids.size()
              << " triangulated tracks (need >=8)";
    return result;
  }

  // 3. Two-view BA with local data (no store mutation)
  int c0 = image_to_camera_index[im0], c1 = image_to_camera_index[im1];
  std::vector<camera::Intrinsics> ba_cameras;
  std::vector<int> ba_image_camera_index(2);
  if (c0 == c1) {
    ba_cameras.push_back(cameras[static_cast<size_t>(c0)]);
    ba_image_camera_index[0] = ba_image_camera_index[1] = 0;
  } else {
    ba_cameras.push_back(cameras[static_cast<size_t>(c0)]);
    ba_cameras.push_back(cameras[static_cast<size_t>(c1)]);
    ba_image_camera_index[0] = 0;
    ba_image_camera_index[1] = 1;
  }

  BAInput ba_in;
  ba_in.poses_R = {Eigen::Matrix3d::Identity(), R};
  ba_in.poses_C = {Eigen::Vector3d::Zero(), C1};
  ba_in.image_camera_index = ba_image_camera_index;
  ba_in.cameras = ba_cameras;
  ba_in.optimize_intrinsics = false;
  ba_in.fix_pose = {true, false};
  ba_in.fix_point.resize(track_ids.size(), false);
  ba_in.points3d = points3d;

  for (size_t i = 0; i < track_ids.size(); ++i) {
    obs_buf.clear();
    store.get_track_observations(track_ids[i], &obs_buf);
    const int pt_idx = static_cast<int>(i);
    for (const auto& o : obs_buf) {
      if (o.image_index == uim0)
        ba_in.observations.push_back({0, pt_idx, static_cast<double>(o.u), static_cast<double>(o.v),
                                      static_cast<double>(o.scale > 1e-6f ? o.scale : 1.0f)});
      else if (o.image_index == uim1)
        ba_in.observations.push_back({1, pt_idx, static_cast<double>(o.u), static_cast<double>(o.v),
                                      static_cast<double>(o.scale > 1e-6f ? o.scale : 1.0f)});
    }
  }

  BAResult ba_out;
  if (!global_bundle_analytic(ba_in, &ba_out, 50)) {
    LOG(INFO) << "    pair (" << im0 << "," << im1 << "): BA failed";
    return result;
  }
  result.rmse_px = ba_out.rmse_px;
  R = ba_out.poses_R[1];
  C1 = ba_out.poses_C[1];

  // 4. Soft quality check: count inlier tracks WITHOUT modifying store
  const double thresh_sq = outlier_thresh_px * outlier_thresh_px;
  const double min_angle_rad = min_angle_deg * (3.141592653589793 / 180.0);
  int n_inlier = 0;

  for (size_t i = 0; i < ba_out.points3d.size() && i < track_ids.size(); ++i) {
    const Eigen::Vector3d& X = ba_out.points3d[i];
    // Cheirality check (both cameras)
    if (X(2) <= 1e-9)
      continue;
    Eigen::Vector3d p1 = R * (X - C1);
    if (p1(2) <= 1e-9)
      continue;
    // Triangulation angle check
    Eigen::Vector3d r0 = X.normalized();
    Eigen::Vector3d r1 = (X - C1).normalized();
    double cos_a = std::max(-1.0, std::min(1.0, r0.dot(r1)));
    if (std::acos(cos_a) < min_angle_rad)
      continue;
    // Reprojection error check for both views (forward model: apply distortion to prediction)
    double xn0 = X(0) / X(2), yn0 = X(1) / X(2);
    double xd0, yd0;
    camera::apply_distortion(xn0, yn0, K0, &xd0, &yd0);
    double u0_pred = K0.fx * xd0 + K0.cx;
    double v0_pred = K0.fy * yd0 + K0.cy;
    double xn1 = p1(0) / p1(2), yn1 = p1(1) / p1(2);
    double xd1, yd1;
    camera::apply_distortion(xn1, yn1, K1, &xd1, &yd1);
    double u1_pred = K1.fx * xd1 + K1.cx;
    double v1_pred = K1.fy * yd1 + K1.cy;
    obs_buf.clear();
    store.get_track_observations(track_ids[i], &obs_buf);
    bool ok = true;
    for (const auto& o : obs_buf) {
      if (o.image_index != uim0 && o.image_index != uim1)
        continue;
      const double u_obs = static_cast<double>(o.u);
      const double v_obs = static_cast<double>(o.v);
      double u_pred = (o.image_index == uim0) ? u0_pred : u1_pred;
      double v_pred = (o.image_index == uim0) ? v0_pred : v1_pred;
      if ((u_obs - u_pred) * (u_obs - u_pred) + (v_obs - v_pred) * (v_obs - v_pred) > thresh_sq) {
        ok = false;
        break;
      }
    }
    if (ok)
      ++n_inlier;
  }
  result.n_inlier_tracks = n_inlier;

  LOG(INFO) << "    pair (" << im0 << "," << im1 << "): tri=" << track_ids.size()
            << " inlier=" << n_inlier << "/" << track_ids.size() << " RMSE=" << ba_out.rmse_px
            << " px";

  if (n_inlier < min_tracks_after) {
    LOG(INFO) << "    pair (" << im0 << "," << im1 << "): REJECTED (need " << min_tracks_after
              << " inlier tracks, got " << n_inlier << ")";
    return result;
  }
  if (ba_out.rmse_px > ba_rmse_max) {
    LOG(INFO) << "    pair (" << im0 << "," << im1 << "): REJECTED (RMSE " << ba_out.rmse_px
              << " > " << ba_rmse_max << ")";
    return result;
  }

  // 5. OK — package result (still no store mutation)
  result.success = true;
  result.R1 = R;
  result.C1 = C1;
  for (size_t i = 0; i < ba_out.points3d.size() && i < track_ids.size(); ++i) {
    result.track_xyz.push_back({track_ids[i], ba_out.points3d[i]});
  }
  return result;
}

} // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Initial pair selection: COLMAP-style two-level search from TrackStore
// ─────────────────────────────────────────────────────────────────────────────
bool run_initial_pair_loop(const ViewGraph& view_graph, const std::string& geo_dir,
                           TrackStore* store, const std::vector<camera::Intrinsics>& cameras,
                           const std::vector<int>& image_to_camera_index, int min_tracks_after,
                           uint32_t* initial_im0_out, uint32_t* initial_im1_out,
                           std::vector<Eigen::Matrix3d>* poses_R_out,
                           std::vector<Eigen::Vector3d>* poses_C_out,
                           std::vector<bool>* registered_out) {
  if (!store || !poses_R_out || !poses_C_out || !registered_out || cameras.empty())
    return false;
  const int n_images = store->num_images();
  std::string dir = geo_dir;
  if (!dir.empty() && dir.back() != '/')
    dir += '/';

  LOG(INFO) << "================================================================";
  LOG(INFO) << "Initial pair selection (COLMAP-style, track-based)";
  LOG(INFO) << "  n_images=" << n_images << ", n_tracks=" << store->num_tracks()
            << ", n_obs=" << store->num_observations();
  LOG(INFO) << "  min_tracks_after=" << min_tracks_after;
  LOG(INFO) << "  ViewGraph pairs (informational): " << view_graph.num_pairs();

  // Step 1: Rank first images by total correspondence count
  auto first_images = find_initial_first_images(*store);
  LOG(INFO) << "  First-image candidates: " << first_images.size();
  for (size_t i = 0; i < std::min(first_images.size(), size_t(10)); ++i) {
    LOG(INFO) << "    #" << i << ": image " << first_images[i].first
              << " (corr_count=" << first_images[i].second << ")";
  }
  if (first_images.empty()) {
    LOG(ERROR) << "  No images with correspondences found!";
    return false;
  }

  std::set<std::pair<int, int>> tried_pairs;
  const size_t max_first = std::min(first_images.size(), size_t(100));

  for (size_t fi = 0; fi < max_first; ++fi) {
    const int im1 = first_images[fi].first;
    auto second_images = find_initial_second_images(*store, im1, std::min(30, min_tracks_after));
    if (second_images.empty())
      continue;

    LOG(INFO) << "  --- Trying first image " << im1 << " (corr=" << first_images[fi].second << "), "
              << second_images.size() << " second candidates ---";
    if (second_images.size() <= 5u) {
      for (size_t si = 0; si < second_images.size(); ++si)
        LOG(INFO) << "    second #" << si << ": image " << second_images[si].first
                  << " (shared=" << second_images[si].second << ")";
    } else {
      for (size_t si = 0; si < 3; ++si)
        LOG(INFO) << "    second #" << si << ": image " << second_images[si].first
                  << " (shared=" << second_images[si].second << ")";
      LOG(INFO) << "    ... (" << second_images.size() - 3 << " more)";
    }

    const size_t max_second = std::min(second_images.size(), size_t(50));
    for (size_t si = 0; si < max_second; ++si) {
      const int im2 = second_images[si].first;
      auto pair_key = std::make_pair(std::min(im1, im2), std::max(im1, im2));
      if (tried_pairs.count(pair_key))
        continue;
      tried_pairs.insert(pair_key);

      // Try this pair (NO store mutation)
      auto trial = try_initial_pair_candidate(*store, im1, im2, dir, cameras, image_to_camera_index,
                                              min_tracks_after, 10.0, 4.0, 2.0);
      if (!trial.success)
        continue;

      // ── Pair accepted! Now commit to store ──
      LOG(INFO) << "  >> ACCEPTED pair (" << im1 << ", " << im2 << "): " << trial.n_inlier_tracks
                << " inlier tracks, RMSE=" << trial.rmse_px << " px";

      // Write refined XYZ to store
      for (const auto& p : trial.track_xyz) {
        store->set_track_xyz(p.first, static_cast<float>(p.second(0)),
                             static_cast<float>(p.second(1)), static_cast<float>(p.second(2)));
      }
      // Outlier rejection (safe now — pair is confirmed)
      // Note: we do NOT call filter_tracks_two_view here — that would delete all tracks not
      // visible in the initial pair, leaving no tracks for future cameras to triangulate.
      // Only reject individual outlier observations in the initial pair.
      int rej_obs = reject_outliers_two_view(store, trial.R1, trial.C1, im1, im2, cameras,
                                             image_to_camera_index, 4.0);
      int n_valid = count_two_view_valid_tracks(*store, im1, im2);

      LOG(INFO) << "  After commit: valid_tracks=" << n_valid << ", rejected_obs=" << rej_obs;

      if (initial_im0_out)
        *initial_im0_out = static_cast<uint32_t>(im1);
      if (initial_im1_out)
        *initial_im1_out = static_cast<uint32_t>(im2);
      poses_R_out->resize(static_cast<size_t>(n_images));
      poses_C_out->resize(static_cast<size_t>(n_images));
      registered_out->resize(static_cast<size_t>(n_images), false);
      (*poses_R_out)[static_cast<size_t>(im1)] = Eigen::Matrix3d::Identity();
      (*poses_C_out)[static_cast<size_t>(im1)] = Eigen::Vector3d::Zero();
      (*poses_R_out)[static_cast<size_t>(im2)] = trial.R1;
      (*poses_C_out)[static_cast<size_t>(im2)] = trial.C1;
      (*registered_out)[static_cast<size_t>(im1)] = true;
      (*registered_out)[static_cast<size_t>(im2)] = true;
      LOG(INFO) << "================================================================";
      return true;
    }
  }

  LOG(ERROR) << "  No initial pair found after trying " << tried_pairs.size() << " pairs";
  LOG(INFO) << "================================================================";
  return false;
}

// ─── Multiview DLT helper (N views) ─────────────────────────────────────────
namespace {
Eigen::Vector3d triangulate_point_multiview(const std::vector<Eigen::Matrix3d>& R_list,
                                            const std::vector<Eigen::Vector3d>& C_list,
                                            const std::vector<Eigen::Vector2d>& rays_n) {
  const int N = static_cast<int>(R_list.size());
  if (N < 2 || static_cast<int>(C_list.size()) != N || static_cast<int>(rays_n.size()) != N)
    return Eigen::Vector3d(0, 0, 0);
  Eigen::MatrixXd A(2 * N, 4);
  for (int i = 0; i < N; ++i) {
    const Eigen::Matrix3d& R = R_list[i];
    const Eigen::Vector3d t = -R * C_list[i];
    const double nx = rays_n[i](0), ny = rays_n[i](1);
    // Standard Hartley DLT: row 2i = y-constraint (ny·r₃ - r₂), row 2i+1 = x-constraint (nx·r₃ - r₁).
    // Previous code used the cross-product row (nx·r₂ - ny·r₁) which is degenerate when nx≈0
    // (features near the vertical image centre) because it collapses to a scalar multiple of row 2i+1.
    A.row(2 * i)     << ny * R.row(2) - R.row(1), ny * t(2) - t(1);
    A.row(2 * i + 1) << nx * R.row(2) - R.row(0), nx * t(2) - t(0);
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
  const Eigen::Vector4d v = svd.matrixV().col(3);
  if (std::fabs(v(3)) < 1e-12)
    return Eigen::Vector3d(0, 0, 0);
  return v.head<3>() / v(3);
}

// Check cheirality: X must have positive depth (positive camera-frame Z) in ALL cameras
bool check_all_depths_positive(const Eigen::Vector3d& X, const std::vector<Eigen::Matrix3d>& R_list,
                               const std::vector<Eigen::Vector3d>& C_list) {
  for (size_t i = 0; i < R_list.size(); ++i) {
    const Eigen::Vector3d Xc = R_list[i] * (X - C_list[i]);
    if (Xc(2) <= 0.0)
      return false;
  }
  return true;
}

// Compute maximum pairwise ray angle (degrees) between cameras looking at X.
// A large angle means good triangulation geometry (high parallax).
double compute_max_ray_angle_deg(const Eigen::Vector3d& X,
                                 const std::vector<Eigen::Vector3d>& C_list) {
  double max_angle = 0.0;
  const int N = static_cast<int>(C_list.size());
  for (int i = 0; i < N; ++i) {
    const Eigen::Vector3d ri = (X - C_list[i]).normalized();
    for (int j = i + 1; j < N; ++j) {
      const Eigen::Vector3d rj = (X - C_list[j]).normalized();
      double d = ri.dot(rj);
      d = d < -1.0 ? -1.0 : (d > 1.0 ? 1.0 : d);
      const double angle_deg = std::acos(d) * (180.0 / M_PI);
      if (angle_deg > max_angle)
        max_angle = angle_deg;
    }
  }
  return max_angle;
}

} // namespace

std::vector<int> choose_next_resection_batch(const TrackStore& store,
                                             const std::vector<bool>& registered,
                                             int batch_size_k) {
  const int n_images = store.num_images();
  if (n_images == 0 || static_cast<int>(registered.size()) != n_images || batch_size_k <= 0)
    return {};
  std::vector<std::pair<int, int>> unreg_count;
  for (int im = 0; im < n_images; ++im) {
    if (registered[static_cast<size_t>(im)])
      continue;
    std::vector<int> track_ids;
    store.get_image_track_observations(im, &track_ids, nullptr);
    int count = 0;
    for (int tid : track_ids) {
      if (store.track_has_triangulated_xyz(tid))
        ++count;
    }
    unreg_count.push_back({im, count});
  }
  std::sort(unreg_count.begin(), unreg_count.end(),
            [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
              return a.second > b.second;
            });
  std::vector<int> out;
  for (size_t i = 0; i < unreg_count.size() && static_cast<int>(i) < batch_size_k; ++i)
    out.push_back(unreg_count[i].first);
  // Logging: show top candidates and selection
  if (!unreg_count.empty()) {
    LOG(INFO) << "choose_next_resection_batch: " << unreg_count.size()
              << " unregistered images, selecting " << out.size();
    for (size_t i = 0; i < std::min(unreg_count.size(), size_t(5)); ++i) {
      LOG(INFO) << "  candidate #" << i << ": image " << unreg_count[i].first << " ("
                << unreg_count[i].second << " 3D-2D correspondences)";
    }
  } else {
    LOG(INFO) << "choose_next_resection_batch: no unregistered images with 3D-2D correspondences";
  }
  return out;
}

int run_batch_resection(const TrackStore& store, const std::vector<int>& image_indices,
                        const std::vector<camera::Intrinsics>& cameras,
                        const std::vector<int>& image_to_camera_index,
                        std::vector<Eigen::Matrix3d>* poses_R,
                        std::vector<Eigen::Vector3d>* poses_C, std::vector<bool>* registered,
                        int min_inliers) {
  if (!poses_R || !poses_C || !registered)
    return 0;
  int added = 0;
  for (int im : image_indices) {
    if (im < 0 || static_cast<size_t>(im) >= image_to_camera_index.size())
      continue;
    const camera::Intrinsics& K = cameras[static_cast<size_t>(image_to_camera_index[im])];
    // Count available 3D-2D correspondences for this image
    std::vector<int> track_ids_for_log;
    store.get_image_track_observations(im, &track_ids_for_log, nullptr);
    int n_3d2d = 0;
    for (int tid : track_ids_for_log) {
      if (store.track_has_triangulated_xyz(tid))
        ++n_3d2d;
    }
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    int inliers = 0;
    if (!resection_single_image(K, store, im, &R, &t, min_inliers, 8.0, &inliers)) {
      LOG(INFO) << "  resection image " << im << ": FAILED (3D-2D=" << n_3d2d
                << ", inliers=" << inliers << ", need " << min_inliers << ")";
      continue;
    }
    Eigen::Vector3d C = -R.transpose() * t;
    LOG(INFO) << "  resection image " << im << ": OK (3D-2D=" << n_3d2d << ", inliers=" << inliers
              << ", C=[" << C(0) << "," << C(1) << "," << C(2) << "])";
    if (static_cast<size_t>(im) >= poses_R->size())
      poses_R->resize(static_cast<size_t>(im) + 1);
    if (static_cast<size_t>(im) >= poses_C->size())
      poses_C->resize(static_cast<size_t>(im) + 1);
    if (static_cast<size_t>(im) >= registered->size())
      registered->resize(static_cast<size_t>(im) + 1);
    (*poses_R)[static_cast<size_t>(im)] = R;
    (*poses_C)[static_cast<size_t>(im)] = C;
    (*registered)[static_cast<size_t>(im)] = true;
    ++added;
  }
  LOG(INFO) << "run_batch_resection: " << added << "/" << image_indices.size()
            << " images registered";
  return added;
}

int run_batch_triangulation(TrackStore* store, const std::vector<int>& new_registered_image_indices,
                            const std::vector<Eigen::Matrix3d>& poses_R,
                            const std::vector<Eigen::Vector3d>& poses_C,
                            const std::vector<bool>& registered,
                            const std::vector<camera::Intrinsics>& cameras,
                            const std::vector<int>& image_to_camera_index,
                            double min_tri_angle_deg) {
  if (!store)
    return 0;
  const int n_images = store->num_images();
  if (static_cast<int>(poses_R.size()) != n_images ||
      static_cast<int>(poses_C.size()) != n_images ||
      static_cast<int>(registered.size()) != n_images)
    return 0;
  std::set<int> new_set(new_registered_image_indices.begin(), new_registered_image_indices.end());
  int updated = 0;
  std::vector<Observation> obs_buf;
  for (size_t ti = 0; ti < store->num_tracks(); ++ti) {
    const int track_id = static_cast<int>(ti);
    if (!store->is_track_valid(track_id) || store->track_has_triangulated_xyz(track_id))
      continue;
    obs_buf.clear();
    store->get_track_observations(track_id, &obs_buf);
    bool has_new = false;
    std::vector<int> reg_inds;
    std::vector<Eigen::Vector2d> rays_n;
    for (const Observation& o : obs_buf) {
      const int im = static_cast<int>(o.image_index);
      if (new_set.count(im))
        has_new = true;
      if (im < 0 || im >= n_images || !registered[static_cast<size_t>(im)])
        continue;
      double u = static_cast<double>(o.u), v = static_cast<double>(o.v);
      const camera::Intrinsics& K = cameras[static_cast<size_t>(image_to_camera_index[im])];
      if (K.has_distortion())
        camera::undistort_point(K, u, v, &u, &v);
      rays_n.push_back(Eigen::Vector2d((u - K.cx) / K.fx, (v - K.cy) / K.fy));
      reg_inds.push_back(im);
    }
    if (!has_new || static_cast<int>(reg_inds.size()) < 2)
      continue;
    std::vector<Eigen::Matrix3d> R_list;
    std::vector<Eigen::Vector3d> C_list;
    for (int im : reg_inds) {
      R_list.push_back(poses_R[static_cast<size_t>(im)]);
      C_list.push_back(poses_C[static_cast<size_t>(im)]);
    }
    Eigen::Vector3d X = triangulate_point_multiview(R_list, C_list, rays_n);
    // Cheirality: positive depth in every observing camera
    if (!check_all_depths_positive(X, R_list, C_list))
      continue;
    // Angle filter: reject degenerate (near-zero parallax) triangulations
    if (compute_max_ray_angle_deg(X, C_list) < min_tri_angle_deg)
      continue;
    store->set_track_xyz(track_id, static_cast<float>(X(0)), static_cast<float>(X(1)),
                         static_cast<float>(X(2)));
    ++updated;
  }
  LOG(INFO) << "run_batch_triangulation: " << updated << " tracks newly triangulated";
  return updated;
}

int run_retriangulation(TrackStore* store, const std::vector<Eigen::Matrix3d>& poses_R,
                        const std::vector<Eigen::Vector3d>& poses_C,
                        const std::vector<bool>& registered,
                        const std::vector<camera::Intrinsics>& cameras,
                        const std::vector<int>& image_to_camera_index, double min_tri_angle_deg) {
  if (!store)
    return 0;
  const int n_images = store->num_images();
  if (static_cast<int>(poses_R.size()) != n_images ||
      static_cast<int>(poses_C.size()) != n_images ||
      static_cast<int>(registered.size()) != n_images)
    return 0;
  int updated = 0;
  std::vector<Observation> obs_buf;
  for (size_t ti = 0; ti < store->num_tracks(); ++ti) {
    const int track_id = static_cast<int>(ti);
    if (!store->is_track_valid(track_id) || !store->track_needs_retriangulation(track_id))
      continue;
    obs_buf.clear();
    store->get_track_observations(track_id, &obs_buf);
    std::vector<int> reg_inds;
    std::vector<Eigen::Vector2d> rays_n;
    for (const Observation& o : obs_buf) {
      const int im = static_cast<int>(o.image_index);
      if (im < 0 || im >= n_images || !registered[static_cast<size_t>(im)])
        continue;
      double u = static_cast<double>(o.u), v = static_cast<double>(o.v);
      const camera::Intrinsics& K = cameras[static_cast<size_t>(image_to_camera_index[im])];
      if (K.has_distortion())
        camera::undistort_point(K, u, v, &u, &v);
      rays_n.push_back(Eigen::Vector2d((u - K.cx) / K.fx, (v - K.cy) / K.fy));
      reg_inds.push_back(im);
    }
    if (static_cast<int>(reg_inds.size()) < 2) {
      store->set_track_retriangulation_flag(track_id, false);
      continue;
    }
    std::vector<Eigen::Matrix3d> R_list;
    std::vector<Eigen::Vector3d> C_list;
    for (int im : reg_inds) {
      R_list.push_back(poses_R[static_cast<size_t>(im)]);
      C_list.push_back(poses_C[static_cast<size_t>(im)]);
    }
    Eigen::Vector3d X = triangulate_point_multiview(R_list, C_list, rays_n);
    // Cheirality: positive depth in every observing camera
    if (!check_all_depths_positive(X, R_list, C_list)) {
      store->set_track_retriangulation_flag(track_id, false);
      continue;
    }
    // Angle filter: reject degenerate (near-zero parallax) triangulations
    if (compute_max_ray_angle_deg(X, C_list) < min_tri_angle_deg) {
      store->set_track_retriangulation_flag(track_id, false);
      continue;
    }
    store->set_track_xyz(track_id, static_cast<float>(X(0)), static_cast<float>(X(1)),
                         static_cast<float>(X(2)));
    store->set_track_retriangulation_flag(track_id, false);
    ++updated;
  }
  LOG(INFO) << "run_retriangulation: " << updated << " tracks re-triangulated";
  return updated;
}

// ─── BA: build from Store, write back ────────────────────────────────────────
namespace {

/// Greedy angular-coverage observation selector (design doc §5.3).
/// Returns up to k indices (into obs_list) that maximise the minimum pairwise angle
/// between selected view directions.  Only observations from registered cameras are considered.
/// If fewer than k valid observations exist, all valid indices are returned.
std::vector<int> select_observations_angular_coverage(
    const Eigen::Vector3d& point3d, const std::vector<Observation>& obs_list,
    const std::vector<Eigen::Vector3d>& poses_C, const std::vector<bool>& registered, int k) {
  if (k <= 0)
    return {};

  struct Candidate {
    int obs_idx;
    Eigen::Vector3d dir; // normalised direction from point to camera centre
  };
  std::vector<Candidate> cands;
  cands.reserve(obs_list.size());
  const int n_images = static_cast<int>(poses_C.size());
  for (int i = 0; i < static_cast<int>(obs_list.size()); ++i) {
    const int im = static_cast<int>(obs_list[static_cast<size_t>(i)].image_index);
    if (im < 0 || im >= n_images || !registered[static_cast<size_t>(im)])
      continue;
    const Eigen::Vector3d d = poses_C[static_cast<size_t>(im)] - point3d;
    const double len = d.norm();
    if (len < 1e-10)
      continue;
    cands.push_back({i, d / len});
  }

  if (static_cast<int>(cands.size()) <= k) {
    std::vector<int> result;
    result.reserve(cands.size());
    for (const auto& c : cands)
      result.push_back(c.obs_idx);
    return result;
  }

  // Step 1: find the pair with the largest baseline angle
  int sel_a = 0, sel_b = 1;
  double max_angle = -1.0;
  for (int i = 0; i < static_cast<int>(cands.size()); ++i) {
    for (int j = i + 1; j < static_cast<int>(cands.size()); ++j) {
      const double cos_a =
          std::max(-1.0, std::min(1.0, cands[static_cast<size_t>(i)].dir.dot(
                                           cands[static_cast<size_t>(j)].dir)));
      const double angle = std::acos(cos_a);
      if (angle > max_angle) {
        max_angle = angle;
        sel_a = i;
        sel_b = j;
      }
    }
  }

  std::vector<bool> selected(cands.size(), false);
  selected[static_cast<size_t>(sel_a)] = true;
  selected[static_cast<size_t>(sel_b)] = true;
  std::vector<int> sel_cand_indices = {sel_a, sel_b};

  // Step 2: greedily add the observation that maximises the minimum angle to the selected set
  while (static_cast<int>(sel_cand_indices.size()) < k) {
    int best_ci = -1;
    double best_min_angle = -1.0;
    for (int i = 0; i < static_cast<int>(cands.size()); ++i) {
      if (selected[static_cast<size_t>(i)])
        continue;
      double min_angle = std::numeric_limits<double>::max();
      for (int si : sel_cand_indices) {
        const double cos_a =
            std::max(-1.0, std::min(1.0, cands[static_cast<size_t>(i)].dir.dot(
                                             cands[static_cast<size_t>(si)].dir)));
        const double angle = std::acos(cos_a);
        if (angle < min_angle)
          min_angle = angle;
      }
      if (min_angle > best_min_angle) {
        best_min_angle = min_angle;
        best_ci = i;
      }
    }
    if (best_ci < 0)
      break;
    selected[static_cast<size_t>(best_ci)] = true;
    sel_cand_indices.push_back(best_ci);
  }

  std::vector<int> result;
  result.reserve(sel_cand_indices.size());
  for (int ci : sel_cand_indices)
    result.push_back(cands[static_cast<size_t>(ci)].obs_idx);
  return result;
}

bool build_ba_input_from_store(
    const TrackStore& store, const std::vector<Eigen::Matrix3d>& poses_R,
    const std::vector<Eigen::Vector3d>& poses_C, const std::vector<bool>& registered,
    const std::vector<int>& image_to_camera_index, const std::vector<camera::Intrinsics>& cameras,
    const std::vector<int>* image_subset, std::vector<int>* ba_image_index_to_global_out,
    BAInput* ba_input_out, std::vector<int>* point_index_to_track_id_out,
    const std::set<int>* active_cameras_for_points = nullptr, int max_obs_per_track = -1,
    int max_points = -1) {
  if (!ba_input_out || !point_index_to_track_id_out || !ba_image_index_to_global_out)
    return false;
  const int n_images = store.num_images();
  if (n_images == 0 || static_cast<int>(poses_R.size()) != n_images ||
      static_cast<int>(poses_C.size()) != n_images ||
      static_cast<int>(registered.size()) != n_images ||
      static_cast<int>(image_to_camera_index.size()) != n_images)
    return false;

  std::vector<int> global_indices;
  if (image_subset && !image_subset->empty()) {
    for (int g : *image_subset) {
      if (g >= 0 && g < n_images && registered[static_cast<size_t>(g)])
        global_indices.push_back(g);
    }
    std::sort(global_indices.begin(), global_indices.end());
  } else {
    for (int g = 0; g < n_images; ++g)
      if (registered[static_cast<size_t>(g)])
        global_indices.push_back(g);
  }
  if (global_indices.empty())
    return false;

  std::unordered_set<int> ba_global_set(global_indices.begin(), global_indices.end());
  *ba_image_index_to_global_out = global_indices;

  BAInput& ba = *ba_input_out;
  ba.poses_R.clear();
  ba.poses_C.clear();
  ba.points3d.clear();
  ba.observations.clear();
  ba.image_camera_index.clear();
  ba.cameras = cameras;
  ba.fix_pose.clear();
  ba.fix_point.clear();
  ba.fix_intrinsics_flags.clear();

  for (int g : global_indices) {
    ba.poses_R.push_back(poses_R[static_cast<size_t>(g)]);
    ba.poses_C.push_back(poses_C[static_cast<size_t>(g)]);
    const int cam_idx = image_to_camera_index[static_cast<size_t>(g)];
    ba.image_camera_index.push_back(
        cam_idx >= 0 && cam_idx < static_cast<int>(cameras.size()) ? cam_idx : 0);
  }
  const int n_ba_images = static_cast<int>(global_indices.size());

  std::vector<int> track_id_to_point_index(store.num_tracks(), -1);
  point_index_to_track_id_out->clear();
  std::vector<Observation> obs_buf;

  // ── Phase 1: collect eligible tracks ──────────────────────────────────────
  // Scored candidates for optional max_points subsampling: {track_length, track_id}
  std::vector<std::pair<int, int>> eligible_tracks; // {score, track_id}

  for (size_t ti = 0; ti < store.num_tracks(); ++ti) {
    const int track_id = static_cast<int>(ti);
    if (!store.is_track_valid(track_id) || !store.track_has_triangulated_xyz(track_id))
      continue;
    // Guard: reject NaN/Inf 3D coords (can result from degenerate DLT).
    float px, py, pz;
    store.get_track_xyz(track_id, &px, &py, &pz);
    if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz))
      continue;
    obs_buf.clear();
    store.get_track_observations(track_id, &obs_buf);

    int visible_in_ba = 0;
    bool visible_in_active = false;
    for (const auto& o : obs_buf) {
      const int im = static_cast<int>(o.image_index);
      if (!ba_global_set.count(im))
        continue;
      ++visible_in_ba;
      if (active_cameras_for_points && active_cameras_for_points->count(im))
        visible_in_active = true;
    }
    if (visible_in_ba < 2)
      continue;
    // If active_cameras_for_points filter is set, require at least one active camera observation.
    if (active_cameras_for_points && !visible_in_active)
      continue;

    eligible_tracks.push_back({static_cast<int>(obs_buf.size()), track_id});
  }

  // ── Phase 2: optional max_points subsampling (keep highest-score tracks) ──
  if (max_points > 0 && static_cast<int>(eligible_tracks.size()) > max_points) {
    // Sort descending by track length (longer tracks are more reliable)
    std::sort(eligible_tracks.begin(), eligible_tracks.end(),
              [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
                return a.first > b.first;
              });
    eligible_tracks.resize(static_cast<size_t>(max_points));
  }

  // ── Phase 3: build point list ──────────────────────────────────────────────
  for (const auto& sc_tid : eligible_tracks) {
    const int track_id = sc_tid.second;
    const int pt_idx = static_cast<int>(point_index_to_track_id_out->size());
    track_id_to_point_index[track_id] = pt_idx;
    point_index_to_track_id_out->push_back(track_id);
    float x, y, z;
    store.get_track_xyz(track_id, &x, &y, &z);
    ba.points3d.emplace_back(static_cast<double>(x), static_cast<double>(y),
                             static_cast<double>(z));
  }
  ba.fix_point.resize(ba.points3d.size(), false);

  // ── Phase 4: build observations (with optional angular coverage compression) ──
  for (const auto& sc_tid : eligible_tracks) {
    const int track_id = sc_tid.second;
    const int pt_idx = track_id_to_point_index[track_id];
    if (pt_idx < 0)
      continue;
    obs_buf.clear();
    store.get_track_observations(track_id, &obs_buf);

    // Determine which observations to include
    std::vector<int> selected_obs_indices;
    if (max_obs_per_track > 0 && static_cast<int>(obs_buf.size()) > max_obs_per_track) {
      // Angular coverage compression: select best K observations
      float px, py, pz;
      store.get_track_xyz(track_id, &px, &py, &pz);
      const Eigen::Vector3d pt3d(static_cast<double>(px), static_cast<double>(py),
                                 static_cast<double>(pz));
      selected_obs_indices = select_observations_angular_coverage(pt3d, obs_buf, poses_C,
                                                                  registered, max_obs_per_track);
    } else {
      selected_obs_indices.resize(obs_buf.size());
      for (int i = 0; i < static_cast<int>(obs_buf.size()); ++i)
        selected_obs_indices[static_cast<size_t>(i)] = i;
    }

    for (int oi : selected_obs_indices) {
      const auto& o = obs_buf[static_cast<size_t>(oi)];
      const int g = static_cast<int>(o.image_index);
      auto it = std::lower_bound(global_indices.begin(), global_indices.end(), g);
      if (it == global_indices.end() || *it != g)
        continue;
      const int ba_im = static_cast<int>(it - global_indices.begin());
      const double scale = (o.scale > 1e-6f) ? static_cast<double>(o.scale) : 1.0;
      ba.observations.push_back(
          {ba_im, pt_idx, static_cast<double>(o.u), static_cast<double>(o.v), scale});
    }
  }

  ba.fix_pose.resize(static_cast<size_t>(n_ba_images), false);
  return true;
}

void write_ba_result_back(const BAResult& result, const std::vector<int>& ba_image_index_to_global,
                          const std::vector<int>& point_index_to_track_id, TrackStore* store,
                          std::vector<Eigen::Matrix3d>* poses_R,
                          std::vector<Eigen::Vector3d>* poses_C,
                          std::vector<camera::Intrinsics>* cameras_in_out) {
  if (!result.success || !store)
    return;
  for (size_t i = 0; i < result.poses_R.size() && i < ba_image_index_to_global.size(); ++i) {
    const int g = ba_image_index_to_global[i];
    if (poses_R && g < static_cast<int>(poses_R->size()))
      (*poses_R)[static_cast<size_t>(g)] = result.poses_R[i];
    if (poses_C && g < static_cast<int>(poses_C->size()))
      (*poses_C)[static_cast<size_t>(g)] = result.poses_C[i];
  }
  for (size_t i = 0; i < result.points3d.size() && i < point_index_to_track_id.size(); ++i) {
    const int track_id = point_index_to_track_id[i];
    const Eigen::Vector3d& p = result.points3d[i];
    store->set_track_xyz(track_id, static_cast<float>(p(0)), static_cast<float>(p(1)),
                         static_cast<float>(p(2)));
  }
  if (cameras_in_out && !result.cameras.empty())
    *cameras_in_out = result.cameras;
}

// Log all camera intrinsics at INFO level (called after each global BA)
static void log_cameras(const std::vector<camera::Intrinsics>& cameras, const char* label) {
  for (size_t i = 0; i < cameras.size(); ++i) {
    const auto& K = cameras[i];
    LOG(INFO) << label << "  cam[" << i << "]:"
              << "  fx=" << K.fx << "  fy=" << K.fy << "  cx=" << K.cx << "  cy=" << K.cy
              << "  k1=" << K.k1 << "  k2=" << K.k2 << "  k3=" << K.k3 << "  p1=" << K.p1
              << "  p2=" << K.p2;
  }
}

} // namespace

bool run_global_ba(TrackStore* store, std::vector<Eigen::Matrix3d>* poses_R,
                   std::vector<Eigen::Vector3d>* poses_C, const std::vector<bool>& registered,
                   const std::vector<int>& image_to_camera_index,
                   const std::vector<camera::Intrinsics>& cameras,
                   std::vector<camera::Intrinsics>* cameras_in_out, bool optimize_intrinsics,
                   int max_iterations, double* rmse_px_out, int anchor_image,
                   double max_rmse_px) {
  if (!store || !poses_R || !poses_C)
    return false;
  std::vector<int> ba_image_index_to_global;
  std::vector<int> point_index_to_track_id;
  BAInput ba_in;
  if (!build_ba_input_from_store(*store, *poses_R, *poses_C, registered, image_to_camera_index,
                                 cameras, nullptr, &ba_image_index_to_global, &ba_in,
                                 &point_index_to_track_id))
    return false;
  ba_in.optimize_intrinsics = optimize_intrinsics;
  // Fix the coordinate-system anchor: prefer the initial-pair reference camera (anchor_image).
  // Falling back to index 0 risks anchoring a newly-PnP-estimated camera instead of the true
  // origin, causing the whole reconstruction to drift/collapse.
  {
    int anchor_ba_idx = 0; // default: lowest registered image index
    if (anchor_image >= 0) {
      for (size_t bi = 0; bi < ba_image_index_to_global.size(); ++bi) {
        if (ba_image_index_to_global[bi] == anchor_image) {
          anchor_ba_idx = static_cast<int>(bi);
          break;
        }
      }
    }
    ba_in.fix_pose[static_cast<size_t>(anchor_ba_idx)] = true;
    LOG(INFO) << "run_global_ba: anchor global=" << anchor_image
              << " → ba_idx=" << anchor_ba_idx
              << " (ba_idx 0 = global " << ba_image_index_to_global[0] << ")";
  }
  if (!optimize_intrinsics)
    ba_in.fix_intrinsics_flags.resize(ba_in.cameras.size(),
                                      static_cast<uint32_t>(FixIntrinsicsMask::kFixIntrAll));

  // ── Diagnostic: sample first camera pose, first 3 points, first 3 obs ────
  {
    const auto& K = ba_in.cameras[0];
    LOG(INFO) << "run_global_ba: cam[0] fx=" << K.fx << " fy=" << K.fy
              << " cx=" << K.cx << " cy=" << K.cy;
    for (size_t di = 0; di < std::min<size_t>(3, ba_in.poses_C.size()); ++di) {
      const int g = ba_image_index_to_global[di];
      LOG(INFO) << "run_global_ba: pose[" << di << "] global=" << g
                << " C=" << ba_in.poses_C[di].transpose()
                << " fixed=" << (ba_in.fix_pose.size() > di && ba_in.fix_pose[di] ? "YES" : "no");
    }
    for (size_t di = 0; di < std::min<size_t>(3, ba_in.points3d.size()); ++di) {
      LOG(INFO) << "run_global_ba: pt[" << di << "]=" << ba_in.points3d[di].transpose();
    }
    for (size_t di = 0; di < std::min<size_t>(3, ba_in.observations.size()); ++di) {
      const auto& ob = ba_in.observations[di];
      LOG(INFO) << "run_global_ba: obs[" << di << "] im=" << ob.image_index
                << " pt=" << ob.point_index << " u=" << ob.u << " v=" << ob.v;
    }
  }
  // ─────────────────────────────────────────────────────────────────────────

  LOG(INFO) << "run_global_ba: " << ba_in.poses_R.size() << " images, " << ba_in.points3d.size()
            << " points, " << ba_in.observations.size() << " obs, max_iter=" << max_iterations;
  BAResult ba_out;
  if (!global_bundle_analytic(ba_in, &ba_out, max_iterations)) {
    LOG(WARNING) << "run_global_ba: solver failed";
    return false;
  }
  LOG(INFO) << "run_global_ba: RMSE=" << ba_out.rmse_px << " px, success=" << ba_out.success;
  // Sanity guard: if RMSE exceeds the caller-supplied ceiling, discard the result to prevent
  // corrupt intrinsics/poses from propagating into the track store.
  if (max_rmse_px > 0.0 && ba_out.rmse_px > max_rmse_px) {
    LOG(WARNING) << "run_global_ba: RMSE " << ba_out.rmse_px << " > max " << max_rmse_px
                 << " px; discarding BA result (intrinsics/poses not updated)";
    if (rmse_px_out)
      *rmse_px_out = ba_out.rmse_px;
    return false;
  }
  write_ba_result_back(ba_out, ba_image_index_to_global, point_index_to_track_id, store, poses_R,
                       poses_C, cameras_in_out);
  // Log current intrinsics so drift / optimisation progress is visible
  log_cameras(cameras_in_out ? *cameras_in_out : cameras, "run_global_ba");
  if (rmse_px_out)
    *rmse_px_out = ba_out.rmse_px;
  return true;
}

bool run_local_ba(TrackStore* store, std::vector<Eigen::Matrix3d>* poses_R,
                  std::vector<Eigen::Vector3d>* poses_C, const std::vector<bool>& registered,
                  const std::vector<int>& image_to_camera_index,
                  const std::vector<camera::Intrinsics>& cameras, int local_ba_window,
                  int max_iterations, double* rmse_px_out,
                  const std::vector<int>* indices_to_optimize,
                  const std::vector<int>* fixed_indices,
                  int max_points,
                  int max_obs_per_track,
                  double max_rmse_px) {
  if (!store || !poses_R || !poses_C || local_ba_window <= 0)
    return false;

  // ── Build ACTIVE camera set ──────────────────────────────────────────────
  std::set<int> active_set;
  if (indices_to_optimize && !indices_to_optimize->empty()) {
    for (int im : *indices_to_optimize)
      if (im >= 0 && im < store->num_images() && registered[static_cast<size_t>(im)])
        active_set.insert(im);
  }
  if (active_set.empty()) {
    // Fallback: last local_ba_window registered images by index
    std::vector<int> reg_indices;
    const int n_images = store->num_images();
    for (int i = 0; i < n_images; ++i)
      if (registered[static_cast<size_t>(i)])
        reg_indices.push_back(i);
    if (reg_indices.empty())
      return false;
    const int n_opt = std::min(local_ba_window, static_cast<int>(reg_indices.size()));
    active_set.insert(reg_indices.end() - static_cast<size_t>(n_opt), reg_indices.end());
  }

  // ── Build FIXED camera set ───────────────────────────────────────────────
  // FIXED cameras participate as structural constraints (fix_pose=true) but are not optimised.
  // Only points visible in at least one ACTIVE camera are included (§4 design doc).
  std::set<int> fixed_set;
  if (fixed_indices) {
    for (int im : *fixed_indices)
      if (im >= 0 && im < store->num_images() && registered[static_cast<size_t>(im)] &&
          !active_set.count(im))
        fixed_set.insert(im);
  }

  // Combined image subset for BA input: active + fixed
  const bool has_fixed = !fixed_set.empty();
  std::vector<int> combined_subset;
  combined_subset.reserve(active_set.size() + fixed_set.size());
  for (int im : active_set)
    combined_subset.push_back(im);
  for (int im : fixed_set)
    combined_subset.push_back(im);
  std::sort(combined_subset.begin(), combined_subset.end());

  // When FIXED cameras are present, restrict points to those visible in ACTIVE cameras,
  // reducing BA problem size (design doc §4, §8).
  const std::set<int>* active_filter = has_fixed ? &active_set : nullptr;
  const int eff_max_points = (max_points > 0) ? max_points : -1;
  const int eff_max_obs = (max_obs_per_track > 0) ? max_obs_per_track : -1;

  std::vector<int> ba_image_index_to_global;
  std::vector<int> point_index_to_track_id;
  BAInput ba_in;
  if (!build_ba_input_from_store(*store, *poses_R, *poses_C, registered, image_to_camera_index,
                                 cameras, &combined_subset, &ba_image_index_to_global, &ba_in,
                                 &point_index_to_track_id, active_filter, eff_max_obs,
                                 eff_max_points))
    return false;

  ba_in.optimize_intrinsics = false;

  // Mark poses: ACTIVE → optimise, FIXED → hold constant
  for (size_t i = 0; i < ba_image_index_to_global.size(); ++i) {
    const int g = ba_image_index_to_global[i];
    ba_in.fix_pose[i] = (active_set.count(g) == 0); // fixed if not in active_set
  }
  // Gauge-freedom safety net: if no camera was marked fixed (all-active window),
  // fix BA index 0 to anchor the coordinate system and remove the 6-DOF rigid-body ambiguity.
  {
    bool any_fixed = false;
    for (bool f : ba_in.fix_pose) if (f) { any_fixed = true; break; }
    if (!any_fixed && !ba_in.fix_pose.empty()) {
      ba_in.fix_pose[0] = true;
      LOG(INFO) << "run_local_ba: no fixed cameras; anchoring BA idx 0 (global "
                << ba_image_index_to_global[0] << ") to remove gauge freedom";
    }
  }

  ba_in.fix_intrinsics_flags.resize(ba_in.cameras.size(),
                                    static_cast<uint32_t>(FixIntrinsicsMask::kFixIntrAll));

  int n_active = 0, n_fixed_ba = 0;
  for (size_t i = 0; i < ba_in.fix_pose.size(); ++i)
    (ba_in.fix_pose[i] ? n_fixed_ba : n_active)++;

  LOG(INFO) << "run_local_ba: " << ba_image_index_to_global.size() << " images (" << n_active
            << " active, " << n_fixed_ba << " fixed), " << ba_in.points3d.size() << " points, "
            << ba_in.observations.size() << " obs";

  BAResult ba_out;
  if (!global_bundle_analytic(ba_in, &ba_out, max_iterations)) {
    LOG(WARNING) << "run_local_ba: solver failed";
    return false;
  }
  if (max_rmse_px > 0.0 && ba_out.rmse_px > max_rmse_px) {
    LOG(WARNING) << "run_local_ba: RMSE " << ba_out.rmse_px << " > max " << max_rmse_px
                 << " px; discarding BA result (poses/points not updated)";
    if (rmse_px_out)
      *rmse_px_out = ba_out.rmse_px;
    return false;
  }
  LOG(INFO) << "run_local_ba: RMSE=" << ba_out.rmse_px << " px";
  write_ba_result_back(ba_out, ba_image_index_to_global, point_index_to_track_id, store, poses_R,
                       poses_C, nullptr);
  log_cameras(cameras, "run_local_ba");
  if (rmse_px_out)
    *rmse_px_out = ba_out.rmse_px;
  return true;
}

bool run_incremental_sfm_pipeline(const std::string& tracks_idc_path,
                                  const std::string& pairs_json_path, const std::string& geo_dir,
                                  std::vector<camera::Intrinsics>* cameras,
                                  const std::vector<int>& image_to_camera_index,
                                  const IncrementalSfMOptions& opts, TrackStore* store_out,
                                  std::vector<Eigen::Matrix3d>* poses_R_out,
                                  std::vector<Eigen::Vector3d>* poses_C_out,
                                  std::vector<bool>* registered_out, uint32_t* initial_im0_out,
                                  uint32_t* initial_im1_out) {
  if (!store_out || !poses_R_out || !poses_C_out || !registered_out || !cameras)
    return false;
  if (!load_track_store_from_idc(tracks_idc_path, store_out, nullptr)) {
    LOG(ERROR) << "run_incremental_sfm_pipeline: failed to load tracks from " << tracks_idc_path;
    return false;
  }
  const int n_images = store_out->num_images();
  if (n_images == 0 || static_cast<int>(image_to_camera_index.size()) != n_images) {
    LOG(ERROR) << "run_incremental_sfm_pipeline: num_images mismatch";
    return false;
  }
  poses_R_out->resize(static_cast<size_t>(n_images));
  poses_C_out->resize(static_cast<size_t>(n_images));
  registered_out->resize(static_cast<size_t>(n_images), false);

  ViewGraph view_graph;
  if (!build_view_graph_from_geo(pairs_json_path, geo_dir, &view_graph)) {
    LOG(ERROR) << "run_incremental_sfm_pipeline: failed to build view graph";
    return false;
  }
  // Always capture anchor internally; caller's output pointers may be null.
  uint32_t local_im0 = 0, local_im1 = 0;
  uint32_t* im0_ptr = initial_im0_out ? initial_im0_out : &local_im0;
  uint32_t* im1_ptr = initial_im1_out ? initial_im1_out : &local_im1;
  if (!run_initial_pair_loop(view_graph, geo_dir, store_out, *cameras, image_to_camera_index,
                             opts.min_tracks_after, im0_ptr, im1_ptr, poses_R_out,
                             poses_C_out, registered_out)) {
    LOG(ERROR) << "run_incremental_sfm_pipeline: no initial pair succeeded";
    return false;
  }
  int num_registered = 0;
  for (bool r : *registered_out)
    if (r)
      ++num_registered;
  LOG(INFO) << "Initial pair done. Registered: " << num_registered << "/" << n_images;
  // ── Initial-pair diagnostic ──────────────────────────────────────────────
  {
    const int ip0 = static_cast<int>(*im0_ptr);
    const int ip1 = static_cast<int>(*im1_ptr);
    LOG(INFO) << "  Initial pair images: im0=" << ip0 << " im1=" << ip1;
    const auto& C0 = (*poses_C_out)[static_cast<size_t>(ip0)];
    const auto& C1 = (*poses_C_out)[static_cast<size_t>(ip1)];
    LOG(INFO) << "  C_im0=" << C0.transpose() << "  C_im1=" << C1.transpose()
              << "  baseline=" << (C1 - C0).norm();
    // Print first few triangulated 3D points and depth in im0's camera frame
    int shown = 0;
    for (size_t ti = 0; ti < store_out->num_tracks() && shown < 5; ++ti) {
      const int tid = static_cast<int>(ti);
      if (!store_out->is_track_valid(tid) || !store_out->track_has_triangulated_xyz(tid))
        continue;
      float px, py, pz;
      store_out->get_track_xyz(tid, &px, &py, &pz);
      const Eigen::Vector3d X(px, py, pz);
      const double depth0 = ((*poses_R_out)[static_cast<size_t>(ip0)] * (X - C0))(2);
      const double depth1 = ((*poses_R_out)[static_cast<size_t>(ip1)] * (X - C1))(2);
      LOG(INFO) << "  track " << tid << ": X=" << X.transpose() << "  depth0=" << depth0
                << "  depth1=" << depth1;
      ++shown;
    }
  }
  // ────────────────────────────────────────────────────────────────────────
  // The initial-pair im0 defines the world coordinate origin (C=0, R=I).
  // All global BA calls must fix this camera to preserve that origin.
  const int anchor_image = static_cast<int>(*im0_ptr);
  LOG(INFO) << "run_incremental_sfm_pipeline: anchor_image=" << anchor_image;

  // Count triangulated tracks for diagnostics
  auto count_tri_tracks = [&]() -> int {
    int n = 0;
    for (size_t ti = 0; ti < store_out->num_tracks(); ++ti)
      if (store_out->is_track_valid(static_cast<int>(ti)) &&
          store_out->track_has_triangulated_xyz(static_cast<int>(ti)))
        ++n;
    return n;
  };
  LOG(INFO) << "Triangulated tracks after initial pair: " << count_tri_tracks();

  // Compute adaptive rejection threshold: max(fixed, prev_rmse * factor).
  // This prevents premature culling when RMSE is temporarily high (e.g. early intrinsics convergence).
  auto eff_reject_threshold = [&](double current_rmse) -> double {
    if (opts.outlier_adaptive_factor > 0.0 && current_rmse > 0.0)
      return std::max(opts.outlier_threshold_px, current_rmse * opts.outlier_adaptive_factor);
    return opts.outlier_threshold_px;
  };

  // ── BA scheduling counters (design doc §7, §9) ──────────────────────────
  // local_ba_interval: run local BA only every N newly registered images.
  // global_ba_interval: run a lightweight periodic global BA every N images.
  int local_ba_images_since_last = 0;
  int global_ba_images_since_last = 0;

  // Helper: run a lightweight periodic global BA using a random subset of points.
  // Uses 10% of total triangulated tracks (design doc §9) to bound computation.
  auto run_periodic_global_ba = [&]() {
    const int total_tri = count_tri_tracks();
    const int target_pts = std::max(1000, total_tri / 10);
    LOG(INFO) << "run_periodic_global_ba: " << num_registered << " images, ~" << target_pts
              << " points (10% of " << total_tri << " tri tracks)";
    std::vector<int> ba_img_to_global;
    std::vector<int> pt_to_track;
    BAInput ba_in;
    if (!build_ba_input_from_store(*store_out, *poses_R_out, *poses_C_out, *registered_out,
                                   image_to_camera_index, *cameras, nullptr, &ba_img_to_global,
                                   &ba_in, &pt_to_track,
                                   /*active_cameras_for_points=*/nullptr,
                                   /*max_obs_per_track=*/-1,
                                   /*max_points=*/target_pts))
      return;
    ba_in.optimize_intrinsics = false;
    ba_in.fix_intrinsics_flags.resize(ba_in.cameras.size(),
                                      static_cast<uint32_t>(FixIntrinsicsMask::kFixIntrAll));
    // Fix coordinate-system anchor; fall back to BA idx 0 if anchor not found
    {
      bool found = false;
      for (size_t bi = 0; bi < ba_img_to_global.size(); ++bi) {
        if (ba_img_to_global[bi] == anchor_image) {
          ba_in.fix_pose[bi] = true;
          found = true;
          break;
        }
      }
      if (!found && !ba_in.fix_pose.empty()) {
        ba_in.fix_pose[0] = true; // gauge freedom safety net
        LOG(INFO) << "run_periodic_global_ba: anchor " << anchor_image
                  << " not in BA set; anchoring BA idx 0 as fallback";
      }
    }
    BAResult ba_out;
    if (!global_bundle_analytic(ba_in, &ba_out, opts.max_global_ba_iterations))
      return;
    if (opts.max_reasonable_rmse_px > 0.0 && ba_out.rmse_px > opts.max_reasonable_rmse_px) {
      LOG(WARNING) << "run_periodic_global_ba: RMSE " << ba_out.rmse_px << " > max "
                   << opts.max_reasonable_rmse_px
                   << " px; discarding BA result (poses/points not updated)";
      return;
    }
    LOG(INFO) << "run_periodic_global_ba: RMSE=" << ba_out.rmse_px << " px";
    write_ba_result_back(ba_out, ba_img_to_global, pt_to_track, store_out, poses_R_out, poses_C_out,
                         nullptr);
    global_ba_images_since_last = 0;
  };

  auto run_ba_and_reject_outliers = [&](double* rmse_out) {
    // Capture RMSE from the preceding BA (passed in via rmse_out) before resetting.
    double rmse = (rmse_out && *rmse_out > 0.0) ? *rmse_out : 0.0;
    if (rmse_out)
      *rmse_out = 0.0;
    bool do_reject = (num_registered >= opts.reject_min_registered_images);
    int rejected = 0;
    if (do_reject) {
      double thr = eff_reject_threshold(rmse);
      rejected =
          reject_outliers_multiview(store_out, *poses_R_out, *poses_C_out, *registered_out,
                                    *cameras, image_to_camera_index, thr);
      const bool run_angle_reject =
          (opts.angle_reject_max_rmse_px <= 0.0 || rmse <= opts.angle_reject_max_rmse_px);
      if (run_angle_reject) {
        rejected += reject_outliers_angle_multiview(store_out, *poses_R_out, *poses_C_out,
                                                    *registered_out,
                                                    opts.min_observation_angle_deg);
      } else {
        LOG(INFO) << "  outlier rejection: skip angle-based reject (RMSE=" << rmse
                  << " px > " << opts.angle_reject_max_rmse_px << " px)";
      }
      if (rejected > 0)
        LOG(INFO) << "  outlier rejection: " << rejected << " observations marked (thr=" << thr
                  << " px)";
    }
    for (int r = 0;
         rejected >= std::max(1, opts.outlier_iter_min_rejections) &&
         r < opts.max_outlier_iterations - 1;
         ++r) {
      if (num_registered < opts.local_ba_after_n_images) {
        const bool opt_intr = opts.global_ba_optimize_intrinsics &&
                              num_registered >= opts.global_ba_optimize_intrinsics_min_images;
        if (!run_global_ba(store_out, poses_R_out, poses_C_out, *registered_out,
                           image_to_camera_index, *cameras, cameras, opt_intr,
                           opts.max_global_ba_iterations, &rmse, anchor_image))
          break;
      } else {
        // Outlier-iteration local BA: apply same compression as scheduled BA
        const int oi_max_pts = (opts.max_points_local_ba > 0) ? opts.max_points_local_ba : -1;
        const int oi_max_obs = (opts.enable_obs_compression && opts.max_track_length_ba > 0)
                                   ? opts.max_track_length_ba
                                   : -1;
        if (!run_local_ba(store_out, poses_R_out, poses_C_out, *registered_out,
                          image_to_camera_index, *cameras, opts.local_ba_window, 25, &rmse,
                          nullptr, nullptr, oi_max_pts, oi_max_obs,
                          opts.max_reasonable_rmse_px))
          break;
      }
      double thr = eff_reject_threshold(rmse);
      rejected =
          reject_outliers_multiview(store_out, *poses_R_out, *poses_C_out, *registered_out,
                                    *cameras, image_to_camera_index, thr);
      const bool run_angle_reject =
          (opts.angle_reject_max_rmse_px <= 0.0 || rmse <= opts.angle_reject_max_rmse_px);
      if (run_angle_reject) {
        rejected += reject_outliers_angle_multiview(store_out, *poses_R_out, *poses_C_out,
                                                    *registered_out,
                                                    opts.min_observation_angle_deg);
      }
    }
    if (do_reject && opts.enable_sigma_filter && rmse > 0.01) {
      reject_outliers_multiview(store_out, *poses_R_out, *poses_C_out, *registered_out, *cameras,
                                image_to_camera_index, rmse * 3.0);
    }
    if (rmse_out)
      *rmse_out = rmse;
  };

  for (;;) {
    LOG(INFO) << "════ SfM iteration: registered=" << num_registered << "/" << n_images
              << ", tri_tracks=" << count_tri_tracks() << " ════";
    std::vector<int> batch =
        choose_next_resection_batch(*store_out, *registered_out, opts.resection_batch_k);
    if (batch.empty()) {
      if (opts.retry_resection_after_cleanup &&
          num_registered >= opts.reject_min_registered_images) {
        double rmse = 0.0;
        // Cleanup BA: NEVER optimise intrinsics here.  Its sole purpose is to
        // clean up outliers so that retry-resection can succeed.  Optimising
        // intrinsics with a partially-degenerate scene causes catastrophic
        // divergence (cx/k1 blow up) that destroys the entire reconstruction.
        if (run_global_ba(store_out, poses_R_out, poses_C_out, *registered_out,
                          image_to_camera_index, *cameras, cameras,
                          /*optimize_intrinsics=*/false,
                          opts.max_global_ba_iterations, &rmse, anchor_image,
                          opts.max_reasonable_rmse_px)) {
          double cleanup_thr = eff_reject_threshold(rmse);
          int rej =
              reject_outliers_multiview(store_out, *poses_R_out, *poses_C_out, *registered_out,
                                        *cameras, image_to_camera_index, cleanup_thr);
          if (opts.angle_reject_max_rmse_px <= 0.0 || rmse <= opts.angle_reject_max_rmse_px)
            rej += reject_outliers_angle_multiview(store_out, *poses_R_out, *poses_C_out,
                                                   *registered_out,
                                                   opts.min_observation_angle_deg);
          if (rej > 0) {
            LOG(INFO) << "run_incremental_sfm_pipeline: cleanup BA + reject " << rej
                      << " (thr=" << cleanup_thr << " px), retry resection";
            batch =
                choose_next_resection_batch(*store_out, *registered_out, opts.resection_batch_k);
          }
        }
      }
      if (batch.empty())
        break;
    }
    int added = run_batch_resection(*store_out, batch, *cameras, image_to_camera_index, poses_R_out,
                                    poses_C_out, registered_out, opts.resection_min_inliers);
    if (added == 0) {
      LOG(WARNING) << "  No images could be resected in this batch, stopping";
      break;
    }
    int n_new_tri =
        run_batch_triangulation(store_out, batch, *poses_R_out, *poses_C_out, *registered_out,
                                *cameras, image_to_camera_index, opts.min_tri_angle_deg);
    num_registered = 0;
    for (bool r : *registered_out)
      if (r)
        ++num_registered;
    LOG(INFO) << "  After resection+triangulation: registered=" << num_registered
              << ", new_tri=" << n_new_tri << ", total_tri=" << count_tri_tracks();

    // Update scheduling counters
    local_ba_images_since_last += added;
    global_ba_images_since_last += added;

    double rmse = 0.0;
    if (num_registered < opts.local_ba_after_n_images) {
      // Early stage: always run global BA (small scene, stability matters more than speed)
      const bool opt_intr = opts.global_ba_optimize_intrinsics &&
                            num_registered >= opts.global_ba_optimize_intrinsics_min_images;
      if (run_global_ba(store_out, poses_R_out, poses_C_out, *registered_out, image_to_camera_index,
                        *cameras, cameras, opt_intr, opts.max_global_ba_iterations, &rmse,
                        anchor_image)) {
        LOG(INFO) << "run_incremental_sfm_pipeline: global BA (early), n=" << num_registered
                  << " RMSE=" << rmse << " px";
        run_ba_and_reject_outliers(&rmse);
        local_ba_images_since_last = 0;
        global_ba_images_since_last = 0;
      }
    } else {
      // ── Periodic Global BA (design doc §9) ────────────────────────────────
      // Every global_ba_interval images: run global BA with 10% of points to fix drift.
      if (opts.global_ba_interval > 0 &&
          global_ba_images_since_last >= opts.global_ba_interval) {
        run_periodic_global_ba();
        // Also reset local BA counter so we don't immediately do local + global back-to-back
        local_ba_images_since_last = 0;
      }

      // ── Local BA with interval scheduling (design doc §7) ─────────────────
      // Only trigger local BA every local_ba_interval newly registered images.
      const int eff_local_interval = std::max(1, opts.local_ba_interval);
      if (local_ba_images_since_last >= eff_local_interval) {
        const std::vector<int>* local_active = nullptr;
        const std::vector<int>* local_fixed = nullptr;
        std::vector<int> active_indices, fixed_indices_vec;

        if (opts.local_ba_by_connectivity && !batch.empty()) {
          choose_local_ba_indices_by_connectivity(*store_out, *registered_out, batch,
                                                  opts.local_ba_window, opts.local_ba_fixed_window,
                                                  &active_indices, &fixed_indices_vec);
          if (!active_indices.empty())
            local_active = &active_indices;
          if (!fixed_indices_vec.empty())
            local_fixed = &fixed_indices_vec;
        }

        const int eff_max_obs =
            (opts.enable_obs_compression && opts.max_track_length_ba > 0)
                ? opts.max_track_length_ba
                : -1;
        const int eff_max_pts = (opts.max_points_local_ba > 0) ? opts.max_points_local_ba : -1;

        if (run_local_ba(store_out, poses_R_out, poses_C_out, *registered_out,
                         image_to_camera_index, *cameras, opts.local_ba_window, 25, &rmse,
                         local_active, local_fixed, eff_max_pts, eff_max_obs,
                         opts.max_reasonable_rmse_px)) {
          LOG(INFO) << "run_incremental_sfm_pipeline: local BA, n=" << num_registered
                    << " RMSE=" << rmse << " px";
          run_ba_and_reject_outliers(&rmse);
        }
        local_ba_images_since_last = 0;
      }
    }
  }

  run_retriangulation(store_out, *poses_R_out, *poses_C_out, *registered_out, *cameras,
                      image_to_camera_index, opts.min_tri_angle_deg);
  LOG(INFO) << "Final retriangulation done. tri_tracks=" << count_tri_tracks();

  if (opts.do_global_ba) {
    LOG(INFO) << "Running final global BA...";
    double rmse = 0.0;
    if (run_global_ba(store_out, poses_R_out, poses_C_out, *registered_out, image_to_camera_index,
                      *cameras, cameras, opts.global_ba_optimize_intrinsics,
                      opts.max_global_ba_iterations, &rmse, anchor_image,
                      opts.max_reasonable_rmse_px)) {
      int rejected = 0;
      if (num_registered >= opts.reject_min_registered_images) {
        double thr = eff_reject_threshold(rmse);
        rejected =
            reject_outliers_multiview(store_out, *poses_R_out, *poses_C_out, *registered_out,
                                      *cameras, image_to_camera_index, thr);
        if (opts.angle_reject_max_rmse_px <= 0.0 || rmse <= opts.angle_reject_max_rmse_px)
          rejected += reject_outliers_angle_multiview(
              store_out, *poses_R_out, *poses_C_out, *registered_out,
              opts.min_observation_angle_deg);
        if (rejected > 0)
          LOG(INFO) << "  final outlier rejection: " << rejected << " observations marked (thr="
                    << thr << " px)";
      }
      for (int r = 0;
           rejected >= std::max(1, opts.outlier_iter_min_rejections) &&
           r < opts.final_reject_max_rounds - 1;
           ++r) {
        if (!run_global_ba(store_out, poses_R_out, poses_C_out, *registered_out,
                           image_to_camera_index, *cameras, cameras,
                           opts.global_ba_optimize_intrinsics, opts.max_global_ba_iterations, &rmse,
                           anchor_image, opts.max_reasonable_rmse_px))
          break;
        double thr = eff_reject_threshold(rmse);
        rejected =
            reject_outliers_multiview(store_out, *poses_R_out, *poses_C_out, *registered_out,
                                      *cameras, image_to_camera_index, thr);
        if (opts.angle_reject_max_rmse_px <= 0.0 || rmse <= opts.angle_reject_max_rmse_px)
          rejected += reject_outliers_angle_multiview(
              store_out, *poses_R_out, *poses_C_out, *registered_out,
              opts.min_observation_angle_deg);
      }
      if (opts.enable_sigma_filter && rmse > 0.01) {
        reject_outliers_multiview(store_out, *poses_R_out, *poses_C_out, *registered_out, *cameras,
                                  image_to_camera_index, rmse * 3.0);
      }
    }
  }
  return true;
}

} // namespace sfm
} // namespace insight
