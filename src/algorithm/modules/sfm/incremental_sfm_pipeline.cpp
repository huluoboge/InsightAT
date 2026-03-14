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
#include <chrono>
#include <cmath>
#include <glog/logging.h>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace insight {
namespace sfm {

namespace {

enum class GeoLoadStatus { kOk, kFileNotFound, kNoTwoview, kBadBlobs };

GeoLoadStatus load_pair_geo_ex(const std::string& geo_path, Eigen::Matrix3d* R,
                               Eigen::Vector3d* t) {
  if (!R || !t)
    return GeoLoadStatus::kFileNotFound;
  io::IDCReader reader(geo_path);
  if (!reader.is_valid())
    return GeoLoadStatus::kFileNotFound;
  const auto& meta = reader.get_metadata();
  if (!meta.contains("twoview"))
    return GeoLoadStatus::kNoTwoview;
  auto R_blob = reader.read_blob<float>("R_matrix");
  auto t_blob = reader.read_blob<float>("t_vector");
  if (R_blob.size() != 9u || t_blob.size() != 3u)
    return GeoLoadStatus::kBadBlobs;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      (*R)(i, j) = static_cast<double>(R_blob[static_cast<size_t>(i * 3 + j)]);
  (*t)(0) = static_cast<double>(t_blob[0]);
  (*t)(1) = static_cast<double>(t_blob[1]);
  (*t)(2) = static_cast<double>(t_blob[2]);
  return GeoLoadStatus::kOk;
}

bool load_pair_geo(const std::string& geo_path, Eigen::Matrix3d* R, Eigen::Vector3d* t) {
  return load_pair_geo_ex(geo_path, R, t) == GeoLoadStatus::kOk;
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
  std::unordered_set<int> dirty_tracks; // tracks that lost at least one obs this call
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
      dirty_tracks.insert(tid);
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
      dirty_tracks.insert(tid);
      ++marked;
    }
  }
  // Clear XYZ for tracks that now have fewer than 2 valid observations so that
  // run_retriangulation can re-triangulate them once more images are registered.
  std::vector<int> obs_ids_chk;
  for (int tid : dirty_tracks) {
    if (!store->is_track_valid(tid) || !store->track_has_triangulated_xyz(tid))
      continue; // already cleared or invalid
    obs_ids_chk.clear();
    store->get_track_obs_ids(tid, &obs_ids_chk);
    int valid_obs = 0;
    for (int oid : obs_ids_chk)
      if (store->is_obs_valid(oid))
        ++valid_obs;
    if (valid_obs < 2)
      store->clear_track_xyz(tid);
  }
  return marked;
}

/// Mark observations whose max parallax angle (with any other view in the same track) is <
/// min_angle_deg.  Iterates tracks (not obs) to avoid redundant get_track_observations calls.
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
  std::vector<int> obs_ids_buf;
  // Iterate triangulated tracks — each track's obs fetched once via get_track_obs_ids.
  for (size_t ti = 0; ti < store->num_tracks(); ++ti) {
    const int tid = static_cast<int>(ti);
    if (!store->is_track_valid(tid) || !store->track_has_triangulated_xyz(tid))
      continue;
    float tx, ty, tz;
    store->get_track_xyz(tid, &tx, &ty, &tz);
    const Eigen::Vector3d X(static_cast<double>(tx), static_cast<double>(ty),
                            static_cast<double>(tz));
    obs_ids_buf.clear();
    store->get_track_obs_ids(tid, &obs_ids_buf);
    if (obs_ids_buf.size() < 2)
      continue;
    // Build (obs_id, camera_centre, ray) for registered observations only.
    struct RegObs { int obs_id; Eigen::Vector3d ray; };
    std::vector<RegObs> reg;
    reg.reserve(obs_ids_buf.size());
    for (int oid : obs_ids_buf) {
      Observation obs;
      store->get_obs(oid, &obs);
      const int im = static_cast<int>(obs.image_index);
      if (im < 0 || im >= n_images || !registered[static_cast<size_t>(im)])
        continue;
      reg.push_back({oid, (X - poses_C[static_cast<size_t>(im)]).normalized()});
    }
    if (reg.size() < 2)
      continue;
    // For each registered obs compute max angle against all others; mark if below threshold.
    for (size_t i = 0; i < reg.size(); ++i) {
      double max_angle = 0.0;
      for (size_t j = 0; j < reg.size(); ++j) {
        if (j == i) continue;
        double cos_a = std::max(-1.0, std::min(1.0, reg[i].ray.dot(reg[j].ray)));
        double angle = std::acos(cos_a);
        if (angle > max_angle) max_angle = angle;
      }
      if (max_angle < min_angle_rad) {
        store->mark_observation_deleted(reg[i].obs_id);
        ++marked;
      }
    }
    // If this track lost enough observations that fewer than 2 registered views
    // remain, clear its XYZ so run_retriangulation can recover it later.
    int still_valid = 0;
    for (const auto& ro : reg)
      if (store->is_obs_valid(ro.obs_id))
        ++still_valid;
    if (still_valid < 2)
      store->clear_track_xyz(tid);
  }
  return marked;
}

/// Return image indices to optimize for local BA: new_registered + most connected neighbors (by
/// shared tracks), up to local_ba_window.
std::vector<int> choose_local_ba_indices_by_connectivity(
    const TrackStore& store, const std::vector<bool>& registered,
    const std::vector<int>& new_registered_indices, int local_ba_window) {
  const int n_images = store.num_images();
  if (n_images == 0 || static_cast<int>(registered.size()) != n_images || local_ba_window <= 0)
    return {};
  std::set<int> optimize_set;
  for (int im : new_registered_indices)
    if (im >= 0 && im < n_images && registered[static_cast<size_t>(im)])
      optimize_set.insert(im);
  std::vector<int> reg_list;
  for (int i = 0; i < n_images; ++i)
    if (registered[static_cast<size_t>(i)])
      reg_list.push_back(i);
  if (reg_list.size() <= static_cast<size_t>(local_ba_window))
    return reg_list;
  std::unordered_map<int, int> shared_count;
  for (int im : reg_list) {
    if (optimize_set.count(im))
      continue;
    std::vector<int> track_ids;
    store.get_image_track_observations(im, &track_ids, nullptr);
    int count = 0;
    for (int tid : track_ids) {
      if (!store.track_has_triangulated_xyz(tid))
        continue;
      std::vector<Observation> obs_buf;
      store.get_track_observations(tid, &obs_buf);
      for (const auto& o : obs_buf) {
        int oim = static_cast<int>(o.image_index);
        if (optimize_set.count(oim)) {
          ++count;
          break;
        }
      }
    }
    shared_count[im] = count;
  }
  std::vector<std::pair<int, int>> by_count(shared_count.begin(), shared_count.end());
  std::sort(by_count.begin(), by_count.end(),
            [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
              return a.second > b.second;
            });
  for (const auto& p : by_count) {
    if (static_cast<int>(optimize_set.size()) >= local_ba_window)
      break;
    optimize_set.insert(p.first);
  }
  return std::vector<int>(optimize_set.begin(), optimize_set.end());
}

// int filter_tracks_two_view(TrackStore* store, int im0, int im1, const Eigen::Vector3d& C,
//                            int min_observations, double min_angle_deg) {
//   if (!store)
//     return 0;
//   const uint32_t uim0 = static_cast<uint32_t>(im0), uim1 = static_cast<uint32_t>(im1);
//   const double min_angle_rad = min_angle_deg * (3.141592653589793 / 180.0);
//   int marked = 0;
//   std::vector<Observation> obs_buf;
//   for (size_t ti = 0; ti < store->num_tracks(); ++ti) {
//     const int track_id = static_cast<int>(ti);
//     if (!store->is_track_valid(track_id))
//       continue;
//     obs_buf.clear();
//     store->get_track_observations(track_id, &obs_buf);
//     int n_in_pair = 0;
//     for (const auto& o : obs_buf) {
//       if (o.image_index == uim0 || o.image_index == uim1)
//         ++n_in_pair;
//     }
//     if (n_in_pair < min_observations) {
//       store->mark_track_deleted(track_id);
//       ++marked;
//       continue;
//     }
//     if (n_in_pair >= 2 && min_angle_deg > 0 && store->track_has_triangulated_xyz(track_id)) {
//       float tx, ty, tz;
//       store->get_track_xyz(track_id, &tx, &ty, &tz);
//       Eigen::Vector3d X(tx, ty, tz);
//       Eigen::Vector3d r0 = X.normalized();
//       Eigen::Vector3d r1 = (X - C).normalized();
//       double cos_a = std::max(-1.0, std::min(1.0, r0.dot(r1)));
//       if (std::acos(cos_a) < min_angle_rad) {
//         store->mark_track_deleted(track_id);
//         ++marked;
//       }
//     }
//   }
//   return marked;
// }

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
// Returns kOk and fills R/t on success; otherwise returns status and leaves R/t untouched.
GeoLoadStatus load_pair_geo_flexible(const std::string& dir, int im0, int im1, Eigen::Matrix3d* R,
                                     Eigen::Vector3d* t) {
  const int lo = std::min(im0, im1);
  const int hi = std::max(im0, im1);
  const std::string path = dir + std::to_string(lo) + "_" + std::to_string(hi) + ".isat_geo";
  Eigen::Matrix3d R_file;
  Eigen::Vector3d t_file;
  const GeoLoadStatus st = load_pair_geo_ex(path, &R_file, &t_file);
  if (st != GeoLoadStatus::kOk)
    return st;
  if (im0 == lo) {
    // File stores lo→hi; caller wants im0→im1, same direction
    *R = R_file;
    *t = t_file;
  } else {
    // File stores lo→hi (= im1→im0); invert so im0 is reference
    *R = R_file.transpose();
    *t = -(R_file.transpose() * t_file);
  }
  return GeoLoadStatus::kOk;
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

  // 1. Load geo (handles im0>im1 by inverting; file is always stored as lo_hi.isat_geo)
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  {
    const GeoLoadStatus st = load_pair_geo_flexible(dir, im0, im1, &R, &t);
    if (st != GeoLoadStatus::kOk) {
      if (st == GeoLoadStatus::kNoTwoview) {
        // File exists but was written without --twoview; R/t not available.
        // Re-run isat_geo with --twoview to enable initial pair selection.
        DLOG(INFO) << "    pair (" << im0 << "," << im1
                   << "): geo file exists but no twoview R/t (run isat_geo --twoview)";
      } else {
        DLOG(INFO) << "    pair (" << im0 << "," << im1 << "): geo file not found";
      }
      return result;
    }
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
    {
      const size_t show_n = std::min(second_images.size(), size_t(5));
      for (size_t si = 0; si < show_n; ++si) {
        const int im2c = second_images[si].first;
        const int f_inl = view_graph.get_F_inliers(static_cast<uint32_t>(im1),
                                                    static_cast<uint32_t>(im2c));
        LOG(INFO) << "    second #" << si << ": image " << im2c
                  << " (shared=" << second_images[si].second << ", F_inliers=" << f_inl << ")";
      }
      if (second_images.size() > show_n)
        LOG(INFO) << "    ... (" << second_images.size() - show_n << " more)";
    }

    const size_t max_second = std::min(second_images.size(), size_t(50));
    for (size_t si = 0; si < max_second; ++si) {
      const int im2 = second_images[si].first;
      auto pair_key = std::make_pair(std::min(im1, im2), std::max(im1, im2));
      if (tried_pairs.count(pair_key))
        continue;
      tried_pairs.insert(pair_key);

      // Skip pairs flagged as degenerate in view graph (pure rotation / planar scene)
      {
        const int f_inl = view_graph.get_F_inliers(static_cast<uint32_t>(im1),
                                                    static_cast<uint32_t>(im2));
        // Look up full PairGeoInfo for degeneracy flag
        bool skip_deg = false;
        for (size_t pi = 0; pi < view_graph.num_pairs(); ++pi) {
          const auto& pg = view_graph.pair_at(pi);
          const uint32_t lo = std::min(static_cast<uint32_t>(im1), static_cast<uint32_t>(im2));
          const uint32_t hi = std::max(static_cast<uint32_t>(im1), static_cast<uint32_t>(im2));
          if (pg.image1_index == lo && pg.image2_index == hi) {
            if (pg.is_degenerate) {
              skip_deg = true;
              DLOG(INFO) << "    pair (" << im1 << "," << im2
                         << "): skip (degenerate, F_inliers=" << f_inl << ")";
            }
            break;
          }
        }
        if (skip_deg)
          continue;
      }

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
    A.row(2 * i) << nx * R.row(1) - ny * R.row(0), nx * t(1) - ny * t(0);
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
                                             int min_3d2d_count, double batch_ratio,
                                             int batch_max) {
  using Clock = std::chrono::steady_clock;
  const int n_images = store.num_images();
  if (n_images == 0 || static_cast<int>(registered.size()) != n_images || batch_max <= 0)
    return {};

  auto t0 = Clock::now();
  // Score every unregistered image by its number of triangulated 3D-2D correspondences.
  std::vector<std::pair<int, int>> unreg_count; // (count, global_image_idx)
  int total_track_obs_checked = 0;
  for (int im = 0; im < n_images; ++im) {
    if (registered[static_cast<size_t>(im)])
      continue;
    std::vector<int> track_ids;
    store.get_image_track_observations(im, &track_ids, nullptr);
    total_track_obs_checked += static_cast<int>(track_ids.size());
    int count = 0;
    for (int tid : track_ids)
      if (store.track_has_triangulated_xyz(tid))
        ++count;
    if (count >= min_3d2d_count)
      unreg_count.push_back({count, im});
  }
  auto t1 = Clock::now();

  if (unreg_count.empty()) {
    LOG(INFO) << "[PERF] choose_next_resection_batch: no candidates above min_3d2d=" << min_3d2d_count
              << "  total_obs_checked=" << total_track_obs_checked;
    return {};
  }

  // Sort descending by score.
  std::sort(unreg_count.begin(), unreg_count.end(),
            [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
              return a.first > b.first; // higher count first
            });

  // OpenMVG 75%-ratio threshold: include all within ratio of best.
  const int best_count = unreg_count[0].first;
  const int ratio_floor = static_cast<int>(std::ceil(best_count * batch_ratio));
  std::vector<int> out;
  for (const auto& p : unreg_count) {
    if (p.first < ratio_floor || static_cast<int>(out.size()) >= batch_max)
      break;
    out.push_back(p.second);
  }

  auto ms_count = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
  LOG(INFO) << "[PERF] choose_next_resection_batch: " << unreg_count.size()
            << " candidates (above min_3d2d=" << min_3d2d_count << "), best=" << best_count
            << ", ratio_floor=" << ratio_floor << ", selected=" << out.size()
            << ", total_obs_checked=" << total_track_obs_checked
            << ", count_loop=" << ms_count << "ms";
  for (size_t i = 0; i < std::min(unreg_count.size(), size_t(5)); ++i) {
    LOG(INFO) << "  candidate #" << i << ": image " << unreg_count[i].second << " ("
              << unreg_count[i].first << " 3D-2D correspondences)"
              << (i < out.size() ? "  [selected]" : "  [below ratio]");
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
                            double min_tri_angle_deg,
                            std::vector<int>* new_track_ids_out) {
  if (new_track_ids_out) new_track_ids_out->clear();
  using Clock = std::chrono::steady_clock;
  if (!store)
    return 0;
  const int n_images = store->num_images();
  if (static_cast<int>(poses_R.size()) != n_images ||
      static_cast<int>(poses_C.size()) != n_images ||
      static_cast<int>(registered.size()) != n_images)
    return 0;
  // ── Collect candidate tracks from new images' reverse index ─────────────────
  // Visiting only new-image observations instead of all tracks reduces the scan
  // from O(#all_tracks) to ≈ O(#new_obs).  has_new is guaranteed by construction.
  std::unordered_set<int> candidate_set;
  {
    std::vector<int> tid_buf;
    for (int new_im : new_registered_image_indices) {
      tid_buf.clear();
      store->get_image_track_observations(new_im, &tid_buf, nullptr);
      for (int tid : tid_buf) {
        if (store->is_track_valid(tid) && !store->track_has_triangulated_xyz(tid))
          candidate_set.insert(tid);
      }
    }
  }
  int updated = 0;
  int tracks_scanned = 0, tracks_skipped_few_views = 0;
  int tracks_skipped_depth = 0, tracks_skipped_angle = 0;
  auto t0 = Clock::now();
  std::vector<Observation> obs_buf;
  for (int track_id : candidate_set) {
    ++tracks_scanned;
    obs_buf.clear();
    store->get_track_observations(track_id, &obs_buf);
    // has_new is guaranteed: candidate was collected from a new-image reverse index.
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
      ++tracks_skipped_few_views;
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
      ++tracks_skipped_depth;
      continue;
    }
    // Angle filter: reject degenerate (near-zero parallax) triangulations
    if (compute_max_ray_angle_deg(X, C_list) < min_tri_angle_deg) {
      ++tracks_skipped_angle;
      continue;
    }
    store->set_track_xyz(track_id, static_cast<float>(X(0)), static_cast<float>(X(1)),
                         static_cast<float>(X(2)));
    ++updated;
    if (new_track_ids_out) new_track_ids_out->push_back(track_id);
  }
  auto t1 = std::chrono::steady_clock::now();
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
  LOG(INFO) << "[PERF] run_batch_triangulation: " << ms << "ms"
            << "  total_tracks=" << store->num_tracks()
            << "  candidates=" << candidate_set.size()
            << "  scanned=" << tracks_scanned
            << "  skip_few_views=" << tracks_skipped_few_views
            << "  skip_depth=" << tracks_skipped_depth
            << "  skip_angle=" << tracks_skipped_angle
            << "  newly_tri=" << updated;
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
  using Clock = std::chrono::steady_clock;
  auto t0 = Clock::now();
  // Full scan: try every track that does NOT yet have a valid 3D position.
  // This covers two cases:
  //   (a) tracks never triangulated (angle/depth filters previously rejected them)
  //   (b) tracks whose XYZ was cleared by reject_outliers after losing all obs support
  // The flag/pending mechanism is no longer used here; it is kept in TrackStore for
  // external callers but drain_retriangulation_pending() is called here to keep the
  // pending list tidy.
  std::vector<int> dummy_pending;
  store->drain_retriangulation_pending(&dummy_pending); // drain to prevent unbounded growth
  int updated = 0;
  int scanned = 0, skip_few = 0, skip_depth = 0, skip_angle = 0;
  std::vector<Observation> obs_buf;
  for (size_t ti = 0; ti < store->num_tracks(); ++ti) {
    const int track_id = static_cast<int>(ti);
    if (!store->is_track_valid(track_id) || store->track_has_triangulated_xyz(track_id))
      continue; // skip already-triangulated tracks
    ++scanned;
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
      ++skip_few;
      continue;
    }
    std::vector<Eigen::Matrix3d> R_list;
    std::vector<Eigen::Vector3d> C_list;
    for (int im : reg_inds) {
      R_list.push_back(poses_R[static_cast<size_t>(im)]);
      C_list.push_back(poses_C[static_cast<size_t>(im)]);
    }
    Eigen::Vector3d X = triangulate_point_multiview(R_list, C_list, rays_n);
    if (!check_all_depths_positive(X, R_list, C_list)) {
      ++skip_depth;
      continue;
    }
    if (compute_max_ray_angle_deg(X, C_list) < min_tri_angle_deg) {
      ++skip_angle;
      continue;
    }
    store->set_track_xyz(track_id, static_cast<float>(X(0)), static_cast<float>(X(1)),
                         static_cast<float>(X(2)));
    ++updated;
  }
  auto t1 = std::chrono::steady_clock::now();
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
  LOG(INFO) << "[PERF] run_retriangulation: " << ms << "ms"
            << "  total_tracks=" << store->num_tracks()
            << "  scanned_untri=" << scanned
            << "  skip_few_views=" << skip_few
            << "  skip_depth=" << skip_depth
            << "  skip_angle=" << skip_angle
            << "  re_tri=" << updated;
  return updated;
}

// ─── BA: build from Store, write back ────────────────────────────────────────
namespace {

bool build_ba_input_from_store(
    const TrackStore& store, const std::vector<Eigen::Matrix3d>& poses_R,
    const std::vector<Eigen::Vector3d>& poses_C, const std::vector<bool>& registered,
    const std::vector<int>& image_to_camera_index, const std::vector<camera::Intrinsics>& cameras,
    const std::vector<int>* image_subset, std::vector<int>* ba_image_index_to_global_out,
    BAInput* ba_input_out, std::vector<int>* point_index_to_track_id_out) {
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

  std::set<int> ba_global_set(global_indices.begin(), global_indices.end());
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

  for (size_t ti = 0; ti < store.num_tracks(); ++ti) {
    const int track_id = static_cast<int>(ti);
    if (!store.is_track_valid(track_id) || !store.track_has_triangulated_xyz(track_id))
      continue;
    obs_buf.clear();
    store.get_track_observations(track_id, &obs_buf);
    int visible_in_ba = 0;
    for (const auto& o : obs_buf) {
      int im = static_cast<int>(o.image_index);
      if (ba_global_set.count(im))
        ++visible_in_ba;
    }
    if (visible_in_ba < 2)
      continue;
    const int pt_idx = static_cast<int>(point_index_to_track_id_out->size());
    track_id_to_point_index[track_id] = pt_idx;
    point_index_to_track_id_out->push_back(track_id);
    float x, y, z;
    store.get_track_xyz(track_id, &x, &y, &z);
    ba.points3d.emplace_back(static_cast<double>(x), static_cast<double>(y),
                             static_cast<double>(z));
  }
  ba.fix_point.resize(ba.points3d.size(), false);

  for (size_t ti = 0; ti < store.num_tracks(); ++ti) {
    const int track_id = static_cast<int>(ti);
    const int pt_idx = track_id_to_point_index[track_id];
    if (pt_idx < 0)
      continue;
    obs_buf.clear();
    store.get_track_observations(track_id, &obs_buf);
    for (const auto& o : obs_buf) {
      int g = static_cast<int>(o.image_index);
      auto it = std::lower_bound(global_indices.begin(), global_indices.end(), g);
      if (it == global_indices.end() || *it != g)
        continue;
      int ba_im = static_cast<int>(it - global_indices.begin());
      double scale = (o.scale > 1e-6f) ? static_cast<double>(o.scale) : 1.0;
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

/// COLMAP-style local BA input builder.
///
/// Expansion from `batch` (newly registered images):
///   Hop-1  seed_points  = triangulated tracks visible from any image in batch.
///   Hop-2  variable_set = registered cameras observing seed_points, sorted by
///                         shared-point count descending, capped at max_variable_cameras.
///   Hop-3  constant_set = remaining registered observers of seed_points (frozen).
///
/// BAInput produced:
///   images  = variable_set ∪ constant_set (sorted by global index)
///   points  = seed_points with ≥2 observations in that image set
///   fix_pose: false for variable, true for constant
///   fix_intrinsics: all fixed (local BA never touches intrinsics)
static bool build_ba_input_colmap_local(
    const TrackStore& store, const std::vector<Eigen::Matrix3d>& poses_R,
    const std::vector<Eigen::Vector3d>& poses_C, const std::vector<bool>& registered,
    const std::vector<int>& image_to_camera_index, const std::vector<camera::Intrinsics>& cameras,
    const std::vector<int>& batch, int max_variable_cameras,
    std::vector<int>* ba_image_index_to_global_out, BAInput* ba_input_out,
    std::vector<int>* point_index_to_track_id_out) {
  if (!ba_input_out || !ba_image_index_to_global_out || !point_index_to_track_id_out)
    return false;
  const int n_images = store.num_images();
  const size_t n_tracks = store.num_tracks();

  // ── Hop 1: seed tracks visible from at least one batch image ─────────────
  std::set<int> batch_set(batch.begin(), batch.end());
  std::vector<int> seed_track_ids;
  seed_track_ids.reserve(n_tracks / 4);
  std::vector<Observation> obs_buf;
  for (size_t ti = 0; ti < n_tracks; ++ti) {
    const int track_id = static_cast<int>(ti);
    if (!store.is_track_valid(track_id) || !store.track_has_triangulated_xyz(track_id))
      continue;
    obs_buf.clear();
    store.get_track_observations(track_id, &obs_buf);
    for (const auto& o : obs_buf) {
      if (batch_set.count(static_cast<int>(o.image_index))) {
        seed_track_ids.push_back(track_id);
        break;
      }
    }
  }
  if (seed_track_ids.empty())
    return false;

  // ── Hop 2: score cameras by number of seed tracks observed ───────────────
  std::unordered_map<int, int> cam_score;
  for (int track_id : seed_track_ids) {
    obs_buf.clear();
    store.get_track_observations(track_id, &obs_buf);
    for (const auto& o : obs_buf) {
      int g = static_cast<int>(o.image_index);
      if (g >= 0 && g < n_images && registered[static_cast<size_t>(g)])
        cam_score[g]++;
    }
  }
  // Sort descending by score; take top max_variable_cameras.
  std::vector<std::pair<int, int>> score_list; // (score, global_idx)
  score_list.reserve(cam_score.size());
  for (const auto& kv : cam_score)
    score_list.push_back({kv.second, kv.first});
  std::sort(score_list.begin(), score_list.end(),
            [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
              return a.first > b.first;
            });
  // Batch images MUST be variable: their pose is a raw PnP result not yet BA-refined.
  // Freezing any batch image as a constant would corrupt the BA (its stale pose acts as a
  // wrong anchor and pulls 3D points to wrong positions).
  // Strategy: fill the variable set from batch first, then top-scored historical cameras
  // up to max_variable_cameras total.
  std::set<int> variable_set;
  for (int g : batch)
    if (cam_score.count(g))
      variable_set.insert(g);
  const int historical_slots =
      std::max(0, max_variable_cameras - static_cast<int>(variable_set.size()));
  int historical_taken = 0;
  for (const auto& p : score_list) {
    if (historical_taken >= historical_slots)
      break;
    if (!variable_set.count(p.second)) {
      variable_set.insert(p.second);
      ++historical_taken;
    }
  }

  // ── Hop 3: constant cameras – observe seed points but outside variable set ─
  std::set<int> constant_set;
  for (const auto& kv : cam_score)
    if (!variable_set.count(kv.first))
      constant_set.insert(kv.first);

  // ── Union image list (sorted) ─────────────────────────────────────────────
  std::set<int> all_image_set;
  for (int g : variable_set) all_image_set.insert(g);
  for (int g : constant_set) all_image_set.insert(g);
  std::vector<int>& global_indices = *ba_image_index_to_global_out;
  global_indices.assign(all_image_set.begin(), all_image_set.end());
  if (global_indices.empty())
    return false;

  // ── Populate BAInput ──────────────────────────────────────────────────────
  BAInput& ba = *ba_input_out;
  ba.poses_R.clear(); ba.poses_C.clear(); ba.points3d.clear();
  ba.observations.clear(); ba.image_camera_index.clear();
  ba.cameras = cameras;
  ba.fix_pose.clear(); ba.fix_point.clear(); ba.fix_intrinsics_flags.clear();
  ba.optimize_intrinsics = false;
  for (int g : global_indices) {
    ba.poses_R.push_back(poses_R[static_cast<size_t>(g)]);
    ba.poses_C.push_back(poses_C[static_cast<size_t>(g)]);
    const int cam_idx = image_to_camera_index[static_cast<size_t>(g)];
    ba.image_camera_index.push_back(
        cam_idx >= 0 && cam_idx < static_cast<int>(cameras.size()) ? cam_idx : 0);
  }
  const int n_ba_images = static_cast<int>(global_indices.size());
  // variable cameras free, constant cameras frozen.
  ba.fix_pose.resize(static_cast<size_t>(n_ba_images), false);
  for (int bi = 0; bi < n_ba_images; ++bi)
    ba.fix_pose[static_cast<size_t>(bi)] = (constant_set.count(global_indices[bi]) > 0);
  ba.fix_intrinsics_flags.resize(ba.cameras.size(),
                                  static_cast<uint32_t>(FixIntrinsicsMask::kFixIntrAll));

  // ── Seed points: tracks with ≥2 observations within all_image_set ─────────
  std::vector<int> track_id_to_point_index(n_tracks, -1);
  point_index_to_track_id_out->clear();
  for (int track_id : seed_track_ids) {
    obs_buf.clear();
    store.get_track_observations(track_id, &obs_buf);
    int visible_in_ba = 0;
    for (const auto& o : obs_buf)
      if (all_image_set.count(static_cast<int>(o.image_index)))
        ++visible_in_ba;
    if (visible_in_ba < 2)
      continue;
    const int pt_idx = static_cast<int>(point_index_to_track_id_out->size());
    track_id_to_point_index[track_id] = pt_idx;
    point_index_to_track_id_out->push_back(track_id);
    float x, y, z;
    store.get_track_xyz(track_id, &x, &y, &z);
    ba.points3d.emplace_back(static_cast<double>(x), static_cast<double>(y),
                             static_cast<double>(z));
  }
  if (ba.points3d.empty())
    return false;
  ba.fix_point.resize(ba.points3d.size(), false);

  // ── Observations ─────────────────────────────────────────────────────────
  for (size_t ti = 0; ti < n_tracks; ++ti) {
    const int track_id = static_cast<int>(ti);
    const int pt_idx = track_id_to_point_index[track_id];
    if (pt_idx < 0)
      continue;
    obs_buf.clear();
    store.get_track_observations(track_id, &obs_buf);
    for (const auto& o : obs_buf) {
      int g = static_cast<int>(o.image_index);
      auto it = std::lower_bound(global_indices.begin(), global_indices.end(), g);
      if (it == global_indices.end() || *it != g)
        continue;
      int ba_im = static_cast<int>(it - global_indices.begin());
      double scale = (o.scale > 1e-6f) ? static_cast<double>(o.scale) : 1.0;
      ba.observations.push_back(
          {ba_im, pt_idx, static_cast<double>(o.u), static_cast<double>(o.v), scale});
    }
  }
  return !ba.observations.empty();
}

} // namespace

bool run_global_ba(TrackStore* store, std::vector<Eigen::Matrix3d>* poses_R,
                   std::vector<Eigen::Vector3d>* poses_C, const std::vector<bool>& registered,
                   const std::vector<int>& image_to_camera_index,
                   const std::vector<camera::Intrinsics>& cameras,
                   std::vector<camera::Intrinsics>* cameras_in_out, bool optimize_intrinsics,
                   int max_iterations, double* rmse_px_out, int anchor_image,
                   const std::vector<bool>* camera_frozen,
                   uint32_t partial_intr_fix, double focal_prior_weight) {
  if (!store || !poses_R || !poses_C)
    return false;
  std::vector<int> ba_image_index_to_global;
  std::vector<int> point_index_to_track_id;
  BAInput ba_in;
  if (!build_ba_input_from_store(*store, *poses_R, *poses_C, registered, image_to_camera_index,
                                 cameras, nullptr, &ba_image_index_to_global, &ba_in,
                                 &point_index_to_track_id))
    return false;
  // Build per-camera intrinsics fix flags.
  // camera_frozen: frozen cameras keep kFixIntrAll (Schur-complement sparsity benefit).
  // partial_intr_fix: additional fix bits OR-ed into every non-frozen camera's flags,
  //   e.g. kFixIntrK3|kFixIntrP1|kFixIntrP2 during the stage-limited phase.
  {
    ba_in.fix_intrinsics_flags.assign(ba_in.cameras.size(), 0u);
    if (!optimize_intrinsics) {
      ba_in.fix_intrinsics_flags.assign(
          ba_in.cameras.size(), static_cast<uint32_t>(FixIntrinsicsMask::kFixIntrAll));
    } else {
      for (size_t c = 0; c < ba_in.cameras.size(); ++c) {
        const bool frozen = (camera_frozen && c < camera_frozen->size() && (*camera_frozen)[c]);
        ba_in.fix_intrinsics_flags[c] = frozen
            ? static_cast<uint32_t>(FixIntrinsicsMask::kFixIntrAll)
            : partial_intr_fix;
      }
    }
    bool any_variable = false;
    for (const uint32_t f : ba_in.fix_intrinsics_flags)
      if (f != static_cast<uint32_t>(FixIntrinsicsMask::kFixIntrAll)) { any_variable = true; break; }
    ba_in.optimize_intrinsics = any_variable;
    ba_in.focal_prior_weight = focal_prior_weight;
  }
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
  }
  LOG(INFO) << "run_global_ba: " << ba_in.poses_R.size() << " images, " << ba_in.points3d.size()
            << " points, " << ba_in.observations.size() << " obs, max_iter=" << max_iterations;
  BAResult ba_out;
  if (!global_bundle_analytic(ba_in, &ba_out, max_iterations)) {
    LOG(WARNING) << "run_global_ba: solver failed";
    return false;
  }
  LOG(INFO) << "run_global_ba: RMSE=" << ba_out.rmse_px << " px, success=" << ba_out.success;
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
                  const std::vector<int>* indices_to_optimize, int anchor_image) {
  if (!store || !poses_R || !poses_C || local_ba_window <= 0)
    return false;
  std::set<int> optimize_set;
  if (indices_to_optimize && !indices_to_optimize->empty()) {
    for (int im : *indices_to_optimize)
      if (im >= 0 && im < store->num_images() && registered[static_cast<size_t>(im)])
        optimize_set.insert(im);
  }
  if (optimize_set.empty()) {
    std::vector<int> reg_indices;
    const int n_images = store->num_images();
    for (int i = 0; i < n_images; ++i)
      if (registered[static_cast<size_t>(i)])
        reg_indices.push_back(i);
    if (reg_indices.empty())
      return false;
    const int n_optimize = std::min(local_ba_window, static_cast<int>(reg_indices.size()));
    optimize_set.insert(reg_indices.end() - static_cast<size_t>(n_optimize), reg_indices.end());
  }
  // Always freeze the anchor (gauge fix): connectivity-based selection may include it.
  if (anchor_image >= 0)
    optimize_set.erase(anchor_image);

  std::vector<int> ba_image_index_to_global;
  std::vector<int> point_index_to_track_id;
  BAInput ba_in;
  if (!build_ba_input_from_store(*store, *poses_R, *poses_C, registered, image_to_camera_index,
                                 cameras, nullptr, &ba_image_index_to_global, &ba_in,
                                 &point_index_to_track_id))
    return false;
  ba_in.optimize_intrinsics = false;
  for (size_t i = 0; i < ba_image_index_to_global.size(); ++i)
    ba_in.fix_pose[i] = (optimize_set.count(ba_image_index_to_global[i]) == 0);
  ba_in.fix_intrinsics_flags.resize(ba_in.cameras.size(),
                                    static_cast<uint32_t>(FixIntrinsicsMask::kFixIntrAll));
  int n_optimized = 0;
  for (size_t i = 0; i < ba_in.fix_pose.size(); ++i)
    if (!ba_in.fix_pose[i])
      ++n_optimized;
  LOG(INFO) << "run_local_ba: " << ba_image_index_to_global.size() << " images (" << n_optimized
            << " optimized), " << ba_in.points3d.size() << " points";
  BAResult ba_out;
  if (!global_bundle_analytic(ba_in, &ba_out, max_iterations)) {
    LOG(WARNING) << "run_local_ba: solver failed";
    return false;
  }
  LOG(INFO) << "run_local_ba: RMSE=" << ba_out.rmse_px << " px";
  write_ba_result_back(ba_out, ba_image_index_to_global, point_index_to_track_id, store, poses_R,
                       poses_C, nullptr);
  // Intrinsics are fixed in local BA, but log current values for tracking.
  log_cameras(cameras, "run_local_ba");
  if (rmse_px_out)
    *rmse_px_out = ba_out.rmse_px;
  return true;
}

bool run_local_ba_colmap(TrackStore* store, std::vector<Eigen::Matrix3d>* poses_R,
                         std::vector<Eigen::Vector3d>* poses_C,
                         const std::vector<bool>& registered,
                         const std::vector<int>& image_to_camera_index,
                         const std::vector<camera::Intrinsics>& cameras,
                         const std::vector<int>& batch, int max_variable_cameras,
                         int max_iterations, double* rmse_px_out) {
  if (!store || !poses_R || !poses_C || batch.empty())
    return false;
  std::vector<int> ba_image_index_to_global;
  std::vector<int> point_index_to_track_id;
  BAInput ba_in;
  if (!build_ba_input_colmap_local(*store, *poses_R, *poses_C, registered, image_to_camera_index,
                                   cameras, batch, max_variable_cameras,
                                   &ba_image_index_to_global, &ba_in, &point_index_to_track_id))
    return false;
  int n_variable = 0, n_constant = 0;
  for (bool f : ba_in.fix_pose) {
    if (f) ++n_constant; else ++n_variable;
  }
  LOG(INFO) << "run_local_ba_colmap: variable=" << n_variable << " constant=" << n_constant
            << " images=" << ba_image_index_to_global.size()
            << " points=" << ba_in.points3d.size()
            << " obs=" << ba_in.observations.size();
  BAResult ba_out;
  if (!global_bundle_analytic(ba_in, &ba_out, max_iterations)) {
    LOG(WARNING) << "run_local_ba_colmap: solver failed";
    return false;
  }
  LOG(INFO) << "run_local_ba_colmap: RMSE=" << ba_out.rmse_px << " px";
  write_ba_result_back(ba_out, ba_image_index_to_global, point_index_to_track_id, store, poses_R,
                       poses_C, nullptr);
  if (rmse_px_out)
    *rmse_px_out = ba_out.rmse_px;
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// kBatchNeighbor local BA
// ─────────────────────────────────────────────────────────────────────────────

/// Build BA input for the neighbor-anchored strategy.
/// Variable cameras : batch.
/// Variable 3D points: new_track_ids (just triangulated this iteration).
/// Constant cameras : union of per-batch-image top-neighbor_k historical neighbors
///                    (ranked by shared triangulated point count).
/// Constant 3D points: old triangulated tracks visible from ≥1 batch AND ≥1 constant camera.
static bool build_ba_input_batch_neighbor(
    const TrackStore& store, const std::vector<Eigen::Matrix3d>& poses_R,
    const std::vector<Eigen::Vector3d>& poses_C, const std::vector<bool>& registered,
    const std::vector<int>& image_to_camera_index, const std::vector<camera::Intrinsics>& cameras,
    const std::vector<int>& batch, const std::vector<int>& new_track_ids, int neighbor_k,
    std::vector<int>* ba_image_index_to_global_out, BAInput* ba_input_out,
    std::vector<int>* point_index_to_track_id_out) {
  if (!ba_input_out || !ba_image_index_to_global_out || !point_index_to_track_id_out)
    return false;
  const int n_images = store.num_images();
  const size_t n_tracks = store.num_tracks();
  std::set<int> batch_set(batch.begin(), batch.end());
  std::set<int> new_track_set(new_track_ids.begin(), new_track_ids.end());

  // ── Per-batch-image top-K neighbors ──────────────────────────────────────
  std::set<int> constant_set;
  std::vector<int> track_ids_buf;
  std::vector<Observation> obs_buf;
  for (int b : batch) {
    // Count shared triangulated tracks between b and each registered historical image.
    std::unordered_map<int, int> neighbor_score;
    track_ids_buf.clear();
    store.get_image_track_observations(b, &track_ids_buf, nullptr);
    for (int tid : track_ids_buf) {
      if (!store.is_track_valid(tid) || !store.track_has_triangulated_xyz(tid))
        continue;
      obs_buf.clear();
      store.get_track_observations(tid, &obs_buf);
      for (const auto& o : obs_buf) {
        int g = static_cast<int>(o.image_index);
        if (g >= 0 && g < n_images && registered[static_cast<size_t>(g)] && !batch_set.count(g))
          neighbor_score[g]++;
      }
    }
    // Sort descending; take top-neighbor_k for this batch image.
    std::vector<std::pair<int, int>> sorted(neighbor_score.begin(), neighbor_score.end());
    std::sort(sorted.begin(), sorted.end(),
              [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
                return a.second > b.second;
              });
    int taken = 0;
    for (const auto& p : sorted) {
      if (taken >= neighbor_k) break;
      constant_set.insert(p.first);
      ++taken;
    }
  }

  // ── Image sets ────────────────────────────────────────────────────────────
  std::set<int> all_image_set;
  for (int g : batch) all_image_set.insert(g);
  for (int g : constant_set) all_image_set.insert(g);
  if (all_image_set.empty()) return false;

  std::vector<int>& global_indices = *ba_image_index_to_global_out;
  global_indices.assign(all_image_set.begin(), all_image_set.end());

  // ── BAInput: poses + cameras ──────────────────────────────────────────────
  BAInput& ba = *ba_input_out;
  ba.poses_R.clear(); ba.poses_C.clear(); ba.points3d.clear();
  ba.observations.clear(); ba.image_camera_index.clear();
  ba.cameras = cameras;
  ba.fix_pose.clear(); ba.fix_point.clear(); ba.fix_intrinsics_flags.clear();
  ba.optimize_intrinsics = false;
  for (int g : global_indices) {
    ba.poses_R.push_back(poses_R[static_cast<size_t>(g)]);
    ba.poses_C.push_back(poses_C[static_cast<size_t>(g)]);
    const int cam_idx = image_to_camera_index[static_cast<size_t>(g)];
    ba.image_camera_index.push_back(
        cam_idx >= 0 && cam_idx < static_cast<int>(cameras.size()) ? cam_idx : 0);
  }
  const int n_ba_images = static_cast<int>(global_indices.size());
  ba.fix_pose.resize(static_cast<size_t>(n_ba_images), false);
  for (int bi = 0; bi < n_ba_images; ++bi)
    ba.fix_pose[static_cast<size_t>(bi)] = (constant_set.count(global_indices[bi]) > 0);
  ba.fix_intrinsics_flags.resize(ba.cameras.size(),
                                  static_cast<uint32_t>(FixIntrinsicsMask::kFixIntrAll));

  // ── Points ────────────────────────────────────────────────────────────────
  // Variable: new_track_ids (just triangulated, XYZ needs refinement).
  // Constant: old tracks visible from ≥1 batch AND ≥1 constant camera (anchors).
  std::vector<int> track_id_to_point_index(n_tracks, -1);
  point_index_to_track_id_out->clear();

  // Helper: add a track if ≥2 obs in all_image_set. Returns true if added.
  auto try_add_track = [&](int track_id, bool fix_pt) -> bool {
    if (!store.is_track_valid(track_id) || !store.track_has_triangulated_xyz(track_id))
      return false;
    obs_buf.clear();
    store.get_track_observations(track_id, &obs_buf);
    int vis = 0;
    for (const auto& o : obs_buf)
      if (all_image_set.count(static_cast<int>(o.image_index))) ++vis;
    if (vis < 2) return false;
    const int pt_idx = static_cast<int>(point_index_to_track_id_out->size());
    track_id_to_point_index[track_id] = pt_idx;
    point_index_to_track_id_out->push_back(track_id);
    float x, y, z;
    store.get_track_xyz(track_id, &x, &y, &z);
    ba.points3d.emplace_back(static_cast<double>(x), static_cast<double>(y),
                             static_cast<double>(z));
    ba.fix_point.push_back(fix_pt);
    return true;
  };

  // Variable points: new tracks (order matters for readability only).
  for (int tid : new_track_ids) try_add_track(tid, false);

  // Constant anchor points: old tracks shared between batch and constant cameras.
  // Collect candidates from constant cameras' reverse index, then check batch visibility.
  std::unordered_set<int> old_candidates;
  for (int c : constant_set) {
    track_ids_buf.clear();
    store.get_image_track_observations(c, &track_ids_buf, nullptr);
    for (int tid : track_ids_buf)
      if (!new_track_set.count(tid)) old_candidates.insert(tid);
  }
  for (int tid : old_candidates) {
    // Must also be visible from at least one batch image to constrain the new camera pose.
    obs_buf.clear();
    store.get_track_observations(tid, &obs_buf);
    bool seen_from_batch = false;
    for (const auto& o : obs_buf)
      if (batch_set.count(static_cast<int>(o.image_index))) { seen_from_batch = true; break; }
    if (seen_from_batch) try_add_track(tid, true);
  }

  if (ba.points3d.empty()) return false;

  // ── Observations ─────────────────────────────────────────────────────────
  for (size_t ti = 0; ti < n_tracks; ++ti) {
    const int track_id = static_cast<int>(ti);
    const int pt_idx = track_id_to_point_index[track_id];
    if (pt_idx < 0) continue;
    obs_buf.clear();
    store.get_track_observations(track_id, &obs_buf);
    for (const auto& o : obs_buf) {
      int g = static_cast<int>(o.image_index);
      auto it = std::lower_bound(global_indices.begin(), global_indices.end(), g);
      if (it == global_indices.end() || *it != g) continue;
      int ba_im = static_cast<int>(it - global_indices.begin());
      double scale = (o.scale > 1e-6f) ? static_cast<double>(o.scale) : 1.0;
      ba.observations.push_back(
          {ba_im, pt_idx, static_cast<double>(o.u), static_cast<double>(o.v), scale});
    }
  }
  return !ba.observations.empty();
}

bool run_local_ba_batch_neighbor(TrackStore* store, std::vector<Eigen::Matrix3d>* poses_R,
                                 std::vector<Eigen::Vector3d>* poses_C,
                                 const std::vector<bool>& registered,
                                 const std::vector<int>& image_to_camera_index,
                                 const std::vector<camera::Intrinsics>& cameras,
                                 const std::vector<int>& batch,
                                 const std::vector<int>& new_track_ids, int neighbor_k,
                                 int max_iterations, double* rmse_px_out) {
  if (!store || !poses_R || !poses_C || batch.empty()) return false;
  std::vector<int> ba_image_index_to_global;
  std::vector<int> point_index_to_track_id;
  BAInput ba_in;
  if (!build_ba_input_batch_neighbor(*store, *poses_R, *poses_C, registered, image_to_camera_index,
                                     cameras, batch, new_track_ids, neighbor_k,
                                     &ba_image_index_to_global, &ba_in, &point_index_to_track_id))
    return false;
  int n_variable_cams = 0, n_constant_cams = 0, n_variable_pts = 0, n_constant_pts = 0;
  for (bool f : ba_in.fix_pose) { if (f) ++n_constant_cams; else ++n_variable_cams; }
  for (bool f : ba_in.fix_point) { if (f) ++n_constant_pts; else ++n_variable_pts; }
  LOG(INFO) << "run_local_ba_batch_neighbor: var_cams=" << n_variable_cams
            << " const_cams=" << n_constant_cams << " var_pts=" << n_variable_pts
            << " const_pts=" << n_constant_pts << " obs=" << ba_in.observations.size();
  BAResult ba_out;
  if (!global_bundle_analytic(ba_in, &ba_out, max_iterations)) {
    LOG(WARNING) << "run_local_ba_batch_neighbor: solver failed";
    return false;
  }
  LOG(INFO) << "run_local_ba_batch_neighbor: RMSE=" << ba_out.rmse_px << " px";
  write_ba_result_back(ba_out, ba_image_index_to_global, point_index_to_track_id, store, poses_R,
                       poses_C, nullptr);
  if (rmse_px_out) *rmse_px_out = ba_out.rmse_px;
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

  // count_tri_tracks: O(1) via TrackStore counter instead of full scan.
  auto count_tri_tracks = [&]() -> int {
    return store_out->num_triangulated_tracks();
  };
  LOG(INFO) << "Triangulated tracks after initial pair: " << count_tri_tracks();

  // Compute adaptive rejection threshold: max(fixed, prev_rmse * factor).
  // This prevents premature culling when RMSE is temporarily high (e.g. early intrinsics convergence).
  auto eff_reject_threshold = [&](double current_rmse) -> double {
    if (opts.outlier_adaptive_factor > 0.0 && current_rmse > 0.0)
      return std::max(opts.outlier_threshold_px, current_rmse * opts.outlier_adaptive_factor);
    return opts.outlier_threshold_px;
  };

  // Stage-based partial intrinsics: fix k3/p1/p2 until n_registered reaches threshold.
  // k3 (r⁶ term) is weakly observable in nadir imagery; p1/p2 tangential distortion similarly.
  auto k3p12_fix = [&]() -> uint32_t {
    if (opts.intrinsics_k3p12_free_min_images <= 0) return 0u;
    return (num_registered < opts.intrinsics_k3p12_free_min_images)
               ? static_cast<uint32_t>(FixIntrinsicsMask::kFixIntrK3 |
                                       FixIntrinsicsMask::kFixIntrP1 |
                                       FixIntrinsicsMask::kFixIntrP2)
               : 0u;
  };

  using Clock = std::chrono::steady_clock; // shared by lambdas and main loop below
  auto run_ba_and_reject_outliers = [&](double* rmse_out) {
    auto _t_rej_start = Clock::now();
    // Capture RMSE from the preceding BA (passed in via rmse_out) before resetting.
    double rmse = (rmse_out && *rmse_out > 0.0) ? *rmse_out : 0.0;
    if (rmse_out)
      *rmse_out = 0.0;
    bool do_reject = (num_registered >= opts.reject_min_registered_images);
    int rejected = 0;
    int rej_ba_rounds = 0;
    if (do_reject) {
      double thr = eff_reject_threshold(rmse);
      auto _t0 = Clock::now();
      rejected =
          reject_outliers_multiview(store_out, *poses_R_out, *poses_C_out, *registered_out,
                                    *cameras, image_to_camera_index, thr);
      auto _t1 = Clock::now();
      rejected += reject_outliers_angle_multiview(store_out, *poses_R_out, *poses_C_out,
                                                  *registered_out, opts.min_observation_angle_deg);
      auto _t2 = Clock::now();
      if (rejected > 0)
        LOG(INFO) << "  [PERF] outlier rejection round 0: " << rejected
                  << " obs marked (thr=" << thr << " px)"
                  << "  reproj=" << std::chrono::duration_cast<std::chrono::milliseconds>(_t1 - _t0).count() << "ms"
                  << "  angle=" << std::chrono::duration_cast<std::chrono::milliseconds>(_t2 - _t1).count() << "ms";
    }
    for (int r = 0; rejected >= std::max(1, opts.min_outliers_for_ba_retry) &&
                    r < opts.max_outlier_iterations - 1; ++r) {
      ++rej_ba_rounds;
      auto _t_rba0 = Clock::now();
      if (num_registered < opts.local_ba_after_n_images) {
        const bool opt_intr = opts.global_ba_optimize_intrinsics &&
                              num_registered >= opts.global_ba_optimize_intrinsics_min_images;
        if (!run_global_ba(store_out, poses_R_out, poses_C_out, *registered_out,
                           image_to_camera_index, *cameras, cameras, opt_intr,
                           opts.max_global_ba_iterations, &rmse, anchor_image,
                           nullptr, k3p12_fix(), opts.focal_prior_weight))
          break;
      } else {
        if (!run_local_ba(store_out, poses_R_out, poses_C_out, *registered_out,
                          image_to_camera_index, *cameras, opts.local_ba_window, 25, &rmse,
                          nullptr, anchor_image))
          break;
      }
      auto _t_rba1 = Clock::now();
      double thr = eff_reject_threshold(rmse);
      rejected =
          reject_outliers_multiview(store_out, *poses_R_out, *poses_C_out, *registered_out,
                                    *cameras, image_to_camera_index, thr);
      auto _t_rba2 = Clock::now();
      rejected += reject_outliers_angle_multiview(store_out, *poses_R_out, *poses_C_out,
                                                  *registered_out, opts.min_observation_angle_deg);
      auto _t_rba3 = Clock::now();
      LOG(INFO) << "  [PERF] reject_outliers round " << r + 1 << ": ba="
                << std::chrono::duration_cast<std::chrono::milliseconds>(_t_rba1 - _t_rba0).count() << "ms"
                << "  reproj=" << std::chrono::duration_cast<std::chrono::milliseconds>(_t_rba2 - _t_rba1).count() << "ms"
                << "  angle=" << std::chrono::duration_cast<std::chrono::milliseconds>(_t_rba3 - _t_rba2).count() << "ms"
                << "  rejected=" << rejected;
    }
    if (do_reject && opts.enable_sigma_filter && rmse > 0.01) {
      reject_outliers_multiview(store_out, *poses_R_out, *poses_C_out, *registered_out, *cameras,
                                image_to_camera_index, rmse * 3.0);
    }
    if (rmse_out)
      *rmse_out = rmse;
    LOG(INFO) << "[PERF] reject_outliers_total: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - _t_rej_start).count()
              << "ms  extra_ba_rounds=" << rej_ba_rounds;
  };

  // ─── Per-camera intrinsics freeze state ─────────────────────────────────────
  struct CamFreezeState {
    bool frozen = false;
    camera::Intrinsics prev;
    int stable_rounds = 0;
  };
  const int n_cameras = static_cast<int>(cameras->size());
  std::vector<CamFreezeState> cam_freeze(static_cast<size_t>(n_cameras));

  // Call after every global BA that may have updated intrinsics.
  // recheck_frozen=false (default): skip already-frozen cameras (fast path).
  // recheck_frozen=true: re-evaluate frozen cameras too — unfreeze if delta now exceeds
  //   thresholds (used after periodic global BA that re-opens all intrinsics).
  auto update_freeze_state = [&](bool recheck_frozen = false) {
    if (!opts.intrinsics_progressive_freeze) return;
    for (int c = 0; c < n_cameras; ++c) {
      auto& fs = cam_freeze[static_cast<size_t>(c)];
      const camera::Intrinsics& K = (*cameras)[static_cast<size_t>(c)];
      if (fs.frozen) {
        if (!recheck_frozen) continue;
        // Re-evaluate: unfreeze if intrinsics drifted beyond thresholds.
        const auto d = K.delta(fs.prev);
        if (!d.stable(opts.intrinsics_freeze_delta_focal, opts.intrinsics_freeze_delta_pp,
                      opts.intrinsics_freeze_delta_dist)) {
          fs.frozen = false;
          fs.stable_rounds = 0;
          LOG(INFO) << "  [freeze] camera " << c << " UNFROZEN"
                    << " focal_rel=" << d.focal_rel
                    << " pp_px=" << d.pp_px
                    << " dist=" << d.distortion;
        } else {
          LOG(INFO) << "  [freeze] camera " << c << " still FROZEN"
                    << " focal_rel=" << d.focal_rel
                    << " pp_px=" << d.pp_px
                    << " dist=" << d.distortion;
        }
        fs.prev = K;
        continue;
      }
      int cam_reg = 0;
      for (int i = 0; i < n_images; ++i)
        if ((*registered_out)[static_cast<size_t>(i)] &&
            image_to_camera_index[static_cast<size_t>(i)] == c)
          ++cam_reg;
      if (cam_reg < opts.intrinsics_freeze_min_images) {
        // Level-1 gate not met yet; don't count stable_rounds but still update prev.
        LOG(INFO) << "  [freeze] camera " << c << " L1-pending: reg=" << cam_reg
                  << "/" << opts.intrinsics_freeze_min_images << " images (need more)";
        fs.prev = K;
        continue;
      }
      const auto d = K.delta(fs.prev);
      if (d.stable(opts.intrinsics_freeze_delta_focal, opts.intrinsics_freeze_delta_pp,
                   opts.intrinsics_freeze_delta_dist)) {
        ++fs.stable_rounds;
        if (fs.stable_rounds >= opts.intrinsics_freeze_stable_rounds) {
          fs.frozen = true;
          LOG(INFO) << "  [freeze] camera " << c << " FROZEN"
                    << " focal_rel=" << d.focal_rel
                    << " pp_px=" << d.pp_px
                    << " dist=" << d.distortion
                    << " reg_imgs=" << cam_reg;
        } else {
          LOG(INFO) << "  [freeze] camera " << c << " stable_rounds=" << fs.stable_rounds
                    << "/" << opts.intrinsics_freeze_stable_rounds
                    << " focal_rel=" << d.focal_rel
                    << " pp_px=" << d.pp_px
                    << " dist=" << d.distortion
                    << " reg_imgs=" << cam_reg;
        }
      } else {
        LOG(INFO) << "  [freeze] camera " << c << " not-stable → reset"
                  << " focal_rel=" << d.focal_rel
                  << " (thr=" << opts.intrinsics_freeze_delta_focal << ")"
                  << " pp_px=" << d.pp_px
                  << " (thr=" << opts.intrinsics_freeze_delta_pp << ")"
                  << " dist=" << d.distortion
                  << " (thr=" << opts.intrinsics_freeze_delta_dist << ")"
                  << " reg_imgs=" << cam_reg;
        fs.stable_rounds = 0;
      }
      fs.prev = K;
    }
  };

  // Build per-camera frozen bool vector for passing to run_global_ba.
  auto make_frozen_vec = [&]() -> std::vector<bool> {
    std::vector<bool> v(static_cast<size_t>(n_cameras));
    for (int c = 0; c < n_cameras; ++c)
      v[static_cast<size_t>(c)] = cam_freeze[static_cast<size_t>(c)].frozen;
    return v;
  };

  // Registered count at last periodic global BA (local-BA phase tracker).
  int last_periodic_global_ba_registered = 0;
  int periodic_ba_count = 0; // counts mid-freq BA calls; triggers recalib every N
  // ───────────────────────────────────────────────────────────────────────────────
  // Warm-up GPU context before entering the main loop so the first resection
  // does not absorb the ~1-second EGL/shader-compile cost.
  resection_init_gpu();

  // COLMAP strategy registers exactly one image per iteration, then immediately runs
  // local BA on it (2-hop expansion centered on that single camera).  Batching >1
  // violates this design: the "batch" in run_local_ba_colmap is meant to be a single
  // newly-registered image, and its COLMAP-paper variable-set expansion only makes
  // sense when the batch contains just one new camera.
  const int effective_batch_max =
      (opts.local_ba_strategy == LocalBAStrategy::kColmap) ? 1 : opts.resection_batch_max;

  auto pipeline_start = Clock::now();
  int sfm_iter = 0;
  for (;;) {
    ++sfm_iter;
    auto iter_start = Clock::now();
    LOG(INFO) << "════ SfM iter #" << sfm_iter << ": registered=" << num_registered << "/"
              << n_images << ", tri_tracks=" << count_tri_tracks() << " ════";

    auto t_choose0 = Clock::now();
    std::vector<int> batch =
        choose_next_resection_batch(*store_out, *registered_out, opts.resection_min_3d2d_count,
                                    opts.resection_batch_ratio, effective_batch_max);
    auto t_choose1 = Clock::now();
    LOG(INFO) << "[PERF] choose_batch: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t_choose1 - t_choose0).count()
              << "ms";

    if (batch.empty()) {
      if (opts.retry_resection_after_cleanup &&
          num_registered >= opts.reject_min_registered_images) {
        double rmse = 0.0;
        // Cleanup BA: allow intrinsics only when enough cameras are registered.
        const bool cleanup_opt_intr =
            opts.global_ba_optimize_intrinsics &&
            num_registered >= opts.global_ba_optimize_intrinsics_min_images;
        const auto frozen_cleanup =
            (cleanup_opt_intr && opts.intrinsics_progressive_freeze) ? make_frozen_vec()
                                                                     : std::vector<bool>{};
        auto t_cba0 = Clock::now();
        if (run_global_ba(store_out, poses_R_out, poses_C_out, *registered_out,
                          image_to_camera_index, *cameras, cameras, cleanup_opt_intr,
                          opts.max_global_ba_iterations, &rmse, anchor_image,
                          cleanup_opt_intr ? &frozen_cleanup : nullptr,
                          cleanup_opt_intr ? k3p12_fix() : 0u, opts.focal_prior_weight)) {
          if (cleanup_opt_intr) update_freeze_state();
          LOG(INFO) << "[PERF] cleanup_global_ba: "
                    << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t_cba0).count()
                    << "ms  RMSE=" << rmse;
          double cleanup_thr = eff_reject_threshold(rmse);
          int rej =
              reject_outliers_multiview(store_out, *poses_R_out, *poses_C_out, *registered_out,
                                        *cameras, image_to_camera_index, cleanup_thr);
          rej += reject_outliers_angle_multiview(store_out, *poses_R_out, *poses_C_out,
                                                 *registered_out, opts.min_observation_angle_deg);
          if (rej > 0) {
            LOG(INFO) << "run_incremental_sfm_pipeline: cleanup BA + reject " << rej
                      << " (thr=" << cleanup_thr << " px), retry resection";
          batch =
                choose_next_resection_batch(*store_out, *registered_out,
                                            opts.resection_min_3d2d_count,
                                            opts.resection_batch_ratio, effective_batch_max);
          }
        }
      }
      if (batch.empty())
        break;
    }

    auto t_resect0 = Clock::now();
    int added = run_batch_resection(*store_out, batch, *cameras, image_to_camera_index, poses_R_out,
                                    poses_C_out, registered_out, opts.resection_min_inliers);
    LOG(INFO) << "[PERF] resection: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t_resect0).count()
              << "ms  added=" << added;
    if (added == 0) {
      LOG(WARNING) << "  No images could be resected in this batch, stopping";
      break;
    }

    auto t_tri0 = Clock::now();
    std::vector<int> new_track_ids;
    int n_new_tri =
        run_batch_triangulation(store_out, batch, *poses_R_out, *poses_C_out, *registered_out,
                                *cameras, image_to_camera_index, opts.min_tri_angle_deg,
                                &new_track_ids);
    LOG(INFO) << "[PERF] triangulation: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t_tri0).count()
              << "ms  new_tri=" << n_new_tri;

    num_registered = 0;
    for (bool r : *registered_out)
      if (r)
        ++num_registered;
    LOG(INFO) << "  After resection+triangulation: registered=" << num_registered
              << ", new_tri=" << n_new_tri << ", total_tri=" << count_tri_tracks();

    double rmse = 0.0;
    if (num_registered < opts.local_ba_after_n_images) {
      // Run global BA every global_ba_every_n_images registrations, and always
      // on the last image before switching to local BA.
      const bool do_global_ba =
          (opts.global_ba_every_n_images <= 1) ||
          (num_registered % opts.global_ba_every_n_images == 0) ||
          (num_registered + 1 == opts.local_ba_after_n_images);
      if (do_global_ba) {
        const bool opt_intr = opts.global_ba_optimize_intrinsics &&
                              num_registered >= opts.global_ba_optimize_intrinsics_min_images;
        auto t_ba0 = Clock::now();
        const auto frozen_early =
            (opt_intr && opts.intrinsics_progressive_freeze) ? make_frozen_vec()
                                                             : std::vector<bool>{};
        if (run_global_ba(store_out, poses_R_out, poses_C_out, *registered_out,
                          image_to_camera_index, *cameras, cameras, opt_intr,
                          opts.max_global_ba_iterations, &rmse, anchor_image,
                          opt_intr ? &frozen_early : nullptr,
                          opt_intr ? k3p12_fix() : 0u, opts.focal_prior_weight)) {
          LOG(INFO) << "[PERF] global_ba: "
                    << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t_ba0).count()
                    << "ms  n=" << num_registered << "  RMSE=" << rmse << " px";
          // No freeze evaluation in early phase: too few images, intrinsics still converging.
          run_ba_and_reject_outliers(&rmse);
        }
      } else {
        LOG(INFO) << "[PERF] global_ba: skipped (every_n=" << opts.global_ba_every_n_images
                  << ", n=" << num_registered << ")";
      }
    } else {
      // ── Two-tier BA schedule (mid-freq + low-freq recalib) during local-BA phase ──────
      if (opts.periodic_global_ba_every_n_images > 0 &&
          (num_registered - last_periodic_global_ba_registered) >=
              opts.periodic_global_ba_every_n_images) {
        last_periodic_global_ba_registered = num_registered;
        ++periodic_ba_count;
        const bool opt_intr_p =
            opts.global_ba_optimize_intrinsics &&
            num_registered >= opts.global_ba_optimize_intrinsics_min_images;

        // Low-freq recalib: every recalib_global_ba_every_n_periodic mid-freq BAs.
        // Opens ALL intrinsics, then re-evaluates freeze state for every camera.
        const bool do_recalib =
            opt_intr_p && opts.intrinsics_progressive_freeze &&
            opts.recalib_global_ba_every_n_periodic > 0 &&
            (periodic_ba_count % opts.recalib_global_ba_every_n_periodic == 0);
        if (do_recalib) {
          auto t_pba = Clock::now();
          if (run_global_ba(store_out, poses_R_out, poses_C_out, *registered_out,
                            image_to_camera_index, *cameras, cameras, true,
                            opts.max_global_ba_iterations, &rmse, anchor_image,
                            nullptr /* all intrinsics open */)) {
            update_freeze_state(/*recheck_frozen=*/true);
            LOG(INFO) << "[PERF] recalib_global_ba #" << periodic_ba_count << ": "
                      << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t_pba).count()
                      << "ms  n=" << num_registered << "  RMSE=" << rmse << " px";
            run_ba_and_reject_outliers(&rmse);
          }
        } else {
          // Mid-freq global BA: frozen cameras use kFixIntrAll (Schur sparsity).
          // Unfrozen cameras optimise intrinsics; evaluates freeze convergence.
          const bool mid_opt_intr = opt_intr_p;
          const auto frozen_mid =
              (mid_opt_intr && opts.intrinsics_progressive_freeze) ? make_frozen_vec()
                                                                   : std::vector<bool>{};
          auto t_pba = Clock::now();
          if (run_global_ba(store_out, poses_R_out, poses_C_out, *registered_out,
                            image_to_camera_index, *cameras, cameras, mid_opt_intr,
                            opts.max_global_ba_iterations, &rmse, anchor_image,
                            mid_opt_intr ? &frozen_mid : nullptr,
                            mid_opt_intr ? k3p12_fix() : 0u, opts.focal_prior_weight)) {
            if (mid_opt_intr) update_freeze_state(/*recheck_frozen=*/false);
            LOG(INFO) << "[PERF] periodic_global_ba #" << periodic_ba_count << ": "
                      << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t_pba).count()
                      << "ms  n=" << num_registered << "  RMSE=" << rmse << " px";
            run_ba_and_reject_outliers(&rmse);
          }
        }
      }
      auto t_ba0 = Clock::now();
      bool ba_ok = false;
      if (opts.local_ba_strategy == LocalBAStrategy::kColmap) {
        // ── COLMAP-style: 2-hop visibility expansion from newly registered batch ──
        ba_ok = run_local_ba_colmap(store_out, poses_R_out, poses_C_out, *registered_out,
                                    image_to_camera_index, *cameras, batch,
                                    opts.local_ba_colmap_max_variable_images, 25, &rmse);
        LOG(INFO) << "[PERF] local_ba_colmap: "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t_ba0).count()
                  << "ms  n=" << num_registered << "  RMSE=" << rmse << " px";
      } else if (opts.local_ba_strategy == LocalBAStrategy::kBatchNeighbor) {
        // ── Neighbor-anchored: variable=batch+new_pts, constant=per-image top-K neighbors ──
        ba_ok = run_local_ba_batch_neighbor(store_out, poses_R_out, poses_C_out, *registered_out,
                                            image_to_camera_index, *cameras, batch, new_track_ids,
                                            opts.local_ba_neighbor_k, 25, &rmse);
        LOG(INFO) << "[PERF] local_ba_batch_neighbor: "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t_ba0).count()
                  << "ms  n=" << num_registered << "  RMSE=" << rmse << " px";
      } else {
        // ── kWindow: last-N / connectivity-ranked set, all registered in BA ──
        const std::vector<int>* local_indices = nullptr;
        std::vector<int> connectivity_indices;
        if (opts.local_ba_by_connectivity && !batch.empty()) {
          connectivity_indices = choose_local_ba_indices_by_connectivity(
              *store_out, *registered_out, batch, opts.local_ba_window);
          if (!connectivity_indices.empty())
            local_indices = &connectivity_indices;
        }
        ba_ok = run_local_ba(store_out, poses_R_out, poses_C_out, *registered_out,
                             image_to_camera_index, *cameras, opts.local_ba_window, 25, &rmse,
                             local_indices, anchor_image);
        LOG(INFO) << "[PERF] local_ba_window: "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t_ba0).count()
                  << "ms  n=" << num_registered << "  RMSE=" << rmse << " px";
      }
      if (ba_ok)
        run_ba_and_reject_outliers(&rmse);
    }

    // Periodic re-triangulation: recover tracks that were (a) cleared by outlier
    // rejection after losing their 3D support, or (b) skippable previously but now
    // have sufficient registered observers.  Full scan but fast in practice.
    if (opts.retriangulation_every_n_iters > 0 &&
        sfm_iter % opts.retriangulation_every_n_iters == 0) {
      auto t_retri = Clock::now();
      int n_retri = run_retriangulation(store_out, *poses_R_out, *poses_C_out, *registered_out,
                                        *cameras, image_to_camera_index, opts.min_tri_angle_deg);
      LOG(INFO) << "[PERF] periodic_retri: "
                << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t_retri).count()
                << "ms  re_tri=" << n_retri << "  total_tri=" << count_tri_tracks();
    }

    LOG(INFO) << "[PERF] iter #" << sfm_iter << " total: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - iter_start).count()
              << "ms";
  }
  LOG(INFO) << "[PERF] main_loop total: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - pipeline_start).count()
            << "ms  iters=" << sfm_iter;

  run_retriangulation(store_out, *poses_R_out, *poses_C_out, *registered_out, *cameras,
                      image_to_camera_index, opts.min_tri_angle_deg);
  LOG(INFO) << "Final retriangulation done. tri_tracks=" << count_tri_tracks();

  if (opts.do_global_ba) {
    LOG(INFO) << "Running final global BA...";
    double rmse = 0.0;
    if (run_global_ba(store_out, poses_R_out, poses_C_out, *registered_out, image_to_camera_index,
                      *cameras, cameras, opts.global_ba_optimize_intrinsics,
                      opts.max_global_ba_iterations, &rmse, anchor_image)) {
      int rejected = 0;
      if (num_registered >= opts.reject_min_registered_images) {
        double thr = eff_reject_threshold(rmse);
        rejected =
            reject_outliers_multiview(store_out, *poses_R_out, *poses_C_out, *registered_out,
                                      *cameras, image_to_camera_index, thr);
        rejected += reject_outliers_angle_multiview(
            store_out, *poses_R_out, *poses_C_out, *registered_out, opts.min_observation_angle_deg);
        if (rejected > 0)
          LOG(INFO) << "  final outlier rejection: " << rejected << " observations marked (thr="
                    << thr << " px)";
      }
      for (int r = 0; rejected > 0 && r < opts.final_reject_max_rounds - 1; ++r) {
        if (!run_global_ba(store_out, poses_R_out, poses_C_out, *registered_out,
                           image_to_camera_index, *cameras, cameras,
                           opts.global_ba_optimize_intrinsics, opts.max_global_ba_iterations, &rmse,
                           anchor_image))
          break;
        double thr = eff_reject_threshold(rmse);
        rejected =
            reject_outliers_multiview(store_out, *poses_R_out, *poses_C_out, *registered_out,
                                      *cameras, image_to_camera_index, thr);
        rejected += reject_outliers_angle_multiview(
            store_out, *poses_R_out, *poses_C_out, *registered_out, opts.min_observation_angle_deg);
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
