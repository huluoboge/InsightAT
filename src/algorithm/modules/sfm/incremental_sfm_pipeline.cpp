/**
 * @file  incremental_sfm_pipeline.cpp
 * @brief Initial pair loop and (later) full incremental SfM pipeline.
 */

#include "incremental_sfm_pipeline.h"

#include "../../io/idc_reader.h"
#include "../../io/track_store_idc.h"
#include "../camera/camera_utils.h"
#include "../geometry/gpu_geo_ransac.h"
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

// ─────────────────────────────────────────────────────────────────────────────
// IntrinsicsSchedule implementation
// ─────────────────────────────────────────────────────────────────────────────

uint32_t IntrinsicsSchedule::fix_mask_for(int n_registered) const {
  using M = FixIntrinsicsMask;
  // Phase 0: too few images — freeze everything
  if (n_registered < 5)
    return static_cast<uint32_t>(M::kFixIntrAll);
  // Phase 1: only fx/fy free; fix sigma, cx, cy, all distortion
  if (n_registered < 10)
    return static_cast<uint32_t>(M::kFixIntrSigma | M::kFixIntrCx | M::kFixIntrCy | M::kFixIntrK1 |
                                 M::kFixIntrK2 | M::kFixIntrK3 | M::kFixIntrP1 | M::kFixIntrP2);
  // Phase 2: fx/fy + k1/k2 free; fix sigma, cx, cy, k3, p1, p2
  if (n_registered < 30)
    return static_cast<uint32_t>(M::kFixIntrSigma | M::kFixIntrCx | M::kFixIntrCy | M::kFixIntrK3 |
                                 M::kFixIntrP1 | M::kFixIntrP2);
  // Phase 3: all intrinsics free
  return 0u;
}

std::vector<uint32_t>
IntrinsicsSchedule::fix_masks_per_camera(const std::vector<bool>& registered,
                                         const std::vector<int>& image_to_camera_index,
                                         int n_cameras) const {
  if (n_cameras <= 0)
    return {};
  std::vector<uint32_t> masks(static_cast<size_t>(n_cameras));
  std::vector<int> cam_registered_count(static_cast<size_t>(n_cameras), 0);
  for (int i = 0; i < static_cast<int>(registered.size()); ++i) {
    if (registered[i]) {
      int cam_idx = image_to_camera_index[i];
      if (cam_idx >= 0 && cam_idx < n_cameras) {
        cam_registered_count[static_cast<size_t>(cam_idx)]++;
      }
    }
  }
  for (int c = 0; c < n_cameras; ++c) {
    masks[static_cast<size_t>(c)] = fix_mask_for(cam_registered_count[static_cast<size_t>(c)]);
  }
  return masks;

  // // Count registered images per camera.
  // std::vector<int> cam_count(static_cast<size_t>(n_cameras), 0);
  // int total_registered = 0;
  // const size_t n = std::min(registered.size(), image_to_camera_index.size());
  // for (size_t i = 0; i < n; ++i) {
  //   if (registered[i]) {
  //     const int c = image_to_camera_index[i];
  //     if (c >= 0 && c < n_cameras) {
  //       ++cam_count[static_cast<size_t>(c)];
  //       ++total_registered;
  //     }
  //   }
  // }
  // // Synchronise intrinsics phase across all cameras by using the global average
  // // registered count (total_registered / n_cameras) as the effective count for
  // // every active camera.
  // //
  // // Rationale: in a multi-camera rig (e.g. 5-direction oblique aerial rig),
  // // cameras facing LEFT/RIGHT register images faster than BACK/DOWN/FRONT
  // // because of their wider cross-strip overlap.  If each camera advances its
  // // own intrinsics phase independently, early cameras unlock focal length while
  // // late cameras are still frozen, creating a systematic cross-camera fx
  // // mismatch that triggers outlier-rejection cascades and breaks the cross-
  // // direction track graph.
  // //
  // // Using total/n_cameras means all active cameras share the same phase gate.
  // // The phase thresholds (5, 10, 30) are now interpreted as
  // // "N images registered on average across all cameras", which is a stable,
  // // rig-level criterion independent of individual direction connectivity.
  // //
  // // Exception: cameras with zero registered images remain fully frozen
  // // (effective = 0 → kFixIntrAll).  If a direction never registers, it does
  // // not block the remaining cameras from progressing.
  // //
  // // NOTE: The ideal long-term solution is to supply GNSS position priors to
  // // the incremental BA (as in the reference InsightMap pipeline).  With GPS
  // // anchoring the metric scale from the very first BA round, intrinsics can
  // // be released freely without any phase schedule.
  // const int effective_count = (n_cameras > 0) ? (total_registered / n_cameras) : 0;
  // const uint32_t global_mask = fix_mask_for(effective_count);
  // std::vector<uint32_t> masks(static_cast<size_t>(n_cameras));
  // for (int c = 0; c < n_cameras; ++c) {
  //   masks[static_cast<size_t>(c)] =
  //       (cam_count[static_cast<size_t>(c)] == 0)
  //           ? static_cast<uint32_t>(FixIntrinsicsMask::kFixIntrAll)
  //           : global_mask;
  // }
  // return masks;
}

// ─────────────────────────────────────────────────────────────────────────────
// Preset constructors
// ─────────────────────────────────────────────────────────────────────────────

IncrementalSfMOptions make_aerial_preset() {
  IncrementalSfMOptions o;
  // Initial pair
  o.init.min_tracks_for_intital_pair = 50;
  o.init.max_first_images = 100;
  o.init.max_second_images = 50;
  o.init.ba_rmse_max = 8.0;
  o.init.outlier_threshold_px = 4.0;
  o.init.min_angle_deg = 2.0;
  // Resection
  o.resection.min_inliers = 15; // 15=6 * 2.5; // 2.5× the inliers needed to accept an image, to be
                                // more robust to early outliers before local BA can clean up.
  o.resection.min_3d2d_count = 30;
  o.resection.batch_ratio = 0.75;
  o.resection.batch_max = 20;
  o.resection.retry_after_cleanup = true;
  // Intrinsics schedule (4-phase progressive unlock)
  o.intrinsics.phase1_min_images = 3;
  o.intrinsics.phase2_min_images = 10;
  o.intrinsics.phase3_min_images = 100;
  o.intrinsics.progressive_freeze = true;
  o.intrinsics.freeze_min_images = 30;
  o.intrinsics.freeze_delta_focal = 1e-3;
  o.intrinsics.freeze_delta_pp = 0.5;
  o.intrinsics.freeze_delta_dist = 1e-4;
  o.intrinsics.freeze_stable_rounds = 2;
  o.intrinsics.recalib_every_n_periodic = 4;
  o.intrinsics.focal_prior_weight = 0.0;
  // Local BA — COLMAP-style 2-hop
  o.local_ba.switch_after_n_images = 20;
  o.local_ba.window = 20;
  o.local_ba.by_connectivity = true;
  o.local_ba.strategy = LocalBAStrategy::kColmap;
  o.local_ba.colmap_max_variable_images = 6;
  o.local_ba.neighbor_k = 5;
  o.local_ba.skip = false;
  o.local_ba.max_iterations = 25;
  // Global BA
  o.global_ba.enabled = true;
  o.global_ba.optimize_intrinsics = true;
  o.global_ba.optimize_intrinsics_min_images = 10;
  o.global_ba.max_iterations = 50;
  o.global_ba.every_n_images = 1;
  o.global_ba.periodic_every_n_images = 20;
  // Outlier rejection (Huber-MAD unified)
  o.outlier.threshold_px = 4.0;
  o.outlier.huber_delta_lo_px = 0.5;
  o.outlier.huber_delta_hi_px = 3.0;
  o.outlier.mad_k = 2.5;
  o.outlier.min_angle_deg = 2.0;
  o.outlier.max_depth_factor = 200.0;
  o.outlier.max_rounds = 5;
  o.outlier.min_for_retry = 30;
  o.outlier.min_registered_images = 10;
  o.outlier.final_max_rounds = 10;
  // Triangulation
  o.triangulation.min_angle_deg = 2.0;
  o.triangulation.retriangulation_every_n_iters = 3;
  return o;
}

IncrementalSfMOptions make_object_scan_preset() {
  IncrementalSfMOptions o = make_aerial_preset();
  // Skip local BA (circumferential capture)
  o.local_ba.skip = true;
  // Late-stage resection relaxation (dense object, high-count best candidates)
  o.resection.late_registered_threshold = 30;
  o.resection.late_absolute_min = 100;
  o.resection.late_batch_max = 5;
  // Unlock all intrinsics earlier (more oblique views → better observability)
  o.intrinsics.phase3_min_images = 30;
  // Tight BA solver for large object maps
  o.global_ba.max_iterations = 5000;
  o.global_ba.solver_overrides.gradient_tolerance = 1e-10;
  o.global_ba.solver_overrides.function_tolerance = 1e-6;
  o.global_ba.solver_overrides.parameter_tolerance = 1e-8;
  o.global_ba.solver_overrides.dense_schur_max_variable_cams = 50;
  return o;
}

IncrementalSfMOptions make_general_preset() {
  // Default-constructed opts are already suitable for general capture
  return IncrementalSfMOptions{};
}

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

/// Post-resection cleanup for newly registered images only.
///
/// Purpose:
///   A successful PnP/RANSAC only guarantees a consistent inlier subset for the
///   new camera pose. Other existing image↔track associations on that image may
///   still be wrong. Before triangulating new tracks or running BA, drop those
///   bad observations so they do not pollute the map.
///
/// Policy:
///   - Only inspect the supplied `image_indices`.
///   - Only inspect observations whose tracks already have triangulated XYZ.
///   - Delete the observation if the point projects behind the camera or the
///     pixel reprojection error exceeds `threshold_px`.
///   - If a track loses enough support that <2 valid observations remain,
///     clear its XYZ so later retriangulation can recover it.
static int reject_outliers_post_resection(TrackStore* store, const std::vector<int>& image_indices,
                                          const std::vector<Eigen::Matrix3d>& poses_R,
                                          const std::vector<Eigen::Vector3d>& poses_C,
                                          const std::vector<bool>& registered,
                                          const std::vector<camera::Intrinsics>& cameras,
                                          const std::vector<int>& image_to_camera_index,
                                          double threshold_px) {
  if (!store || image_indices.empty())
    return 0;
  const int n_images = store->num_images();
  if (static_cast<int>(poses_R.size()) != n_images ||
      static_cast<int>(poses_C.size()) != n_images ||
      static_cast<int>(registered.size()) != n_images ||
      static_cast<int>(image_to_camera_index.size()) != n_images)
    return 0;

  const double thresh_sq = threshold_px * threshold_px;
  int marked = 0;
  std::unordered_set<int> dirty_tracks;
  std::vector<int> obs_ids;

  for (int im : image_indices) {
    if (im < 0 || im >= n_images || !registered[static_cast<size_t>(im)])
      continue;
    obs_ids.clear();
    store->get_image_observation_indices(im, &obs_ids);
    const camera::Intrinsics& K =
        cameras[static_cast<size_t>(image_to_camera_index[static_cast<size_t>(im)])];
    const Eigen::Matrix3d& R = poses_R[static_cast<size_t>(im)];
    const Eigen::Vector3d& C = poses_C[static_cast<size_t>(im)];
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

static int count_valid_triangulated_observations_on_image(const TrackStore& store,
                                                          int image_index) {
  std::vector<int> obs_ids;
  store.get_image_observation_indices(image_index, &obs_ids);
  int count = 0;
  for (int obs_id : obs_ids) {
    if (!store.is_obs_valid(obs_id))
      continue;
    const int tid = store.obs_track_id(obs_id);
    if (store.is_track_valid(tid) && store.track_has_triangulated_xyz(tid))
      ++count;
  }
  return count;
}

static int rollback_weak_post_resection_images(
    const TrackStore& store, const std::vector<int>& image_indices, int min_support,
    std::vector<Eigen::Matrix3d>* poses_R, std::vector<Eigen::Vector3d>* poses_C,
    std::vector<bool>* registered, std::vector<int>* kept_images_out) {
  if (kept_images_out)
    kept_images_out->clear();
  if (!poses_R || !poses_C || !registered)
    return 0;

  int removed = 0;
  for (int im : image_indices) {
    if (im < 0 || static_cast<size_t>(im) >= registered->size() ||
        !(*registered)[static_cast<size_t>(im)])
      continue;
    const int support = count_valid_triangulated_observations_on_image(store, im);
    if (support < min_support) {
      (*registered)[static_cast<size_t>(im)] = false;
      if (static_cast<size_t>(im) < poses_R->size())
        (*poses_R)[static_cast<size_t>(im)] = Eigen::Matrix3d::Identity();
      if (static_cast<size_t>(im) < poses_C->size())
        (*poses_C)[static_cast<size_t>(im)] = Eigen::Vector3d::Zero();
      ++removed;
      LOG(INFO) << "  post_resection_cleanup: rollback image " << im
                << " (remaining triangulated obs=" << support << " < " << min_support << ")";
      continue;
    }
    if (kept_images_out)
      kept_images_out->push_back(im);
  }
  return removed;
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
  const double max_angle_rad = 60 * (3.141592653589793 / 180.0);
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
    struct RegObs {
      int obs_id;
      Eigen::Vector3d ray;
    };
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
        if (j == i)
          continue;
        double cos_a = std::max(-1.0, std::min(1.0, reg[i].ray.dot(reg[j].ray)));
        double angle = std::acos(cos_a);
        if (angle > max_angle)
          max_angle = angle;
      }
      if (max_angle < min_angle_rad || max_angle > max_angle_rad) {
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

/// Reject observations whose corresponding 3D point lies behind the camera (non-positive depth)
/// or whose depth exceeds max_depth_factor × median scene depth.  The median is computed once
/// over all valid observations so that the extreme points drive the threshold, not contaminate it.
/// max_depth_factor <= 0 disables the upper-bound check; only cheirality (depth ≤ 0) is tested.
int reject_outliers_depth(TrackStore* store, const std::vector<Eigen::Matrix3d>& poses_R,
                          const std::vector<Eigen::Vector3d>& poses_C,
                          const std::vector<bool>& registered, double max_depth_factor) {
  if (!store)
    return 0;
  const int n_images = store->num_images();
  if (static_cast<int>(poses_R.size()) != n_images ||
      static_cast<int>(poses_C.size()) != n_images ||
      static_cast<int>(registered.size()) != n_images)
    return 0;

  // Pass 1: collect all positive depths to compute median.
  std::vector<double> depths;
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
    const double depth =
        (poses_R[static_cast<size_t>(im)] * (X - poses_C[static_cast<size_t>(im)]))(2);
    if (depth > 0.0)
      depths.push_back(depth);
  }

  // Determine max allowed depth from median of positive depths.
  double max_depth = std::numeric_limits<double>::max();
  // if (max_depth_factor > 0.0 && !depths.empty()) {
  //   std::nth_element(depths.begin(), depths.begin() + static_cast<ptrdiff_t>(depths.size() / 2),
  //                    depths.end());
  //   const double median_depth = depths[depths.size() / 2];
  //   max_depth = median_depth * max_depth_factor;
  // }

  // Pass 2: mark observations that violate depth bounds.
  int marked = 0;
  std::unordered_set<int> dirty_tracks;
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
    const double depth =
        (poses_R[static_cast<size_t>(im)] * (X - poses_C[static_cast<size_t>(im)]))(2);
    if (depth <= 0.0 || depth > max_depth) {
      store->mark_observation_deleted(obs_id);
      dirty_tracks.insert(tid);
      ++marked;
    }
  }

  // Clear XYZ of tracks that lost support (< 2 valid observations).
  std::vector<int> obs_ids_chk;
  for (int tid : dirty_tracks) {
    if (!store->is_track_valid(tid) || !store->track_has_triangulated_xyz(tid))
      continue;
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

// ─────────────────────────────────────────────────────────────────────────────
// MAD-based coarse rejection helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Collect per-observation reprojection errors (px) for all valid triangulated observations
/// across registered images.  Used to compute a MAD-adaptive coarse rejection threshold.
static std::vector<double> collect_reproj_errors(const TrackStore& store,
                                                 const std::vector<Eigen::Matrix3d>& poses_R,
                                                 const std::vector<Eigen::Vector3d>& poses_C,
                                                 const std::vector<bool>& registered,
                                                 const std::vector<camera::Intrinsics>& cameras,
                                                 const std::vector<int>& image_to_camera_index,
                                                 const std::vector<int>* only_images = nullptr) {
  std::vector<double> errors;
  const int n_images = store.num_images();
  if (static_cast<int>(poses_R.size()) != n_images ||
      static_cast<int>(poses_C.size()) != n_images ||
      static_cast<int>(registered.size()) != n_images ||
      static_cast<int>(image_to_camera_index.size()) != n_images)
    return errors;
  // Build fast-lookup set when filtering to specific images.
  std::unordered_set<int> only_set;
  if (only_images && !only_images->empty())
    only_set.insert(only_images->begin(), only_images->end());
  for (size_t i = 0; i < store.num_observations(); ++i) {
    const int obs_id = static_cast<int>(i);
    if (!store.is_obs_valid(obs_id))
      continue;
    Observation obs;
    store.get_obs(obs_id, &obs);
    const int im = static_cast<int>(obs.image_index);
    if (im < 0 || im >= n_images || !registered[static_cast<size_t>(im)])
      continue;
    const int tid = store.obs_track_id(obs_id);
    if (!store.is_track_valid(tid) || !store.track_has_triangulated_xyz(tid))
      continue;
    float tx, ty, tz;
    store.get_track_xyz(tid, &tx, &ty, &tz);
    const Eigen::Vector3d X(static_cast<double>(tx), static_cast<double>(ty),
                            static_cast<double>(tz));
    const Eigen::Matrix3d& R = poses_R[static_cast<size_t>(im)];
    const Eigen::Vector3d& C = poses_C[static_cast<size_t>(im)];
    Eigen::Vector3d p = R * (X - C);
    if (p(2) <= 1e-12)
      continue; // behind camera — cheirality filter handles this
    const camera::Intrinsics& K =
        cameras[static_cast<size_t>(image_to_camera_index[static_cast<size_t>(im)])];
    const double xn = p(0) / p(2), yn = p(1) / p(2);
    double xd, yd;
    camera::apply_distortion(xn, yn, K, &xd, &yd);
    const double u_pred = K.fx * xd + K.cx;
    const double v_pred = K.fy * yd + K.cy;
    const double du = static_cast<double>(obs.u) - u_pred;
    const double dv = static_cast<double>(obs.v) - v_pred;
    errors.push_back(std::sqrt(du * du + dv * dv));
  }
  return errors;
}

/// Compute MAD-based adaptive threshold from a pre-collected error vector.
/// threshold = median(e) + k_sigma * 1.4826 * MAD(e), clamped to >= floor_px.
static double compute_mad_threshold_from_errors(const std::vector<double>& errors, double k_sigma,
                                                double floor_px) {
  if (errors.empty())
    return floor_px;
  std::vector<double> sorted = errors;
  std::sort(sorted.begin(), sorted.end());
  const double median_e = sorted[sorted.size() / 2];
  std::vector<double> abs_dev;
  abs_dev.reserve(sorted.size());
  for (double e : sorted)
    abs_dev.push_back(std::abs(e - median_e));
  std::sort(abs_dev.begin(), abs_dev.end());
  const double mad = abs_dev[abs_dev.size() / 2];
  const double sigma_hat = 1.4826 * mad;
  return std::max(floor_px, median_e + k_sigma * sigma_hat);
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

/// Trial result for an initial pair candidate (no store mutation).
struct InitPairTrialResult {
  bool success = false;
  Eigen::Matrix3d R1 = Eigen::Matrix3d::Identity();
  Eigen::Vector3d C1 = Eigen::Vector3d::Zero();
  double rmse_px = 1e9;
  double reproject_px = 1e9;
  int n_inlier_tracks = 0;
  int n_triangulated = 0;
  std::vector<std::pair<int, Eigen::Vector3d>> track_xyz; ///< inlier track_id -> refined XYZ
  std::unordered_set<int> inlier_track_ids;               ///< set version for O(1) lookup
};

/// Try an initial pair WITHOUT mutating the store. Triangulates into a local buffer,
/// runs BA locally, checks quality (reprojection + angle + RMSE). Returns trial result.
InitPairTrialResult try_initial_pair_candidate(const TrackStore& store, int im0, int im1,
                                               const std::string& dir,
                                               const std::vector<camera::Intrinsics>& cameras,
                                               const std::vector<int>& image_to_camera_index,
                                               int min_tracks_for_intital_pair, double ba_rmse_max,
                                               double outlier_thresh_px, double min_angle_deg) {

  const double kPi = 3.141592653589793;
  const double min_angle_rad_q = min_angle_deg * (kPi / 180.0);
  const double max_angle_rad_q = 60.0 * (kPi / 180.0);

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

    // ── Pre-BA geometric filters (independent of focal length) ──────────────
    // Step 1: Cheirality — point must be in front of BOTH cameras.
    if (X(2) <= 1e-9)
      continue;
    Eigen::Vector3d X_in_cam1_pre = R * (X - C1);
    if (X_in_cam1_pre(2) <= 1e-9)
      continue;

    // Step 2 & 3: Triangulation angle must be in [min_angle_deg, 60°].
    // Large angle (>60°) = degenerate far-field or bad match; small angle = nearly parallel rays.
    {
      const double max_angle_rad_q = 60.0 * (kPi / 180.0);
      const double min_angle_rad_q = min_angle_deg * (kPi / 180.0);
      Eigen::Vector3d r0_dir = X.normalized();
      Eigen::Vector3d r1_dir = (X - C1).normalized();
      double cos_a = std::max(-1.0, std::min(1.0, r0_dir.dot(r1_dir)));
      double ang = std::acos(cos_a);
      if (ang > max_angle_rad_q || ang < min_angle_rad_q)
        continue;
    }
    // ────────────────────────────────────────────────────────────────────────

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
  const int kIterForInitialBA = 500;
  if (!global_bundle_analytic(ba_in, &ba_out, kIterForInitialBA)) {
    LOG(INFO) << "    pair (" << im0 << "," << im1 << "): BA failed";
    return result;
  }
  result.rmse_px = ba_out.rmse_px;
  R = ba_out.poses_R[1];
  C1 = ba_out.poses_C[1];

  // 4. Adaptive inlier selection via MAD-based reprojection threshold.
  //
  // Using a fixed pixel threshold is unreliable at this stage because the initial
  // focal length estimate may be off by 5-15%, introducing a systematic reproj
  // bias proportional to image_diag * (f_err/f).  Instead we use the sample
  // distribution itself to set the cutoff:
  //
  //   threshold = median(errors) + k * 1.4826 * MAD(errors)
  //
  // MAD has 50% breakdown point — up to half the points can be outliers without
  // distorting the estimate.  We try k = 3.0, 3.5, 4.0 progressively until we
  // have enough inliers; if still too few, fall back to the p99 error quantile.
  //
  // Hard floor / ceiling: threshold is clamped to [2 px, 50 px] to avoid
  // degenerate cases (near-zero MAD when most points are perfect, or huge MAD
  // when reconstruction is poor).

  // Per-point geometry check + reprojection error (max over both views).
  struct PointQuality {
    size_t idx;           // index into track_ids / ba_out.points3d
    double max_reproj_px; // max reprojection error over both views
  };
  std::vector<PointQuality> pt_quality;
  pt_quality.reserve(ba_out.points3d.size());

  for (size_t i = 0; i < ba_out.points3d.size() && i < track_ids.size(); ++i) {
    const Eigen::Vector3d& X = ba_out.points3d[i];

    // Step 1: Cheirality — positive depth in both cameras after BA refinement.
    if (X(2) <= 1e-9)
      continue;
    Eigen::Vector3d p1_q = R * (X - C1);
    if (p1_q(2) <= 1e-9)
      continue;

    // Step 2: Max angle (<= 60°).  Step 3: Min angle (>= min_angle_deg).
    // Double-check after BA: point positions may shift slightly.
    {
      Eigen::Vector3d r0_q = X.normalized();
      Eigen::Vector3d r1_q = (X - C1).normalized();
      double cos_a_q = std::max(-1.0, std::min(1.0, r0_q.dot(r1_q)));
      double ang_q = std::acos(cos_a_q);
      if (ang_q > max_angle_rad_q || ang_q < min_angle_rad_q)
        continue;
    }

    // Compute reprojection errors for both views.
    double xn0_q = X(0) / X(2), yn0_q = X(1) / X(2);
    double xd0_q, yd0_q;
    camera::apply_distortion(xn0_q, yn0_q, K0, &xd0_q, &yd0_q);
    const double u0_pred = K0.fx * xd0_q + K0.cx;
    const double v0_pred = K0.fy * yd0_q + K0.cy;

    double xn1_q = p1_q(0) / p1_q(2), yn1_q = p1_q(1) / p1_q(2);
    double xd1_q, yd1_q;
    camera::apply_distortion(xn1_q, yn1_q, K1, &xd1_q, &yd1_q);
    const double u1_pred = K1.fx * xd1_q + K1.cx;
    const double v1_pred = K1.fy * yd1_q + K1.cy;

    // TODO，这里图像内参应该加上图像的宽高，这样可以剔除外面的点
    obs_buf.clear();
    store.get_track_observations(track_ids[i], &obs_buf);
    double max_e = 0.0;
    for (const auto& o : obs_buf) {
      double u_pred, v_pred;
      if (o.image_index == uim0) {
        u_pred = u0_pred;
        v_pred = v0_pred;
      } else if (o.image_index == uim1) {
        u_pred = u1_pred;
        v_pred = v1_pred;
      } else
        continue;
      double du = static_cast<double>(o.u) - u_pred;
      double dv = static_cast<double>(o.v) - v_pred;
      max_e = std::max(max_e, std::sqrt(du * du + dv * dv));
    }
    pt_quality.push_back({i, max_e});
  }

  // Step 4: MAD-based adaptive threshold.
  int n_inlier = 0;
  std::vector<std::pair<int, Eigen::Vector3d>> accepted_xyz;
  double mad_thr = outlier_thresh_px; // fallback to caller-supplied value

  if (!pt_quality.empty()) {
    // Collect errors and sort for median / MAD computation.
    std::vector<double> errors;
    errors.reserve(pt_quality.size());
    for (const auto& pq : pt_quality)
      errors.push_back(pq.max_reproj_px);
    std::sort(errors.begin(), errors.end());

    const double median_e = errors[errors.size() / 2];

    std::vector<double> abs_dev;
    abs_dev.reserve(errors.size());
    for (double e : errors)
      abs_dev.push_back(std::abs(e - median_e));
    std::sort(abs_dev.begin(), abs_dev.end());
    const double mad = abs_dev[abs_dev.size() / 2];
    const double sigma_hat = 1.4826 * mad;

    // Progressive k: try 3.0 → 3.5 → 4.0 until we have enough inliers.
    const double k_levels[] = {3.0, 3.5, 4.0};
    double chosen_thr = 0.0;
    for (double k : k_levels) {
      double thr_k = std::max(2.0, std::min(50.0, median_e + k * sigma_hat));
      int cnt = 0;
      for (const auto& pq : pt_quality)
        if (pq.max_reproj_px <= thr_k)
          ++cnt;
      chosen_thr = thr_k;
      if (cnt >= min_tracks_for_intital_pair)
        break;
    }
    // If still short, extend to p99 as a last resort.
    {
      int cnt = 0;
      for (const auto& pq : pt_quality)
        if (pq.max_reproj_px <= chosen_thr)
          ++cnt;
      if (cnt < min_tracks_for_intital_pair && errors.size() >= 10) {
        const size_t p99_idx = (errors.size() * 99) / 100;
        chosen_thr = std::max(chosen_thr, errors[std::min(p99_idx, errors.size() - 1)]);
      }
    }
    mad_thr = chosen_thr;

    LOG(INFO) << "    pair (" << im0 << "," << im1 << ") MAD filter:"
              << " median=" << median_e << " MAD=" << mad << " sigma_hat=" << sigma_hat
              << " thr=" << mad_thr << " geom_ok=" << pt_quality.size();

    for (const auto& pq : pt_quality) {
      if (pq.max_reproj_px <= mad_thr) {
        ++n_inlier;
        accepted_xyz.push_back({track_ids[pq.idx], ba_out.points3d[pq.idx]});
      }
    }
    result.reproject_px = mad_thr;
  } else {
    result.success = false;
    return result;
  }

  result.n_inlier_tracks = n_inlier;

  LOG(INFO) << "    pair (" << im0 << "," << im1 << "): tri=" << track_ids.size()
            << " geom_ok=" << pt_quality.size() << " inlier=" << n_inlier
            << " RMSE=" << ba_out.rmse_px << " px"
            << " mad_thr=" << mad_thr;

  // Step 5: Minimum inlier count and RMSE gate.
  if (n_inlier < min_tracks_for_intital_pair) {
    LOG(INFO) << "    pair (" << im0 << "," << im1 << "): REJECTED (need "
              << min_tracks_for_intital_pair << " inlier tracks, got " << n_inlier << ")";
    return result;
  }
  if (ba_out.rmse_px > ba_rmse_max) {
    LOG(INFO) << "    pair (" << im0 << "," << im1 << "): REJECTED (RMSE " << ba_out.rmse_px
              << " > " << ba_rmse_max << ")";
    return result;
  }

  // All filters passed — package result (no store mutation).
  result.success = true;
  result.R1 = R;
  result.C1 = C1;
  result.track_xyz = std::move(accepted_xyz);
  result.inlier_track_ids.reserve(result.track_xyz.size());
  for (const auto& p : result.track_xyz)
    result.inlier_track_ids.insert(p.first);
  return result;
}

} // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Initial pair selection: COLMAP-style two-level search from TrackStore
// ─────────────────────────────────────────────────────────────────────────────
bool run_initial_pair_loop(const ViewGraph& view_graph, const std::string& geo_dir,
                           TrackStore* store, const std::vector<camera::Intrinsics>& cameras,
                           const std::vector<int>& image_to_camera_index,
                           int min_tracks_for_intital_pair, uint32_t* initial_im0_out,
                           uint32_t* initial_im1_out, std::vector<Eigen::Matrix3d>* poses_R_out,
                           std::vector<Eigen::Vector3d>* poses_C_out,
                           std::vector<bool>* registered_out) {
  if (!store || !poses_R_out || !poses_C_out || !registered_out || cameras.empty())
    return false;
  const int n_images = store->num_images();
  std::string dir = geo_dir;
  if (!dir.empty() && dir.back() != '/')
    dir += '/';

  LOG(INFO) << "================================================================";
  LOG(INFO)
      << "Initial pair selection: first image by track correspondences, second by ViewGraph score";
  LOG(INFO) << "  n_images=" << n_images << ", n_tracks=" << store->num_tracks()
            << ", n_obs=" << store->num_observations();
  LOG(INFO) << "  min_tracks_for_intital_pair=" << min_tracks_for_intital_pair;
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

    // Phase 2: rank second-image candidates via ViewGraph quality score.
    // Hard filters (F_ok && !is_degenerate) are applied inside get_second_image_candidates_sorted.
    // Soft score rewards E_ok, twoview_ok, stable, num_valid_points (w_pt is highest).
    const std::set<uint32_t> registered_so_far; // empty during initial-pair search
    auto second_candidates = view_graph.get_second_image_candidates_sorted(
        static_cast<uint32_t>(im1), registered_so_far);
    if (second_candidates.empty())
      continue;

    LOG(INFO) << "  --- Trying first image " << im1 << " (corr=" << first_images[fi].second << "), "
              << second_candidates.size() << " second candidates (ViewGraph-scored) ---";
    {
      const size_t show_n = std::min(second_candidates.size(), size_t(5));
      for (size_t si = 0; si < show_n; ++si) {
        const auto& sc = second_candidates[si];
        LOG(INFO) << "    second #" << si << ": image " << sc.image_index << " score=" << sc.score
                  << " F_inliers=" << sc.F_inliers << " E_ok=" << sc.E_ok
                  << " twoview_ok=" << sc.twoview_ok << " stable=" << sc.stable
                  << " n_pts=" << sc.num_valid_points;
      }
      if (second_candidates.size() > show_n)
        LOG(INFO) << "    ... (" << second_candidates.size() - show_n << " more)";
    }

    const size_t max_second = std::min(second_candidates.size(), size_t(50));
    for (size_t si = 0; si < max_second; ++si) {
      const int im2 = static_cast<int>(second_candidates[si].image_index);
      auto pair_key = std::make_pair(std::min(im1, im2), std::max(im1, im2));
      if (tried_pairs.count(pair_key))
        continue;
      tried_pairs.insert(pair_key);

      // Try this pair (NO store mutation)
      const float ba_rmse_max = 10.0;
      double outlier_thresh_px = 4.0;
      double min_angle_deg = 2.0;
      auto trial = try_initial_pair_candidate(*store, im1, im2, dir, cameras, image_to_camera_index,
                                              min_tracks_for_intital_pair, ba_rmse_max,
                                              outlier_thresh_px, min_angle_deg);
      if (!trial.success)
        continue;

      // ── Pair accepted! Now commit to store ──
      LOG(INFO) << "  >> ACCEPTED pair (" << im1 << ", " << im2 << "): " << trial.n_inlier_tracks
                << " inlier tracks, RMSE=" << trial.rmse_px
                << " px, reproj_thr=" << trial.reproject_px << " px";

      // Write refined XYZ for inlier tracks.
      for (const auto& p : trial.track_xyz) {
        store->set_track_xyz(p.first, static_cast<float>(p.second(0)),
                             static_cast<float>(p.second(1)), static_cast<float>(p.second(2)));
      }

      // Reject observations of NON-inlier tracks in im1/im2 using the per-image reverse index.
      // get_image_observation_indices() returns only obs for that image (O(obs_in_image)),
      // so total cost here is O(obs_im1 + obs_im2), not O(num_observations_total).
      // Observations from OTHER images on the same track are intentionally left intact
      // so future cameras can still observe those tracks.
      {
        int rej_obs = 0;
        std::vector<int> img_obs_ids;
        for (int target_im : {im1, im2}) {
          img_obs_ids.clear();
          store->get_image_observation_indices(target_im, &img_obs_ids);
          for (int obs_id : img_obs_ids) {
            const int tid = store->obs_track_id(obs_id);
            if (trial.inlier_track_ids.count(tid) == 0) {
              store->mark_observation_deleted(obs_id);
              ++rej_obs;
            }
          }
        }
        int n_valid = count_two_view_valid_tracks(*store, im1, im2);
        LOG(INFO) << "  After commit: valid_tracks=" << n_valid << ", rejected_obs=" << rej_obs
                  << " (MAD-consistent, image reverse-index)";
      }

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

// ─────────────────────────────────────────────────────────────────────────────
// Resection candidate helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Compute the spatial coverage score [0,1] for the triangulated 3D-2D observations
/// of a given image.  The score is the fraction of a 3×3 bounding-box grid that
/// contains ≥1 observation.  Low values (< 0.33) indicate collinear/clustered
/// geometry that is likely to produce a degenerate PnP solution.
static float compute_3d2d_coverage(const TrackStore& store, int image_index) {
  std::vector<int> track_ids;
  std::vector<Observation> obs;
  store.get_image_track_observations(image_index, &track_ids, &obs);

  float min_u = 1e9f, max_u = -1e9f, min_v = 1e9f, max_v = -1e9f;
  int n_tri = 0;
  for (size_t i = 0; i < track_ids.size(); ++i) {
    if (!store.track_has_triangulated_xyz(track_ids[i]))
      continue;
    min_u = std::min(min_u, obs[i].u);
    max_u = std::max(max_u, obs[i].u);
    min_v = std::min(min_v, obs[i].v);
    max_v = std::max(max_v, obs[i].v);
    ++n_tri;
  }
  if (n_tri < 4)
    return 0.0f;

  const float ru = max_u - min_u + 1.0f;
  const float rv = max_v - min_v + 1.0f;
  std::array<bool, 9> cells{};
  for (size_t i = 0; i < track_ids.size(); ++i) {
    if (!store.track_has_triangulated_xyz(track_ids[i]))
      continue;
    const int cx = std::min(2, static_cast<int>(3.0f * (obs[i].u - min_u) / ru));
    const int cy = std::min(2, static_cast<int>(3.0f * (obs[i].v - min_v) / rv));
    cells[static_cast<size_t>(cy * 3 + cx)] = true;
  }
  return static_cast<float>(std::count(cells.cbegin(), cells.cend(), true)) / 9.0f;
}

std::vector<int> choose_next_resection_batch(const TrackStore& store,
                                             const std::vector<bool>& registered,
                                             int min_3d2d_count, double batch_ratio, int batch_max,
                                             int late_registered_threshold, int late_absolute_min,
                                             int late_batch_max) {
  using Clock = std::chrono::steady_clock;
  const int n_images = store.num_images();
  if (n_images == 0 || static_cast<int>(registered.size()) != n_images || batch_max <= 0)
    return {};

  // Count registered for late-stage logic.
  const int num_registered =
      static_cast<int>(std::count(registered.begin(), registered.end(), true));

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
    LOG(INFO) << "[PERF] choose_next_resection_batch: no candidates above min_3d2d="
              << min_3d2d_count << "  total_obs_checked=" << total_track_obs_checked;
    return {};
  }

  // Sort descending by score.
  std::sort(unreg_count.begin(), unreg_count.end(),
            [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
              return a.first > b.first; // higher count first
            });

  // OpenMVG 75%-ratio threshold: include all within ratio of best.
  const int best_count = unreg_count[0].first;
  int ratio_floor = static_cast<int>(std::ceil(best_count * batch_ratio));
  // Late-stage relaxation: once enough images are registered, replace the strict
  // ratio floor with an absolute minimum, so images with e.g. 150 obs are not
  // excluded just because the best has 300 obs (ratio_floor would be 225).
  const bool late_mode = (late_registered_threshold > 0 && late_absolute_min > 0 &&
                          num_registered > late_registered_threshold);
  // In late-stage mode: relax ratio floor AND cap batch size separately.
  const int effective_batch_cap = (late_mode && late_batch_max > 0) ? late_batch_max : batch_max;
  if (late_mode) {
    ratio_floor = std::min(ratio_floor, late_absolute_min);
    LOG(INFO) << "  [late-mode] num_registered=" << num_registered
              << " > threshold=" << late_registered_threshold << ": ratio_floor relaxed to min("
              << static_cast<int>(std::ceil(best_count * batch_ratio)) << ", " << late_absolute_min
              << ")=" << ratio_floor << ", batch_cap=" << effective_batch_cap;
  }
  std::vector<int> out;
  for (const auto& p : unreg_count) {
    if (p.first < ratio_floor || static_cast<int>(out.size()) >= effective_batch_cap)
      break;
    out.push_back(p.second);
  }

  auto ms_count = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
  LOG(INFO) << "[PERF] choose_next_resection_batch: " << unreg_count.size()
            << " candidates (above min_3d2d=" << min_3d2d_count << "), best=" << best_count
            << ", ratio_floor=" << ratio_floor << ", selected=" << out.size()
            << ", total_obs_checked=" << total_track_obs_checked << ", count_loop=" << ms_count
            << "ms";
  for (size_t i = 0; i < std::min(unreg_count.size(), size_t(5)); ++i) {
    LOG(INFO) << "  candidate #" << i << ": image " << unreg_count[i].second << " ("
              << unreg_count[i].first << " 3D-2D correspondences)"
              << (i < out.size() ? "  [selected]" : "  [below ratio]");
  }
  return out;
}

std::vector<ResectionCandidate>
choose_resection_candidates(const TrackStore& store, const std::vector<bool>& registered,
                            int min_3d2d_count, double batch_ratio, int batch_max,
                            int max_candidates, int late_registered_threshold,
                            int late_absolute_min, int late_batch_max) {
  const int n_images = store.num_images();
  if (n_images == 0 || static_cast<int>(registered.size()) != n_images || batch_max <= 0 ||
      max_candidates <= 0)
    return {};

  const int num_registered =
      static_cast<int>(std::count(registered.begin(), registered.end(), true));

  // Score every unregistered image by triangulated 3D-2D count.
  std::vector<std::pair<int, int>> unreg_count; // (count, image_idx)
  for (int im = 0; im < n_images; ++im) {
    if (registered[static_cast<size_t>(im)])
      continue;
    std::vector<int> track_ids;
    store.get_image_track_observations(im, &track_ids, nullptr);
    int count = 0;
    for (int tid : track_ids)
      if (store.track_has_triangulated_xyz(tid))
        ++count;
    if (count >= min_3d2d_count)
      unreg_count.push_back({count, im});
  }
  if (unreg_count.empty())
    return {};

  std::sort(unreg_count.begin(), unreg_count.end(),
            [](const auto& a, const auto& b) { return a.first > b.first; });

  // Compute ratio_floor and batch_cap (same late-mode logic as choose_next_resection_batch).
  const int best_count = unreg_count[0].first;
  int ratio_floor = static_cast<int>(std::ceil(best_count * batch_ratio));
  const bool late_mode = (late_registered_threshold > 0 && late_absolute_min > 0 &&
                          num_registered > late_registered_threshold);
  const int effective_batch_cap = (late_mode && late_batch_max > 0) ? late_batch_max : batch_max;
  if (late_mode)
    ratio_floor = std::min(ratio_floor, late_absolute_min);

  // Build candidate list up to max_candidates; tag is_primary for the normal-batch subset.
  std::vector<ResectionCandidate> out;
  out.reserve(std::min(static_cast<int>(unreg_count.size()), max_candidates));
  int primary_count = 0;
  for (const auto& p : unreg_count) {
    if (static_cast<int>(out.size()) >= max_candidates)
      break;
    const bool primary = (p.first >= ratio_floor && primary_count < effective_batch_cap);
    ResectionCandidate c;
    c.image_index = p.second;
    c.num_3d2d = p.first;
    c.coverage = compute_3d2d_coverage(store, p.second);
    c.is_primary = primary;
    if (primary)
      ++primary_count;
    out.push_back(c);
  }
  LOG(INFO) << "choose_resection_candidates: " << unreg_count.size()
            << " total, selected=" << out.size() << " (primary=" << primary_count
            << ", fallback=" << (static_cast<int>(out.size()) - primary_count) << ")"
            << "  best=" << best_count << " ratio_floor=" << ratio_floor;
  return out;
}

int run_batch_resection(const TrackStore& store, const std::vector<int>& image_indices,
                        const std::vector<camera::Intrinsics>& cameras,
                        const std::vector<int>& image_to_camera_index,
                        std::vector<Eigen::Matrix3d>* poses_R,
                        std::vector<Eigen::Vector3d>* poses_C, std::vector<bool>* registered,
                        int min_inliers, std::vector<int>* registered_images_out) {
  if (!poses_R || !poses_C || !registered)
    return 0;
  if (registered_images_out)
    registered_images_out->clear();
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
    double rmse_px = 0.0;
    // const double ransac_thresh_px = 8.0;
    const double ransac_thresh_px =
        4.0; // tighter threshold since we have good initialization from tracks
    if (!resection_single_image(K, store, im, &R, &t, min_inliers, ransac_thresh_px, &inliers,
                                &rmse_px)) {
      LOG(INFO) << "  resection image " << im << ": FAILED (3D-2D=" << n_3d2d
                << ", inliers=" << inliers << ", rmse=" << rmse_px << ", need " << min_inliers
                << ")";
      continue;
    }
    Eigen::Vector3d C = -R.transpose() * t;
    LOG(INFO) << "  resection image " << im << ": OK (3D-2D=" << n_3d2d << ", inliers=" << inliers
              << ", rmse=" << rmse_px << ", C=[" << C(0) << "," << C(1) << "," << C(2) << "])";
    if (static_cast<size_t>(im) >= poses_R->size())
      poses_R->resize(static_cast<size_t>(im) + 1);
    if (static_cast<size_t>(im) >= poses_C->size())
      poses_C->resize(static_cast<size_t>(im) + 1);
    if (static_cast<size_t>(im) >= registered->size())
      registered->resize(static_cast<size_t>(im) + 1);
    (*poses_R)[static_cast<size_t>(im)] = R;
    (*poses_C)[static_cast<size_t>(im)] = C;
    (*registered)[static_cast<size_t>(im)] = true;
    if (registered_images_out)
      registered_images_out->push_back(im);
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
                            const std::vector<int>& image_to_camera_index, double min_tri_angle_deg,
                            std::vector<int>* new_track_ids_out) {
  if (new_track_ids_out)
    new_track_ids_out->clear();
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
    if (new_track_ids_out)
      new_track_ids_out->push_back(track_id);
  }
  auto t1 = std::chrono::steady_clock::now();
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
  LOG(INFO) << "[PERF] run_batch_triangulation: " << ms << "ms"
            << "  total_tracks=" << store->num_tracks() << "  candidates=" << candidate_set.size()
            << "  scanned=" << tracks_scanned << "  skip_few_views=" << tracks_skipped_few_views
            << "  skip_depth=" << tracks_skipped_depth << "  skip_angle=" << tracks_skipped_angle
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
            << "  total_tracks=" << store->num_tracks() << "  scanned_untri=" << scanned
            << "  skip_few_views=" << skip_few << "  skip_depth=" << skip_depth
            << "  skip_angle=" << skip_angle << "  re_tri=" << updated;
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
  std::sort(
      score_list.begin(), score_list.end(),
      [](const std::pair<int, int>& a, const std::pair<int, int>& b) { return a.first > b.first; });
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
  for (int g : variable_set)
    all_image_set.insert(g);
  for (int g : constant_set)
    all_image_set.insert(g);
  std::vector<int>& global_indices = *ba_image_index_to_global_out;
  global_indices.assign(all_image_set.begin(), all_image_set.end());
  if (global_indices.empty())
    return false;

  // ── Populate BAInput ──────────────────────────────────────────────────────
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
                   const std::vector<bool>* camera_frozen, uint32_t partial_intr_fix,
                   double focal_prior_weight, const BASolverOverrides& solver_overrides,
                   const std::vector<uint32_t>* partial_intr_fix_per_cam) {
  if (!store || !poses_R || !poses_C)
    return false;
  std::vector<int> ba_image_index_to_global;
  std::vector<int> point_index_to_track_id;
  BAInput ba_in;
  if (!build_ba_input_from_store(*store, *poses_R, *poses_C, registered, image_to_camera_index,
                                 cameras, nullptr, &ba_image_index_to_global, &ba_in,
                                 &point_index_to_track_id))
    return false;
  // Apply optional Ceres solver overrides.
  ba_in.solver_gradient_tolerance = solver_overrides.gradient_tolerance;
  ba_in.solver_function_tolerance = solver_overrides.function_tolerance;
  ba_in.solver_parameter_tolerance = solver_overrides.parameter_tolerance;
  ba_in.solver_dense_schur_max_variable_cams = solver_overrides.dense_schur_max_variable_cams;
  if (solver_overrides.max_num_iterations > 0)
    ba_in.solver_max_num_iterations = solver_overrides.max_num_iterations;
  if (solver_overrides.huber_loss_delta > 0.0)
    ba_in.huber_loss_delta = solver_overrides.huber_loss_delta;
  // Build per-camera intrinsics fix flags.
  // camera_frozen: frozen cameras keep kFixIntrAll (Schur-complement sparsity benefit).
  // partial_intr_fix_per_cam (priority): per-camera schedule mask based on each camera's own
  //   registered image count. When provided, replaces the scalar partial_intr_fix for
  //   non-frozen cameras, enabling independent phase progression per camera.
  // partial_intr_fix (fallback): scalar applied to all non-frozen cameras when
  //   partial_intr_fix_per_cam is not provided.
  {
    ba_in.fix_intrinsics_flags.assign(ba_in.cameras.size(), 0u);
    if (!optimize_intrinsics) {
      ba_in.fix_intrinsics_flags.assign(ba_in.cameras.size(),
                                        static_cast<uint32_t>(FixIntrinsicsMask::kFixIntrAll));
    } else {
      const bool use_per_cam =
          partial_intr_fix_per_cam && partial_intr_fix_per_cam->size() == ba_in.cameras.size();
      for (size_t c = 0; c < ba_in.cameras.size(); ++c) {
        const bool frozen = (camera_frozen && c < camera_frozen->size() && (*camera_frozen)[c]);
        if (frozen) {
          ba_in.fix_intrinsics_flags[c] = static_cast<uint32_t>(FixIntrinsicsMask::kFixIntrAll);
        } else {
          ba_in.fix_intrinsics_flags[c] =
              use_per_cam ? (*partial_intr_fix_per_cam)[c] : partial_intr_fix;
        }
      }
    }
    bool any_variable = false;
    for (const uint32_t f : ba_in.fix_intrinsics_flags)
      if (f != static_cast<uint32_t>(FixIntrinsicsMask::kFixIntrAll)) {
        any_variable = true;
        break;
      }
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
                  const std::vector<int>* indices_to_optimize, int anchor_image,
                  const BASolverOverrides& overrides) {
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
  if (overrides.max_num_iterations > 0)
    ba_in.solver_max_num_iterations = overrides.max_num_iterations;
  if (overrides.huber_loss_delta > 0.0)
    ba_in.huber_loss_delta = overrides.huber_loss_delta;
  if (overrides.function_tolerance > 0.0)
    ba_in.solver_function_tolerance = overrides.function_tolerance;
  if (overrides.gradient_tolerance > 0.0)
    ba_in.solver_gradient_tolerance = overrides.gradient_tolerance;
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
                         std::vector<Eigen::Vector3d>* poses_C, const std::vector<bool>& registered,
                         const std::vector<int>& image_to_camera_index,
                         const std::vector<camera::Intrinsics>& cameras,
                         const std::vector<int>& batch, int max_variable_cameras,
                         int max_iterations, double* rmse_px_out,
                         const BASolverOverrides& overrides) {
  if (!store || !poses_R || !poses_C || batch.empty())
    return false;
  std::vector<int> ba_image_index_to_global;
  std::vector<int> point_index_to_track_id;
  BAInput ba_in;
  if (!build_ba_input_colmap_local(*store, *poses_R, *poses_C, registered, image_to_camera_index,
                                   cameras, batch, max_variable_cameras, &ba_image_index_to_global,
                                   &ba_in, &point_index_to_track_id))
    return false;
  // Local BA never optimises intrinsics: ensure consistent kFixIntrAll regardless of how
  // build_ba_input_colmap_local initialised fix_intrinsics_flags.
  ba_in.optimize_intrinsics = false;
  ba_in.fix_intrinsics_flags.assign(ba_in.cameras.size(),
                                    static_cast<uint32_t>(FixIntrinsicsMask::kFixIntrAll));
  if (overrides.max_num_iterations > 0)
    ba_in.solver_max_num_iterations = overrides.max_num_iterations;
  if (overrides.huber_loss_delta > 0.0)
    ba_in.huber_loss_delta = overrides.huber_loss_delta;
  if (overrides.function_tolerance > 0.0)
    ba_in.solver_function_tolerance = overrides.function_tolerance;
  if (overrides.gradient_tolerance > 0.0)
    ba_in.solver_gradient_tolerance = overrides.gradient_tolerance;
  int n_variable = 0, n_constant = 0;
  for (bool f : ba_in.fix_pose) {
    if (f)
      ++n_constant;
    else
      ++n_variable;
  }
  LOG(INFO) << "run_local_ba_colmap: variable=" << n_variable << " constant=" << n_constant
            << " images=" << ba_image_index_to_global.size() << " points=" << ba_in.points3d.size()
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
      if (taken >= neighbor_k)
        break;
      constant_set.insert(p.first);
      ++taken;
    }
  }

  // ── Image sets ────────────────────────────────────────────────────────────
  std::set<int> all_image_set;
  for (int g : batch)
    all_image_set.insert(g);
  for (int g : constant_set)
    all_image_set.insert(g);
  if (all_image_set.empty())
    return false;

  std::vector<int>& global_indices = *ba_image_index_to_global_out;
  global_indices.assign(all_image_set.begin(), all_image_set.end());

  // ── BAInput: poses + cameras ──────────────────────────────────────────────
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
      if (all_image_set.count(static_cast<int>(o.image_index)))
        ++vis;
    if (vis < 2)
      return false;
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
  for (int tid : new_track_ids)
    try_add_track(tid, false);

  // Constant anchor points: old tracks shared between batch and constant cameras.
  // Collect candidates from constant cameras' reverse index, then check batch visibility.
  std::unordered_set<int> old_candidates;
  for (int c : constant_set) {
    track_ids_buf.clear();
    store.get_image_track_observations(c, &track_ids_buf, nullptr);
    for (int tid : track_ids_buf)
      if (!new_track_set.count(tid))
        old_candidates.insert(tid);
  }
  for (int tid : old_candidates) {
    // Must also be visible from at least one batch image to constrain the new camera pose.
    obs_buf.clear();
    store.get_track_observations(tid, &obs_buf);
    bool seen_from_batch = false;
    for (const auto& o : obs_buf)
      if (batch_set.count(static_cast<int>(o.image_index))) {
        seen_from_batch = true;
        break;
      }
    if (seen_from_batch)
      try_add_track(tid, true);
  }

  if (ba.points3d.empty())
    return false;

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

bool run_local_ba_batch_neighbor(
    TrackStore* store, std::vector<Eigen::Matrix3d>* poses_R, std::vector<Eigen::Vector3d>* poses_C,
    const std::vector<bool>& registered, const std::vector<int>& image_to_camera_index,
    const std::vector<camera::Intrinsics>& cameras, const std::vector<int>& batch,
    const std::vector<int>& new_track_ids, int neighbor_k, int max_iterations, double* rmse_px_out,
    const BASolverOverrides& overrides) {
  if (!store || !poses_R || !poses_C || batch.empty())
    return false;
  std::vector<int> ba_image_index_to_global;
  std::vector<int> point_index_to_track_id;
  BAInput ba_in;
  if (!build_ba_input_batch_neighbor(*store, *poses_R, *poses_C, registered, image_to_camera_index,
                                     cameras, batch, new_track_ids, neighbor_k,
                                     &ba_image_index_to_global, &ba_in, &point_index_to_track_id))
    return false;
  int n_variable_cams = 0, n_constant_cams = 0, n_variable_pts = 0, n_constant_pts = 0;
  for (bool f : ba_in.fix_pose) {
    if (f)
      ++n_constant_cams;
    else
      ++n_variable_cams;
  }
  for (bool f : ba_in.fix_point) {
    if (f)
      ++n_constant_pts;
    else
      ++n_variable_pts;
  }
  LOG(INFO) << "run_local_ba_batch_neighbor: var_cams=" << n_variable_cams
            << " const_cams=" << n_constant_cams << " var_pts=" << n_variable_pts
            << " const_pts=" << n_constant_pts << " obs=" << ba_in.observations.size();
  if (overrides.max_num_iterations > 0)
    ba_in.solver_max_num_iterations = overrides.max_num_iterations;
  if (overrides.huber_loss_delta > 0.0)
    ba_in.huber_loss_delta = overrides.huber_loss_delta;
  if (overrides.function_tolerance > 0.0)
    ba_in.solver_function_tolerance = overrides.function_tolerance;
  if (overrides.gradient_tolerance > 0.0)
    ba_in.solver_gradient_tolerance = overrides.gradient_tolerance;
  BAResult ba_out;
  if (!global_bundle_analytic(ba_in, &ba_out, max_iterations)) {
    LOG(WARNING) << "run_local_ba_batch_neighbor: solver failed";
    return false;
  }
  LOG(INFO) << "run_local_ba_batch_neighbor: RMSE=" << ba_out.rmse_px << " px";
  write_ba_result_back(ba_out, ba_image_index_to_global, point_index_to_track_id, store, poses_R,
                       poses_C, nullptr);
  if (rmse_px_out)
    *rmse_px_out = ba_out.rmse_px;
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Huber δ estimation from reprojection errors via robust MAD.
//
//   σ_est = 1.4826 × MAD(e)
//   δ     = clip(1.345 × σ_est, lo_px, hi_px)   ≈ clip(1.993 × MAD(e), lo, hi)
//
// The 1.345 factor gives ≈95% efficiency vs. OLS for normal residuals.
// lo/hi clips prevent the Huber kernel from becoming too narrow (aggressive
// outlier treatment when σ is underestimated) or too wide (no robustness).
// ─────────────────────────────────────────────────────────────────────────────
static double compute_huber_delta(const std::vector<double>& errors, double lo_px, double hi_px) {
  if (errors.empty())
    return hi_px;
  const size_t n = errors.size();
  std::vector<double> tmp(errors);
  auto mid = tmp.begin() + static_cast<ptrdiff_t>(n / 2);
  std::nth_element(tmp.begin(), mid, tmp.end());
  const double med = *mid;
  for (double& v : tmp)
    v = std::abs(v - med);
  std::nth_element(tmp.begin(), mid, tmp.end());
  const double mad = *mid;
  const double sigma = 1.4826 * mad;
  const double delta = 1.345 * sigma;
  return std::max(lo_px, std::min(hi_px, delta));
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
                             opts.init.min_tracks_for_intital_pair, im0_ptr, im1_ptr, poses_R_out,
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
  // Warm-up GPU context before entering the main loop so the first resection
  // does not absorb the ~1-second EGL/shader-compile cost.
  set_resection_backend(opts.resection.backend);
  LOG(INFO) << "run_incremental_sfm_pipeline: resection_backend="
            << resection_backend_name(get_resection_backend());
  resection_init_gpu();
  if (resection_backend_uses_gpu(get_resection_backend())) {
    gpu_geo_set_solver(0); // use jacobian svd, resection slower but more robust
  }

  // ────────────────────────────────────────────────────────────────────────
  // The initial-pair im0 defines the world coordinate origin (C=0, R=I).
  // All global BA calls must fix this camera to preserve that origin.
  const int anchor_image = static_cast<int>(*im0_ptr);
  LOG(INFO) << "run_incremental_sfm_pipeline: anchor_image=" << anchor_image;
  const BASolverOverrides& ba_solver_ov = opts.global_ba.solver_overrides;

  // count_tri_tracks: O(1) via TrackStore counter instead of full scan.
  auto count_tri_tracks = [&]() -> int { return store_out->num_triangulated_tracks(); };
  LOG(INFO) << "Triangulated tracks after initial pair: " << count_tri_tracks();

  using Clock = std::chrono::steady_clock;

  // ─────────────────────────────────────────────────────────────────────────────
  // Local BA dispatcher — selects colmap / batch-neighbor / window strategy.
  // ─────────────────────────────────────────────────────────────────────────────
  auto dispatch_local_ba = [&](const std::vector<int>& lba_batch,
                               const std::vector<int>& lba_new_tracks, const BASolverOverrides& ov,
                               double* rmse_out) -> bool {
    if (opts.local_ba.strategy == LocalBAStrategy::kColmap) {
      return run_local_ba_colmap(store_out, poses_R_out, poses_C_out, *registered_out,
                                 image_to_camera_index, *cameras, lba_batch,
                                 opts.local_ba.colmap_max_variable_images,
                                 opts.local_ba.max_iterations, rmse_out, ov);
    } else if (opts.local_ba.strategy == LocalBAStrategy::kBatchNeighbor) {
      return run_local_ba_batch_neighbor(store_out, poses_R_out, poses_C_out, *registered_out,
                                         image_to_camera_index, *cameras, lba_batch, lba_new_tracks,
                                         opts.local_ba.neighbor_k, opts.local_ba.max_iterations,
                                         rmse_out, ov);
    } else {
      const std::vector<int>* local_indices = nullptr;
      std::vector<int> connectivity_indices;
      if (opts.local_ba.by_connectivity && !lba_batch.empty()) {
        connectivity_indices = choose_local_ba_indices_by_connectivity(
            *store_out, *registered_out, lba_batch, opts.local_ba.window);
        if (!connectivity_indices.empty())
          local_indices = &connectivity_indices;
      }
      return run_local_ba(store_out, poses_R_out, poses_C_out, *registered_out,
                          image_to_camera_index, *cameras, opts.local_ba.window,
                          opts.local_ba.max_iterations, rmse_out, local_indices, anchor_image, ov);
    }
  };

  // ─────────────────────────────────────────────────────────────────────────────
  // Unified BA + iterative outlier detection.
  //
  // Each round:
  //   1. Collect reprojection errors → compute Huber δ via MAD.
  //   2. Run BA (global or local) with that Huber kernel.
  //   3. Collect post-BA errors → MAD hard-rejection threshold.
  //   4. reject_outliers_depth + multiview + angle.
  //   5. Repeat until convergence (rejected < min_for_retry) or max_rounds.
  //
  // Parameters:
  //   use_global          – true = global BA, false = dispatch_local_ba
  //   optimize_intrinsics – passed to run_global_ba (global BA only)
  //   fix_mask            – partial intrinsics fix flags (global BA only)
  //   focal_prior_weight  – focal length prior (global BA only)
  //   frozen_ptr          – per-camera frozen flags (global BA only)
  //   lba_batch / lba_new_tracks – for local BA
  //   new_images          – newly registered image indices; when non-empty, Huber δ is estimated
  //                         from THEIR residuals only (new matches are noisier than settled ones).
  //                         MAD hard-rejection threshold still uses all registered images.
  //   max_rounds          – max Huber-BA + reject iterations
  //   base_ov             – base solver overrides; detection rounds tighten tolerances
  //   rmse_out            – output: last BA RMSE (px)
  // ─────────────────────────────────────────────────────────────────────────────
  auto run_ba_with_outlier_detection =
      [&](bool use_global, bool optimize_intrinsics, uint32_t fix_mask, double focal_prior_weight,
          const std::vector<bool>* frozen_ptr, const std::vector<int>& lba_batch,
          const std::vector<int>& lba_new_tracks, const std::vector<int>& new_images,
          int max_rounds, const BASolverOverrides& base_ov, double* rmse_out) -> bool {
    double rmse = 0.0;
    bool ok = false;
    const bool do_reject = (num_registered >= opts.outlier.min_registered_images);
    const int n_rounds = std::max(1, max_rounds);
    for (int r = 0; r < n_rounds; ++r) {
      BASolverOverrides ov = base_ov;
      if (do_reject) {
        // Compute Huber δ from NEW images when available: they carry the freshest
        // (and noisiest) matches and set the appropriate scale for robust BA.
        // The already-settled global reconstruction has tiny residuals → would
        // produce an unrealistically tight δ if used naively.
        const std::vector<int>* delta_filter = (!new_images.empty()) ? &new_images : nullptr;
        auto errs_delta =
            collect_reproj_errors(*store_out, *poses_R_out, *poses_C_out, *registered_out, *cameras,
                                  image_to_camera_index, delta_filter);
        if (errs_delta.empty()) // fallback: use global if new images have no tracked points yet
          errs_delta = collect_reproj_errors(*store_out, *poses_R_out, *poses_C_out,
                                             *registered_out, *cameras, image_to_camera_index);
        if (!errs_delta.empty()) {
          const double delta = compute_huber_delta(errs_delta, opts.outlier.huber_delta_lo_px,
                                                   opts.outlier.huber_delta_hi_px);
          // ov.huber_loss_delta = delta;
          ov.huber_loss_delta = 4.0;
          LOG(INFO) << "  [ba_outlier] round " << r << ": Huber \xce\xb4=" << delta << " px"
                    << (delta_filter
                            ? " (from " + std::to_string(new_images.size()) + " new images, " +
                                  std::to_string(errs_delta.size()) + " obs)"
                            : " (global)");
        }
      }
      ov.huber_loss_delta = 10.0;
      // Detection rounds (r>0): tighten solver tolerances for speed.
      // if (r > 0)
      if (!optimize_intrinsics) {
        if (ov.function_tolerance == 0.0)
          ov.function_tolerance = 1e-3;
        if (ov.gradient_tolerance == 0.0)
          ov.gradient_tolerance = 1e-4;
        if (ov.max_num_iterations == 0)
          ov.max_num_iterations = 15;
      }
      auto t_ba = Clock::now();
      use_global = true;
      if (use_global) {
        // Compute per-camera phase masks: each camera's unlock stage is determined by
        // the number of its own registered images, not the total registered count.
        std::vector<uint32_t> per_cam_masks;
        if (optimize_intrinsics)
          per_cam_masks = opts.intrinsics.fix_masks_per_camera(
              *registered_out, image_to_camera_index, static_cast<int>(cameras->size()));
        ok = run_global_ba(store_out, poses_R_out, poses_C_out, *registered_out,
                           image_to_camera_index, *cameras, cameras, optimize_intrinsics,
                           opts.global_ba.max_iterations, &rmse, anchor_image, frozen_ptr, fix_mask,
                           focal_prior_weight, ov, optimize_intrinsics ? &per_cam_masks : nullptr);
      } else {
        ok = dispatch_local_ba(lba_batch, lba_new_tracks, ov, &rmse);
      }
      LOG(INFO)
          << "[PERF] ba_outlier round " << r << ": "
          << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t_ba).count()
          << "ms  RMSE=" << rmse << " px";
      if (!ok)
        break;
      if (!do_reject)
        break;
      // auto errs = collect_reproj_errors(*store_out, *poses_R_out, *poses_C_out, *registered_out,
      //                                   *cameras, image_to_camera_index);
      // double thr =
      //     compute_mad_threshold_from_errors(errs, opts.outlier.mad_k, opts.outlier.threshold_px);
      // if (thr <= 4.0) {
      //   thr = 4.0;
      // }
      double thr = 8.0;
      if (optimize_intrinsics) {
        thr = 4.0;
      }
      int rejected = 0;
      // reject_outliers_depth(store_out, *poses_R_out, *poses_C_out, *registered_out,
      //                                      opts.outlier.max_depth_factor);
      rejected += reject_outliers_multiview(store_out, *poses_R_out, *poses_C_out, *registered_out,
                                            *cameras, image_to_camera_index, thr);
      rejected += reject_outliers_angle_multiview(store_out, *poses_R_out, *poses_C_out,
                                                  *registered_out, opts.outlier.min_angle_deg);
      LOG(INFO) << "  [ba_outlier] round " << r << ": MAD_thr=" << thr
                << " px  rejected=" << rejected;
      if (rejected < std::max(1, opts.outlier.min_for_retry))
        break;
    }
    if (rmse_out)
      *rmse_out = rmse;
    return ok;
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
    if (!opts.intrinsics.progressive_freeze)
      return;
    for (int c = 0; c < n_cameras; ++c) {
      auto& fs = cam_freeze[static_cast<size_t>(c)];
      const camera::Intrinsics& K = (*cameras)[static_cast<size_t>(c)];
      if (fs.frozen) {
        if (!recheck_frozen)
          continue;
        // Re-evaluate: unfreeze if intrinsics drifted beyond thresholds.
        const auto d = K.delta(fs.prev);
        if (!d.stable(opts.intrinsics.freeze_delta_focal, opts.intrinsics.freeze_delta_pp,
                      opts.intrinsics.freeze_delta_dist)) {
          fs.frozen = false;
          fs.stable_rounds = 0;
          LOG(INFO) << "  [freeze] camera " << c << " UNFROZEN"
                    << " focal_rel=" << d.focal_rel << " pp_px=" << d.pp_px
                    << " dist=" << d.distortion;
        } else {
          LOG(INFO) << "  [freeze] camera " << c << " still FROZEN"
                    << " focal_rel=" << d.focal_rel << " pp_px=" << d.pp_px
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
      if (cam_reg < opts.intrinsics.freeze_min_images) {
        // Level-1 gate not met yet; don't count stable_rounds but still update prev.
        LOG(INFO) << "  [freeze] camera " << c << " L1-pending: reg=" << cam_reg << "/"
                  << opts.intrinsics.freeze_min_images << " images (need more)";
        fs.prev = K;
        continue;
      }
      const auto d = K.delta(fs.prev);
      if (d.stable(opts.intrinsics.freeze_delta_focal, opts.intrinsics.freeze_delta_pp,
                   opts.intrinsics.freeze_delta_dist)) {
        ++fs.stable_rounds;
        if (fs.stable_rounds >= opts.intrinsics.freeze_stable_rounds) {
          fs.frozen = true;
          LOG(INFO) << "  [freeze] camera " << c << " FROZEN"
                    << " focal_rel=" << d.focal_rel << " pp_px=" << d.pp_px
                    << " dist=" << d.distortion << " reg_imgs=" << cam_reg;
        } else {
          LOG(INFO) << "  [freeze] camera " << c << " stable_rounds=" << fs.stable_rounds << "/"
                    << opts.intrinsics.freeze_stable_rounds << " focal_rel=" << d.focal_rel
                    << " pp_px=" << d.pp_px << " dist=" << d.distortion << " reg_imgs=" << cam_reg;
        }
      } else {
        LOG(INFO) << "  [freeze] camera " << c << " not-stable → reset"
                  << " focal_rel=" << d.focal_rel << " (thr=" << opts.intrinsics.freeze_delta_focal
                  << ")"
                  << " pp_px=" << d.pp_px << " (thr=" << opts.intrinsics.freeze_delta_pp << ")"
                  << " dist=" << d.distortion << " (thr=" << opts.intrinsics.freeze_delta_dist
                  << ")"
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

  // COLMAP strategy registers exactly one image per iteration, then immediately runs
  // local BA on it (2-hop expansion centered on that single camera).  Batching >1
  // violates this design: the "batch" in run_local_ba_colmap is meant to be a single
  // newly-registered image, and its COLMAP-paper variable-set expansion only makes
  // sense when the batch contains just one new camera.

  const bool kColmapOneByOne =
      (opts.local_ba.strategy == LocalBAStrategy::kColmap && !opts.local_ba.skip);
  int effective_batch_max = kColmapOneByOne ? 1 : opts.resection.batch_max;

  // In kColmap mode: ranked candidate list (primary + fallbacks) rebuilt each iter.
  // The outer loop tries them one-by-one and uses the first that succeeds.
  std::vector<ResectionCandidate> colmap_candidates;

  auto pipeline_start = Clock::now();
  int sfm_iter = 0;
  for (;;) {
    ++sfm_iter;
    auto iter_start = Clock::now();
    LOG(INFO) << "════ SfM iter #" << sfm_iter << ": registered=" << num_registered << "/"
              << n_images << ", tri_tracks=" << count_tri_tracks() << " ════";

    auto t_choose0 = Clock::now();
    std::vector<int> batch;
    if (kColmapOneByOne) {
      colmap_candidates = choose_resection_candidates(
          *store_out, *registered_out, opts.resection.min_3d2d_count, opts.resection.batch_ratio,
          /*batch_max=*/5, /*max_candidates=*/20 + opts.local_ba.colmap_fallback_n,
          opts.resection.late_registered_threshold, opts.resection.late_absolute_min,
          opts.resection.late_batch_max);
      if (!colmap_candidates.empty())
        batch = {colmap_candidates[0].image_index};
    } else {
      batch = choose_next_resection_batch(
          *store_out, *registered_out, opts.resection.min_3d2d_count, opts.resection.batch_ratio,
          effective_batch_max, opts.resection.late_registered_threshold,
          opts.resection.late_absolute_min, opts.resection.late_batch_max);
    }
    auto t_choose1 = Clock::now();
    LOG(INFO)
        << "[PERF] choose_batch: "
        << std::chrono::duration_cast<std::chrono::milliseconds>(t_choose1 - t_choose0).count()
        << "ms";

    if (batch.empty()) {
      if (opts.resection.retry_after_cleanup &&
          num_registered >= opts.outlier.min_registered_images) {
        double rmse = 0.0;
        // Cleanup BA: allow intrinsics only when enough cameras are registered.
        const bool cleanup_opt_intr =
            opts.global_ba.optimize_intrinsics &&
            num_registered >= opts.global_ba.optimize_intrinsics_min_images;
        const uint32_t refine_mask_c =
            cleanup_opt_intr ? opts.intrinsics.fix_mask_for(num_registered) : 0u;
        auto t_cba0 = Clock::now();
        const auto frozen_cleanup = (cleanup_opt_intr && opts.intrinsics.progressive_freeze)
                                        ? make_frozen_vec()
                                        : std::vector<bool>{};
        if (run_ba_with_outlier_detection(true, cleanup_opt_intr, refine_mask_c,
                                          opts.intrinsics.focal_prior_weight,
                                          cleanup_opt_intr ? &frozen_cleanup : nullptr, {}, {}, {},
                                          opts.outlier.max_rounds, ba_solver_ov, &rmse)) {
          if (cleanup_opt_intr)
            update_freeze_state();
          LOG(INFO) << "[PERF] cleanup_global_ba: "
                    << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t_cba0)
                           .count()
                    << "ms  RMSE=" << rmse;
          if (kColmapOneByOne) {
            colmap_candidates = choose_resection_candidates(
                *store_out, *registered_out, opts.resection.min_3d2d_count,
                opts.resection.batch_ratio, /*batch_max=*/5,
                /*max_candidates=*/20 + opts.local_ba.colmap_fallback_n,
                opts.resection.late_registered_threshold, opts.resection.late_absolute_min,
                opts.resection.late_batch_max);
            batch = colmap_candidates.empty() ? std::vector<int>{}
                                              : std::vector<int>{colmap_candidates[0].image_index};
          } else {
            batch = choose_next_resection_batch(
                *store_out, *registered_out, opts.resection.min_3d2d_count,
                opts.resection.batch_ratio, effective_batch_max,
                opts.resection.late_registered_threshold, opts.resection.late_absolute_min,
                opts.resection.late_batch_max);
          }
        }
      }
      if (batch.empty())
        break;
    }

    auto t_resect0 = Clock::now();
    int added = 0;
    if (kColmapOneByOne) {
      // Try each candidate in descending-score order; use the first that succeeds.
      // This guards against degenerate configurations (collinear/planar points, etc.)
      // that would cause the top-ranked image's resection to fail silently.
      for (const ResectionCandidate& cand : colmap_candidates) {
        std::vector<int> registered_images;
        int resectin_minliers = 50;
        const int n =
            run_batch_resection(*store_out, {cand.image_index}, *cameras, image_to_camera_index,
                                poses_R_out, poses_C_out, registered_out,
                                static_cast<int>(resectin_minliers), &registered_images);
        LOG(INFO) << "  [kColmap] img=" << cand.image_index << " 3d2d=" << cand.num_3d2d
                  << " cov=" << cand.coverage << (cand.is_primary ? "" : " [fallback]") << " → "
                  << (n > 0 ? "OK" : "FAIL");
        if (n <= 0)
          continue;
        added =  1;
        batch = {registered_images[0]}; // downstream sees only accepted new image

        // const double post_resection_reproj_thr_px = std::max(6.0, opts.outlier.threshold_px * 2.0);
        // const int n_rejected_post_resection = reject_outliers_post_resection(
        //     store_out, registered_images, *poses_R_out, *poses_C_out, *registered_out, *cameras,
        //     image_to_camera_index, post_resection_reproj_thr_px);
        // std::vector<int> kept_images;
        // rollback_weak_post_resection_images(*store_out, registered_images,
        //                                     static_cast<int>(opts.resection.min_inliers),
        //                                     poses_R_out, poses_C_out, registered_out, &kept_images);
        // LOG(INFO) << "  post_resection_cleanup: rejected=" << n_rejected_post_resection
        //           << " obs on new images, reproj_thr=" << post_resection_reproj_thr_px
        //           << " px, kept_images=" << kept_images.size();
        // if (kept_images.empty())
        //   continue;
            
        // added = static_cast<int>(kept_images.size());
        // batch = kept_images; // downstream sees only accepted new images
        break;
      }
    } else {
      std::vector<int> registered_images;
      added = run_batch_resection(
          *store_out, batch, *cameras, image_to_camera_index, poses_R_out, poses_C_out,
          registered_out, static_cast<int>(2.5 * opts.resection.min_inliers), &registered_images);
      const double post_resection_reproj_thr_px = std::max(6.0, opts.outlier.threshold_px * 2.0);
      const int n_rejected_post_resection = reject_outliers_post_resection(
          store_out, registered_images, *poses_R_out, *poses_C_out, *registered_out, *cameras,
          image_to_camera_index, post_resection_reproj_thr_px);
      std::vector<int> kept_images;
      rollback_weak_post_resection_images(*store_out, registered_images,
                                          static_cast<int>(opts.resection.min_inliers), poses_R_out,
                                          poses_C_out, registered_out, &kept_images);
      LOG(INFO) << "  post_resection_cleanup: rejected=" << n_rejected_post_resection
                << " obs on new images, reproj_thr=" << post_resection_reproj_thr_px
                << " px, kept_images=" << kept_images.size();
      batch = kept_images;
      added = static_cast<int>(kept_images.size());
    }
    LOG(INFO)
        << "[PERF] resection: "
        << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t_resect0).count()
        << "ms  added=" << added;
    if (added == 0) {
      LOG(WARNING) << "  No images could be resected in this batch, stopping";
      break;
    }

    auto t_tri0 = Clock::now();
    std::vector<int> new_track_ids;
    int n_new_tri = run_batch_triangulation(store_out, batch, *poses_R_out, *poses_C_out,
                                            *registered_out, *cameras, image_to_camera_index,
                                            opts.triangulation.min_angle_deg, &new_track_ids);
    LOG(INFO)
        << "[PERF] triangulation: "
        << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t_tri0).count()
        << "ms  new_tri=" << n_new_tri;

    num_registered = 0;
    for (bool r : *registered_out)
      if (r)
        ++num_registered;
    LOG(INFO) << "  After resection+triangulation: registered=" << num_registered
              << ", new_tri=" << n_new_tri << ", total_tri=" << count_tri_tracks();

    double rmse = 0.0;
    // if (num_registered < opts.local_ba.switch_after_n_images)
    if (true) {
      // Run global BA every global_ba_every_n_images registrations, and always
      // on the last image before switching to local BA.
      const bool do_global_ba = (opts.global_ba.every_n_images <= 1) ||
                                (num_registered % opts.global_ba.every_n_images == 0) ||
                                (num_registered + 1 == opts.local_ba.switch_after_n_images);

      // if (do_global_ba)
      if (true) {
        // const bool opt_intr = opts.global_ba.optimize_intrinsics &&
        //                       num_registered >= opts.global_ba.optimize_intrinsics_min_images;
        // bool opt_intr = opts.global_ba.optimize_intrinsics;
        bool opt_intr = true;
        // 主要由phase策略来决定内参的优化顺序
        uint32_t refine_mask_e = opt_intr ? opts.intrinsics.fix_mask_for(num_registered) : 0u;
        auto t_ba0 = Clock::now();
        auto frozen_early = (opt_intr && opts.intrinsics.progressive_freeze) ? make_frozen_vec()
                                                                             : std::vector<bool>{};
        run_ba_with_outlier_detection(true, opt_intr, refine_mask_e,
                                      opts.intrinsics.focal_prior_weight,
                                      opt_intr ? &frozen_early : nullptr, {}, {}, batch,
                                      opts.outlier.max_rounds, ba_solver_ov, &rmse);
        LOG(INFO)
            << "[PERF] global_ba (early): "
            << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t_ba0).count()
            << "ms  n=" << num_registered << "  RMSE=" << rmse << " px";
        // No freeze in early phase: too few images, intrinsics still converging.
        opt_intr = true;
        refine_mask_e = opt_intr ? opts.intrinsics.fix_mask_for(num_registered) : 0u;
        t_ba0 = Clock::now();
        frozen_early = (opt_intr && opts.intrinsics.progressive_freeze) ? make_frozen_vec()
                                                                        : std::vector<bool>{};
        run_ba_with_outlier_detection(true, opt_intr, refine_mask_e,
                                      opts.intrinsics.focal_prior_weight,
                                      opt_intr ? &frozen_early : nullptr, {}, {}, batch,
                                      opts.outlier.max_rounds, ba_solver_ov, &rmse);
        LOG(INFO)
            << "[PERF] global_ba (early): "
            << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t_ba0).count()
            << "ms  n=" << num_registered << "  RMSE=" << rmse << " px";
      } else {
        LOG(INFO) << "[PERF] global_ba: skipped (every_n=" << opts.global_ba.every_n_images
                  << ", n=" << num_registered << ")";
      }
    } else if (opts.local_ba.skip) {
      // ── Object-scan mode: skip local BA, run global BA every iteration ───────────────────
      const bool opt_skip = opts.global_ba.optimize_intrinsics &&
                            num_registered >= opts.global_ba.optimize_intrinsics_min_images;
      const uint32_t refine_mask_skip =
          opt_skip ? opts.intrinsics.fix_mask_for(num_registered) : 0u;
      auto t_sba = Clock::now();
      const auto frozen_skip = (opt_skip && opts.intrinsics.progressive_freeze)
                                   ? make_frozen_vec()
                                   : std::vector<bool>{};
      if (run_ba_with_outlier_detection(true, opt_skip, refine_mask_skip,
                                        opts.intrinsics.focal_prior_weight,
                                        opt_skip ? &frozen_skip : nullptr, {}, {}, batch,
                                        opts.outlier.max_rounds, ba_solver_ov, &rmse)) {
        if (opt_skip)
          update_freeze_state();
        LOG(INFO)
            << "[PERF] global_ba (skip_local_ba): "
            << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t_sba).count()
            << "ms  n=" << num_registered << "  RMSE=" << rmse << " px";
      }
    } else {
      // ── Two-tier BA schedule (mid-freq + low-freq recalib) during local-BA phase ──────
      if (opts.global_ba.periodic_every_n_images > 0 &&
          (num_registered - last_periodic_global_ba_registered) >=
              opts.global_ba.periodic_every_n_images) {
        last_periodic_global_ba_registered = num_registered;
        ++periodic_ba_count;
        const bool opt_intr_p = opts.global_ba.optimize_intrinsics &&
                                num_registered >= opts.global_ba.optimize_intrinsics_min_images;

        // Low-freq recalib: every recalib_global_ba_every_n_periodic mid-freq BAs.
        // Opens ALL intrinsics, then re-evaluates freeze state for every camera.
        const bool do_recalib = opt_intr_p && opts.intrinsics.progressive_freeze &&
                                opts.intrinsics.recalib_every_n_periodic > 0 &&
                                (periodic_ba_count % opts.intrinsics.recalib_every_n_periodic == 0);
        if (do_recalib) {
          auto t_pba = Clock::now();
          // Recalib BA — open all intrinsics, then re-evaluate freeze state.
          if (run_ba_with_outlier_detection(true, true, 0u, 0.0, nullptr, {}, {}, {},
                                            opts.outlier.max_rounds, ba_solver_ov, &rmse)) {
            update_freeze_state(/*recheck_frozen=*/true);
            LOG(INFO) << "[PERF] recalib_global_ba #" << periodic_ba_count << ": "
                      << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t_pba)
                             .count()
                      << "ms  n=" << num_registered << "  RMSE=" << rmse << " px";
          }
        } else {
          // Mid-freq global BA: frozen cameras use kFixIntrAll (Schur sparsity).
          // Unfrozen cameras optimise intrinsics; evaluates freeze convergence.
          const bool mid_opt_intr = opt_intr_p;
          const uint32_t refine_mask_mid =
              mid_opt_intr ? opts.intrinsics.fix_mask_for(num_registered) : 0u;
          const auto frozen_mid = (mid_opt_intr && opts.intrinsics.progressive_freeze)
                                      ? make_frozen_vec()
                                      : std::vector<bool>{};
          auto t_pba = Clock::now();
          if (run_ba_with_outlier_detection(true, mid_opt_intr, refine_mask_mid,
                                            opts.intrinsics.focal_prior_weight,
                                            mid_opt_intr ? &frozen_mid : nullptr, {}, {}, {},
                                            opts.outlier.max_rounds, ba_solver_ov, &rmse)) {
            if (mid_opt_intr)
              update_freeze_state(/*recheck_frozen=*/false);
            LOG(INFO) << "[PERF] periodic_global_ba #" << periodic_ba_count << ": "
                      << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t_pba)
                             .count()
                      << "ms  n=" << num_registered << "  RMSE=" << rmse << " px";
          }
        }
      }
      auto t_ba0 = Clock::now();
      run_ba_with_outlier_detection(false, false, 0u, 0.0, nullptr, batch, new_track_ids, batch,
                                    opts.outlier.max_rounds, BASolverOverrides{}, &rmse);
      LOG(INFO)
          << "[PERF] local_ba: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t_ba0).count()
          << "ms  n=" << num_registered << "  RMSE=" << rmse << " px";
    }

    // Periodic re-triangulation: recover tracks that were (a) cleared by outlier
    // rejection after losing their 3D support, or (b) skippable previously but now
    // have sufficient registered observers.  Full scan but fast in practice.
    if (false && opts.triangulation.retriangulation_every_n_iters > 0 &&
        sfm_iter % opts.triangulation.retriangulation_every_n_iters == 0) {
      auto t_retri = Clock::now();
      int n_retri =
          run_retriangulation(store_out, *poses_R_out, *poses_C_out, *registered_out, *cameras,
                              image_to_camera_index, opts.triangulation.min_angle_deg);
      LOG(INFO)
          << "[PERF] periodic_retri: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t_retri).count()
          << "ms  re_tri=" << n_retri << "  total_tri=" << count_tri_tracks();
    }

    LOG(INFO)
        << "[PERF] iter #" << sfm_iter << " total: "
        << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - iter_start).count()
        << "ms";
  }
  LOG(INFO) << "[PERF] main_loop total: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - pipeline_start)
                   .count()
            << "ms  iters=" << sfm_iter;

  // run_retriangulation(store_out, *poses_R_out, *poses_C_out, *registered_out, *cameras,
  //                     image_to_camera_index, opts.triangulation.min_angle_deg);
  // LOG(INFO) << "Final retriangulation done. tri_tracks=" << count_tri_tracks();

  if (opts.global_ba.enabled) {
    LOG(INFO) << "Running final global BA...";
    double rmse = 0.0;
    run_ba_with_outlier_detection(true, opts.global_ba.optimize_intrinsics, 0u, 0.0, nullptr, {},
                                  {}, {}, opts.outlier.final_max_rounds, ba_solver_ov, &rmse);
    LOG(INFO) << "Final BA RMSE=" << rmse << " px";
  }
  return true;
}

} // namespace sfm
} // namespace insight
