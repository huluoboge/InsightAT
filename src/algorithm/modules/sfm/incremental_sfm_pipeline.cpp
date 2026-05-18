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
#include "scene_normalization.h"
#include "track_store.h"
#include "two_view_reconstruction.h"
#include "view_graph.h"
#include "view_graph_loader.h"

#include <Eigen/Dense>
#include <PoseLib/robust.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <glog/logging.h>
#include <map>
#include <numeric>
#include <omp.h>
#include <set>
#include <sstream>
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
  if (n_registered < phase1_min_images)
    return static_cast<uint32_t>(M::kFixIntrAll);
  // Phase 1 [phase1_min, phase2_min): fx + k1; fix sigma, principal point, k2+, tangential
  if (n_registered < phase2_min_images)
    return static_cast<uint32_t>(M::kFixIntrSigma | M::kFixIntrCx | M::kFixIntrCy | M::kFixIntrK2 |
                                 M::kFixIntrK3 | M::kFixIntrP1 | M::kFixIntrP2);
  // Phase 2 [phase2_min, phase3_min): fx + k1 + k2
  if (n_registered < phase3_min_images)
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

/// Multi-view: mark observations with reprojection error > threshold_px.
/// Image-indexed traversal (only registered images) + parallel collect / serial apply.
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

  // ── Parallel phase: collect obs to delete ──────────────────────────────────
  // Each entry: (obs_id, restorable).  TrackStore writes happen in the serial phase.
  const int n_threads = omp_get_max_threads();
  std::vector<std::vector<std::pair<int, bool>>> to_delete(static_cast<size_t>(n_threads));

#pragma omp parallel for schedule(dynamic, 4)
  for (int im = 0; im < n_images; ++im) {
    if (!registered[static_cast<size_t>(im)])
      continue;
    const Eigen::Matrix3d& R = poses_R[static_cast<size_t>(im)];
    const Eigen::Vector3d& C = poses_C[static_cast<size_t>(im)];
    const camera::Intrinsics& K =
        cameras[static_cast<size_t>(image_to_camera_index[static_cast<size_t>(im)])];
    // Reuse per-thread vector to avoid repeated heap allocation across OMP iterations.
    thread_local std::vector<int> obs_ids;
    obs_ids.clear();
    store->get_image_observation_indices(im, &obs_ids);
    auto& local = to_delete[static_cast<size_t>(omp_get_thread_num())];
    for (int obs_id : obs_ids) {
      const int tid = store->obs_track_id(obs_id);
      if (!store->is_track_valid(tid) || !store->track_has_triangulated_xyz(tid))
        continue;
      float tx, ty, tz;
      store->get_track_xyz(tid, &tx, &ty, &tz);
      const Eigen::Vector3d X(static_cast<double>(tx), static_cast<double>(ty),
                              static_cast<double>(tz));
      const Eigen::Vector3d p = R * (X - C);
      if (p(2) <= 1e-12) {
        local.emplace_back(obs_id, false); // cheirality → permanent delete
        continue;
      }
      const double xn = p(0) / p(2), yn = p(1) / p(2);
      double xd, yd;
      camera::apply_distortion(xn, yn, K, &xd, &yd);
      Observation obs;
      store->get_obs(obs_id, &obs);
      const double du = static_cast<double>(obs.u) - (K.fx * xd + K.cx);
      const double dv = static_cast<double>(obs.v) - (K.fy * yd + K.cy);
      if (du * du + dv * dv > thresh_sq)
        local.emplace_back(obs_id, true); // reproj error → restorable delete
    }
  }

  // ── Serial phase: apply deletions ─────────────────────────────────────────
  int marked = 0;
  std::unordered_set<int> dirty_tracks;
  for (auto& local : to_delete) {
    for (const auto& [obs_id, restorable] : local) {
      const int tid = store->obs_track_id(obs_id);
      if (restorable)
        store->mark_observation_deleted_restorable(obs_id);
      else
        store->mark_observation_deleted(obs_id);
      dirty_tracks.insert(tid);
      ++marked;
    }
  }

  // Clear XYZ for tracks that now have fewer than 2 valid registered observations.
  std::vector<int> obs_ids_chk;
  for (int tid : dirty_tracks) {
    if (!store->is_track_valid(tid) || !store->track_has_triangulated_xyz(tid))
      continue;
    obs_ids_chk.clear();
    store->get_track_obs_ids(tid, &obs_ids_chk);
    int valid_registered_obs = 0;
    for (int oid : obs_ids_chk) {
      if (!store->is_obs_valid(oid))
        continue;
      Observation o;
      store->get_obs(oid, &o);
      const int im = static_cast<int>(o.image_index);
      if (im >= 0 && im < n_images && registered[static_cast<size_t>(im)])
        ++valid_registered_obs;
    }
    if (valid_registered_obs < 2)
      store->clear_track_xyz(tid);
  }
  return marked;
}

/// Mark observations whose max parallax angle (with any other view in the same track) is <
/// min_angle_deg.  Iterates tracks (not obs) to avoid redundant get_track_observations calls.
///
/// Fix A: OMP-parallelised over tracks (was the worst serial bottleneck at 3000+ images).
///   Parallel phase: O(N_tracks × k²) angle computation; results collected in per-thread vectors.
///   Serial phase 1: apply kRestorable deletions to TrackStore.
///   Serial phase 2: clear XYZ for tracks with < 2 valid registered observations.
int reject_outliers_angle_multiview(TrackStore* store, const std::vector<Eigen::Matrix3d>& poses_R,
                                    const std::vector<Eigen::Vector3d>& poses_C,
                                    const std::vector<bool>& registered, double min_angle_deg,
                                    double max_angle_deg) {
  if (!store || min_angle_deg <= 0)
    return 0;
  const int n_images = store->num_images();
  if (static_cast<int>(poses_R.size()) != n_images ||
      static_cast<int>(poses_C.size()) != n_images ||
      static_cast<int>(registered.size()) != n_images)
    return 0;
  const double min_angle_rad = min_angle_deg * (3.141592653589793 / 180.0);
  const double max_angle_rad = max_angle_deg * (3.141592653589793 / 180.0);
  // Precompute cos thresholds: angle ∈ [min_angle, max_angle] ⇔ dot ∈ [cos(max_angle), cos(min_angle)]
  // Using dot-product comparison avoids expensive std::acos() entirely.
  const double cos_min_angle = std::cos(min_angle_rad);
  const double cos_max_angle = std::cos(max_angle_rad);
  const int n_tracks = static_cast<int>(store->num_tracks());

  // ── CSR build: single-pass over registered images → contiguous per-track obs array ──────────
  // We store obs_id only (not image_index) — the latter is fetched on the fly from
  // store->obs_image_index() in Pass B, avoiding an extra parallel array and its fill pass.
  // Pass 0: count registered valid triangulated obs per track.
  std::vector<int> track_obs_count(static_cast<size_t>(n_tracks), 0);
  {
    std::vector<int> img_obs;
    for (int im = 0; im < n_images; ++im) {
      if (!registered[static_cast<size_t>(im)])
        continue;
      img_obs.clear();
      store->get_image_observation_indices(im, &img_obs);
      for (int obs_id : img_obs) {
        const int tid = store->obs_track_id(obs_id);
        if (tid < 0 || tid >= n_tracks)
          continue;
        if (!store->is_track_valid(tid) || !store->track_has_triangulated_xyz(tid))
          continue;
        ++track_obs_count[static_cast<size_t>(tid)];
      }
    }
  }

  // Build compact list of active tracks (count ≥ 2) to avoid O(n_total_tracks) loop in Pass B.
  // n_total_tracks includes all feature tracks (millions); active tracks are O(n_tri) ≪ n_total.
  std::vector<int> active_tracks;
  active_tracks.reserve(static_cast<size_t>(200000));
  for (int tid = 0; tid < n_tracks; ++tid)
    if (track_obs_count[static_cast<size_t>(tid)] >= 2)
      active_tracks.push_back(tid);

  // Prefix-sum → CSR start offsets.
  std::vector<int> track_start(static_cast<size_t>(n_tracks + 1), 0);
  for (int tid = 0; tid < n_tracks; ++tid)
    track_start[static_cast<size_t>(tid + 1)] =
        track_start[static_cast<size_t>(tid)] + track_obs_count[static_cast<size_t>(tid)];
  const int total_obs = track_start[static_cast<size_t>(n_tracks)];

  std::vector<int> obs_flat(static_cast<size_t>(total_obs));

  // Pass A: fill CSR obs_flat.  Write cursors start at track_start[tid].
  // image_flat is NOT stored — image index is obtained from store->obs_image_index()
  // during Pass B, saving ~8 bytes × total_obs of memory and the fill pass bandwidth.
  {
    std::vector<int> fill_ptr(track_start.begin(), track_start.begin() + n_tracks);
    std::vector<int> img_obs;
    for (int im = 0; im < n_images; ++im) {
      if (!registered[static_cast<size_t>(im)])
        continue;
      img_obs.clear();
      store->get_image_observation_indices(im, &img_obs);
      for (int obs_id : img_obs) {
        const int tid = store->obs_track_id(obs_id);
        if (tid < 0 || tid >= n_tracks)
          continue;
        if (!store->is_track_valid(tid) || !store->track_has_triangulated_xyz(tid))
          continue;
        obs_flat[static_cast<size_t>(fill_ptr[static_cast<size_t>(tid)]++)] = obs_id;
      }
    }
  }

  // ── Pass B (OMP-parallel over active tracks): angle check ────────────────────────────────────
  // For each observation i, find the first j≠i such that dot(ray_i, ray_j) ∈ [cos_max, cos_min].
  // If found → observation has sufficient parallax AND is not excessively far → good (skip).
  //
  // Optimisations applied (from slowest → fastest):
  //   1. Eliminated std::acos: compare dot product directly against cos thresholds.
  //      angle ∈ [θ_min, θ_max] ⇔ dot ∈ [cos(θ_max), cos(θ_min)]
  //   2. Early termination: break inner j-loop as soon as a valid pair is found.
  //   3. Split j-loop: j ∈ [0, i) ∪ (i, k) avoids the `j == i` branch in the hot inner path.
  //   4. image_flat eliminated: image index read from store->obs_image_index() on the fly.
  //      No parallel array allocation → less memory traffic and no second fill pass.
  //   5. Larger OMP chunk (256 → less scheduling overhead for fast per-track kernels).
  const int n_active = static_cast<int>(active_tracks.size());
  const int n_threads = omp_get_max_threads();
  std::vector<std::vector<int>> per_thread_marked(static_cast<size_t>(n_threads));
  std::vector<std::vector<int>> per_thread_dirty(static_cast<size_t>(n_threads));

  // Grab flat SoA views once (avoid per-call function overhead in the OMP parallel loop).
  const std::vector<uint32_t>& obs_img = store->obs_image_id_view();
  const float* track_xyz = store->track_xyz_data();

#pragma omp parallel for schedule(dynamic, 256)
  for (int ai = 0; ai < n_active; ++ai) {
    const int tid = active_tracks[static_cast<size_t>(ai)];
    const int k = track_obs_count[static_cast<size_t>(tid)];
    // Direct SoA access: track_xyz[tid*3 + 0..2]
    const float* xyz = track_xyz + static_cast<size_t>(tid) * 3;
    const Eigen::Vector3d X(static_cast<double>(xyz[0]), static_cast<double>(xyz[1]),
                            static_cast<double>(xyz[2]));
    const int start = track_start[static_cast<size_t>(tid)];

    // Compute all k rays once (cache-friendly: sequential access to poses_C via obs_img view).
    std::vector<Eigen::Vector3d> rays(static_cast<size_t>(k));
    for (int i = 0; i < k; ++i) {
      const int oid = obs_flat[static_cast<size_t>(start + i)];
      const int im = static_cast<int>(obs_img[static_cast<size_t>(oid)]);
      rays[static_cast<size_t>(i)] =
          (X - poses_C[static_cast<size_t>(im)]).normalized();
    }

    const int th = omp_get_thread_num();
    bool any_marked = false;
    for (int i = 0; i < k; ++i) {
      bool good = false;
      const Eigen::Vector3d& r_i = rays[static_cast<size_t>(i)];
      // j in [0, i): no `j == i` check needed.
      for (int j = 0; j < i; ++j) {
        const double dot = r_i.dot(rays[static_cast<size_t>(j)]);
        if (dot >= cos_max_angle && dot <= cos_min_angle) {
          good = true;
          goto next_obs;  // single-level break into the outer loop
        }
      }
      // j in (i, k): no `j == i` check needed.
      for (int j = i + 1; j < k; ++j) {
        const double dot = r_i.dot(rays[static_cast<size_t>(j)]);
        if (dot >= cos_max_angle && dot <= cos_min_angle) {
          good = true;
          break;
        }
      }
    next_obs:
      if (!good) {
        per_thread_marked[static_cast<size_t>(th)].push_back(
            obs_flat[static_cast<size_t>(start + i)]);
        any_marked = true;
      }
    }
    if (any_marked)
      per_thread_dirty[static_cast<size_t>(th)].push_back(tid);
  }

  // ── Merge & apply ─────────────────────────────────────────────────────────────────────────────
  int marked = 0;
  std::vector<bool> track_dirty(static_cast<size_t>(n_tracks), false);
  for (int th = 0; th < n_threads; ++th) {
    for (int oid : per_thread_marked[static_cast<size_t>(th)]) {
      store->mark_observation_deleted_restorable(oid);
      ++marked;
    }
    for (int tid : per_thread_dirty[static_cast<size_t>(th)])
      track_dirty[static_cast<size_t>(tid)] = true;
  }

  // Clear XYZ for dirty tracks that now have <2 valid registered views.
  for (int ai = 0; ai < n_active; ++ai) {
    const int tid = active_tracks[static_cast<size_t>(ai)];
    if (!track_dirty[static_cast<size_t>(tid)])
      continue;
    if (!store->is_track_valid(tid) || !store->track_has_triangulated_xyz(tid))
      continue;
    const int start = track_start[static_cast<size_t>(tid)];
    const int k = track_obs_count[static_cast<size_t>(tid)];
    int still_valid = 0;
    for (int i = 0; i < k; ++i) {
      if (store->is_obs_valid(obs_flat[static_cast<size_t>(start + i)]))
        ++still_valid;
    }
    if (still_valid < 2)
      store->clear_track_xyz(tid);
  }
  return marked;
}

/// Reject observations whose 3D point is behind the camera or depth > max_depth_factor × median.
/// Image-indexed traversal; pass 1 is parallel for fast median computation.
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

  // Pass 1 (parallel): collect positive depths from registered images for median computation.
  const int n_threads = omp_get_max_threads();
  std::vector<std::vector<double>> per_thread_depths(static_cast<size_t>(n_threads));
#pragma omp parallel for schedule(dynamic, 4)
  for (int im = 0; im < n_images; ++im) {
    if (!registered[static_cast<size_t>(im)])
      continue;
    // Reuse per-thread vector to avoid repeated heap allocation across OMP iterations.
    thread_local std::vector<int> obs_ids;
    obs_ids.clear();
    store->get_image_observation_indices(im, &obs_ids);
    auto& local = per_thread_depths[static_cast<size_t>(omp_get_thread_num())];
    for (int obs_id : obs_ids) {
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
        local.push_back(depth);
    }
  }

  std::vector<double> depths;
  {
    size_t total = 0;
    for (const auto& v : per_thread_depths)
      total += v.size();
    depths.reserve(total);
    for (auto& v : per_thread_depths)
      depths.insert(depths.end(), v.begin(), v.end());
  }

  double max_depth = std::numeric_limits<double>::max();
  if (max_depth_factor > 0.0 && !depths.empty()) {
    std::nth_element(depths.begin(), depths.begin() + static_cast<ptrdiff_t>(depths.size() / 2),
                     depths.end());
    max_depth = depths[depths.size() / 2] * max_depth_factor;
  }

  // Pass 2 (serial): mark observations violating depth bounds.
  int marked = 0;
  std::unordered_set<int> dirty_tracks;
  std::vector<int> obs_ids; // hoisted outside loop — reused across images
  for (int im = 0; im < n_images; ++im) {
    if (!registered[static_cast<size_t>(im)])
      continue;
    obs_ids.clear();
    store->get_image_observation_indices(im, &obs_ids);
    for (int obs_id : obs_ids) {
      const int tid = store->obs_track_id(obs_id);
      if (!store->is_track_valid(tid) || !store->track_has_triangulated_xyz(tid))
        continue;
      float tx, ty, tz;
      store->get_track_xyz(tid, &tx, &ty, &tz);
      const Eigen::Vector3d X(static_cast<double>(tx), static_cast<double>(ty),
                              static_cast<double>(tz));
      const double depth =
          (poses_R[static_cast<size_t>(im)] * (X - poses_C[static_cast<size_t>(im)]))(2);
      if (depth <= 0.0) {
        store->mark_observation_deleted(obs_id);
        dirty_tracks.insert(tid);
        ++marked;
      } else if (depth > max_depth) {
        store->mark_observation_deleted_restorable(obs_id);
        dirty_tracks.insert(tid);
        ++marked;
      }
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
/// across registered images.  Image-indexed traversal (skips unregistered images entirely).
/// Thread-parallel via OpenMP.
static std::vector<double> collect_reproj_errors(const TrackStore& store,
                                                 const std::vector<Eigen::Matrix3d>& poses_R,
                                                 const std::vector<Eigen::Vector3d>& poses_C,
                                                 const std::vector<bool>& registered,
                                                 const std::vector<camera::Intrinsics>& cameras,
                                                 const std::vector<int>& image_to_camera_index) {
  std::vector<double> errors;
  const int n_images = store.num_images();
  if (static_cast<int>(poses_R.size()) != n_images ||
      static_cast<int>(poses_C.size()) != n_images ||
      static_cast<int>(registered.size()) != n_images ||
      static_cast<int>(image_to_camera_index.size()) != n_images)
    return errors;

  const int n_threads = omp_get_max_threads();
  std::vector<std::vector<double>> per_thread(static_cast<size_t>(n_threads));

#pragma omp parallel for schedule(dynamic, 4)
  for (int im = 0; im < n_images; ++im) {
    if (!registered[static_cast<size_t>(im)])
      continue;
    const Eigen::Matrix3d& R = poses_R[static_cast<size_t>(im)];
    const Eigen::Vector3d& C = poses_C[static_cast<size_t>(im)];
    const camera::Intrinsics& K =
        cameras[static_cast<size_t>(image_to_camera_index[static_cast<size_t>(im)])];
    // Reuse per-thread vector to avoid repeated heap allocation across OMP iterations.
    thread_local std::vector<int> obs_ids;
    obs_ids.clear();
    store.get_image_observation_indices(im, &obs_ids);
    auto& local = per_thread[static_cast<size_t>(omp_get_thread_num())];
    for (int obs_id : obs_ids) {
      const int tid = store.obs_track_id(obs_id);
      if (!store.is_track_valid(tid) || !store.track_has_triangulated_xyz(tid))
        continue;
      float tx, ty, tz;
      store.get_track_xyz(tid, &tx, &ty, &tz);
      const Eigen::Vector3d X(static_cast<double>(tx), static_cast<double>(ty),
                              static_cast<double>(tz));
      const Eigen::Vector3d p = R * (X - C);
      if (p(2) <= 1e-12)
        continue;
      const double xn = p(0) / p(2), yn = p(1) / p(2);
      double xd, yd;
      camera::apply_distortion(xn, yn, K, &xd, &yd);
      Observation obs;
      store.get_obs(obs_id, &obs);
      const double du = static_cast<double>(obs.u) - (K.fx * xd + K.cx);
      const double dv = static_cast<double>(obs.v) - (K.fy * yd + K.cy);
      local.push_back(std::sqrt(du * du + dv * dv));
    }
  }

  size_t total = 0;
  for (const auto& v : per_thread)
    total += v.size();
  errors.reserve(total);
  for (auto& v : per_thread)
    errors.insert(errors.end(), v.begin(), v.end());
  return errors;
}

struct BAObsCandidate {
  Observation obs;
  int ba_image_index = -1;
  int global_image_index = -1;
  int camera_index = -1;
  double reproj_error_px = 1e9;
  Eigen::Vector3d ray = Eigen::Vector3d::Zero();
};

static std::vector<BAObsCandidate> select_track_observations_for_ba(
    const std::vector<Observation>& obs_buf, const std::vector<int>& global_indices,
    const std::vector<Eigen::Matrix3d>& poses_R, const std::vector<Eigen::Vector3d>& poses_C,
    const std::vector<camera::Intrinsics>& cameras, const std::vector<int>& image_to_camera_index,
    const Eigen::Vector3d& X, int max_observations_per_track,
    const std::vector<uint8_t>* required_image_mask = nullptr, int* n_considered_out = nullptr,
    int* n_required_considered_out = nullptr, int* n_required_selected_out = nullptr) {
  if (n_considered_out)
    *n_considered_out = 0;
  if (n_required_considered_out)
    *n_required_considered_out = 0;
  if (n_required_selected_out)
    *n_required_selected_out = 0;

  auto is_required_image = [&](int g) -> bool {
    return required_image_mask && g >= 0 && static_cast<size_t>(g) < required_image_mask->size() &&
           (*required_image_mask)[static_cast<size_t>(g)] != 0;
  };

  std::vector<BAObsCandidate> cands;
  cands.reserve(obs_buf.size());
  for (const auto& o : obs_buf) {
    const int g = static_cast<int>(o.image_index);
    auto it = std::lower_bound(global_indices.begin(), global_indices.end(), g);
    if (it == global_indices.end() || *it != g)
      continue;
    BAObsCandidate c;
    c.obs = o;
    c.ba_image_index = static_cast<int>(it - global_indices.begin());
    c.global_image_index = g;
    c.camera_index = image_to_camera_index[static_cast<size_t>(g)];
    if (c.camera_index >= 0 && c.camera_index < static_cast<int>(cameras.size())) {
      const Eigen::Vector3d p =
          poses_R[static_cast<size_t>(g)] * (X - poses_C[static_cast<size_t>(g)]);
      if (p(2) > 1e-12) {
        const camera::Intrinsics& K = cameras[static_cast<size_t>(c.camera_index)];
        const double xn = p(0) / p(2), yn = p(1) / p(2);
        double xd, yd;
        camera::apply_distortion(xn, yn, K, &xd, &yd);
        const double du = static_cast<double>(o.u) - (K.fx * xd + K.cx);
        const double dv = static_cast<double>(o.v) - (K.fy * yd + K.cy);
        c.reproj_error_px = std::sqrt(du * du + dv * dv);
      }
    }
    c.ray = (X - poses_C[static_cast<size_t>(g)]).normalized();
    cands.push_back(c);
  }
  if (n_considered_out)
    *n_considered_out = static_cast<int>(cands.size());
  if (n_required_considered_out) {
    int n = 0;
    for (const auto& c : cands)
      if (is_required_image(c.global_image_index))
        ++n;
    *n_required_considered_out = n;
  }
  if (max_observations_per_track <= 0 ||
      static_cast<int>(cands.size()) <= max_observations_per_track) {
    if (n_required_selected_out) {
      int n = 0;
      for (const auto& c : cands)
        if (is_required_image(c.global_image_index))
          ++n;
      *n_required_selected_out = n;
    }
    return cands;
  }

  const int cap = std::max(2, max_observations_per_track);
  const int keep_min = std::min({3, cap, static_cast<int>(cands.size())});
  std::vector<int> order(cands.size());
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
    return cands[a].reproj_error_px < cands[b].reproj_error_px;
  });

  std::vector<int> selected;
  selected.reserve(cap);
  std::vector<int> selected_cam;
  selected_cam.reserve(cap);
  std::vector<uint8_t> selected_mark(cands.size(), 0);
  auto has_selected_cam = [&](int cam_idx) -> bool {
    return std::find(selected_cam.begin(), selected_cam.end(), cam_idx) != selected_cam.end();
  };

  // Phase 0 (hard constraint): in local BA, preserve at least one observation from variable
  // cameras if this track has any, so variable camera blocks keep direct constraints.
  if (required_image_mask && !required_image_mask->empty()) {
    int best_req = -1;
    double best_req_err = 1e18;
    for (size_t i = 0; i < cands.size(); ++i) {
      if (!is_required_image(cands[i].global_image_index))
        continue;
      if (cands[i].reproj_error_px < best_req_err) {
        best_req_err = cands[i].reproj_error_px;
        best_req = static_cast<int>(i);
      }
    }
    if (best_req >= 0) {
      selected.push_back(best_req);
      selected_mark[static_cast<size_t>(best_req)] = 1;
      if (!has_selected_cam(cands[static_cast<size_t>(best_req)].camera_index))
        selected_cam.push_back(cands[static_cast<size_t>(best_req)].camera_index);
    }
  }

  // Phase 0b: if required set has >=2 observations on this track, keep a second required
  // observation when possible to avoid collapsing to a single-view variable constraint.
  if (required_image_mask && !required_image_mask->empty()) {
    int required_count = 0;
    for (const auto& c : cands)
      if (is_required_image(c.global_image_index))
        ++required_count;
    if (required_count >= 2 && static_cast<int>(selected.size()) < keep_min) {
      int best_req2 = -1;
      double best_req2_score = -1e18;
      for (size_t i = 0; i < cands.size(); ++i) {
        if (selected_mark[i])
          continue;
        if (!is_required_image(cands[i].global_image_index))
          continue;
        const BAObsCandidate& c = cands[i];
        double angle_gap = 0.0;
        for (int sidx : selected) {
          const BAObsCandidate& s = cands[static_cast<size_t>(sidx)];
          const double cosv = std::max(-1.0, std::min(1.0, c.ray.dot(s.ray)));
          constexpr double kRad2Deg = 57.2957795130823208768;
          const double ang = std::acos(cosv) * kRad2Deg;
          angle_gap = std::max(angle_gap, std::min(1.0, ang / 20.0));
        }
        const double score = -std::min(1.0, c.reproj_error_px / 8.0) + 0.6 * angle_gap;
        if (score > best_req2_score) {
          best_req2_score = score;
          best_req2 = static_cast<int>(i);
        }
      }
      if (best_req2 >= 0) {
        selected.push_back(best_req2);
        selected_mark[static_cast<size_t>(best_req2)] = 1;
        if (!has_selected_cam(cands[static_cast<size_t>(best_req2)].camera_index))
          selected_cam.push_back(cands[static_cast<size_t>(best_req2)].camera_index);
      }
    }
  }
  for (int idx : order) {
    if (static_cast<int>(selected.size()) >= keep_min)
      break;
    if (selected_mark[static_cast<size_t>(idx)])
      continue;
    if (!has_selected_cam(cands[idx].camera_index)) {
      selected.push_back(idx);
      selected_mark[static_cast<size_t>(idx)] = 1;
      selected_cam.push_back(cands[idx].camera_index);
    }
  }
  for (int idx : order) {
    if (static_cast<int>(selected.size()) >= keep_min)
      break;
    if (!selected_mark[static_cast<size_t>(idx)]) {
      selected.push_back(idx);
      selected_mark[static_cast<size_t>(idx)] = 1;
      if (!has_selected_cam(cands[idx].camera_index))
        selected_cam.push_back(cands[idx].camera_index);
    }
  }

  while (static_cast<int>(selected.size()) < cap) {
    int best_idx = -1;
    double best_score = -1e18;
    for (size_t i = 0; i < cands.size(); ++i) {
      if (selected_mark[i])
        continue;
      const BAObsCandidate& c = cands[i];
      const double err_term = -std::min(1.0, c.reproj_error_px / 8.0);
      const double cam_term = has_selected_cam(c.camera_index) ? 0.0 : 0.8;
      double time_gap = 1.0;
      double angle_gap = 1.0;
      if (!selected.empty()) {
        time_gap = 0.0;
        angle_gap = 0.0;
        for (int sidx : selected) {
          const BAObsCandidate& s = cands[sidx];
          time_gap =
              std::max(time_gap,
                       std::min(1.0, std::abs(c.global_image_index - s.global_image_index) / 50.0));
          const double cosv = std::max(-1.0, std::min(1.0, c.ray.dot(s.ray)));
          constexpr double kRad2Deg = 57.2957795130823208768;
          const double ang = std::acos(cosv) * kRad2Deg;
          angle_gap = std::max(angle_gap, std::min(1.0, ang / 20.0));
        }
      }
      const double score = 1.0 + err_term + cam_term + 0.35 * time_gap + 0.45 * angle_gap;
      if (score > best_score) {
        best_score = score;
        best_idx = static_cast<int>(i);
      }
    }
    if (best_idx < 0)
      break;
    selected.push_back(best_idx);
    selected_mark[static_cast<size_t>(best_idx)] = 1;
    if (!has_selected_cam(cands[static_cast<size_t>(best_idx)].camera_index))
      selected_cam.push_back(cands[static_cast<size_t>(best_idx)].camera_index);
  }

  std::vector<BAObsCandidate> out;
  out.reserve(selected.size());
  for (int idx : selected)
    out.push_back(cands[static_cast<size_t>(idx)]);
  std::sort(out.begin(), out.end(), [](const BAObsCandidate& a, const BAObsCandidate& b) {
    return a.global_image_index < b.global_image_index;
  });
  if (n_required_selected_out) {
    int n = 0;
    for (const auto& c : out)
      if (is_required_image(c.global_image_index))
        ++n;
    *n_required_selected_out = n;
  }
  return out;
}

/// Same projection as `collect_reproj_errors`, plus cheirality skips — for step-wise pipeline
/// debug.
struct PipelineReprojStats {
  int n_obs = 0;                ///< valid observations used in sums
  int n_skipped_behind = 0;     ///< triangulated obs skipped (depth ≤ 0)
  double sum_euclidean = 0.0;   ///< sum of per-obs Euclidean px
  double sum_sq_residual = 0.0; ///< sum of (du²+dv²) = sum of e²
  double max_euclidean_px = 0.0;
};

static PipelineReprojStats compute_pipeline_reproj_stats(
    const TrackStore& store, const std::vector<Eigen::Matrix3d>& poses_R,
    const std::vector<Eigen::Vector3d>& poses_C, const std::vector<bool>& registered,
    const std::vector<camera::Intrinsics>& cameras, const std::vector<int>& image_to_camera_index) {
  PipelineReprojStats s;
  const int n_images = store.num_images();
  if (static_cast<int>(poses_R.size()) != n_images ||
      static_cast<int>(poses_C.size()) != n_images ||
      static_cast<int>(registered.size()) != n_images ||
      static_cast<int>(image_to_camera_index.size()) != n_images)
    return s;
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
    if (p(2) <= 1e-12) {
      ++s.n_skipped_behind;
      continue;
    }
    const int cam_ix = image_to_camera_index[static_cast<size_t>(im)];
    if (cam_ix < 0 || cam_ix >= static_cast<int>(cameras.size()))
      continue;
    const camera::Intrinsics& K = cameras[static_cast<size_t>(cam_ix)];
    const double xn = p(0) / p(2), yn = p(1) / p(2);
    double xd, yd;
    camera::apply_distortion(xn, yn, K, &xd, &yd);
    const double u_pred = K.fx * xd + K.cx;
    const double v_pred = K.fy * yd + K.cy;
    const double du = static_cast<double>(obs.u) - u_pred;
    const double dv = static_cast<double>(obs.v) - v_pred;
    const double e = std::sqrt(du * du + dv * dv);
    ++s.n_obs;
    s.sum_euclidean += e;
    s.sum_sq_residual += du * du + dv * dv;
    if (e > s.max_euclidean_px)
      s.max_euclidean_px = e;
  }
  return s;
}

// #region agent log
static void agent_log_pipeline_reproj_step(const char* step, int sfm_iter, int num_registered,
                                           const PipelineReprojStats& st) {
  const double mean_e = (st.n_obs > 0) ? (st.sum_euclidean / static_cast<double>(st.n_obs)) : 0.0;
  // Match Ceres bundle_adjustment_analytic: sqrt(2*cost/n_residuals) with 2 residuals per obs.
  const double rmse_ceres_equiv =
      (st.n_obs > 0) ? std::sqrt(st.sum_sq_residual / (2.0 * static_cast<double>(st.n_obs))) : 0.0;
  LOG(INFO) << "[reproj_diag] step=" << step << "  sfm_iter=" << sfm_iter
            << "  registered=" << num_registered << "  n_obs=" << st.n_obs
            << "  n_behind=" << st.n_skipped_behind << "  mean_e_px=" << mean_e
            << "  rmse_ceres_equiv_px=" << rmse_ceres_equiv << "  max_e_px=" << st.max_euclidean_px;
  std::ofstream f("/home/jones/Git/01jones/InsightAT/InsightAT/.cursor/debug-2ba246.log",
                  std::ios::app);
  if (!f)
    return;
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::system_clock::now().time_since_epoch())
                      .count();
  f << "{\"sessionId\":\"2ba246\",\"hypothesisId\":\"H_pipeline_step\",\"location\":"
       "\"incremental_sfm_pipeline.cpp:reproj_diag\",\"message\":\"pipeline_reproj_step\","
       "\"data\":{\"step\":\""
    << step << "\",\"sfm_iter\":" << sfm_iter << ",\"num_registered\":" << num_registered
    << ",\"n_obs\":" << st.n_obs << ",\"n_skipped_behind\":" << st.n_skipped_behind
    << ",\"mean_euclidean_px\":" << mean_e << ",\"rmse_ceres_equiv_px\":" << rmse_ceres_equiv
    << ",\"max_euclidean_px\":" << st.max_euclidean_px << "},\"timestamp\":" << ms << "}\n";
}
// #endregion

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
  std::vector<int> track_ids_conn; // hoisted — reused across images
  for (int im : reg_list) {
    if (optimize_set.count(im))
      continue;
    track_ids_conn.clear();
    store.get_image_track_observations(im, &track_ids_conn, nullptr);
    int count = 0;
    for (int tid : track_ids_conn) {
      if (!store.track_has_triangulated_xyz(tid))
        continue;
      const auto& track_obs_ids = store.track_all_obs_ids_view(tid);
      for (int obs_id : track_obs_ids) {
        if (!store.is_obs_valid(obs_id))
          continue;
        const int oim = static_cast<int>(store.obs_image_index(obs_id));
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

int count_two_view_valid_tracks(const TrackStore& store, int im0, int im1) {
  const uint32_t uim0 = static_cast<uint32_t>(im0), uim1 = static_cast<uint32_t>(im1);
  int n = 0;
  for (size_t ti = 0; ti < store.num_tracks(); ++ti) {
    const int tid = static_cast<int>(ti);
    if (!store.is_track_valid(tid))
      continue;
    bool has0 = false, has1 = false;
    const auto& track_obs_ids = store.track_all_obs_ids_view(tid);
    for (int obs_id : track_obs_ids) {
      if (!store.is_obs_valid(obs_id))
        continue;
      const uint32_t image_index = store.obs_image_index(obs_id);
      if (image_index == uim0)
        has0 = true;
      if (image_index == uim1)
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
      int n_alive_obs = 0;
      const auto& track_obs_ids = store.track_all_obs_ids_view(tid);
      for (int obs_id : track_obs_ids) {
        if (store.is_obs_valid(obs_id))
          ++n_alive_obs;
      }
      corr_count += std::max(0, n_alive_obs - 1);
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

std::vector<double> build_init_median_angle_schedule(double min_median_angle_deg) {
  if (min_median_angle_deg <= 0.0)
    return {0.0};

  std::vector<double> schedule;
  schedule.push_back(min_median_angle_deg);

  // Strict-first, then progressively relax for low-parallax datasets (e.g. ETH3D).
  // Keep 10° as the lower bound to avoid collapsing to very weak baselines.
  if (min_median_angle_deg > 20.0)
    schedule.push_back(std::max(20.0, min_median_angle_deg * 0.67));
  if (min_median_angle_deg > 15.0)
    schedule.push_back(15.0);
  if (min_median_angle_deg > 10.0)
    schedule.push_back(10.0);

  // Deduplicate near-identical entries caused by rounding or small input values.
  std::vector<double> uniq;
  uniq.reserve(schedule.size());
  for (double v : schedule) {
    bool exists = false;
    for (double u : uniq) {
      if (std::abs(u - v) < 1e-6) {
        exists = true;
        break;
      }
    }
    if (!exists)
      uniq.push_back(v);
  }
  return uniq;
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

/// Try an initial pair WITHOUT mutating the store. Estimates R/t from shared track
/// correspondences via 5-pt RANSAC (poselib) with prior K, triangulates inliers into
/// a local buffer, runs BA locally, checks quality. Returns trial result.
InitPairTrialResult try_initial_pair_candidate(const TrackStore& store, int im0, int im1,
                                               const std::vector<camera::Intrinsics>& cameras,
                                               const std::vector<int>& image_to_camera_index,
                                               int min_tracks_for_intital_pair,
                                               int min_num_inliers,
                                               double max_forward_motion,
                                               double ba_rmse_max,
                                               double outlier_thresh_px,
                                               double min_angle_deg,
                                               double max_angle_deg,
                                               double min_median_angle_deg) {

  const double kPi = 3.141592653589793;
  const double min_angle_rad_q = min_angle_deg * (kPi / 180.0);
  const double max_angle_rad_q = max_angle_deg * (kPi / 180.0);

  InitPairTrialResult result;

  const camera::Intrinsics& K0 = cameras[static_cast<size_t>(image_to_camera_index[im0])];
  const camera::Intrinsics& K1 = cameras[static_cast<size_t>(image_to_camera_index[im1])];
  const uint32_t uim0 = static_cast<uint32_t>(im0);
  const uint32_t uim1 = static_cast<uint32_t>(im1);

  // 1. Collect ALL shared track pixel correspondences (raw pixel coords).
  struct TrackCorr {
    int tid;
    double u0, v0, u1, v1;
  };
  std::vector<TrackCorr> corrs;
  std::vector<Observation> obs_buf;
  for (size_t ti = 0; ti < store.num_tracks(); ++ti) {
    const int tid = static_cast<int>(ti);
    if (!store.is_track_valid(tid))
      continue;
    double u0 = 0, v0 = 0, u1 = 0, v1 = 0;
    bool has0 = false, has1 = false;
    const auto& track_obs_ids = store.track_all_obs_ids_view(tid);
    for (int obs_id : track_obs_ids) {
      if (!store.is_obs_valid(obs_id))
        continue;
      const uint32_t image_index = store.obs_image_index(obs_id);
      if (image_index == uim0) {
        u0 = static_cast<double>(store.obs_u(obs_id));
        v0 = static_cast<double>(store.obs_v(obs_id));
        has0 = true;
      }
      if (image_index == uim1) {
        u1 = static_cast<double>(store.obs_u(obs_id));
        v1 = static_cast<double>(store.obs_v(obs_id));
        has1 = true;
      }
    }
    if (!has0 || !has1)
      continue;
    corrs.push_back({tid, u0, v0, u1, v1});
  }

  if (static_cast<int>(corrs.size()) < min_tracks_for_intital_pair) {
    DLOG(INFO) << "    pair (" << im0 << "," << im1 << "): only " << corrs.size()
               << " shared tracks (need " << min_tracks_for_intital_pair << ")";
    return result;
  }

  // 2. Estimate relative pose via 5-pt RANSAC with prior K (no geo file needed).
  Eigen::Matrix3d R;
  Eigen::Vector3d C1;
  std::vector<int> track_ids;
  std::vector<Eigen::Vector3d> points3d;
  {
    std::vector<poselib::Point2D> x0_pts, x1_pts;
    x0_pts.reserve(corrs.size());
    x1_pts.reserve(corrs.size());
    for (const auto& c : corrs) {
      x0_pts.push_back({c.u0, c.v0});
      x1_pts.push_back({c.u1, c.v1});
    }

    poselib::Camera cam0_pl(poselib::CameraModelId::PINHOLE, {K0.fx, K0.fy, K0.cx, K0.cy});
    poselib::Camera cam1_pl(poselib::CameraModelId::PINHOLE, {K1.fx, K1.fy, K1.cx, K1.cy});
    poselib::RelativePoseOptions opt;
    opt.max_error = outlier_thresh_px;
    opt.ransac.max_iterations = 2000;
    opt.ransac.min_iterations = 100;
    opt.bundle.max_iterations = 250;
    opt.bundle.loss_type = poselib::BundleOptions::CAUCHY;
    opt.bundle.loss_scale = outlier_thresh_px;

    poselib::CameraPose pose;
    std::vector<char> pl_inliers;
    const poselib::RansacStats stats =
        poselib::estimate_relative_pose(x0_pts, x1_pts, cam0_pl, cam1_pl, opt, &pose, &pl_inliers);

    LOG(INFO) << "    pair (" << im0 << "," << im1 << "): E-RANSAC inliers=" << stats.num_inliers
              << "/" << corrs.size();

    if (static_cast<int>(stats.num_inliers) < min_num_inliers) {
      LOG(INFO) << "    pair (" << im0 << "," << im1
                << "): REJECTED (E-RANSAC inliers too few, need " << min_num_inliers
                << ")";
      return result;
    }

    // COLMAP-style forward-motion gate.
    // t is up-to-scale; forwardness ratio uses direction only.
    const Eigen::Vector3d t01 = pose.t;
    const double t_norm = t01.norm();
    if (t_norm <= 1e-12) {
      LOG(INFO) << "    pair (" << im0 << "," << im1 << "): REJECTED (invalid relative t)";
      return result;
    }
    const double forward_motion = std::abs(t01.z()) / t_norm;
    if (max_forward_motion > 0.0 && forward_motion >= max_forward_motion) {
      LOG(INFO) << "    pair (" << im0 << "," << im1 << "): REJECTED (forward motion "
                << forward_motion << " >= " << max_forward_motion << ")";
      return result;
    }

    // Apply inlier mask: keep only RANSAC inliers for triangulation.
    std::vector<TrackCorr> inlier_corrs;
    inlier_corrs.reserve(static_cast<size_t>(stats.num_inliers));
    for (size_t ci = 0; ci < corrs.size(); ++ci) {
      if (pl_inliers[ci])
        inlier_corrs.push_back(corrs[ci]);
    }
    corrs = std::move(inlier_corrs);

    R = pose.R();
    C1 = -R.transpose() * pose.t;
  }

  // 3. Triangulate RANSAC-inlier tracks into LOCAL buffer (no store mutation).
  for (const auto& c : corrs) {
    double u0 = c.u0, v0 = c.v0, u1 = c.u1, v1 = c.v1;
    if (K0.has_distortion())
      camera::undistort_point(K0, u0, v0, &u0, &v0);
    if (K1.has_distortion())
      camera::undistort_point(K1, u1, v1, &u1, &v1);
    Eigen::Vector2d n0((u0 - K0.cx) / K0.fx, (v0 - K0.cy) / K0.fy);
    Eigen::Vector2d n1((u1 - K1.cx) / K1.fx, (v1 - K1.cy) / K1.fy);
    const Eigen::Vector3d t_cam = -R * C1;
    const Eigen::Vector3d X = triangulate_point(n0, n1, R, t_cam);

    // ── Pre-BA geometric filters ─────────────────────────────────────────
    // Cheirality — point must be in front of BOTH cameras.
    if (X(2) <= 1e-9)
      continue;
    Eigen::Vector3d X_in_cam1_pre = R * (X - C1);
    if (X_in_cam1_pre(2) <= 1e-9)
      continue;

    // Triangulation angle must be in [min_angle_deg, max_angle_deg].
    {
      Eigen::Vector3d r0_dir = X.normalized();
      Eigen::Vector3d r1_dir = (X - C1).normalized();
      double cos_a = std::max(-1.0, std::min(1.0, r0_dir.dot(r1_dir)));
      double ang = std::acos(cos_a);
      if (ang > max_angle_rad_q || ang < min_angle_rad_q)
        continue;
    }
    // ────────────────────────────────────────────────────────────────────

    track_ids.push_back(c.tid);
    points3d.push_back(X);
  }
  result.n_triangulated = static_cast<int>(track_ids.size());

  if (track_ids.size() < 8u) {
    LOG(INFO) << "    pair (" << im0 << "," << im1 << "): only " << track_ids.size()
              << " triangulated tracks (need >=8)";
    return result;
  }

  // 4. Two-view BA with local data (no store mutation)
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
    const auto& track_obs_ids = store.track_all_obs_ids_view(track_ids[i]);
    obs_buf.reserve(track_obs_ids.size());
    for (int obs_id : track_obs_ids) {
      if (!store.is_obs_valid(obs_id))
        continue;
      Observation o;
      o.image_index = store.obs_image_index(obs_id);
      o.feature_id = store.obs_feature_id(obs_id);
      o.u = store.obs_u(obs_id);
      o.v = store.obs_v(obs_id);
      o.scale = store.obs_scale(obs_id);
      obs_buf.push_back(o);
    }
    const int pt_idx = static_cast<int>(i);
    for (const auto& o : obs_buf) {
      const double sigma_feat = (o.scale > 1e-6f) ? static_cast<double>(o.scale) : 1.0;
      const double std_sigma_obs_px = sigma_to_ba_stddev(sigma_feat);
      if (o.image_index == uim0) {
        BAObservation bo;
        bo.image_index = 0;
        bo.point_index = pt_idx;
        bo.u = static_cast<double>(o.u);
        bo.v = static_cast<double>(o.v);
        bo.std_sigma_obs_px = std_sigma_obs_px;
        ba_in.observations.push_back(bo);
      } else if (o.image_index == uim1) {
        BAObservation bo;
        bo.image_index = 1;
        bo.point_index = pt_idx;
        bo.u = static_cast<double>(o.u);
        bo.v = static_cast<double>(o.v);
        bo.std_sigma_obs_px = std_sigma_obs_px;
        ba_in.observations.push_back(bo);
      }
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

  // 5. Adaptive inlier selection via MAD-based reprojection threshold.
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
    const auto& track_obs_ids_q = store.track_all_obs_ids_view(track_ids[i]);
    obs_buf.reserve(track_obs_ids_q.size());
    for (int obs_id : track_obs_ids_q) {
      if (!store.is_obs_valid(obs_id))
        continue;
      Observation o;
      o.image_index = store.obs_image_index(obs_id);
      o.feature_id = store.obs_feature_id(obs_id);
      o.u = store.obs_u(obs_id);
      o.v = store.obs_v(obs_id);
      o.scale = store.obs_scale(obs_id);
      obs_buf.push_back(o);
    }
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

  // Step 6: Median triangulation angle gate.
  // Rejects near-degenerate forward-motion pairs (e.g. two consecutive nadir frames) whose
  // per-point angles each pass min_angle_deg but whose MEDIAN angle is still very small,
  // indicating poor baseline-to-depth ratio and unstable scale.
  if (min_median_angle_deg > 0.0 && !accepted_xyz.empty()) {
    const double kPiOver180 = 3.141592653589793 / 180.0;
    std::vector<double> tri_angles;
    tri_angles.reserve(accepted_xyz.size());
    for (const auto& kv : accepted_xyz) {
      const Eigen::Vector3d& X = kv.second;
      Eigen::Vector3d r0 = X.normalized();
      Eigen::Vector3d r1 = (X - C1).normalized();
      double cos_a = std::max(-1.0, std::min(1.0, r0.dot(r1)));
      tri_angles.push_back(std::acos(cos_a) / kPiOver180);
    }
    const size_t mid = tri_angles.size() / 2;
    std::nth_element(tri_angles.begin(), tri_angles.begin() + static_cast<ptrdiff_t>(mid),
                     tri_angles.end());
    const double median_tri_deg = tri_angles[mid];
    LOG(INFO) << "    pair (" << im0 << "," << im1 << "): median tri angle = " << median_tri_deg
              << "° (min=" << min_median_angle_deg << "°)";
    if (median_tri_deg < min_median_angle_deg) {
      LOG(INFO) << "    pair (" << im0 << "," << im1 << "): REJECTED (median tri angle "
                << median_tri_deg << "° < " << min_median_angle_deg << "°)";
      return result;
    }
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
bool run_initial_pair_loop(
    const ViewGraph& view_graph, TrackStore* store, const std::vector<camera::Intrinsics>& cameras,
  const std::vector<int>& image_to_camera_index, int min_tracks_for_intital_pair,
  int min_num_inliers, double max_forward_motion, double min_angle_deg,
  double min_median_angle_deg, uint32_t* initial_im0_out, uint32_t* initial_im1_out,
    std::vector<Eigen::Matrix3d>* poses_R_out, std::vector<Eigen::Vector3d>* poses_C_out,
    std::vector<bool>* registered_out, int max_first_images, int max_second_images) {
  if (!store || !poses_R_out || !poses_C_out || !registered_out || cameras.empty())
    return false;
  const int n_images = store->num_images();

  LOG(INFO) << "================================================================";
  LOG(INFO)
      << "Initial pair selection: first image by track correspondences, second by ViewGraph score";
  LOG(INFO) << "  n_images=" << n_images << ", n_tracks=" << store->num_tracks()
            << ", n_obs=" << store->num_observations();
  LOG(INFO) << "  min_tracks_for_intital_pair=" << min_tracks_for_intital_pair
            << ", min_num_inliers=" << min_num_inliers
            << ", max_forward_motion=" << max_forward_motion
            << ", min_angle_deg=" << min_angle_deg
            << ", min_median_angle_deg=" << min_median_angle_deg;
  LOG(INFO) << "  ViewGraph pairs (informational): " << view_graph.num_pairs();

  const std::vector<double> median_angle_schedule =
      build_init_median_angle_schedule(min_median_angle_deg);
  if (median_angle_schedule.size() > 1) {
    std::ostringstream oss;
    oss << "  init median-angle fallback schedule:";
    for (double v : median_angle_schedule)
      oss << " " << v << "°";
    LOG(INFO) << oss.str();
  }

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

  const size_t max_first = std::min(first_images.size(), static_cast<size_t>(max_first_images));
  size_t total_tried_pairs = 0;

  for (size_t pass = 0; pass < median_angle_schedule.size(); ++pass) {
    const double pass_min_median_angle_deg = median_angle_schedule[pass];
    if (pass > 0) {
      LOG(INFO) << "  [fallback pass " << pass << "] Relax min_median_angle_deg to "
                << pass_min_median_angle_deg << "°";
    }
    std::set<std::pair<int, int>> tried_pairs;

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
        LOG(INFO) << "    second #" << si << ": image " << sc.image_index
                  << " score_prelim=" << sc.score_prelim << " F_inliers=" << sc.F_inliers
                  << " E_ok=" << sc.E_ok << " twoview_ok=" << sc.twoview_ok
                  << " stable=" << sc.stable << " n_pts=" << sc.num_valid_points;
      }
      if (second_candidates.size() > show_n)
        LOG(INFO) << "    ... (" << second_candidates.size() - show_n << " more)";
    }

      const size_t max_second =
          std::min(second_candidates.size(), static_cast<size_t>(max_second_images));
      for (size_t si = 0; si < max_second; ++si) {
        const int im2 = static_cast<int>(second_candidates[si].image_index);
        auto pair_key = std::make_pair(std::min(im1, im2), std::max(im1, im2));
        if (tried_pairs.count(pair_key))
          continue;
        tried_pairs.insert(pair_key);
        ++total_tried_pairs;

      // Try this pair (NO store mutation)
      const float ba_rmse_max = 10.0;
      double outlier_thresh_px = 4.0;
        double max_angle_deg = 120.0;
        auto trial = try_initial_pair_candidate(
            *store, im1, im2, cameras, image_to_camera_index,
            min_tracks_for_intital_pair, min_num_inliers, max_forward_motion,
            ba_rmse_max, outlier_thresh_px, min_angle_deg, max_angle_deg,
          pass_min_median_angle_deg);
        if (!trial.success)
          continue;

      // ── Pair accepted! Now commit to store ──
        LOG(INFO) << "  >> ACCEPTED pair (" << im1 << ", " << im2
            << "): " << trial.n_inlier_tracks << " inlier tracks, RMSE=" << trial.rmse_px
            << " px, reproj_thr=" << trial.reproject_px
            << " px, min_median_angle_deg=" << pass_min_median_angle_deg;

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
  }

  LOG(ERROR) << "  No initial pair found after trying " << total_tried_pairs << " pairs";
  LOG(INFO) << "================================================================";
  return false;
}

// ─── BA: build from Store, write back ────────────────────────────────────────
namespace {

/// For each registered image, compute whether it has enough high-quality track support
/// to allow 2-degree tracks to be skipped in global BA.
/// S(track) = sin²(parallax_angle) computed from poses_C and track XYZ.
/// Only 2-view tracks (exactly 2 registered observers) contribute variable S;
/// higher-degree tracks always contribute S=1.0 (implicitly stable) to both their images.
static std::vector<bool> compute_image_stability(const TrackStore& store,
                                                 const std::vector<Eigen::Vector3d>& poses_C,
                                                 const std::vector<bool>& registered,
                                                 double min_angle_score, int min_stable_obs,
                                                 double stable_ratio) {
  const int n = store.num_images();
  std::vector<int> obs_count(static_cast<size_t>(n), 0);
  std::vector<int> stable_count(static_cast<size_t>(n), 0);
  for (size_t ti = 0; ti < store.num_tracks(); ++ti) {
    const int tid = static_cast<int>(ti);
    if (!store.is_track_valid(tid) || !store.track_has_triangulated_xyz(tid))
      continue;
    const auto& track_obs_ids = store.track_all_obs_ids_view(tid);
    // Collect registered observer image indices.
    int reg_imgs[2];
    int n_reg = 0;
    for (int obs_id : track_obs_ids) {
      if (!store.is_obs_valid(obs_id))
        continue;
      const int im = static_cast<int>(store.obs_image_index(obs_id));
      if (im >= 0 && im < n && registered[static_cast<size_t>(im)]) {
        if (n_reg < 2)
          reg_imgs[n_reg] = im;
        ++n_reg;
      }
    }
    if (n_reg == 0)
      continue;
    // Compute S = sin²(θ) only for 2-degree tracks; higher-degree tracks are always stable.
    double S = 1.0;
    if (n_reg == 2) {
      float tx, ty, tz;
      store.get_track_xyz(tid, &tx, &ty, &tz);
      const Eigen::Vector3d X(static_cast<double>(tx), static_cast<double>(ty),
                              static_cast<double>(tz));
      const Eigen::Vector3d r0 = (X - poses_C[static_cast<size_t>(reg_imgs[0])]).normalized();
      const Eigen::Vector3d r1 = (X - poses_C[static_cast<size_t>(reg_imgs[1])]).normalized();
      const double cos_a = std::max(-1.0, std::min(1.0, r0.dot(r1)));
      const double sin_a = std::sqrt(1.0 - cos_a * cos_a);
      S = sin_a * sin_a;
    }
    // Accumulate per-image stats (cap n_reg at actual array size for higher-degree tracks).
    int alive_obs_count = 0;
    for (int obs_id : track_obs_ids)
      if (store.is_obs_valid(obs_id))
        ++alive_obs_count;
    const int cap = std::min(n_reg, alive_obs_count);
    int counted = 0;
    for (int obs_id : track_obs_ids) {
      if (!store.is_obs_valid(obs_id))
        continue;
      if (counted >= cap)
        break;
      const int im = static_cast<int>(store.obs_image_index(obs_id));
      if (im >= 0 && im < n && registered[static_cast<size_t>(im)]) {
        obs_count[static_cast<size_t>(im)]++;
        if (S >= min_angle_score)
          stable_count[static_cast<size_t>(im)]++;
        ++counted;
      }
    }
  }
  std::vector<bool> stable(static_cast<size_t>(n), false);
  for (int i = 0; i < n; ++i) {
    if (!registered[static_cast<size_t>(i)])
      continue;
    const int oc = obs_count[static_cast<size_t>(i)];
    stable[static_cast<size_t>(i)] =
        (oc >= min_stable_obs) &&
        (static_cast<double>(stable_count[static_cast<size_t>(i)]) / oc >= stable_ratio);
  }
  return stable;
}

// ─────────────────────────────────────────────────────────────────────────────
// Adaptive grid-NMS BA subset selection
// ─────────────────────────────────────────────────────────────────────────────

/// Per-image adaptive grid-NMS: for each registered image, split into cells of size
/// sqrt(W*H/target) and keep the highest-scoring track per cell.  A track is included in
/// BA iff it is the winner in at least one cell of at least one image.  All other valid
/// triangulated tracks get kSkipFromBA; the flag persists and is reused across rounds.
///
/// Track score = w_deg × clamp(deg/deg_cap, 0, 1)  +  w_score × S(track)
///   S=sin²(parallax_angle) for degree-2 tracks; S=1.0 for degree≥3.
static void select_ba_subset(const TrackStore& store, const std::vector<Eigen::Vector3d>& poses_C,
                             const std::vector<bool>& registered,
                             const std::vector<int>& image_to_camera_index,
                             const std::vector<camera::Intrinsics>& cameras,
                             const GlobalBAOptions& opts, TrackStore* store_mut) {
  if (!store_mut)
    return;
  const int n_images = store.num_images();
  const int n_tracks = static_cast<int>(store.num_tracks());

  // ── Pass 1: per-track registered degree + sin²(θ) for 2-degree tracks ────────────────────────
  // Old: iterate N_total_tracks, for each tri track scan all obs → O(N_tri_tracks × avg_obs).
  // New: iterate registered images, accumulate degree per track → O(N_registered_obs).
  std::vector<int> track_degree(static_cast<size_t>(n_tracks), 0);
  std::vector<float> track_S(static_cast<size_t>(n_tracks), 1.0f);
  std::vector<int> track_reg_im0(static_cast<size_t>(n_tracks), -1);
  std::vector<int> track_reg_im1(static_cast<size_t>(n_tracks), -1);
  {
    std::vector<int> img_obs_ids;
    for (int im = 0; im < n_images; ++im) {
      if (!registered[static_cast<size_t>(im)])
        continue;
      img_obs_ids.clear();
      store.get_image_observation_indices(im, &img_obs_ids);
      for (int obs_id : img_obs_ids) {
        const int tid = store.obs_track_id(obs_id);
        if (tid < 0 || tid >= n_tracks)
          continue;
        if (!store.is_track_valid(tid) || !store.track_has_triangulated_xyz(tid))
          continue;
        const int deg = track_degree[static_cast<size_t>(tid)];
        if (deg == 0)
          track_reg_im0[static_cast<size_t>(tid)] = im;
        else if (deg == 1)
          track_reg_im1[static_cast<size_t>(tid)] = im;
        ++track_degree[static_cast<size_t>(tid)];
      }
    }
  }
  // Compute sin²(θ) for degree-2 tracks (needed for grid NMS scoring).
  for (int tid = 0; tid < n_tracks; ++tid) {
    if (track_degree[static_cast<size_t>(tid)] != 2)
      continue;
    if (!store.is_track_valid(tid) || !store.track_has_triangulated_xyz(tid))
      continue;
    const int reg_im0 = track_reg_im0[static_cast<size_t>(tid)];
    const int reg_im1 = track_reg_im1[static_cast<size_t>(tid)];
    if (reg_im0 < 0 || reg_im1 < 0)
      continue;
    float tx, ty, tz;
    store.get_track_xyz(tid, &tx, &ty, &tz);
    const Eigen::Vector3d X(static_cast<double>(tx), static_cast<double>(ty),
                            static_cast<double>(tz));
    const Eigen::Vector3d r0 = (X - poses_C[static_cast<size_t>(reg_im0)]).normalized();
    const Eigen::Vector3d r1 = (X - poses_C[static_cast<size_t>(reg_im1)]).normalized();
    const double cos_a = std::max(-1.0, std::min(1.0, r0.dot(r1)));
    const double sin_a = std::sqrt(1.0 - cos_a * cos_a);
    track_S[static_cast<size_t>(tid)] = static_cast<float>(sin_a * sin_a);
  }

  // ── Pass 2: per-image grid NMS → collect winning track IDs ───────────────────────────────────
  const double w_deg = opts.ba_grid_w_degree;
  const double w_S = opts.ba_grid_w_score;
  const double deg_cap_d = static_cast<double>(opts.ba_grid_degree_cap);
  const int target = std::max(1, opts.ba_grid_target_per_image);

  std::unordered_set<int> selected;

  std::vector<int> track_ids_im;
  std::vector<Observation> obs_im;

  for (int im = 0; im < n_images; ++im) {
    if (!registered[static_cast<size_t>(im)])
      continue;

    // Get image dimensions from camera intrinsics
    int W = 0, H = 0;
    const int cam_idx =
        (im < static_cast<int>(image_to_camera_index.size())) ? image_to_camera_index[im] : -1;
    if (cam_idx >= 0 && cam_idx < static_cast<int>(cameras.size())) {
      W = cameras[static_cast<size_t>(cam_idx)].width;
      H = cameras[static_cast<size_t>(cam_idx)].height;
      if (W <= 0)
        W = static_cast<int>(cameras[static_cast<size_t>(cam_idx)].cx * 2.0);
      if (H <= 0)
        H = static_cast<int>(cameras[static_cast<size_t>(cam_idx)].cy * 2.0);
    }

    track_ids_im.clear();
    obs_im.clear();
    store.get_image_track_observations(im, &track_ids_im, &obs_im);

    if (W <= 0 || H <= 0) {
      // Unknown image size: select all tracks for this image (safe fallback).
      for (int tid : track_ids_im) {
        if (store.is_track_valid(tid) && store.track_has_triangulated_xyz(tid) &&
            track_degree[static_cast<size_t>(tid)] >= 2)
          selected.insert(tid);
      }
      continue;
    }

    // Adaptive cell size so the image contains ~target cells.
    const int gridsize =
        std::max(1, static_cast<int>(std::ceil(std::sqrt(static_cast<double>(W) * H / target))));
    const int gw = (W + gridsize - 1) / gridsize;
    const int gh = (H + gridsize - 1) / gridsize;
    const int n_cells = gw * gh;

    // Per-cell winner: (score, track_id), initialised empty.
    std::vector<std::pair<float, int>> grid(static_cast<size_t>(n_cells), {-1.0f, -1});

    for (size_t k = 0; k < track_ids_im.size(); ++k) {
      const int tid = track_ids_im[k];
      if (!store.is_track_valid(tid) || !store.track_has_triangulated_xyz(tid))
        continue;
      const int deg = track_degree[static_cast<size_t>(tid)];
      if (deg < 2)
        continue;
      const double deg_contrib = std::min(1.0, static_cast<double>(deg) / deg_cap_d);
      const float score =
          static_cast<float>(w_deg * deg_contrib + w_S * track_S[static_cast<size_t>(tid)]);
      const float u = obs_im[k].u;
      const float v = obs_im[k].v;
      const int cx_cell = std::min(static_cast<int>(u / gridsize), gw - 1);
      const int cy_cell = std::min(static_cast<int>(v / gridsize), gh - 1);
      const int cell_idx = cy_cell * gw + cx_cell;
      auto& cell = grid[static_cast<size_t>(cell_idx)];
      if (score > cell.first)
        cell = {score, tid};
    }

    for (const auto& [sc, tid] : grid) {
      if (tid >= 0)
        selected.insert(tid);
    }
  }

  // ── Pass 3: update kSkipFromBA flags atomically ───────────────────────────────────────────────
  store_mut->clear_all_skip_ba_flags();
  int n_skip = 0;
  for (int tid = 0; tid < n_tracks; ++tid) {
    if (!store.is_track_valid(tid) || !store.track_has_triangulated_xyz(tid))
      continue;
    if (track_degree[static_cast<size_t>(tid)] < 2)
      continue;
    if (selected.find(tid) == selected.end()) {
      store_mut->set_track_skip_ba(tid, true);
      ++n_skip;
    }
  }
  LOG(INFO) << "  [ba_grid] select_ba_subset: " << selected.size() << " in BA, " << n_skip
            << " marked kSkipFromBA  (target/img=" << target << "  gridsize≈"
            << std::max(
                   1, static_cast<int>(std::ceil(std::sqrt(
                          static_cast<double>(std::max(1, cameras.empty() ? 1 : cameras[0].width)) *
                          std::max(1, cameras.empty() ? 1 : cameras[0].height) / target))))
            << "px)";
}

// ─────────────────────────────────────────────────────────────────────────────
// Fixed-pose point optimisation for kSkipFromBA tracks
// ─────────────────────────────────────────────────────────────────────────────

/// Build a BAInput containing only the kSkipFromBA tracks.
/// ALL poses and intrinsics are fixed; ONLY 3D points are free.
static bool build_ba_input_skipped_tracks_fixed_pose(
    const TrackStore& store, const std::vector<Eigen::Matrix3d>& poses_R,
    const std::vector<Eigen::Vector3d>& poses_C, const std::vector<bool>& registered,
    const std::vector<int>& image_to_camera_index, const std::vector<camera::Intrinsics>& cameras,
    BAInput* ba_input_out, std::vector<int>* ba_image_index_to_global_out,
    std::vector<int>* point_index_to_track_id_out) {
  if (!ba_input_out || !ba_image_index_to_global_out || !point_index_to_track_id_out)
    return false;
  const int n_images = store.num_images();

  // Collect skipped tracks and their registered observers.
  struct SkipEntry {
    int track_id;
    float x, y, z;
    std::vector<std::pair<int, Observation>> obs; // (global_img_idx, obs)
  };
  std::vector<SkipEntry> entries;
  std::unordered_set<int> relevant_imgs;

  for (size_t ti = 0; ti < store.num_tracks(); ++ti) {
    const int tid = static_cast<int>(ti);
    if (!store.is_track_valid(tid) || !store.track_has_triangulated_xyz(tid))
      continue;
    if (!store.is_track_skip_ba(tid))
      continue;
    SkipEntry e;
    e.track_id = tid;
    const auto& track_obs_ids = store.track_all_obs_ids_view(tid);
    for (int obs_id : track_obs_ids) {
      if (!store.is_obs_valid(obs_id))
        continue;
      Observation o;
      o.image_index = store.obs_image_index(obs_id);
      o.feature_id = store.obs_feature_id(obs_id);
      o.u = store.obs_u(obs_id);
      o.v = store.obs_v(obs_id);
      o.scale = store.obs_scale(obs_id);
      const int im = static_cast<int>(o.image_index);
      if (im >= 0 && im < n_images && registered[static_cast<size_t>(im)]) {
        e.obs.emplace_back(im, o);
        relevant_imgs.insert(im);
      }
    }
    if (e.obs.size() < 2)
      continue;
    store.get_track_xyz(tid, &e.x, &e.y, &e.z);
    entries.push_back(std::move(e));
  }
  if (entries.empty())
    return false;

  // Build global→ba index.
  std::vector<int> global_indices(relevant_imgs.begin(), relevant_imgs.end());
  std::sort(global_indices.begin(), global_indices.end());
  std::unordered_map<int, int> g2ba;
  for (int bi = 0; bi < static_cast<int>(global_indices.size()); ++bi)
    g2ba[global_indices[bi]] = bi;

  BAInput& ba = *ba_input_out;
  ba.poses_R.clear();
  ba.poses_C.clear();
  ba.points3d.clear();
  ba.observations.clear();
  ba.image_camera_index.clear();
  ba.cameras = cameras;
  for (int g : global_indices) {
    ba.poses_R.push_back(poses_R[static_cast<size_t>(g)]);
    ba.poses_C.push_back(poses_C[static_cast<size_t>(g)]);
    const int ci = image_to_camera_index[static_cast<size_t>(g)];
    ba.image_camera_index.push_back(ci >= 0 && ci < static_cast<int>(cameras.size()) ? ci : 0);
  }
  // Fix ALL poses and intrinsics.
  ba.fix_pose.assign(global_indices.size(), true);
  ba.fix_intrinsics_flags.assign(cameras.size(),
                                 static_cast<uint32_t>(FixIntrinsicsMask::kFixIntrAll));
  ba.optimize_intrinsics = false;

  point_index_to_track_id_out->clear();
  for (const auto& e : entries) {
    const int pt_idx = static_cast<int>(point_index_to_track_id_out->size());
    point_index_to_track_id_out->push_back(e.track_id);
    ba.points3d.emplace_back(static_cast<double>(e.x), static_cast<double>(e.y),
                             static_cast<double>(e.z));
    for (const auto& [g, o] : e.obs) {
      auto it = g2ba.find(g);
      if (it == g2ba.end())
        continue;
      const double sc = (o.scale > 1e-6f) ? static_cast<double>(o.scale) : 1.0;
      ba.observations.push_back(
          {it->second, pt_idx, static_cast<double>(o.u), static_cast<double>(o.v), sc});
    }
  }
  ba.fix_point.assign(ba.points3d.size(), false); // 3D points are FREE
  *ba_image_index_to_global_out = global_indices;
  return true;
}

/// Run a fixed-pose Ceres solve for all kSkipFromBA tracks.
/// Poses + intrinsics are frozen; only 3D point positions are optimised.
/// Writes back updated XYZ to the store. Returns false if nothing to optimise.
static bool retri_skipped_tracks_fixed_pose(
    TrackStore* store, const std::vector<Eigen::Matrix3d>& poses_R,
    const std::vector<Eigen::Vector3d>& poses_C, const std::vector<bool>& registered,
    const std::vector<int>& image_to_camera_index, const std::vector<camera::Intrinsics>& cameras,
    int max_iterations, double* rmse_px_out, int ceres_num_threads) {
  BAInput ba_in;
  std::vector<int> ba_image_index_to_global;
  std::vector<int> point_index_to_track_id;
  if (!build_ba_input_skipped_tracks_fixed_pose(
          *store, poses_R, poses_C, registered, image_to_camera_index, cameras, &ba_in,
          &ba_image_index_to_global, &point_index_to_track_id))
    return false;
  if (ceres_num_threads > 0)
    ba_in.num_threads = ceres_num_threads;
  LOG(INFO) << "retri_skipped_tracks_fixed_pose: " << ba_in.poses_R.size() << " images, "
            << ba_in.points3d.size() << " skipped points, " << ba_in.observations.size() << " obs";
  BAResult ba_out;
  if (!global_bundle_analytic(ba_in, &ba_out, max_iterations)) {
    LOG(WARNING) << "retri_skipped_tracks_fixed_pose: solver failed";
    return false;
  }
  // Write back only 3D points (poses/intrinsics are fixed and unchanged).
  for (size_t pi = 0; pi < ba_out.points3d.size() && pi < point_index_to_track_id.size(); ++pi) {
    const int tid = point_index_to_track_id[pi];
    const Eigen::Vector3d& p = ba_out.points3d[pi];
    store->set_track_xyz(tid, static_cast<float>(p(0)), static_cast<float>(p(1)),
                         static_cast<float>(p(2)));
  }
  if (rmse_px_out)
    *rmse_px_out = ba_out.rmse_px;
  LOG(INFO) << "retri_skipped_tracks_fixed_pose: RMSE=" << ba_out.rmse_px
            << " px  success=" << ba_out.success;
  return ba_out.success;
}

bool build_ba_input_from_store(
    const TrackStore& store, const std::vector<Eigen::Matrix3d>& poses_R,
    const std::vector<Eigen::Vector3d>& poses_C, const std::vector<bool>& registered,
    const std::vector<int>& image_to_camera_index, const std::vector<camera::Intrinsics>& cameras,
    const std::vector<int>* image_subset, std::vector<int>* ba_image_index_to_global_out,
    BAInput* ba_input_out, std::vector<int>* point_index_to_track_id_out,
    const std::vector<bool>* skip_2deg_image_stable, // nullptr = disabled
    double skip_2deg_min_angle_score, int max_observations_per_track, int* n_skipped_2deg_out,
    int* n_skipped_grid_out) { // nullptr = don't report grid-skip count
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

  // ── Image-indexed two-pass approach ──────────────────────────────────────────────────────────
  // Old: iterate N_total_tracks (~400K), load obs for every tri+non-skip track to count
  //      how many obs land in BA window → O(N_tri_tracks × avg_obs_per_track).
  //      At n_reg=383 with 70K BA tracks out of 400K tri tracks, 90%+ obs loads are wasted.
  //
  // New:
  //   Pass A – iterate BA-window images (n_reg images × avg_obs/img),
  //            build track_ba_degree[tid] and track_ba_im_a/b for each tid seen ≥1 times.
  //            Cost: O(N_window_obs) — e.g. 383×~10K = ~3.8M ops, vs 400K×10 = 4M (similar)
  //            but memory access pattern is image_obs_ids_[] which is sequential/warm.
  //   Pass B – iterate only tids with track_ba_degree[tid] >= 2 (the accepted tracks),
  //            load track_all_obs_ids_view for those tracks only → O(N_accepted × avg_obs).
  //            Saves loading obs for N_tri_tracks - N_accepted ≈ 330K tracks.

  const int n_tracks = static_cast<int>(store.num_tracks());
  // Per-track BA-window visibility counters. -1 = unseen. Use a flat vector for O(1) access.
  // We also store the first two BA images that see each track (needed for 2-deg check and
  // select_track_observations_for_ba which sorts by view angle).
  std::vector<int8_t> track_ba_degree(static_cast<size_t>(n_tracks), 0);
  std::vector<int> track_ba_im_a(static_cast<size_t>(n_tracks), -1);
  std::vector<int> track_ba_im_b(static_cast<size_t>(n_tracks), -1);

  // Pass A: accumulate per-track BA-window degree by iterating registered images.
  {
    std::vector<int> img_obs_ids;
    for (int im : global_indices) {
      img_obs_ids.clear();
      store.get_image_observation_indices(im, &img_obs_ids);
      for (int obs_id : img_obs_ids) {
        const int tid = store.obs_track_id(obs_id);
        if (tid < 0 || tid >= n_tracks)
          continue;
        if (!store.is_track_valid(tid) || !store.track_has_triangulated_xyz(tid))
          continue;
        if (store.is_track_skip_ba(tid))
          continue;
        const int8_t deg = track_ba_degree[static_cast<size_t>(tid)];
        if (deg == 0) {
          track_ba_im_a[static_cast<size_t>(tid)] = im;
        } else if (deg == 1) {
          track_ba_im_b[static_cast<size_t>(tid)] = im;
        }
        // Cap at 127 to avoid int8_t overflow (only degree < 2 and == 2 matter for logic).
        if (deg < 127)
          track_ba_degree[static_cast<size_t>(tid)] = deg + 1;
      }
    }
  }

  // Count grid-skipped tracks for reporting (Pass A already skips them above; count them once).
  if (n_skipped_grid_out) {
    for (int tid = 0; tid < n_tracks; ++tid) {
      if (store.is_track_valid(tid) && store.track_has_triangulated_xyz(tid) &&
          store.is_track_skip_ba(tid))
        ++(*n_skipped_grid_out);
    }
  }

  point_index_to_track_id_out->clear();
  std::vector<Observation> obs_buf;

  int obs_before_sampling = 0;
  int obs_after_sampling = 0;
  int tracks_sampled = 0;

  // Pass B: process only tracks with BA-window degree >= 2.
  for (int track_id = 0; track_id < n_tracks; ++track_id) {
    if (track_ba_degree[static_cast<size_t>(track_id)] < 2)
      continue;

    const int im_a = track_ba_im_a[static_cast<size_t>(track_id)];
    const int im_b = track_ba_im_b[static_cast<size_t>(track_id)];
    const int visible_in_ba = static_cast<int>(track_ba_degree[static_cast<size_t>(track_id)]);

    // 2-degree track pruning: only when exactly 2 BA images see the track.
    if (skip_2deg_image_stable && !skip_2deg_image_stable->empty() && visible_in_ba == 2) {
      if (im_a >= 0 && im_b >= 0 && static_cast<size_t>(im_a) < skip_2deg_image_stable->size() &&
          static_cast<size_t>(im_b) < skip_2deg_image_stable->size() &&
          (*skip_2deg_image_stable)[static_cast<size_t>(im_a)] &&
          (*skip_2deg_image_stable)[static_cast<size_t>(im_b)]) {
        float tx, ty, tz;
        store.get_track_xyz(track_id, &tx, &ty, &tz);
        const Eigen::Vector3d X(static_cast<double>(tx), static_cast<double>(ty),
                                static_cast<double>(tz));
        const Eigen::Vector3d r0 = (X - poses_C[static_cast<size_t>(im_a)]).normalized();
        const Eigen::Vector3d r1 = (X - poses_C[static_cast<size_t>(im_b)]).normalized();
        const double cos_a = std::max(-1.0, std::min(1.0, r0.dot(r1)));
        const double sin_a_sq = 1.0 - cos_a * cos_a;
        if (sin_a_sq >= skip_2deg_min_angle_score) {
          if (n_skipped_2deg_out)
            ++(*n_skipped_2deg_out);
          continue;
        }
      }
    }

    // Load all obs for this accepted track (needed by select_track_observations_for_ba).
    obs_buf.clear();
    const auto& track_obs_ids = store.track_all_obs_ids_view(track_id);
    obs_buf.reserve(track_obs_ids.size());
    for (int obs_id : track_obs_ids) {
      if (!store.is_obs_valid(obs_id))
        continue;
      Observation o;
      o.image_index = store.obs_image_index(obs_id);
      o.feature_id = store.obs_feature_id(obs_id);
      o.u = store.obs_u(obs_id);
      o.v = store.obs_v(obs_id);
      o.scale = store.obs_scale(obs_id);
      obs_buf.push_back(o);
    }

    // Register point.
    const int pt_idx = static_cast<int>(point_index_to_track_id_out->size());
    point_index_to_track_id_out->push_back(track_id);
    float x, y, z;
    store.get_track_xyz(track_id, &x, &y, &z);
    ba.points3d.emplace_back(static_cast<double>(x), static_cast<double>(y),
                             static_cast<double>(z));

    const Eigen::Vector3d X = ba.points3d.back();
    int n_considered = 0;
    const auto selected_obs = select_track_observations_for_ba(
        obs_buf, global_indices, poses_R, poses_C, cameras, image_to_camera_index, X,
        max_observations_per_track, nullptr, &n_considered, nullptr, nullptr);
    obs_before_sampling += n_considered;
    obs_after_sampling += static_cast<int>(selected_obs.size());
    ++tracks_sampled;
    for (const auto& c : selected_obs) {
      const double scale = (c.obs.scale > 1e-6f) ? static_cast<double>(c.obs.scale) : 1.0;
      BAObservation bo;
      bo.image_index = c.ba_image_index;
      bo.point_index = pt_idx;
      bo.u = static_cast<double>(c.obs.u);
      bo.v = static_cast<double>(c.obs.v);
      bo.std_sigma_obs_px = scale;
      ba.observations.push_back(bo);
    }
  }

  ba.fix_point.resize(ba.points3d.size(), false);

  if (tracks_sampled > 0 && obs_before_sampling > 0) {
    LOG(INFO) << "BA obs sampling(global): tracks=" << tracks_sampled
              << " before=" << obs_before_sampling << " after=" << obs_after_sampling
              << " kept_ratio="
              << (100.0 * static_cast<double>(obs_after_sampling) /
                  static_cast<double>(obs_before_sampling))
              << "% max_per_track=" << max_observations_per_track;
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
    const std::vector<int>& batch, int max_variable_cameras, int max_observations_per_track,
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
  for (size_t ti = 0; ti < n_tracks; ++ti) {
    const int track_id = static_cast<int>(ti);
    if (!store.is_track_valid(track_id) || !store.track_has_triangulated_xyz(track_id))
      continue;
    const auto& track_obs_ids = store.track_all_obs_ids_view(track_id);
    for (int obs_id : track_obs_ids) {
      if (!store.is_obs_valid(obs_id))
        continue;
      if (batch_set.count(static_cast<int>(store.obs_image_index(obs_id)))) {
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
    const auto& track_obs_ids = store.track_all_obs_ids_view(track_id);
    for (int obs_id : track_obs_ids) {
      if (!store.is_obs_valid(obs_id))
        continue;
      const int g = static_cast<int>(store.obs_image_index(obs_id));
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
  std::vector<uint8_t> batch_image_mask(static_cast<size_t>(n_images), 0);
  for (int g : batch_set) {
    if (g >= 0 && g < n_images)
      batch_image_mask[static_cast<size_t>(g)] = 1;
  }
  std::vector<uint8_t> variable_image_mask(static_cast<size_t>(n_images), 0);
  for (int g : variable_set) {
    if (g >= 0 && g < n_images)
      variable_image_mask[static_cast<size_t>(g)] = 1;
  }

  // ── Seed points: ≥2 observations in all_image_set AND ≥1 in variable cameras ─
  // Points only seen by constant cameras contribute zero gradient to variable
  // camera parameters (their Jacobian rows only affect constant blocks).
  // Filtering them out reduces BA scale without losing any information for the
  // variables we actually want to optimize.
  std::vector<int> track_id_to_point_index(n_tracks, -1);
  point_index_to_track_id_out->clear();
  std::vector<Observation> obs_buf;
  for (int track_id : seed_track_ids) {
    int visible_in_ba = 0;
    int visible_in_variable = 0;
    const auto& track_obs_ids = store.track_all_obs_ids_view(track_id);
    for (int obs_id : track_obs_ids) {
      if (!store.is_obs_valid(obs_id))
        continue;
      const int g = static_cast<int>(store.obs_image_index(obs_id));
      if (all_image_set.count(g)) {
        ++visible_in_ba;
        if (variable_set.count(g))
          ++visible_in_variable;
      }
    }
    if (visible_in_ba < 2 || visible_in_variable < 1)
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
  ba.observations.reserve(ba.points3d.size() * static_cast<size_t>(max_observations_per_track));

  // ── Observations ─────────────────────────────────────────────────────────
  int obs_before_sampling = 0;
  int obs_after_sampling = 0;
  int required_obs_before_sampling = 0;
  int required_obs_after_sampling = 0;
  int tracks_sampled = 0;
  int tracks_with_required = 0;
  for (size_t ti = 0; ti < n_tracks; ++ti) {
    const int track_id = static_cast<int>(ti);
    const int pt_idx = track_id_to_point_index[track_id];
    if (pt_idx < 0)
      continue;
    obs_buf.clear();
    const auto& track_obs_ids = store.track_all_obs_ids_view(track_id);
    obs_buf.reserve(track_obs_ids.size());
    for (int obs_id : track_obs_ids) {
      if (!store.is_obs_valid(obs_id))
        continue;
      Observation o;
      o.image_index = store.obs_image_index(obs_id);
      o.feature_id = store.obs_feature_id(obs_id);
      o.u = store.obs_u(obs_id);
      o.v = store.obs_v(obs_id);
      o.scale = store.obs_scale(obs_id);
      obs_buf.push_back(o);
    }
    float tx, ty, tz;
    store.get_track_xyz(track_id, &tx, &ty, &tz);
    const Eigen::Vector3d X(static_cast<double>(tx), static_cast<double>(ty),
                            static_cast<double>(tz));
    int n_considered = 0;
    int n_required_considered = 0;
    int n_required_selected = 0;
    const auto selected_obs = select_track_observations_for_ba(
        obs_buf, global_indices, poses_R, poses_C, cameras, image_to_camera_index, X,
        max_observations_per_track, &variable_image_mask, &n_considered, &n_required_considered,
        &n_required_selected);
    obs_before_sampling += n_considered;
    obs_after_sampling += static_cast<int>(selected_obs.size());
    required_obs_before_sampling += n_required_considered;
    required_obs_after_sampling += n_required_selected;
    if (n_required_considered > 0)
      ++tracks_with_required;
    ++tracks_sampled;
    for (const auto& c : selected_obs) {
      const double scale = (c.obs.scale > 1e-6f) ? static_cast<double>(c.obs.scale) : 1.0;
      BAObservation bo;
      bo.image_index = c.ba_image_index;
      bo.point_index = pt_idx;
      bo.u = static_cast<double>(c.obs.u);
      bo.v = static_cast<double>(c.obs.v);
      bo.std_sigma_obs_px = scale;
      ba.observations.push_back(bo);
    }
  }
  if (tracks_sampled > 0 && obs_before_sampling > 0) {
    const double kept_ratio =
        100.0 * static_cast<double>(obs_after_sampling) / static_cast<double>(obs_before_sampling);
    const double required_kept_ratio =
        (required_obs_before_sampling > 0)
            ? (100.0 * static_cast<double>(required_obs_after_sampling) /
               static_cast<double>(required_obs_before_sampling))
            : 100.0;
    LOG(INFO) << "BA obs sampling(local-colmap): tracks=" << tracks_sampled
              << " tracks_with_required=" << tracks_with_required
              << " before=" << obs_before_sampling << " after=" << obs_after_sampling
              << " kept_ratio=" << kept_ratio
              << "% required_before=" << required_obs_before_sampling
              << " required_after=" << required_obs_after_sampling
              << " required_kept_ratio=" << required_kept_ratio
              << "% max_per_track=" << max_observations_per_track;
  }
  return !ba.observations.empty();
}

} // namespace

namespace {
/// Weight for soft prior ‖C_anchor − C_im1‖ (distance target = current baseline at BA entry).
constexpr double kInitialPairDistancePriorWeight = 1e4;
constexpr double kInitialPairDistancePriorMinBaselineM = 1e-9;
} // namespace

bool run_global_ba(TrackStore* store, std::vector<Eigen::Matrix3d>* poses_R,
                   std::vector<Eigen::Vector3d>* poses_C, const std::vector<bool>& registered,
                   const std::vector<int>& image_to_camera_index,
                   const std::vector<camera::Intrinsics>& cameras,
                   std::vector<camera::Intrinsics>* cameras_in_out, bool optimize_intrinsics,
                   int max_iterations, double* rmse_px_out, int anchor_image,
                   int max_observations_per_track, double focal_prior_weight,
                   const BASolverOverrides& solver_overrides,
                   const std::vector<uint32_t>* partial_intr_fix_per_cam,
                   int initial_pair_global_im1, const std::vector<bool>* precomputed_image_stable,
                   double skip_2deg_min_angle_score) {
  if (!store || !poses_R || !poses_C)
    return false;
  using Clock = std::chrono::steady_clock;
  double initial_pair_baseline_m = 0.0;
  if (anchor_image >= 0 && initial_pair_global_im1 >= 0 &&
      static_cast<size_t>(anchor_image) < poses_C->size() &&
      static_cast<size_t>(initial_pair_global_im1) < poses_C->size() &&
      registered[static_cast<size_t>(anchor_image)] &&
      registered[static_cast<size_t>(initial_pair_global_im1)]) {
    initial_pair_baseline_m =
        ((*poses_C)[anchor_image] - (*poses_C)[initial_pair_global_im1]).norm();
  }
  std::vector<int> ba_image_index_to_global;
  std::vector<int> point_index_to_track_id;
  BAInput ba_in;
  int n_skipped_2deg = 0;
  int n_skipped_grid = 0;
  const auto t_build0 = Clock::now();
  if (!build_ba_input_from_store(*store, *poses_R, *poses_C, registered, image_to_camera_index,
                                 cameras, nullptr, &ba_image_index_to_global, &ba_in,
                                 &point_index_to_track_id, precomputed_image_stable,
                                 skip_2deg_min_angle_score, max_observations_per_track,
                                 &n_skipped_2deg, &n_skipped_grid)) {
    CHECK(false) << "Failed to build BA input from store";
    return false;
  }
  // Apply optional Ceres solver overrides.
  ba_in.solver_gradient_tolerance = solver_overrides.gradient_tolerance;
  ba_in.solver_function_tolerance = solver_overrides.function_tolerance;
  ba_in.solver_parameter_tolerance = solver_overrides.parameter_tolerance;
  ba_in.solver_dense_schur_max_variable_cams = solver_overrides.dense_schur_max_variable_cams;
  if (solver_overrides.max_num_iterations > 0)
    ba_in.solver_max_num_iterations = solver_overrides.max_num_iterations;
  if (solver_overrides.huber_loss_delta > 0.0)
    ba_in.huber_loss_delta = solver_overrides.huber_loss_delta;
  ba_in.tikhonov_lambda = solver_overrides.tikhonov_lambda;
  if (solver_overrides.num_threads > 0)
    ba_in.num_threads = solver_overrides.num_threads;
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
      CHECK(partial_intr_fix_per_cam != nullptr &&
            partial_intr_fix_per_cam->size() == ba_in.cameras.size())
          << "partial_intr_fix_per_cam size must match number of cameras";
      for (size_t c = 0; c < ba_in.cameras.size(); ++c) {
        ba_in.fix_intrinsics_flags[c] = (*partial_intr_fix_per_cam)[c];
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
  int anchor_ba_idx = 0;
  {
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
  // Initial pair: soft prior ‖C_anchor − C_im1‖ → baseline at BA entry (anchor fixed; im1 free).
  if (initial_pair_global_im1 >= 0) {
    int ba_im1 = -1;
    for (size_t bi = 0; bi < ba_image_index_to_global.size(); ++bi) {
      if (ba_image_index_to_global[bi] == initial_pair_global_im1) {
        ba_im1 = static_cast<int>(bi);
        break;
      }
    }
    if (ba_im1 >= 0 && ba_im1 != anchor_ba_idx &&
        initial_pair_baseline_m > kInitialPairDistancePriorMinBaselineM) {
      BACameraDistancePrior p;
      p.image_index_a = anchor_ba_idx;
      p.image_index_b = ba_im1;
      p.distance_m = initial_pair_baseline_m;
      p.weight = kInitialPairDistancePriorWeight;
      ba_in.camera_distance_priors.push_back(p);
      LOG(INFO) << "run_global_ba: initial-pair distance prior anchor_ba=" << anchor_ba_idx
                << " im1_ba=" << ba_im1 << " target_m=" << initial_pair_baseline_m
                << " weight=" << kInitialPairDistancePriorWeight;
    } else if (ba_im1 >= 0 && ba_im1 != anchor_ba_idx) {
      LOG(WARNING) << "run_global_ba: skipping initial-pair distance prior (baseline too small): "
                   << initial_pair_baseline_m;
    }
  }
  const int ms_build = static_cast<int>(
      std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t_build0).count());
  const int effective_max_iter =
      (ba_in.solver_max_num_iterations > 0) ? ba_in.solver_max_num_iterations : max_iterations;
  LOG(INFO) << "run_global_ba: " << ba_in.poses_R.size() << " images, " << ba_in.points3d.size()
            << " points (" << n_skipped_2deg << " 2deg-skip, " << n_skipped_grid << " grid-skip), "
            << ba_in.observations.size() << " obs, max_iter=" << effective_max_iter
            << "  build=" << ms_build << "ms  anchor_ba_idx=" << anchor_ba_idx;
  const auto t_ceres0 = Clock::now();
  BAResult ba_out;
  if (!global_bundle_analytic(ba_in, &ba_out, max_iterations)) {
    LOG(INFO) << "run_global_ba: solver failed";
    return false;
  }
  const int ms_ceres = static_cast<int>(
      std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t_ceres0).count());
  LOG(INFO) << "run_global_ba: RMSE=" << ba_out.rmse_px << " px  ceres=" << ms_ceres << "ms";
  write_ba_result_back(ba_out, ba_image_index_to_global, point_index_to_track_id, store, poses_R,
                       poses_C, cameras_in_out);
  // Log current intrinsics so drift / optimisation progress is visible
  log_cameras(cameras_in_out ? *cameras_in_out : cameras, "run_global_ba");
  // Epoch-gate: bump pose epoch so that the next full-scan retriangulation knows
  // poses have changed and all stable tracks are stale.  Between two GBA calls,
  // local BA does NOT bump the epoch, so periodic full scans can skip tracks
  // that were successfully triangulated after the last GBA.
  store->bump_global_pose_epoch();
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
                  int max_observations_per_track, const BASolverOverrides& overrides) {
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
                                 &point_index_to_track_id, nullptr, 0.0, max_observations_per_track,
                                 nullptr, nullptr))
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
  if (overrides.parameter_tolerance > 0.0)
    ba_in.solver_parameter_tolerance = overrides.parameter_tolerance;
  if (overrides.num_threads > 0)
    ba_in.num_threads = overrides.num_threads;
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

/// Outlier rejection pass for kColmap local BA, called after write_ba_result_back.
///
/// Three-tier deletion policy based on camera category AND point type:
///
///   For batch cameras (newly registered):
///     ALL tracks — tight threshold: max(2·1.4826·MAD(batch_errors), 2px)
///     Reason: fresh PnP pose may have incorrect observations for any track.
///
///   For variable historical cameras:
///     VARIABLE tracks only (newly triangulated, fix_point=false):
///       loose threshold: max(2·tight, 8px) + min-obs-count protection
///     CONSTANT tracks (old established tracks, fix_point=true):
///       NEVER deleted — their XYZ was not changed by this BA, they are stable.
///
///   Constant cameras (gauge anchors, frozen pose):
///     DEFAULT: never delete (historical behaviour).
///     OPTIONAL (late scene): if scene_num_registered ≥ gross_min_reg and reproj > gross_px,
///       mark kRestorable — gross mismatches only (intrinsics assumed stable); still honour
///       kMinObsProtect using per-image valid obs counts for those constant cameras.
///
/// This mirrors the kBatchNeighbor philosophy: only clean up observations that
/// involve newly introduced geometry; leave established tracks untouched so that
/// global BA (with full scene context) decides their fate.
///
/// After deleting observations, tracks with <2 valid registered observations
/// have XYZ cleared so run_retriangulation can recover them on the next pass.
///
/// Returns number of observations deleted.
static int reject_outliers_local_colmap(
    TrackStore* store, const std::vector<Eigen::Matrix3d>& poses_R,
    const std::vector<Eigen::Vector3d>& poses_C, const std::vector<bool>& registered,
    const std::vector<camera::Intrinsics>& cameras, const std::vector<int>& image_to_camera_index,
    const std::vector<int>& ba_image_index_to_global,
    const std::vector<int>& point_index_to_track_id, const std::vector<bool>& fix_pose,
    const std::unordered_set<int>& batch_global_set,
    const std::unordered_set<int>& variable_track_set, int scene_num_registered,
    double constant_cam_gross_outlier_px, int constant_cam_gross_outlier_min_registered) {
  if (!store || ba_image_index_to_global.empty() || point_index_to_track_id.empty())
    return 0;

  const int n_global = static_cast<int>(poses_R.size());

  // Build lookup: global image index → BA-local index
  std::unordered_map<int, int> global_to_ba_local;
  global_to_ba_local.reserve(ba_image_index_to_global.size() * 2);
  for (int bi = 0; bi < static_cast<int>(ba_image_index_to_global.size()); ++bi)
    global_to_ba_local[ba_image_index_to_global[bi]] = bi;

  // Computes Euclidean reprojection error (px) using updated global poses.
  // Returns -1 if point/camera is invalid, 1e9 if point is behind camera.
  auto compute_error = [&](int g, float u_obs, float v_obs, int track_id) -> double {
    if (g < 0 || g >= n_global || !registered[static_cast<size_t>(g)])
      return -1.0;
    if (!store->is_track_valid(track_id) || !store->track_has_triangulated_xyz(track_id))
      return -1.0;
    float tx, ty, tz;
    store->get_track_xyz(track_id, &tx, &ty, &tz);
    const Eigen::Vector3d X(static_cast<double>(tx), static_cast<double>(ty),
                            static_cast<double>(tz));
    const Eigen::Vector3d p =
        poses_R[static_cast<size_t>(g)] * (X - poses_C[static_cast<size_t>(g)]);
    if (p(2) <= 1e-12)
      return 1e9;
    const int cam_idx = image_to_camera_index[static_cast<size_t>(g)];
    const camera::Intrinsics& K = cameras[static_cast<size_t>(cam_idx)];
    const double xn = p(0) / p(2), yn = p(1) / p(2);
    double xd, yd;
    camera::apply_distortion(xn, yn, K, &xd, &yd);
    const double du = static_cast<double>(u_obs) - (K.fx * xd + K.cx);
    const double dv = static_cast<double>(v_obs) - (K.fy * yd + K.cy);
    return std::sqrt(du * du + dv * dv);
  };

  // Pass 1: collect batch-camera errors to estimate tight threshold via MAD
  std::vector<double> batch_errors;
  batch_errors.reserve(1024);
  std::vector<Observation> obs_buf;
  for (int track_id : point_index_to_track_id) {
    if (!store->is_track_valid(track_id) || !store->track_has_triangulated_xyz(track_id))
      continue;
    obs_buf.clear();
    const auto& track_obs_ids = store->track_all_obs_ids_view(track_id);
    obs_buf.reserve(track_obs_ids.size());
    for (int obs_id : track_obs_ids) {
      if (!store->is_obs_valid(obs_id))
        continue;
      Observation o;
      o.image_index = store->obs_image_index(obs_id);
      o.feature_id = store->obs_feature_id(obs_id);
      o.u = store->obs_u(obs_id);
      o.v = store->obs_v(obs_id);
      o.scale = store->obs_scale(obs_id);
      obs_buf.push_back(o);
    }
    for (const auto& o : obs_buf) {
      const int g = static_cast<int>(o.image_index);
      if (!global_to_ba_local.count(g) || !batch_global_set.count(g))
        continue;
      const double err = compute_error(g, o.u, o.v, track_id);
      if (err >= 0.0 && err < 1e8)
        batch_errors.push_back(err);
    }
  }

  // MAD-based tight threshold; fall back to 4px if too few samples
  double tight_thresh = 4.0;
  if (batch_errors.size() >= 4) {
    std::vector<double> sorted_errs = batch_errors;
    std::sort(sorted_errs.begin(), sorted_errs.end());
    const double median = sorted_errs[sorted_errs.size() / 2];
    std::vector<double> abs_devs;
    abs_devs.reserve(sorted_errs.size());
    for (double e : sorted_errs)
      abs_devs.push_back(std::abs(e - median));
    std::sort(abs_devs.begin(), abs_devs.end());
    const double mad = abs_devs[abs_devs.size() / 2];
    tight_thresh = std::max(2.0 * 1.4826 * mad, 2.0);
  }
  // Loose threshold for variable historical cameras: use a generous floor (8px) to
  // prevent cascading observation loss.  Tight = for batch cameras only.
  const double loose_thresh = std::max(2.0 * tight_thresh, 8.0);

  // Precompute valid-obs count for variable historical cameras so we can protect
  // cameras that are close to losing all constraints.
  // kMinObsProtect: once a historical camera has ≤ this many valid triangulated
  // observations, local BA will not delete any more of its observations.
  // Global BA will perform final cleanup with full scene context.
  const int kMinObsProtect = 50;
  std::unordered_map<int, int> cam_valid_obs;
  {
    std::vector<int> img_obs_ids;
    for (const auto& kv : global_to_ba_local) {
      const int g = kv.first;
      if (batch_global_set.count(g))
        continue; // batch cameras don't need this protection
      const int ba_local = kv.second;
      if (ba_local < static_cast<int>(fix_pose.size()) && fix_pose[static_cast<size_t>(ba_local)])
        continue; // constant cameras are already protected
      img_obs_ids.clear();
      store->get_image_observation_indices(g, &img_obs_ids);
      int cnt = 0;
      for (int oid : img_obs_ids) {
        if (!store->is_obs_valid(oid))
          continue;
        const int tid = store->obs_track_id(oid);
        if (store->is_track_valid(tid) && store->track_has_triangulated_xyz(tid))
          ++cnt;
      }
      cam_valid_obs[g] = cnt;
    }
  }

  const bool gross_constant = constant_cam_gross_outlier_px > 0.0 &&
                              constant_cam_gross_outlier_min_registered > 0 &&
                              scene_num_registered >= constant_cam_gross_outlier_min_registered;

  // Valid triangulated obs count per constant camera in the BA window (for gross-outlier path).
  std::unordered_map<int, int> constant_cam_valid_obs;
  if (gross_constant) {
    std::vector<int> img_obs_ids_gc;
    for (const auto& kv : global_to_ba_local) {
      const int g = kv.first;
      const int ba_local = kv.second;
      if (!(ba_local < static_cast<int>(fix_pose.size()) &&
            fix_pose[static_cast<size_t>(ba_local)]))
        continue;
      img_obs_ids_gc.clear();
      store->get_image_observation_indices(g, &img_obs_ids_gc);
      int cnt = 0;
      for (int oid : img_obs_ids_gc) {
        if (!store->is_obs_valid(oid))
          continue;
        const int tid = store->obs_track_id(oid);
        if (store->is_track_valid(tid) && store->track_has_triangulated_xyz(tid))
          ++cnt;
      }
      constant_cam_valid_obs[g] = cnt;
    }
  }

  // Pass 2: delete outlier observations.
  // Iterate only over BA-window images (not the full global obs array) to stay O(N_window_obs).
  int marked = 0;
  std::unordered_set<int> dirty_tracks;
  {
    std::vector<int> img_obs_ids_p2;
    for (const auto& kv : global_to_ba_local) {
      const int g = kv.first;
      const int ba_local = kv.second;
      const bool is_constant =
          (ba_local < static_cast<int>(fix_pose.size())) && fix_pose[static_cast<size_t>(ba_local)];

      img_obs_ids_p2.clear();
      store->get_image_observation_indices(g, &img_obs_ids_p2);
      for (int obs_id : img_obs_ids_p2) {
        if (!store->is_obs_valid(obs_id))
          continue;
        const int track_id = store->obs_track_id(obs_id);
        if (!store->is_track_valid(track_id) || !store->track_has_triangulated_xyz(track_id))
          continue;
        const double err = compute_error(g, store->obs_u(obs_id), store->obs_v(obs_id), track_id);
        if (err < 0.0)
          continue;

        if (is_constant) {
          if (!gross_constant || err <= constant_cam_gross_outlier_px)
            continue;
          auto gc_it = constant_cam_valid_obs.find(g);
          if (gc_it != constant_cam_valid_obs.end() && gc_it->second <= kMinObsProtect)
            continue;
          store->mark_observation_deleted_restorable(obs_id);
          if (gc_it != constant_cam_valid_obs.end())
            gc_it->second--;
          dirty_tracks.insert(track_id);
          ++marked;
          continue;
        }

        const bool is_batch = batch_global_set.count(g) > 0;
        const bool is_variable_track = variable_track_set.count(track_id) > 0;

        // Historical cameras only get pruned for VARIABLE (newly triangulated) tracks.
        // Observations on CONSTANT (old established) tracks are left for global BA to evaluate.
        // Batch cameras are always pruned for all tracks (their PnP pose may have errors anywhere).
        if (!is_batch && !is_variable_track)
          continue;

        const double thresh = is_batch ? tight_thresh : loose_thresh;
        if (err > thresh) {
          // Protect historical cameras with few remaining valid observations.
          if (!is_batch) {
            auto cnt_it = cam_valid_obs.find(g);
            if (cnt_it != cam_valid_obs.end() && cnt_it->second <= kMinObsProtect)
              continue; // protected: let global BA handle this camera
          }
          // Mark as kRestorable (not permanently deleted) so that kPendingOnly/kFullScan
          // retriangulation can still use this observation via include_deleted_restorable_obs=true.
          //
          // Why: local-BA reproj errors can be inflated by a poor initial focal-length estimate —
          // observations rejected here might be geometrically correct but show up as outliers
          // only because intrinsics haven't converged yet.  Permanently deleting them cuts off the
          // observation chain for unregistered images whose tracks rely on these registered views,
          // causing those images to be missed by choose_resection_candidates and never registered.
          //
          // With kRestorable the observation is excluded from BA (deleted) but is still used
          // by triangulate_track_common(include_deleted_restorable=true), allowing kPendingOnly
          // to re-triangulate the track once better poses/intrinsics are available.  Truly bad
          // matches will still fail the robust triangulation angle/reproj test.
          store->mark_observation_deleted_restorable(obs_id);
          // Track remaining valid obs count for the protection check above.
          if (!is_batch) {
            auto cnt_it = cam_valid_obs.find(g);
            if (cnt_it != cam_valid_obs.end())
              cnt_it->second--;
          }
          dirty_tracks.insert(track_id);
          ++marked;
        }
      }
    }
  }

  // Clear XYZ for tracks that now lack ≥2 valid registered observations
  const int n_total = static_cast<int>(registered.size());
  std::vector<int> obs_ids_chk;
  for (int tid : dirty_tracks) {
    if (!store->is_track_valid(tid) || !store->track_has_triangulated_xyz(tid))
      continue;
    obs_ids_chk.clear();
    store->get_track_obs_ids(tid, &obs_ids_chk);
    int valid_reg = 0;
    for (int oid : obs_ids_chk) {
      if (!store->is_obs_valid(oid))
        continue;
      Observation o;
      store->get_obs(oid, &o);
      const int im = static_cast<int>(o.image_index);
      if (im >= 0 && im < n_total && registered[static_cast<size_t>(im)])
        ++valid_reg;
    }
    if (valid_reg < 2)
      store->clear_track_xyz(tid);
  }
  return marked;
}

bool run_local_ba_colmap(TrackStore* store, std::vector<Eigen::Matrix3d>* poses_R,
                         std::vector<Eigen::Vector3d>* poses_C, const std::vector<bool>& registered,
                         const std::vector<int>& image_to_camera_index,
                         const std::vector<camera::Intrinsics>& cameras,
                         const std::vector<int>& batch, int max_variable_cameras,
                         int max_iterations, double* rmse_px_out, int max_observations_per_track,
                         const BASolverOverrides& overrides, int scene_num_registered,
                         double constant_cam_gross_outlier_px,
                         int constant_cam_gross_outlier_min_registered) {
  if (!store || !poses_R || !poses_C || batch.empty())
    return false;
  std::vector<int> ba_image_index_to_global;
  std::vector<int> point_index_to_track_id;
  BAInput ba_in;
  if (!build_ba_input_colmap_local(*store, *poses_R, *poses_C, registered, image_to_camera_index,
                                   cameras, batch, max_variable_cameras, max_observations_per_track,
                                   &ba_image_index_to_global, &ba_in, &point_index_to_track_id))
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
  if (overrides.parameter_tolerance > 0.0)
    ba_in.solver_parameter_tolerance = overrides.parameter_tolerance;
  if (overrides.num_threads > 0)
    ba_in.num_threads = overrides.num_threads;
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
  // Outlier rejection on the local window using updated poses/points.
  // Only variable 3D points (newly triangulated, fix_point=false) are subject to deletion
  // for historical cameras.  Constant (old established) tracks are left for global BA.
  const std::unordered_set<int> batch_global_set(batch.begin(), batch.end());
  std::unordered_set<int> variable_track_set;
  for (size_t pi = 0; pi < point_index_to_track_id.size(); ++pi) {
    if (pi < ba_in.fix_point.size() && !ba_in.fix_point[pi])
      variable_track_set.insert(point_index_to_track_id[pi]);
  }
  const int n_rejected = reject_outliers_local_colmap(
      store, *poses_R, *poses_C, registered, cameras, image_to_camera_index,
      ba_image_index_to_global, point_index_to_track_id, ba_in.fix_pose, batch_global_set,
      variable_track_set, scene_num_registered, constant_cam_gross_outlier_px,
      constant_cam_gross_outlier_min_registered);
  LOG(INFO) << "run_local_ba_colmap: outlier_rejected=" << n_rejected
            << " (variable_tracks=" << variable_track_set.size() << ")";
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
    int max_observations_per_track, std::vector<int>* ba_image_index_to_global_out,
    BAInput* ba_input_out, std::vector<int>* point_index_to_track_id_out) {
  if (!ba_input_out || !ba_image_index_to_global_out || !point_index_to_track_id_out)
    return false;
  const int n_images = store.num_images();
  const size_t n_tracks = store.num_tracks();
  std::set<int> batch_set(batch.begin(), batch.end());
  std::set<int> new_track_set(new_track_ids.begin(), new_track_ids.end());
  std::vector<uint8_t> batch_image_mask(static_cast<size_t>(n_images), 0);
  for (int g : batch_set) {
    if (g >= 0 && g < n_images)
      batch_image_mask[static_cast<size_t>(g)] = 1;
  }

  // ── Per-batch-image top-K neighbors ──────────────────────────────────────
  std::set<int> constant_set;
  std::vector<Observation> obs_buf;
  for (int b : batch) {
    // Count shared triangulated tracks between b and each registered historical image.
    std::unordered_map<int, int> neighbor_score;
    const auto& batch_obs_ids = store.image_all_obs_ids_view(b);
    for (int obs_id : batch_obs_ids) {
      if (!store.is_obs_valid(obs_id))
        continue;
      const int tid = store.obs_track_id(obs_id);
      if (!store.is_track_valid(tid) || !store.track_has_triangulated_xyz(tid))
        continue;
      const auto& track_obs_ids = store.track_all_obs_ids_view(tid);
      for (int track_obs_id : track_obs_ids) {
        if (!store.is_obs_valid(track_obs_id))
          continue;
        const int g = static_cast<int>(store.obs_image_index(track_obs_id));
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
    int vis = 0;
    const auto& track_obs_ids = store.track_all_obs_ids_view(track_id);
    for (int obs_id : track_obs_ids) {
      if (!store.is_obs_valid(obs_id))
        continue;
      if (all_image_set.count(static_cast<int>(store.obs_image_index(obs_id))))
        ++vis;
    }
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
    const auto& const_obs_ids = store.image_all_obs_ids_view(c);
    for (int obs_id : const_obs_ids) {
      if (!store.is_obs_valid(obs_id))
        continue;
      const int tid = store.obs_track_id(obs_id);
      if (!new_track_set.count(tid))
        old_candidates.insert(tid);
    }
  }
  for (int tid : old_candidates) {
    // Must also be visible from at least one batch image to constrain the new camera pose.
    bool seen_from_batch = false;
    const auto& track_obs_ids = store.track_all_obs_ids_view(tid);
    for (int obs_id : track_obs_ids) {
      if (!store.is_obs_valid(obs_id))
        continue;
      if (batch_set.count(static_cast<int>(store.obs_image_index(obs_id)))) {
        seen_from_batch = true;
        break;
      }
    }
    if (seen_from_batch)
      try_add_track(tid, true);
  }

  if (ba.points3d.empty())
    return false;

  ba.observations.reserve(ba.points3d.size() * static_cast<size_t>(max_observations_per_track));

  // ── Observations ─────────────────────────────────────────────────────────
  int obs_before_sampling = 0;
  int obs_after_sampling = 0;
  int required_obs_before_sampling = 0;
  int required_obs_after_sampling = 0;
  int tracks_sampled = 0;
  int tracks_with_required = 0;
  for (size_t ti = 0; ti < n_tracks; ++ti) {
    const int track_id = static_cast<int>(ti);
    const int pt_idx = track_id_to_point_index[track_id];
    if (pt_idx < 0)
      continue;
    obs_buf.clear();
    const auto& track_obs_ids = store.track_all_obs_ids_view(track_id);
    obs_buf.reserve(track_obs_ids.size());
    for (int obs_id : track_obs_ids) {
      if (!store.is_obs_valid(obs_id))
        continue;
      Observation o;
      o.image_index = store.obs_image_index(obs_id);
      o.feature_id = store.obs_feature_id(obs_id);
      o.u = store.obs_u(obs_id);
      o.v = store.obs_v(obs_id);
      o.scale = store.obs_scale(obs_id);
      obs_buf.push_back(o);
    }
    float tx, ty, tz;
    store.get_track_xyz(track_id, &tx, &ty, &tz);
    const Eigen::Vector3d X(static_cast<double>(tx), static_cast<double>(ty),
                            static_cast<double>(tz));
    int n_considered = 0;
    int n_required_considered = 0;
    int n_required_selected = 0;
    const auto selected_obs = select_track_observations_for_ba(
        obs_buf, global_indices, poses_R, poses_C, cameras, image_to_camera_index, X,
        max_observations_per_track, &batch_image_mask, &n_considered, &n_required_considered,
        &n_required_selected);
    obs_before_sampling += n_considered;
    obs_after_sampling += static_cast<int>(selected_obs.size());
    required_obs_before_sampling += n_required_considered;
    required_obs_after_sampling += n_required_selected;
    if (n_required_considered > 0)
      ++tracks_with_required;
    ++tracks_sampled;
    for (const auto& c : selected_obs) {
      const double sigma_feat = (c.obs.scale > 1e-6f) ? static_cast<double>(c.obs.scale) : 1.0;
      BAObservation bo;
      bo.image_index = c.ba_image_index;
      bo.point_index = pt_idx;
      bo.u = static_cast<double>(c.obs.u);
      bo.v = static_cast<double>(c.obs.v);
      bo.std_sigma_obs_px = sigma_to_ba_stddev(sigma_feat);
      ba.observations.push_back(bo);
    }
  }
  if (tracks_sampled > 0 && obs_before_sampling > 0) {
    const double kept_ratio =
        100.0 * static_cast<double>(obs_after_sampling) / static_cast<double>(obs_before_sampling);
    const double required_kept_ratio =
        (required_obs_before_sampling > 0)
            ? (100.0 * static_cast<double>(required_obs_after_sampling) /
               static_cast<double>(required_obs_before_sampling))
            : 100.0;
    LOG(INFO) << "BA obs sampling(local-batch-neighbor): tracks=" << tracks_sampled
              << " tracks_with_required=" << tracks_with_required
              << " before=" << obs_before_sampling << " after=" << obs_after_sampling
              << " kept_ratio=" << kept_ratio
              << "% required_before=" << required_obs_before_sampling
              << " required_after=" << required_obs_after_sampling
              << " required_kept_ratio=" << required_kept_ratio
              << "% max_per_track=" << max_observations_per_track;
  }
  return !ba.observations.empty();
}

bool run_local_ba_batch_neighbor(
    TrackStore* store, std::vector<Eigen::Matrix3d>* poses_R, std::vector<Eigen::Vector3d>* poses_C,
    const std::vector<bool>& registered, const std::vector<int>& image_to_camera_index,
    const std::vector<camera::Intrinsics>& cameras, const std::vector<int>& batch,
    const std::vector<int>& new_track_ids, int neighbor_k, int max_iterations, double* rmse_px_out,
    int max_observations_per_track, const BASolverOverrides& overrides, int scene_num_registered,
    double constant_cam_gross_outlier_px, int constant_cam_gross_outlier_min_registered) {
  if (!store || !poses_R || !poses_C || batch.empty())
    return false;
  std::vector<int> ba_image_index_to_global;
  std::vector<int> point_index_to_track_id;
  BAInput ba_in;
  if (!build_ba_input_batch_neighbor(*store, *poses_R, *poses_C, registered, image_to_camera_index,
                                     cameras, batch, new_track_ids, neighbor_k,
                                     max_observations_per_track, &ba_image_index_to_global, &ba_in,
                                     &point_index_to_track_id))
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
  if (overrides.parameter_tolerance > 0.0)
    ba_in.solver_parameter_tolerance = overrides.parameter_tolerance;
  if (overrides.num_threads > 0)
    ba_in.num_threads = overrides.num_threads;
  BAResult ba_out;
  if (!global_bundle_analytic(ba_in, &ba_out, max_iterations)) {
    LOG(WARNING) << "run_local_ba_batch_neighbor: solver failed";
    return false;
  }
  LOG(INFO) << "run_local_ba_batch_neighbor: RMSE=" << ba_out.rmse_px << " px";
  write_ba_result_back(ba_out, ba_image_index_to_global, point_index_to_track_id, store, poses_R,
                       poses_C, nullptr);

  // Outlier rejection: batch cameras — MAD tight thresh on all tracks; constant cameras — optional
  // gross-outlier path when scene_num_registered is past intrinsics convergence (see
  // LocalBAOptions).
  const std::unordered_set<int> batch_global_set(batch.begin(), batch.end());
  std::unordered_set<int> variable_track_set;
  for (size_t pi = 0; pi < point_index_to_track_id.size(); ++pi) {
    if (pi < ba_in.fix_point.size() && !ba_in.fix_point[pi])
      variable_track_set.insert(point_index_to_track_id[pi]);
  }
  const int n_rejected = reject_outliers_local_colmap(
      store, *poses_R, *poses_C, registered, cameras, image_to_camera_index,
      ba_image_index_to_global, point_index_to_track_id, ba_in.fix_pose, batch_global_set,
      variable_track_set, scene_num_registered, constant_cam_gross_outlier_px,
      constant_cam_gross_outlier_min_registered);
  LOG(INFO) << "run_local_ba_batch_neighbor: outlier_rejected=" << n_rejected
            << " (variable_tracks=" << variable_track_set.size() << ")";

  if (rmse_px_out)
    *rmse_px_out = ba_out.rmse_px;
  return true;
}

bool run_local_ba_dispatch(const LocalBAOptions& opts, int anchor_image, int scene_num_registered,
                           TrackStore* store, std::vector<Eigen::Matrix3d>* poses_R,
                           std::vector<Eigen::Vector3d>* poses_C,
                           const std::vector<bool>& registered,
                           const std::vector<int>& image_to_camera_index,
                           const std::vector<camera::Intrinsics>& cameras,
                           const std::vector<int>& batch, const std::vector<int>& new_track_ids,
                           const BASolverOverrides& ov, double* rmse_px_out) {
  const int gross_min_eff = effective_local_ba_gross_constant_cam_min_reg(opts);
  switch (opts.strategy) {
  case LocalBAStrategy::kColmap:
    return run_local_ba_colmap(store, poses_R, poses_C, registered, image_to_camera_index, cameras,
                               batch, opts.colmap_max_variable_images, opts.max_iterations,
                               rmse_px_out, opts.max_observations_per_track, ov,
                               scene_num_registered, opts.constant_cam_gross_outlier_px,
                               gross_min_eff);
  case LocalBAStrategy::kBatchNeighbor:
    return run_local_ba_batch_neighbor(
        store, poses_R, poses_C, registered, image_to_camera_index, cameras, batch, new_track_ids,
        opts.neighbor_k, opts.max_iterations, rmse_px_out, opts.max_observations_per_track, ov,
        scene_num_registered, opts.constant_cam_gross_outlier_px, gross_min_eff);
  case LocalBAStrategy::kWindow: {
    std::vector<int> connectivity_indices;
    const std::vector<int>* local_indices = nullptr;
    if (opts.by_connectivity && !batch.empty()) {
      connectivity_indices =
          choose_local_ba_indices_by_connectivity(*store, registered, batch, opts.window);
      if (!connectivity_indices.empty())
        local_indices = &connectivity_indices;
    }
    return run_local_ba(store, poses_R, poses_C, registered, image_to_camera_index, cameras,
                        opts.window, opts.max_iterations, rmse_px_out, local_indices, anchor_image,
                        opts.max_observations_per_track, ov);
  }
  default:
    LOG(ERROR) << "run_local_ba_dispatch: unknown LocalBAStrategy";
    return false;
  }
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

// ── Track structure diagnostics (called on BA failure to understand JtJ degeneracy) ──────────
static void diagnose_track_structure(const TrackStore& store, const std::vector<bool>& registered,
                                     const std::vector<int>& image_to_camera_index) {
  const int n_images = store.num_images();
  const int n_tracks = static_cast<int>(store.num_tracks());

  // Per-image observation count
  std::vector<int> obs_per_image(static_cast<size_t>(n_images), 0);
  // Track length histogram: track_len_hist[k] = number of tracks seen by exactly k cameras
  std::map<int, int> track_len_hist;
  // Co-visibility: covis[i][j] = number of shared tracks between image i and j
  // We use a flat upper-triangle map to avoid O(n^2) memory for large n
  std::unordered_map<uint64_t, int> covis_map;

  for (int t = 0; t < n_tracks; ++t) {
    if (!store.is_track_valid(t) || !store.track_has_triangulated_xyz(t))
      continue;
    // Count only registered images
    std::vector<int> reg_imgs;
    const auto& track_obs_ids = store.track_all_obs_ids_view(t);
    for (int obs_id : track_obs_ids) {
      if (!store.is_obs_valid(obs_id))
        continue;
      const int im = static_cast<int>(store.obs_image_index(obs_id));
      if (im >= 0 && im < n_images && registered[static_cast<size_t>(im)])
        reg_imgs.push_back(im);
    }
    const int len = static_cast<int>(reg_imgs.size());
    if (len < 1)
      continue;
    track_len_hist[len]++;
    for (int im : reg_imgs)
      obs_per_image[static_cast<size_t>(im)]++;
    // Co-visibility pairs (only for tracks seen by ≥2 cameras)
    for (int a = 0; a < len; ++a) {
      for (int b = a + 1; b < len; ++b) {
        const int ia = reg_imgs[static_cast<size_t>(a)];
        const int ib = reg_imgs[static_cast<size_t>(b)];
        const uint64_t key = (static_cast<uint64_t>(std::min(ia, ib)) << 32) |
                             static_cast<uint64_t>(std::max(ia, ib));
        covis_map[key]++;
      }
    }
  }

  // ── Track length distribution ─────────────────────────────────────────────
  int total_tracks = 0, short_tracks = 0;
  for (const auto& kv : track_len_hist)
    total_tracks += kv.second;
  for (const auto& kv : track_len_hist)
    if (kv.first <= 2)
      short_tracks += kv.second;

  std::ostringstream oss;
  oss << "[track_diag] track_len_distribution (len: count):";
  for (const auto& kv : track_len_hist)
    oss << "  " << kv.first << ":" << kv.second;
  LOG(INFO) << oss.str();
  LOG(INFO) << "[track_diag] total_triangulated_tracks=" << total_tracks
            << "  short_tracks(len<=2)=" << short_tracks
            << "  short_ratio=" << (total_tracks > 0 ? 100.0 * short_tracks / total_tracks : 0.0)
            << "%";

  // ── Per-image observation count; flag under-constrained cameras ──────────
  int n_registered = 0;
  for (bool r : registered)
    if (r)
      ++n_registered;
  const int kMinObsForWellConstrained = 30;
  int n_underconstrained = 0;
  for (int i = 0; i < n_images; ++i) {
    if (!registered[static_cast<size_t>(i)])
      continue;
    if (obs_per_image[static_cast<size_t>(i)] < kMinObsForWellConstrained) {
      ++n_underconstrained;
      LOG(WARNING) << "[track_diag] underconstrained image " << i
                   << " (cam=" << image_to_camera_index[static_cast<size_t>(i)]
                   << ")  obs=" << obs_per_image[static_cast<size_t>(i)];
    }
  }
  // Obs count percentiles
  std::vector<int> obs_sorted;
  obs_sorted.reserve(static_cast<size_t>(n_registered));
  for (int i = 0; i < n_images; ++i)
    if (registered[static_cast<size_t>(i)])
      obs_sorted.push_back(obs_per_image[static_cast<size_t>(i)]);
  std::sort(obs_sorted.begin(), obs_sorted.end());
  const int p10 = obs_sorted.empty() ? 0 : obs_sorted[obs_sorted.size() / 10];
  const int p50 = obs_sorted.empty() ? 0 : obs_sorted[obs_sorted.size() / 2];
  const int p90 = obs_sorted.empty() ? 0 : obs_sorted[obs_sorted.size() * 9 / 10];
  LOG(INFO) << "[track_diag] obs_per_registered_image  p10=" << p10 << "  p50=" << p50
            << "  p90=" << p90 << "  underconstrained(<" << kMinObsForWellConstrained
            << ")=" << n_underconstrained << "/" << n_registered;

  // ── Co-visibility sparsity ────────────────────────────────────────────────
  const int n_pairs_possible = n_registered * (n_registered - 1) / 2;
  const int n_pairs_with_covis = static_cast<int>(covis_map.size());
  // Collect covis counts for percentile
  std::vector<int> covis_counts;
  covis_counts.reserve(covis_map.size());
  int isolated_pairs = 0;
  for (const auto& kv : covis_map) {
    covis_counts.push_back(kv.second);
    if (kv.second < 5)
      ++isolated_pairs;
  }
  std::sort(covis_counts.begin(), covis_counts.end());
  const int cp10 = covis_counts.empty() ? 0 : covis_counts[covis_counts.size() / 10];
  const int cp50 = covis_counts.empty() ? 0 : covis_counts[covis_counts.size() / 2];
  const int cp90 = covis_counts.empty() ? 0 : covis_counts[covis_counts.size() * 9 / 10];
  LOG(INFO) << "[track_diag] covis_pairs=" << n_pairs_with_covis << "/" << n_pairs_possible
            << "  sparsity="
            << (n_pairs_possible > 0
                    ? 100.0 * (n_pairs_possible - n_pairs_with_covis) / n_pairs_possible
                    : 0.0)
            << "%  covis_count p10=" << cp10 << " p50=" << cp50 << " p90=" << cp90
            << "  weak_pairs(<5)=" << isolated_pairs;

  // ── Per-camera summary ────────────────────────────────────────────────────
  if (image_to_camera_index.empty())
    return;
  const int n_cameras = static_cast<int>(
      *std::max_element(image_to_camera_index.begin(), image_to_camera_index.end()) + 1);
  std::vector<int> obs_per_camera(static_cast<size_t>(n_cameras), 0);
  std::vector<int> images_per_camera(static_cast<size_t>(n_cameras), 0);
  for (int i = 0; i < n_images; ++i) {
    if (!registered[static_cast<size_t>(i)])
      continue;
    const int c = image_to_camera_index[static_cast<size_t>(i)];
    if (c >= 0 && c < n_cameras) {
      obs_per_camera[static_cast<size_t>(c)] += obs_per_image[static_cast<size_t>(i)];
      images_per_camera[static_cast<size_t>(c)]++;
    }
  }
  for (int c = 0; c < n_cameras; ++c) {
    LOG(INFO) << "[track_diag] camera " << c
              << ": registered_images=" << images_per_camera[static_cast<size_t>(c)]
              << "  total_obs=" << obs_per_camera[static_cast<size_t>(c)];
  }
}

// ─── Alternating BA: fallback when joint SPARSE_SCHUR/CHOLMOD fails ──────────
//
// When the joint Schur complement is not positive definite (e.g. due to near-collinear
// cameras, distortion over-fitting, or degenerate scene geometry), we fall back to
// alternating optimization:
//   Step A: fix all 3D points, optimize poses + intrinsics  (small Schur complement,
//           DENSE_SCHUR is robust even for near-degenerate configurations)
//   Step B: fix all poses + intrinsics, optimize 3D points  (block-diagonal Hessian,
//           never singular)
// Iterate until convergence (RMSE change < tol) or max_outer_iters reached.
//
// Returns true if at least one outer iteration succeeded and the final RMSE is finite.
static bool run_alternating_ba(
    TrackStore* store, std::vector<Eigen::Matrix3d>* poses_R, std::vector<Eigen::Vector3d>* poses_C,
    const std::vector<bool>& registered, const std::vector<int>& image_to_camera_index,
    std::vector<camera::Intrinsics>* cameras, int anchor_image, bool optimize_intrinsics,
    const std::vector<uint32_t>* partial_intr_fix_per_cam, double focal_prior_weight,
    double* rmse_px_out, int max_outer_iters = 5, int inner_iters = 100,
    double convergence_tol = 1e-3, int ceres_num_threads = 0) {
  if (!store || !poses_R || !poses_C || !cameras)
    return false;

  double prev_rmse = std::numeric_limits<double>::max();
  bool any_success = false;

  for (int outer = 0; outer < max_outer_iters; ++outer) {
    // ── Step A: fix points, optimize poses + intrinsics ───────────────────
    {
      std::vector<int> ba_image_index_to_global;
      std::vector<int> point_index_to_track_id;
      BAInput ba_in;
      if (!build_ba_input_from_store(*store, *poses_R, *poses_C, registered, image_to_camera_index,
                                     *cameras, nullptr, &ba_image_index_to_global, &ba_in,
                                     &point_index_to_track_id, nullptr, 0.0, 0, nullptr, nullptr))
        break;

      // Fix all 3D points
      ba_in.fix_point.assign(ba_in.points3d.size(), true);

      // Intrinsics flags
      ba_in.fix_intrinsics_flags.assign(ba_in.cameras.size(), 0u);
      if (!optimize_intrinsics) {
        ba_in.fix_intrinsics_flags.assign(ba_in.cameras.size(),
                                          static_cast<uint32_t>(FixIntrinsicsMask::kFixIntrAll));
      } else if (partial_intr_fix_per_cam &&
                 partial_intr_fix_per_cam->size() == ba_in.cameras.size()) {
        for (size_t c = 0; c < ba_in.cameras.size(); ++c)
          ba_in.fix_intrinsics_flags[c] = (*partial_intr_fix_per_cam)[c];
      }
      bool any_variable_intr = false;
      for (const uint32_t f : ba_in.fix_intrinsics_flags)
        if (f != static_cast<uint32_t>(FixIntrinsicsMask::kFixIntrAll)) {
          any_variable_intr = true;
          break;
        }
      ba_in.optimize_intrinsics = any_variable_intr;
      ba_in.focal_prior_weight = focal_prior_weight;

      // Anchor
      ba_in.fix_pose.assign(ba_in.poses_R.size(), false);
      for (size_t bi = 0; bi < ba_image_index_to_global.size(); ++bi) {
        if (ba_image_index_to_global[bi] == anchor_image) {
          ba_in.fix_pose[bi] = true;
          break;
        }
      }

      // Use DENSE_SCHUR — robust for near-degenerate pose-only problems
      ba_in.solver_dense_schur_max_variable_cams = static_cast<int>(ba_in.poses_R.size()) + 1;
      ba_in.huber_loss_delta = 4.0;
      if (ceres_num_threads > 0)
        ba_in.num_threads = ceres_num_threads;

      BAResult ba_out;
      if (!global_bundle_analytic(ba_in, &ba_out, inner_iters) || !ba_out.success) {
        LOG(WARNING) << "  [alternating_ba] outer=" << outer << " step A failed";
        break;
      }
      write_ba_result_back(ba_out, ba_image_index_to_global, point_index_to_track_id, store,
                           poses_R, poses_C, cameras);
      any_success = true;
      LOG(INFO) << "  [alternating_ba] outer=" << outer << " step A RMSE=" << ba_out.rmse_px
                << " px";
    }

    // ── Step B: fix poses + intrinsics, optimize 3D points ───────────────
    double rmse_b = 0.0;
    {
      std::vector<int> ba_image_index_to_global;
      std::vector<int> point_index_to_track_id;
      BAInput ba_in;
      if (!build_ba_input_from_store(*store, *poses_R, *poses_C, registered, image_to_camera_index,
                                     *cameras, nullptr, &ba_image_index_to_global, &ba_in,
                                     &point_index_to_track_id, nullptr, 0.0, 0, nullptr, nullptr))
        break;

      // Fix all poses
      ba_in.fix_pose.assign(ba_in.poses_R.size(), true);
      // Fix all intrinsics
      ba_in.fix_intrinsics_flags.assign(ba_in.cameras.size(),
                                        static_cast<uint32_t>(FixIntrinsicsMask::kFixIntrAll));
      ba_in.optimize_intrinsics = false;
      ba_in.focal_prior_weight = 0.0;

      // Points are free (default fix_point = false from build_ba_input_from_store)
      ba_in.huber_loss_delta = 4.0;
      // DENSE_SCHUR with all poses fixed degenerates to point-only; use SPARSE_NORMAL_CHOLESKY
      // which is block-diagonal and always positive definite for point-only problems.
      ba_in.solver_dense_schur_max_variable_cams = 0; // force SPARSE_SCHUR path
      if (ceres_num_threads > 0)
        ba_in.num_threads = ceres_num_threads;

      BAResult ba_out;
      if (!global_bundle_analytic(ba_in, &ba_out, inner_iters) || !ba_out.success) {
        LOG(WARNING) << "  [alternating_ba] outer=" << outer << " step B failed";
        break;
      }
      write_ba_result_back(ba_out, ba_image_index_to_global, point_index_to_track_id, store,
                           poses_R, poses_C, cameras);
      rmse_b = ba_out.rmse_px;
      LOG(INFO) << "  [alternating_ba] outer=" << outer << " step B RMSE=" << rmse_b << " px";
    }

    // ── Convergence check ─────────────────────────────────────────────────
    if (prev_rmse < std::numeric_limits<double>::max()) {
      const double delta = std::abs(prev_rmse - rmse_b) / std::max(prev_rmse, 1e-6);
      if (delta < convergence_tol) {
        LOG(INFO) << "  [alternating_ba] converged at outer=" << outer << " delta_rmse=" << delta
                  << " rmse=" << rmse_b << " px";
        if (rmse_px_out)
          *rmse_px_out = rmse_b;
        return true;
      }
    }
    prev_rmse = rmse_b;
  }

  if (rmse_px_out)
    *rmse_px_out = prev_rmse;

  log_cameras(*cameras, "run_global_ba");
  return any_success;
}

bool run_ba_with_outlier_detection(TrackStore* store, std::vector<Eigen::Matrix3d>* poses_R,
                                   std::vector<Eigen::Vector3d>* poses_C,
                                   const std::vector<bool>& registered,
                                   const std::vector<int>& image_to_camera_index,
                                   std::vector<camera::Intrinsics>* cameras, int anchor_image,
                                   int num_registered, const IncrementalSfMOptions& opts,
                                   double* rmse_px_out, int initial_pair_im1_global) {
  using Clock = std::chrono::steady_clock;
  double rmse = 0.0;
  bool ok = false;
  const bool do_reject = (num_registered >= opts.outlier.min_registered_images);
  const int n_fine_rounds = std::max(1, opts.outlier.max_rounds);

  // Sub-phase timing accumulators (profiling).
  // ms_run_global_ba: total time inside run_global_ba (build_input + Ceres + writeback).
  uint64_t ms_run_global_ba = 0, ms_collect_reproj = 0, ms_outlier_reject = 0;
  int n_ba_calls = 0;
  auto add_ms_u = [](uint64_t* acc, const Clock::time_point& a, const Clock::time_point& b) {
    *acc +=
        static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(b - a).count());
  };

  // Precompute per-image stability for 2-degree track pruning once; reused across all fine rounds.
  std::vector<bool> precomputed_image_stable_2deg;
  if (opts.global_ba.skip_2degree_tracks) {
    precomputed_image_stable_2deg = compute_image_stability(
        *store, *poses_C, registered, opts.global_ba.skip_2degree_min_angle_score,
        opts.global_ba.skip_2degree_min_stable_obs, opts.global_ba.skip_2degree_stable_ratio);
    int n_stable = 0;
    for (bool b : precomputed_image_stable_2deg)
      if (b)
        ++n_stable;
    LOG(INFO) << "  [ba_outlier] 2deg-skip: " << n_stable << " / "
              << precomputed_image_stable_2deg.size() << " images stable";
  }
  const std::vector<bool>* p_image_stable_2deg =
      opts.global_ba.skip_2degree_tracks ? &precomputed_image_stable_2deg : nullptr;
  // Per-camera intrinsics fix-mask only depends on registered/image_to_camera_index/camera count
  // in this BA call; reuse across all fallback rounds.
  const std::vector<uint32_t> per_cam_masks = opts.intrinsics.fix_masks_per_camera(
      registered, image_to_camera_index, static_cast<int>(cameras->size()));

  // Grid-NMS BA subset selection: recompute every ba_grid_reselect_every_n calls.
  if (opts.global_ba.ba_grid_subset && num_registered > 50) {
    static thread_local int ba_grid_call_count = 0;
    ++ba_grid_call_count;
    if ((ba_grid_call_count - 1) % std::max(1, opts.global_ba.ba_grid_reselect_every_n) == 0) {
      using Clock = std::chrono::steady_clock;
      auto t0 = Clock::now();
      select_ba_subset(*store, *poses_C, registered, image_to_camera_index, *cameras,
                       opts.global_ba, store);
      LOG(INFO) << "  [ba_grid] reselect done in "
                << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t0).count()
                << " ms";
    }
  }

  // ── Fix C: BA input cache ─────────────────────────────────────────────────────────────────────
  // Between fine rounds that reject < 0.5% of observations the observation set barely changes.
  // Skipping build_ba_input_from_store (O(N_tracks), ~33 s at 3000 images) and updating only
  // poses/XYZ in the cached BAInput saves most of this cost.
  //
  // Cache layout: BAInput + index mappings from the most recent full rebuild.
  // Invalidated when: (a) gba_cache.valid == false (first call)
  //                   (b) rejection_rate > 0.5%  (significant obs deleted → rebuild)
  //                   (c) force_rebuild == true   (error-recovery paths after angle rejection)
  //
  // Stale observations (deleted but still in cache) represent < 0.5% of total.
  // Ceres Huber loss (δ ≈ 4 px) down-weights high-error residuals so the impact is negligible.
  struct GlobalBACache {
    BAInput ba_in;
    std::vector<int> ba_image_index_to_global;
    std::vector<int> point_index_to_track_id;
    int n_skipped_2deg = 0;
    int n_skipped_grid = 0;
    bool valid = false;
  } gba_cache;

  // Pre-compute initial-pair baseline once (used for the distance prior on every BA round).
  const double gba_init_pair_baseline_m = [&]() -> double {
    if (anchor_image >= 0 && initial_pair_im1_global >= 0 &&
        static_cast<size_t>(anchor_image) < poses_C->size() &&
        static_cast<size_t>(initial_pair_im1_global) < poses_C->size() &&
        registered[static_cast<size_t>(anchor_image)] &&
        registered[static_cast<size_t>(initial_pair_im1_global)])
      return ((*poses_C)[anchor_image] - (*poses_C)[initial_pair_im1_global]).norm();
    return 0.0;
  }();

  // O(1): use the incrementally-maintained counter instead of scanning all observations.
  int total_obs_before = store->num_valid_observations();
  int last_rejected_count = 0; // updated after each fine round's outlier rejection

  // run_one_ba: build (or incrementally update) BA input, apply options, run Ceres, write back.
  // force_rebuild=true must be used whenever the observation set may have changed significantly
  // (angle-rejection fallbacks, Tikhonov recovery paths — called with !ok after BA failure).
  const auto run_one_ba = [&](BASolverOverrides ov, bool force_rebuild = false) -> bool {
    auto t_ba = Clock::now();
    ++n_ba_calls;

    // Decide rebuild vs. incremental-update.
    const float rej_rate = (total_obs_before > 0) ? static_cast<float>(last_rejected_count) /
                                                        static_cast<float>(total_obs_before)
                                                  : 1.0f;
    const bool do_rebuild = force_rebuild || !gba_cache.valid || rej_rate > 0.005f;

    if (do_rebuild) {
      const auto t_build0 = Clock::now();
      gba_cache.ba_in = BAInput{};
      gba_cache.ba_in.cameras = *cameras;
      gba_cache.ba_image_index_to_global.clear();
      gba_cache.point_index_to_track_id.clear();
      gba_cache.n_skipped_2deg = 0;
      gba_cache.n_skipped_grid = 0;
      if (!build_ba_input_from_store(*store, *poses_R, *poses_C, registered, image_to_camera_index,
                                     *cameras, nullptr, &gba_cache.ba_image_index_to_global,
                                     &gba_cache.ba_in, &gba_cache.point_index_to_track_id,
                                     p_image_stable_2deg,
                                     opts.global_ba.skip_2degree_min_angle_score,
                                     opts.global_ba.max_observations_per_track,
                                     &gba_cache.n_skipped_2deg, &gba_cache.n_skipped_grid)) {
        CHECK(false) << "Failed to build BA input from store";
        return false;
      }
      gba_cache.valid = true;
      const int ms_build = static_cast<int>(
          std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t_build0).count());
      LOG(INFO) << "run_global_ba(rebuild): " << gba_cache.ba_in.poses_R.size() << " images, "
                << gba_cache.ba_in.points3d.size() << " points (" << gba_cache.n_skipped_2deg
                << " 2deg-skip, " << gba_cache.n_skipped_grid << " grid-skip), "
                << gba_cache.ba_in.observations.size() << " obs  build=" << ms_build
                << "ms  rej_rate=" << (rej_rate * 100.0f) << "%";
    } else {
      // Incremental update: re-read poses and XYZ from current state.
      // Observation list stays unchanged (stale < 0.5% are down-weighted by Huber loss).
      for (size_t bi = 0; bi < gba_cache.ba_image_index_to_global.size(); ++bi) {
        const int g = gba_cache.ba_image_index_to_global[bi];
        if (g >= 0 && g < static_cast<int>(poses_R->size())) {
          gba_cache.ba_in.poses_R[bi] = (*poses_R)[static_cast<size_t>(g)];
          gba_cache.ba_in.poses_C[bi] = (*poses_C)[static_cast<size_t>(g)];
        }
      }
      for (size_t pi = 0; pi < gba_cache.point_index_to_track_id.size(); ++pi) {
        const int tid = gba_cache.point_index_to_track_id[pi];
        if (store->is_track_valid(tid) && store->track_has_triangulated_xyz(tid)) {
          float x, y, z;
          store->get_track_xyz(tid, &x, &y, &z);
          gba_cache.ba_in.points3d[pi] = Eigen::Vector3d(x, y, z);
        }
      }
      gba_cache.ba_in.cameras = *cameras; // pick up any intrinsics updates from prior rounds
      LOG(INFO) << "run_global_ba(cached): reused " << gba_cache.ba_in.poses_R.size() << " images, "
                << gba_cache.ba_in.points3d.size() << " points, "
                << gba_cache.ba_in.observations.size() << " obs  (rej_rate=" << (rej_rate * 100.0f)
                << "%, skipped rebuild)";
    }

    // Apply solver overrides.
    BAInput& ba = gba_cache.ba_in;
    ba.solver_gradient_tolerance = ov.gradient_tolerance;
    ba.solver_function_tolerance = ov.function_tolerance;
    ba.solver_parameter_tolerance = ov.parameter_tolerance;
    ba.solver_dense_schur_max_variable_cams = ov.dense_schur_max_variable_cams;
    if (ov.max_num_iterations > 0)
      ba.solver_max_num_iterations = ov.max_num_iterations;
    if (ov.huber_loss_delta > 0.0)
      ba.huber_loss_delta = ov.huber_loss_delta;
    ba.tikhonov_lambda = ov.tikhonov_lambda;
    if (ov.num_threads > 0)
      ba.num_threads = ov.num_threads;

    // Build per-camera intrinsics fix flags.
    {
      ba.fix_intrinsics_flags.assign(ba.cameras.size(), 0u);
      if (!opts.global_ba.optimize_intrinsics) {
        ba.fix_intrinsics_flags.assign(ba.cameras.size(),
                                       static_cast<uint32_t>(FixIntrinsicsMask::kFixIntrAll));
      } else {
        for (size_t c = 0; c < ba.cameras.size(); ++c)
          ba.fix_intrinsics_flags[c] = per_cam_masks[c];
      }
      bool any_variable = false;
      for (const uint32_t f : ba.fix_intrinsics_flags)
        if (f != static_cast<uint32_t>(FixIntrinsicsMask::kFixIntrAll)) {
          any_variable = true;
          break;
        }
      ba.optimize_intrinsics = any_variable;
      ba.focal_prior_weight = opts.intrinsics.focal_prior_weight;
    }

    // Fix the coordinate-system anchor.
    int anchor_ba_idx = 0;
    {
      if (anchor_image >= 0) {
        for (size_t bi = 0; bi < gba_cache.ba_image_index_to_global.size(); ++bi) {
          if (gba_cache.ba_image_index_to_global[bi] == anchor_image) {
            anchor_ba_idx = static_cast<int>(bi);
            break;
          }
        }
      }
      ba.fix_pose[static_cast<size_t>(anchor_ba_idx)] = true;
    }

    // Initial-pair distance prior (soft scale anchor).
    ba.camera_distance_priors.clear();
    if (initial_pair_im1_global >= 0 &&
        gba_init_pair_baseline_m > kInitialPairDistancePriorMinBaselineM) {
      int ba_im1 = -1;
      for (size_t bi = 0; bi < gba_cache.ba_image_index_to_global.size(); ++bi) {
        if (gba_cache.ba_image_index_to_global[bi] == initial_pair_im1_global) {
          ba_im1 = static_cast<int>(bi);
          break;
        }
      }
      if (ba_im1 >= 0 && ba_im1 != anchor_ba_idx) {
        BACameraDistancePrior pr;
        pr.image_index_a = anchor_ba_idx;
        pr.image_index_b = ba_im1;
        pr.distance_m = gba_init_pair_baseline_m;
        pr.weight = kInitialPairDistancePriorWeight;
        ba.camera_distance_priors.push_back(pr);
      }
    }

    const int effective_max_iter = (ba.solver_max_num_iterations > 0)
                                       ? ba.solver_max_num_iterations
                                       : opts.global_ba.max_iterations;
    LOG(INFO) << "run_global_ba: " << ba.poses_R.size() << " images, " << ba.points3d.size()
              << " points (" << gba_cache.n_skipped_2deg << " 2deg-skip, "
              << gba_cache.n_skipped_grid << " grid-skip), " << ba.observations.size()
              << " obs, max_iter=" << effective_max_iter << "  anchor_ba_idx=" << anchor_ba_idx;
    const auto t_ceres0 = Clock::now();
    BAResult ba_out;
    const bool step_ok = global_bundle_analytic(ba, &ba_out, opts.global_ba.max_iterations);
    const int ms_ceres = static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t_ceres0).count());
    LOG(INFO) << "run_global_ba: RMSE=" << ba_out.rmse_px << " px  ceres=" << ms_ceres << "ms";

    if (step_ok) {
      // Update cache in-place from Ceres results (fast: avoids re-reading from store next round).
      for (size_t bi = 0; bi < ba_out.poses_R.size() && bi < ba.poses_R.size(); ++bi) {
        ba.poses_R[bi] = ba_out.poses_R[bi];
        ba.poses_C[bi] = ba_out.poses_C[bi];
      }
      for (size_t pi = 0; pi < ba_out.points3d.size() && pi < ba.points3d.size(); ++pi)
        ba.points3d[pi] = ba_out.points3d[pi];
      if (!ba_out.cameras.empty())
        ba.cameras = ba_out.cameras;
      // Write results back to the external state (store XYZ, poses_R/C, cameras).
      write_ba_result_back(ba_out, gba_cache.ba_image_index_to_global,
                           gba_cache.point_index_to_track_id, store, poses_R, poses_C, cameras);
      log_cameras(*cameras, "run_global_ba");
      rmse = ba_out.rmse_px;
    }

    add_ms_u(&ms_run_global_ba, t_ba, Clock::now());
    VLOG(1) << "[PERF] ba_outlier: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t_ba).count()
            << "ms  RMSE=" << rmse << " px";
    return step_ok;
  };

  // Collect reproj errors once; returns errors AND sets huber delta on ov.
  // Result is reused for both Huber-delta (next round) and MAD threshold (current round).
  const auto collect_and_set_huber = [&](BASolverOverrides* ov) -> std::vector<double> {
    auto t0 = Clock::now();
    std::vector<double> errs = collect_reproj_errors(*store, *poses_R, *poses_C, registered,
                                                     *cameras, image_to_camera_index);
    add_ms_u(&ms_collect_reproj, t0, Clock::now());
    if (!errs.empty()) {
      const double delta =
          compute_huber_delta(errs, opts.outlier.huber_delta_lo_px, opts.outlier.huber_delta_hi_px);
      ov->huber_loss_delta = std::max(delta, 4.0);
      LOG(INFO) << "  [ba_outlier] Huber \xce\xb4=" << ov->huber_loss_delta << " px";
    } else {
      ov->huber_loss_delta = 4.0;
    }
    return errs;
  };

  // No outlier pass: single BA (Huber δ from residuals when possible).
  if (!do_reject) {
    BASolverOverrides ov = opts.global_ba.solver_overrides;
    collect_and_set_huber(&ov);
    ok = run_one_ba(ov);
    if (rmse_px_out)
      *rmse_px_out = rmse;
    return ok;
  }

  // ── Fine BA + MAD reproj + angle rejection ────────────────────────────────────────────────────
  // Collect errors once before round 0 to seed Huber delta; thereafter reuse the post-BA errors
  // from the previous round (which also compute MAD threshold for outlier rejection).
  // const bool intermediate_loose_enabled =
  //     opts.global_ba.intermediate_loose_after_images > 0 &&
  //     num_registered >= opts.global_ba.intermediate_loose_after_images &&
  //     opts.global_ba.intermediate_function_tolerance > 0.0;
  const bool intermediate_loose_enabled = false;

  // Pre-round Huber delta seed.
  BASolverOverrides ov_seed = opts.global_ba.solver_overrides;
  collect_and_set_huber(&ov_seed);
  double next_huber_delta = ov_seed.huber_loss_delta;

  for (int r = 0; r < n_fine_rounds; ++r) {
    BASolverOverrides ov = opts.global_ba.solver_overrides;
    ov.huber_loss_delta = next_huber_delta;
    const BASolverOverrides& g = opts.global_ba.solver_overrides;
    if (ov.function_tolerance == 0.0)
      ov.function_tolerance = (g.function_tolerance > 0.0) ? g.function_tolerance : 1e-6;
    if (ov.gradient_tolerance == 0.0)
      ov.gradient_tolerance = (g.gradient_tolerance > 0.0) ? g.gradient_tolerance : 1e-8;
    if (ov.parameter_tolerance == 0.0 && g.parameter_tolerance > 0.0)
      ov.parameter_tolerance = g.parameter_tolerance;
    if (ov.max_num_iterations == 0)
      ov.max_num_iterations = opts.global_ba.max_iterations;
    // Intermediate rounds (r>0): pose already established, just re-converge after outlier removal.
    // Apply loose tolerances to exit quickly; only when intrinsics are mature enough.
    if (r > 0 && intermediate_loose_enabled) {
      ov.function_tolerance =
          std::max(ov.function_tolerance, opts.global_ba.intermediate_function_tolerance);
      if (opts.global_ba.intermediate_max_iterations > 0)
        ov.max_num_iterations =
            std::min(ov.max_num_iterations, opts.global_ba.intermediate_max_iterations);
    }
    ok = run_one_ba(ov);
    if (!ok) {
      double min_deg_angle = 1.5;
      int rejected = reject_outliers_angle_multiview(store, *poses_R, *poses_C, registered,
                                                     min_deg_angle, opts.outlier.max_angle_deg);
      LOG(INFO) << "  [ba_outlier] fine round " << r << ": BA failed; angle-rejected " << rejected
                << " obs with angle < " << min_deg_angle << " deg";
      // Fallback after outlier rejection: observation set changed, force BA input rebuild.
      ok = run_one_ba(ov, true);
      if (!ok) {
        min_deg_angle = 2.0;
        rejected = reject_outliers_angle_multiview(store, *poses_R, *poses_C, registered,
                                                   min_deg_angle, opts.outlier.max_angle_deg);
        LOG(INFO) << "  [ba_outlier] fine round " << r << ": BA failed; angle-rejected " << rejected
                  << " obs with angle < " << min_deg_angle << " deg";
        // Second angle-rejection fallback: force rebuild again.
        ok = run_one_ba(ov, true);
        if (!ok) {
          // Alternating BA: fallback when joint solver fails (degenerate geometry / near-collinear
          // cameras). Alternate between fixing 3D points (optimize poses+intrinsics) and fixing
          // poses+intrinsics (optimize 3D points) until convergence.
          LOG(INFO) << "  [ba_outlier] fine round " << r
                    << ": joint BA failed; trying alternating BA fallback";
          ok = run_alternating_ba(store, poses_R, poses_C, registered, image_to_camera_index,
                                  cameras, anchor_image, opts.global_ba.optimize_intrinsics,
                                  &per_cam_masks, opts.intrinsics.focal_prior_weight, &rmse,
                                  /*max_outer_iters=*/5, /*inner_iters=*/100,
                                  /*convergence_tol=*/1e-3,
                                  opts.global_ba.solver_overrides.num_threads);
          if (ok) {
            LOG(INFO) << "  [ba_outlier] fine round " << r
                      << ": alternating BA succeeded, RMSE=" << rmse << " px";
            if (rmse_px_out)
              *rmse_px_out = rmse;
          } else {
            LOG(WARNING) << "  [ba_outlier] fine round " << r << ": alternating BA also failed";
          }
        }
        if (!ok) {
          LOG(WARNING) << "  [ba_outlier] fine round " << r
                       << ": BA failed even after angle-based outlier rejection";
          // Diagnose track structure to understand JtJ degeneracy
          diagnose_track_structure(*store, registered, image_to_camera_index);
          // Step 3: Tikhonov λ=1e-4
          BASolverOverrides ov3 = ov;
          ov3.tikhonov_lambda = 1e-4;
          LOG(INFO) << "  [ba_recovery] step 3/4: tikhonov lambda=1e-4";
          auto t_step3_0 = Clock::now();
          // Tikhonov recovery: forced rebuild (stale cached observations from prior failures).
          ok = run_one_ba(ov3, true);
          auto t_step3_1 = Clock::now();
          const double ms_step3 =
              std::chrono::duration<double, std::milli>(t_step3_1 - t_step3_0).count();
          VLOG(1) << "[PERF] ba_recovery_step3: " << ms_step3 << "ms";
          if (ok) {
            LOG(INFO) << "  [ba_recovery] recovered at step 3 using tikhonov lambda=1e-4"
                      << ", RMSE=" << rmse << " px";
          } else {
            LOG(WARNING) << "  [ba_recovery] step 3 failed (tikhonov lambda=1e-4)";
            // Step 4: Tikhonov λ=1e-2
            BASolverOverrides ov4 = ov;
            ov4.tikhonov_lambda = 1e-2;
            LOG(INFO) << "  [ba_recovery] step 4/4: tikhonov lambda=1e-2";
            auto t_step4_0 = Clock::now();
            // Tikhonov recovery: forced rebuild (stale cached observations from prior failures).
            ok = run_one_ba(ov4, true);
            auto t_step4_1 = Clock::now();
            const double ms_step4 =
                std::chrono::duration<double, std::milli>(t_step4_1 - t_step4_0).count();
            VLOG(1) << "[PERF] ba_recovery_step4: " << ms_step4 << "ms";
            if (ok) {
              LOG(INFO) << "  [ba_recovery] recovered at step 4 using tikhonov lambda=1e-2"
                        << ", RMSE=" << rmse << " px";
            } else {
              LOG(WARNING) << "  [ba_recovery] step 4 failed (tikhonov lambda=1e-2)";
              LOG(ERROR) << "  [ba_recovery] all recovery steps failed"
                         << " (num_registered=" << num_registered
                         << ", n_pts=" << store->num_tracks()
                         << ", n_obs=" << store->num_observations() << ")";
              return false;
            }
          }
        }
      }
    }

    // Single post-BA collect: errors used for both MAD threshold and next-round Huber delta.
    BASolverOverrides ov_next = opts.global_ba.solver_overrides;
    std::vector<double> errs = collect_and_set_huber(&ov_next);
    next_huber_delta = ov_next.huber_loss_delta;

    const double thr =
        compute_mad_threshold_from_errors(errs, opts.outlier.mad_k, opts.outlier.threshold_px);
    const double outlier_threshold = std::max(thr, 4.0);

    auto t_rej = Clock::now();
    int rejected = 0;
    auto t_r0 = Clock::now();
    rejected += reject_outliers_multiview(store, *poses_R, *poses_C, registered, *cameras,
                                          image_to_camera_index, outlier_threshold);
    auto t_r1 = Clock::now();
    rejected +=
        reject_outliers_angle_multiview(store, *poses_R, *poses_C, registered,
                                        opts.outlier.min_angle_deg, opts.outlier.max_angle_deg);
    auto t_r2 = Clock::now();
    rejected +=
        reject_outliers_depth(store, *poses_R, *poses_C, registered, opts.outlier.max_depth_factor);
    auto t_r3 = Clock::now();
    add_ms_u(&ms_outlier_reject, t_rej, Clock::now());
    const int ms_multiview = static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(t_r1 - t_r0).count());
    const int ms_angle = static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(t_r2 - t_r1).count());
    const int ms_depth = static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(t_r3 - t_r2).count());

    LOG(INFO) << "  [ba_outlier] fine round " << r << ": MAD reproj_thr=" << thr
              << " px  rejected=" << rejected << "  [diag] collect=" << ms_collect_reproj << "ms"
              << " multiview=" << ms_multiview << "ms"
              << " angle=" << ms_angle << "ms"
              << " depth=" << ms_depth << "ms";
    // Fix B: dynamic threshold scales with scene size (0.02% of total obs).
    // 476 imgs (399K obs): max(100, 80) = 100 → unchanged.
    // 3000 imgs (10.5M obs): max(100, 2100) = 2100 → ~2-3 fewer rounds per global BA call.
    last_rejected_count = rejected;
    const int dynamic_min_for_retry =
        std::max(opts.outlier.min_for_retry, static_cast<int>(total_obs_before * 0.0002));
    if (rejected < std::max(1, dynamic_min_for_retry))
      break;
  }
  if (rmse_px_out)
    *rmse_px_out = rmse;

  // Fixed-pose solve for skipped tracks (after main BA converges).
  if (ok && opts.global_ba.ba_fixed_pose_optimize_skipped &&
      (opts.global_ba.ba_grid_subset || opts.global_ba.skip_2degree_tracks)) {
    double skip_rmse = 0.0;
    retri_skipped_tracks_fixed_pose(store, *poses_R, *poses_C, registered, image_to_camera_index,
                                    *cameras, opts.global_ba.ba_fixed_pose_max_iterations,
                                    &skip_rmse, opts.global_ba.solver_overrides.num_threads);
  }

  // ms_run_global_ba covers: build_ba_input + Ceres solve + write_ba_result_back.
  const uint64_t ms_total = ms_run_global_ba + ms_collect_reproj + ms_outlier_reject;
  const auto pct_u = [&](uint64_t ms) -> double {
    return ms_total > 0 ? 100.0 * static_cast<double>(ms) / static_cast<double>(ms_total) : 0.0;
  };
  LOG(INFO) << "[ba_outlier][timing] run_global_ba=" << ms_run_global_ba << "ms("
            << static_cast<int>(pct_u(ms_run_global_ba))
            << "%) collect_reproj=" << ms_collect_reproj << "ms("
            << static_cast<int>(pct_u(ms_collect_reproj))
            << "%) outlier_reject=" << ms_outlier_reject << "ms("
            << static_cast<int>(pct_u(ms_outlier_reject)) << "%)  (ba_calls=" << n_ba_calls
            << ", total=" << ms_total << "ms)";

  return ok;
}

bool run_incremental_sfm_pipeline(const std::string& tracks_idc_path,
                                  const std::string& pairs_json_path, const std::string& geo_dir,
                                  std::vector<camera::Intrinsics>* cameras,
                                  const std::vector<int>& image_to_camera_index,
                                  const IncrementalSfMOptions& opts, TrackStore* store_out,
                                  std::vector<Eigen::Matrix3d>* poses_R_out,
                                  std::vector<Eigen::Vector3d>* poses_C_out,
                                  std::vector<bool>* registered_out) {
  if (!store_out || !poses_R_out || !poses_C_out || !registered_out || !cameras)
    return false;

  // Apply OMP thread count if explicitly set (> 0).
  // -1 (default) leaves the system/OMP default unchanged (typically = hardware threads).
  if (opts.omp_num_threads > 0) {
    omp_set_num_threads(opts.omp_num_threads);
    LOG(INFO) << "run_incremental_sfm_pipeline: omp_num_threads=" << opts.omp_num_threads;
  }

  ViewGraph view_graph;
  if (!load_track_store_from_idc(tracks_idc_path, store_out, nullptr, &view_graph)) {
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

  if (view_graph.num_pairs() == 0) {
    LOG(INFO) << "run_incremental_sfm_pipeline: no embedded view graph in tracks IDC; "
                 "building from pairs.json + geo_dir";
    if (!build_view_graph_from_geo(pairs_json_path, geo_dir, &view_graph)) {
      LOG(ERROR) << "run_incremental_sfm_pipeline: failed to build view graph";
      return false;
    }
  } else {
    LOG(INFO) << "run_incremental_sfm_pipeline: using embedded view graph from tracks IDC ("
              << view_graph.num_pairs() << " pairs)";
  }
  // Always capture anchor internally; caller's output pointers may be null.
  uint32_t local_im0 = 0, local_im1 = 0;
  uint32_t* im0_ptr = &local_im0;
  uint32_t* im1_ptr = &local_im1;
  if (!run_initial_pair_loop(view_graph, store_out, *cameras, image_to_camera_index,
                             opts.init.min_tracks_for_intital_pair,
                             opts.init.min_num_inliers,
                             opts.init.max_forward_motion,
                             opts.init.min_angle_deg,
                             opts.init.min_median_angle_deg,
                             im0_ptr, im1_ptr, poses_R_out, poses_C_out, registered_out,
                             opts.init.max_first_images, opts.init.max_second_images)) {
    LOG(ERROR) << "run_incremental_sfm_pipeline: no initial pair succeeded";
    return false;
  }
  int num_registered = 2;

  if (opts.max_registered_images > 0 && num_registered >= opts.max_registered_images) {
    LOG(INFO) << "Early stop: max_registered_images reached at initial pair ("
              << num_registered << "/" << opts.max_registered_images << ")";
    return true;
  }

  LOG(INFO) << "Initial pair done. Registered: " << num_registered << "/" << n_images;
  // ── Initial-pair diagnostic ──────────────────────────────────────────────
  {
    const int ip0 = static_cast<int>(*im0_ptr);
    const int ip1 = static_cast<int>(*im1_ptr);
    VLOG(1) << "  Initial pair images: im0=" << ip0 << " im1=" << ip1;
    const auto& C0 = (*poses_C_out)[static_cast<size_t>(ip0)];
    const auto& C1 = (*poses_C_out)[static_cast<size_t>(ip1)];
    VLOG(1) << "  C_im0=" << C0.transpose() << "  C_im1=" << C1.transpose()
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
      VLOG(1) << "  track " << tid << ": X=" << X.transpose() << "  depth0=" << depth0
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

  // count_tri_tracks: O(1) via TrackStore counter instead of full scan.
  auto count_tri_tracks = [&]() -> int { return store_out->num_triangulated_tracks(); };
  LOG(INFO) << "Triangulated tracks after initial pair: " << count_tri_tracks();

  using Clock = std::chrono::steady_clock;

  // Resection: one new image per iteration; try candidates in sorted order until one succeeds.
  // Local BA (e.g. kColmap) still expects a single new camera per iteration.
  constexpr int kResectionCandidateListMax = 40; ///< max ranked candidates to try per iteration
  const int resection_max_candidates = kResectionCandidateListMax;

  std::vector<ResectionCandidate> resection_candidates;

  // Per-image VisibilityPyramid score cache: avoids rebuilding the pyramid when n_tri has not
  // changed since the previous iteration (amortizes Eigen allocation + SetPoint cost).
  // n_tri watermark ensures automatic invalidation after triangulation without explicit
  // bookkeeping.
  ResectionScoreCache resection_score_cache;

  // Snapshot of camera intrinsics taken just before each global BA.
  // Used to detect significant focal-length changes that warrant re-evaluating kRestorable obs.
  std::vector<camera::Intrinsics> ba_cameras_snapshot;

  auto pipeline_start = Clock::now();
  int sfm_iter = 0;
  uint64_t ms_choose_candidates = 0;
  uint64_t ms_resection = 0;
  uint64_t ms_triangulation = 0;
  uint64_t ms_local_ba = 0;
  uint64_t ms_global_ba = 0;
  uint64_t ms_retriangulation = 0;
  auto add_ms = [](uint64_t* acc, const Clock::time_point& t0, const Clock::time_point& t1) {
    *acc += static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count());
  };
  // How many consecutive iterations ended with zero resection candidates.
  // On each such iteration we run a global BA + kFullScan triangulation pass before retrying.
  // Only give up when we fail twice in a row with no improvement.
  int no_candidate_consecutive = 0;
  constexpr int kMaxNoCandidateRetries = 2;
  int next_periodic_global_registered = -1; ///< Late-phase linear milestone for periodic global BA.
  int next_mid_global_registered = -1; ///< Mid-phase linear milestone for full global BA cadence.
  for (;;) {
    if (opts.max_registered_images > 0 && num_registered >= opts.max_registered_images) {
      LOG(INFO) << "Early stop: max_registered_images reached (" << num_registered << "/"
                << opts.max_registered_images << ")";
      break;
    }

    ++sfm_iter;
    auto iter_start = Clock::now();
    const uint64_t iter_ms_choose_t0 = ms_choose_candidates;
    const uint64_t iter_ms_resect_t0 = ms_resection;
    const uint64_t iter_ms_tri_t0 = ms_triangulation;
    const uint64_t iter_ms_lba_t0 = ms_local_ba;
    const uint64_t iter_ms_gba_t0 = ms_global_ba;
    const uint64_t iter_ms_retri_t0 = ms_retriangulation;
    LOG(INFO) << "════ SfM iter #" << sfm_iter << ": registered=" << num_registered << "/"
              << n_images << ", tri_tracks=" << count_tri_tracks() << " ════";

    auto t_choose0 = Clock::now();
    std::vector<int> new_registered_image_indices;
    resection_candidates = choose_resection_candidates(
        *store_out, *registered_out, *cameras, image_to_camera_index, opts.resection.min_3d2d_count,
        resection_max_candidates, opts.resection.min_visibility_coverage,
        static_cast<size_t>(std::max(1, opts.resection.visibility_pyramid_levels)),
        &resection_score_cache);
    auto t_choose1 = Clock::now();
    add_ms(&ms_choose_candidates, t_choose0, t_choose1);
    VLOG(1) << "[PERF] choose_resection_candidates: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(t_choose1 - t_choose0).count()
            << "ms";
    if (resection_candidates.empty()) {
      if (no_candidate_consecutive >= kMaxNoCandidateRetries) {
        LOG(INFO) << "  No resection candidates after " << no_candidate_consecutive
                  << " consecutive BA+triangulation retries, stopping";
        break;
      }
      ++no_candidate_consecutive;
      LOG(INFO) << "  No resection candidates (retry " << no_candidate_consecutive << "/"
                << kMaxNoCandidateRetries
                << "): running global BA + kFullScan triangulation to unlock new candidates";

      // ── Retry BA ─────────────────────────────────────────────────────────
      double rmse_retry = 0.0;
      auto t_retry_ba = Clock::now();
      if (!run_ba_with_outlier_detection(
              store_out, poses_R_out, poses_C_out, *registered_out, image_to_camera_index, cameras,
              anchor_image, num_registered, opts, &rmse_retry, static_cast<int>(*im1_ptr))) {
        LOG(WARNING) << "  [no_cand_retry] BA failed (RMSE=" << rmse_retry << " px)";
      } else {
        LOG(INFO) << "  [no_cand_retry] BA ok (RMSE=" << rmse_retry << " px); "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() -
                                                                           t_retry_ba)
                         .count()
                  << "ms";
      }
      add_ms(&ms_global_ba, t_retry_ba, Clock::now());

      // ── Retry kFullScan triangulation ─────────────────────────────────────
      // Use a more permissive min_tri_angle (0.5°) than the normal threshold (2.0°).
      // Late-stage unregistered images are typically covered only by weak-baseline tracks
      // (overlap strips at low elevation or far range). These tracks pass angle=0.5° but fail
      // angle=2.0°, which is why the normal post-BA full scan added only +200 tracks while
      // a diagnostic re-triangulation at 0.5° recovers ~25,000 tracks.
      {
        auto t_retry_fs = Clock::now();
        IncrementalRetriangulationOptions fs_retry;
        fs_retry.scope = RetriangulationScope::kFullScan;
        fs_retry.robust.min_tri_angle_deg = 0.5; // rescue mode: permissive angle
        fs_retry.robust.max_tri_angle_deg = opts.triangulation.max_angle_deg;
        fs_retry.full_scan_commit_reproj_px = opts.triangulation.commit_reproj_px;
        fs_retry.restore_strict_reproj_px = opts.triangulation.restore_reproj_px;
        const int n_retry_fs =
            run_retriangulation(store_out, *poses_R_out, *poses_C_out, *registered_out, *cameras,
                                image_to_camera_index, fs_retry);
        LOG(INFO) << "  [no_cand_retry] kFullScan: score=" << n_retry_fs
                  << "  total_tri=" << count_tri_tracks() << "  "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() -
                                                                           t_retry_fs)
                         .count()
                  << "ms";
        add_ms(&ms_retriangulation, t_retry_fs, Clock::now());
      }
      // Invalidate score cache so choose_resection_candidates re-evaluates all images.
      resection_score_cache.invalidate_all();

      // ── Retry with relaxed min_3d2d_count ────────────────────────────────────
      // If BA + kFullScan still can't produce candidates at the normal threshold,
      // try immediately with a halved threshold using the same resection_score_cache.
      // This handles the late-stage case where a few remaining images have fewer
      // 3D-2D correspondences — enough to PnP-RANSAC successfully but not enough
      // to pass the stricter candidate filter.
      {
        const int relaxed_min =
            std::max(opts.resection.min_inliers, opts.resection.min_3d2d_count / 2);
        if (relaxed_min < opts.resection.min_3d2d_count) {
          auto relaxed_candidates = choose_resection_candidates(
              *store_out, *registered_out, *cameras, image_to_camera_index, relaxed_min,
              resection_max_candidates, opts.resection.min_visibility_coverage,
              static_cast<size_t>(std::max(1, opts.resection.visibility_pyramid_levels)),
              &resection_score_cache);
          if (!relaxed_candidates.empty()) {
            LOG(INFO) << "  [no_cand_retry] relaxed min_3d2d=" << relaxed_min << " unlocked "
                      << relaxed_candidates.size() << " candidates"
                      << " (best: im=" << relaxed_candidates[0].image_index
                      << " 3d2d=" << relaxed_candidates[0].num_3d2d << ")";
            // Swap into resection_candidates and fall through to normal processing.
            resection_candidates = std::move(relaxed_candidates);
            // Do NOT reset no_candidate_consecutive here: relaxed unlock only lowers the 3D-2D
            // threshold — PnP can still fail every time. Resetting to 0 caused oscillation with
            // the [resection_fail] path (empty → BA → relaxed 1 candidate → fail → counter never
            // reached kMaxNoCandidateRetries). Successful resection still clears the counter below.
            // Don't continue — fall through to the rest of the loop (BA, tri, etc.).
            goto has_candidates;
          }
        }
      }
      continue; // re-run choose_resection_candidates next iteration
    }
    // Do NOT reset no_candidate_consecutive here: non-empty resection_candidates only means
    // images passed the candidate filter — PnP can still fail every time. Resetting here
    // prevented the [resection_fail] path from ever reaching kMaxNoCandidateRetries (infinite
    // outer iterations). Counter resets on successful resection below (no_candidate_consecutive
    // = 0 after new_registered_image_indices non-empty).
  has_candidates:
    // Determine whether to use local BA for this iteration BEFORE the resection loop,
    // because local BA mode processes one image at a time (breaks after first success).
    //
    // Three-phase progressive BA strategy:
    //   Phase 1 (n_registered < early_phase_global_only_images): global BA every registration.
    //   Phase 2 [early_phase_global_only_images, switch_after_n_images): mid-phase; typically local
    //   BA
    //     most iterations with full global BA when n_registered hits a milestone — either linear
    //     spacing (mid_phase_global_linear_spacing) or legacy every_n_images modulo.
    //   Phase 3 (>= switch_after_n_images): local-BA-per-iter + periodic global BA.
    const int early_end = opts.global_ba.early_phase_global_only_images;
    const int mid_end = opts.local_ba.switch_after_n_images;
    const bool in_late_phase = opts.local_ba.enable && num_registered >= mid_end;
    const bool in_mid_phase = opts.local_ba.enable && early_end > 0 &&
                              num_registered >= early_end && num_registered < mid_end;
    // bool use_local_ba = false;
    bool use_local_ba = num_registered >  opts.local_ba.switch_after_n_images;
    // if (in_late_phase) {
    //   use_local_ba = true;
    // } else if (in_mid_phase) {
    //   const bool mid_lin =
    //       opts.global_ba.mid_phase_global_linear_spacing &&
    //       (opts.global_ba.mid_global_spacing_a > 0.0 || opts.global_ba.mid_global_spacing_b > 0.0);
    //   if (mid_lin) {
    //     if (next_mid_global_registered < 0)
    //       next_mid_global_registered = early_end;
    //     // Full global when registration count reaches milestone; otherwise local BA only.
    //     use_local_ba = (num_registered < next_mid_global_registered);
    //   } else {
    //     const int n = std::max(1, opts.global_ba.every_n_images);
    //     use_local_ba = ((num_registered - early_end) % n != n - 1);
    //   }
    // }
    VLOG(1) << "  [ba_phase] n_reg=" << num_registered
            << (in_late_phase  ? " late"
                : in_mid_phase ? " mid"
                               : " early")
            << " use_local=" << use_local_ba;

    auto t_resect0 = Clock::now();
    // Accumulate new track IDs across all successful resections in this SfM iteration
    // so that kBatchNeighbor local BA can see all newly triangulated points.
    std::vector<int> all_new_track_ids;
    std::vector<int> registered_images_buf;
    std::vector<int> new_track_ids_buf;
    registered_images_buf.reserve(4);
    new_track_ids_buf.reserve(2048);
    new_registered_image_indices.reserve(static_cast<size_t>(resection_candidates.size()));
    all_new_track_ids.reserve(4096);
    // Try each candidate in score order; use the first that succeeds (degenerate configs, etc.).
    for (const ResectionCandidate& cand : resection_candidates) {
      // resection one image every time is very important, and then do triangulation
      registered_images_buf.clear();
      const int resection_minliers = opts.resection.min_inliers;
      double resection_min_inlier_ratio = opts.resection.min_inlier_ratio;
      const bool use_large_scene_ratio =
          (n_images >= opts.resection.large_scene_min_images &&
           num_registered >= opts.resection.large_scene_min_registered);
      if (use_large_scene_ratio) {
        resection_min_inlier_ratio =
            std::max(resection_min_inlier_ratio, opts.resection.min_inlier_ratio_large_scene);
      }
      auto t_resect_cand0 = Clock::now();
      const int n = run_batch_resection(*store_out, {cand.image_index}, *cameras,
                                        image_to_camera_index, poses_R_out, poses_C_out,
                                        registered_out, resection_minliers, &registered_images_buf,
                                        resection_min_inlier_ratio,
                                        opts.resection.post_resection_reproj_thresh_px);
      add_ms(&ms_resection, t_resect_cand0, Clock::now());
      VLOG(1) << "  [resection] img=" << cand.image_index << " 3d2d=" << cand.num_3d2d
              << " cov=" << cand.coverage << " min_ratio=" << resection_min_inlier_ratio
              << " → " << (n > 0 ? "OK" : "FAIL");
      if (n <= 0)
        continue;
      num_registered += n;
      for (int idx : registered_images_buf) {
        new_registered_image_indices.emplace_back(idx);
      }
      VLOG(1)
          << "[PERF] resection: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t_resect0).count()
          << "ms  added=" << new_registered_image_indices.size();
      if (VLOG_IS_ON(1)) {
        const PipelineReprojStats st_re =
            compute_pipeline_reproj_stats(*store_out, *poses_R_out, *poses_C_out, *registered_out,
                                          *cameras, image_to_camera_index);
        agent_log_pipeline_reproj_step("after_resection", sfm_iter, num_registered, st_re);
      }
      if (new_registered_image_indices.empty()) {
        break;
      }
      auto t_tri0 = Clock::now();
      new_track_ids_buf.clear();
      double triangle_error_thresh_px = opts.triangulation.commit_reproj_px;
      int n_new_tri = run_batch_triangulation(
          store_out, new_registered_image_indices, *poses_R_out, *poses_C_out, *registered_out,
          *cameras, image_to_camera_index, opts.triangulation.min_angle_deg, &new_track_ids_buf,
          triangle_error_thresh_px);
      add_ms(&ms_triangulation, t_tri0, Clock::now());
      all_new_track_ids.insert(all_new_track_ids.end(), new_track_ids_buf.begin(),
                               new_track_ids_buf.end());
      VLOG(1)
          << "[PERF] triangulation: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t_tri0).count()
          << "ms  new_tri=" << n_new_tri;

      if (VLOG_IS_ON(1)) {
        const PipelineReprojStats st_tri =
            compute_pipeline_reproj_stats(*store_out, *poses_R_out, *poses_C_out, *registered_out,
                                          *cameras, image_to_camera_index);
        agent_log_pipeline_reproj_step("after_triangulation", sfm_iter, num_registered, st_tri);
      }
      LOG(INFO) << "  After resection+triangulation: registered=" << num_registered
                << ", new_tri=" << n_new_tri << ", total_tri=" << count_tri_tracks();
      // In local BA mode, process exactly one image per outer SfM iteration so that
      // each newly registered image immediately gets a local BA pass before the next
      // candidate is attempted.  Global BA mode can batch multiple images per iteration,
      // except during the conservative early phase (early_phase_max_cameras > 0) where we
      // also limit to one image per iteration to give intrinsics time to converge.
      if (use_local_ba)
        break;
      if (opts.global_ba.early_phase_max_cameras > 0 &&
          num_registered < opts.global_ba.early_phase_max_cameras)
        break;
    }

    if (new_registered_image_indices.empty()) {
      // All candidates were found but every one failed PnP-RANSAC.
      // Apply the same BA + kFullScan rescue as the "no candidates" path so that
      // freshly registered cameras improve 3D coverage and unlock these images.
      LOG(INFO) << "  [resection_fail] All " << resection_candidates.size()
                << " candidate(s) failed PnP-RANSAC (iter=" << sfm_iter << ")";
      if (no_candidate_consecutive >= kMaxNoCandidateRetries) {
        LOG(INFO) << "  No progress after " << no_candidate_consecutive
                  << " consecutive retry(s), stopping";
        break;
      }
      ++no_candidate_consecutive;
      LOG(INFO) << "  [resection_fail] retry " << no_candidate_consecutive << "/"
                << kMaxNoCandidateRetries << ": running BA + kFullScan to rescue remaining images";

      // ── Retry BA ─────────────────────────────────────────────────────────
      double rmse_rescue = 0.0;
      auto t_rescue_ba = Clock::now();
      if (!run_ba_with_outlier_detection(
              store_out, poses_R_out, poses_C_out, *registered_out, image_to_camera_index, cameras,
              anchor_image, num_registered, opts, &rmse_rescue, static_cast<int>(*im1_ptr))) {
        LOG(WARNING) << "  [resection_fail_retry] BA failed (RMSE=" << rmse_rescue << " px)";
      } else {
        LOG(INFO) << "  [resection_fail_retry] BA ok (RMSE=" << rmse_rescue << " px), "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() -
                                                                           t_rescue_ba)
                         .count()
                  << "ms";
      }
      add_ms(&ms_global_ba, t_rescue_ba, Clock::now());

      // ── Retry kFullScan triangulation (permissive angle) ─────────────────
      {
        auto t_rescue_fs = Clock::now();
        IncrementalRetriangulationOptions fs_rescue;
        fs_rescue.scope = RetriangulationScope::kFullScan;
        fs_rescue.robust.min_tri_angle_deg = 0.5; // rescue mode: permissive angle
        fs_rescue.robust.max_tri_angle_deg = opts.triangulation.max_angle_deg;
        fs_rescue.full_scan_commit_reproj_px = opts.triangulation.commit_reproj_px;
        fs_rescue.restore_strict_reproj_px = opts.triangulation.restore_reproj_px;
        const int n_rescue_fs =
            run_retriangulation(store_out, *poses_R_out, *poses_C_out, *registered_out, *cameras,
                                image_to_camera_index, fs_rescue);
        LOG(INFO) << "  [resection_fail_retry] kFullScan: score=" << n_rescue_fs
                  << "  total_tri=" << count_tri_tracks() << "  "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() -
                                                                           t_rescue_fs)
                         .count()
                  << "ms";
        add_ms(&ms_retriangulation, t_rescue_fs, Clock::now());
      }

      // Invalidate score cache so choose_resection_candidates re-evaluates all images.
      resection_score_cache.invalidate_all();
      continue; // re-run choose_resection_candidates next iteration
    }
    no_candidate_consecutive = 0; // reset on successful resection
    double rmse = 0.0;
    // Scene normalization helper (shared between both branches).
    auto maybe_normalize = [&]() {
      const auto& sn = opts.scene_normalization;
      bool do_normalize = false;
      if (sn.normalize_scene_every_n_sfm_iters > 0 &&
          sfm_iter % sn.normalize_scene_every_n_sfm_iters == 0)
        do_normalize = true;
      if (sn.normalize_scene_every_n_registered_images > 0 &&
          num_registered >= sn.normalize_scene_every_n_registered_images &&
          num_registered % sn.normalize_scene_every_n_registered_images == 0)
        do_normalize = true;
      if (do_normalize)
        normalize_scene_median_tracks_and_poses(store_out, poses_C_out, *registered_out);
    };

    bool ran_global_ba_this_iter = false;
    if (!use_local_ba) {
      // ── Global-only phase ─────────────────────────────────────────────────
      ran_global_ba_this_iter = true;
      LOG(INFO) << "Running global BA (n=" << num_registered << ")";
      maybe_normalize();
      ba_cameras_snapshot = *cameras; // snapshot before BA for intrinsics-change restore
      auto t_ba0 = Clock::now();
      const bool ok_scheduled_global = run_ba_with_outlier_detection(
          store_out, poses_R_out, poses_C_out, *registered_out, image_to_camera_index, cameras,
          anchor_image, num_registered, opts, &rmse, static_cast<int>(*im1_ptr));
      if (!ok_scheduled_global) {
        LOG(ERROR) << "Global BA with outlier detection failed.";
      }
      if (ok_scheduled_global && opts.local_ba.enable &&
          opts.global_ba.mid_phase_global_linear_spacing &&
          (opts.global_ba.mid_global_spacing_a > 0.0 ||
           opts.global_ba.mid_global_spacing_b > 0.0) &&
          early_end > 0 && num_registered >= early_end && num_registered < mid_end) {
        bump_mid_global_milestone_linear(num_registered, opts.global_ba,
                                         &next_mid_global_registered);
      }
      add_ms(&ms_global_ba, t_ba0, Clock::now());
      VLOG(1) << "[PERF] global_ba: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t_ba0).count()
              << "ms  n=" << num_registered << "  RMSE=" << rmse << " px";
    } else {
      // ── Local BA phase ────────────────────────────────────────────────────
      LOG(INFO) << "Running local BA ( n=" << num_registered << ")";
      if (opts.local_ba.strategy == LocalBAStrategy::kColmap) {
        LOG(INFO) << "  Local BA strategy: COLMAP-style local window optimization";
      } else if (opts.local_ba.strategy == LocalBAStrategy::kBatchNeighbor) {
        LOG(INFO) << "  Local BA strategy: batch neighbor optimization (all registered images)";
      }
      auto t_lba0 = Clock::now();
      BASolverOverrides local_ov{};
      local_ov.num_threads = opts.global_ba.solver_overrides.num_threads;
      // Local BA warm-starts periodic / full global BA.  Never reuse global BA's optional loose
      // solver_overrides (e.g. intermediate rounds use function_tolerance≈1e-4): enforce Ceres
      // defaults explicitly so local steps converge tightly before the next global solve.
      local_ov.function_tolerance = 1e-6;
      local_ov.gradient_tolerance = 1e-10;
      local_ov.parameter_tolerance = 1e-8;
      const bool local_ba_ok =
          run_local_ba_dispatch(opts.local_ba, anchor_image, num_registered, store_out, poses_R_out,
                                poses_C_out, *registered_out, image_to_camera_index, *cameras,
                                new_registered_image_indices, all_new_track_ids, local_ov, &rmse);
      add_ms(&ms_local_ba, t_lba0, Clock::now());
      if (!local_ba_ok) {
        LOG(WARNING) << "Local BA failed; falling back to global BA";
        maybe_normalize();
        ran_global_ba_this_iter = true;
        auto t_fallback_gba0 = Clock::now();
        if (!run_ba_with_outlier_detection(
                store_out, poses_R_out, poses_C_out, *registered_out, image_to_camera_index,
                cameras, anchor_image, num_registered, opts, &rmse, static_cast<int>(*im1_ptr))) {
          LOG(ERROR) << "Fallback global BA also failed.";
        }
        add_ms(&ms_global_ba, t_fallback_gba0, Clock::now());
      }
      VLOG(1)
          << "[PERF] local_ba: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t_lba0).count()
          << "ms  n=" << num_registered << "  RMSE=" << rmse << " px";

      if (local_ba_ok && opts.triangulation.enable_post_local_ba_retriangulation &&
          !new_registered_image_indices.empty()) {
        auto t_rni = Clock::now();
        IncrementalRetriangulationOptions rni;
        rni.scope = RetriangulationScope::kNewImages;
        rni.restrict_image_indices = new_registered_image_indices;
        rni.robust.min_tri_angle_deg = opts.triangulation.min_angle_deg;
        rni.robust.max_tri_angle_deg = opts.triangulation.max_angle_deg;
        rni.robust.ransac_inlier_px = opts.triangulation.commit_reproj_px;
        rni.restore_strict_reproj_px = opts.triangulation.restore_reproj_px;
        const int n_rn = run_retriangulation(store_out, *poses_R_out, *poses_C_out, *registered_out,
                                             *cameras, image_to_camera_index, rni);
        add_ms(&ms_retriangulation, t_rni, Clock::now());
        VLOG(1)
            << "[PERF] post_local_retri: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t_rni).count()
            << "ms  score=" << n_rn << "  total_tri=" << count_tri_tracks();
      }

      // Periodic global BA in the local-BA phase (late cadence only: milestone-based linear spacing
      // or fixed n % periodic_every_n_images).
      const bool periodic_extra = periodic_global_ba_should_run_after_local(
          num_registered, in_late_phase, opts.global_ba, opts.local_ba.switch_after_n_images,
          &next_periodic_global_registered);
      if (periodic_extra) {
        if (opts.global_ba.periodic_global_linear_spacing &&
            (opts.global_ba.periodic_global_spacing_a > 0.0 ||
             opts.global_ba.periodic_global_spacing_b > 0.0)) {
          const double raw_lin =
              opts.global_ba.periodic_global_spacing_a +
              opts.global_ba.periodic_global_spacing_b * static_cast<double>(num_registered);
          const int gap_lin = std::max(1, static_cast<int>(std::ceil(raw_lin)));
          LOG(INFO) << "Running periodic global BA (n=" << num_registered
                    << ")  linear_gap=" << gap_lin
                    << "  spacing=a+b*n with a=" << opts.global_ba.periodic_global_spacing_a
                    << " b=" << opts.global_ba.periodic_global_spacing_b;
        } else {
          LOG(INFO) << "Running periodic global BA (n=" << num_registered
                    << ")  fixed_every_n=" << opts.global_ba.periodic_every_n_images;
        }
        ran_global_ba_this_iter = true;
        maybe_normalize();
        ba_cameras_snapshot = *cameras; // snapshot before periodic BA
        auto t_gba0 = Clock::now();
        const bool periodic_ok = run_ba_with_outlier_detection(
            store_out, poses_R_out, poses_C_out, *registered_out, image_to_camera_index, cameras,
            anchor_image, num_registered, opts, &rmse, static_cast<int>(*im1_ptr));
        if (!periodic_ok) {
          LOG(ERROR) << "Periodic global BA failed.";
        }
        add_ms(&ms_global_ba, t_gba0, Clock::now());
        VLOG(1)
            << "[PERF] periodic_global_ba: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t_gba0).count()
            << "ms  n=" << num_registered << "  RMSE=" << rmse << " px";
        if (periodic_ok && opts.global_ba.periodic_global_linear_spacing &&
            (opts.global_ba.periodic_global_spacing_a > 0.0 ||
             opts.global_ba.periodic_global_spacing_b > 0.0)) {
          bump_periodic_global_milestone_linear(num_registered, opts.global_ba,
                                                &next_periodic_global_registered);
        }
      }
    }
    // Intrinsics-change observation restore: if any camera's focal length changed significantly
    // during the global BA above, re-evaluate kRestorable deleted observations from that camera.
    // These are observations that were rejected by reject_outliers_multiview under wrong intrinsics
    // and may now have acceptable reprojection error under the updated intrinsics.
    // Uses the per-image obs index so only observations from affected cameras are scanned.
    if (opts.triangulation.enable_intrinsics_change_restore && !ba_cameras_snapshot.empty()) {
      const int n_cam_models = static_cast<int>(cameras->size());
      std::unordered_set<int> changed_cams;
      for (int c = 0; c < n_cam_models && c < static_cast<int>(ba_cameras_snapshot.size()); ++c) {
        const double prev_fx = ba_cameras_snapshot[static_cast<size_t>(c)].fx;
        const double cur_fx = (*cameras)[static_cast<size_t>(c)].fx;
        if (prev_fx > 1.0 && std::abs(cur_fx - prev_fx) / prev_fx >
                                 opts.triangulation.intrinsics_change_restore_threshold)
          changed_cams.insert(c);
      }
      if (!changed_cams.empty()) {
        auto t_restore0 = Clock::now();
        const int n_restored = restore_observations_from_cameras(
            store_out, *poses_R_out, *poses_C_out, *registered_out, *cameras, image_to_camera_index,
            changed_cams, opts.triangulation.restore_reproj_px);
        LOG(INFO) << "  Intrinsics-change restore: recovered " << n_restored << " obs from "
                  << changed_cams.size() << " changed camera(s) (thresh="
                  << opts.triangulation.intrinsics_change_restore_threshold * 100.0 << "%)";
        VLOG(1) << "[PERF] intrinsics_restore: "
                << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t_restore0)
                       .count()
                << "ms";
      }
    }

    if (ran_global_ba_this_iter && opts.triangulation.enable_full_scan_after_global_ba) {
      auto t_pfs = Clock::now();
      IncrementalRetriangulationOptions fs_opts;
      fs_opts.scope = RetriangulationScope::kFullScan;
      fs_opts.robust.min_tri_angle_deg = opts.triangulation.min_angle_deg;
      fs_opts.robust.max_tri_angle_deg = opts.triangulation.max_angle_deg;
      fs_opts.full_scan_commit_reproj_px = opts.triangulation.commit_reproj_px;
      fs_opts.restore_strict_reproj_px = opts.triangulation.restore_reproj_px;
      const int n_fs = run_retriangulation(store_out, *poses_R_out, *poses_C_out, *registered_out,
                                           *cameras, image_to_camera_index, fs_opts);
      add_ms(&ms_retriangulation, t_pfs, Clock::now());
      LOG(INFO) << "  Post-global-BA retriangulation (full_scan): score=" << n_fs
                << "  total_tri=" << count_tri_tracks();
      VLOG(1) << "[PERF] post_global_full_scan_retri: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t_pfs).count()
              << "ms";
    }

    // Periodic re-triangulation: recover tracks flagged by outlier rejection.
    // ── Periodic kPendingOnly re-triangulation ────────────────────────────────
    // Drains the pending queue (tracks flagged when BA outlier rejection deleted observations).
    // After Fix-2 (mark_observation_deleted_restorable), these tracks retain their observations
    // as kRestorable so triangulate_track_common(include_deleted_restorable=true) can succeed.
    if (num_registered > 3 && opts.triangulation.retriangulation_every_n_iters > 0 &&
        sfm_iter % opts.triangulation.retriangulation_every_n_iters == 0) {
      auto t_retri = Clock::now();
      IncrementalRetriangulationOptions retri_opts;
      retri_opts.scope = RetriangulationScope::kPendingOnly;
      retri_opts.robust.min_tri_angle_deg = opts.triangulation.min_angle_deg;
      retri_opts.robust.max_tri_angle_deg = opts.triangulation.max_angle_deg;
      retri_opts.restore_strict_reproj_px = 4.0;

      int n_retri = run_retriangulation(store_out, *poses_R_out, *poses_C_out, *registered_out,
                                        *cameras, image_to_camera_index, retri_opts);
      add_ms(&ms_retriangulation, t_retri, Clock::now());
      LOG(INFO)
          << "[PERF] periodic_retri: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t_retri).count()
          << "ms  re_tri=" << n_retri << "  total_tri=" << count_tri_tracks();
    }

    // ── Periodic kFullScan re-triangulation ───────────────────────────────────
    // Independent of global BA frequency.  Catches lag tracks: tracks visible in multiple
    // recently-registered cameras that couldn't be triangulated when those cameras were first
    // added (not enough views yet), but are now triangulable with the current pose set.
    //
    // Skip if we already ran a full_scan via enable_full_scan_after_global_ba this iteration,
    // to avoid doing two consecutive full scans in the same iter.
    if (!ran_global_ba_this_iter && opts.triangulation.full_scan_every_n_iters > 0 &&
        num_registered > 3 && sfm_iter % opts.triangulation.full_scan_every_n_iters == 0) {
      auto t_pfs2 = Clock::now();
      IncrementalRetriangulationOptions fs2_opts;
      fs2_opts.scope = RetriangulationScope::kFullScan;
      fs2_opts.robust.min_tri_angle_deg = opts.triangulation.min_angle_deg;
      fs2_opts.robust.max_tri_angle_deg = opts.triangulation.max_angle_deg;
      fs2_opts.full_scan_commit_reproj_px = opts.triangulation.commit_reproj_px;
      fs2_opts.restore_strict_reproj_px = opts.triangulation.restore_reproj_px;
      const int n_fs2 = run_retriangulation(store_out, *poses_R_out, *poses_C_out, *registered_out,
                                            *cameras, image_to_camera_index, fs2_opts);
      add_ms(&ms_retriangulation, t_pfs2, Clock::now());
      LOG(INFO) << "  Periodic full_scan retriangulation: score=" << n_fs2
                << "  total_tri=" << count_tri_tracks();
      VLOG(1)
          << "[PERF] periodic_full_scan_retri: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t_pfs2).count()
          << "ms";
    }

    // Debug snapshot callback — fires every N iters if configured.
    if (opts.debug.snapshot_every_n_iters > 0 &&
        sfm_iter % opts.debug.snapshot_every_n_iters == 0 && opts.debug.on_snapshot) {
      opts.debug.on_snapshot(sfm_iter, num_registered, *poses_R_out, *poses_C_out, *registered_out,
                             *store_out);
    }

    {
      const uint64_t d_ch = ms_choose_candidates - iter_ms_choose_t0;
      const uint64_t d_re = ms_resection - iter_ms_resect_t0;
      const uint64_t d_tr = ms_triangulation - iter_ms_tri_t0;
      const uint64_t d_lb = ms_local_ba - iter_ms_lba_t0;
      const uint64_t d_gb = ms_global_ba - iter_ms_gba_t0;
      const uint64_t d_rt = ms_retriangulation - iter_ms_retri_t0;
      const uint64_t d_tot = d_ch + d_re + d_tr + d_lb + d_gb + d_rt;
      LOG(INFO) << "[timing][iter] iter=" << sfm_iter << "  n_reg=" << num_registered
                << "  total=" << d_tot << "ms"
                << "  choose=" << d_ch << "ms"
                << "  resect=" << d_re << "ms"
                << "  tri=" << d_tr << "ms"
                << "  lba=" << d_lb << "ms"
                << "  gba=" << d_gb << "ms"
                << "  retri=" << d_rt << "ms";
    }
  }
  VLOG(1) << "[PERF] main_loop total: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - pipeline_start)
                 .count()
          << "ms  iters=" << sfm_iter;
  {
    const uint64_t ms_pipe_total = ms_choose_candidates + ms_resection + ms_triangulation +
                                   ms_local_ba + ms_global_ba + ms_retriangulation;
    const auto pct = [&](uint64_t ms) -> int {
      return ms_pipe_total > 0 ? static_cast<int>(100.0 * static_cast<double>(ms) /
                                                  static_cast<double>(ms_pipe_total))
                               : 0;
    };
    LOG(INFO) << "[timing][pipeline] total=" << ms_pipe_total
              << "ms  choose=" << ms_choose_candidates << "ms(" << pct(ms_choose_candidates)
              << "%) resection=" << ms_resection << "ms(" << pct(ms_resection)
              << "%) triangulation=" << ms_triangulation << "ms(" << pct(ms_triangulation)
              << "%) local_ba=" << ms_local_ba << "ms(" << pct(ms_local_ba)
              << "%) global_ba=" << ms_global_ba << "ms(" << pct(ms_global_ba)
              << "%) retriangulation=" << ms_retriangulation << "ms(" << pct(ms_retriangulation)
              << "%)";
  }

  // Drain any remaining pending-queue tracks before final BA.
  IncrementalRetriangulationOptions retri_opts_pre;
  retri_opts_pre.scope = RetriangulationScope::kPendingOnly;
  retri_opts_pre.robust.min_tri_angle_deg = opts.triangulation.min_angle_deg;
  retri_opts_pre.robust.max_tri_angle_deg = opts.triangulation.max_angle_deg;
  retri_opts_pre.restore_strict_reproj_px = 4.0;
  run_retriangulation(store_out, *poses_R_out, *poses_C_out, *registered_out, *cameras,
                      image_to_camera_index, retri_opts_pre);
  LOG(INFO) << "Pre-final-BA retriangulation done. total_tri=" << count_tri_tracks();

  LOG(INFO) << "Running final global BA...";
  double rmse = 0.0;
  run_ba_with_outlier_detection(store_out, poses_R_out, poses_C_out, *registered_out,
                                image_to_camera_index, cameras, anchor_image, num_registered, opts,
                                &rmse, static_cast<int>(*im1_ptr));
  LOG(INFO) << "Final BA RMSE=" << rmse << " px";

  // ── Post-BA cleanup ───────────────────────────────────────────────────────
  // No kFullScan retriangulation here: final BA already rejected outliers at a tight threshold
  // (~4 px); a kFullScan pass would attempt to re-triangulate those same cleared tracks at a
  // looser commit_reproj_px (16 px default), causing tug-of-war.
  // kSkipFromBA track refinement (fixed-pose Ceres) is performed internally by
  // run_ba_with_outlier_detection; calling it again here is redundant.
// ── Post-BA cleanup ───────────────────────────────────────────────────────
  // No kFullScan retriangulation here: final BA already rejected outliers at a tight threshold
  // (~4 px); a kFullScan pass would attempt to re-triangulate those same cleared tracks at a
  // looser commit_reproj_px (16 px default), causing tug-of-war.  Instead we only:
  //   (a) refine kSkipFromBA track positions with fixed-pose Ceres (they never entered BA),
  //   (b) apply the same strict outlier rejection to any newly refined points.
  if (opts.global_ba.ba_fixed_pose_optimize_skipped &&
      (opts.global_ba.ba_grid_subset || opts.global_ba.skip_2degree_tracks)) {
    double skip_rmse = 0.0;
    retri_skipped_tracks_fixed_pose(store_out, *poses_R_out, *poses_C_out, *registered_out,
                                    image_to_camera_index, *cameras,
                                    opts.global_ba.ba_fixed_pose_max_iterations, &skip_rmse,
                                    opts.global_ba.solver_overrides.num_threads);
    LOG(INFO) << "Post-final fixed-pose BA (skipped tracks): RMSE=" << skip_rmse << " px";
  }

  {
    const double strict_thr = opts.outlier.threshold_px;
    int rej = 0;
    rej += reject_outliers_multiview(store_out, *poses_R_out, *poses_C_out, *registered_out,
                                     *cameras, image_to_camera_index, strict_thr);
    rej += reject_outliers_angle_multiview(store_out, *poses_R_out, *poses_C_out, *registered_out,
                                           opts.outlier.min_angle_deg, opts.outlier.max_angle_deg);
    rej += reject_outliers_depth(store_out, *poses_R_out, *poses_C_out, *registered_out,
                                 opts.outlier.max_depth_factor);
    LOG(INFO) << "Post-final outlier cleanup: strict_thr=" << strict_thr << " px  rejected=" << rej
              << "  total_tri=" << count_tri_tracks();
  }

  // ── Diagnostic: report unregistered images ───────────────────────────────
  // For each image that was never registered, log:
  //   - 3D-2D match count (how many triangulated tracks it observes)
  //   - All ViewGraph pairs it has, with both neighbors' registration status,
  //     F/E/twoview quality so we can judge if the root cause is bad matches.
  {
    const int n_total = store_out->num_images();
    std::vector<int> unregistered_imgs;
    unregistered_imgs.reserve(static_cast<size_t>(n_total));
    for (int i = 0; i < n_total; ++i) {
      if (!(*registered_out)[static_cast<size_t>(i)])
        unregistered_imgs.push_back(i);
    }
    if (unregistered_imgs.empty()) {
      LOG(INFO) << "[unregistered_diag] All " << n_total << " images registered.";
    } else {
      LOG(INFO) << "[unregistered_diag] " << unregistered_imgs.size() << "/" << n_total
                << " images NOT registered.";
      if (VLOG_IS_ON(1)) {
        // Expensive deep diagnostics are only enabled at verbose level.
        std::vector<int> match_3d2d(static_cast<size_t>(n_total), 0);
        const int n_tracks = static_cast<int>(store_out->num_tracks());
        for (int t = 0; t < n_tracks; ++t) {
          if (!store_out->is_track_valid(t) || !store_out->track_has_triangulated_xyz(t))
            continue;
          const auto& track_obs_ids = store_out->track_all_obs_ids_view(t);
          for (int obs_id : track_obs_ids) {
            if (!store_out->is_obs_valid(obs_id))
              continue;
            const int image_index = static_cast<int>(store_out->obs_image_index(obs_id));
            if (image_index >= 0 && image_index < n_total)
              match_3d2d[static_cast<size_t>(image_index)]++;
          }
        }
        VLOG(1) << "[unregistered_diag] verbose pair-wise diagnostics:";
        for (int im : unregistered_imgs) {
          VLOG(1) << "  img=" << im << "  3d2d_obs=" << match_3d2d[static_cast<size_t>(im)];
          int n_pairs = 0;
          int n_F_ok = 0, n_E_ok = 0, n_twoview_ok = 0, n_stable = 0;
          int max_F_inliers = 0, max_valid_pts = 0;
          for (size_t pi = 0; pi < view_graph.num_pairs(); ++pi) {
            const PairGeoInfo& pg = view_graph.pair_at(pi);
            const int other =
                (static_cast<int>(pg.image1_index) == im)
                    ? static_cast<int>(pg.image2_index)
                    : (static_cast<int>(pg.image2_index) == im ? static_cast<int>(pg.image1_index)
                                                               : -1);
            if (other < 0)
              continue;
            ++n_pairs;
            if (pg.F_ok)
              ++n_F_ok;
            if (pg.E_ok)
              ++n_E_ok;
            if (pg.twoview_ok)
              ++n_twoview_ok;
            if (pg.stable)
              ++n_stable;
            max_F_inliers = std::max(max_F_inliers, pg.F_inliers);
            max_valid_pts = std::max(max_valid_pts, pg.num_valid_points);
            const bool other_registered =
                other < n_total && (*registered_out)[static_cast<size_t>(other)];
            VLOG(1) << "    pair(" << im << "," << other << ")"
                    << "  F_ok=" << pg.F_ok << "  F_inliers=" << pg.F_inliers
                    << "  E_ok=" << pg.E_ok << "  twoview_ok=" << pg.twoview_ok
                    << "  stable=" << pg.stable << "  valid_pts=" << pg.num_valid_points
                    << "  degenerate=" << pg.is_degenerate
                    << "  other_registered=" << other_registered;
          }
          VLOG(1) << "  img=" << im << "  summary: pairs=" << n_pairs << "  F_ok=" << n_F_ok
                  << "  E_ok=" << n_E_ok << "  twoview_ok=" << n_twoview_ok
                  << "  stable=" << n_stable << "  max_F_inliers=" << max_F_inliers
                  << "  max_valid_pts=" << max_valid_pts;
        }
      }
    }
  }

  return true;
}

} // namespace sfm
} // namespace insight
