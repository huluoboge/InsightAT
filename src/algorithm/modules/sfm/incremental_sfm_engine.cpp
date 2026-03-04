/**
 * @file  incremental_sfm_engine.cpp
 * @brief Main orchestration for incremental SfM pipeline.
 */

#include "incremental_sfm_engine.h"
#include "bundle_adjustment.h"
#include "incremental_sfm.h"
#include "incremental_sfm_helpers.h"
#include "track_store.h"
#include <glog/logging.h>

namespace insight {
namespace sfm {

namespace {

double get_fx(const IncrementalSfmConfig& config) {
  return config.intrinsics ? config.intrinsics->fx : config.fx;
}
double get_fy(const IncrementalSfmConfig& config) {
  return config.intrinsics ? config.intrinsics->fy : config.fy;
}
double get_cx(const IncrementalSfmConfig& config) {
  return config.intrinsics ? config.intrinsics->cx : config.cx;
}
double get_cy(const IncrementalSfmConfig& config) {
  return config.intrinsics ? config.intrinsics->cy : config.cy;
}

camera::Intrinsics get_shared_intrinsics(const IncrementalSfmConfig& config) {
  camera::Intrinsics K;
  K.fx = get_fx(config);
  K.fy = get_fy(config);
  K.cx = get_cx(config);
  K.cy = get_cy(config);
  if (config.intrinsics) {
    K.k1 = config.intrinsics->k1;
    K.k2 = config.intrinsics->k2;
    K.k3 = config.intrinsics->k3;
    K.p1 = config.intrinsics->p1;
    K.p2 = config.intrinsics->p2;
  }
  return K;
}

/// Build intrinsics_per_image from config + image_ids. Size = n_images; fallback to shared.
std::vector<camera::Intrinsics> build_intrinsics_per_image(
    const IncrementalSfmConfig& config, int n_images,
    const std::vector<uint32_t>& image_ids) {
  std::vector<camera::Intrinsics> out(static_cast<size_t>(n_images), get_shared_intrinsics(config));
  if (!config.intrinsics_by_image_id || image_ids.size() < static_cast<size_t>(n_images))
    return out;
  for (int i = 0; i < n_images; ++i) {
    const size_t idx = static_cast<size_t>(i);
    if (idx >= image_ids.size())
      continue;
    auto it = config.intrinsics_by_image_id->find(image_ids[idx]);
    if (it != config.intrinsics_by_image_id->end())
      out[idx] = it->second;
  }
  return out;
}

} // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Stage implementations
// ─────────────────────────────────────────────────────────────────────────────

bool run_stage_initial_pair(const IncrementalSfmConfig& config, TrackStore* store_out,
                             Eigen::Matrix3d* R1_out, Eigen::Vector3d* t1_out,
                             uint32_t* image1_id_out, uint32_t* image2_id_out) {
  if (!store_out || !R1_out || !t1_out)
    return false;
  if (config.intrinsics && config.intrinsics->fx > 0) {
    return run_initial_pair_loop(config.pairs_json_path, config.geo_dir,
                                  config.match_dir, *config.intrinsics, store_out,
                                  R1_out, t1_out, config.min_tracks_after_initial,
                                  image1_id_out, image2_id_out);
  }
  return run_initial_pair_loop(config.pairs_json_path, config.geo_dir,
                               config.match_dir, config.fx, config.fy, config.cx,
                               config.cy, store_out, R1_out, t1_out,
                               config.min_tracks_after_initial, image1_id_out,
                               image2_id_out);
}

int run_stage_resection_loop(const IncrementalSfmConfig& config, TrackStore* store,
                             std::vector<Eigen::Matrix3d>* poses_R,
                             std::vector<Eigen::Vector3d>* poses_t,
                             std::vector<bool>* registered,
                             const std::vector<uint32_t>* image_ids) {
  if (!store || !poses_R || !poses_t || !registered)
    return 0;
  const int n_images = store->num_images();
  if (config.intrinsics_by_image_id && image_ids && !image_ids->empty()) {
    std::vector<camera::Intrinsics> per_image =
        build_intrinsics_per_image(config, n_images, *image_ids);
    return run_resection_loop(store, poses_R, poses_t, registered, &per_image,
                              get_fx(config), get_fy(config), get_cx(config),
                              get_cy(config), config.min_correspondences_resection);
  }
  if (config.intrinsics && config.intrinsics->fx > 0) {
    return run_resection_loop(store, poses_R, poses_t, registered, *config.intrinsics,
                               config.min_correspondences_resection);
  }
  return run_resection_loop(store, poses_R, poses_t, registered, config.fx,
                            config.fy, config.cx, config.cy,
                            config.min_correspondences_resection);
}

int run_stage_reject_outliers(TrackStore* store,
                              const std::vector<Eigen::Matrix3d>& poses_R,
                              const std::vector<Eigen::Vector3d>& poses_t,
                              const std::vector<bool>& registered, double fx, double fy,
                              double cx, double cy, double threshold_px) {
  if (!store)
    return 0;
  return reject_outliers_multiview(store, poses_R, poses_t, registered, fx, fy,
                                   cx, cy, threshold_px);
}

int run_stage_reject_outliers(TrackStore* store,
                              const std::vector<Eigen::Matrix3d>& poses_R,
                              const std::vector<Eigen::Vector3d>& poses_t,
                              const std::vector<bool>& registered,
                              const camera::Intrinsics& K, double threshold_px) {
  if (!store)
    return 0;
  return reject_outliers_multiview(store, poses_R, poses_t, registered, K,
                                   threshold_px);
}

int run_stage_reject_outliers(TrackStore* store,
                              const std::vector<Eigen::Matrix3d>& poses_R,
                              const std::vector<Eigen::Vector3d>& poses_t,
                              const std::vector<bool>& registered,
                              const std::vector<camera::Intrinsics>* intrinsics_per_image,
                              double fx, double fy, double cx, double cy,
                              double threshold_px) {
  if (!store)
    return 0;
  if (intrinsics_per_image && intrinsics_per_image->size() == poses_R.size())
    return reject_outliers_multiview(store, poses_R, poses_t, registered,
                                     *intrinsics_per_image, threshold_px);
  return reject_outliers_multiview(store, poses_R, poses_t, registered, fx, fy,
                                   cx, cy, threshold_px);
}

int run_stage_filter_tracks(TrackStore* store,
                            const std::vector<Eigen::Matrix3d>& poses_R,
                            const std::vector<Eigen::Vector3d>& poses_t,
                            const std::vector<bool>& registered, int min_observations,
                            double min_angle_deg) {
  if (!store)
    return 0;
  return filter_tracks_multiview(store, poses_R, poses_t, registered,
                                 min_observations, min_angle_deg);
}

bool run_stage_global_ba(const TrackStore& store,
                         const std::vector<Eigen::Matrix3d>& poses_R,
                         const std::vector<Eigen::Vector3d>& poses_t, double fx,
                         double fy, double cx, double cy, int max_iterations,
                         std::vector<Eigen::Matrix3d>* poses_R_out,
                         std::vector<Eigen::Vector3d>* poses_t_out,
                         TrackStore* store_out, double* rmse_px_out) {
  if (!poses_R_out || !poses_t_out)
    return false;
  if (poses_R.size() != poses_t.size() || poses_R.empty())
    return false;

  GlobalBAInput ba_in;
  ba_in.poses_R = poses_R;
  ba_in.poses_t = poses_t;
  ba_in.fx = fx;
  ba_in.fy = fy;
  ba_in.cx = cx;
  ba_in.cy = cy;

  const size_t n_tracks = store.num_tracks();
  std::vector<int> valid_track_ids;
  for (size_t ti = 0; ti < n_tracks; ++ti) {
    const int tid = static_cast<int>(ti);
    if (!store.is_track_valid(tid) || !store.track_has_triangulated_xyz(tid))
      continue;
    float tx, ty, tz;
    store.get_track_xyz(tid, &tx, &ty, &tz);
    ba_in.points3d.emplace_back(tx, ty, tz);
    valid_track_ids.push_back(tid);
  }

  std::vector<Observation> obs_buf;
  for (size_t i = 0; i < valid_track_ids.size(); ++i) {
    const int tid = valid_track_ids[i];
    obs_buf.clear();
    store.get_track_observations(tid, &obs_buf);
    const int pt_idx = static_cast<int>(i);
    for (const auto& o : obs_buf) {
      ba_in.observations.push_back(
          {static_cast<int>(o.image_index), pt_idx, static_cast<double>(o.u),
           static_cast<double>(o.v)});
    }
  }

  if (ba_in.points3d.empty() || ba_in.observations.empty()) {
    *poses_R_out = poses_R;
    *poses_t_out = poses_t;
    if (rmse_px_out)
      *rmse_px_out = 0.0;
    return false;
  }

#if defined(INSIGHTAT_USE_CERES) && INSIGHTAT_USE_CERES
  GlobalBAResult ba_out;
  if (!global_bundle(ba_in, &ba_out, max_iterations)) {
    *poses_R_out = poses_R;
    *poses_t_out = poses_t;
    if (rmse_px_out)
      *rmse_px_out = 0.0;
    return false;
  }
  *poses_R_out = std::move(ba_out.poses_R);
  *poses_t_out = std::move(ba_out.poses_t);
  if (rmse_px_out)
    *rmse_px_out = ba_out.rmse_px;
  if (store_out && ba_out.points3d.size() == valid_track_ids.size()) {
    for (size_t i = 0; i < valid_track_ids.size(); ++i)
      store_out->set_track_xyz(
          valid_track_ids[i],
          static_cast<float>(ba_out.points3d[i](0)),
          static_cast<float>(ba_out.points3d[i](1)),
          static_cast<float>(ba_out.points3d[i](2)));
  }
  return true;
#else
  (void)max_iterations;
  (void)store_out;
  (void)valid_track_ids;
  *poses_R_out = poses_R;
  *poses_t_out = poses_t;
  if (rmse_px_out)
    *rmse_px_out = 0.0;
  return false;
#endif
}

bool run_stage_global_ba(const TrackStore& store,
                         const std::vector<Eigen::Matrix3d>& poses_R,
                         const std::vector<Eigen::Vector3d>& poses_t,
                         const std::vector<camera::Intrinsics>* intrinsics_per_image,
                         double fx, double fy, double cx, double cy, int max_iterations,
                         std::vector<Eigen::Matrix3d>* poses_R_out,
                         std::vector<Eigen::Vector3d>* poses_t_out,
                         TrackStore* store_out, double* rmse_px_out) {
  if (!poses_R_out || !poses_t_out)
    return false;
  if (poses_R.size() != poses_t.size() || poses_R.empty())
    return false;

  GlobalBAInput ba_in;
  ba_in.poses_R = poses_R;
  ba_in.poses_t = poses_t;
  ba_in.fx = fx;
  ba_in.fy = fy;
  ba_in.cx = cx;
  ba_in.cy = cy;
  const bool use_per_camera =
      intrinsics_per_image &&
      intrinsics_per_image->size() == poses_R.size();
  if (use_per_camera) {
    ba_in.fx_per_camera.resize(poses_R.size());
    ba_in.fy_per_camera.resize(poses_R.size());
    ba_in.cx_per_camera.resize(poses_R.size());
    ba_in.cy_per_camera.resize(poses_R.size());
    for (size_t i = 0; i < poses_R.size(); ++i) {
      ba_in.fx_per_camera[i] = (*intrinsics_per_image)[i].fx;
      ba_in.fy_per_camera[i] = (*intrinsics_per_image)[i].fy;
      ba_in.cx_per_camera[i] = (*intrinsics_per_image)[i].cx;
      ba_in.cy_per_camera[i] = (*intrinsics_per_image)[i].cy;
    }
  }

  const size_t n_tracks = store.num_tracks();
  std::vector<int> valid_track_ids;
  for (size_t ti = 0; ti < n_tracks; ++ti) {
    const int tid = static_cast<int>(ti);
    if (!store.is_track_valid(tid) || !store.track_has_triangulated_xyz(tid))
      continue;
    float tx, ty, tz;
    store.get_track_xyz(tid, &tx, &ty, &tz);
    ba_in.points3d.emplace_back(tx, ty, tz);
    valid_track_ids.push_back(tid);
  }

  std::vector<Observation> obs_buf;
  for (size_t i = 0; i < valid_track_ids.size(); ++i) {
    const int tid = valid_track_ids[i];
    obs_buf.clear();
    store.get_track_observations(tid, &obs_buf);
    const int pt_idx = static_cast<int>(i);
    for (const auto& o : obs_buf) {
      ba_in.observations.push_back(
          {static_cast<int>(o.image_index), pt_idx, static_cast<double>(o.u),
           static_cast<double>(o.v)});
    }
  }

  if (ba_in.points3d.empty() || ba_in.observations.empty()) {
    *poses_R_out = poses_R;
    *poses_t_out = poses_t;
    if (rmse_px_out)
      *rmse_px_out = 0.0;
    return false;
  }

#if defined(INSIGHTAT_USE_CERES) && INSIGHTAT_USE_CERES
  GlobalBAResult ba_out;
  if (!global_bundle(ba_in, &ba_out, max_iterations)) {
    *poses_R_out = poses_R;
    *poses_t_out = poses_t;
    if (rmse_px_out)
      *rmse_px_out = 0.0;
    return false;
  }
  *poses_R_out = std::move(ba_out.poses_R);
  *poses_t_out = std::move(ba_out.poses_t);
  if (rmse_px_out)
    *rmse_px_out = ba_out.rmse_px;
  if (store_out && ba_out.points3d.size() == valid_track_ids.size()) {
    for (size_t i = 0; i < valid_track_ids.size(); ++i)
      store_out->set_track_xyz(
          valid_track_ids[i],
          static_cast<float>(ba_out.points3d[i](0)),
          static_cast<float>(ba_out.points3d[i](1)),
          static_cast<float>(ba_out.points3d[i](2)));
  }
  return true;
#else
  (void)max_iterations;
  (void)store_out;
  (void)valid_track_ids;
  (void)intrinsics_per_image;
  *poses_R_out = poses_R;
  *poses_t_out = poses_t;
  if (rmse_px_out)
    *rmse_px_out = 0.0;
  return false;
#endif
}

// ─────────────────────────────────────────────────────────────────────────────
// Main entry
// ─────────────────────────────────────────────────────────────────────────────

bool run_incremental_reconstruction(const IncrementalSfmConfig& config,
                                    IncrementalSfmResult* result) {
  if (!result)
    return false;

  result->success = false;
  result->n_registered = 0;
  result->final_rmse_px = 0.0;

  // ── Stage 1: Initial pair ───────────────────────────────────────────────
  TrackStore store;
  Eigen::Matrix3d R1;
  Eigen::Vector3d t1;
  uint32_t image1_id = 0, image2_id = 0;
  if (!run_stage_initial_pair(config, &store, &R1, &t1, &image1_id, &image2_id)) {
    LOG(ERROR) << "Incremental SfM: initial pair stage failed";
    return false;
  }

  result->image1_id = image1_id;
  result->image2_id = image2_id;
  result->image_ids = {image1_id, image2_id};

  const int n_images = store.num_images();
  result->store = std::move(store);
  result->poses_R.resize(static_cast<size_t>(n_images));
  result->poses_t.resize(static_cast<size_t>(n_images));
  result->registered.resize(static_cast<size_t>(n_images), false);

  result->poses_R[0] = Eigen::Matrix3d::Identity();
  result->poses_t[0] = Eigen::Vector3d::Zero();
  result->poses_R[1] = R1;
  result->poses_t[1] = t1;
  result->registered[0] = true;
  result->registered[1] = true;
  result->n_registered = 2;

  // ── Stage 2: Resection loop (only if n_images >= 3) ──────────────────────
  if (n_images >= 3) {
    const int added =
        run_stage_resection_loop(config, &result->store, &result->poses_R,
                                 &result->poses_t, &result->registered,
                                 &result->image_ids);
    result->n_registered += added;
    LOG(INFO) << "Incremental SfM: resection added " << added << " images";
  } else {
    LOG(INFO) << "Incremental SfM: only 2 images, skipping resection loop";
  }

  // ── Stage 3: Reject outliers (multiview) ──────────────────────────────────
  const double fx = get_fx(config);
  const double fy = get_fy(config);
  const double cx = get_cx(config);
  const double cy = get_cy(config);
  std::vector<camera::Intrinsics> intrinsics_per_image =
      build_intrinsics_per_image(config, result->store.num_images(), result->image_ids);
  run_stage_reject_outliers(&result->store, result->poses_R, result->poses_t,
                            result->registered, &intrinsics_per_image, fx, fy,
                            cx, cy, config.outlier_threshold_px);

  // ── Stage 4: Filter tracks (multiview) ────────────────────────────────────
  run_stage_filter_tracks(&result->store, result->poses_R, result->poses_t,
                          result->registered, config.min_observations_per_track,
                          config.min_track_angle_deg);

  // ── Stage 5: Global BA ────────────────────────────────────────────────────
  if (config.run_global_ba && result->n_registered >= 2) {
    std::vector<Eigen::Matrix3d> ba_R;
    std::vector<Eigen::Vector3d> ba_t;
    double rmse = 0.0;
    bool ba_ok = run_stage_global_ba(result->store, result->poses_R, result->poses_t,
                                    &intrinsics_per_image, fx, fy, cx, cy,
                                    config.global_ba_max_iterations, &ba_R,
                                    &ba_t, &result->store, &rmse);
    if (!ba_ok)
      ba_ok = run_stage_global_ba(result->store, result->poses_R, result->poses_t,
                                  fx, fy, cx, cy, config.global_ba_max_iterations,
                                  &ba_R, &ba_t, &result->store, &rmse);
    if (ba_ok) {
      result->poses_R = std::move(ba_R);
      result->poses_t = std::move(ba_t);
      result->final_rmse_px = rmse;
      LOG(INFO) << "Incremental SfM: Global BA done, RMSE=" << rmse << " px";
    } else {
      LOG(WARNING) << "Incremental SfM: Global BA skipped or failed (Ceres may be disabled)";
    }
  }

  result->success = result->n_registered >= 2;
  return result->success;
}

} // namespace sfm
} // namespace insight
