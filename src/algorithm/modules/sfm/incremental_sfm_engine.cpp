/**
 * @file  incremental_sfm_engine.cpp
 * @brief Main orchestration for incremental SfM pipeline.
 */

#include "incremental_sfm_engine.h"
#include "bundle_adjustment_analytic.h"
#include "bundle_adjustment_analytic.h"
#include "full_track_builder.h"
#include "incremental_sfm.h"
#include "incremental_sfm_helpers.h"
#include "track_store.h"
#include <glog/logging.h>

namespace insight {
namespace sfm {

namespace {

/// Per-image intrinsics (size = n_images). Index i → cameras.for_image_index(i).
std::vector<camera::Intrinsics> build_intrinsics_per_image(
    const MultiCameraSetup& cameras, int n_images) {
  const camera::Intrinsics* ref = cameras.reference();
  camera::Intrinsics fallback = ref ? *ref : camera::Intrinsics{};
  std::vector<camera::Intrinsics> out(static_cast<size_t>(n_images), fallback);
  for (int i = 0; i < n_images; ++i) {
    const camera::Intrinsics* K = cameras.for_image_index(i);
    if (K)
      out[static_cast<size_t>(i)] = *K;
  }
  return out;
}

/// Build camera assignment for GlobalBA. image_to_camera[i] = camera index.
void build_camera_assignment(const MultiCameraSetup& cam_setup, int n_images,
                             std::vector<int>* image_camera_index_out,
                             std::vector<camera::Intrinsics>* cameras_out) {
  image_camera_index_out->assign(static_cast<size_t>(n_images), 0);
  for (int i = 0; i < n_images && i < cam_setup.num_images(); ++i) {
    int c = cam_setup.image_to_camera[static_cast<size_t>(i)];
    if (c >= 0 && c < cam_setup.num_cameras())
      (*image_camera_index_out)[static_cast<size_t>(i)] = c;
  }
  *cameras_out = cam_setup.cameras;
}

} // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Stage implementations
// ─────────────────────────────────────────────────────────────────────────────

bool run_stage_initial_pair(const MultiCameraSetup& cameras,
                             const IncrementalSfmConfig& config,
                             TrackStore* store_in_out, Eigen::Matrix3d* R1_out,
                             Eigen::Vector3d* C1_out,
                             uint32_t* image1_index_out, uint32_t* image2_index_out) {
  if (!store_in_out || !R1_out || !C1_out || !config.id_mapping)
    return false;
  return run_initial_pair_loop(config.pairs_json_path, config.geo_dir, config.match_dir,
                               &cameras, config.id_mapping, store_in_out,
                               R1_out, C1_out, config.min_tracks_after_initial,
                               image1_index_out, image2_index_out);
}

int run_stage_resection_loop(const MultiCameraSetup& cameras,
                             const IncrementalSfmConfig& config,
                             TrackStore* store,
                             std::vector<Eigen::Matrix3d>* poses_R,
                             std::vector<Eigen::Vector3d>* poses_C,
                             std::vector<bool>* registered) {
  if (!store || !poses_R || !poses_C || !registered)
    return 0;
  const int n_images = store->num_images();
  const camera::Intrinsics* ref = cameras.reference();
  camera::Intrinsics fallback = ref ? *ref : camera::Intrinsics{};
  std::vector<camera::Intrinsics> per_image = build_intrinsics_per_image(cameras, n_images);
  return run_resection_loop(store, poses_R, poses_C, registered, &per_image,
                            fallback.fx, fallback.fy, fallback.cx, fallback.cy,
                            config.min_correspondences_resection);
}

int run_stage_reject_outliers(TrackStore* store,
                              const std::vector<Eigen::Matrix3d>& poses_R,
                              const std::vector<Eigen::Vector3d>& poses_C,
                              const std::vector<bool>& registered,
                              const std::vector<camera::Intrinsics>& intrinsics_per_image,
                              double threshold_px) {
  if (!store)
    return 0;
  if (intrinsics_per_image.size() == poses_R.size())
    return reject_outliers_multiview(store, poses_R, poses_C, registered,
                                     intrinsics_per_image, threshold_px);
  return 0;
}

int run_stage_filter_tracks(TrackStore* store,
                            const std::vector<Eigen::Matrix3d>& poses_R,
                            const std::vector<Eigen::Vector3d>& poses_C,
                            const std::vector<bool>& registered,
                            int min_observations, double min_angle_deg) {
  if (!store)
    return 0;
  return filter_tracks_multiview(store, poses_R, poses_C, registered,
                                 min_observations, min_angle_deg);
}

bool run_stage_global_ba(const MultiCameraSetup& cameras,
                          bool optimize_intrinsics, int max_iterations,
                          const TrackStore& store,
                          const std::vector<Eigen::Matrix3d>& poses_R,
                          const std::vector<Eigen::Vector3d>& poses_C,
                          std::vector<Eigen::Matrix3d>* poses_R_out,
                          std::vector<Eigen::Vector3d>* poses_C_out,
                          TrackStore* store_out,
                          std::vector<int>* camera_index_for_image_out,
                          std::vector<camera::Intrinsics>* cameras_out,
                          double* rmse_px_out) {
  if (!poses_R_out || !poses_C_out)
    return false;
  if (poses_R.size() != poses_C.size() || poses_R.empty())
    return false;

  const int n_images = static_cast<int>(poses_R.size());

  // Build camera assignment: same camera_id → same analytic parameter block
  std::vector<int> cam_index;
  std::vector<camera::Intrinsics> cam_models;
  build_camera_assignment(cameras, n_images, &cam_index, &cam_models);

  if (camera_index_for_image_out)
    *camera_index_for_image_out = cam_index;
  if (cameras_out)
    *cameras_out = cam_models;

  // Collect valid triangulated tracks
  BAInput ba_in;
  ba_in.poses_R = poses_R;
  ba_in.poses_C = poses_C;
  ba_in.image_camera_index = cam_index;
  ba_in.cameras = cam_models;
  ba_in.optimize_intrinsics = optimize_intrinsics;

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
          {static_cast<int>(o.image_index), pt_idx,
           static_cast<double>(o.u), static_cast<double>(o.v)});
    }
  }

  if (ba_in.points3d.empty() || ba_in.observations.empty()) {
    *poses_R_out = poses_R;
    *poses_C_out = poses_C;
    if (rmse_px_out)
      *rmse_px_out = 0.0;
    return false;
  }

  BAResult ba_out;
  if (!global_bundle_analytic(ba_in, &ba_out, max_iterations) || !ba_out.success) {
    *poses_R_out = poses_R;
    *poses_C_out = poses_C;
    if (rmse_px_out)
      *rmse_px_out = 0.0;
    return false;
  }
  *poses_R_out = std::move(ba_out.poses_R);
  *poses_C_out = std::move(ba_out.poses_C);
  if (rmse_px_out)
    *rmse_px_out = ba_out.rmse_px;
  if (store_out && ba_out.points3d.size() == valid_track_ids.size()) {
    for (size_t i = 0; i < valid_track_ids.size(); ++i)
      store_out->set_track_xyz(valid_track_ids[i],
                               static_cast<float>(ba_out.points3d[i](0)),
                               static_cast<float>(ba_out.points3d[i](1)),
                               static_cast<float>(ba_out.points3d[i](2)));
  }
  if (cameras_out && !ba_out.cameras.empty())
    *cameras_out = std::move(ba_out.cameras);
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Main entry
// ─────────────────────────────────────────────────────────────────────────────

bool run_incremental_reconstruction(const MultiCameraSetup& cameras,
                                    const IncrementalSfmConfig& config,
                                    IncrementalSfmResult* result) {
  if (!result)
    return false;
  if (cameras.empty()) {
    LOG(ERROR) << "Incremental SfM: empty camera setup";
    return false;
  }

  result->success = false;
  result->n_registered = 0;
  result->final_rmse_px = 0.0;

  if (!config.id_mapping || config.id_mapping->empty()) {
    LOG(ERROR) << "Incremental SfM: id_mapping required for full track build";
    return false;
  }

  // ── Stage 0: Build full TrackStore from all pairs (union-find) ─────────────
  TrackStore store;
  if (!build_full_track_store_from_pairs(config.pairs_json_path, config.geo_dir,
                                         config.match_dir, config.id_mapping, &store)) {
    LOG(ERROR) << "Incremental SfM: failed to build full track store from pairs";
    return false;
  }
  const int n_images = store.num_images();
  LOG(INFO) << "Incremental SfM: Stage 0 done, " << n_images << " images, "
            << store.num_tracks() << " tracks";

  // ── Stage 1: Initial pair (choose pair, triangulate, BA, reject, filter) ───
  Eigen::Matrix3d R1;
  Eigen::Vector3d C1;
  uint32_t image1_index = 0, image2_index = 0;
  if (!run_stage_initial_pair(cameras, config, &store, &R1, &C1, &image1_index, &image2_index)) {
    LOG(ERROR) << "Incremental SfM: initial pair stage failed";
    return false;
  }

  result->image1_index = image1_index;
  result->image2_index = image2_index;
  result->store = std::move(store);
  result->poses_R.resize(static_cast<size_t>(n_images));
  result->poses_C.resize(static_cast<size_t>(n_images));
  result->registered.resize(static_cast<size_t>(n_images), false);
  result->poses_R[static_cast<size_t>(image1_index)] = Eigen::Matrix3d::Identity();
  result->poses_C[static_cast<size_t>(image1_index)] = Eigen::Vector3d::Zero();
  result->poses_R[static_cast<size_t>(image2_index)] = R1;
  result->poses_C[static_cast<size_t>(image2_index)] = C1;
  result->registered[static_cast<size_t>(image1_index)] = true;
  result->registered[static_cast<size_t>(image2_index)] = true;
  result->n_registered = 2;

  // ── Stage 2: Resection loop ───────────────────────────────────────────────
  if (n_images >= 3) {
    const int added = run_stage_resection_loop(cameras, config, &result->store,
                                               &result->poses_R, &result->poses_C,
                                               &result->registered);
    result->n_registered += added;
    LOG(INFO) << "Incremental SfM: resection added " << added << " images";
  } else {
    LOG(INFO) << "Incremental SfM: only 2 images, skipping resection loop";
  }

  // ── Stage 3: Reject outliers ──────────────────────────────────────────────
  std::vector<camera::Intrinsics> intrinsics_per_image =
      build_intrinsics_per_image(cameras, result->store.num_images());
  const int rejected =
      run_stage_reject_outliers(&result->store, result->poses_R, result->poses_C,
                                result->registered, intrinsics_per_image,
                                config.outlier_threshold_px);
  LOG(INFO) << "Incremental SfM: Stage 3 reject outliers, marked " << rejected << " observations";

  // ── Stage 4: Filter tracks ────────────────────────────────────────────────
  const int filtered =
      run_stage_filter_tracks(&result->store, result->poses_R, result->poses_C,
                              result->registered, config.min_observations_per_track,
                              config.min_track_angle_deg);
  LOG(INFO) << "Incremental SfM: Stage 4 filter tracks, marked " << filtered << " tracks deleted";

  // ── Stage 5: Global BA (analytic Jacobians, quaternion + centre parameterization) ──
  if (config.run_global_ba && result->n_registered >= 2) {
    std::vector<Eigen::Matrix3d> ba_R;
    std::vector<Eigen::Vector3d> ba_C;
    double rmse = 0.0;
    const bool ba_ok = run_stage_global_ba(
        cameras, config.optimize_intrinsics, config.global_ba_max_iterations,
        result->store, result->poses_R, result->poses_C,
        &ba_R, &ba_C, &result->store,
        &result->camera_index_for_image, &result->cameras, &rmse);
    if (ba_ok) {
      result->poses_R = std::move(ba_R);
      result->poses_C = std::move(ba_C);
      result->final_rmse_px = rmse;
      LOG(INFO) << "Incremental SfM: Global BA done, RMSE=" << rmse << " px";
    } else {
      LOG(WARNING) << "Incremental SfM: Global BA skipped or failed";
    }
  }

  result->success = result->n_registered >= 2;
  return result->success;
}

} // namespace sfm
} // namespace insight
