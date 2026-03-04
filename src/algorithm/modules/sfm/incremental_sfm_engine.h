/**
 * @file  incremental_sfm_engine.h
 * @brief Main orchestration for incremental SfM: initial pair → resection loop →
 *        outlier rejection → track filtering → (Local/Global) BA.
 *
 * Pipeline
 * ────────
 *  Stage 0: Load view graph, choose initial pair, build 2-image TrackStore.
 *  Stage 1: Initial pair loop (triangulate, two-view BA, reject, filter).
 *  Stage 2: Resection loop (add images one by one). TODO: expand_store_from_pairs
 *           to build full cluster TrackStore (n_images > 2) from pairs.json.
 *  Stage 3: Reject outliers (multiview), filter tracks.
 *  Stage 4: Global BA (Ceres when available; parallel BA planned).
 *
 * BA Backend
 * ──────────
 *  Currently uses Ceres when INSIGHTAT_USE_CERES. Global BA interface exists;
 *  parallel BA (e.g. sba, g2o) is planned but not yet chosen.
 *
 * Usage
 * ─────
 *   IncrementalSfmConfig config;
 *   config.pairs_json_path = "pairs.json";
 *   config.geo_dir = "geo/";
 *   config.match_dir = "match/";
 *   IncrementalSfmResult result;
 *   if (run_incremental_reconstruction(config, &result))
 *     // result.store, result.poses_R, result.poses_t
 */

#pragma once

#include "../camera/camera_types.h"
#include "track_store.h"
#include <Eigen/Core>
#include <string>
#include <unordered_map>
#include <vector>

namespace insight {
namespace sfm {

// ─────────────────────────────────────────────────────────────────────────────
// Configuration
// ─────────────────────────────────────────────────────────────────────────────

struct IncrementalSfmConfig {
  std::string pairs_json_path; ///< Path to pairs.json
  std::string geo_dir;         ///< Directory of .isat_geo files
  std::string match_dir;      ///< Directory of .isat_match files

  /// Shared intrinsics (single camera). Used when intrinsics_by_image_id is null or lookup fails.
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  const camera::Intrinsics* intrinsics = nullptr;

  /// Per-image intrinsics: image_id → K. One camera per group; lookup by result.image_ids[i].
  /// When non-null, used for resection, reject_outliers, filter_tracks, and Global BA.
  const std::unordered_map<uint32_t, camera::Intrinsics>* intrinsics_by_image_id = nullptr;

  int min_tracks_after_initial = 20; ///< Min valid tracks after initial pair
  int min_correspondences_resection = 6;
  double outlier_threshold_px = 4.0;
  int min_observations_per_track = 2;
  double min_track_angle_deg = 2.0;

  /// Run Global BA after resection loop (when Ceres available).
  bool run_global_ba = true;
  int global_ba_max_iterations = 50;

  /// Run Local BA after each new image (optional, for stability).
  bool run_local_ba_per_image = false;
  int local_ba_max_iterations = 20;
};

// ─────────────────────────────────────────────────────────────────────────────
// Result
// ─────────────────────────────────────────────────────────────────────────────

struct IncrementalSfmResult {
  bool success = false;
  TrackStore store;
  std::vector<Eigen::Matrix3d> poses_R; ///< World-to-camera per image
  std::vector<Eigen::Vector3d> poses_t;
  std::vector<bool> registered; ///< Which images were successfully registered

  /// image_ids[i] = image id for store index i. Used for per-camera intrinsics lookup.
  std::vector<uint32_t> image_ids;

  uint32_t image1_id = 0; ///< Initial pair first image
  uint32_t image2_id = 0; ///< Initial pair second image

  int n_registered = 0;
  double final_rmse_px = 0.0; ///< From last BA (if run)
};

// ─────────────────────────────────────────────────────────────────────────────
// Main entry
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Run full incremental SfM pipeline.
 *
 * 1. Build view graph, choose initial pair.
 * 2. Load pair geo/match, build 2-image TrackStore, triangulate, two-view BA,
 *    reject outliers, filter tracks.
 * 3. Expand store to all images in view graph (TODO: merge tracks from other
 *    pairs); run resection loop.
 * 4. Optionally run Global BA.
 * 5. Optionally run multiview outlier rejection and track filtering.
 *
 * @param config  Input paths and parameters.
 * @param result  Output store, poses, and metadata.
 * @return true if at least 2 images registered and min_tracks_after_initial met.
 */
bool run_incremental_reconstruction(const IncrementalSfmConfig& config,
                                    IncrementalSfmResult* result);

// ─────────────────────────────────────────────────────────────────────────────
// Pipeline stages (for fine-grained control or testing)
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Stage 1: Initial pair loop. Chooses pair, loads geo/match, builds 2-image
 * store, triangulates, runs two-view BA, rejects outliers, filters tracks.
 */
bool run_stage_initial_pair(const IncrementalSfmConfig& config, TrackStore* store_out,
                            Eigen::Matrix3d* R1_out, Eigen::Vector3d* t1_out,
                            uint32_t* image1_id_out, uint32_t* image2_id_out);

/**
 * Stage 2: Resection loop. Repeatedly chooses unregistered image with most
 * 3D–2D correspondences, runs resection, triangulates new tracks.
 * When image_ids non-null and config.intrinsics_by_image_id set, uses per-image intrinsics.
 */
int run_stage_resection_loop(const IncrementalSfmConfig& config, TrackStore* store,
                             std::vector<Eigen::Matrix3d>* poses_R,
                             std::vector<Eigen::Vector3d>* poses_t,
                             std::vector<bool>* registered,
                             const std::vector<uint32_t>* image_ids = nullptr);

/**
 * Stage 3: Reject observations with reprojection error > threshold.
 * Multiview version: uses poses_R, poses_t for all registered images.
 */
int run_stage_reject_outliers(TrackStore* store,
                              const std::vector<Eigen::Matrix3d>& poses_R,
                              const std::vector<Eigen::Vector3d>& poses_t,
                              const std::vector<bool>& registered, double fx, double fy,
                              double cx, double cy, double threshold_px);

/**
 * Overload: use camera::Intrinsics for distortion-aware reprojection.
 */
int run_stage_reject_outliers(TrackStore* store,
                              const std::vector<Eigen::Matrix3d>& poses_R,
                              const std::vector<Eigen::Vector3d>& poses_t,
                              const std::vector<bool>& registered,
                              const camera::Intrinsics& K, double threshold_px);

/**
 * Overload: per-image intrinsics. intrinsics_per_image[image_index] = K for that image.
 * Size must match poses_R. When null or empty, fallback to shared fx,fy,cx,cy.
 */
int run_stage_reject_outliers(TrackStore* store,
                              const std::vector<Eigen::Matrix3d>& poses_R,
                              const std::vector<Eigen::Vector3d>& poses_t,
                              const std::vector<bool>& registered,
                              const std::vector<camera::Intrinsics>* intrinsics_per_image,
                              double fx, double fy, double cx, double cy,
                              double threshold_px);

/**
 * Stage 4: Filter tracks (too few observations, bad geometry).
 * Multiview version.
 */
int run_stage_filter_tracks(TrackStore* store,
                            const std::vector<Eigen::Matrix3d>& poses_R,
                            const std::vector<Eigen::Vector3d>& poses_t,
                            const std::vector<bool>& registered, int min_observations,
                            double min_angle_deg);

/**
 * Stage 5: Global BA. Optimizes all poses (except cam0) and all points.
 * Uses Ceres when INSIGHTAT_USE_CERES; otherwise no-op (returns false).
 * On success, updates store_out with optimized 3D points (if non-null).
 * When fx_per_camera non-empty, use per-camera intrinsics; else use shared fx,fy,cx,cy.
 */
bool run_stage_global_ba(const TrackStore& store,
                         const std::vector<Eigen::Matrix3d>& poses_R,
                         const std::vector<Eigen::Vector3d>& poses_t, double fx,
                         double fy, double cx, double cy, int max_iterations,
                         std::vector<Eigen::Matrix3d>* poses_R_out,
                         std::vector<Eigen::Vector3d>* poses_t_out,
                         TrackStore* store_out, double* rmse_px_out);

/**
 * Overload: per-camera intrinsics. intrinsics_per_image[i] = K for image index i.
 * Size must match poses_R. When null or empty, fallback to shared fx,fy,cx,cy.
 */
bool run_stage_global_ba(const TrackStore& store,
                         const std::vector<Eigen::Matrix3d>& poses_R,
                         const std::vector<Eigen::Vector3d>& poses_t,
                         const std::vector<camera::Intrinsics>* intrinsics_per_image,
                         double fx, double fy, double cx, double cy, int max_iterations,
                         std::vector<Eigen::Matrix3d>* poses_R_out,
                         std::vector<Eigen::Vector3d>* poses_t_out,
                         TrackStore* store_out, double* rmse_px_out);

} // namespace sfm
} // namespace insight
