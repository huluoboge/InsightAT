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
 *   MultiCameraSetup cameras;
 *   cameras.image_camera_map = ...;   // image_id → camera_id
 *   cameras.cameras[1] = K1;          // camera_id → intrinsics+distortion
 *
 *   IncrementalSfmConfig config;
 *   config.pairs_json_path = "pairs.json";
 *   config.geo_dir = "geo/";
 *   config.match_dir = "match/";
 *
 *   IncrementalSfmResult result;
 *   if (run_incremental_reconstruction(cameras, config, &result))
 *     // result.store, result.poses_R, result.poses_t, result.cameras
 */

#pragma once

#include "../camera/camera_types.h"
#include "track_store.h"
#include <Eigen/Core>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace insight {
namespace sfm {

// ─────────────────────────────────────────────────────────────────────────────
// Multi-camera setup data
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Multiple independent cameras, each with its own model (intrinsics + distortion
 * as one unit). Every image is assigned to one camera via its camera_id.
 *
 * Images sharing the same camera_id share one parameter block in Global BA,
 * so their intrinsics and distortion are optimized jointly (or fixed jointly).
 *
 * Populated from:
 *   - image_camera_map : Image List Format v2.0  (image_id → camera_id)
 *   - cameras          : intrinsics JSON          (camera_id → model)
 */
struct MultiCameraSetup {
  /// image_id → camera_id  (from images_all.json / Image List Format v2.0)
  std::unordered_map<uint32_t, uint32_t> image_camera_map;

  /// camera_id → camera model (intrinsics + distortion together, one per physical camera)
  std::unordered_map<uint32_t, camera::Intrinsics> cameras;

  bool empty() const { return cameras.empty(); }

  /// Camera model for a given image_id. Falls back to camera_id=1, then any camera.
  const camera::Intrinsics* for_image(uint32_t image_id) const {
    auto it = image_camera_map.find(image_id);
    if (it != image_camera_map.end()) {
      auto it2 = cameras.find(it->second);
      if (it2 != cameras.end())
        return &it2->second;
    }
    auto it2 = cameras.find(1);
    if (it2 != cameras.end())
      return &it2->second;
    return cameras.empty() ? nullptr : &cameras.begin()->second;
  }

  /// Reference camera for initial pair (camera_id=1, or first available).
  const camera::Intrinsics* reference() const {
    auto it = cameras.find(1);
    if (it != cameras.end())
      return &it->second;
    return cameras.empty() ? nullptr : &cameras.begin()->second;
  }
};

// ─────────────────────────────────────────────────────────────────────────────
// Algorithm configuration  (parameters only, no data)
// ─────────────────────────────────────────────────────────────────────────────

struct IncrementalSfmConfig {
  std::string pairs_json_path; ///< Path to pairs.json
  std::string geo_dir;         ///< Directory of .isat_geo files
  std::string match_dir;       ///< Directory of .isat_match files

  int min_tracks_after_initial = 20; ///< Min valid tracks after initial pair
  int min_correspondences_resection = 6;
  double outlier_threshold_px = 4.0;
  int min_observations_per_track = 2;
  double min_track_angle_deg = 2.0;

  /// Run Global BA after resection loop (when Ceres available).
  bool run_global_ba = true;
  int global_ba_max_iterations = 50;

  /// Optimize intrinsics+distortion in Global BA. When false (default), camera
  /// models are fixed constraints; poses and 3D points are optimized.
  bool optimize_intrinsics = false;

  /// Run local BA after each new image (optional, for incremental stability).
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

  /// image_ids[i] = image id for store index i.
  std::vector<uint32_t> image_ids;

  uint32_t image1_id = 0; ///< Initial pair first image
  uint32_t image2_id = 0; ///< Initial pair second image

  int n_registered = 0;
  double final_rmse_px = 0.0; ///< From last BA (if run)

  /// camera_index_for_image[i] = index into cameras for store image i.
  std::vector<int> camera_index_for_image;

  /// Distinct cameras with (possibly BA-optimized) intrinsics+distortion.
  /// Parallel to the unique cameras in the MultiCameraSetup seen across all registered images.
  std::vector<camera::Intrinsics> cameras;
};

// ─────────────────────────────────────────────────────────────────────────────
// Main entry
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Run full incremental SfM pipeline.
 *
 * @param cameras  Camera setup: image_id→camera_id and camera_id→model (intrinsics+distortion).
 * @param config   Algorithm parameters (paths, thresholds, BA flags).
 * @param result   Output: store, poses, per-image camera index, (optionally optimized) cameras.
 * @return true if at least 2 images registered.
 */
bool run_incremental_reconstruction(const MultiCameraSetup& cameras,
                                    const IncrementalSfmConfig& config,
                                    IncrementalSfmResult* result);

// ─────────────────────────────────────────────────────────────────────────────
// Pipeline stages (for fine-grained control or testing)
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Stage 1: Choose initial pair, build 2-image TrackStore, triangulate,
 * two-view BA, reject outliers, filter tracks.
 */
bool run_stage_initial_pair(const MultiCameraSetup& cameras,
                            const IncrementalSfmConfig& config,
                            TrackStore* store_out, Eigen::Matrix3d* R1_out,
                            Eigen::Vector3d* t1_out,
                            uint32_t* image1_id_out, uint32_t* image2_id_out);

/**
 * Stage 2: Resection loop. Repeatedly registers the unregistered image with
 * the most 3D–2D correspondences. Per-image camera models are resolved from
 * cameras + image_ids.
 */
int run_stage_resection_loop(const MultiCameraSetup& cameras,
                             const IncrementalSfmConfig& config,
                             TrackStore* store,
                             std::vector<Eigen::Matrix3d>* poses_R,
                             std::vector<Eigen::Vector3d>* poses_t,
                             std::vector<bool>* registered,
                             const std::vector<uint32_t>& image_ids);

/**
 * Stage 3: Reject observations with reprojection error > threshold.
 * intrinsics_per_image[i] = camera model for store image index i.
 */
int run_stage_reject_outliers(TrackStore* store,
                              const std::vector<Eigen::Matrix3d>& poses_R,
                              const std::vector<Eigen::Vector3d>& poses_t,
                              const std::vector<bool>& registered,
                              const std::vector<camera::Intrinsics>& intrinsics_per_image,
                              double threshold_px);

/**
 * Stage 4: Filter tracks (too few observations, bad triangulation angle).
 */
int run_stage_filter_tracks(TrackStore* store,
                            const std::vector<Eigen::Matrix3d>& poses_R,
                            const std::vector<Eigen::Vector3d>& poses_t,
                            const std::vector<bool>& registered,
                            int min_observations, double min_angle_deg);

/**
 * Stage 5: Global BA. Optimizes poses and 3D points; camera models shared per
 * camera_id. When optimize_intrinsics=true, intrinsics+distortion are also
 * optimized (all images of the same camera contribute to the same parameter block).
 * Returns optimized cameras in cameras_out (parallel to MultiCameraSetup::cameras).
 */
bool run_stage_global_ba(const MultiCameraSetup& cameras,
                         const std::vector<uint32_t>& image_ids,
                         bool optimize_intrinsics, int max_iterations,
                         const TrackStore& store,
                         const std::vector<Eigen::Matrix3d>& poses_R,
                         const std::vector<Eigen::Vector3d>& poses_t,
                         std::vector<Eigen::Matrix3d>* poses_R_out,
                         std::vector<Eigen::Vector3d>* poses_t_out,
                         TrackStore* store_out,
                         std::vector<int>* camera_index_for_image_out,
                         std::vector<camera::Intrinsics>* cameras_out,
                         double* rmse_px_out);

} // namespace sfm
} // namespace insight
