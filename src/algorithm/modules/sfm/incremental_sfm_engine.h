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
 *  Uses Ceres for all BA (two-view, pose-only, global). Alternative backends
 *  should be implemented as separate translation units at a higher layer.
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
 *     // result.store, result.poses_R, result.poses_C (per index 0..n-1), result.cameras
 *
 * Index-only: pipeline uses only image index 0..num_images()-1. Result is keyed by index;
 * caller uses IdMapping at boundary to convert index → original_id when writing output.
 */

#pragma once

#include "../camera/camera_types.h"
#include "id_mapping.h"
#include "track_store.h"
#include <Eigen/Core>
#include <cstdint>
#include <string>
#include <vector>

namespace insight {
namespace sfm {

// ─────────────────────────────────────────────────────────────────────────────
// Multi-camera setup data
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Multiple independent cameras (vector-based for speed and GPU-friendly indexing).
 * Every image is assigned to one camera by index; images sharing the same camera
 * index share one parameter block in Global BA.
 *
 * Populated from IdMapping + intrinsics: image_to_camera[i] = camera index for
 * image index i; cameras[j] = intrinsics for camera index j.
 */
struct MultiCameraSetup {
  /// image index i (0..num_images-1) -> camera index (0..num_cameras-1). Single camera = size 1.
  std::vector<int> image_to_camera;
  /// camera index j -> full intrinsics (fx,fy,cx,cy,k1,k2,k3,p1,p2). Must be non-empty.
  std::vector<camera::Intrinsics> cameras;

  int num_images() const { return static_cast<int>(image_to_camera.size()); }
  int num_cameras() const { return static_cast<int>(cameras.size()); }
  bool empty() const { return cameras.empty(); }

  /// Intrinsics for image index i. Returns nullptr if out of range or camera missing.
  const camera::Intrinsics* for_image_index(int i) const {
    if (i < 0 || static_cast<size_t>(i) >= image_to_camera.size())
      return nullptr;
    int c = image_to_camera[static_cast<size_t>(i)];
    if (c < 0 || static_cast<size_t>(c) >= cameras.size())
      return nullptr;
    return &cameras[static_cast<size_t>(c)];
  }

  const camera::Intrinsics* for_image(uint32_t image_id) const {
    return for_image_index(static_cast<int>(image_id));
  }

  /// First camera (e.g. for fallback in build_intrinsics_per_image when index out of range).
  const camera::Intrinsics* reference() const {
    return cameras.empty() ? nullptr : &cameras[0];
  }
};

// ─────────────────────────────────────────────────────────────────────────────
// Algorithm configuration  (parameters only, no data)
// ─────────────────────────────────────────────────────────────────────────────

struct IncrementalSfmConfig {
  std::string pairs_json_path; ///< Path to pairs.json
  std::string geo_dir;         ///< Directory of .isat_geo files
  std::string match_dir;       ///< Directory of .isat_match files

  /// When non-null, pairs are converted to internal indices and file paths use original IDs.
  const IdMapping* id_mapping = nullptr;

  int min_tracks_after_initial = 20; ///< Min valid tracks after initial pair
  int min_correspondences_resection = 6;
  double outlier_threshold_px = 4.0;
  int min_observations_per_track = 2;
  double min_track_angle_deg = 2.0;

  /// Run Global BA after resection loop.
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
  std::vector<Eigen::Matrix3d> poses_R; ///< Per image index 0..n-1 (world-to-camera rotation)
  std::vector<Eigen::Vector3d> poses_C; ///< Per image index (camera centre in world; C = -Rᵀ·t)
  std::vector<bool> registered;         ///< Which image indices were successfully registered

  uint32_t image1_index = 0; ///< Initial pair first image (index)
  uint32_t image2_index = 0; ///< Initial pair second image (index)

  int n_registered = 0;
  double final_rmse_px = 0.0; ///< From last BA (if run)

  /// camera_index_for_image[i] = camera index for image index i (into cameras[]).
  std::vector<int> camera_index_for_image;
  /// Distinct cameras (possibly BA-optimized). Parallel to MultiCameraSetup::cameras.
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
 * @param result   Output: store, poses per index, camera index per image, (optionally optimized) cameras.
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
                            Eigen::Vector3d* C1_out,
                            uint32_t* image1_index_out, uint32_t* image2_index_out);

/**
 * Stage 2: Resection loop. Repeatedly registers the unregistered image (by index)
 * with the most 3D–2D correspondences. Uses store.num_images() and cameras.for_image_index(i).
 */
int run_stage_resection_loop(const MultiCameraSetup& cameras,
                             const IncrementalSfmConfig& config,
                             TrackStore* store,
                             std::vector<Eigen::Matrix3d>* poses_R,
                             std::vector<Eigen::Vector3d>* poses_C,
                             std::vector<bool>* registered);

/**
 * Stage 3: Reject observations with reprojection error > threshold.
 * intrinsics_per_image[i] = camera model for store image index i.
 */
int run_stage_reject_outliers(TrackStore* store,
                              const std::vector<Eigen::Matrix3d>& poses_R,
                              const std::vector<Eigen::Vector3d>& poses_C,
                              const std::vector<bool>& registered,
                              const std::vector<camera::Intrinsics>& intrinsics_per_image,
                              double threshold_px);

/**
 * Stage 4: Filter tracks (too few observations, bad triangulation angle).
 */
int run_stage_filter_tracks(TrackStore* store,
                            const std::vector<Eigen::Matrix3d>& poses_R,
                            const std::vector<Eigen::Vector3d>& poses_C,
                            const std::vector<bool>& registered,
                            int min_observations, double min_angle_deg);

/**
 * Stage 5: Global BA. Optimizes poses and 3D points; camera models shared per camera index.
 * When optimize_intrinsics=true, intrinsics+distortion are also optimized.
 * Returns optimized cameras in cameras_out (parallel to MultiCameraSetup::cameras).
 */
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
                         double* rmse_px_out);

} // namespace sfm
} // namespace insight
