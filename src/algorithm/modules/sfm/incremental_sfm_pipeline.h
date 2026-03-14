/**
 * @file  incremental_sfm_pipeline.h
 * @brief Incremental SfM pipeline (index-only, cameras + image_to_camera_index, no IdMapping).
 *
 * Initial pair loop: try candidate pairs from ViewGraph until one succeeds (two-view triang + BA +
 * check). Resection loop, batch triangulation, re-triangulation, optional local/global BA.
 */

#pragma once

#include "../camera/camera_types.h"
#include "track_store.h"
#include "view_graph.h"
#include <Eigen/Core>
#include <string>
#include <vector>

namespace insight {
namespace sfm {

/**
 * Run initial-pair loop: get candidate pairs (twoview_ok && stable) sorted by score,
 * for each try load geo, triangulate two-view tracks, two-view BA, reject/filter, check count/RMSE.
 * Uses cameras[image_to_camera_index[im]] for intrinsics; no intrinsics_per_image.
 *
 * @param view_graph       Built from pairs + geo_dir (degeneracy from geo fields).
 * @param geo_dir          Directory of .isat_geo (path: geo_dir/im0_im1.isat_geo, index-based).
 * @param store            Track store (from IDC); two-view tracks will be triangulated and updated.
 * @param cameras          Current camera intrinsics (may be updated by BA later).
 * @param image_to_camera_index  image_to_camera_index[i] = camera index for image i.
 * @param min_tracks_after Minimum valid two-view tracks after filter to accept the pair.
 * @param initial_im0_out  Output: first image index of chosen pair.
 * @param initial_im1_out  Output: second image index of chosen pair.
 * @param poses_R_out      Output: poses_R for all images; only [im0],[im1] filled (world = im0).
 * @param poses_C_out      Output: poses_C for all images.
 * @param registered_out   Output: registered flags; only [im0],[im1] set true.
 * @return true if a pair was chosen and store/poses updated; false if none succeeded.
 */
bool run_initial_pair_loop(const ViewGraph& view_graph, const std::string& geo_dir,
                           TrackStore* store, const std::vector<camera::Intrinsics>& cameras,
                           const std::vector<int>& image_to_camera_index, int min_tracks_after,
                           uint32_t* initial_im0_out, uint32_t* initial_im1_out,
                           std::vector<Eigen::Matrix3d>* poses_R_out,
                           std::vector<Eigen::Vector3d>* poses_C_out,
                           std::vector<bool>* registered_out);

/**
 * Choose next resection batch using OpenMVG-style ratio threshold.
 *
 * Algorithm:
 *   1. Score every unregistered image by its number of 3D-2D correspondences
 *      (triangulated tracks visible from that image); discard images with
 *      fewer than min_3d2d_count correspondences.
 *   2. best_count = score of the top candidate.
 *   3. batch = all candidates with score >= batch_ratio * best_count,
 *      capped at batch_max images (highest scores first).
 *
 * @param min_3d2d_count  Absolute floor: images below this are never candidates.
 *                        COLMAP uses 15; 6 is the PnP hard minimum.
 * @param batch_ratio     Relative threshold vs best candidate (OpenMVG default 0.75).
 * @param batch_max       Upper cap on batch size to avoid over-registration per iter.
 */
std::vector<int> choose_next_resection_batch(const TrackStore& store,
                                             const std::vector<bool>& registered,
                                             int min_3d2d_count, double batch_ratio,
                                             int batch_max);

/**
 * Run resection for each image in the batch; intrinsics = cameras[image_to_camera_index[im]].
 * Updates poses_R, poses_C, registered for each successful resection.
 * @return Number of images newly registered.
 */
int run_batch_resection(const TrackStore& store, const std::vector<int>& image_indices,
                        const std::vector<camera::Intrinsics>& cameras,
                        const std::vector<int>& image_to_camera_index,
                        std::vector<Eigen::Matrix3d>* poses_R,
                        std::vector<Eigen::Vector3d>* poses_C, std::vector<bool>* registered,
                        int min_inliers = 6);

/**
 * Triangulate tracks that have observations in the given new images and in already-registered
 * images, but no triangulated xyz yet. Uses current cameras for undistortion.
 * @param min_tri_angle_deg  Minimum max pairwise ray angle (degrees); reject degenerate points.
 * @return Number of tracks newly triangulated.
 */
int run_batch_triangulation(TrackStore* store, const std::vector<int>& new_registered_image_indices,
                            const std::vector<Eigen::Matrix3d>& poses_R,
                            const std::vector<Eigen::Vector3d>& poses_C,
                            const std::vector<bool>& registered,
                            const std::vector<camera::Intrinsics>& cameras,
                            const std::vector<int>& image_to_camera_index,
                            double min_tri_angle_deg = 0.5,
                            std::vector<int>* new_track_ids_out = nullptr);

/**
 * Re-triangulate tracks with track_needs_retriangulation; clear flag. Use current cameras for
 * undistortion.
 * @param min_tri_angle_deg  Minimum max pairwise ray angle (degrees); reject degenerate points.
 * @return Number of tracks re-triangulated.
 */
int run_retriangulation(TrackStore* store, const std::vector<Eigen::Matrix3d>& poses_R,
                        const std::vector<Eigen::Vector3d>& poses_C,
                        const std::vector<bool>& registered,
                        const std::vector<camera::Intrinsics>& cameras,
                        const std::vector<int>& image_to_camera_index,
                        double min_tri_angle_deg = 0.5);

// ─── BA (build from Store, solve, write back) ───────────────────────────────

/**
 * Run global BA: build BAInput from store for all registered images, solve, write back poses and
 * points.
 * @param cameras Current intrinsics (required for BA). If cameras_in_out is non-null, optimised
 * intrinsics are written back when optimize_intrinsics is true.
 * @param anchor_image Global image index to fix as the coordinate-system anchor. If -1, the first
 * BA image (lowest registered index) is fixed.
 */
bool run_global_ba(TrackStore* store, std::vector<Eigen::Matrix3d>* poses_R,
                   std::vector<Eigen::Vector3d>* poses_C, const std::vector<bool>& registered,
                   const std::vector<int>& image_to_camera_index,
                   const std::vector<camera::Intrinsics>& cameras,
                   std::vector<camera::Intrinsics>* cameras_in_out, bool optimize_intrinsics,
                   int max_iterations, double* rmse_px_out, int anchor_image = -1);

/**
 * Run local BA: optimize a subset of images (other poses fixed). Intrinsics not optimized.
 * If indices_to_optimize is non-null, use that set; else optimize the last local_ba_window
 * registered images.
 */
/// @param anchor_image  Global image index that is always kept fixed (gauge fix).
///                      Pass -1 to use the default frozen-pose mechanism only.
bool run_local_ba(TrackStore* store, std::vector<Eigen::Matrix3d>* poses_R,
                  std::vector<Eigen::Vector3d>* poses_C, const std::vector<bool>& registered,
                  const std::vector<int>& image_to_camera_index,
                  const std::vector<camera::Intrinsics>& cameras, int local_ba_window,
                  int max_iterations, double* rmse_px_out,
                  const std::vector<int>* indices_to_optimize = nullptr,
                  int anchor_image = -1);

/**
 * COLMAP-style local BA: 2-hop visibility expansion from `batch` (newly registered images).
 *   Hop-1: seed 3D points visible from batch.
 *   Hop-2: registered cameras observing seed points, scored by count; top
 *          max_variable_cameras become the "variable" (optimised) set.
 *   Hop-3: remaining observers of seed points → "constant" (frozen) cameras providing gauge fix.
 * Only the seed points are included, so the BA problem is genuinely local.
 * Intrinsics are not optimised.
 */
bool run_local_ba_colmap(TrackStore* store, std::vector<Eigen::Matrix3d>* poses_R,
                         std::vector<Eigen::Vector3d>* poses_C,
                         const std::vector<bool>& registered,
                         const std::vector<int>& image_to_camera_index,
                         const std::vector<camera::Intrinsics>& cameras,
                         const std::vector<int>& batch, int max_variable_cameras,
                         int max_iterations, double* rmse_px_out);

/**
 * Neighbor-anchored local BA (kBatchNeighbor strategy).
 *   Variable cameras : batch (newly resected, PnP poses).
 *   Variable 3D points: new_track_ids (just triangulated this iteration).
 *   Constant cameras : per-batch-image top-neighbor_k neighbors by shared triangulated point count
 *                      (union across all batch images, excluding batch itself).
 *   Constant 3D points: old triangulated tracks visible from ≥1 batch AND ≥1 constant camera.
 * Intrinsics are not optimised.
 */
bool run_local_ba_batch_neighbor(TrackStore* store, std::vector<Eigen::Matrix3d>* poses_R,
                                 std::vector<Eigen::Vector3d>* poses_C,
                                 const std::vector<bool>& registered,
                                 const std::vector<int>& image_to_camera_index,
                                 const std::vector<camera::Intrinsics>& cameras,
                                 const std::vector<int>& batch,
                                 const std::vector<int>& new_track_ids, int neighbor_k,
                                 int max_iterations, double* rmse_px_out);

// ─── Full pipeline ───────────────────────────────────────────────────────────

/// Strategy for the per-iteration local BA in the incremental SfM loop.
enum class LocalBAStrategy {
  /// Current default: select the last N / connectivity-ranked images as the
  /// variable window; all registered images are fed to the solver (outer
  /// cameras frozen).  Simple and robust.
  kWindow,
  /// COLMAP-style: 2-hop visibility graph expansion from the newly registered
  /// batch.  Variable cameras = top-scored neighbours (up to
  /// local_ba_colmap_max_variable_images).  Constant cameras = additional
  /// observers of the same seed points (frozen, provide gauge fix).  Only seed
  /// points are included, so the problem is genuinely local.
  kColmap,
  /// Neighbor-anchored: variable = batch cameras + newly triangulated points;
  /// constant = per-batch-image top-K neighbors by co-visibility (union).
  /// No magic window parameter – neighborhood size is data-driven.
  kBatchNeighbor,
};

struct IncrementalSfMOptions {
  // ─── Initial pair selection (COLMAP-style, track-based) ───────────────────
  int min_tracks_after = 50; ///< Minimum valid two-view tracks after filter to accept initial pair.
  int init_max_first_images = 100; ///< Max first-image candidates to try in COLMAP-style selection.
  int init_max_second_images = 50; ///< Max second-image candidates per first image.
  int init_min_shared_tracks =
      30; ///< Min shared tracks for an image to be a second-image candidate.
  double init_ba_rmse_max = 10.0; ///< Max BA RMSE (px) to accept initial pair.
  double init_outlier_threshold_px =
      4.0; ///< Reprojection threshold for soft inlier count during initial pair trial.
  double init_min_angle_deg = 2.0; ///< Min triangulation angle for inlier tracks in initial pair.

  // ─── Resection loop ──────────────────────────────────────────────────────
  /// Minimum RANSAC inliers for a resection to be accepted (hard PnP minimum = 6).
  int resection_min_inliers = 6;
  /// Minimum 3D-2D correspondence count for an image to be a resection candidate.
  /// Images below this floor are not attempted even if they have some correspondences.
  /// COLMAP uses 15; raise to 30 for very high-overlap datasets.
  int resection_min_3d2d_count = 15;
  /// OpenMVG-style ratio threshold: include all candidates whose 3D-2D count is
  /// >= resection_batch_ratio * best_candidate_count.  0.75 = OpenMVG default.
  double resection_batch_ratio = 0.75;
  /// Hard upper cap on batch size per SfM iteration.  Prevents registering too
  /// many images at once before the next BA pass stabilises the map.
  int resection_batch_max = 20;

  // ─── Bundle adjustment ───────────────────────────────────────────────────
  int local_ba_after_n_images =
      20; ///< Use local BA only when num_registered >= this; before that run global BA each time.
         ///< Set low (20) since global BA cost grows fast; local BA is nearly equivalent at small n.
  int local_ba_window = 20;
  bool local_ba_by_connectivity = true; ///< If true, local BA optimizes new images + most connected
                                        ///< neighbors; else last N by index.
  /// Which local-BA strategy to use.  kWindow is the default (original behaviour).
  /// Switch to kColmap to evaluate COLMAP-style 2-hop visibility expansion.
  LocalBAStrategy local_ba_strategy = LocalBAStrategy::kColmap;
  /// Maximum number of variable (freely optimised) cameras in kColmap local BA.
  /// Cameras beyond this cap are assigned to the constant (frozen) set.
  /// COLMAP default is 6 (LocalBundleAdjustor::Options::max_num_images).
  int local_ba_colmap_max_variable_images = 6;
  /// kBatchNeighbor: for each batch image, take this many top neighbors (by shared triangulated
  /// point count) as constant anchor cameras.  Union is taken across all batch images so every
  /// new camera has at least min(neighbor_k, registered_count) anchors.
  int local_ba_neighbor_k = 5;
  /// Run global BA only every N newly-registered images during the early global-BA phase
  /// (num_registered < local_ba_after_n_images).  The last BA before switching to local BA
  /// is always executed regardless.  Default = 1 (original: BA after every image).
  /// Setting to 3 reduces early-phase global BA cost ~3× with negligible accuracy loss.
  int global_ba_every_n_images = 1;
  bool do_global_ba = true;             ///< Run global BA at end of pipeline.
  bool global_ba_optimize_intrinsics = true;
  /// Only enable intrinsics optimization in the early global BA when
  /// num_registered >= this threshold (avoids instability with few cameras).
  int global_ba_optimize_intrinsics_min_images = 10;
  int max_global_ba_iterations = 50; ///< Was 50: 50 is sufficient for convergence in most cases.

  // ─── Triangulation ───────────────outlier rejection:────────────────────
  /// Minimum max pairwise ray angle (degrees) for a newly triangulated point to be accepted.
  /// 0.5° is suitable for dense aerial overlap; use 2.0° for sparser datasets.
  double min_tri_angle_deg = 0.5;

  // ─── Outlier rejection ───────────────────────────────────────────────────
  double outlier_threshold_px =
      4.0; ///< Mark observation deleted if reprojection error > this (after each BA).
  /// Adaptive outlier threshold factor. Effective threshold = max(outlier_threshold_px, prev_rmse * this).
  /// Prevents aggressive culling when RMSE is temporarily high (e.g. during early intrinsics convergence).
  /// Set to 0 to disable adaptive behavior.
  double outlier_adaptive_factor = 2.0;
  int max_outlier_iterations = 5; ///< Max BA+reject rounds after each BA until no new outliers.
  /// Stop extra BA+reject rounds early if the number of newly-rejected observations drops below
  /// this threshold. Avoids expensive BA calls that buy very little (e.g. round 4: rejected=8).
  /// Set to 0 to disable (always continue while rejected > 0, original behaviour).
  int min_outliers_for_ba_retry = 30;
  int reject_min_registered_images =
      10; ///< Do not run pixel/angle reject until num_registered >= this.
  double min_observation_angle_deg =
      2.0; ///< Mark observation deleted if max parallax angle (with other views in track) < this.
  bool enable_sigma_filter = false; ///< After reject loop, one more pass with threshold = rmse * 3.
  int final_reject_max_rounds =
      10; ///< After final global BA, iterate BA+reject until no outliers or this many rounds.
  /// Run run_retriangulation() every this many SfM iterations inside the main loop.
  /// 0 = disable periodic retriangulation (only the final pass at the end runs).
  /// 3 is a good default: covers tracks cleared by outlier rejection AND tracks
  /// that became triangulatable only after more cameras were registered.
  int retriangulation_every_n_iters = 3;
  bool retry_resection_after_cleanup =
      true; ///< When no resection candidates, run one global BA+reject then try again once.
};

/**
 * Run full incremental SfM: load store from IDC, build view graph, initial pair, resection loop,
 * batch triangulation, optional local BA, re-triangulation, optional global BA. Uses current
 * cameras for all undistortion; writes back poses and optionally cameras.
 *
 * @param tracks_idc_path     Path to .isat_tracks IDC.
 * @param pairs_json_path     Path to pairs JSON for view graph.
 * @param geo_dir             Directory of .isat_geo files (index-based paths).
 * @param cameras             Camera intrinsics (size = num_cameras). Updated if do_global_ba &&
 * global_ba_optimize_intrinsics.
 * @param image_to_camera_index  Per-image camera index; length must match store.num_images() after
 * load.
 * @param opts                Pipeline options.
 * @param store_out           Output: track store (loaded from IDC; num_images must match
 * image_to_camera_index.size()).
 * @param poses_R_out         Output: rotation per image (world to camera).
 * @param poses_C_out         Output: camera centre per image.
 * @param registered_out     Output: true for each registered image.
 * @param initial_im0_out    Optional: first image index of initial pair.
 * @param initial_im1_out    Optional: second image index of initial pair.
 * @return true if pipeline completed (at least initial pair + some resections or full loop).
 */
bool run_incremental_sfm_pipeline(
    const std::string& tracks_idc_path, const std::string& pairs_json_path,
    const std::string& geo_dir, std::vector<camera::Intrinsics>* cameras,
    const std::vector<int>& image_to_camera_index, const IncrementalSfMOptions& opts,
    TrackStore* store_out, std::vector<Eigen::Matrix3d>* poses_R_out,
    std::vector<Eigen::Vector3d>* poses_C_out, std::vector<bool>* registered_out,
    uint32_t* initial_im0_out = nullptr, uint32_t* initial_im1_out = nullptr);

} // namespace sfm
} // namespace insight
