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
 *   3. ratio_floor = ceil(best_count * batch_ratio).
 *      Late-stage relaxation (object-scan): when num_registered > late_registered_threshold
 *      and late_absolute_min > 0, ratio_floor = min(ratio_floor, late_absolute_min),
 *      allowing more candidates into the batch once the map is large enough.
 *   4. batch = all candidates with score >= ratio_floor, capped at
 *      late_batch_max (in late-stage mode) or batch_max (normal mode).
 *
 * @param min_3d2d_count            Absolute floor: images below this are never candidates.
 *                                  COLMAP uses 15; raise to 30 for high-overlap datasets.
 * @param batch_ratio               Relative threshold vs best candidate (OpenMVG default 0.75).
 * @param batch_max                 Upper cap on batch size (normal mode).
 * @param late_registered_threshold When num_registered exceeds this, switch to late-stage mode.
 *                                  0 = disabled (pure ratio mode always).
 * @param late_absolute_min         Late-stage absolute floor: ratio_floor = min(ratio_floor, this).
 *                                  0 = disabled.
 * @param late_batch_max            Hard cap on batch size in late-stage mode.  Use a lower value
 *                                  than batch_max to avoid adding too many images at once when the
 *                                  map is large (which makes global BA very expensive).
 *                                  0 = use batch_max (no separate late-stage cap).
 */
std::vector<int> choose_next_resection_batch(const TrackStore& store,
                                             const std::vector<bool>& registered,
                                             int min_3d2d_count, double batch_ratio,
                                             int batch_max,
                                             int late_registered_threshold = 0,
                                             int late_absolute_min = 0,
                                             int late_batch_max = 0);

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

/// Optional per-call Ceres solver overrides for run_global_ba.
/// Zero / 0.0 fields are ignored — built-in defaults apply.
struct BASolverOverrides {
  double gradient_tolerance          = 0.0; ///< Ceres gradient_tolerance.  0 = Ceres default.
  double function_tolerance          = 0.0; ///< Ceres function_tolerance.  0 = Ceres default.
  double parameter_tolerance         = 0.0; ///< Ceres parameter_tolerance. 0 = Ceres default.
  int dense_schur_max_variable_cams  = 0;   ///< DENSE↔SPARSE Schur threshold. 0 = built-in (30).
};

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
                   int max_iterations, double* rmse_px_out, int anchor_image = -1,
                   const std::vector<bool>* camera_frozen = nullptr,
                   uint32_t partial_intr_fix = 0u, double focal_prior_weight = 0.0,
                   const BASolverOverrides& solver_overrides = {});

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

// ─────────────────────────────────────────────────────────────────────────────
// Sub-option groups
// ─────────────────────────────────────────────────────────────────────────────

/// Options for initial pair selection.
struct InitPairOptions {
  int min_tracks_after = 50;         ///< Min inlier tracks after MAD filter to accept pair.
  int max_first_images = 100;        ///< Max first-image candidates to try.
  int max_second_images = 50;        ///< Max second-image candidates per first image.
  double ba_rmse_max = 10.0;         ///< Max BA RMSE (px) to accept pair.
  double outlier_threshold_px = 4.0; ///< MAD floor (fallback when distribution is very narrow).
  double min_angle_deg = 2.0;        ///< Min triangulation angle; max is hard-coded to 60°.
};

/// Options for the resection batch loop.
struct ResectionOptions {
  int min_inliers = 6;                  ///< Min PnP RANSAC inliers to accept resection.
  int min_3d2d_count = 15;              ///< Min 3D-2D correspondences to consider an image.
  double batch_ratio = 0.75;            ///< Include images with score ≥ ratio × best_score.
  int batch_max = 20;                   ///< Hard cap on batch size (normal mode).
  int late_registered_threshold = 0;    ///< Switch to late-stage mode above this count (0=off).
  int late_absolute_min = 0;            ///< Late-stage absolute-minimum 3D-2D floor (0=off).
  int late_batch_max = 0;               ///< Late-stage batch cap (0 = use batch_max).
  bool retry_after_cleanup = true;      ///< Retry after cleanup BA+reject when batch is empty.
};

/// Progressive intrinsics unlock schedule + freeze policy.
///
/// Four phases keyed on num_registered:
///   Phase 0 (< phase1_min):             fix ALL — no intrinsics optimisation.
///   Phase 1 [phase1_min, phase2_min):   optimise fx/fy only.
///   Phase 2 [phase2_min, phase3_min):   optimise fx/fy + k1/k2.
///   Phase 3 (≥ phase3_min):             optimise all (cx/cy, k3, p1, p2 unlocked).
///
/// fix_mask_for(n) returns the FixIntrinsicsMask to pass as partial_intr_fix
/// to run_global_ba().  It supersedes the old intrinsics_k3p12_free_min_images field.
struct IntrinsicsSchedule {
  // ── Phase unlock thresholds ───────────────────────────────────────────────
  int phase1_min_images = 3;    ///< Start optimising fx/fy.
  int phase2_min_images = 10;   ///< Also unlock k1/k2.
  int phase3_min_images = 100;  ///< Unlock cx/cy + k3/p1/p2 (full intrinsics).

  // ── Progressive freeze (local-BA phase) ──────────────────────────────────
  bool progressive_freeze = true;
  int freeze_min_images = 30;           ///< Level-1 gate: min registered images per camera.
  double freeze_delta_focal = 1e-3;     ///< Relative focal convergence threshold.
  double freeze_delta_pp = 0.5;         ///< Principal-point convergence threshold (px).
  double freeze_delta_dist = 1e-4;      ///< Distortion L1 convergence threshold.
  int freeze_stable_rounds = 2;         ///< Consecutive stable BAs needed to confirm freeze.

  // ── Low-frequency recalibration ───────────────────────────────────────────
  int recalib_every_n_periodic = 4;    ///< Full-recalib every N mid-freq BAs (0 = disabled).

  // ── Focal soft prior ──────────────────────────────────────────────────────
  double focal_prior_weight = 0.0;     ///< Tikhonov weight on focal deviation (0 = disabled).

  /// Returns the partial_intr_fix bitmask for run_global_ba() based on registration count.
  uint32_t fix_mask_for(int n_registered) const;
};

/// Options for local BA.
struct LocalBAOptions {
  int switch_after_n_images = 20;                        ///< Switch from global to local BA.
  int window = 20;                                       ///< Window size for kWindow strategy.
  bool by_connectivity = true;                           ///< Rank by co-visibility, not index.
  LocalBAStrategy strategy = LocalBAStrategy::kColmap;
  int colmap_max_variable_images = 6;                    ///< Max variable cameras in kColmap.
  int neighbor_k = 5;                                    ///< Anchor neighbors in kBatchNeighbor.
  bool skip = false;                                     ///< Skip local BA (object-scan mode).
  int max_iterations = 25;
};

/// Options for global BA.
struct GlobalBAOptions {
  bool enabled = true;
  bool optimize_intrinsics = true;
  int optimize_intrinsics_min_images = 10;  ///< Gate: don't touch intrinsics below this count.
  int max_iterations = 50;
  int every_n_images = 1;              ///< Early-phase: run global BA every N registrations.
  int periodic_every_n_images = 20;    ///< Local-BA phase: mid-freq global BA every N images.
  BASolverOverrides solver_overrides;  ///< Ceres solver parameter overrides.
};

/// Options for outlier rejection.
struct OutlierOptions {
  double threshold_px = 4.0;          ///< Fixed outlier threshold (px).
  double adaptive_factor = 2.0;       ///< Effective thr = max(threshold_px, rmse * factor).
  int max_iterations = 5;             ///< Max BA+reject rounds after each BA call.
  int min_for_retry = 30;             ///< Skip extra BA round if newly rejected < this.
  int min_registered_images = 10;     ///< Don't reject outliers until this many are registered.
  double min_angle_deg = 2.0;         ///< Reject if max parallax angle < this.
  bool sigma_filter = false;          ///< Extra pass at rmse×3 after reject loop.
  int final_max_rounds = 10;          ///< Max rounds in the final global-BA reject loop.
  // ── Two-pass coarse/refine rejection ─────────────────────────────────────
  /// When true, every global BA is preceded by a coarse pass (all intrinsics fixed,
  /// MAD-based adaptive threshold) that cleans gross outliers before opening intrinsics.
  bool   two_pass_rejection = true;
  /// Coarse-pass MAD multiplier: threshold = median(e) + coarse_mad_k * 1.4826 * MAD(e).
  /// More aggressive than the fine-pass adaptive_factor (typical fine ≈ 2.0, coarse ≈ 2.5).
  double coarse_mad_k       = 2.5;
  /// Maximum BA + MAD-reject rounds in the coarse pass.
  int    coarse_max_rounds  = 5;
};

/// Options for triangulation and periodic re-triangulation.
struct TriangulationOptions {
  double min_angle_deg = 2.0;             ///< Min max pairwise angle for a triangulated point.
  int retriangulation_every_n_iters = 3;  ///< Periodic re-triangulation every N SfM iterations.
};

// ─────────────────────────────────────────────────────────────────────────────
// IncrementalSfMOptions — hierarchical pipeline configuration
// ─────────────────────────────────────────────────────────────────────────────

struct IncrementalSfMOptions {
  InitPairOptions      init;
  ResectionOptions     resection;
  IntrinsicsSchedule   intrinsics;
  LocalBAOptions       local_ba;
  GlobalBAOptions      global_ba;
  OutlierOptions       outlier;
  TriangulationOptions triangulation;
};

// ─────────────────────────────────────────────────────────────────────────────
// Preset factory functions
// ─────────────────────────────────────────────────────────────────────────────

/// UAV nadir aerial photogrammetry (default local_ba_strategy = kColmap).
IncrementalSfMOptions make_aerial_preset();
/// Circumferential / object-scan captures (skip_local_ba, more aggressive BA).
IncrementalSfMOptions make_object_scan_preset();
/// Conservative general-purpose preset.
IncrementalSfMOptions make_general_preset();

// ─────────────────────────────────────────────────────────────────────────────
// (Legacy flat-struct fields — kept as a migration reference; NOT part of
// the struct above.  Delete this block once all callers are updated.)
// ─────────────────────────────────────────────────────────────────────────────
// NOTE: the following grouped fields were previously in IncrementalSfMOptions:
//   init.*         ← was: min_tracks_after, init_ba_rmse_max, init_min_angle_deg, …
//   resection.*    ← was: resection_min_inliers, resection_min_3d2d_count, …
//   intrinsics.*   ← was: intrinsics_progressive_freeze, intrinsics_freeze_*, …
//                       intrinsics_k3p12_free_min_images (→ intrinsics.fix_mask_for())
//   local_ba.*     ← was: local_ba_after_n_images, local_ba_window, skip_local_ba, …
//   global_ba.*    ← was: do_global_ba, max_global_ba_iterations, ba_gradient_tolerance, …
//   outlier.*      ← was: outlier_threshold_px, reject_min_registered_images, …
//   triangulation.*← was: min_tri_angle_deg, retriangulation_every_n_iters
// ─────────────────────────────────────────────────────────────────────────────

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
