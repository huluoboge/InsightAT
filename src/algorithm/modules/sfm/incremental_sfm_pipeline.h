/**
 * @file  incremental_sfm_pipeline.h
 * @brief Incremental SfM pipeline (index-only, cameras + image_to_camera_index, no IdMapping).
 *
 * Initial pair loop: try candidate pairs from ViewGraph until one succeeds (two-view triang + BA +
 * check). Resection loop, batch triangulation, re-triangulation, optional local/global BA.
 */

#pragma once

#include "../camera/camera_types.h"
#include "incremental_triangulation.h"
#include "resection.h"
#include "resection_batch.h"
#include "track_store.h"
#include "view_graph.h"
#include <Eigen/Core>
#include <cmath>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace insight {
namespace sfm {

/**
 * Run initial-pair loop: get candidate pairs (twoview_ok && stable) sorted by score,
 * for each try load geo, triangulate two-view tracks, two-view BA, reject/filter, check count/RMSE.
 * Uses cameras[image_to_camera_index[im]] for intrinsics; no intrinsics_per_image.
 *
 * @param view_graph       Built from pairs (used for second-image candidate ranking).
 * @param store            Track store (from IDC); two-view tracks will be triangulated and updated.
 * @param cameras          Current camera intrinsics (may be updated by BA later).
 * @param image_to_camera_index  image_to_camera_index[i] = camera index for image i.
 * @param min_tracks_for_intital_pair Minimum valid two-view tracks after filter to accept the pair.
 * @param initial_im0_out  Output: first image index of chosen pair.
 * @param initial_im1_out  Output: second image index of chosen pair.
 * @param poses_R_out      Output: poses_R for all images; only [im0],[im1] filled (world = im0).
 * @param poses_C_out      Output: poses_C for all images.
 * @param registered_out   Output: registered flags; only [im0],[im1] set true.
 * @return true if a pair was chosen and store/poses updated; false if none succeeded.
 */
bool run_initial_pair_loop(const ViewGraph& view_graph, TrackStore* store,
                           const std::vector<camera::Intrinsics>& cameras,
                           const std::vector<int>& image_to_camera_index,
                           int min_tracks_for_intital_pair, int min_num_inliers,
                           double max_forward_motion, double min_angle_deg,
                           double min_median_angle_deg,
                           uint32_t* initial_im0_out, uint32_t* initial_im1_out,
                           std::vector<Eigen::Matrix3d>* poses_R_out,
                           std::vector<Eigen::Vector3d>* poses_C_out,
                           std::vector<bool>* registered_out,
                           int max_first_images = 100, int max_second_images = 50);

// ─── BA (build from Store, solve, write back) ───────────────────────────────

/// Optional per-call Ceres solver overrides for run_global_ba.
/// Zero / 0.0 fields are ignored — built-in defaults apply.
struct BASolverOverrides {
  double gradient_tolerance = 0.0;       ///< Ceres gradient_tolerance.  0 = Ceres default.
  double function_tolerance = 0.0;       ///< Ceres function_tolerance.  0 = Ceres default.
  double parameter_tolerance = 0.0;      ///< Ceres parameter_tolerance. 0 = Ceres default.
  int dense_schur_max_variable_cams = 0; ///< DENSE↔SPARSE Schur threshold. 0 = built-in (30).
  int max_num_iterations = 0;    ///< 0 = use max_iterations param of run_global_ba / local BA.
  double huber_loss_delta = 0.0; ///< Huber loss δ (px). 0 = BAInput default (4.0 px).
  double tikhonov_lambda = 0.0;  ///< Tikhonov regularization lambda. 0 = disabled. Passed to BAInput::tikhonov_lambda.
  /// Ceres `Solver::Options::num_threads`. 0 = use BAInput default (hardware concurrency).
  int num_threads = 0;
};

/**
 * Run global BA: build BAInput from store for all registered images, solve, write back poses and
 * points.
 * @param cameras Current intrinsics (required for BA). If cameras_in_out is non-null, optimised
 * intrinsics are written back when optimize_intrinsics is true.
 * @param anchor_image Global image index to fix as the coordinate-system anchor (initial pair
 *   first image). If -1, the first BA image (lowest registered index) is fixed.
 * @param initial_pair_global_im1 Global index of the initial pair’s second image. If ≥ 0, adds an
 *   internal soft prior ‖C_anchor−C_im1‖ → ‖C_anchor−C_im1‖ at BA entry (snapshot; anchor pose
 *   fixed). -1 = off.
 */
bool run_global_ba(TrackStore* store, std::vector<Eigen::Matrix3d>* poses_R,
                   std::vector<Eigen::Vector3d>* poses_C, const std::vector<bool>& registered,
                   const std::vector<int>& image_to_camera_index,
                   const std::vector<camera::Intrinsics>& cameras,
                   std::vector<camera::Intrinsics>* cameras_in_out, bool optimize_intrinsics,
                   int max_iterations, double* rmse_px_out, int anchor_image = -1,
                   int max_observations_per_track = 0,
                   double focal_prior_weight = 0.0, const BASolverOverrides& solver_overrides = {},
                   const std::vector<uint32_t>* partial_intr_fix_per_cam = nullptr,
                   int initial_pair_global_im1 = -1,
                   const std::vector<bool>* precomputed_image_stable = nullptr,
                   double skip_2deg_min_angle_score = 0.0);

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
                  const std::vector<int>* indices_to_optimize = nullptr, int anchor_image = -1,
                  int max_observations_per_track = 0,
                  const BASolverOverrides& overrides = {});

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
                         std::vector<Eigen::Vector3d>* poses_C, const std::vector<bool>& registered,
                         const std::vector<int>& image_to_camera_index,
                         const std::vector<camera::Intrinsics>& cameras,
                         const std::vector<int>& batch, int max_variable_cameras,
                         int max_iterations, double* rmse_px_out, int max_observations_per_track = 0,
                         const BASolverOverrides& overrides = {}, int scene_num_registered = 0,
                         double constant_cam_gross_outlier_px = 0.0,
                         int constant_cam_gross_outlier_min_registered = 0);

/**
 * Neighbor-anchored local BA (kBatchNeighbor strategy).
 *   Variable cameras : batch (newly resected, PnP poses).
 *   Variable 3D points: new_track_ids (just triangulated this iteration).
 *   Constant cameras : per-batch-image top-neighbor_k neighbors by shared triangulated point count
 *                      (union across all batch images, excluding batch itself).
 *   Constant 3D points: old triangulated tracks visible from ≥1 batch AND ≥1 constant camera.
 * Intrinsics are not optimised.
 */
bool run_local_ba_batch_neighbor(
    TrackStore* store, std::vector<Eigen::Matrix3d>* poses_R, std::vector<Eigen::Vector3d>* poses_C,
    const std::vector<bool>& registered, const std::vector<int>& image_to_camera_index,
    const std::vector<camera::Intrinsics>& cameras, const std::vector<int>& batch,
    const std::vector<int>& new_track_ids, int neighbor_k, int max_iterations, double* rmse_px_out,
    int max_observations_per_track = 0,
    const BASolverOverrides& overrides = {}, int scene_num_registered = 0,
    double constant_cam_gross_outlier_px = 0.0,
    int constant_cam_gross_outlier_min_registered = 0);

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
  int min_tracks_for_intital_pair = 50; ///< Min inlier tracks after MAD filter to accept pair.
  int min_num_inliers = 100;            ///< Min E-RANSAC inliers (COLMAP-style gate).
  double max_forward_motion = 0.95;     ///< Reject near-pure forward motion: |tz|/||t|| must be < this.
  int max_first_images = 100;           ///< Max first-image candidates to try.
  int max_second_images = 50;           ///< Max second-image candidates per first image.
  double ba_rmse_max = 10.0;            ///< Max BA RMSE (px) to accept pair.
  double outlier_threshold_px = 4.0;    ///< MAD floor (fallback when distribution is very narrow).
  double min_angle_deg = 2.0;           ///< Min triangulation angle per point; max is hard-coded to 60°.
  /// Minimum MEDIAN triangulation angle (degrees) of accepted inlier tracks after BA.
  /// Rejects forward-motion adjacent-frame pairs whose per-point angles individually pass
  /// min_angle_deg but collectively form a near-degenerate narrow-baseline geometry.
  /// Typical aerial surveys: set to 5.0°; lower for very-high-overlap data.
  double min_median_angle_deg = 5.0;
};

/// Options for the incremental resection loop (one new image per iteration).
struct ResectionOptions {
  ResectionBackend backend = ResectionBackend::kPoseLib; ///< Absolute-pose backend.
  int min_inliers = 9;     ///< Min PnP RANSAC inliers to accept resection (3× P3P min-sample).
  int min_3d2d_count = 15; ///< Min 3D-2D correspondences to list a candidate.
  /// Stability gate: min PnP inlier ratio (inliers / total 3D-2D correspondences).
  /// Helps reject large-support false positives (e.g. 9 inliers out of 400+ correspondences).
  double min_inlier_ratio = 0.02;
  /// For large scenes, use a stricter inlier-ratio gate once enough cameras are registered.
  double min_inlier_ratio_large_scene = 0.05;
  int large_scene_min_images = 100;
  int large_scene_min_registered = 20;
  /// Optional second pass after PnP inlier writeback: drop obs with reproj error > this (px). 0 =
  /// off.
  double post_resection_reproj_thresh_px = 0.0;
  /// First-sort tier: prefer candidates with normalized VisibilityPyramid coverage ≥ this (see
  /// COLMAP scene/visibility_pyramid). Typical range 0.01–0.05; legacy 3×3 bbox metric used ~0.33.
  float min_visibility_coverage = 0.02f;
  /// Pyramid depth (COLMAP default 6 → finest grid 2^6 per side).
  int visibility_pyramid_levels = 6;
  /// GpuResectionContext `max_pts`: upper bound on 3D–2D pairs **per image** for CUDA LM refine
  /// (one image at a time). Typically set to ~2× your per-image feature extraction cap (e.g. 10k
  /// features → 20000). Must be ≥ the largest `nk` passed to gpu_resection_upload for any image.
  int max_gpu_resection_points_per_image = 20000;
};

/// Progressive intrinsics unlock schedule + freeze policy.
///
/// Per **camera model**: phases use that camera's registered-image count (see fix_masks_per_camera).
///   Phase 0 (< phase1_min):              fix ALL.
///   Phase 1 [phase1_min, phase2_min):    optimise **fx + k1**; sigma, cx, cy, k2, k3, p1, p2 fixed.
///   Phase 2 [phase2_min, phase3_min):    optimise **fx + k1 + k2**; sigma, cx, cy, k3, p1, p2 fixed.
///   Phase 3 (≥ phase3_min):             all intrinsics free.
///
/// fix_mask_for(n) returns the FixIntrinsicsMask to pass as partial_intr_fix
/// to run_global_ba().  It supersedes the old intrinsics_k3p12_free_min_images field.
struct IntrinsicsSchedule {
  // ── Phase unlock thresholds ───────────────────────────────────────────────
  int phase1_min_images = 3;   ///< Start optimising fx + k1 (distortion k2+ still fixed).
  int phase2_min_images = 10;  ///< Also unlock k2 (still fix sigma, cx/cy, k3, p1, p2 until phase 3).
  int phase3_min_images = 50;  ///< Unlock cx/cy + k3/p1/p2 (full intrinsics).

  double focal_prior_weight = 100.0; ///< Tikhonov weight on focal deviation (0 = disabled).

  /// Returns the partial_intr_fix bitmask for run_global_ba() based on registration count.
  uint32_t fix_mask_for(int n_registered) const;

  /// Returns a per-camera vector of partial_intr_fix bitmasks.
  /// Each camera's phase is determined by its own registered image count (images of that
  /// specific camera that have been successfully registered), rather than the total.
  std::vector<uint32_t> fix_masks_per_camera(const std::vector<bool>& registered,
                                             const std::vector<int>& image_to_camera_index,
                                             int n_cameras) const;
};

/// Options for local BA.
struct LocalBAOptions {
  int switch_after_n_images = 200; ///< Switch from global to local BA.
  int window = 20;                ///< Window size for kWindow strategy.
  bool by_connectivity = true;    ///< Rank by co-visibility, not index.
  LocalBAStrategy strategy = LocalBAStrategy::kColmap;
  int colmap_max_variable_images = 6; ///< Max variable cameras in kColmap.
  int neighbor_k = 5;                 ///< Anchor neighbors in kBatchNeighbor.
  /// When true, run the local-BA phase after switch_after_n_images (periodic global + local BA).
  /// Default false: global-only loop until you opt in.
  bool enable = false;
  int max_iterations = 250;
  /// Cap per-track observations inserted into local BA (0 = no cap).
  int max_observations_per_track = 8;

  /// After local BA, mark **constant** (frozen-pose) cameras' observations as kRestorable deleted when
  /// reproj error exceeds this (px). 0 = disabled. Intended for late incremental stages: intrinsics are
  /// stable and gross outliers on neighbors poison the next periodic global. Early phase: keep 0 so we
  /// do not strip observations that look bad only because fx/distortion are still moving.
  double constant_cam_gross_outlier_px = 0.0;
  /// Gate for `constant_cam_gross_outlier_px`: require `scene_num_registered >= this`. If 0 while
  /// `constant_cam_gross_outlier_px > 0`, `switch_after_n_images` is used instead.
  int constant_cam_gross_outlier_min_registered = 0;
};

/// Effective minimum registration count for LocalBAOptions::constant_cam_gross_outlier_px (see struct).
inline int effective_local_ba_gross_constant_cam_min_reg(const LocalBAOptions& o) {
  if (o.constant_cam_gross_outlier_px <= 0.0)
    return 0;
  return o.constant_cam_gross_outlier_min_registered > 0 ? o.constant_cam_gross_outlier_min_registered
                                                         : o.switch_after_n_images;
}

/**
 * Dispatch one local-BA solve according to `opts.strategy` (kColmap / kBatchNeighbor / kWindow).
 * Intrinsics are not optimised. Extend here when adding LocalBAStrategy values.
 */
bool run_local_ba_dispatch(const LocalBAOptions& opts, int anchor_image, int scene_num_registered,
                           TrackStore* store, std::vector<Eigen::Matrix3d>* poses_R,
                           std::vector<Eigen::Vector3d>* poses_C,
                           const std::vector<bool>& registered,
                           const std::vector<int>& image_to_camera_index,
                           const std::vector<camera::Intrinsics>& cameras,
                           const std::vector<int>& batch, const std::vector<int>& new_track_ids,
                           const BASolverOverrides& ov, double* rmse_px_out);

/// Options for global BA.
struct GlobalBAOptions {
  bool enabled = true;
  bool optimize_intrinsics = true;
  int max_iterations = 500;
  int every_n_images = 1; ///< Early-phase: run global BA every N registrations.
  /// Late phase only (after local_ba.switch_after_n_images): fixed-period fallback when linear
  /// spacing is off. When periodic_global_linear_spacing is true and (spacing_a>0 || spacing_b>0),
  /// spacing uses spacing_a + spacing_b·n instead.
  int periodic_every_n_images = 50;
  /// If true and at least one of spacing_a/spacing_b is positive, next periodic global when
  /// n_registered ≥ milestone where milestone advances by ceil(max(1, spacing_a + spacing_b·n))
  /// after each successful periodic solve (first milestone = switch_after_n_images).
  bool periodic_global_linear_spacing = true;
  /// Linear gap after a periodic global at registration count n: gap = ceil(max(1, a + b·n)).
  /// At n≈100 with the defaults below, gap≈28 (was a=40,b=0.1 → gap≈50): fewer local-only
  /// stretches before the next full global, which tends to avoid ill-conditioned joint Schur steps.
  double periodic_global_spacing_a = 22.0;
  double periodic_global_spacing_b = 0.06;
  BASolverOverrides solver_overrides; ///< Ceres solver parameter overrides.
  /// Conservative early phase: limit to 1 new camera per SfM iteration (identical to local-BA
  /// behaviour) until this many images are registered.  0 = disabled.  Improves stability when
  /// intrinsics are still converging in the first ~100 frames.
  int early_phase_max_cameras = 100;
  /// Phase boundary: run full **global** BA on every registration while `n_registered <` this value.
  /// Default 41 → global every new registration for the first 40 registered images; at n≥41 enter
  /// mid-phase (linear or fixed-step globals + local BA) until `switch_after_n_images`.
  /// 0 = disabled (global-BA-only until switch_after_n_images when local BA is off; see pipeline).
  int early_phase_global_only_images = 41;

  /// Mid phase [early_phase_global_only_images, switch_after_n_images): when true and (a>0||b>0),
  /// full global BA runs when n_registered ≥ milestone; milestone += ceil(max(1,a+b·n)) after each
  /// successful scheduled global. When false, use every_n_images modulo (legacy).
  bool mid_phase_global_linear_spacing = true;
  double mid_global_spacing_a = 5.0;
  double mid_global_spacing_b = 0.12;

  // ── 2-degree track pruning in global BA ──────────────────────────────────────────────────────
  /// If true, 2-view tracks (visible in exactly 2 registered images) whose both observing images
  /// are already "stable" and whose parallax angle is good are excluded from BA points/obs.
  /// They remain in TrackStore and are re-triangulated by the normal kFullScan pass after BA.
  /// Default false (off) so existing behaviour is preserved until explicitly enabled.
  bool skip_2degree_tracks = false;
  /// Minimum sin²(parallax_angle) for a 2-degree track to qualify as skippable.
  /// Tracks with S < threshold have weak geometry and are kept in BA.
  /// sin²(1.8°)≈0.001  sin²(2.6°)≈0.002  sin²(3.1°)≈0.003
  double skip_2degree_min_angle_score = 0.002;
  /// An image is considered "stable" if it has at least this many valid triangulated observations.
  int skip_2degree_min_stable_obs = 50;
  /// An image is "stable" if at least this fraction of its observations have S ≥ min_angle_score.
  double skip_2degree_stable_ratio = 0.5;

  // ── Adaptive grid-NMS BA subset selection ────────────────────────────────────────────────────
  /// Enable adaptive grid-NMS subset selection: per image, cells of adaptive size each keep the
  /// highest-scoring track. Selected tracks enter BA; others get kSkipFromBA and are later
  /// optimised by retri_skipped_tracks_fixed_pose (if enabled). Stacks with skip_2degree_tracks.
  bool ba_grid_subset = false;
  /// Target number of selected tracks per image. Adaptive cell size = sqrt(W*H/target).
  /// 4000×3000 imagery: target=3000 → ~63 px cells; target=5000 → ~49 px cells.
  int ba_grid_target_per_image = 3000;
  /// Score weight for track degree: score = w_deg×clamp(deg/deg_cap,0,1) + w_score×S.
  double ba_grid_w_degree = 0.6;
  /// Score weight for geometric stability S = sin²(θ) (1.0 for degree≥3 tracks).
  double ba_grid_w_score = 0.4;
  /// Degree cap for score saturation (degree ≥ cap → full weight).
  int ba_grid_degree_cap = 8;
  /// Recompute the grid-NMS subset every N global-BA calls. 1 = always (safest). Increase to
  /// amortise selection cost when quality is confirmed.
  int ba_grid_reselect_every_n = 1;
  /// Cap per-track observations inserted into global BA (0 = no cap).
  int max_observations_per_track = 8;

  // ── Fixed-pose point optimisation for kSkipFromBA tracks ─────────────────────────────────────
  /// After global BA, run a Ceres solve that fixes ALL poses+intrinsics and only optimises the
  /// 3D positions of kSkipFromBA tracks. Independent per-point → near-linear in thread count.
  bool ba_fixed_pose_optimize_skipped = false;
  /// Max Ceres iterations for the fixed-pose skipped-track solve.
  int ba_fixed_pose_max_iterations = 30;

  // ── Intermediate-round loose tolerances ──────────────────────────────────────────────────────
  /// When the outlier-rejection fine loop runs round r>0, the pose is already established and we
  /// only need to re-converge after deleting bad observations.  Round r=0 always uses the full
  /// solver_overrides / Ceres defaults.  Set intermediate_loose_after_images=0 to disable.
  ///
  /// Profiling on 301-image data showed that keeping this at 31 caused 37% of all Ceres time to
  /// be spent on early-phase (n<31) cleanup rounds using full strict tolerances.  Setting it to 3
  /// applies loose tolerances from the very first multi-round BA and typically saves 15–20s.
  int intermediate_loose_after_images = 3; ///< Gate: n_registered >= this to apply loose tol.
  /// function_tolerance for intermediate rounds (r>0). 0.0 = use Ceres default (1e-6).
  double intermediate_function_tolerance = 1e-4;
  /// Max iterations cap for intermediate rounds (r>0). 0 = use max_iterations.
  int intermediate_max_iterations = 20;
};

/// Advance linear periodic-global milestone after a successful solve at `num_registered`.
inline void bump_periodic_global_milestone_linear(int num_registered, const GlobalBAOptions& g,
                                                  int* next_milestone) {
  const double raw =
      g.periodic_global_spacing_a + g.periodic_global_spacing_b * static_cast<double>(num_registered);
  const int gap = std::max(1, static_cast<int>(std::ceil(raw)));
  *next_milestone = num_registered + gap;
}

/// Whether to run an extra full global BA after local BA this iteration (late phase cadence).
inline bool periodic_global_ba_should_run_after_local(
    int num_registered, bool in_late_phase, const GlobalBAOptions& g, int switch_after_n_images,
    int* next_milestone) {
  if (!g.enabled || !in_late_phase)
    return false;
  if (g.periodic_global_linear_spacing &&
      (g.periodic_global_spacing_a > 0.0 || g.periodic_global_spacing_b > 0.0)) {
    if (*next_milestone < 0)
      *next_milestone = switch_after_n_images;
    return num_registered >= *next_milestone;
  }
  return g.periodic_every_n_images > 0 &&
         (num_registered % g.periodic_every_n_images == 0);
}

inline void bump_mid_global_milestone_linear(int num_registered, const GlobalBAOptions& g,
                                             int* next_milestone) {
  const double raw =
      g.mid_global_spacing_a + g.mid_global_spacing_b * static_cast<double>(num_registered);
  const int gap = std::max(1, static_cast<int>(std::ceil(raw)));
  *next_milestone = num_registered + gap;
}

/// Median-based scene normalization (tracks + registered camera centres). Default 0 = off.
/// When enabled, call before periodic global BA so BA runs in normalized world units; no inverse
/// transform is applied (metric is in normalized units).
struct SceneNormalizationOptions {
  /// 0 = disabled. If > 0, normalize when `sfm_iter % N == 0` (see run_incremental_sfm_pipeline).
  int normalize_scene_every_n_sfm_iters = 1;
  /// 0 = disabled. If > 0, normalize when `num_registered >= M` and `num_registered % M == 0`.
  int normalize_scene_every_n_registered_images = 1;
};

/// Options for outlier rejection and iterative Huber-BA-based coarse detection.
struct OutlierOptions {
  double threshold_px = 4.0;      ///< Hard floor for fine-phase MAD rejection thresholds (px).
  double huber_delta_lo_px = 0.5; ///< Lower clip for MAD-derived Huber δ (fine phase, px).
  double huber_delta_hi_px = 3.0; ///< Upper clip for MAD-derived Huber δ (fine phase, px).
  double mad_k = 2.5;             ///< Fine phase: hard_thr = median(e) + mad_k × 1.4826 × MAD(e).
  double min_angle_deg = 0.50;    ///< Reject observation if max parallax angle < this (fine phase).
  double max_angle_deg = 120.0;   ///< Reject observation if max parallax angle > this (fine phase).
  double max_depth_factor = 200.0; ///< Reject if depth > median_scene_depth × factor (0 = off).
  int min_for_retry = 100;         ///< Continue iterating when newly rejected >= this.
  int min_registered_images = 2;   ///< Minimum registered images before outlier rejection runs.
  int max_rounds = 10;             ///< Max rounds in the BA outlier-rejection loop.
};
/// Options for triangulation and periodic re-triangulation.
struct TriangulationOptions {
  double min_angle_deg = 0.5;   ///< Min max pairwise angle for a triangulated point.
  double max_angle_deg = 120.0; ///< Max max pairwise angle (beyond which we assume outlier).
  double min_baseline_depth_ratio = 0.01; ///< Min baseline-to-depth ratio for triangulated points.
  int retriangulation_every_n_iters = 3;  ///< Periodic kPendingOnly re-triangulation every N SfM iterations.

  /// Periodic kFullScan re-triangulation every N SfM iterations, independent of global BA.
  /// kFullScan visits ALL unresolved tracks and can recover tracks whose supporting cameras
  /// were registered in recent iterations but whose cross-image tracks were not yet
  /// triangulable at the time of the last full scan.  Set to 0 to disable.
  /// Typical cost: ~20 ms at 200 registered images, ~600 ms at 600 registered images.
  /// Setting = 10 adds ~30 s overhead for a 700-image dataset; setting = 5 adds ~60 s.
  int full_scan_every_n_iters = 10;

  /// After GN: reject view if reproj > this (px). Loosen if too few points pass incremental tri
  /// (logs show many `two_reproj` / robust fails at 4px while BA RMSE is ~0.5px).
  double commit_reproj_px = 16.0;

  // ── Intrinsics-change observation restoration ─────────────────────────────
  /// When true, after each global BA where any camera's focal length changed by more than
  /// intrinsics_change_restore_threshold, re-evaluate kRestorable deleted observations using
  /// the updated intrinsics and restore those whose reprojection error is now ≤ restore_reproj_px.
  bool enable_intrinsics_change_restore = true;
  /// Relative focal-length change threshold (|Δfx/fx|) that triggers a restore pass.
  double intrinsics_change_restore_threshold = 0.02;
  /// Reprojection-error ceiling (px) for restoring a previously-deleted kRestorable observation.
  /// Should match the typical BA outlier-rejection threshold (~4 px) so that observations
  /// rejected by wrong intrinsics/distortion are given a fair second chance.
  double restore_reproj_px = 4.0;

  /// After successful local BA: run_retriangulation with kNewImages on new_registered_image_indices.
  bool enable_post_local_ba_retriangulation = true;
  /// After any global BA in the main loop: run_retriangulation with kFullScan (heavy).
  bool enable_full_scan_after_global_ba = true;
};

// ─────────────────────────────────────────────────────────────────────────────
// IncrementalSfMOptions — hierarchical pipeline configuration
// ─────────────────────────────────────────────────────────────────────────────

/// Debug options: per-iteration snapshot callback for diagnosing pose drift.
/// The callback is invoked at the end of each SfM iteration (after BA + retriangulation)
/// when snapshot_every_n_iters > 0 and sfm_iter % snapshot_every_n_iters == 0.
/// Signature: (sfm_iter, num_registered, poses_R, poses_C, registered, store)
struct DebugOptions {
  /// Invoke the snapshot callback every N SfM iterations.  0 = disabled.
  int snapshot_every_n_iters = 0;
  /// User-supplied callback; called on matching iterations.  May be empty (no-op).
  std::function<void(int /*sfm_iter*/, int /*num_registered*/,
                     const std::vector<Eigen::Matrix3d>& /*poses_R*/,
                     const std::vector<Eigen::Vector3d>& /*poses_C*/,
                     const std::vector<bool>& /*registered*/, const TrackStore& /*store*/)>
      on_snapshot;
};

struct IncrementalSfMOptions {
  InitPairOptions init;
  ResectionOptions resection;
  IntrinsicsSchedule intrinsics;
  LocalBAOptions local_ba;
  GlobalBAOptions global_ba;
  SceneNormalizationOptions scene_normalization;
  OutlierOptions outlier;
  TriangulationOptions triangulation;
  DebugOptions debug; ///< Per-iteration debug snapshots (disabled by default).

  /// Early stop when registered images reach this cap (including initial pair).
  /// 0 = disabled (run full incremental reconstruction).
  int max_registered_images = 0;

  /// Number of OpenMP threads for parallel loops (outlier rejection, select_ba_subset, etc.).
  /// -1 (default) means use the system/OMP default (typically all hardware threads).
  /// Set to a positive value to cap parallelism, e.g. when multiple pipeline instances run
  /// concurrently and you want to divide cores evenly.
  int omp_num_threads = -1;
};

/// One BA + iterative outlier-rejection call (global or local). Keeps
/// `run_incremental_sfm_pipeline` readable; fill only fields that apply (e.g. local batches when
/// use_global_ba is false). Ceres overrides are taken from opts.global_ba.solver_overrides; coarse
/// gross-outlier BA uses fixed tolerances inside run_coarse_outlier_rejection_global_ba.
// struct BaOutlierCall {
//   bool use_global_ba = true;
//   bool optimize_intrinsics = true;
//   // uint32_t partial_intr_fix = 0;
//   double focal_prior_weight = 0.0;
//   /// Per-camera: if true, that camera's intrinsics stay fixed in global BA (progressive freeze).
//   std::vector<int> local_ba_batch;
//   std::vector<int> local_ba_new_tracks;
//   /// If non-empty, Huber δ is estimated from residuals on these images only; if empty, all
//   images. std::vector<int> huber_residual_image_filter;
//   /// If set, overrides opts.outlier.enable_coarse_outlier_rejection for this call only.
//   std::optional<bool> enable_coarse_outlier_rejection;
// };

bool run_ba_with_outlier_detection(TrackStore* store, std::vector<Eigen::Matrix3d>* poses_R,
                                   std::vector<Eigen::Vector3d>* poses_C,
                                   const std::vector<bool>& registered,
                                   const std::vector<int>& image_to_camera_index,
                                   std::vector<camera::Intrinsics>* cameras, int anchor_image,
                                   int num_registered, const IncrementalSfMOptions& opts,
                                   double* rmse_px_out, int initial_pair_im1_global = -1);

// ─────────────────────────────────────────────────────────────────────────────
// (Legacy flat-struct fields — kept as a migration reference; NOT part of
// the struct above.  Delete this block once all callers are updated.)
// ─────────────────────────────────────────────────────────────────────────────
// NOTE: the following grouped fields were previously in IncrementalSfMOptions:
//   init.*         ← was: min_tracks_for_intital_pair, init_ba_rmse_max, init_min_angle_deg, …
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
 * @param pairs_json_path     Path to pairs JSON (used to build view graph only when tracks IDC has
 * no embedded view_graph_pairs; schema 1.1 from isat_tracks embeds the graph).
 * @param geo_dir             Directory of .isat_geo files (same fallback as pairs_json).
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
bool run_incremental_sfm_pipeline(const std::string& tracks_idc_path,
                                  const std::string& pairs_json_path, const std::string& geo_dir,
                                  std::vector<camera::Intrinsics>* cameras,
                                  const std::vector<int>& image_to_camera_index,
                                  const IncrementalSfMOptions& opts, TrackStore* store_out,
                                  std::vector<Eigen::Matrix3d>* poses_R_out,
                                  std::vector<Eigen::Vector3d>* poses_C_out,
                                  std::vector<bool>* registered_out);

} // namespace sfm
} // namespace insight
