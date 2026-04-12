/**
 * @file  incremental_triangulation.h
 * @brief Incremental SfM: multiview triangulation from TrackStore + registered poses.
 */

#pragma once

#include "../camera/camera_types.h"
#include "track_store.h"
#include <Eigen/Core>
#include <unordered_set>
#include <vector>

namespace insight {
namespace sfm {

/**
 * Triangulate tracks that have observations in the given new images and in already-registered
 * images, but no triangulated xyz yet. Uses current cameras for undistortion.
 * Per track: one pipeline — `robust_triangulate_point_multiview` (handles N=2 and N≥3), then GN on
 * all views, then the same reproj threshold for every view; observations above the threshold are
 * marked deleted and the track is retried until success or fewer than two views remain.
 * @param min_tri_angle_deg  Minimum max pairwise ray angle (degrees); reject degenerate points.
 * @param commit_reproj_px   Strategy B gate: max per-view reproj after GN (also RANSAC inlier band
 *                           for multiview). Same as RobustTriangulationOptions::ransac_inlier_px.
 * @return Number of tracks newly triangulated.
 */
int run_batch_triangulation(TrackStore* store, const std::vector<int>& new_registered_image_indices,
                            const std::vector<Eigen::Matrix3d>& poses_R,
                            const std::vector<Eigen::Vector3d>& poses_C,
                            const std::vector<bool>& registered,
                            const std::vector<camera::Intrinsics>& cameras,
                            const std::vector<int>& image_to_camera_index,
                            double min_tri_angle_deg = 0.5,
                            std::vector<int>* new_track_ids_out = nullptr,
                            double commit_reproj_px = 6.0);

/// Ceres-free robust triangulation: N=2 uses DLT + inlier reproj check + GN; N≥3 uses RANSAC over
/// two-view pairs, inlier voting, DLT on inliers, then GN. Used as the initializer for incremental
/// track triangulation.
struct RobustTriangulationOptions {
  double ransac_inlier_px = 16.0;
  int ransac_max_pair_samples = 500;
  int min_inlier_views = 2;
  // double min_tri_angle_deg = 2.0;
  double min_tri_angle_deg = 0.5;
  double max_tri_angle_deg = 120.0;
  int gn_max_iterations = 10;
  double gn_tolerance = 1e-8;
};

struct RobustTriangulationResult {
  bool success = false;
  Eigen::Vector3d X = Eigen::Vector3d::Zero();
  /// Per input view; true if view agrees with winning model within ransac_inlier_px (after refine).
  std::vector<bool> inlier_mask;
};

bool robust_triangulate_point_multiview(const std::vector<Eigen::Matrix3d>& R_list,
                                        const std::vector<Eigen::Vector3d>& C_list,
                                        const std::vector<Eigen::Vector2d>& rays_n,
                                        const std::vector<camera::Intrinsics>& cameras_per_view,
                                        const std::vector<double>& u_px, const std::vector<double>& v_px,
                                        const RobustTriangulationOptions& opt,
                                        RobustTriangulationResult* out);

/// Scope for run_retriangulation: one entry point, three behaviors (see .cpp).
enum class RetriangulationScope {
  kPendingOnly = 0, ///< Drain retri_pending_ids_ only (periodic / pre-final pending drain).
  kNewImages = 1,   ///< Tracks that touch restrict_image_indices (post–local BA).
  kFullScan = 2,    ///< All valid tracks: no-XYZ triangulation + restore deleted obs on triangulated.
};

/// Options for run_retriangulation.
///
/// Two branches per track (when applicable):
///   - XYZ intact  → restore previously-deleted observations within restore_strict_reproj_px.
///   - XYZ cleared → triangulate_track_common (RANSAC path inside).
struct IncrementalRetriangulationOptions {
  RetriangulationScope scope = RetriangulationScope::kPendingOnly;
  /// When scope == kNewImages: only tracks with at least one observation on these image indices.
  std::vector<int> restrict_image_indices;
  /// When scope == kFullScan for no-XYZ: RANSAC band (px). <= 0 falls back to robust.ransac_inlier_px.
  double full_scan_commit_reproj_px = 16.0;
  double restore_strict_reproj_px = 2.0;
  RobustTriangulationOptions robust;
};

int run_retriangulation(TrackStore* store, const std::vector<Eigen::Matrix3d>& poses_R,
                        const std::vector<Eigen::Vector3d>& poses_C,
                        const std::vector<bool>& registered,
                        const std::vector<camera::Intrinsics>& cameras,
                        const std::vector<int>& image_to_camera_index,
                        const IncrementalRetriangulationOptions& opts);

/// Legacy: only adjusts robust.min_tri_angle_deg (defaults for the rest).
// int run_retriangulation(TrackStore* store, const std::vector<Eigen::Matrix3d>& poses_R,
//                         const std::vector<Eigen::Vector3d>& poses_C,
//                         const std::vector<bool>& registered,
//                         const std::vector<camera::Intrinsics>& cameras,
//                         const std::vector<int>& image_to_camera_index,
//                         double min_tri_angle_deg = 0.50);

/**
 * Full-scan triangulation: iterate over ALL valid tracks in the store and attempt to triangulate
 * any track that currently has no XYZ (never triangulated, or XYZ cleared by outlier rejection).
 *
 * Unlike run_batch_triangulation (which only visits tracks visible from newly-registered images),
 * this function is designed to be called after a global BA pass so that:
 *   (a) Tracks cleared by BA outlier rejection get fresh XYZ from improved poses.
 *   (b) Tracks whose endpoint images were both registered long ago but were never successfully
 *       triangulated get another chance with the refined scene geometry.
 *
 * @param commit_reproj_px  RANSAC inlier band / commit threshold (pixels). Default 16 px
 *                          (more generous than incremental pass to maximize recovery).
 * @return Number of tracks newly triangulated.
 */
int run_full_scan_triangulation(TrackStore* store,
                                const std::vector<Eigen::Matrix3d>& poses_R,
                                const std::vector<Eigen::Vector3d>& poses_C,
                                const std::vector<bool>& registered,
                                const std::vector<camera::Intrinsics>& cameras,
                                const std::vector<int>& image_to_camera_index,
                                double min_tri_angle_deg = 0.5,
                                double commit_reproj_px = 16.0);

/**
 * Re-evaluate kRestorable deleted observations after a significant intrinsics change.
 *
 * For every triangulated track (track_has_triangulated_xyz == true), walks all
 * observations — including logically-deleted ones — and restores those that
 *   (a) carry obs_flags::kRestorable (deleted by reproj outlier rejection, not geometry), AND
 *   (b) belong to a registered image whose camera model index is in changed_cam_model_indices, AND
 *   (c) now have reprojection error ≤ strict_reproj_px under the current intrinsics/poses.
 *
 * Degenerate tracks whose XYZ has been cleared (clear_track_xyz) are skipped here; they
 * remain the responsibility of the pending-queue run_retriangulation mechanism.
 *
 * Uses the pre-built per-image obs index (image_obs_ids_) inside TrackStore so that only
 * observations belonging to the changed cameras are visited, not the full observation list.
 *
 * @param changed_cam_model_indices  Set of camera-model indices (into `cameras`) that changed.
 *                                   Pass an empty set to process observations from ALL cameras.
 * @param strict_reproj_px           Reprojection ceiling for restoration (px); should match
 *                                   the typical BA outlier-rejection threshold (~4 px).
 * @return Number of observations restored.
 */
int restore_observations_from_cameras(
    TrackStore* store,
    const std::vector<Eigen::Matrix3d>& poses_R,
    const std::vector<Eigen::Vector3d>& poses_C,
    const std::vector<bool>& registered,
    const std::vector<camera::Intrinsics>& cameras,
    const std::vector<int>& image_to_camera_index,
    const std::unordered_set<int>& changed_cam_model_indices,
    double strict_reproj_px);

} // namespace sfm
} // namespace insight
