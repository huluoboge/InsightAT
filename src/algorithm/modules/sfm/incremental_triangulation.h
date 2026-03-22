/**
 * @file  incremental_triangulation.h
 * @brief Incremental SfM: multiview triangulation from TrackStore + registered poses.
 */

#pragma once

#include "../camera/camera_types.h"
#include "track_store.h"
#include <Eigen/Core>
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

/// Full re-triangulation pass: restore deleted observations on triangulated tracks (strict reproj),
/// then robust triangulation for tracks still without 3D (requires enough views).
struct IncrementalRetriangulationOptions {
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
int run_retriangulation(TrackStore* store, const std::vector<Eigen::Matrix3d>& poses_R,
                        const std::vector<Eigen::Vector3d>& poses_C,
                        const std::vector<bool>& registered,
                        const std::vector<camera::Intrinsics>& cameras,
                        const std::vector<int>& image_to_camera_index,
                        double min_tri_angle_deg = 0.50);

} // namespace sfm
} // namespace insight
