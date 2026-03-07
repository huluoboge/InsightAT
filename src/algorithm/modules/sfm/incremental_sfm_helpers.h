/**
 * @file  incremental_sfm_helpers.h
 * @brief Helpers for incremental SfM: outlier rejection, track filtering.
 *
 * Pose convention: poses_R[i] is world-to-camera rotation;
 * poses_C[i] is the camera centre in world coordinates (C = -Rᵀ·t).
 *
 * Two-view convention: cam0 = identity at origin (C0 = 0), cam1 = (R, C).
 */

#pragma once

#include "../camera/camera_types.h"
#include "track_store.h"
#include <Eigen/Core>

namespace insight {
namespace sfm {

/**
 * Mark observations as deleted when reprojection error exceeds threshold.
 * Only considers observations with image_index 0 or 1 (two-view).
 * Cam0 = identity at origin; cam1 = (R, C). Points in world (cam0) frame.
 */
int reject_outliers_two_view(TrackStore* store, const Eigen::Matrix3d& R,
                             const Eigen::Vector3d& C,
                             double fx, double fy, double cx, double cy, double threshold_px);

/**
 * Overload: same as above but accepts algorithm intrinsics (camera::Intrinsics).
 *
 * When K.has_distortion() the raw pixel observations are undistorted
 * before comparing with the pinhole-projected 3D points.
 */
int reject_outliers_two_view(TrackStore* store, const Eigen::Matrix3d& R,
                             const Eigen::Vector3d& C,
                             const camera::Intrinsics& K, double threshold_px);

/**
 * Per-camera intrinsics for two-view: K0 for image 0, K1 for image 1.
 * Uses full model (fx,fy,cx,cy,k1..p2) and undistortion when needed.
 */
int reject_outliers_two_view(TrackStore* store, const Eigen::Matrix3d& R,
                             const Eigen::Vector3d& C,
                             const camera::Intrinsics& K0, const camera::Intrinsics& K1,
                             double threshold_px);

/**
 * Mark tracks deleted if valid observation count < min_observations, or
 * (for two-view) if the angle between the two rays is below min_angle_deg.
 * Cam0 = identity at origin; cam1 = (R, C).
 */
int filter_tracks_two_view(TrackStore* store, const Eigen::Matrix3d& R,
                           const Eigen::Vector3d& C,
                           int min_observations, double min_angle_deg);

/**
 * Re-triangulate all tracks that have exactly 2 observations (image 0 and 1)
 * using DLT with (R, C). Updates track xyz; invalid points are set to (0,0,0).
 */
int retriangulate_two_view_tracks(TrackStore* store, const Eigen::Matrix3d& R,
                                  const Eigen::Vector3d& C,
                                  double fx, double fy, double cx, double cy);

/**
 * Overload: per-camera intrinsics K0 (image 0), K1 (image 1). Uses undistortion when needed.
 */
int retriangulate_two_view_tracks(TrackStore* store, const Eigen::Matrix3d& R,
                                  const Eigen::Vector3d& C,
                                  const camera::Intrinsics& K0, const camera::Intrinsics& K1);

/**
 * Same as above but for a full store: only observations with image_index == image0_index or
 * image1_index are considered. image0 = identity (world), image1 = (R, C). K0/K1 for those images.
 */
int retriangulate_two_view_tracks(TrackStore* store, int image0_index, int image1_index,
                                  const Eigen::Matrix3d& R, const Eigen::Vector3d& C,
                                  const camera::Intrinsics& K0, const camera::Intrinsics& K1);

/**
 * Same as reject_outliers_two_view(store, R, C, K0, K1, ...) but only observations with
 * image_index == image0_index or image1_index. image0 = identity, image1 = (R, C).
 */
int reject_outliers_two_view(TrackStore* store, const Eigen::Matrix3d& R,
                             const Eigen::Vector3d& C, int image0_index, int image1_index,
                             const camera::Intrinsics& K0, const camera::Intrinsics& K1,
                             double threshold_px);

/**
 * Same as filter_tracks_two_view but only considers tracks that have observations in both
 * image0_index and image1_index. Angle check uses (R, C) for image1.
 */
int filter_tracks_two_view(TrackStore* store, const Eigen::Matrix3d& R,
                           const Eigen::Vector3d& C, int image0_index, int image1_index,
                           int min_observations, double min_angle_deg);

/**
 * Triangulate tracks that have an observation in new_image_index and at least
 * one in another registered image, but no triangulated xyz yet. Uses multi-view
 * DLT (world frame = cam0). poses_R[i], poses_C[i] = world-to-camera for image i.
 *
 * @param registered_indices  true for each image that has a valid pose.
 * @return Number of tracks newly triangulated.
 */
int triangulate_tracks_for_new_image(TrackStore* store, int new_image_index,
                                     const std::vector<Eigen::Matrix3d>& poses_R,
                                     const std::vector<Eigen::Vector3d>& poses_C,
                                     const std::vector<bool>& registered_indices, double fx,
                                     double fy, double cx, double cy);

/**
 * Overload: per-image intrinsics for normalised ray computation.
 * intrinsics_per_image.size() must match number of images.
 */
int triangulate_tracks_for_new_image(TrackStore* store, int new_image_index,
                                     const std::vector<Eigen::Matrix3d>& poses_R,
                                     const std::vector<Eigen::Vector3d>& poses_C,
                                     const std::vector<bool>& registered_indices,
                                     const std::vector<camera::Intrinsics>& intrinsics_per_image);

// ─────────────────────────────────────────────────────────────────────────────
// Multiview (N images) versions
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Mark observations as deleted when reprojection error exceeds threshold.
 * Multiview: uses poses_R[i], poses_C[i] for each registered image i.
 * World frame = cam0; poses_R is world-to-camera; poses_C is camera centre.
 */
int reject_outliers_multiview(TrackStore* store,
                             const std::vector<Eigen::Matrix3d>& poses_R,
                             const std::vector<Eigen::Vector3d>& poses_C,
                             const std::vector<bool>& registered, double fx, double fy,
                             double cx, double cy, double threshold_px);

/**
 * Overload: use camera::Intrinsics for distortion-aware reprojection.
 */
int reject_outliers_multiview(TrackStore* store,
                             const std::vector<Eigen::Matrix3d>& poses_R,
                             const std::vector<Eigen::Vector3d>& poses_C,
                             const std::vector<bool>& registered,
                             const camera::Intrinsics& K, double threshold_px);

/**
 * Mark tracks deleted if valid observation count < min_observations, or
 * (multiview) if triangulation angle is below min_angle_deg.
 */
int filter_tracks_multiview(TrackStore* store,
                            const std::vector<Eigen::Matrix3d>& poses_R,
                            const std::vector<Eigen::Vector3d>& poses_C,
                            const std::vector<bool>& registered, int min_observations,
                            double min_angle_deg);

/**
 * Multiview outlier rejection with per-image intrinsics.
 * intrinsics_per_image[image_index] = K for that image; size must match poses_R.
 */
int reject_outliers_multiview(TrackStore* store,
                              const std::vector<Eigen::Matrix3d>& poses_R,
                              const std::vector<Eigen::Vector3d>& poses_C,
                              const std::vector<bool>& registered,
                              const std::vector<camera::Intrinsics>& intrinsics_per_image,
                              double threshold_px);

} // namespace sfm
} // namespace insight
