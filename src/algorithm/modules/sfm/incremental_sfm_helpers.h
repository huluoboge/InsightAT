/**
 * @file  incremental_sfm_helpers.h
 * @brief Helpers for incremental SfM: outlier rejection, track filtering.
 *
 * Two-view convention: cam0 = identity, cam1 = (R, t). Points in cam0 frame.
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
 * Cam0 = identity, cam1 = (R, t); points in cam0 frame.
 * Observations are compared against pinhole-projected 3D points.
 */
int reject_outliers_two_view(TrackStore* store, const Eigen::Matrix3d& R, const Eigen::Vector3d& t,
                             double fx, double fy, double cx, double cy, double threshold_px);

/**
 * Overload: same as above but accepts algorithm intrinsics (camera::Intrinsics).
 *
 * When K.has_distortion() the raw pixel observations are undistorted
 * before comparing with the pinhole-projected 3D points.
 */
int reject_outliers_two_view(TrackStore* store, const Eigen::Matrix3d& R, const Eigen::Vector3d& t,
                             const camera::Intrinsics& K, double threshold_px);

/**
 * Mark tracks deleted if valid observation count < min_observations, or
 * (for two-view) if the angle between the two rays is below min_angle_deg.
 * Uses cam0 = identity, cam1 = (R, t); ray angle computed in cam0 frame.
 */
int filter_tracks_two_view(TrackStore* store, const Eigen::Matrix3d& R, const Eigen::Vector3d& t,
                           int min_observations, double min_angle_deg);

/**
 * Re-triangulate all tracks that have exactly 2 observations (image 0 and 1)
 * using DLT with (R, t). Updates track xyz; invalid points are set to (0,0,0).
 */
int retriangulate_two_view_tracks(TrackStore* store, const Eigen::Matrix3d& R,
                                  const Eigen::Vector3d& t, double fx, double fy, double cx,
                                  double cy);

/**
 * Triangulate tracks that have an observation in new_image_index and at least
 * one in another registered image, but no triangulated xyz yet. Uses multi-view
 * DLT (world frame = cam0). poses_R[i], poses_t[i] = world-to-camera for image i.
 *
 * @param registered_indices  true for each image that has a valid pose.
 * @return Number of tracks newly triangulated.
 */
int triangulate_tracks_for_new_image(TrackStore* store, int new_image_index,
                                     const std::vector<Eigen::Matrix3d>& poses_R,
                                     const std::vector<Eigen::Vector3d>& poses_t,
                                     const std::vector<bool>& registered_indices, double fx,
                                     double fy, double cx, double cy);

/**
 * Overload: per-image intrinsics for normalised ray computation.
 * intrinsics_per_image.size() must match number of images.
 */
int triangulate_tracks_for_new_image(TrackStore* store, int new_image_index,
                                     const std::vector<Eigen::Matrix3d>& poses_R,
                                     const std::vector<Eigen::Vector3d>& poses_t,
                                     const std::vector<bool>& registered_indices,
                                     const std::vector<camera::Intrinsics>& intrinsics_per_image);

// ─────────────────────────────────────────────────────────────────────────────
// Multiview (N images) versions
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Mark observations as deleted when reprojection error exceeds threshold.
 * Multiview: uses poses_R[i], poses_t[i] for each registered image i.
 * World frame = cam0; poses are world-to-camera.
 */
int reject_outliers_multiview(TrackStore* store,
                             const std::vector<Eigen::Matrix3d>& poses_R,
                             const std::vector<Eigen::Vector3d>& poses_t,
                             const std::vector<bool>& registered, double fx, double fy,
                             double cx, double cy, double threshold_px);

/**
 * Overload: use camera::Intrinsics for distortion-aware reprojection.
 */
int reject_outliers_multiview(TrackStore* store,
                             const std::vector<Eigen::Matrix3d>& poses_R,
                             const std::vector<Eigen::Vector3d>& poses_t,
                             const std::vector<bool>& registered,
                             const camera::Intrinsics& K, double threshold_px);

/**
 * Mark tracks deleted if valid observation count < min_observations, or
 * (multiview) if triangulation angle is below min_angle_deg.
 */
int filter_tracks_multiview(TrackStore* store,
                            const std::vector<Eigen::Matrix3d>& poses_R,
                            const std::vector<Eigen::Vector3d>& poses_t,
                            const std::vector<bool>& registered, int min_observations,
                            double min_angle_deg);

/**
 * Multiview outlier rejection with per-image intrinsics.
 * intrinsics_per_image[image_index] = K for that image; size must match poses_R.
 * When null or size mismatch, behaviour is undefined (caller must ensure consistency).
 */
int reject_outliers_multiview(TrackStore* store,
                              const std::vector<Eigen::Matrix3d>& poses_R,
                              const std::vector<Eigen::Vector3d>& poses_t,
                              const std::vector<bool>& registered,
                              const std::vector<camera::Intrinsics>& intrinsics_per_image,
                              double threshold_px);

} // namespace sfm
} // namespace insight
