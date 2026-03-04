/**
 * @file  resection.h
 * @brief Resection (PnP) for incremental SfM: RANSAC PnP + pose-only BA.
 *
 * Uses GPU RANSAC PnP (gpu_ransac_pnp) when gpu_geo_init() has been called;
 * pose refinement via pose_only_bundle (Ceres).
 * World frame = cam0 frame; poses are world-to-camera.
 *
 * When intrinsics have distortion (camera::Intrinsics), 2D observations are
 * pre-undistorted (via undistort_points) before the pinhole PnP kernel and BA.
 */

#pragma once

#include "../camera/camera_types.h"
#include "track_store.h"
#include <Eigen/Core>
#include <vector>

namespace insight {
namespace sfm {

/**
 * Check if a resection result is stable: inlier ratio and reprojection RMSE.
 *
 * @param inlier_count    Number of inliers.
 * @param total_correspondences  Total 3D–2D pairs used.
 * @param rmse_px        Reprojection RMSE in pixels (e.g. from pose_only_bundle).
 * @param min_inlier_ratio  Minimum inlier ratio (default 0.4).
 * @param max_rmse_px    Maximum acceptable RMSE in pixels (default 10.0).
 * @return true if resection is considered stable.
 */
bool is_resection_stable(int inlier_count, int total_correspondences, double rmse_px,
                         double min_inlier_ratio = 0.4, double max_rmse_px = 10.0);

/**
 * Run resection for a single unregistered image: collect 3D–2D from tracks
 * that have triangulated XYZ, run PnP RANSAC, then pose-only BA.
 *
 * @param store           Full track store (world 3D in track xyz).
 * @param image_index     Unregistered image index.
 * @param fx, fy, cx, cy  Intrinsics.
 * @param min_inliers     Minimum RANSAC inliers to accept (default 6).
 * @param ransac_thresh_px RANSAC reprojection threshold in pixels.
 * @param R_out, t_out    Output pose (world to camera).
 * @param inliers_out     Optional: number of inliers after RANSAC.
 * @return true if resection succeeded (enough inliers and BA converged).
 */
bool resection_single_image(const TrackStore& store, int image_index, double fx, double fy,
                            double cx, double cy, Eigen::Matrix3d* R_out, Eigen::Vector3d* t_out,
                            int min_inliers = 6, double ransac_thresh_px = 8.0,
                            int* inliers_out = nullptr);

/**
 * Overload: run resection with algorithm intrinsics (camera::Intrinsics).
 *
 * If K.has_distortion(), observations are batch-undistorted before PnP.
 *
 * @param K               Algorithm intrinsics (from JSON or filled at call boundary).
 * @param store           Full track store.
 * @param image_index     Unregistered image index.
 * @param min_inliers     Minimum RANSAC inliers (default 6).
 * @param ransac_thresh_px RANSAC reprojection threshold in pixels.
 * @param R_out, t_out    Output pose (world to camera).
 * @param inliers_out     Optional: number of RANSAC inliers.
 * @return true if resection succeeded.
 */
bool resection_single_image(const camera::Intrinsics& K, const TrackStore& store, int image_index,
                            Eigen::Matrix3d* R_out, Eigen::Vector3d* t_out,
                            int min_inliers = 6, double ransac_thresh_px = 8.0,
                            int* inliers_out = nullptr);

/**
 * Count grid cells that contain at least one 3D–2D observation (COLMAP-style
 * distribution check). Only observations from tracks with triangulated xyz are used.
 * Image area is the bounding box of those observations, divided into grid_cols x grid_rows.
 *
 * @return Number of cells with >= 1 observation (0 if none or no triangulated obs).
 */
int resection_image_grid_coverage(const TrackStore& store, int image_index, int grid_cols = 4,
                                  int grid_rows = 4);

/**
 * Select next image to register: unregistered image with the most
 * 3D–2D correspondences (tracks with triangulated xyz observed in that image).
 * Optionally requires minimum grid coverage (feature distribution) to avoid degenerate resection.
 *
 * @param store              Track store.
 * @param registered_indices  Set of image indices already registered.
 * @param skip_indices       Optional: if non-null, images with skip_indices[i]==true are excluded.
 * @param min_grid_cells     Minimum number of grid cells with observations (0 = disable).
 * Default 3.
 * @return Image index with max correspondences (and meeting min_grid_cells), or -1 if none.
 */
int choose_next_resection_image(const TrackStore& store,
                                const std::vector<bool>& registered_indices,
                                const std::vector<bool>* skip_indices = nullptr,
                                int min_grid_cells = 3);

} // namespace sfm
} // namespace insight
