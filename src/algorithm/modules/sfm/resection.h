/**
 * @file  resection.h
 * @brief Resection (PnP) for incremental SfM: RANSAC PnP + pose-only BA.
 *
 * Uses OpenCV solvePnPRansac when available; pose refinement via pose_only_bundle (Ceres).
 * World frame = cam0 frame; poses are world-to-camera.
 */

#pragma once

#include "track_store.h"
#include <Eigen/Core>
#include <vector>

namespace insight {
namespace sfm {

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
bool resection_single_image(const TrackStore& store,
                            int image_index,
                            double fx, double fy, double cx, double cy,
                            Eigen::Matrix3d* R_out,
                            Eigen::Vector3d* t_out,
                            int min_inliers = 6,
                            double ransac_thresh_px = 8.0,
                            int* inliers_out = nullptr);

/**
 * Select next image to register: unregistered image with the most
 * 3D–2D correspondences (tracks with triangulated xyz observed in that image).
 *
 * @param store              Track store.
 * @param registered_indices Set of image indices already registered.
 * @param skip_indices       Optional: if non-null, images with skip_indices[i]==true are excluded.
 * @return Image index with max correspondences, or -1 if none.
 */
int choose_next_resection_image(const TrackStore& store,
                                const std::vector<bool>& registered_indices,
                                const std::vector<bool>* skip_indices = nullptr);

}  // namespace sfm
}  // namespace insight
