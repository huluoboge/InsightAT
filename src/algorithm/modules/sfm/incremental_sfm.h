/**
 * @file  incremental_sfm.h
 * @brief Initial pair selection, two-view bootstrap, and resection loop for incremental SfM.
 *
 * Requires IDCReader (link with algorithm target that has io).
 */

#pragma once

#include "track_store.h"
#include <Eigen/Core>
#include <string>
#include <vector>

namespace insight {
namespace sfm {

struct InitialPairResult {
  bool success = false;
  uint32_t image1_id = 0;
  uint32_t image2_id = 0;
  Eigen::Matrix3d R; ///< Cam2 rotation w.r.t. cam1
  Eigen::Vector3d t; ///< Cam2 translation w.r.t. cam1
};

/**
 * Build ViewGraph from geo, choose best initial pair (both unregistered),
 * load that pair's R/t/points and match, build TrackStore with 2 images,
 * re-triangulate, run two-view BA, reject outliers, filter tracks.
 * Fills store_out (2 images), R1_out and t1_out (cam2 pose). Cam1 = identity.
 *
 * @param min_tracks_after  Minimum valid tracks after filtering to consider success.
 * @param image1_id_out     Optional: set to the chosen first image id.
 * @param image2_id_out     Optional: set to the chosen second image id.
 * @return true if a pair was chosen and the store has at least min_tracks_after tracks.
 */
bool run_initial_pair_loop(const std::string& pairs_json_path, const std::string& geo_dir,
                           const std::string& match_dir, double fx, double fy, double cx, double cy,
                           TrackStore* store_out, Eigen::Matrix3d* R1_out, Eigen::Vector3d* t1_out,
                           int min_tracks_after = 20, uint32_t* image1_id_out = nullptr,
                           uint32_t* image2_id_out = nullptr);

/**
 * Resection loop on an existing TrackStore with n_images. Expects the first two
 * images (indices 0 and 1) to be already registered with poses in poses_R/poses_t
 * (world = cam0; cam0 = identity, cam1 = R1,t1). Triangulated tracks must have
 * track_has_triangulated_xyz set (e.g. from run_initial_pair_loop on a 2-image store
 * that was then merged into this store, or caller must set poses and triangulate).
 *
 * Repeatedly: choose unregistered image with most 3D–2D correspondences, run
 * resection_single_image, then triangulate_tracks_for_new_image for that image.
 *
 * @param store         Track store (n_images, some tracks with triangulated xyz).
 * @param poses_R       Per-image rotation (world to camera); only used for registered.
 * @param poses_t       Per-image translation (world to camera).
 * @param registered    On input: true for images 0 and 1; on output: true for all registered.
 * @param fx,fy,cx,cy   Intrinsics.
 * @param min_correspondences  Skip image if 3D–2D count below this (default 6).
 * @return Number of newly registered images.
 */
int run_resection_loop(TrackStore* store, std::vector<Eigen::Matrix3d>* poses_R,
                       std::vector<Eigen::Vector3d>* poses_t, std::vector<bool>* registered,
                       double fx, double fy, double cx, double cy, int min_correspondences = 6);

} // namespace sfm
} // namespace insight
