/**
 * @file  incremental_sfm.h
 * @brief Initial pair selection, two-view bootstrap, and resection loop for incremental SfM.
 *
 * Requires IDCReader (link with algorithm target that has io).
 */

#pragma once

#include "../camera/camera_types.h"
#include "id_mapping.h"
#include "track_store.h"
#include <Eigen/Core>
#include <string>
#include <vector>

namespace insight {
namespace sfm {

struct MultiCameraSetup;

struct InitialPairResult {
  bool success = false;
  uint32_t image1_index = 0; ///< First image index (0..n-1)
  uint32_t image2_index = 0; ///< Second image index (0..n-1)
  Eigen::Matrix3d R; ///< Cam2 rotation (world-to-camera)
  Eigen::Vector3d C; ///< Cam2 centre in world (cam1) coordinates
};

/**
 * Build ViewGraph from geo, choose best initial pair (both unregistered),
 * load that pair's R/C/points and match, build TrackStore with 2 images,
 * re-triangulate, run two-view BA, reject outliers, filter tracks.
 * Fills store_out (2 images), R1_out and C1_out (cam2 pose). Cam1 = identity at origin.
 *
 * @param min_tracks_after  Minimum valid tracks after filtering to consider success.
 * @param image1_index_out  Optional: chosen first image index (0..n-1).
 * @param image2_index_out  Optional: chosen second image index (0..n-1).
 * @param id_mapping        Boundary only: for pair JSON (orig_id→index) and geo/match paths (index→orig_id).
 * @param intrinsics_initial_pair  Optional: when non-null and size()>=2, use [0] for image 0 and [1]
 *                                  for image 1 (full intrinsics + distortion). Otherwise use fx,fy,cx,cy.
 * @return true if a pair was chosen and the store has at least min_tracks_after tracks.
 */
bool run_initial_pair_loop(const std::string& pairs_json_path, const std::string& geo_dir,
                           const std::string& match_dir, double fx, double fy, double cx, double cy,
                           TrackStore* store_out, Eigen::Matrix3d* R1_out, Eigen::Vector3d* C1_out,
                           int min_tracks_after = 20, uint32_t* image1_index_out = nullptr,
                           uint32_t* image2_index_out = nullptr,
                           const IdMapping* id_mapping = nullptr,
                           const std::vector<camera::Intrinsics>* intrinsics_initial_pair = nullptr);

/**
 * Run initial pair on an already-built full TrackStore (n_images). Chooses best pair from
 * ViewGraph, loads geo for that pair, triangulates tracks visible in both images, runs
 * two-view BA, rejects outliers, filters tracks. Uses cameras->for_image_index(idx) for
 * correct intrinsics. Store is modified in-place; poses for the chosen pair are output
 * (cam1 = identity, cam2 = R1_out, C1_out).
 *
 * @param cameras  Multi-camera setup; for_image_index(idx1), for_image_index(idx2) must be valid.
 * @param store_in_out  Full track store (from build_full_track_store_from_pairs); modified in-place.
 * @return true if at least min_tracks_after valid tracks remain in the two-view subset.
 */
bool run_initial_pair_loop(const std::string& pairs_json_path, const std::string& geo_dir,
                           const std::string& match_dir, const MultiCameraSetup* cameras,
                           const IdMapping* id_mapping, TrackStore* store_in_out,
                           Eigen::Matrix3d* R1_out, Eigen::Vector3d* C1_out,
                           int min_tracks_after = 20, uint32_t* image1_index_out = nullptr,
                           uint32_t* image2_index_out = nullptr);

/**
 * Resection loop on an existing TrackStore with n_images. Expects the first two
 * images (indices 0 and 1) to be already registered with poses in poses_R/poses_C
 * (world = cam0; cam0 = identity at origin, cam1 = R1, C1). Triangulated tracks must have
 * track_has_triangulated_xyz set.
 *
 * Repeatedly: choose unregistered image with most 3D–2D correspondences, run
 * resection_single_image, then triangulate_tracks_for_new_image for that image.
 *
 * @param store         Track store (n_images, some tracks with triangulated xyz).
 * @param poses_R       Per-image rotation (world-to-camera); only used for registered.
 * @param poses_C       Per-image camera centre (world coordinates).
 * @param registered    On input: true for images 0 and 1; on output: true for all registered.
 * @param fx,fy,cx,cy   Intrinsics.
 * @param min_correspondences  Skip image if 3D–2D count below this (default 6).
 * @return Number of newly registered images.
 */
int run_resection_loop(TrackStore* store, std::vector<Eigen::Matrix3d>* poses_R,
                       std::vector<Eigen::Vector3d>* poses_C, std::vector<bool>* registered,
                       double fx, double fy, double cx, double cy, int min_correspondences = 6);

/**
 * Overload: run_initial_pair_loop with algorithm intrinsics (camera::Intrinsics).
 *
 * If K.has_distortion(), outlier rejection uses the intrinsics-aware overload
 * to pre-undistort observations.
 */
bool run_initial_pair_loop(const std::string& pairs_json_path, const std::string& geo_dir,
                           const std::string& match_dir, const camera::Intrinsics& K,
                           TrackStore* store_out, Eigen::Matrix3d* R1_out, Eigen::Vector3d* C1_out,
                           int min_tracks_after = 20, uint32_t* image1_index_out = nullptr,
                           uint32_t* image2_index_out = nullptr,
                           const IdMapping* id_mapping = nullptr);

/**
 * Overload: run_resection_loop with algorithm intrinsics (camera::Intrinsics).
 *
 * Uses resection_single_image(K, ...) so distorted observations are
 * pre-undistorted before the PnP solver.
 */
int run_resection_loop(TrackStore* store, std::vector<Eigen::Matrix3d>* poses_R,
                       std::vector<Eigen::Vector3d>* poses_C, std::vector<bool>* registered,
                       const camera::Intrinsics& K, int min_correspondences = 6);

/**
 * Overload: per-image intrinsics (multiple cameras). intrinsics_per_image[i] = K for image i.
 * Size must match store->num_images(). Fallback fx,fy,cx,cy used when vector null or lookup missing.
 */
int run_resection_loop(TrackStore* store, std::vector<Eigen::Matrix3d>* poses_R,
                       std::vector<Eigen::Vector3d>* poses_C, std::vector<bool>* registered,
                       const std::vector<camera::Intrinsics>* intrinsics_per_image,
                       double fx, double fy, double cx, double cy,
                       int min_correspondences = 6);

} // namespace sfm
} // namespace insight
