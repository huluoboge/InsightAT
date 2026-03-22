/**
 * @file  resection_batch.h
 * @brief Incremental SfM: resection candidate selection and registration (incl. multi-image API).
 *
 * Sits above single-image PnP in resection.h / resection.cpp (run_batch_resection).
 */

#pragma once

#include "../camera/camera_types.h"
#include "track_store.h"
#include <Eigen/Core>
#include <vector>

namespace insight {
namespace sfm {

/// Per-image resection candidate returned by choose_resection_candidates() (best first).
struct ResectionCandidate {
  int   image_index = -1;
  int   num_3d2d    = 0;    ///< Triangulated 3D-2D correspondences.
  float coverage    = 0.0f; ///< Spatial spread [0,1] (3×3 grid occupancy).
};

/**
 * All unregistered images with ≥ min_3d2d_count triangulated matches, then sort:
 *   1. Prefer coverage ≥ min_coverage_good (default 0.33, spread enough for stable PnP).
 *   2. Then num_3d2d descending, then coverage descending, then image_index ascending.
 * Truncate to max_candidates. Pipeline tries candidates one image per iteration.
 */
std::vector<ResectionCandidate> choose_resection_candidates(const TrackStore& store,
                                                            const std::vector<bool>& registered,
                                                            int min_3d2d_count, int max_candidates,
                                                            float min_coverage_good = 0.33f);

/**
 * Run resection for each image in the batch; intrinsics = cameras[image_to_camera_index[im]].
 * Updates poses_R, poses_C, registered for each successful resection.
 * @return Number of images newly registered.
 */
int run_batch_resection(TrackStore& store, const std::vector<int>& image_indices,
                        const std::vector<camera::Intrinsics>& cameras,
                        const std::vector<int>& image_to_camera_index,
                        std::vector<Eigen::Matrix3d>* poses_R,
                        std::vector<Eigen::Vector3d>* poses_C, std::vector<bool>* registered,
                        int min_inliers = 15,
                        std::vector<int>* registered_images_out = nullptr,
                        double post_resection_reproj_thresh_px = 0.0);

} // namespace sfm
} // namespace insight
