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

/**
 * Per-image score cache for choose_resection_candidates().
 *
 * Stores (n_tri_last, score_last) for every image.  The cache is valid as long as the
 * triangulated-3D-point count has not changed — no explicit invalidation is needed because
 * we re-count n_tri cheaply on every call and compare it against the watermark.
 *
 * Lifecycle: allocate once in the pipeline main loop, pass the same pointer on every
 * choose_resection_candidates() call.  Entries are updated in-place.
 */
struct ResectionScoreCache {
  struct Entry {
    int   n_tri = -1;  ///< −1 = never computed.
    float score = 0.0f;
  };
  std::vector<Entry> entries;

  void ensure_size(int n_images) {
    if (static_cast<int>(entries.size()) < n_images)
      entries.resize(static_cast<size_t>(n_images));
  }
  void invalidate_all() {
    for (auto& e : entries)
      e.n_tri = -1;
  }
};

/// Per-image resection candidate returned by choose_resection_candidates() (best first).
struct ResectionCandidate {
  int   image_index = -1;
  int   num_3d2d    = 0;    ///< Triangulated 3D-2D correspondences.
  /// Normalized COLMAP-style VisibilityPyramid score in [0,1] (absolute image coords).
  float coverage    = 0.0f;
};

/**
 * All unregistered images with ≥ min_3d2d_count triangulated matches, then sort:
 *   1. Prefer coverage ≥ min_coverage_good (normalized VisibilityPyramid score in [0,1]).
 *   2. Then num_3d2d descending, then coverage descending, then image_index ascending.
 * Truncate to max_candidates. Pipeline tries candidates one image per iteration.
 *
 * @param cameras                  Per-camera intrinsics (width/height used for absolute pyramid).
 * @param score_cache              Optional persistent cache.  Pass the same pointer every call to
 *                                 amortize VisibilityPyramid construction across iterations.
 *                                 Score is reused when n_tri for the image has not changed since
 *                                 the last call; pyramid is rebuilt only on first use or after
 *                                 new triangulation.  nullptr disables caching.
 */
std::vector<ResectionCandidate> choose_resection_candidates(
    const TrackStore& store, const std::vector<bool>& registered,
    const std::vector<camera::Intrinsics>& cameras, const std::vector<int>& image_to_camera_index,
    int min_3d2d_count, int max_candidates, float min_coverage_good = 0.02f,
    size_t visibility_pyramid_levels = 6, ResectionScoreCache* score_cache = nullptr);

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
