/**
 * @file  scene_normalization.h
 * @brief Median-based scene normalization on triangulated tracks + registered camera centres.
 *
 * μ = (median x, median y, median z) over valid triangulated points.
 * s = median_i ‖X_i − μ‖.  Transform: X' = (X − μ) / s,  C' = (C − μ) / s for registered cameras.
 * poses_R unchanged (uniform scale + translation preserves projection ratios).
 *
 * Internal median helpers use nth_element; replace with histogram binning for speed (TODO).
 * No inverse transform: pipeline stays in normalized world units.
 */

#pragma once

#include <Eigen/Core>
#include <vector>

namespace insight {
namespace sfm {

class TrackStore;

/// Returns false if too few points or degenerate scale.
bool normalize_scene_median_tracks_and_poses(TrackStore* store, std::vector<Eigen::Vector3d>* poses_C,
                                             const std::vector<bool>& registered);

} // namespace sfm
} // namespace insight
