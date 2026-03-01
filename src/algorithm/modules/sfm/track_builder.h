/**
 * @file  track_builder.h
 * @brief Build TrackStore from geo inliers + match data (one pair or Phase1/Phase2).
 *
 * Callers load .isat_geo and .isat_match via IDCReader, then call these functions.
 * No IDC dependency inside sfm_module.
 */

#pragma once

#ifndef TRACK_BUILDER_H
#define TRACK_BUILDER_H

#include "track_store.h"
#include <cstdint>
#include <vector>

namespace insight {
namespace sfm {

/**
 * Build tracks for one image pair from inlier mask + match data + optional 3D points.
 *
 * For each match index m with inlier_mask[m]==1: creates one track with two observations
 * (image1_index, feat1, x1, y1, s1) and (image2_index, feat2, x2, y2, s2). If points3d
 * is non-null, track XYZ is set from points3d[3*i] (i = inlier index); otherwise XYZ = (0,0,0).
 *
 * @param store          Target TrackStore (must have set_num_images and capacity)
 * @param image1_index   Image index for first view
 * @param image2_index   Image index for second view
 * @param inlier_mask    Per-match inlier (1) or outlier (0), size = num_matches
 * @param indices        Match feature indices, size num_matches*2 [idx1, idx2, ...]
 * @param coords_pixel   Match pixel coords, size num_matches*4 [x1,y1,x2,y2, ...]
 * @param scales         Optional scales per match, size num_matches*2 [s1,s2, ...]; if empty, 1.f is used
 * @param points3d       Optional 3D points for inliers, size 3*n_inliers (x,y,z per point); if null, track xyz = (0,0,0)
 * @return               Number of tracks added
 */
int build_tracks_from_pair_inliers(
    TrackStore& store,
    int image1_index,
    int image2_index,
    const std::vector<uint8_t>& inlier_mask,
    const std::vector<uint16_t>& indices,
    const std::vector<float>& coords_pixel,
    const std::vector<float>& scales,
    const std::vector<float>* points3d);

}  // namespace sfm
}  // namespace insight

#endif /* TRACK_BUILDER_H */
