/**
 * @file  track_builder.cpp
 * @brief Build TrackStore from inlier mask + match data.
 */

#include "track_builder.h"
#include <cassert>

namespace insight {
namespace sfm {

int build_tracks_from_pair_inliers(
    TrackStore& store,
    int image1_index,
    int image2_index,
    const std::vector<uint8_t>& inlier_mask,
    const std::vector<uint16_t>& indices,
    const std::vector<float>& coords_pixel,
    const std::vector<float>& scales,
    const std::vector<float>* points3d) {
    const int num_matches = static_cast<int>(inlier_mask.size());
    if (num_matches == 0) return 0;
    assert(static_cast<size_t>(indices.size()) >= static_cast<size_t>(num_matches) * 2u);
    assert(static_cast<size_t>(coords_pixel.size()) >= static_cast<size_t>(num_matches) * 4u);
    const bool have_scales = (scales.size() >= static_cast<size_t>(num_matches) * 2u);
    int n_inliers = 0;
    for (int m = 0; m < num_matches; ++m)
        if (inlier_mask[static_cast<size_t>(m)]) ++n_inliers;
    if (points3d)
        assert(static_cast<int>(points3d->size()) >= n_inliers * 3);

    int inlier_idx = 0;
    int added = 0;
    for (int m = 0; m < num_matches; ++m) {
        if (!inlier_mask[static_cast<size_t>(m)]) continue;
        const size_t mi = static_cast<size_t>(m);
        float x = 0.f, y = 0.f, z = 0.f;
        if (points3d && static_cast<size_t>(inlier_idx * 3 + 2) < points3d->size()) {
            x = (*points3d)[static_cast<size_t>(inlier_idx * 3)];
            y = (*points3d)[static_cast<size_t>(inlier_idx * 3 + 1)];
            z = (*points3d)[static_cast<size_t>(inlier_idx * 3 + 2)];
        }
        const int track_id = store.add_track(x, y, z);
        const uint16_t idx1 = indices[mi * 2];
        const uint16_t idx2 = indices[mi * 2 + 1];
        const float x1 = coords_pixel[mi * 4];
        const float y1 = coords_pixel[mi * 4 + 1];
        const float x2 = coords_pixel[mi * 4 + 2];
        const float y2 = coords_pixel[mi * 4 + 3];
        float s1 = 1.f, s2 = 1.f;
        if (have_scales) {
            s1 = scales[mi * 2];
            s2 = scales[mi * 2 + 1];
        }
        store.add_observation(track_id, static_cast<uint32_t>(image1_index), static_cast<uint32_t>(idx1), x1, y1, s1);
        store.add_observation(track_id, static_cast<uint32_t>(image2_index), static_cast<uint32_t>(idx2), x2, y2, s2);
        ++inlier_idx;
        ++added;
    }
    return added;
}

}  // namespace sfm
}  // namespace insight
