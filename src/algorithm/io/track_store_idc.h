/**
 * @file  track_store_idc.h
 * @brief Load TrackStore from .isat_tracks IDC (shared by isat_tracks and incremental SfM).
 */

#pragma once

#include "../modules/sfm/track_store.h"
#include <string>
#include <vector>

namespace insight {
namespace sfm {

/**
 * Load TrackStore from a .isat_tracks IDC file.
 * Fills store with tracks and observations; image indices in store are 0..num_images-1.
 *
 * @param path               Path to .isat_tracks IDC.
 * @param store_out          Output track store (must not be null).
 * @param image_indices_out  Optional: if non-null, filled with image_indices from IDC metadata.
 * @return true on success, false on read/parse error or blob size mismatch.
 */
bool load_track_store_from_idc(const std::string& path, TrackStore* store_out,
                               std::vector<uint32_t>* image_indices_out = nullptr);

} // namespace sfm
} // namespace insight
