/**
 * @file  track_store_idc.h
 * @brief Load/save TrackStore from .isat_tracks IDC (shared by isat_tracks and incremental SfM).
 *
 * Schema 1.0: tracks + observations only.
 * Schema 1.1: adds optional "view_graph_pairs" JSON array (PairGeoInfo per covisible edge, aligned
 *              with filtered tracks when written by isat_tracks).
 */

#pragma once

#include "../modules/sfm/track_store.h"
#include <string>
#include <vector>

namespace insight {
namespace sfm {

class ViewGraph;

/**
 * Load TrackStore from a .isat_tracks IDC file.
 * Fills store with tracks and observations; image indices in store are 0..num_images-1.
 *
 * @param path               Path to .isat_tracks IDC.
 * @param store_out          Output track store (must not be null).
 * @param image_indices_out  Optional: if non-null, filled with image_indices from IDC metadata.
 * @param view_graph_out     Optional: if non-null, filled from embedded view_graph_pairs when present.
 * @return true on success, false on read/parse error or blob size mismatch.
 */
bool load_track_store_from_idc(const std::string& path, TrackStore* store_out,
                               std::vector<uint32_t>* image_indices_out = nullptr,
                               ViewGraph* view_graph_out = nullptr);

/**
 * Write TrackStore to .isat_tracks IDC. When \p view_graph is non-null and non-empty, embeds
 * view_graph_pairs and sets schema_version to "1.1".
 */
bool save_track_store_to_idc(const TrackStore& store, const std::vector<uint32_t>& image_indices,
                             const std::string& path, const ViewGraph* view_graph = nullptr);

} // namespace sfm
} // namespace insight
