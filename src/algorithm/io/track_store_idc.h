/**
 * @file  track_store_idc.h
 * @brief Load/save TrackStore from .isat_tracks IDC (shared by isat_tracks and incremental SfM).
 *
 * Schema 1.0: tracks + observations only.
 * Schema 1.1: adds optional "view_graph_pairs" JSON array (PairGeoInfo per covisible edge, aligned
 *              with filtered tracks when written by isat_tracks).
 * Schema 1.2: adds SfM-result metadata (is_sfm_result, num_triangulated, num_inlier,
 *              num_registered_images) and preserves kHasTriangulated bit in track_flags.
 *              Backward compatible: load side only tests &kAlive; extra bits are ignored.
 */

#pragma once

#include "../modules/sfm/track_store.h"
#include <string>
#include <vector>

namespace insight {
namespace sfm {

class ViewGraph;

// ─────────────────────────────────────────────────────────────────────────────
// Save options
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Optional extra metadata to embed when saving a TrackStore that is the result
 * of incremental SfM (as opposed to a raw track-building step).
 *
 * When is_sfm_result == true the writer:
 *   - Preserves kHasTriangulated in the saved track_flags byte (backward compat:
 *     load side only checks &kAlive, additional bits are silently ignored).
 *   - Bumps schema_version to "1.2".
 *   - Embeds is_sfm_result, num_registered_images, num_triangulated,
 *     num_inlier, num_outlier, num_not_triangulated in the JSON header.
 */
struct TrackSaveOptions {
    // ── SfM-result metadata ─────────────────────────────────────────────────
    bool is_sfm_result          = false; ///< Mark this as a post-SfM result file.
    int  num_registered_images  = 0;     ///< How many images were successfully registered.
    int  num_triangulated       = -1;    ///< Tracks with kHasTriangulated set (-1 = auto-count).
    int  num_inlier             = -1;    ///< Alive triangulated tracks (-1 = auto-count).
};

// ─────────────────────────────────────────────────────────────────────────────
// Load / Save API
// ─────────────────────────────────────────────────────────────────────────────

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
 * Write TrackStore to .isat_tracks IDC.
 *
 * When \p view_graph is non-null and non-empty, embeds view_graph_pairs.
 * When \p opts is non-null with opts->is_sfm_result == true, emits SfM stats
 * and preserves kHasTriangulated in the track_flags blob.
 */
bool save_track_store_to_idc(const TrackStore& store, const std::vector<uint32_t>& image_indices,
                             const std::string& path,
                             const ViewGraph* view_graph        = nullptr,
                             const TrackSaveOptions* opts       = nullptr);

} // namespace sfm
} // namespace insight
