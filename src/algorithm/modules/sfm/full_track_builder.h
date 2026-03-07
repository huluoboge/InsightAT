/**
 * @file  full_track_builder.h
 * @brief Build full N-image TrackStore from all pairs via union-find (match + geo).
 *
 * Phase 1: Union-Find over (original_image_id, feature_id) from all .isat_match.
 * Phase 2: Fill observations and optional xyz from match + geo inliers; image identity
 *          in the store is internal index (0..n-1). When IdMapping is provided, index
 *          order matches id_mapping; otherwise image order is unique original ids from
 *          pairs, sorted.
 *
 * Used by incremental SfM to build the full track set before selecting the initial pair.
 * Requires IDCReader and pair JSON utils (link with algorithm target that has io).
 */

#pragma once

#include "id_mapping.h"
#include "track_store.h"
#include <string>

namespace insight {
namespace sfm {

/**
 * Build a full TrackStore from pairs.json and geo/match directories.
 *
 * - Loads all pairs from pairs_json_path; for each pair reads .isat_match and .isat_geo.
 * - Union-Find merges (original_image_id, feature_id) across pairs into track equivalence classes.
 * - Fills store with observations (image_index = internal 0..n-1); optionally sets track xyz
 *   from geo points3d when twoview data is present.
 *
 * @param pairs_json_path  Path to pairs JSON (must have "pairs" array with image1_id, image2_id).
 * @param geo_dir          Directory of .isat_geo files (path: geo_dir/orig1_orig2.isat_geo).
 * @param match_dir       Directory of .isat_match files (path: match_dir/orig1_orig2.isat_match).
 * @param id_mapping      When non-null: image count and order from id_mapping; observations use
 *                        internal index. When null: image list = unique original ids from pairs,
 *                        sorted; store uses that order as index 0..n-1.
 * @param store_out       Output track store (set_num_images, tracks, observations filled).
 * @return true if at least one pair was loaded and store has tracks; false on parse/io error or
 *         no valid pairs.
 */
bool build_full_track_store_from_pairs(const std::string& pairs_json_path,
                                       const std::string& geo_dir,
                                       const std::string& match_dir,
                                       const IdMapping* id_mapping,
                                       TrackStore* store_out);

} // namespace sfm
} // namespace insight
