/**
 * @file  view_graph_loader.h
 * @brief Build ViewGraph from pairs.json + geo directory (.isat_geo files).
 *
 * Convention: pairs JSON uses image1_index / image2_index; geo path = geo_dir + "{idx1}_{idx2}.isat_geo".
 *
 * Also: embed ViewGraph when building tracks — only **direct pairs from pairs.json** (same as matching)
 * intersected with **co-visibility in the filtered TrackStore**; not all-pairs cliques on a track
 * (transitive image pairs have no .isat_geo). (De)serialize to tracks IDC metadata JSON.
 */

#pragma once

#include "view_graph.h"
#include <nlohmann/json.hpp>
#include <string>
#include <utility>
#include <vector>

namespace insight {
namespace sfm {

class TrackStore;

/// Fill \p info's geometry fields from an .isat_geo IDC metadata JSON (geometry + optional twoview).
/// Caller sets image1_index / image2_index on \p info before calling.
void pair_geo_info_from_isat_geo_metadata(const nlohmann::json& meta, PairGeoInfo* info);

/// Load one pair's summary from a single .isat_geo file (canonical idx1 <= idx2 in filename).
bool load_pair_geo_info_from_isat_geo_file(const std::string& geo_path, uint32_t image1_index,
                                           uint32_t image2_index, PairGeoInfo* out);

/// Covisibility edges from filtered tracks: all unordered pairs (i,j) with i<j that co-appear on ≥1 track
/// (cliques per track — includes transitive pairs that may have no direct geo file).
void collect_covisible_image_pairs_from_track_store(const TrackStore& store,
                                                    std::vector<std::pair<uint32_t, uint32_t>>* out_pairs);

/// ViewGraph for incremental init: **canonical_pairs** must be the same direct list as pairs.json
/// (image1_index ≤ image2_index). Only pairs that still have co-visibility in \p store after filtering
/// are kept; then geo is loaded from geo_dir (skips missing files, e.g. too few inliers to write geo).
bool build_view_graph_from_pairs_list_and_track_store(
    const std::vector<std::pair<uint32_t, uint32_t>>& canonical_pairs, const std::string& geo_dir,
    const TrackStore& store, ViewGraph* out);

/// Serialize view graph for embedding in tracks IDC metadata under key "view_graph_pairs".
nlohmann::json view_graph_pairs_to_json_array(const ViewGraph& vg);
bool view_graph_from_json_array(const nlohmann::json& arr, ViewGraph* out);

/// Load pairs from JSON (array "pairs" with image1_index, image2_index),
/// then fill ViewGraph from each pair's .isat_geo in geo_dir. Skips missing/invalid geo.
/// Geo path: geo_dir + "/" + idx1 + "_" + idx2 + ".isat_geo" (index-based).
bool build_view_graph_from_geo(const std::string& pairs_json_path, const std::string& geo_dir,
                               ViewGraph* out);

} // namespace sfm
} // namespace insight
