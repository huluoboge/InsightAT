/**
 * @file  view_graph_loader.h
 * @brief Build ViewGraph from pairs.json + geo directory (.isat_geo files).
 *
 * Uses IDCReader and nlohmann::json; link with algorithm target that has io.
 */

#pragma once

#include "id_mapping.h"
#include "view_graph.h"
#include <string>

namespace insight {
namespace sfm {

/// Load pairs from JSON (array "pairs" with image1_id, image2_id), then fill
/// ViewGraph from each pair's .isat_geo in geo_dir. Skips missing/invalid geo.
/// Geo path: geo_dir + "/" + orig_id1 + "_" + orig_id2 + ".isat_geo" (orig_id from id_mapping at boundary).
/// When id_mapping is non-null: pair JSON uses original ids → we convert to indices for the graph.
bool build_view_graph_from_geo(const std::string& pairs_json_path, const std::string& geo_dir,
                               ViewGraph* out, const IdMapping* id_mapping = nullptr);

} // namespace sfm
} // namespace insight
