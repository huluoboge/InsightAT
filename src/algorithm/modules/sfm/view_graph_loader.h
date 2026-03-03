/**
 * @file  view_graph_loader.h
 * @brief Build ViewGraph from pairs.json + geo directory (.isat_geo files).
 *
 * Uses IDCReader and nlohmann::json; link with algorithm target that has io.
 */

#pragma once

#include "view_graph.h"
#include <string>

namespace insight {
namespace sfm {

/// Load pairs from JSON (array "pairs" with image1_id, image2_id), then fill
/// ViewGraph from each pair's .isat_geo in geo_dir. Skips missing/invalid geo.
/// Geo path: geo_dir + "/" + image1_id + "_" + image2_id + ".isat_geo"
bool build_view_graph_from_geo(const std::string& pairs_json_path, const std::string& geo_dir,
                               ViewGraph* out);

} // namespace sfm
} // namespace insight
