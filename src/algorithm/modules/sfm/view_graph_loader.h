/**
 * @file  view_graph_loader.h
 * @brief Build ViewGraph from pairs.json + geo directory (.isat_geo files).
 *
 * Convention: pairs JSON uses image1_index / image2_index; geo path = geo_dir + "{idx1}_{idx2}.isat_geo".
 */

#pragma once

#include "view_graph.h"
#include <string>

namespace insight {
namespace sfm {

/// Load pairs from JSON (array "pairs" with image1_index, image2_index),
/// then fill ViewGraph from each pair's .isat_geo in geo_dir. Skips missing/invalid geo.
/// Geo path: geo_dir + "/" + idx1 + "_" + idx2 + ".isat_geo" (index-based).
bool build_view_graph_from_geo(const std::string& pairs_json_path, const std::string& geo_dir,
                               ViewGraph* out);

} // namespace sfm
} // namespace insight
