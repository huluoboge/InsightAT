#pragma once

#include "../modules/sfm/track_graph_store.h"
#include <string>

namespace insight {
namespace sfm {

bool save_track_graph_to_idc(const TrackGraphStore& graph, const std::string& path);
bool load_track_graph_from_idc(const std::string& path, TrackGraphStore* graph);

} // namespace sfm
} // namespace insight
