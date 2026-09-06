#include "track_graph_idc.h"
#include "idc_reader.h"
#include "idc_writer.h"
#include <glog/logging.h>
#include <nlohmann/json.hpp>

namespace insight {
namespace sfm {

bool save_track_graph_to_idc(const TrackGraphStore& graph, const std::string& path) {
  const auto& owners = graph.graph_owner_track_id();
  const auto& nodes = graph.graph_node_offset();
  const auto& adj = graph.graph_adj_offset();
  const auto& neighbors = graph.graph_adj_neighbor();
  nlohmann::json meta;
  meta["schema_version"] = "1.0";
  meta["task_type"] = "track_graph";
  meta["num_graphs"] = owners.size();
  meta["num_graph_nodes"] = nodes.empty() ? 0 : nodes.back();
  meta["num_adjacency_entries"] = neighbors.size();
  meta["num_undirected_edges"] = neighbors.size() / 2u;
  meta["min_split_degree"] = TrackGraphStore::kMinSplitDegree;
  meta["adj_mask_bit"] = 15;
  meta["adj_index_bits"] = 15;
  meta["offset_bits"] = 64;
  meta["observation_mapping"] = "owner_track_parent_observation_order";
  io::IDCWriter writer(path);
  writer.set_metadata(meta);
  writer.add_blob("graph_owner_track_id", owners.data(), owners.size() * sizeof(uint32_t), "uint32",
                  {static_cast<int>(owners.size())});
  writer.add_blob("graph_node_offset", nodes.data(), nodes.size() * sizeof(uint64_t), "uint64",
                  {static_cast<int>(nodes.size())});
  writer.add_blob("graph_adj_offset", adj.data(), adj.size() * sizeof(uint64_t), "uint64",
                  {static_cast<int>(adj.size())});
  writer.add_blob("graph_adj_neighbor", neighbors.data(), neighbors.size() * sizeof(uint16_t), "uint16",
                  {static_cast<int>(neighbors.size())});
  return writer.write();
}

bool load_track_graph_from_idc(const std::string& path, TrackGraphStore* graph) {
  if (!graph) return false;
  io::IDCReader reader(path);
  if (!reader.is_valid()) return false;
  const auto& meta = reader.get_metadata();
  if (meta.value("task_type", std::string()) != "track_graph") return false;
  if (meta.value("adj_index_bits", 0) != 15 || meta.value("adj_mask_bit", -1) != 15)
    return false;
  const auto owners = reader.read_blob<uint32_t>("graph_owner_track_id");
  const auto nodes = reader.read_blob<uint64_t>("graph_node_offset");
  const auto adj = reader.read_blob<uint64_t>("graph_adj_offset");
  const auto neighbors = reader.read_blob<uint16_t>("graph_adj_neighbor");
  std::string error;
  if (!graph->assign_serialized(owners, nodes, adj, neighbors, &error)) {
    LOG(ERROR) << "load_track_graph_from_idc: " << error;
    return false;
  }
  const uint64_t node_count = nodes.empty() ? 0u : nodes.back();
  if (meta.value("num_graphs", owners.size()) != owners.size() ||
      meta.value("num_graph_nodes", node_count) != node_count ||
      meta.value("num_adjacency_entries", neighbors.size()) != neighbors.size())
    return false;
  return true;
}

} // namespace sfm
} // namespace insight
