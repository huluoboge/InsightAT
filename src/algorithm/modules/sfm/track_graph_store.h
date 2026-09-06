/**
 * @file track_graph_store.h
 * @brief Sparse verified-correspondence graphs shared by soft-split tracks.
 */
#pragma once

#include "track_store.h"
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace insight {
namespace sfm {

class TrackGraphStore {
public:
  using GraphAdjValue = uint16_t;
  static constexpr GraphAdjValue kGraphAdjMask = 0x8000u;
  static constexpr GraphAdjValue kGraphAdjIndexMask = 0x7fffu;
  static constexpr GraphAdjValue kInvalidGraphAdjValue = 0xffffu;
  static constexpr uint32_t kMinSplitDegree = 4u;

  struct Edge {
    uint32_t local_u = 0;
    uint32_t local_v = 0;
  };

  struct GraphMaskTransaction {
    uint32_t graph_id = TrackStore::kInvalidGraphId;
    bool active = false;
    TrackStore::GraphMutationState track_state;
    std::vector<std::pair<size_t, GraphAdjValue>> old_adj_values;
  };

  static GraphAdjValue encode_neighbor(uint32_t local, bool masked = false);
  static uint32_t decode_neighbor(GraphAdjValue value);
  static bool is_masked(GraphAdjValue value) { return (value & kGraphAdjMask) != 0; }

  /// Build one graph from the owner's stable observation order and real input edges.
  /// Duplicate undirected edges are removed; self edges and out-of-range endpoints fail.
  uint32_t add_graph(uint32_t owner_track_id, size_t node_count,
                     const std::vector<Edge>& edges, std::string* error = nullptr);
  /// Replace the flat CSR arrays loaded from an .isat_graph sidecar.
  bool assign_serialized(const std::vector<uint32_t>& owners,
                         const std::vector<uint64_t>& node_offsets,
                         const std::vector<uint64_t>& adj_offsets,
                         const std::vector<GraphAdjValue>& neighbors,
                         std::string* error = nullptr);
  void clear();

  size_t num_graphs() const { return graph_owner_track_id_.size(); }
  size_t num_nodes(uint32_t graph_id) const;
  size_t num_adjacency_entries(uint32_t graph_id) const;
  uint32_t graph_owner_track_id(uint32_t graph_id) const;
  const std::vector<uint32_t>& graph_owner_track_id() const { return graph_owner_track_id_; }
  const std::vector<uint64_t>& graph_node_offset() const { return graph_node_offset_; }
  const std::vector<uint64_t>& graph_adj_offset() const { return graph_adj_offset_; }
  const std::vector<GraphAdjValue>& graph_adj_neighbor() const { return graph_adj_neighbor_; }

  bool set_edge_mask(uint32_t graph_id, uint32_t local_u, uint32_t local_v,
                    bool masked, GraphMaskTransaction* transaction = nullptr);
  bool edge_masked(uint32_t graph_id, uint32_t local_u, uint32_t local_v) const;

  bool begin_transaction(TrackStore* store, uint32_t graph_id, GraphMaskTransaction* out);
  bool rollback_transaction(TrackStore* store, GraphMaskTransaction* transaction);
  bool commit_transaction(GraphMaskTransaction* transaction);

  /// Recompute active components from the graph owner list and create child tracks.
  /// A component with fewer than two observations is rejected. The graph owner remains
  /// the immutable local-index mapping and is retained as a split parent.
  bool rebuild_active_tracks_from_graph(TrackStore* store, uint32_t graph_id,
                                       GraphMaskTransaction* transaction,
                                       std::vector<int>* active_tracks = nullptr);

  bool split(TrackStore* store, uint32_t graph_id,
             const std::vector<Edge>& edges_to_mask,
             std::vector<int>* children = nullptr);
  bool merge(TrackStore* store, uint32_t graph_id,
             const std::vector<Edge>& edges_to_unmask,
             std::vector<int>* active_tracks = nullptr);

  bool validate_graph(uint32_t graph_id, std::string* error = nullptr) const;
  bool validate_track_graph_consistency(const TrackStore& store, uint32_t graph_id,
                                        std::string* error = nullptr) const;

private:
  bool graph_valid(uint32_t graph_id) const;
  bool row_range(uint32_t graph_id, uint32_t local, size_t* begin, size_t* end) const;
  bool find_adj_position(uint32_t graph_id, uint32_t local, uint32_t neighbor,
                         size_t* position) const;
  void remember_old_value(GraphMaskTransaction* transaction, size_t position);
  bool fail(std::string* error, const std::string& message) const;

  std::vector<uint32_t> graph_owner_track_id_;
  std::vector<uint64_t> graph_node_offset_;
  std::vector<uint64_t> graph_adj_offset_;
  std::vector<GraphAdjValue> graph_adj_neighbor_;
};

} // namespace sfm
} // namespace insight
