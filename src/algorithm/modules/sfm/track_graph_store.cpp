#include "track_graph_store.h"
#include <algorithm>
#include <cassert>
#include <limits>
#include <stdexcept>
#include <unordered_set>

namespace insight {
namespace sfm {
namespace {
uint64_t edge_key(uint32_t u, uint32_t v) {
  if (u > v) std::swap(u, v);
  return (static_cast<uint64_t>(u) << 32) | v;
}
}

TrackGraphStore::GraphAdjValue TrackGraphStore::encode_neighbor(uint32_t local, bool masked) {
  if (local > kGraphAdjIndexMask)
    throw std::out_of_range("graph local node exceeds 15-bit index range");
  return static_cast<GraphAdjValue>(local | (masked ? kGraphAdjMask : 0));
}

uint32_t TrackGraphStore::decode_neighbor(GraphAdjValue value) {
  return static_cast<uint32_t>(value & kGraphAdjIndexMask);
}

bool TrackGraphStore::graph_valid(uint32_t graph_id) const {
  return graph_id < graph_owner_track_id_.size();
}

size_t TrackGraphStore::num_nodes(uint32_t graph_id) const {
  if (!graph_valid(graph_id)) return 0;
  return static_cast<size_t>(graph_node_offset_[graph_id + 1] - graph_node_offset_[graph_id]);
}

size_t TrackGraphStore::num_adjacency_entries(uint32_t graph_id) const {
  if (!graph_valid(graph_id)) return 0;
  const size_t first = static_cast<size_t>(graph_node_offset_[graph_id]);
  const size_t last = static_cast<size_t>(graph_node_offset_[graph_id + 1]);
  return static_cast<size_t>(graph_adj_offset_[last] - graph_adj_offset_[first]);
}

uint32_t TrackGraphStore::graph_owner_track_id(uint32_t graph_id) const {
  return graph_valid(graph_id) ? graph_owner_track_id_[graph_id] : TrackStore::kInvalidGraphId;
}

bool TrackGraphStore::fail(std::string* error, const std::string& message) const {
  if (error) *error = message;
  return false;
}

uint32_t TrackGraphStore::add_graph(uint32_t owner_track_id, size_t node_count,
                                    const std::vector<Edge>& edges, std::string* error) {
  if (node_count > kGraphAdjIndexMask)
    return fail(error, "graph node count exceeds 15-bit local index range")
               ? 0u : TrackStore::kInvalidGraphId;
  std::unordered_set<uint64_t> unique;
  unique.reserve(edges.size());
  std::vector<Edge> filtered;
  filtered.reserve(edges.size());
  for (const Edge& e : edges) {
    if (e.local_u >= node_count || e.local_v >= node_count)
      return fail(error, "graph edge local index out of range") ? 0u : TrackStore::kInvalidGraphId;
    if (e.local_u == e.local_v)
      return fail(error, "graph self edge is not allowed") ? 0u : TrackStore::kInvalidGraphId;
    if (unique.insert(edge_key(e.local_u, e.local_v)).second) {
      Edge normalized = e;
      if (normalized.local_u > normalized.local_v) std::swap(normalized.local_u, normalized.local_v);
      filtered.push_back(normalized);
    }
  }
  const uint32_t graph_id = static_cast<uint32_t>(graph_owner_track_id_.size());
  if (graph_adj_offset_.empty()) graph_adj_offset_.push_back(0u);
  if (graph_node_offset_.empty()) graph_node_offset_.push_back(0u);
  graph_owner_track_id_.push_back(owner_track_id);
  const uint64_t node_base = graph_node_offset_.back();
  std::vector<uint32_t> degree(node_count, 0u);
  for (const Edge& e : filtered) { ++degree[e.local_u]; ++degree[e.local_v]; }
  for (uint32_t d : degree) graph_adj_offset_.push_back(graph_adj_offset_.back() + d);
  graph_node_offset_.push_back(node_base + node_count);
  assert(graph_node_offset_.size() == graph_owner_track_id_.size() + 1u);
  std::vector<uint64_t> write_pos(node_count);
  for (size_t i = 0; i < node_count; ++i)
    write_pos[i] = graph_adj_offset_[static_cast<size_t>(node_base) + i];
  graph_adj_neighbor_.resize(static_cast<size_t>(graph_adj_offset_.back()));
  for (const Edge& e : filtered) {
    graph_adj_neighbor_[write_pos[e.local_u]++] = encode_neighbor(e.local_v);
    graph_adj_neighbor_[write_pos[e.local_v]++] = encode_neighbor(e.local_u);
  }
  return graph_id;
}

bool TrackGraphStore::assign_serialized(const std::vector<uint32_t>& owners,
                                        const std::vector<uint64_t>& node_offsets,
                                        const std::vector<uint64_t>& adj_offsets,
                                        const std::vector<GraphAdjValue>& neighbors,
                                        std::string* error) {
  if (node_offsets.size() != owners.size() + 1u || node_offsets.empty() ||
      adj_offsets.empty() || node_offsets.back() + 1u > adj_offsets.size() ||
      adj_offsets.back() != neighbors.size())
    return fail(error, "serialized graph offset sizes are inconsistent");
  if (node_offsets.front() != 0 || adj_offsets.front() != 0)
    return fail(error, "serialized graph offsets must start at zero");
  for (size_t i = 1; i < node_offsets.size(); ++i)
    if (node_offsets[i] < node_offsets[i - 1]) return fail(error, "node offsets are not monotonic");
  for (size_t i = 1; i < adj_offsets.size(); ++i)
    if (adj_offsets[i] < adj_offsets[i - 1]) return fail(error, "adjacency offsets are not monotonic");
  for (size_t g = 0; g < owners.size(); ++g) {
    const uint64_t graph_nodes = node_offsets[g + 1] - node_offsets[g];
    const size_t row_begin = static_cast<size_t>(node_offsets[g]);
    const size_t row_end = static_cast<size_t>(node_offsets[g + 1]);
    for (size_t row = row_begin; row < row_end; ++row) {
      const size_t adj_begin = static_cast<size_t>(adj_offsets[row]);
      const size_t adj_end = static_cast<size_t>(adj_offsets[row + 1]);
      for (size_t p = adj_begin; p < adj_end; ++p)
        if (decode_neighbor(neighbors[p]) >= graph_nodes)
          return fail(error, "serialized adjacency local index is out of range");
    }
  }
  graph_owner_track_id_ = owners;
  graph_node_offset_ = node_offsets;
  graph_adj_offset_ = adj_offsets;
  graph_adj_neighbor_ = neighbors;
  for (uint32_t g = 0; g < owners.size(); ++g)
    if (!validate_graph(g, error)) { clear(); return false; }
  return true;
}

void TrackGraphStore::clear() {
  graph_owner_track_id_.clear();
  graph_node_offset_.assign(1, 0u);
  graph_adj_offset_.assign(1, 0u);
  graph_adj_neighbor_.clear();
}

bool TrackGraphStore::row_range(uint32_t graph_id, uint32_t local, size_t* begin, size_t* end) const {
  if (!graph_valid(graph_id) || local >= num_nodes(graph_id)) return false;
  const size_t row = static_cast<size_t>(graph_node_offset_[graph_id]) + local;
  *begin = static_cast<size_t>(graph_adj_offset_[row]);
  *end = static_cast<size_t>(graph_adj_offset_[row + 1]);
  return true;
}

bool TrackGraphStore::find_adj_position(uint32_t graph_id, uint32_t local, uint32_t neighbor,
                                        size_t* position) const {
  size_t begin = 0, end = 0;
  if (!row_range(graph_id, local, &begin, &end)) return false;
  for (size_t p = begin; p < end; ++p)
    if (decode_neighbor(graph_adj_neighbor_[p]) == neighbor) { *position = p; return true; }
  return false;
}

void TrackGraphStore::remember_old_value(GraphMaskTransaction* transaction, size_t position) {
  if (!transaction) return;
  for (const auto& old : transaction->old_adj_values)
    if (old.first == position) return;
  transaction->old_adj_values.emplace_back(position, graph_adj_neighbor_[position]);
}

bool TrackGraphStore::set_edge_mask(uint32_t graph_id, uint32_t local_u, uint32_t local_v,
                                    bool masked, GraphMaskTransaction* transaction) {
  size_t p = 0, q = 0;
  if (!find_adj_position(graph_id, local_u, local_v, &p) ||
      !find_adj_position(graph_id, local_v, local_u, &q)) return false;
  remember_old_value(transaction, p); remember_old_value(transaction, q);
  graph_adj_neighbor_[p] = encode_neighbor(local_v, masked);
  graph_adj_neighbor_[q] = encode_neighbor(local_u, masked);
  return true;
}

bool TrackGraphStore::edge_masked(uint32_t graph_id, uint32_t local_u, uint32_t local_v) const {
  size_t p = 0;
  return find_adj_position(graph_id, local_u, local_v, &p) && is_masked(graph_adj_neighbor_[p]);
}

bool TrackGraphStore::begin_transaction(TrackStore* store, uint32_t graph_id,
                                         GraphMaskTransaction* out) {
  if (!store || !out || !graph_valid(graph_id) || out->active) return false;
  out->graph_id = graph_id; out->active = true; out->old_adj_values.clear();
  store->capture_graph_mutation_state(&out->track_state);
  return true;
}

bool TrackGraphStore::rollback_transaction(TrackStore* store, GraphMaskTransaction* transaction) {
  if (!store || !transaction || !transaction->active) return false;
  for (auto it = transaction->old_adj_values.rbegin(); it != transaction->old_adj_values.rend(); ++it)
    graph_adj_neighbor_[it->first] = it->second;
  const bool ok = store->restore_graph_mutation_state(transaction->track_state);
  transaction->active = false;
  return ok;
}

bool TrackGraphStore::commit_transaction(GraphMaskTransaction* transaction) {
  if (!transaction || !transaction->active) return false;
  transaction->active = false;
  transaction->old_adj_values.clear();
  return true;
}

bool TrackGraphStore::rebuild_active_tracks_from_graph(TrackStore* store, uint32_t graph_id,
                                                        GraphMaskTransaction* transaction,
                                                        std::vector<int>* active_tracks) {
  if (!store || !graph_valid(graph_id) || !transaction || !transaction->active ||
      transaction->graph_id != graph_id) return false;
  const int owner = static_cast<int>(graph_owner_track_id_[graph_id]);
  // add_track() below may grow TrackStore's vector-of-vectors. Copy the stable
  // parent-local mapping before creating children so no reference is invalidated.
  const std::vector<int> parent_obs = store->track_all_obs_ids_view(owner);
  if (parent_obs.size() != num_nodes(graph_id)) return false;
  const size_t n = parent_obs.size();
  std::vector<int> parent(n, -1);
  std::vector<uint32_t> images(n, 0);
  for (size_t i = 0; i < n; ++i) {
    if (!store->is_obs_valid(parent_obs[i])) continue;
    images[i] = store->obs_image_index(parent_obs[i]);
    parent[i] = static_cast<int>(i);
  }
  auto find = [&](int x) { while (parent[static_cast<size_t>(x)] != x) {
      parent[static_cast<size_t>(x)] = parent[static_cast<size_t>(parent[static_cast<size_t>(x)])];
      x = parent[static_cast<size_t>(x)]; } return x; };
  const size_t row_base = static_cast<size_t>(graph_node_offset_[graph_id]);
  for (size_t local = 0; local < n; ++local) {
    if (parent[local] < 0) continue;
    const size_t b = static_cast<size_t>(graph_adj_offset_[row_base + local]);
    const size_t e = static_cast<size_t>(graph_adj_offset_[row_base + local + 1]);
    for (size_t p = b; p < e; ++p) {
      const auto value = graph_adj_neighbor_[p];
      if (is_masked(value)) continue;
      const uint32_t other = decode_neighbor(value);
      if (other <= local || other >= n || parent[other] < 0) continue;
      const int a = find(static_cast<int>(local));
      const int b_root = find(static_cast<int>(other));
      if (a == b_root) continue;
      bool duplicate_image = false;
      for (size_t j = 0; j < n && !duplicate_image; ++j) {
        if (parent[j] < 0 || find(static_cast<int>(j)) != a) continue;
        for (size_t k = 0; k < n; ++k) {
          if (parent[k] >= 0 && find(static_cast<int>(k)) == b_root && images[j] == images[k]) {
            duplicate_image = true;
            break;
          }
        }
      }
      if (!duplicate_image) parent[static_cast<size_t>(b_root)] = a;
    }
  }
  std::vector<std::vector<int>> components(n);
  for (size_t i = 0; i < n; ++i)
    if (parent[i] >= 0) components[static_cast<size_t>(find(static_cast<int>(i)))].push_back(static_cast<int>(i));
  std::vector<std::vector<int>> nonempty;
  for (auto& c : components) {
    if (c.empty()) continue;
    if (c.size() < 2) return false;
    nonempty.push_back(std::move(c));
  }
  if (nonempty.empty()) return false;
  // A replay is authoritative for the whole graph. Existing children from a previous
  // split must become history before new components are materialized.
  for (int tid = 0; tid < static_cast<int>(store->num_tracks()); ++tid) {
    if (tid == owner || store->track_graph_id(tid) != graph_id ||
        !store->is_track_valid(tid))
      continue;
    store->set_track_split_parent(tid, true);
    store->clear_track_xyz(tid);
  }
  store->mark_track_split_parent(owner);
  if (active_tracks) active_tracks->clear();
  for (const auto& component : nonempty) {
    const int child = store->add_track(0.f, 0.f, 0.f);
    store->set_track_graph_id(child, graph_id);
    store->set_track_parent_id(child, owner);
    for (int local : component)
      if (!store->attach_existing_observation(child, parent_obs[static_cast<size_t>(local)])) return false;
    store->set_track_retriangulation_flag(child, true);
    if (active_tracks) active_tracks->push_back(child);
  }
  return true;
}

bool TrackGraphStore::split(TrackStore* store, uint32_t graph_id,
                            const std::vector<Edge>& edges_to_mask, std::vector<int>* children) {
  GraphMaskTransaction tx;
  if (!begin_transaction(store, graph_id, &tx)) return false;
  for (const Edge& e : edges_to_mask)
    if (!set_edge_mask(graph_id, e.local_u, e.local_v, true, &tx)) { rollback_transaction(store, &tx); return false; }
  if (!rebuild_active_tracks_from_graph(store, graph_id, &tx, children)) { rollback_transaction(store, &tx); return false; }
  return commit_transaction(&tx);
}

bool TrackGraphStore::merge(TrackStore* store, uint32_t graph_id,
                            const std::vector<Edge>& edges_to_unmask, std::vector<int>* active_tracks) {
  GraphMaskTransaction tx;
  if (!begin_transaction(store, graph_id, &tx)) return false;
  for (const Edge& e : edges_to_unmask)
    if (!set_edge_mask(graph_id, e.local_u, e.local_v, false, &tx)) { rollback_transaction(store, &tx); return false; }
  if (!rebuild_active_tracks_from_graph(store, graph_id, &tx, active_tracks)) { rollback_transaction(store, &tx); return false; }
  return commit_transaction(&tx);
}

bool TrackGraphStore::validate_graph(uint32_t graph_id, std::string* error) const {
  if (!graph_valid(graph_id)) return fail(error, "invalid graph id");
  const size_t n = num_nodes(graph_id);
  for (size_t u = 0; u < n; ++u) {
    size_t b = 0, e = 0; row_range(graph_id, static_cast<uint32_t>(u), &b, &e);
    for (size_t p = b; p < e; ++p) {
      const uint32_t v = decode_neighbor(graph_adj_neighbor_[p]);
      if (v >= n) return fail(error, "adjacency local index out of range");
      size_t q = 0;
      if (!find_adj_position(graph_id, v, static_cast<uint32_t>(u), &q) ||
          is_masked(graph_adj_neighbor_[q]) != is_masked(graph_adj_neighbor_[p]))
        return fail(error, "CSR adjacency is not symmetric");
    }
  }
  return true;
}

bool TrackGraphStore::validate_track_graph_consistency(const TrackStore& store, uint32_t graph_id,
                                                       std::string* error) const {
  if (!validate_graph(graph_id, error)) return false;
  const uint32_t owner = graph_owner_track_id_[graph_id];
  const auto& obs = store.track_all_obs_ids_view(static_cast<int>(owner));
  if (obs.size() != num_nodes(graph_id)) return fail(error, "owner observation list size mismatch");
  for (size_t i = 0; i < obs.size(); ++i) {
    if (!store.is_obs_valid(obs[i])) continue;
    const int tid = store.obs_track_id(obs[i]);
    if (tid < 0 || store.track_graph_id(tid) != graph_id || !store.is_track_valid(tid))
      return fail(error, "alive graph observation has invalid active owner");
    const auto& child_obs = store.track_all_obs_ids_view(tid);
    if (std::find(child_obs.begin(), child_obs.end(), obs[i]) == child_obs.end())
      return fail(error, "observation missing from active track list");
  }
  return true;
}
} // namespace sfm
} // namespace insight
