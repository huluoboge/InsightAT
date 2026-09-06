#include "track_graph_store.h"

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

using insight::sfm::TrackGraphStore;
using insight::sfm::TrackStore;

namespace {
int fail(const std::string& message) {
  std::cerr << "  FAIL: " << message << "\n";
  return 1;
}

int test_encoding() {
  std::cout << "[test1] adjacency encoding\n";
  if (TrackGraphStore::decode_neighbor(TrackGraphStore::encode_neighbor(0)) != 0)
    return fail("zero index did not round-trip");
  const auto max_value = TrackGraphStore::encode_neighbor(0x7fffu, true);
  if (!TrackGraphStore::is_masked(max_value) ||
      TrackGraphStore::decode_neighbor(max_value) != 0x7fffu)
    return fail("masked maximum index did not round-trip");
  try {
    (void)TrackGraphStore::encode_neighbor(0x8000u);
    return fail("overflow index must be rejected");
  } catch (const std::out_of_range&) {
  }
  return 0;
}

TrackStore make_store(int* owner, std::vector<int>* obs) {
  TrackStore store;
  store.set_num_images(8);
  *owner = store.add_track(1.f, 2.f, 3.f);
  for (uint32_t image = 0; image < 6; ++image)
    obs->push_back(store.add_observation(*owner, image, 100u + image, 10.f + image, 20.f));
  return store;
}

int test_build_and_offsets() {
  std::cout << "[test2] sparse CSR build and multi-graph offsets\n";
  TrackGraphStore graph;
  std::string error;
  const std::vector<TrackGraphStore::Edge> edges = {{0, 1}, {1, 0}, {1, 2}, {3, 4}};
  const uint32_t g0 = graph.add_graph(7, 5, edges, &error);
  if (g0 != 0 || graph.num_nodes(g0) != 5 || graph.num_adjacency_entries(g0) != 6)
    return fail("duplicate edges were not removed or CSR size is wrong: " + error);
  const uint32_t g1 = graph.add_graph(8, 2, {{0, 1}}, &error);
  if (g1 != 1 || graph.num_nodes(g1) != 2 || graph.num_adjacency_entries(g1) != 2)
    return fail("second graph offset is wrong");
  if (!graph.validate_graph(g0, &error) || !graph.validate_graph(g1, &error))
    return fail("CSR symmetry validation failed: " + error);
  if (graph.edge_masked(g0, 0, 1)) return fail("edge must start unmasked");
  if (!graph.set_edge_mask(g0, 0, 1, true) || !graph.edge_masked(g0, 1, 0))
    return fail("mask was not synchronized in both directions");
  if (graph.add_graph(9, 0x8000u, {}, &error) != TrackStore::kInvalidGraphId)
    return fail("oversized graph was accepted");
  if (graph.add_graph(9, 2, {{0, 2}}, &error) != TrackStore::kInvalidGraphId)
    return fail("out-of-range graph edge was accepted");
  if (graph.add_graph(9, 2, {{1, 1}}, &error) != TrackStore::kInvalidGraphId)
    return fail("self graph edge was accepted");
  return 0;
}

int test_serialized_validation() {
  std::cout << "[test3] serialized CSR validation\n";
  TrackGraphStore graph;
  std::string error;
  const std::vector<uint32_t> owners = {10};
  const std::vector<uint64_t> nodes = {0, 2};
  const std::vector<uint64_t> adj = {0, 1, 2};
  const std::vector<TrackGraphStore::GraphAdjValue> neighbors = {
      TrackGraphStore::encode_neighbor(1), TrackGraphStore::encode_neighbor(0)};
  if (!graph.assign_serialized(owners, nodes, adj, neighbors, &error) ||
      graph.num_graphs() != 1 || graph.num_nodes(0) != 2)
    return fail("valid serialized CSR was rejected: " + error);

  auto expect_rejected = [&](const std::vector<uint32_t>& o,
                             const std::vector<uint64_t>& n,
                             const std::vector<uint64_t>& a,
                             const std::vector<TrackGraphStore::GraphAdjValue>& v) {
    TrackGraphStore candidate;
    std::string why;
    return !candidate.assign_serialized(o, n, a, v, &why) && !why.empty();
  };
  if (!expect_rejected({}, {}, {0}, {}))
    return fail("empty node offset vector was accepted");
  if (!expect_rejected(owners, {1, 2}, adj, neighbors))
    return fail("node offsets without zero origin were accepted");
  if (!expect_rejected(owners, {0, 2}, {1, 2, 2}, neighbors))
    return fail("adjacency offsets without zero origin were accepted");
  if (!expect_rejected(owners, {0, 2, 1}, {0, 1, 2}, neighbors))
    return fail("non-monotonic node offsets were accepted");
  if (!expect_rejected(owners, {0, 2}, {0, 2, 1}, neighbors))
    return fail("non-monotonic adjacency offsets were accepted");
  if (!expect_rejected(owners, {0, 2}, {0, 1, 3}, neighbors))
    return fail("adjacency size mismatch was accepted");
  if (!expect_rejected(owners, nodes, adj, {TrackGraphStore::encode_neighbor(2),
                                              TrackGraphStore::encode_neighbor(0)}))
    return fail("out-of-range local neighbor was accepted");
  if (!expect_rejected(owners, nodes, adj, {TrackGraphStore::encode_neighbor(1),
                                              TrackGraphStore::encode_neighbor(1)}))
    return fail("asymmetric serialized CSR was accepted");

  graph.clear();
  if (graph.num_graphs() != 0 || graph.graph_node_offset().size() != 1 ||
      graph.graph_adj_offset().size() != 1 || graph.graph_node_offset()[0] != 0 ||
      graph.graph_adj_offset()[0] != 0)
    return fail("clear did not restore empty CSR sentinels");
  if (graph.graph_owner_track_id(0) != TrackStore::kInvalidGraphId ||
      graph.num_nodes(0) != 0 || graph.num_adjacency_entries(0) != 0)
    return fail("invalid graph access did not return safe defaults");
  return 0;
}

int test_split_merge_and_rollback() {
  std::cout << "[test3] split, merge, and rollback\n";
  TrackStore store;
  store.set_num_images(8);
  const int owner = store.add_track(4.f, 5.f, 6.f);
  std::vector<int> obs;
  for (uint32_t image = 0; image < 6; ++image)
    obs.push_back(store.add_observation(owner, image, 10u + image, 1.f, 2.f));
  store.set_track_xyz(owner, 4.f, 5.f, 6.f);

  TrackGraphStore graph;
  const uint32_t gid = graph.add_graph(owner, obs.size(),
      {{0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 5}});
  store.set_track_graph_id(owner, gid);
  std::vector<int> children;
  if (!graph.split(&store, gid, {{2, 3}}, &children) || children.size() != 2)
    return fail("bridge split did not produce two children");
  if (store.is_track_valid(owner) || !store.is_track_split_parent(owner) ||
      store.track_has_triangulated_xyz(owner))
    return fail("parent was not converted to history container");
  if (store.track_all_obs_ids_view(owner).size() != 6)
    return fail("parent observation order was changed");
  if (!store.track_needs_retriangulation(children[0]) ||
      !store.track_needs_retriangulation(children[1]))
    return fail("children were not queued for retriangulation");
  std::string error;
  if (!graph.validate_track_graph_consistency(store, gid, &error))
    return fail("post-split consistency failed: " + error);

  if (!graph.merge(&store, gid, {{2, 3}}, &children) || children.size() != 1)
    return fail("merge did not restore one component");
  if (!graph.validate_track_graph_consistency(store, gid, &error))
    return fail("post-merge consistency failed: " + error);

  const bool before = graph.edge_masked(gid, 0, 1);
  std::vector<int> ignored;
  if (graph.split(&store, gid, {{0, 1}, {99, 100}}, &ignored))
    return fail("invalid split unexpectedly succeeded");
  if (graph.edge_masked(gid, 0, 1) != before)
    return fail("failed transaction did not restore mask");
  if (!store.is_track_valid(children[0]) || store.is_track_split_parent(children[0]))
    return fail("failed transaction did not restore track state");
  return 0;
}

int test_constraint_and_deleted_observation() {
  std::cout << "[test4] image constraint and deleted observations\n";
  TrackStore store;
  store.set_num_images(6);
  const int owner = store.add_track(0.f, 0.f, 0.f);
  const int a = store.add_observation(owner, 0, 1, 0.f, 0.f);
  const int b = store.add_observation(owner, 0, 2, 0.f, 0.f);
  const int c = store.add_observation(owner, 1, 3, 0.f, 0.f);
  const int d = store.add_observation(owner, 2, 4, 0.f, 0.f);
  const int e = store.add_observation(owner, 3, 5, 0.f, 0.f);
  store.mark_observation_deleted(e);
  TrackGraphStore graph;
  const uint32_t gid = graph.add_graph(owner, 5, {{0, 1}, {1, 2}, {2, 3}});
  store.set_track_graph_id(owner, gid);
  std::vector<int> children;
  if (graph.split(&store, gid, {}, &children))
    return fail("duplicate-image constraint should reject singleton component");
  if (store.is_obs_valid(e) || store.obs_track_id(a) != owner || store.obs_track_id(b) != owner)
    return fail("failed constrained replay did not preserve ownership/deletion");
  (void)c;
  (void)d;
  return 0;
}

int test_consistency_failures() {
  std::cout << "[test5] consistency validator failures\n";
  TrackStore store;
  store.set_num_images(3);
  const int owner = store.add_track(0.f, 0.f, 0.f);
  const int o0 = store.add_observation(owner, 0, 1, 0.f, 0.f);
  const int o1 = store.add_observation(owner, 1, 2, 0.f, 0.f);
  TrackGraphStore graph;
  const uint32_t gid = graph.add_graph(owner, 2, {{0, 1}});
  store.set_track_graph_id(owner, gid);
  std::string error;
  TrackStore::GraphMutationState state;
  store.capture_graph_mutation_state(&state);
  const int wrong = store.add_track(0.f, 0.f, 0.f);
  state.obs_track_id[static_cast<size_t>(o0)] = wrong;
  if (!store.restore_graph_mutation_state(state))
    return fail("could not install malformed ownership state");
  if (graph.validate_track_graph_consistency(store, gid, &error))
    return fail("validator accepted observation owned by wrong graph");

  const int child = store.add_track(0.f, 0.f, 0.f);
  store.set_track_graph_id(child, gid);
  store.capture_graph_mutation_state(&state);
  state.obs_track_id[static_cast<size_t>(o0)] = owner;
  state.obs_track_id[static_cast<size_t>(o1)] = child;
  if (!store.restore_graph_mutation_state(state))
    return fail("could not install missing child-list state");
  if (graph.validate_track_graph_consistency(store, gid, &error))
    return fail("validator accepted observation missing from active list");
  return 0;
}
} // namespace

int main() {
  const int failures = test_encoding() + test_build_and_offsets() +
                       test_serialized_validation() +
                       test_split_merge_and_rollback() + test_constraint_and_deleted_observation() +
                       test_consistency_failures();
  if (failures != 0) return EXIT_FAILURE;
  std::cout << "All track graph tests passed\n";
  return EXIT_SUCCESS;
}
