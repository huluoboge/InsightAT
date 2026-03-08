/**
 * isat_tracks.cpp
 * InsightAT Track Building CLI – load match + geo, build tracks, write IDC.
 *
 * Pipeline (two passes; both use geo F/E inliers only for consistent track quality):
 *   Phase 1  Union-Find over geo inlier matches → track equivalence classes
 *   Phase 2  Fill observations (coords + scales) from match + geo inlier masks
 * Track xyz is left for incremental SfM (no two-view 3D). Output: single .isat_tracks IDC.
 *
 * Usage:
 *   isat_tracks -i pairs.json -m match_dir/ -g geo_dir/ -l image_list.json -o tracks.isat_tracks
 *   isat_tracks --stats -o tracks.isat_tracks
 */

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <glog/logging.h>
#include <nlohmann/json.hpp>

#include "pair_json_utils.h"

#include "../io/idc_reader.h"
#include "../io/idc_writer.h"
#include "../modules/sfm/track_builder.h"
#include "../modules/sfm/track_store.h"
#include "cli_logging.h"
#include "cmdLine/cmdLine.h"
#include "isat_intrinsics.h"

namespace fs = std::filesystem;
using json = nlohmann::json;
using namespace insight::io;
using namespace insight::sfm;

static constexpr const char* kEventPrefix = "ISAT_EVENT ";

static void print_event(const json& j) {
  std::cout << kEventPrefix << j.dump() << "\n";
  std::cout.flush();
}

// ─────────────────────────────────────────────────────────────────────────────
// Pairs and image list
// ─────────────────────────────────────────────────────────────────────────────

struct PairDesc {
  uint32_t image1_index = 0;
  uint32_t image2_index = 0;
  std::string match_file;
  std::string geo_file;
};

static std::vector<PairDesc> load_pairs(const std::string& json_path, const std::string& match_dir,
                                       const std::string& geo_dir) {
  std::ifstream file(json_path);
  if (!file.is_open()) {
    LOG(FATAL) << "Cannot open pairs file: " << json_path;
  }
  json j;
  file >> j;
  std::vector<PairDesc> pairs;
  for (const auto& p : j["pairs"]) {
    PairDesc d;
    d.image1_index = insight::tools::get_image_index_from_pair(p, "image1_index");
    d.image2_index = insight::tools::get_image_index_from_pair(p, "image2_index");
    d.match_file = match_dir + "/" + std::to_string(d.image1_index) + "_" +
                   std::to_string(d.image2_index) + ".isat_match";
    d.geo_file = geo_dir + "/" + std::to_string(d.image1_index) + "_" + std::to_string(d.image2_index) +
                 ".isat_geo";
    pairs.push_back(std::move(d));
  }
  LOG(INFO) << "Loaded " << pairs.size() << " pairs from " << json_path;
  return pairs;
}

/** Load image list and validate list index == image_index (design invariant). Returns size = n_images. */
static std::vector<uint32_t> get_image_indices_from_list(const std::string& image_list_path) {
  if (image_list_path.empty())
    return {};
  std::ifstream f(image_list_path);
  if (!f.is_open())
    LOG(FATAL) << "Cannot open image list: " << image_list_path;
  json j;
  f >> j;
  if (!j.contains("images") || !j["images"].is_array())
    LOG(FATAL) << "Image list JSON missing 'images' array";
  const size_t n = j["images"].size();
  std::vector<uint32_t> out;
  out.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    const auto& img = j["images"][i];
    uint32_t idx = img.value("image_index", static_cast<uint32_t>(i));
    if (idx != static_cast<uint32_t>(i))
      LOG(FATAL) << "Image list index != image_index: list[" << i << "].image_index = " << idx
                 << " (expected " << i << "). Export must have list order 0..n-1 = image_index.";
    out.push_back(idx);
  }
  LOG(INFO) << "Image list validated: " << n << " images (index == list position)";
  return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// Union-Find key: (image_index, feature_id) → node (high=image_index, low=feature_id)
// ─────────────────────────────────────────────────────────────────────────────

static uint64_t node_key(uint32_t image_index, uint32_t feature_id) {
  return (static_cast<uint64_t>(image_index) << 32) | feature_id;
}

struct UnionFind {
  std::unordered_map<uint64_t, int> node_id_;  // key (image_index<<32|feat_id) -> internal id
  std::vector<int> parent_;                     // internal id -> parent (internal id)

  int get_or_create(uint64_t key) {
    auto it = node_id_.find(key);
    if (it != node_id_.end())
      return it->second;
    const int id = static_cast<int>(parent_.size());
    node_id_[key] = id;
    parent_.push_back(id);
    return id;
  }

  int find_key(uint64_t key) {
    const int id = get_or_create(key);
    return find_by_id(id);
  }

  void merge_keys(uint64_t k1, uint64_t k2) {
    const int a = find_key(k1);
    const int b = find_key(k2);
    if (a != b)
      parent_[static_cast<size_t>(a)] = b;
  }

  int find_by_id(int i) {
    if (parent_[static_cast<size_t>(i)] == i)
      return i;
    return parent_[static_cast<size_t>(i)] = find_by_id(parent_[static_cast<size_t>(i)]);
  }
};

// ─────────────────────────────────────────────────────────────────────────────
// Track IDC write / read
// ─────────────────────────────────────────────────────────────────────────────

static bool save_track_store_to_idc(const TrackStore& store,
                                    const std::vector<uint32_t>& image_indices,
                                    const std::string& path) {
  const size_t n_tracks = store.num_tracks();
  json meta;
  meta["schema_version"] = "1.0";
  meta["task_type"] = "tracks";
  meta["num_images"] = static_cast<int>(image_indices.size());
  meta["image_indices"] = image_indices; // slot i → global image_index
  meta["num_tracks"] = static_cast<int>(n_tracks);

  std::vector<float> track_xyz(static_cast<size_t>(n_tracks) * 3);
  std::vector<uint8_t> track_flags(static_cast<size_t>(n_tracks));
  for (size_t t = 0; t < n_tracks; ++t) {
    float x, y, z;
    store.get_track_xyz(static_cast<int>(t), &x, &y, &z);
    track_xyz[t * 3] = x;
    track_xyz[t * 3 + 1] = y;
    track_xyz[t * 3 + 2] = z;
    track_flags[static_cast<size_t>(t)] =
        store.is_track_valid(static_cast<int>(t)) ? track_flags::kAlive : 0;
  }

  std::vector<uint32_t> track_obs_offset(static_cast<size_t>(n_tracks) + 1);
  std::vector<uint32_t> obs_image_slot, obs_feature_id;
  std::vector<float> obs_u, obs_v, obs_scale;
  std::vector<uint8_t> obs_flags;
  obs_image_slot.reserve(store.num_observations());
  obs_feature_id.reserve(store.num_observations());
  obs_u.reserve(store.num_observations());
  obs_v.reserve(store.num_observations());
  obs_scale.reserve(store.num_observations());
  obs_flags.reserve(store.num_observations());

  std::vector<Observation> obs_buf;
  size_t offset = 0;
  for (size_t t = 0; t < n_tracks; ++t) {
    track_obs_offset[t] = static_cast<uint32_t>(offset);
    obs_buf.clear();
    store.get_track_observations(static_cast<int>(t), &obs_buf);
    for (const auto& o : obs_buf) {
      obs_image_slot.push_back(o.image_index); // store slot 0..n-1
      obs_feature_id.push_back(o.feature_id);
      obs_u.push_back(o.u);
      obs_v.push_back(o.v);
      obs_scale.push_back(o.scale);
      obs_flags.push_back(obs_flags::kAlive);
    }
    offset += obs_buf.size();
  }
  track_obs_offset[n_tracks] = static_cast<uint32_t>(offset);
  const size_t n_obs = obs_image_slot.size();
  meta["num_observations"] = static_cast<int>(n_obs);

  IDCWriter writer(path);
  writer.set_metadata(meta);
  writer.add_blob("track_xyz", track_xyz.data(), track_xyz.size() * sizeof(float), "float32",
                 {static_cast<int>(n_tracks), 3});
  writer.add_blob("track_flags", track_flags.data(), track_flags.size(), "uint8",
                 {static_cast<int>(n_tracks)});
  writer.add_blob("track_obs_offset", track_obs_offset.data(),
                 track_obs_offset.size() * sizeof(uint32_t), "uint32",
                 {static_cast<int>(n_tracks) + 1});
  writer.add_blob("obs_image_index", obs_image_slot.data(), obs_image_slot.size() * sizeof(uint32_t),
                 "uint32", {static_cast<int>(obs_image_slot.size())});
  writer.add_blob("obs_feature_id", obs_feature_id.data(), obs_feature_id.size() * sizeof(uint32_t),
                 "uint32", {static_cast<int>(obs_feature_id.size())});
  writer.add_blob("obs_u", obs_u.data(), obs_u.size() * sizeof(float), "float32",
                 {static_cast<int>(obs_u.size())});
  writer.add_blob("obs_v", obs_v.data(), obs_v.size() * sizeof(float), "float32",
                 {static_cast<int>(obs_v.size())});
  writer.add_blob("obs_scale", obs_scale.data(), obs_scale.size() * sizeof(float), "float32",
                 {static_cast<int>(obs_scale.size())});
  writer.add_blob("obs_flags", obs_flags.data(), obs_flags.size(), "uint8",
                 {static_cast<int>(obs_flags.size())});
  if (!writer.write()) {
    LOG(ERROR) << "Failed to write " << path;
    return false;
  }
  VLOG(1) << "Wrote " << path << " (" << n_tracks << " tracks, " << n_obs << " observations)";
  return true;
}

static bool load_track_store_from_idc(const std::string& path, TrackStore* store,
                                      std::vector<uint32_t>* image_indices_out) {
  IDCReader reader(path);
  if (!reader.is_valid()) {
    LOG(ERROR) << "Invalid IDC: " << path;
    return false;
  }
  const json& meta = reader.get_metadata();
  const int num_images = meta["num_images"].get<int>();
  const int num_tracks = meta["num_tracks"].get<int>();
  const int num_observations = meta["num_observations"].get<int>();
  std::vector<uint32_t> image_indices;
  const auto& ids_arr = meta.contains("image_indices") ? meta["image_indices"] : meta["image_ids"];
  for (const auto& v : ids_arr) {
    if (v.is_number_unsigned())
      image_indices.push_back(v.get<uint32_t>());
    else if (v.is_string())
      image_indices.push_back(static_cast<uint32_t>(std::stoul(v.get<std::string>())));
    else
      image_indices.push_back(static_cast<uint32_t>(v.get<int64_t>()));
  }

  auto track_xyz = reader.read_blob<float>("track_xyz");
  auto track_flags = reader.read_blob<uint8_t>("track_flags");
  auto track_obs_offset = reader.read_blob<uint32_t>("track_obs_offset");
  auto obs_image_slot = reader.read_blob<uint32_t>("obs_image_index");
  if (obs_image_slot.empty())
    obs_image_slot = reader.read_blob<uint32_t>("obs_image_id");
  auto obs_feature_id = reader.read_blob<uint32_t>("obs_feature_id");
  auto obs_u = reader.read_blob<float>("obs_u");
  auto obs_v = reader.read_blob<float>("obs_v");
  auto obs_scale = reader.read_blob<float>("obs_scale");
  auto obs_flags = reader.read_blob<uint8_t>("obs_flags");

  if (track_xyz.size() != static_cast<size_t>(num_tracks) * 3u ||
      track_flags.size() != static_cast<size_t>(num_tracks) ||
      track_obs_offset.size() != static_cast<size_t>(num_tracks) + 1u ||
      obs_image_slot.size() != static_cast<size_t>(num_observations)) {
    LOG(ERROR) << "IDC blob size mismatch";
    return false;
  }

  store->set_num_images(num_images);
  for (int t = 0; t < num_tracks; ++t) {
    float x = track_xyz[static_cast<size_t>(t) * 3];
    float y = track_xyz[static_cast<size_t>(t) * 3 + 1];
    float z = track_xyz[static_cast<size_t>(t) * 3 + 2];
    store->add_track(x, y, z);
    if ((track_flags[static_cast<size_t>(t)] & track_flags::kAlive) == 0)
      store->mark_track_deleted(t);
  }
  for (int t = 0; t < num_tracks; ++t) {
    const size_t beg = track_obs_offset[static_cast<size_t>(t)];
    const size_t end = track_obs_offset[static_cast<size_t>(t) + 1];
    for (size_t g = beg; g < end; ++g) {
      float s = (g < obs_scale.size()) ? obs_scale[g] : 1.f;
      store->add_observation(t, obs_image_slot[g], obs_feature_id[g], obs_u[g], obs_v[g], s);
    }
  }
  for (int g = 0; g < num_observations; ++g) {
    if (static_cast<size_t>(g) < obs_flags.size() &&
        (obs_flags[static_cast<size_t>(g)] & obs_flags::kAlive) == 0)
      store->mark_observation_deleted(g);
  }
  if (image_indices_out)
    *image_indices_out = image_indices;
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Phase 1: Union-Find over geo inliers only (same inlier logic as Phase 2)
// ─────────────────────────────────────────────────────────────────────────────

static void phase1_union_find(UnionFind* uf, const std::vector<PairDesc>& pairs) {
  for (const auto& p : pairs) {
    IDCReader geo_rd(p.geo_file);
    IDCReader match_rd(p.match_file);
    if (!geo_rd.is_valid() || !match_rd.is_valid())
      continue;
    auto inlier_mask = geo_rd.read_blob<uint8_t>("F_inliers");
    if (inlier_mask.empty())
      inlier_mask = geo_rd.read_blob<uint8_t>("E_inliers");
    if (inlier_mask.empty())
      continue;
    auto indices = match_rd.read_blob<uint16_t>("indices");
    if (indices.size() < 2)
      continue;
    const int num_matches = static_cast<int>(inlier_mask.size());
    for (int m = 0; m < num_matches; ++m) {
      if (!inlier_mask[static_cast<size_t>(m)])
        continue;
      const size_t mi = static_cast<size_t>(m);
      uint16_t idx1 = indices[mi * 2];
      uint16_t idx2 = indices[mi * 2 + 1];
      uf->merge_keys(node_key(p.image1_index, idx1), node_key(p.image2_index, idx2));
    }
  }
}

// Encode (track_id, image_index, feature_id) for dedup. Low 16 bits = feature_id (< 65536);
// high bits = track_id * n_images + image_index (matrix-style, unique and dense).
static inline uint64_t obs_key(int n_images, int track_id, uint32_t image_index, uint16_t feature_id) {
  return (static_cast<uint64_t>(track_id) * n_images + image_index) << 16 |
         static_cast<uint64_t>(feature_id);
}

// ─────────────────────────────────────────────────────────────────────────────
// Phase 2: Fill observations from match + geo inlier masks (no 3D from two-view)
// ─────────────────────────────────────────────────────────────────────────────
// Two-view points3d are in per-pair local frames; track xyz is left for incremental SfM.
static void phase2_fill_observations(TrackStore* store, const std::vector<PairDesc>& pairs,
                                   int n_images,
                                   const std::unordered_map<int, int>& root_to_track_id,
                                   UnionFind& uf) {
  std::unordered_set<uint64_t> added_obs;
  added_obs.reserve(static_cast<size_t>(store->num_tracks()) * 4u);

  const uint32_t n_ui = static_cast<uint32_t>(n_images);
  for (const auto& p : pairs) {
    if (p.image1_index >= n_ui || p.image2_index >= n_ui) {
      LOG(FATAL) << "Image index out of bounds: " << p.image1_index << " or " << p.image2_index
                 << " >= " << n_ui;
    }
    IDCReader geo_rd(p.geo_file);
    IDCReader match_rd(p.match_file);
    if (!geo_rd.is_valid() || !match_rd.is_valid())
      continue;
    auto inlier_mask = geo_rd.read_blob<uint8_t>("F_inliers");
    if (inlier_mask.empty())
      inlier_mask = geo_rd.read_blob<uint8_t>("E_inliers");
    if (inlier_mask.empty())
      continue;
    auto indices = match_rd.read_blob<uint16_t>("indices");
    auto coords = match_rd.read_blob<float>("coords_pixel");
    auto scales = match_rd.read_blob<float>("scales");
    if (indices.size() < 2 || coords.size() < 4)
      continue;
    const int num_matches = static_cast<int>(inlier_mask.size());
    const bool have_scales = (scales.size() >= static_cast<size_t>(num_matches) * 2u);
    const uint32_t slot1 = p.image1_index;
    const uint32_t slot2 = p.image2_index;

    for (int m = 0; m < num_matches; ++m) {
      if (!inlier_mask[static_cast<size_t>(m)])
        continue;
      const size_t mi = static_cast<size_t>(m);
      uint16_t idx1 = indices[mi * 2];
      uint16_t idx2 = indices[mi * 2 + 1];
      uint64_t k1 = node_key(p.image1_index, idx1);
      auto rit = root_to_track_id.find(uf.find_key(k1));
      if (rit == root_to_track_id.end())
        continue;
      int track_id = rit->second;
      float x1 = coords[mi * 4];
      float y1 = coords[mi * 4 + 1];
      float x2 = coords[mi * 4 + 2];
      float y2 = coords[mi * 4 + 3];
      float s1 = 1.f, s2 = 1.f;
      if (have_scales) {
        s1 = scales[mi * 2];
        s2 = scales[mi * 2 + 1];
      }
      const uint64_t key1 = obs_key(n_images, track_id, slot1, idx1);
      if (added_obs.insert(key1).second)
        store->add_observation(track_id, slot1, idx1, x1, y1, s1);
      const uint64_t key2 = obs_key(n_images, track_id, slot2, idx2);
      if (added_obs.insert(key2).second)
        store->add_observation(track_id, slot2, idx2, x2, y2, s2);
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;

  CmdLine cmd("InsightAT Track Building – match + geo → tracks IDC");
  std::string pairs_json, match_dir, geo_dir, image_list, output_path;
  bool stats_only = false;

  cmd.add(make_option('i', pairs_json, "input").doc("Pairs JSON (e.g. from isat_geo output)"));
  cmd.add(make_option('m', match_dir, "match-dir").doc("Directory of .isat_match files"));
  cmd.add(make_option('g', geo_dir, "geo-dir").doc("Directory of .isat_geo files"));
  cmd.add(make_option('l', image_list, "image-list")
              .doc("Image list JSON from isat_project extract (required). Defines image_index baseline for SfM."));
  cmd.add(make_option('o', output_path, "output").doc("Output .isat_tracks IDC path"));
  cmd.add(make_switch(0, "stats").doc("Only load existing IDC and print stats to stderr"));
  std::string log_level;
  cmd.add(make_option(0, log_level, "log-level").doc("Log level: error|warn|info|debug"));
  cmd.add(make_switch('v', "verbose").doc("Verbose (INFO)"));
  cmd.add(make_switch('q', "quiet").doc("Quiet (ERROR only)"));
  cmd.add(make_switch('h', "help").doc("Show help"));

  try {
    cmd.process(argc, argv);
  } catch (const std::string& s) {
    std::cerr << "Error: " << s << "\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 1;
  }
  if (cmd.checkHelp(argv[0]))
    return 0;
  insight::tools::apply_log_level(cmd.used('v'), cmd.used('q'), log_level);
  stats_only = cmd.used("stats");

  if (output_path.empty()) {
    std::cerr << "Error: -o/--output is required\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 1;
  }

  if (stats_only) {
    TrackStore store;
    std::vector<uint32_t> image_indices;
    if (!load_track_store_from_idc(output_path, &store, &image_indices))
      return 1;
    LOG(INFO) << "Tracks: " << store.num_tracks() << "  Observations: " << store.num_observations()
              << "  Images: " << image_indices.size();
    print_event({{"type", "tracks.stats"},
                {"ok", true},
                {"data",
                 {{"num_tracks", static_cast<int>(store.num_tracks())},
                  {"num_observations", static_cast<int>(store.num_observations())},
                  {"num_images", static_cast<int>(image_indices.size())}}}});
    return 0;
  }

  if (pairs_json.empty() || match_dir.empty() || geo_dir.empty()) {
    std::cerr << "Error: -i, -m, -g are required for building tracks\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 1;
  }
  if (image_list.empty()) {
    std::cerr << "Error: -l/--image-list is required (export JSON with image_index) so tracks match SfM baseline\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 1;
  }

  if (!fs::is_directory(match_dir)) {
    LOG(ERROR) << "Match directory not found: " << match_dir;
    return 1;
  }
  if (!fs::is_directory(geo_dir)) {
    LOG(ERROR) << "Geo directory not found: " << geo_dir;
    return 1;
  }

  std::vector<PairDesc> pairs = load_pairs(pairs_json, match_dir, geo_dir);
  if (pairs.empty()) {
    LOG(ERROR) << "No pairs to process";
    return 1;
  }

  std::vector<uint32_t> image_indices = get_image_indices_from_list(image_list);
  if (image_indices.empty()) {
    LOG(ERROR) << "No images from list";
    return 1;
  }
  const int n_images = static_cast<int>(image_indices.size());

  UnionFind uf;
  phase1_union_find(&uf, pairs);

  std::unordered_map<int, int> root_to_track_id;
  int next_track = 0;
  for (const auto& kv : uf.node_id_) {
    int root = uf.find_by_id(kv.second);
    if (root_to_track_id.find(root) == root_to_track_id.end())
      root_to_track_id[root] = next_track++;
  }
  const int num_tracks = next_track;
  LOG(INFO) << "Phase 1: " << num_tracks << " tracks from Union-Find";

  TrackStore store;
  store.set_num_images(n_images);
  store.reserve_tracks(static_cast<size_t>(num_tracks));
  store.reserve_observations(static_cast<size_t>(num_tracks) * 4u);
  for (int t = 0; t < num_tracks; ++t)
    store.add_track(0.f, 0.f, 0.f);

  phase2_fill_observations(&store, pairs, n_images, root_to_track_id, uf);
  LOG(INFO) << "Phase 2: " << store.num_observations() << " observations (track xyz from SfM later)";

  if (!save_track_store_to_idc(store, image_indices, output_path))
    return 1;
  print_event({{"type", "tracks.build"},
              {"ok", true},
              {"data",
               {{"output", output_path},
                {"num_tracks", static_cast<int>(store.num_tracks())},
                {"num_observations", static_cast<int>(store.num_observations())}}}});
  return 0;
}
