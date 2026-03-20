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
#include "../io/track_store_idc.h"
#include "../modules/sfm/track_store.h"
#include "cli_logging.h"
#include "cmdLine/cmdLine.h"

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
    // Match and geo files are always stored as min_max (image1_index < image2_index).
    // Canonicalise so that image1_index <= image2_index to guarantee correct path lookup
    // and correct feature-index interpretation (indices[m*2] belongs to image1).
    if (d.image1_index > d.image2_index)
      std::swap(d.image1_index, d.image2_index);
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

static uint32_t image_index_from_node_key(uint64_t key) {
  return static_cast<uint32_t>(key >> 32);
}

struct UnionFind {
  std::unordered_map<uint64_t, int> node_id_;  // key (image_index<<32|feat_id) -> internal id
  std::vector<int> parent_;                     // internal id -> parent (internal id)
  std::vector<int> component_size_;
  std::vector<std::vector<uint32_t>> component_images_;

  static bool component_has_image(const std::vector<uint32_t>& images, uint32_t image_index) {
    return std::find(images.begin(), images.end(), image_index) != images.end();
  }

  static bool components_overlap_images(const std::vector<uint32_t>& a,
                                        const std::vector<uint32_t>& b) {
    const std::vector<uint32_t>& small = (a.size() <= b.size()) ? a : b;
    const std::vector<uint32_t>& large = (a.size() <= b.size()) ? b : a;
    for (uint32_t image_index : small) {
      if (component_has_image(large, image_index))
        return true;
    }
    return false;
  }

  int get_or_create(uint64_t key) {
    auto it = node_id_.find(key);
    if (it != node_id_.end())
      return it->second;
    const int id = static_cast<int>(parent_.size());
    node_id_[key] = id;
    parent_.push_back(id);
    component_size_.push_back(1);
    component_images_.push_back({image_index_from_node_key(key)});
    return id;
  }

  int find_key(uint64_t key) {
    const int id = get_or_create(key);
    return find_by_id(id);
  }

  bool merge_keys(uint64_t k1, uint64_t k2) {
    int a = find_key(k1);
    int b = find_key(k2);
    if (a == b)
      return true;

    if (components_overlap_images(component_images_[static_cast<size_t>(a)],
                                  component_images_[static_cast<size_t>(b)])) {
      return false;
    }

    if (component_size_[static_cast<size_t>(a)] < component_size_[static_cast<size_t>(b)])
      std::swap(a, b);

    parent_[static_cast<size_t>(b)] = a;
    component_size_[static_cast<size_t>(a)] += component_size_[static_cast<size_t>(b)];
    std::vector<uint32_t>& dst = component_images_[static_cast<size_t>(a)];
    const std::vector<uint32_t>& src = component_images_[static_cast<size_t>(b)];
    dst.insert(dst.end(), src.begin(), src.end());
    component_images_[static_cast<size_t>(b)].clear();
    return true;
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

// ─────────────────────────────────────────────────────────────────────────────
// Phase 1: Union-Find over geo inliers only (same inlier logic as Phase 2)
// ─────────────────────────────────────────────────────────────────────────────

static void phase1_union_find(UnionFind* uf, const std::vector<PairDesc>& pairs) {
  int merged_edges = 0;
  int rejected_same_image_component = 0;
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
      if (uf->merge_keys(node_key(p.image1_index, idx1), node_key(p.image2_index, idx2))) {
        ++merged_edges;
      } else {
        ++rejected_same_image_component;
      }
    }
  }
  LOG(INFO) << "Phase 1 merge stats: merged_edges=" << merged_edges
            << " rejected_same_image_component=" << rejected_same_image_component;
}

// Encode (track_id, image_index, feature_id) for dedup. Low 16 bits = feature_id (< 65536);
// high bits = track_id * n_images + image_index (matrix-style, unique and dense).
static inline uint64_t obs_key(int n_images, int track_id, uint32_t image_index, uint16_t feature_id) {
  return (static_cast<uint64_t>(track_id) * n_images + image_index) << 16 |
         static_cast<uint64_t>(feature_id);
}

static inline uint64_t track_image_key(int track_id, uint32_t image_index) {
  return (static_cast<uint64_t>(static_cast<uint32_t>(track_id)) << 32) |
         static_cast<uint64_t>(image_index);
}

struct ObservationCandidate {
  int track_id = -1;
  uint32_t image_index = 0;
  uint32_t feature_id = 0;
  float u = 0.f;
  float v = 0.f;
  float scale = 1.f;
};

// ─────────────────────────────────────────────────────────────────────────────
// Phase 2: Fill observations from match + geo inlier masks (no 3D from two-view)
// ─────────────────────────────────────────────────────────────────────────────
// Two-view points3d are in per-pair local frames; track xyz is left for incremental SfM.
static void phase2_fill_observations(TrackStore* store, const std::vector<PairDesc>& pairs,
                                   int n_images,
                                   const std::unordered_map<int, int>& root_to_track_id,
                                   UnionFind& uf) {
  std::unordered_set<uint64_t> exact_candidate_obs;
  exact_candidate_obs.reserve(static_cast<size_t>(store->num_tracks()) * 4u);
  std::unordered_set<uint64_t> added_obs;
  added_obs.reserve(static_cast<size_t>(store->num_tracks()) * 4u);
  std::unordered_map<uint64_t, ObservationCandidate> obs_per_track_image;
  obs_per_track_image.reserve(static_cast<size_t>(store->num_tracks()) * 4u);
  std::unordered_set<int> conflicted_tracks;
  conflicted_tracks.reserve(static_cast<size_t>(store->num_tracks()) / 8u + 1u);

  int duplicate_feature_obs = 0;
  int conflicting_same_image_obs = 0;
  int rejected_conflict_tracks = 0;

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
      uint64_t k2 = node_key(p.image2_index, idx2);
      // Look up each side independently: after Phase-1 same-image guard, k1 and k2
      // may belong to different components. Each observation must go to its own track
      // to avoid injecting wrong coordinates into the other side's track.
      auto rit1 = root_to_track_id.find(uf.find_key(k1));
      auto rit2 = root_to_track_id.find(uf.find_key(k2));
      if (rit1 == root_to_track_id.end() && rit2 == root_to_track_id.end())
        continue;
      float x1 = coords[mi * 4];
      float y1 = coords[mi * 4 + 1];
      float x2 = coords[mi * 4 + 2];
      float y2 = coords[mi * 4 + 3];
      float s1 = 1.f, s2 = 1.f;
      if (have_scales) {
        s1 = scales[mi * 2];
        s2 = scales[mi * 2 + 1];
      }
      ObservationCandidate c1, c2;
      const bool have_c1 = (rit1 != root_to_track_id.end());
      const bool have_c2 = (rit2 != root_to_track_id.end());
      if (have_c1) {
        c1.track_id    = rit1->second;
        c1.image_index = slot1;
        c1.feature_id  = idx1;
        c1.u = x1; c1.v = y1; c1.scale = s1;
      }
      if (have_c2) {
        c2.track_id    = rit2->second;
        c2.image_index = slot2;
        c2.feature_id  = idx2;
        c2.u = x2; c2.v = y2; c2.scale = s2;
      }

      for (int _side = 0; _side < 2; ++_side) {
        if (_side == 0 && !have_c1) continue;
        if (_side == 1 && !have_c2) continue;
        const ObservationCandidate& cand = (_side == 0) ? c1 : c2;
        const uint64_t exact_key = obs_key(n_images, cand.track_id, cand.image_index,
                                           static_cast<uint16_t>(cand.feature_id));
        if (!exact_candidate_obs.insert(exact_key).second) {
          ++duplicate_feature_obs;
          continue;
        }

        const uint64_t ti_key = track_image_key(cand.track_id, cand.image_index);
        auto [it, inserted] = obs_per_track_image.emplace(ti_key, cand);
        if (inserted)
          continue;

        if (it->second.feature_id == cand.feature_id) {
          ++duplicate_feature_obs;
          continue;
        }

        ++conflicting_same_image_obs;
        if (conflicted_tracks.insert(cand.track_id).second) {
          ++rejected_conflict_tracks;
        }
      }
    }
  }

  std::vector<ObservationCandidate> final_obs;
  final_obs.reserve(obs_per_track_image.size());
  for (const auto& kv : obs_per_track_image) {
    if (conflicted_tracks.find(kv.second.track_id) != conflicted_tracks.end())
      continue;
    final_obs.push_back(kv.second);
  }
  std::sort(final_obs.begin(), final_obs.end(), [](const ObservationCandidate& a,
                                                   const ObservationCandidate& b) {
    if (a.track_id != b.track_id)
      return a.track_id < b.track_id;
    if (a.image_index != b.image_index)
      return a.image_index < b.image_index;
    return a.feature_id < b.feature_id;
  });

  for (const ObservationCandidate& cand : final_obs) {
    const uint64_t exact_key = obs_key(n_images, cand.track_id, cand.image_index,
                                       static_cast<uint16_t>(cand.feature_id));
    if (!added_obs.insert(exact_key).second)
      continue;
    store->add_observation(cand.track_id, cand.image_index, cand.feature_id, cand.u, cand.v,
                           cand.scale);
  }

  LOG(INFO) << "Phase 2 uniqueness: kept=" << final_obs.size()
            << " duplicate_feature_obs=" << duplicate_feature_obs
            << " conflicting_same_image_obs=" << conflicting_same_image_obs
            << " rejected_conflict_tracks=" << rejected_conflict_tracks
            << " conflicted_tracks=" << conflicted_tracks.size();
}

// ─────────────────────────────────────────────────────────────────────────────
// Post-build filter: remove tracks with fewer than min_degree observations.
// Builds a compact new TrackStore (renumbered) and returns stats.
// ─────────────────────────────────────────────────────────────────────────────

struct FilterStats {
  int in_tracks = 0;
  int in_obs = 0;
  int removed_tracks = 0;
  int removed_obs = 0;
  int out_tracks = 0;
  int out_obs = 0;
};

static FilterStats compact_tracks_min_degree(const TrackStore& src, int n_images, int min_degree,
                                             TrackStore* dst) {
  FilterStats stats;
  stats.in_tracks = static_cast<int>(src.num_tracks());
  stats.in_obs    = static_cast<int>(src.num_observations());

  dst->set_num_images(n_images);
  dst->reserve_tracks(static_cast<size_t>(stats.in_tracks));
  dst->reserve_observations(static_cast<size_t>(stats.in_obs));

  std::vector<Observation> obs_buf;
  for (int t = 0; t < stats.in_tracks; ++t) {
    if (!src.is_track_valid(t)) {
      ++stats.removed_tracks;
      continue;
    }
    obs_buf.clear();
    const int deg = src.get_track_observations(t, &obs_buf);
    if (deg < min_degree) {
      ++stats.removed_tracks;
      stats.removed_obs += deg;
      continue;
    }
    // Keep this track
    float x = 0.f, y = 0.f, z = 0.f;
    src.get_track_xyz(t, &x, &y, &z);
    const int new_tid = dst->add_track(x, y, z);
    for (const Observation& o : obs_buf)
      dst->add_observation(new_tid, o.image_index, o.feature_id, o.u, o.v, o.scale);
  }

  stats.out_tracks = static_cast<int>(dst->num_tracks());
  stats.out_obs    = static_cast<int>(dst->num_observations());
  return stats;
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
  int min_track_length = 1;
  cmd.add(make_option(0, min_track_length, "min-track-length")
              .doc("Remove tracks with fewer than N observations (degree filter). Default=1 (keep all). "
                   "Use 3 to discard degree-1 and degree-2 tracks."));
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

  // ── Optional degree filter ─────────────────────────────────────────────────
  const TrackStore* store_to_save = &store;
  TrackStore filtered_store;
  if (min_track_length > 1) {
    const FilterStats fstats = compact_tracks_min_degree(store, n_images, min_track_length,
                                                         &filtered_store);
    LOG(INFO) << "Degree filter (min=" << min_track_length << "):"
              << "  removed_tracks=" << fstats.removed_tracks
              << "  removed_obs=" << fstats.removed_obs
              << "  kept_tracks=" << fstats.out_tracks
              << "  kept_obs=" << fstats.out_obs;
    store_to_save = &filtered_store;
  }

  if (!save_track_store_to_idc(*store_to_save, image_indices, output_path))
    return 1;
  print_event({{"type", "tracks.build"},
              {"ok", true},
              {"data",
               {{"output", output_path},
                {"min_track_length", min_track_length},
                {"num_tracks", static_cast<int>(store_to_save->num_tracks())},
                {"num_observations", static_cast<int>(store_to_save->num_observations())}}}});
  return 0;
}
