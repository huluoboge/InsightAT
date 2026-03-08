/**
 * @file  full_track_builder.cpp
 * @brief Build full TrackStore from pairs via union-find (Phase 1 + Phase 2).
 */

#include "full_track_builder.h"
#include "../../io/idc_reader.h"
#include "../../tools/pair_json_utils.h"
#include <algorithm>
#include <fstream>
#include <nlohmann/json.hpp>
#include <set>
#include <unordered_map>
#include <vector>

#include <glog/logging.h>

namespace insight {
namespace sfm {

namespace {

struct PairDesc {
  uint32_t image1_index = 0;
  uint32_t image2_index = 0;
  std::string match_file;
  std::string geo_file;
};

uint64_t node_key(uint32_t image_id, uint32_t feature_id) {
  return (static_cast<uint64_t>(image_id) << 32) | feature_id;
}

struct UnionFind {
  std::unordered_map<uint64_t, int> node_id_;
  std::vector<int> parent_;

  int get_or_create(uint64_t key) {
    auto it = node_id_.find(key);
    if (it != node_id_.end())
      return it->second;
    int id = static_cast<int>(parent_.size());
    node_id_[key] = id;
    parent_.push_back(id);
    return id;
  }

  int find(int i) {
    if (parent_[static_cast<size_t>(i)] == i)
      return i;
    return parent_[static_cast<size_t>(i)] = find(parent_[static_cast<size_t>(i)]);
  }

  int find_key(uint64_t key) { return find(get_or_create(key)); }

  void merge(int a, int b) {
    a = find(a);
    b = find(b);
    if (a != b)
      parent_[static_cast<size_t>(a)] = b;
  }

  void merge_keys(uint64_t k1, uint64_t k2) { merge(get_or_create(k1), get_or_create(k2)); }
};

bool load_pairs(const std::string& pairs_json_path, const std::string& geo_dir,
                const std::string& match_dir, std::vector<PairDesc>* out) {
  std::ifstream file(pairs_json_path);
  if (!file.is_open()) {
    LOG(WARNING) << "full_track_builder: cannot open pairs file: " << pairs_json_path;
    return false;
  }
  nlohmann::json j;
  try {
    file >> j;
  } catch (...) {
    LOG(WARNING) << "full_track_builder: failed to parse JSON: " << pairs_json_path;
    return false;
  }
  if (!j.contains("pairs") || !j["pairs"].is_array()) {
    LOG(WARNING) << "full_track_builder: pairs JSON has no 'pairs' array";
    return false;
  }
  std::string dir_geo = geo_dir;
  if (!dir_geo.empty() && dir_geo.back() != '/')
    dir_geo += '/';
  std::string dir_match = match_dir;
  if (!dir_match.empty() && dir_match.back() != '/')
    dir_match += '/';
  out->clear();
  for (const auto& p : j["pairs"]) {
    PairDesc d;
    d.image1_index = insight::tools::get_image_index_from_pair(p, "image1_index");
    d.image2_index = insight::tools::get_image_index_from_pair(p, "image2_index");
    d.match_file = dir_match + std::to_string(d.image1_index) + "_" + std::to_string(d.image2_index) + ".isat_match";
    d.geo_file = dir_geo + std::to_string(d.image1_index) + "_" + std::to_string(d.image2_index) + ".isat_geo";
    out->push_back(std::move(d));
  }
  return true;
}

std::unordered_map<uint32_t, int> build_image_id_to_index(const IdMapping* id_mapping,
                                                           const std::vector<PairDesc>& pairs) {
  std::unordered_map<uint32_t, int> m;
  if (id_mapping) {
    for (size_t i = 0; i < id_mapping->num_images(); ++i) {
      uint32_t orig = id_mapping->original_image_id(static_cast<int>(i));
      m[orig] = static_cast<int>(i);
    }
    return m;
  }
  std::set<uint32_t> seen;
  for (const auto& p : pairs) {
    seen.insert(p.image1_index);
    seen.insert(p.image2_index);
  }
  std::vector<uint32_t> ordered(seen.begin(), seen.end());
  std::sort(ordered.begin(), ordered.end());
  for (size_t i = 0; i < ordered.size(); ++i)
    m[ordered[static_cast<size_t>(i)]] = static_cast<int>(i);
  return m;
}

void phase1_union_find(UnionFind* uf, const std::vector<PairDesc>& pairs) {
  for (const auto& p : pairs) {
    insight::io::IDCReader reader(p.match_file);
    if (!reader.is_valid())
      continue;
    auto indices = reader.read_blob<uint16_t>("indices");
    if (indices.size() < 2)
      continue;
    const int n = static_cast<int>(indices.size() / 2);
    for (int i = 0; i < n; ++i) {
      uint16_t idx1 = indices[static_cast<size_t>(i) * 2];
      uint16_t idx2 = indices[static_cast<size_t>(i) * 2 + 1];
      uf->merge_keys(node_key(p.image1_index, idx1), node_key(p.image2_index, idx2));
    }
  }
}

void phase2_fill_observations(
    TrackStore* store, const std::vector<PairDesc>& pairs,
    const std::unordered_map<uint32_t, int>& image_id_to_index,
    const std::unordered_map<int, int>& root_to_track_id, UnionFind& uf,
    std::set<int>* tracks_with_xyz) {
  std::set<std::tuple<int, int, uint32_t>> added_obs;
  for (const auto& p : pairs) {
    insight::io::IDCReader geo_rd(p.geo_file);
    insight::io::IDCReader match_rd(p.match_file);
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
    auto it1 = image_id_to_index.find(p.image1_index);
    auto it2 = image_id_to_index.find(p.image2_index);
    if (it1 == image_id_to_index.end() || it2 == image_id_to_index.end())
      continue;
    const int img1_idx = it1->second;
    const int img2_idx = it2->second;

    std::vector<float> points3d;
    bool have_points3d = false;
    if (geo_rd.get_metadata().contains("twoview")) {
      points3d = geo_rd.read_blob<float>("points3d");
      have_points3d = (geo_rd.get_metadata()["twoview"].contains("num_valid_points") &&
                       !points3d.empty());
    }
    int inlier_count = 0;
    for (int m = 0; m < num_matches; ++m) {
      if (!inlier_mask[static_cast<size_t>(m)])
        continue;
      const size_t mi = static_cast<size_t>(m);
      uint16_t idx1 = indices[mi * 2];
      uint16_t idx2 = indices[mi * 2 + 1];
      uint64_t k1 = node_key(p.image1_index, idx1);
      int root = uf.find_key(k1);
      auto rit = root_to_track_id.find(root);
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
      auto key1 = std::make_tuple(track_id, img1_idx, static_cast<uint32_t>(idx1));
      if (added_obs.find(key1) == added_obs.end()) {
        store->add_observation(track_id, static_cast<uint32_t>(img1_idx), idx1, x1, y1, s1);
        added_obs.insert(key1);
      }
      auto key2 = std::make_tuple(track_id, img2_idx, static_cast<uint32_t>(idx2));
      if (added_obs.find(key2) == added_obs.end()) {
        store->add_observation(track_id, static_cast<uint32_t>(img2_idx), idx2, x2, y2, s2);
        added_obs.insert(key2);
      }
      if (have_points3d && tracks_with_xyz->count(track_id) == 0) {
        const size_t base = static_cast<size_t>(inlier_count) * 3u;
        if (base + 2 < points3d.size()) {
          store->set_track_xyz(track_id, points3d[base], points3d[base + 1], points3d[base + 2]);
          tracks_with_xyz->insert(track_id);
        }
      }
      ++inlier_count;
    }
  }
}

} // namespace

bool build_full_track_store_from_pairs(const std::string& pairs_json_path,
                                       const std::string& geo_dir,
                                       const std::string& match_dir,
                                       const IdMapping* id_mapping,
                                       TrackStore* store_out) {
  if (!store_out)
    return false;
  std::vector<PairDesc> pairs;
  if (!load_pairs(pairs_json_path, geo_dir, match_dir, &pairs) || pairs.empty()) {
    LOG(WARNING) << "full_track_builder: no pairs loaded";
    return false;
  }

  std::unordered_map<uint32_t, int> image_id_to_index =
      build_image_id_to_index(id_mapping, pairs);
  int n_images = 0;
  if (id_mapping) {
    n_images = static_cast<int>(id_mapping->num_images());
    if (n_images <= 0) {
      LOG(WARNING) << "full_track_builder: id_mapping has no images";
      return false;
    }
  } else {
    for (const auto& kv : image_id_to_index)
      if (kv.second >= n_images)
        n_images = kv.second + 1;
  }

  UnionFind uf;
  phase1_union_find(&uf, pairs);

  std::unordered_map<int, int> root_to_track_id;
  int next_track = 0;
  for (const auto& kv : uf.node_id_) {
    int root = uf.find(kv.second);
    if (root_to_track_id.find(root) == root_to_track_id.end())
      root_to_track_id[root] = next_track++;
  }
  const int num_tracks = next_track;
  LOG(INFO) << "full_track_builder: Phase 1 done, " << num_tracks << " tracks, " << n_images
            << " images";

  store_out->set_num_images(n_images);
  store_out->reserve_tracks(static_cast<size_t>(num_tracks));
  store_out->reserve_observations(static_cast<size_t>(num_tracks) * 4u);
  for (int t = 0; t < num_tracks; ++t)
    store_out->add_track(0.f, 0.f, 0.f);

  std::set<int> tracks_with_xyz;
  phase2_fill_observations(store_out, pairs, image_id_to_index, root_to_track_id, uf,
                           &tracks_with_xyz);
  LOG(INFO) << "full_track_builder: Phase 2 done, " << store_out->num_observations()
            << " observations, " << tracks_with_xyz.size() << " tracks with xyz";

  return store_out->num_tracks() > 0 && store_out->num_observations() > 0;
}

} // namespace sfm
} // namespace insight
