/**
 * isat_tracks.cpp
 * InsightAT Track Building CLI – load match + geo, build tracks, write IDC.
 *
 * Pipeline:
 *   Phase 0+1  Block-interleaved parallel I/O + Union-Find: one geopack block fread at a time
 *              (~1.5 GB peak vs 8.3 GB before). Coords stored per-node at first creation.
 *   Phase 2    Observations from UF node iteration — O(N_unique_features), not O(N_total_inliers).
 *              (~5.7 s vs 104 s before, 18×). UF freed immediately after this phase.
 * Track xyz is left for incremental SfM (no two-view 3D). Output: single .isat_tracks IDC
 * (schema 1.1 embeds view_graph_pairs: PairGeoInfo per covisible edge after degree filter, from geo_dir).
 *
 * Usage:
 *   isat_tracks -i pairs.json -m match_dir/ -g geo_dir/ -l image_list.json -o tracks.isat_tracks
 *   isat_tracks --stats -o tracks.isat_tracks
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <omp.h>

#include <glog/logging.h>
#include <nlohmann/json.hpp>

#include "pair_json_utils.h"

#include "../io/idc_reader.h"
#include "../io/geopack_index.h"
#include "../io/track_store_idc.h"
#include "../modules/sfm/track_store.h"
#include "../modules/sfm/view_graph.h"
#include "../modules/sfm/view_graph_loader.h"
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
  bool use_geopack = false;
  std::string geopack_file;
  std::string geopack_f_blob;
  std::string geopack_e_blob;
};

static std::vector<PairDesc> load_pairs(const std::string& json_path, const std::string& match_dir,
                                       const std::string& geo_dir,
                                       const insight::io::GeoPackIndex* geopack_index) {
  std::ifstream file(json_path);
  if (!file.is_open()) {
    LOG(FATAL) << "Cannot open pairs file: " << json_path;
  }
  json j;
  file >> j;
  std::vector<PairDesc> pairs;
  pairs.reserve(j["pairs"].size());
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
    if (geopack_index) {
      const insight::io::GeoPackPairEntry* e =
          geopack_index->find(d.image1_index, d.image2_index);
      if (e) {
        d.use_geopack = true;
        d.geopack_file = e->pack_path;
        d.geopack_f_blob = e->f_inliers_blob;
        d.geopack_e_blob = e->e_inliers_blob;
      }
    }
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

// ─────────────────────────────────────────────────────────────────────────────
// Union-Find
//
// component_images_ stores the set of image indices per component as a
// sorted small vector.  Most tracks span 2–10 images, so linear scan on a
// contiguous array is faster than a hash table (better cache locality, no
// pointer indirection, branch-predictor-friendly).  Merge keeps the vector
// sorted via std::merge into a temporary, then swaps back.
// ─────────────────────────────────────────────────────────────────────────────

struct UnionFind {
  std::unordered_map<uint64_t, int> node_id_;
  std::vector<int> parent_;
  std::vector<int> component_size_;
  // Sorted small vector per component — cache-friendly for the typical 2–10 image case.
  std::vector<std::vector<uint32_t>> component_images_;
  // Per-node observation coords, parallel to parent_.  Populated at node creation (first-seen
  // wins).  Eliminates the need to keep all PairRawData alive through Phase 2.
  std::vector<float> node_u_, node_v_, node_s_;

  static bool components_overlap_images(const std::vector<uint32_t>& a,
                                        const std::vector<uint32_t>& b) {
    size_t i = 0, j = 0;
    while (i < a.size() && j < b.size()) {
      if (a[i] == b[j]) return true;
      if (a[i] < b[j]) ++i; else ++j;
    }
    return false;
  }

  // Create node with coords if new; return its internal id in both cases.
  int get_or_create(uint64_t key, float u, float v, float s) {
    auto it = node_id_.find(key);
    if (it != node_id_.end())
      return it->second;
    const int id = static_cast<int>(parent_.size());
    node_id_[key] = id;
    parent_.push_back(id);
    component_size_.push_back(1);
    component_images_.push_back({image_index_from_node_key(key)});
    node_u_.push_back(u);
    node_v_.push_back(v);
    node_s_.push_back(s);
    return id;
  }

  // Iterative path compression (avoids stack overflow on deep chains).
  int find_by_id(int i) {
    int root = i;
    while (parent_[static_cast<size_t>(root)] != root)
      root = parent_[static_cast<size_t>(root)];
    while (parent_[static_cast<size_t>(i)] != root) {
      int next = parent_[static_cast<size_t>(i)];
      parent_[static_cast<size_t>(i)] = root;
      i = next;
    }
    return root;
  }

  // Merge two features into the same track.  Creates nodes (storing coords) if they don't
  // exist yet.  Returns false only when the merge would put two features from the same image
  // into the same track (one-feature-per-image-per-track invariant).
  bool merge_keys(uint64_t k1, uint64_t k2,
                  float u1, float v1, float s1,
                  float u2, float v2, float s2) {
    int id1 = get_or_create(k1, u1, v1, s1);
    int id2 = get_or_create(k2, u2, v2, s2);
    int a = find_by_id(id1);
    int b = find_by_id(id2);
    if (a == b) return true;

    if (components_overlap_images(component_images_[static_cast<size_t>(a)],
                                  component_images_[static_cast<size_t>(b)]))
      return false;

    // Union by size: attach smaller to larger.
    if (component_size_[static_cast<size_t>(a)] < component_size_[static_cast<size_t>(b)])
      std::swap(a, b);

    parent_[static_cast<size_t>(b)] = a;
    component_size_[static_cast<size_t>(a)] += component_size_[static_cast<size_t>(b)];

    auto& dst = component_images_[static_cast<size_t>(a)];
    auto& src = component_images_[static_cast<size_t>(b)];
    std::vector<uint32_t> merged;
    merged.reserve(dst.size() + src.size());
    std::merge(dst.begin(), dst.end(), src.begin(), src.end(), std::back_inserter(merged));
    dst = std::move(merged);
    src.clear();
    src.shrink_to_fit();
    return true;
  }
};

// ─────────────────────────────────────────────────────────────────────────────
// Phase 0+1 combined: block-interleaved I/O + Union-Find
//
// Memory model (was 8.3 GB, now ≤ ~1.5 GB peak):
//   - One geopack block payload (~300 MB) live at a time.
//   - Per-block PairRawData (~1.2 GB) allocated, used for serial UF, then freed.
//   - Node coords (u,v,scale) stored once per-node in uf.node_[uvs]_ (240 MB).
//   - No global pair_raw vector: PairRawData never accumulates across blocks.
// ─────────────────────────────────────────────────────────────────────────────

struct InlierMatch {
  uint16_t idx1;
  uint16_t idx2;
  float x1, y1, x2, y2;  // pixel coordinates (image1, image2)
  float s1, s2;           // scales (1.0 if not available)
};

struct PairRawData {
  std::vector<InlierMatch> matches;
};

/**
 * Phase 0+1 pipeline: block-interleaved parallel I/O + serial Union-Find.
 * Each geopack block: fread payload → OMP fill PairRawData → serial UF → free.
 *
 * Peak extra memory: ~1.5 GB/block (vs 8.3 GB total before).
 */
// ─────────────────────────────────────────────────────────────────────────────
// Helper: build InlierMatch vector from raw pointers (used by both code paths)
// ─────────────────────────────────────────────────────────────────────────────
static void fill_pair_raw(PairRawData& out,
                          const uint8_t* mask_ptr, size_t num_matches,
                          const uint16_t* indices_data, size_t n_idx,
                          const float* coords_all, size_t n_coord,
                          const float* scales_all, size_t n_scale) {
  if (!mask_ptr || num_matches == 0 || !indices_data || n_idx < 2 || !coords_all || n_coord < 4)
    return;
  // Compute safe iteration bound once; avoids per-iteration multiply in the hot loop.
  const size_t safe_m = std::min({num_matches, n_idx / 2, n_coord / 4});
  const bool have_scales = (scales_all != nullptr &&
                            n_scale >= safe_m * 2u);
  out.matches.reserve(safe_m);
  for (size_t m = 0; m < safe_m; ++m) {
    if (!mask_ptr[m]) continue;
    InlierMatch im;
    im.idx1 = indices_data[m * 2];
    im.idx2 = indices_data[m * 2 + 1];
    im.x1   = coords_all[m * 4];
    im.y1   = coords_all[m * 4 + 1];
    im.x2   = coords_all[m * 4 + 2];
    im.y2   = coords_all[m * 4 + 3];
    im.s1   = have_scales ? scales_all[m * 2]     : 1.f;
    im.s2   = have_scales ? scales_all[m * 2 + 1] : 1.f;
    out.matches.push_back(im);
  }
  // shrink_to_fit() both trims excess capacity after filtering (non-empty case)
  // and releases the pre-reserved capacity when no inliers were found (empty case).
  out.matches.shrink_to_fit();
}

// Serial UF over one block of pre-loaded pairs, capturing coords on first node creation.
static void uf_block(UnionFind* uf, const std::vector<PairDesc>& pairs,
                     const std::vector<int>& idx_list, const std::vector<PairRawData>& blk_raw,
                     int& merged, int& rejected) {
  const int blk_n = static_cast<int>(idx_list.size());
  for (int bi = 0; bi < blk_n; ++bi) {
    const PairRawData& rd = blk_raw[static_cast<size_t>(bi)];
    if (rd.matches.empty()) continue;
    const uint32_t img1 = pairs[static_cast<size_t>(idx_list[bi])].image1_index;
    const uint32_t img2 = pairs[static_cast<size_t>(idx_list[bi])].image2_index;
    for (const InlierMatch& im : rd.matches) {
      if (uf->merge_keys(node_key(img1, im.idx1), node_key(img2, im.idx2),
                         im.x1, im.y1, im.s1, im.x2, im.y2, im.s2))
        ++merged;
      else
        ++rejected;
    }
  }
}

static void phase0_1_pipeline(const std::vector<PairDesc>& pairs, UnionFind* uf,
                               int& total_loaded, int& total_skipped) {
  const int n = static_cast<int>(pairs.size());
  const int log_interval = std::max(1, n / 20);
  std::atomic<int> total_done{0};

  std::map<std::string, std::vector<int>> geopack_groups;
  std::vector<int> legacy_idx;
  for (int i = 0; i < n; ++i) {
    if (pairs[static_cast<size_t>(i)].use_geopack)
      geopack_groups[pairs[static_cast<size_t>(i)].geopack_file].push_back(i);
    else
      legacy_idx.push_back(i);
  }

  int merged_total = 0, rejected_total = 0;

  // ── Geopack: one block fread at a time (~300 MB), UF, free ───────────────
  int block_no = 0;
  const int num_blocks = static_cast<int>(geopack_groups.size());
  for (auto& [pack_path, idx_list] : geopack_groups) {
    ++block_no;

    IDCReader pack_rd(pack_path);
    if (!pack_rd.is_valid()) {
      const int skip_n = static_cast<int>(idx_list.size());
      total_skipped += skip_n;
      total_done.fetch_add(skip_n, std::memory_order_relaxed);
      LOG(WARNING) << "Phase 0+1 block " << block_no << "/" << num_blocks
                   << ": unreadable, skipping " << skip_n << " pairs";
      continue;
    }
    std::vector<uint8_t> pack_payload = pack_rd.read_full_payload();
    LOG(INFO) << "Phase 0+1 block " << block_no << "/" << num_blocks << ": "
              << idx_list.size() << " pairs, payload=" << (pack_payload.size() >> 20) << " MB";

    const int blk_n = static_cast<int>(idx_list.size());
    std::vector<PairRawData> blk_raw(static_cast<size_t>(blk_n));
    int blk_loaded = 0, blk_skipped = 0;

    // Phase 0 for this block: parallel I/O (mask from payload, match from disk).
#pragma omp parallel for schedule(dynamic, 64) reduction(+:blk_loaded,blk_skipped)
    for (int bi = 0; bi < blk_n; ++bi) {
      const int i = idx_list[static_cast<size_t>(bi)];
      const PairDesc& pd = pairs[static_cast<size_t>(i)];

      size_t mask_size = 0;
      const uint8_t* mask_ptr = nullptr;
      if (!pd.geopack_f_blob.empty())
        mask_ptr = pack_rd.get_blob_from_payload(pd.geopack_f_blob, pack_payload, &mask_size);
      if (!mask_ptr || mask_size == 0) {
        if (!pd.geopack_e_blob.empty())
          mask_ptr = pack_rd.get_blob_from_payload(pd.geopack_e_blob, pack_payload, &mask_size);
      }
      if (!mask_ptr || mask_size == 0) {
        ++blk_skipped;
        const int d = ++total_done;
        if (d % log_interval == 0) {
#pragma omp critical(phase01_log)
          LOG(INFO) << "Phase 0+1: " << d << "/" << n << " pairs processed";
        }
        continue;
      }

      IDCReader match_rd(pd.match_file);
      if (!match_rd.is_valid()) { ++blk_skipped; ++total_done; continue; }
      thread_local std::vector<uint8_t> tl_mpl;
      match_rd.read_full_payload_into(tl_mpl);
      size_t idx_sz = 0, coord_sz = 0, scale_sz = 0;
      const auto* idx_ptr   = reinterpret_cast<const uint16_t*>(
          match_rd.get_blob_from_payload("indices",      tl_mpl, &idx_sz));
      const auto* coord_ptr = reinterpret_cast<const float*>(
          match_rd.get_blob_from_payload("coords_pixel", tl_mpl, &coord_sz));
      const auto* scale_ptr = reinterpret_cast<const float*>(
          match_rd.get_blob_from_payload("scales",       tl_mpl, &scale_sz));

      fill_pair_raw(blk_raw[static_cast<size_t>(bi)], mask_ptr, mask_size,
                    idx_ptr,   idx_sz   / sizeof(uint16_t),
                    coord_ptr, coord_sz / sizeof(float),
                    scale_ptr, scale_sz / sizeof(float));
      if (!blk_raw[static_cast<size_t>(bi)].matches.empty()) ++blk_loaded; else ++blk_skipped;

      const int d = ++total_done;
      if (d % log_interval == 0) {
#pragma omp critical(phase01_log)
        LOG(INFO) << "Phase 0+1: " << d << "/" << n << " pairs processed";
      }
    }  // OMP

    // Phase 1 for this block (serial, coord-capturing UF).
    int blk_merged = 0, blk_rejected = 0;
    uf_block(uf, pairs, idx_list, blk_raw, blk_merged, blk_rejected);
    merged_total  += blk_merged;
    rejected_total += blk_rejected;
    total_loaded  += blk_loaded;
    total_skipped += blk_skipped;
    // blk_raw and pack_payload destroyed here → frees ~1.5 GB.
  }

  // ── Legacy per-pair .isat_geo ─────────────────────────────────────────────
  const int leg_n = static_cast<int>(legacy_idx.size());
  if (leg_n > 0) {
    LOG(INFO) << "Phase 0+1 legacy: " << leg_n << " per-pair .isat_geo pairs";
    std::vector<PairRawData> leg_raw(static_cast<size_t>(leg_n));
    int leg_loaded = 0, leg_skipped = 0;

#pragma omp parallel for schedule(dynamic, 64) reduction(+:leg_loaded,leg_skipped)
    for (int li = 0; li < leg_n; ++li) {
      const int i = legacy_idx[static_cast<size_t>(li)];
      const PairDesc& pd = pairs[static_cast<size_t>(i)];

      IDCReader geo_rd(pd.geo_file);
      if (!geo_rd.is_valid()) { ++leg_skipped; ++total_done; continue; }
      thread_local std::vector<uint8_t> tl_gpl;
      geo_rd.read_full_payload_into(tl_gpl);
      size_t mask_size = 0;
      const uint8_t* mask_ptr = geo_rd.get_blob_from_payload("F_inliers", tl_gpl, &mask_size);
      if (!mask_ptr || mask_size == 0)
        mask_ptr = geo_rd.get_blob_from_payload("E_inliers", tl_gpl, &mask_size);
      if (!mask_ptr || mask_size == 0) { ++leg_skipped; ++total_done; continue; }

      IDCReader match_rd(pd.match_file);
      if (!match_rd.is_valid()) { ++leg_skipped; ++total_done; continue; }
      thread_local std::vector<uint8_t> tl_mpl_leg;
      match_rd.read_full_payload_into(tl_mpl_leg);
      size_t idx_sz = 0, coord_sz = 0, scale_sz = 0;
      const auto* idx_ptr   = reinterpret_cast<const uint16_t*>(
          match_rd.get_blob_from_payload("indices",      tl_mpl_leg, &idx_sz));
      const auto* coord_ptr = reinterpret_cast<const float*>(
          match_rd.get_blob_from_payload("coords_pixel", tl_mpl_leg, &coord_sz));
      const auto* scale_ptr = reinterpret_cast<const float*>(
          match_rd.get_blob_from_payload("scales",       tl_mpl_leg, &scale_sz));

      fill_pair_raw(leg_raw[static_cast<size_t>(li)], mask_ptr, mask_size,
                    idx_ptr,   idx_sz   / sizeof(uint16_t),
                    coord_ptr, coord_sz / sizeof(float),
                    scale_ptr, scale_sz / sizeof(float));
      if (!leg_raw[static_cast<size_t>(li)].matches.empty()) ++leg_loaded; else ++leg_skipped;

      const int d = ++total_done;
      if (d % log_interval == 0) {
#pragma omp critical(phase01_log)
        LOG(INFO) << "Phase 0+1 legacy: " << d << "/" << n << " pairs processed";
      }
    }
    int leg_merged = 0, leg_rejected = 0;
    uf_block(uf, pairs, legacy_idx, leg_raw, leg_merged, leg_rejected);
    merged_total  += leg_merged;
    rejected_total += leg_rejected;
    total_loaded  += leg_loaded;
    total_skipped += leg_skipped;
    // leg_raw freed here.
  }

  LOG(INFO) << "Phase 0+1 done: loaded=" << total_loaded << " skipped=" << total_skipped
            << " merged_edges=" << merged_total
            << " rejected_same_image=" << rejected_total
            << " unique_nodes=" << uf->node_id_.size()
            << " (threads=" << omp_get_max_threads() << ")";
}

// ─────────────────────────────────────────────────────────────────────────────
// Phase 2: observations from UF node iteration — O(N_unique_features)
//
// The UF already has every unique (image, feature) pair as a node, with coords
// stored at first creation.  Iterating nodes directly replaces the old approach
// of re-visiting all 560 M non-unique inlier matches with 560 M hash-set ops.
// Memory: ~20 M × 28 B = ~560 MB temporary sort buffer, freed after insert.
// ─────────────────────────────────────────────────────────────────────────────
static void phase2_from_nodes(TrackStore* store, UnionFind& uf,
                               const std::unordered_map<int, int>& root_to_track_id) {
  struct ObsEntry {
    int      track_id;
    uint32_t image_index;
    uint32_t feature_id;
    float    u, v, scale;
  };

  std::vector<ObsEntry> obs;
  obs.reserve(uf.node_id_.size());

  for (const auto& [key, id] : uf.node_id_) {
    const int root = uf.find_by_id(id);
    auto it = root_to_track_id.find(root);
    if (it == root_to_track_id.end()) continue;
    obs.push_back({it->second,
                   image_index_from_node_key(key),
                   static_cast<uint32_t>(key & 0xFFFFFFFFu),
                   uf.node_u_[static_cast<size_t>(id)],
                   uf.node_v_[static_cast<size_t>(id)],
                   uf.node_s_[static_cast<size_t>(id)]});
  }

  // Sort by (track_id, image_index, feature_id) — required by TrackStore.
  std::sort(obs.begin(), obs.end(), [](const ObsEntry& a, const ObsEntry& b) {
    if (a.track_id    != b.track_id)    return a.track_id    < b.track_id;
    if (a.image_index != b.image_index) return a.image_index < b.image_index;
    return a.feature_id < b.feature_id;
  });

  for (const ObsEntry& e : obs)
    store->add_observation(e.track_id, e.image_index, e.feature_id, e.u, e.v, e.scale);

  LOG(INFO) << "Phase 2: " << obs.size() << " observations from "
            << uf.node_id_.size() << " unique features";
  // obs freed here (~560 MB released).
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
  int num_threads = 0;  // 0 = auto (use all available cores)
  cmd.add(make_option(0, num_threads, "num-threads")
              .doc("Number of threads for parallel I/O pre-load. Default=0 (all cores)."));
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
    ViewGraph view_graph;
    if (!load_track_store_from_idc(output_path, &store, &image_indices, &view_graph))
      return 1;
    LOG(INFO) << "Tracks: " << store.num_tracks() << "  Observations: " << store.num_observations()
              << "  Images: " << image_indices.size()
              << "  view_graph_pairs: " << view_graph.num_pairs();
    print_event({{"type", "tracks.stats"},
                {"ok", true},
                {"data",
                 {{"num_tracks", static_cast<int>(store.num_tracks())},
                  {"num_observations", static_cast<int>(store.num_observations())},
                  {"num_images", static_cast<int>(image_indices.size())},
                  {"view_graph_pairs", static_cast<int>(view_graph.num_pairs())}}}});
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

  insight::io::GeoPackIndex geopack_index;
  const bool has_geopack = geopack_index.load_from_dir(geo_dir);
  if (has_geopack)
    LOG(INFO) << "Geo input mode: geopack_index.isat_gpkx + .isat_geopack (fallback .isat_geo/.json-index)";
  else
    LOG(INFO) << "Geo input mode: legacy per-pair .isat_geo";

  // Apply thread count (0 = all available cores)
  if (num_threads > 0) {
    omp_set_num_threads(num_threads);
    LOG(INFO) << "Using " << num_threads << " threads for parallel I/O";
  } else {
    LOG(INFO) << "Using " << omp_get_max_threads() << " threads for parallel I/O (auto)";
  }

  std::vector<PairDesc> pairs =
      load_pairs(pairs_json, match_dir, geo_dir, has_geopack ? &geopack_index : nullptr);
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

  // ── Phase 0+1 combined: block-interleaved I/O + Union-Find ────────────────
  // Pre-reserve node_id_ to avoid repeated rehash. Estimate: n_images * 512 matched features.
  UnionFind uf;
  uf.node_id_.reserve(static_cast<size_t>(n_images) * 512u);
  uf.parent_.reserve(static_cast<size_t>(n_images) * 512u);
  uf.component_size_.reserve(static_cast<size_t>(n_images) * 512u);
  uf.component_images_.reserve(static_cast<size_t>(n_images) * 512u);
  uf.node_u_.reserve(static_cast<size_t>(n_images) * 512u);
  uf.node_v_.reserve(static_cast<size_t>(n_images) * 512u);
  uf.node_s_.reserve(static_cast<size_t>(n_images) * 512u);

  LOG(INFO) << "Phase 0+1: loading+UF " << pairs.size() << " pairs (block-interleaved)...";
  auto t0 = std::chrono::steady_clock::now();
  int loaded_count = 0, skipped_count = 0;
  phase0_1_pipeline(pairs, &uf, loaded_count, skipped_count);
  auto t1 = std::chrono::steady_clock::now();
  LOG(INFO) << "Phase 0+1 wall time: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() << " ms";

  std::unordered_map<int, int> root_to_track_id;
  root_to_track_id.reserve(uf.node_id_.size());
  int next_track = 0;
  for (const auto& kv : uf.node_id_) {
    int root = uf.find_by_id(kv.second);
    if (root_to_track_id.find(root) == root_to_track_id.end())
      root_to_track_id[root] = next_track++;
  }
  const int num_tracks = next_track;
  LOG(INFO) << "Phase 1 (UF): " << num_tracks << " tracks from " << uf.node_id_.size()
            << " unique features";

  TrackStore store;
  store.set_num_images(n_images);
  store.reserve_tracks(static_cast<size_t>(num_tracks));
  store.reserve_observations(static_cast<size_t>(uf.node_id_.size()));
  for (int t = 0; t < num_tracks; ++t)
    store.add_track(0.f, 0.f, 0.f);

  // ── Phase 2: observations from UF node iteration (O(N_unique_features)) ───
  LOG(INFO) << "Phase 2: filling " << uf.node_id_.size() << " unique feature observations...";
  auto t2 = std::chrono::steady_clock::now();
  phase2_from_nodes(&store, uf, root_to_track_id);
  auto t3 = std::chrono::steady_clock::now();
  LOG(INFO) << "Phase 2 wall time: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count() << " ms";
  LOG(INFO) << "Phase 2: " << store.num_observations() << " observations";

  // Release UF (~1.1 GB: node_id_ ~400 MB, node_uvs ~240 MB, component_images_ headers ~480 MB)
  // and root_to_track_id (~400 MB elements + ~128 MB bucket array) immediately.
  // Use move-assignment from a default-constructed object to guarantee full deallocation
  // (including bucket arrays that unordered_map::clear() would retain).
  uf = UnionFind{};
  root_to_track_id = std::unordered_map<int,int>();

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
    // Release the unfiltered store (~560 MB) now that filtered_store is canonical.
    store = TrackStore{};
  }

  std::vector<std::pair<uint32_t, uint32_t>> direct_pairs;
  direct_pairs.reserve(pairs.size());
  for (const auto& p : pairs)
    direct_pairs.emplace_back(p.image1_index, p.image2_index);

  ViewGraph view_graph;
  if (!build_view_graph_from_pairs_list_and_track_store(direct_pairs, geo_dir, *store_to_save,
                                                      &view_graph)) {
    LOG(ERROR) << "Failed to build view graph from pairs list + filtered tracks + geo_dir";
    return 1;
  }
  if (!save_track_store_to_idc(*store_to_save, image_indices, output_path, &view_graph))
    return 1;
  print_event({{"type", "tracks.build"},
              {"ok", true},
              {"data",
               {{"output", output_path},
                {"min_track_length", min_track_length},
                {"num_tracks", static_cast<int>(store_to_save->num_tracks())},
                {"num_observations", static_cast<int>(store_to_save->num_observations())},
                {"view_graph_pairs", static_cast<int>(view_graph.num_pairs())}}}});
  return 0;
}
