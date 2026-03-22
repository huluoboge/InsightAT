/**
 * @file  view_graph_loader.cpp
 * @brief Load ViewGraph from pairs.json and .isat_geo directory; track-consistent graph + JSON I/O.
 */

#include "view_graph_loader.h"
#include "../../io/idc_reader.h"
#include "../../tools/pair_json_utils.h"
#include "track_store.h"
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <glog/logging.h>
#include <set>
#include <unordered_set>
#include <nlohmann/json.hpp>

namespace insight {
namespace sfm {

namespace {

/// Avoid constructing IDCReader when the file is absent — IDCReader logs ERROR on open failure,
/// which is noisy for optional pairs (e.g. covisibility edges without a direct geo file).
bool geo_file_exists(const std::string& path) {
  std::error_code ec;
  return std::filesystem::is_regular_file(std::filesystem::path(path), ec);
}

} // namespace

void pair_geo_info_from_isat_geo_metadata(const nlohmann::json& meta, PairGeoInfo* info) {
  if (!info)
    return;
  if (meta.contains("geometry")) {
    const auto& gm = meta["geometry"];
    if (gm.contains("F")) {
      const auto& jF = gm["F"];
      if (jF.contains("estimated"))
        info->F_ok = jF["estimated"].get<bool>();
      if (jF.contains("num_inliers"))
        info->F_inliers = jF["num_inliers"].get<int>();
      if (jF.contains("inlier_ratio"))
        info->F_inlier_ratio = jF["inlier_ratio"].get<float>();
    }
    if (gm.contains("H")) {
      const auto& jH = gm["H"];
      if (jH.contains("estimated"))
        info->H_ok = jH["estimated"].get<bool>();
      if (jH.contains("num_inliers"))
        info->H_inliers = jH["num_inliers"].get<int>();
    }
    if (gm.contains("degeneracy") && gm["degeneracy"].contains("is_degenerate"))
      info->is_degenerate = gm["degeneracy"]["is_degenerate"].get<bool>();
    if (gm.contains("E")) {
      const auto& jE = gm["E"];
      if (jE.contains("estimated"))
        info->E_ok = jE["estimated"].get<bool>();
      if (jE.contains("num_inliers"))
        info->E_inliers = jE["num_inliers"].get<int>();
    }
  }
  if (meta.contains("twoview")) {
    const auto& tv = meta["twoview"];
    info->twoview_ok = true;
    if (tv.contains("stable"))
      info->stable = tv["stable"].get<bool>();
    if (tv.contains("num_valid_points"))
      info->num_valid_points = tv["num_valid_points"].get<int>();
  }
}

bool load_pair_geo_info_from_isat_geo_file(const std::string& geo_path, uint32_t image1_index,
                                           uint32_t image2_index, PairGeoInfo* out) {
  if (!out)
    return false;
  if (!geo_file_exists(geo_path))
    return false;
  io::IDCReader reader(geo_path);
  if (!reader.is_valid())
    return false;
  *out = PairGeoInfo{};
  out->image1_index = image1_index;
  out->image2_index = image2_index;
  pair_geo_info_from_isat_geo_metadata(reader.get_metadata(), out);
  return true;
}

void collect_covisible_image_pairs_from_track_store(const TrackStore& store,
                                                    std::vector<std::pair<uint32_t, uint32_t>>* out_pairs) {
  if (!out_pairs)
    return;
  out_pairs->clear();
  std::set<std::pair<uint32_t, uint32_t>> edge_set;
  std::vector<Observation> obs;
  for (int t = 0; t < static_cast<int>(store.num_tracks()); ++t) {
    if (!store.is_track_valid(t))
      continue;
    obs.clear();
    store.get_track_observations(t, &obs);
    if (obs.size() < 2)
      continue;
    std::vector<uint32_t> imgs;
    imgs.reserve(obs.size());
    for (const auto& o : obs)
      imgs.push_back(o.image_index);
    std::sort(imgs.begin(), imgs.end());
    imgs.erase(std::unique(imgs.begin(), imgs.end()), imgs.end());
    for (size_t a = 0; a < imgs.size(); ++a) {
      for (size_t b = a + 1; b < imgs.size(); ++b) {
        uint32_t i = imgs[a];
        uint32_t j = imgs[b];
        if (i > j)
          std::swap(i, j);
        edge_set.insert({i, j});
      }
    }
  }
  out_pairs->assign(edge_set.begin(), edge_set.end());
  std::sort(out_pairs->begin(), out_pairs->end());
}

bool build_view_graph_from_pairs_list_and_track_store(
    const std::vector<std::pair<uint32_t, uint32_t>>& canonical_pairs, const std::string& geo_dir,
    const TrackStore& store, ViewGraph* out) {
  if (!out)
    return false;
  // Co-visibility in the filtered store (any pair of images on the same track at least once).
  std::vector<std::pair<uint32_t, uint32_t>> cov_vec;
  collect_covisible_image_pairs_from_track_store(store, &cov_vec);
  std::unordered_set<uint64_t> covisible;
  covisible.reserve(cov_vec.size() * 2u + 1u);
  for (const auto& e : cov_vec) {
    covisible.insert((static_cast<uint64_t>(e.first) << 32) | static_cast<uint64_t>(e.second));
  }

  std::string dir = geo_dir;
  if (!dir.empty() && dir.back() != '/')
    dir += '/';
  out->reserve(canonical_pairs.size());
  size_t loaded = 0;
  size_t skipped_not_covisible = 0;
  size_t skipped_no_geo = 0;
  for (const auto& e : canonical_pairs) {
    uint32_t i = e.first;
    uint32_t j = e.second;
    if (i > j)
      std::swap(i, j);
    const uint64_t key = (static_cast<uint64_t>(i) << 32) | static_cast<uint64_t>(j);
    if (!covisible.count(key)) {
      ++skipped_not_covisible;
      continue;
    }
    const std::string path = dir + std::to_string(i) + "_" + std::to_string(j) + ".isat_geo";
    PairGeoInfo info;
    if (!load_pair_geo_info_from_isat_geo_file(path, i, j, &info)) {
      ++skipped_no_geo;
      continue;
    }
    out->add_pair(info);
    ++loaded;
  }
  LOG(INFO) << "build_view_graph_from_pairs_list_and_track_store: " << loaded << " pairs embedded (from "
            << canonical_pairs.size() << " direct pairs; skipped_not_covisible=" << skipped_not_covisible
            << " skipped_no_geo_file=" << skipped_no_geo << " geo_dir=" << geo_dir << ")";
  return true;
}

nlohmann::json view_graph_pairs_to_json_array(const ViewGraph& vg) {
  nlohmann::json arr = nlohmann::json::array();
  for (size_t i = 0; i < vg.num_pairs(); ++i) {
    const PairGeoInfo& p = vg.pair_at(i);
    nlohmann::json o;
    o["image1_index"] = p.image1_index;
    o["image2_index"] = p.image2_index;
    o["F_ok"] = p.F_ok;
    o["F_inliers"] = p.F_inliers;
    o["F_inlier_ratio"] = p.F_inlier_ratio;
    o["H_ok"] = p.H_ok;
    o["H_inliers"] = p.H_inliers;
    o["is_degenerate"] = p.is_degenerate;
    o["E_ok"] = p.E_ok;
    o["E_inliers"] = p.E_inliers;
    o["twoview_ok"] = p.twoview_ok;
    o["stable"] = p.stable;
    o["num_valid_points"] = p.num_valid_points;
    arr.push_back(std::move(o));
  }
  return arr;
}

bool view_graph_from_json_array(const nlohmann::json& arr, ViewGraph* out) {
  if (!out || !arr.is_array())
    return false;
  out->clear();
  out->reserve(arr.size());
  for (const auto& el : arr) {
    if (!el.is_object())
      return false;
    PairGeoInfo p;
    p.image1_index = el.value("image1_index", 0u);
    p.image2_index = el.value("image2_index", 0u);
    p.F_ok = el.value("F_ok", false);
    p.F_inliers = el.value("F_inliers", 0);
    p.F_inlier_ratio = el.value("F_inlier_ratio", 0.f);
    p.H_ok = el.value("H_ok", false);
    p.H_inliers = el.value("H_inliers", 0);
    p.is_degenerate = el.value("is_degenerate", false);
    p.E_ok = el.value("E_ok", false);
    p.E_inliers = el.value("E_inliers", 0);
    p.twoview_ok = el.value("twoview_ok", false);
    p.stable = el.value("stable", false);
    p.num_valid_points = el.value("num_valid_points", 0);
    out->add_pair(p);
  }
  return true;
}

bool build_view_graph_from_geo(const std::string& pairs_json_path, const std::string& geo_dir,
                               ViewGraph* out) {
  if (!out)
    return false;
  out->reserve(4096);
  std::ifstream file(pairs_json_path);
  if (!file.is_open())
    return false;
  nlohmann::json j;
  try {
    file >> j;
  } catch (...) {
    return false;
  }
  if (!j.contains("pairs") || !j["pairs"].is_array())
    return false;
  const size_t num_pairs_json = j["pairs"].size();
  std::string dir = geo_dir;
  if (!dir.empty() && dir.back() != '/')
    dir += '/';
  size_t num_loaded = 0;
  for (const auto& p : j["pairs"]) {
    uint32_t idx1 = insight::tools::get_image_index_from_pair(p, "image1_index");
    uint32_t idx2 = insight::tools::get_image_index_from_pair(p, "image2_index");
    if (idx1 > idx2)
      std::swap(idx1, idx2);
    std::string geo_path = dir + std::to_string(idx1) + "_" + std::to_string(idx2) + ".isat_geo";
    if (!geo_file_exists(geo_path))
      continue;
    insight::io::IDCReader reader(geo_path);
    if (!reader.is_valid())
      continue;
    PairGeoInfo info;
    info.image1_index = idx1;
    info.image2_index = idx2;
    pair_geo_info_from_isat_geo_metadata(reader.get_metadata(), &info);
    out->add_pair(info);
    ++num_loaded;
  }
  LOG(INFO) << "build_view_graph_from_geo: " << num_loaded << " pairs loaded from " << num_pairs_json
            << " (geo_dir=" << geo_dir << ")";
  return true;
}

} // namespace sfm
} // namespace insight
