/**
 * @file  view_graph_loader.cpp
 * @brief Load ViewGraph from pairs.json and .isat_geo directory.
 */

#include "view_graph_loader.h"
#include "../../io/idc_reader.h"
#include "../../tools/pair_json_utils.h"
#include <fstream>
#include <glog/logging.h>
#include <nlohmann/json.hpp>

namespace insight {
namespace sfm {

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
    // Geo files are always stored as min_max.isat_geo
    if (idx1 > idx2)
      std::swap(idx1, idx2);
    std::string geo_path = dir + std::to_string(idx1) + "_" + std::to_string(idx2) + ".isat_geo";
    insight::io::IDCReader reader(geo_path);
    if (!reader.is_valid())
      continue;
    const auto& meta = reader.get_metadata();
    PairGeoInfo info;
    info.image1_index = idx1;  // idx1 <= idx2 after canonical swap above
    info.image2_index = idx2;
    if (meta.contains("geometry")) {
      const auto& gm = meta["geometry"];
      // F — primary, always present
      if (gm.contains("F")) {
        const auto& jF = gm["F"];
        if (jF.contains("estimated"))
          info.F_ok = jF["estimated"].get<bool>();
        if (jF.contains("num_inliers"))
          info.F_inliers = jF["num_inliers"].get<int>();
        if (jF.contains("inlier_ratio"))
          info.F_inlier_ratio = jF["inlier_ratio"].get<float>();
      }
      // H — degeneracy signal
      if (gm.contains("H")) {
        const auto& jH = gm["H"];
        if (jH.contains("estimated"))
          info.H_ok = jH["estimated"].get<bool>();
        if (jH.contains("num_inliers"))
          info.H_inliers = jH["num_inliers"].get<int>();
      }
      // Degeneracy
      if (gm.contains("degeneracy") && gm["degeneracy"].contains("is_degenerate"))
        info.is_degenerate = gm["degeneracy"]["is_degenerate"].get<bool>();
      // E — supplementary
      if (gm.contains("E")) {
        const auto& jE = gm["E"];
        if (jE.contains("estimated"))
          info.E_ok = jE["estimated"].get<bool>();
        if (jE.contains("num_inliers"))
          info.E_inliers = jE["num_inliers"].get<int>();
      }
    }
    // twoview — supplementary, only present when isat_geo was run with --twoview
    if (meta.contains("twoview")) {
      const auto& tv = meta["twoview"];
      info.twoview_ok = true;
      if (tv.contains("stable"))
        info.stable = tv["stable"].get<bool>();
      if (tv.contains("num_valid_points"))
        info.num_valid_points = tv["num_valid_points"].get<int>();
    }
    out->add_pair(info);
    ++num_loaded;
  }
  LOG(INFO) << "build_view_graph_from_geo: " << num_loaded << " pairs loaded from " << num_pairs_json
            << " (geo_dir=" << geo_dir << ")";
  return true;
}

} // namespace sfm
} // namespace insight
