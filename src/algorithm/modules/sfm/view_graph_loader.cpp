/**
 * @file  view_graph_loader.cpp
 * @brief Load ViewGraph from pairs.json and .isat_geo directory.
 */

#include "view_graph_loader.h"
#include "../../io/idc_reader.h"
#include "../../tools/pair_json_utils.h"
#include <fstream>
#include <nlohmann/json.hpp>

namespace insight {
namespace sfm {

bool build_view_graph_from_geo(const std::string& pairs_json_path, const std::string& geo_dir,
                               ViewGraph* out, const IdMapping* id_mapping) {
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
  std::string dir = geo_dir;
  if (!dir.empty() && dir.back() != '/')
    dir += '/';
  for (const auto& p : j["pairs"]) {
    uint32_t orig1 = insight::tools::get_image_id_from_pair(p, "image1_id");
    uint32_t orig2 = insight::tools::get_image_id_from_pair(p, "image2_id");
    int idx1 = id_mapping ? id_mapping->internal_image_index(orig1) : static_cast<int>(orig1);
    int idx2 = id_mapping ? id_mapping->internal_image_index(orig2) : static_cast<int>(orig2);
    if (id_mapping && (idx1 < 0 || idx2 < 0))
      continue;
    uint32_t path_id1 = id_mapping ? id_mapping->original_image_id(idx1) : orig1;
    uint32_t path_id2 = id_mapping ? id_mapping->original_image_id(idx2) : orig2;
    std::string geo_path = dir + std::to_string(path_id1) + "_" + std::to_string(path_id2) + ".isat_geo";
    insight::io::IDCReader reader(geo_path);
    if (!reader.is_valid())
      continue;
    const auto& meta = reader.get_metadata();
    PairGeoInfo info;
    info.image1_id = static_cast<uint32_t>(idx1);
    info.image2_id = static_cast<uint32_t>(idx2);
    if (meta.contains("geometry")) {
      const auto& gm = meta["geometry"];
      if (gm.contains("E") && gm["E"].contains("estimated"))
        info.E_ok = gm["E"]["estimated"].get<bool>();
      if (gm.contains("E") && gm["E"].contains("num_inliers"))
        info.E_inliers = gm["E"]["num_inliers"].get<int>();
    }
    if (meta.contains("twoview")) {
      const auto& tv = meta["twoview"];
      info.twoview_ok = true;
      if (tv.contains("stable"))
        info.stable = tv["stable"].get<bool>();
      if (tv.contains("num_valid_points"))
        info.num_valid_points = tv["num_valid_points"].get<int>();
    }
    out->add_pair(info);
  }
  return true;
}

} // namespace sfm
} // namespace insight
