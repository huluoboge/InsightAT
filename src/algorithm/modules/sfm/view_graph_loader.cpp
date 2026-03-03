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
  std::string dir = geo_dir;
  if (!dir.empty() && dir.back() != '/')
    dir += '/';
  for (const auto& p : j["pairs"]) {
    uint32_t id1 = insight::tools::getImageIdFromPair(p, "image1_id");
    uint32_t id2 = insight::tools::getImageIdFromPair(p, "image2_id");
    std::string geo_path = dir + std::to_string(id1) + "_" + std::to_string(id2) + ".isat_geo";
    insight::io::IDCReader reader(geo_path);
    if (!reader.isValid())
      continue;
    const auto& meta = reader.getMetadata();
    PairGeoInfo info;
    info.image1_id = id1;
    info.image2_id = id2;
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
