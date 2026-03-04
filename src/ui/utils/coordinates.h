/**
 * @file  coordinates.h
 * @brief 坐标系 WKT/EPSG 工具（从 Common 迁入，仅 ui 使用）
 */

#ifndef INSIGHT_UI_COORDINATES_H
#define INSIGHT_UI_COORDINATES_H

#include <string>
#include <vector>

class OGRSpatialReference;

namespace insight {

struct WKTTreeItem {
  std::string begin;
  std::string text;
  std::string end;
};

struct Coordinate {
  bool is_ok() const;
  std::string EPSGName;
  std::string CoordinateName;
  std::string WKT;
  bool is_project(bool* ok) const;
  int EPSG(bool* ok = nullptr) const;
  static bool coord_to_sr(const Coordinate& coord, OGRSpatialReference& sr);
};

bool parse_coordinates(std::vector<Coordinate>& coords, const std::string& dataBse);

}  // namespace insight

#endif  // INSIGHT_UI_COORDINATES_H
