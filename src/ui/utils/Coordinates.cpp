/**
 * @file Coordinates.cpp
 * @brief 坐标系实现（从 Common 迁入）
 */

#include "Coordinates.h"
#include "../../util/string_utils.h"
#include <fstream>
#include <sstream>
#include <glog/logging.h>
#include "stlplus3/filesystemSimplified/file_system.hpp"
#include "gdal.h"
#include <ogr_spatialref.h>
#include <cstring>

namespace insight {

bool parseCoordinates(std::vector<Coordinate>& coords, const std::string& dataBse) {
  if (!stlplus::file_exists(dataBse))
    return false;
  coords.clear();
  std::ifstream ifs(dataBse);
  char* buffer = new char[90000];
  coords.reserve(7000);
  while (ifs.getline(buffer, 90000)) {
    std::vector<std::string> values;
    split(buffer, ";", values);
    if (values.size() == 3) {
      Coordinate c;
      c.EPSGName = values[0];
      c.CoordinateName = values[1];
      c.WKT = values[2];
      coords.push_back(c);
    }
  }
  delete[] buffer;
  return true;
}

bool Coordinate::isOK() const {
  if (EPSGName.empty()) return false;
  OGRSpatialReference oPGS;
  std::stringstream ss(EPSGName);
  int epsg = 0;
  ss >> epsg;
  return oPGS.importFromEPSG(epsg) == OGRERR_NONE;
}

bool Coordinate::isProject(bool* ok) const {
  OGRSpatialReference oPGS;
  std::stringstream ss(EPSGName);
  int epsg = 0;
  ss >> epsg;
  if (oPGS.importFromEPSG(epsg) != OGRERR_NONE) {
    LOG(ERROR) << "Can't import epsg " << epsg;
    if (ok) *ok = false;
    return false;
  }
  if (ok) *ok = true;
  return oPGS.IsProjected();
}

int Coordinate::EPSG(bool* ok) const {
  if (EPSGName.empty()) {
    if (ok) *ok = false;
    return 0;
  }
  std::string s = EPSGName;
  trim(s);
  int epsg = 0;
  std::stringstream ss(s);
  ss >> epsg;
  if (ok) *ok = true;
  return epsg;
}

bool Coordinate::coordToSR(const Coordinate& coord, OGRSpatialReference& sr) {
  bool ok = false;
  int fromEPSG = coord.EPSG(&ok);
  if (ok) {
    if (sr.importFromEPSG(fromEPSG) != OGRERR_NONE) {
      return false;
    }
  } else if (!coord.WKT.empty()) {
    char buf[8192] = {0};
    const char* pBuf = buf;
    memcpy(buf, coord.WKT.c_str(), std::min(coord.WKT.size(), sizeof(buf) - 1));
    if (sr.importFromWkt((const char**)&pBuf) != OGRERR_NONE) {
      return false;
    }
  } else {
    return false;
  }
  return true;
}

}  // namespace insight
