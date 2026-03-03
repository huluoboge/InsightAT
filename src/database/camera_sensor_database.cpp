/**
 * @file  camera_sensor_database.cpp
 * @brief 相机传感器数据库单例实现：解析 TSV、归一化查询键、内存查找。
 */

#include "camera_sensor_database.h"

#include <algorithm>
#include <fstream>
#include <sstream>

#include <glog/logging.h>

namespace insight {
namespace database {

CameraSensorDatabase& CameraSensorDatabase::instance() {
  static CameraSensorDatabase inst;
  return inst;
}

bool CameraSensorDatabase::load(const std::string& db_path) {
  std::ifstream file(db_path);
  if (!file.is_open()) {
    LOG(ERROR) << "Failed to open camera sensor database: " << db_path;
    return false;
  }

  sensors_.clear();
  lookup_table_.clear();

  std::string line;
  while (std::getline(file, line)) {
    if (line.empty())
      continue;

    std::stringstream ss(line);
    std::string manufacturer, model, width_str;

    if (std::getline(ss, manufacturer, ';') && std::getline(ss, model, ';') &&
        std::getline(ss, width_str, ';')) {

      try {
        double width = std::stod(width_str);
        SensorInfo info{manufacturer, model, width};
        sensors_.push_back(info);

        std::string key = normalize_string(manufacturer) + "|" + normalize_string(model);
        lookup_table_[key] = width;
      } catch (...) {
        LOG(WARNING) << "Failed to parse sensor width in line: " << line;
      }
    }
  }

  LOG(INFO) << "Loaded " << sensors_.size() << " sensors from " << db_path;
  return !sensors_.empty();
}

bool CameraSensorDatabase::query_sensor_width(const std::string& manufacturer,
                                              const std::string& model,
                                              double& width_mm_out) const {
  std::string key = normalize_string(manufacturer) + "|" + normalize_string(model);
  auto it = lookup_table_.find(key);
  if (it != lookup_table_.end()) {
    width_mm_out = it->second;
    return true;
  }
  return false;
}

std::string CameraSensorDatabase::normalize_string(const std::string& s) const {
  std::string res = s;
  std::transform(res.begin(), res.end(), res.begin(), ::tolower);
  res.erase(0, res.find_first_not_of(" \t\r\n"));
  res.erase(res.find_last_not_of(" \t\r\n") + 1);
  return res;
}

} // namespace database
} // namespace insight
