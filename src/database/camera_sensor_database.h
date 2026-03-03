/**
 * @file  camera_sensor_database.h
 * @brief 相机传感器数据库单例：加载 camera_sensor_database.txt，按制造商/型号查询传感器宽度。
 */

#pragma once
#ifndef INSIGHT_CAMERA_SENSOR_DATABASE_H
#define INSIGHT_CAMERA_SENSOR_DATABASE_H

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace insight {
namespace database {

/**
 * @struct SensorInfo
 * @brief 传感器信息（制造商、型号、传感器宽度 mm）。
 */
struct SensorInfo {
  std::string manufacturer;
  std::string model;
  double sensor_width_mm = 0.0;
};

/**
 * @class CameraSensorDatabase
 * @brief 相机传感器数据库单例：加载 TSV、按制造商/型号查询传感器宽度。
 */
class CameraSensorDatabase {
public:
  static CameraSensorDatabase& instance();

  /**
   * @brief 加载数据库文件
   * @param db_path 数据库文件路径 (camera_sensor_database.txt)
   * @return 成功返回 true
   */
  bool load(const std::string& db_path);

  /**
   * @brief 根据制造商和型号查询传感器宽度
   * @param manufacturer 制造商 (Brand)
   * @param model 型号
   * @param width_mm_out 传感器宽度 (mm)，输出
   * @return 找到返回 true
   */
  bool query_sensor_width(const std::string& manufacturer, const std::string& model,
                          double& width_mm_out) const;

  /**
   * @brief 获取所有已加载的传感器信息
   */
  const std::vector<SensorInfo>& all_sensors() const { return sensors_; }

private:
  CameraSensorDatabase() = default;
  ~CameraSensorDatabase() = default;
  CameraSensorDatabase(const CameraSensorDatabase&) = delete;
  CameraSensorDatabase& operator=(const CameraSensorDatabase&) = delete;

  std::vector<SensorInfo> sensors_;
  std::map<std::string, double> lookup_table_; ///< "manufacturer|model" -> sensor_width (mm)

  std::string normalize_string(const std::string& s) const;
};

} // namespace database
} // namespace insight

#endif // INSIGHT_CAMERA_SENSOR_DATABASE_H
