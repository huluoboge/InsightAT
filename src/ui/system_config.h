/**
 * @file  system_config.h
 * @brief UI 系统配置单例，管理坐标系数据库。
 */

#ifndef UI_SYSTEM_CONFIG_H
#define UI_SYSTEM_CONFIG_H

#include <memory>
#include <string>
#include <vector>

namespace insight {

struct Coordinate;

namespace ui {

/**
 * @class UISystemConfig
 * @brief UI 层系统配置单例
 *
 * 负责管理并加载坐标系数据库（GEOGCS 和 PROJCS）
 */
class UISystemConfig {
public:
  static UISystemConfig& instance();
  static void exit();

  void set_config_path(const std::string& config_path);
  std::string config_path() const;

  bool load_coordinate_databases();

  const std::vector<Coordinate>& get_geo_coordinates() const;
  const std::vector<Coordinate>& get_proj_coordinates() const;
  const std::vector<Coordinate>& get_all_coordinates() const;

  Coordinate find_by_epsg(int epsg) const;
  std::vector<Coordinate> search_by_keyword(const std::string& keyword) const;

  bool is_loaded() const { return m_loaded; }

private:
  UISystemConfig();
  UISystemConfig(const UISystemConfig&);
  UISystemConfig& operator=(const UISystemConfig&);
  ~UISystemConfig();

  std::string m_configPath;
  std::vector<Coordinate> m_geoCoordinates;
  std::vector<Coordinate> m_projCoordinates;
  std::vector<Coordinate> m_allCoordinates; // 合并列表
  bool m_loaded = false;
};

}  // namespace ui
}  // namespace insight

#endif  // UI_UISYSTEMCONFIG_H
