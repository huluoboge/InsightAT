/**
 * @file UISystemConfig.h
 * @brief UI 系统配置单例 - 管理坐标系数据库
 */

#ifndef UI_UISYSTEMCONFIG_H
#define UI_UISYSTEMCONFIG_H

#include <string>
#include <vector>
#include <memory>

namespace insight {

struct Coordinate;

namespace ui {

/**
 * @class UISystemConfig
 * @brief UI 层系统配置单例
 * 
 * 负责管理并加载坐标系数据库（GEOGCS 和 PROJCS）
 */
class UISystemConfig
{
public:
    static UISystemConfig& instance();
    static void exit();

    // 设置配置路径
    void setConfigPath(const std::string& configPath);
    std::string configPath() const;

    // 加载坐标系数据库
    bool loadCoordinateDatabases();

    // 获取加载的坐标系
    const std::vector<Coordinate>& getGeoCoordinates() const;
    const std::vector<Coordinate>& getProjCoordinates() const;
    
    // 合并的坐标系列表
    const std::vector<Coordinate>& getAllCoordinates() const;

    // 查询功能
    Coordinate findByEPSG(int epsg) const;
    std::vector<Coordinate> searchByKeyword(const std::string& keyword) const;
    
    // 检查是否已加载
    bool isLoaded() const { return m_loaded; }

private:
    UISystemConfig();
    UISystemConfig(const UISystemConfig&);
    UISystemConfig& operator=(const UISystemConfig&);
    ~UISystemConfig();

    std::string m_configPath;
    std::vector<Coordinate> m_geoCoordinates;
    std::vector<Coordinate> m_projCoordinates;
    std::vector<Coordinate> m_allCoordinates;  // 合并列表
    bool m_loaded = false;
};

}  // namespace ui
}  // namespace insight

#endif  // UI_UISYSTEMCONFIG_H
