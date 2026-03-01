/**
 * @file UISystemConfig.cpp
 * @brief UI 系统配置单例 - 实现
 */

#include "UISystemConfig.h"

#include "Common/Coordinates.h"
#include <algorithm>
#include <cctype>
#include <glog/logging.h>
#include <stlplus3/filesystemSimplified/file_system.hpp>

namespace insight {
namespace ui {

static UISystemConfig* g_uiSystemConfig = nullptr;

UISystemConfig& UISystemConfig::instance()
{
    if (g_uiSystemConfig == nullptr) {
        g_uiSystemConfig = new UISystemConfig();
    }
    return *g_uiSystemConfig;
}

void UISystemConfig::exit()
{
    if (g_uiSystemConfig) {
        delete g_uiSystemConfig;
        g_uiSystemConfig = nullptr;
    }
}

UISystemConfig::UISystemConfig()
{
    // 默认配置路径为 ./config
    m_configPath = "./config";
}

UISystemConfig::~UISystemConfig() = default;

void UISystemConfig::setConfigPath(const std::string& configPath)
{
    m_configPath = configPath;
}

std::string UISystemConfig::configPath() const
{
    return m_configPath;
}

bool UISystemConfig::loadCoordinateDatabases()
{
    // 构建完整路径
    std::string geoCoord = stlplus::create_filespec(m_configPath, "GEOGCS_Database.csv");
    std::string prjCoord = stlplus::create_filespec(m_configPath, "PROJCS_Database.csv");

    // 清空现有数据
    m_geoCoordinates.clear();
    m_projCoordinates.clear();
    m_allCoordinates.clear();
    m_loaded = false;

    // 检查地理坐标系数据库文件
    if (!stlplus::file_exists(geoCoord)) {
        LOG(ERROR) << "Can't read GEOGCS database: " << geoCoord;
        return false;
    }

    // 检查投影坐标系数据库文件
    if (!stlplus::file_exists(prjCoord)) {
        LOG(ERROR) << "Can't read PROJCS database: " << prjCoord;
        return false;
    }

    // 加载地理坐标系
    if (!insight::parseCoordinates(m_geoCoordinates, geoCoord)) {
        LOG(ERROR) << "Failed to parse GEOGCS database from: " << geoCoord;
        return false;
    }
    LOG(INFO) << "Loaded " << m_geoCoordinates.size() << " geographic coordinate systems";

    // 加载投影坐标系
    if (!insight::parseCoordinates(m_projCoordinates, prjCoord)) {
        LOG(ERROR) << "Failed to parse PROJCS database from: " << prjCoord;
        return false;
    }
    LOG(INFO) << "Loaded " << m_projCoordinates.size() << " projected coordinate systems";

    // 合并到统一列表
    m_allCoordinates.reserve(m_geoCoordinates.size() + m_projCoordinates.size());
    m_allCoordinates.insert(m_allCoordinates.end(),
                           m_geoCoordinates.begin(), m_geoCoordinates.end());
    m_allCoordinates.insert(m_allCoordinates.end(),
                           m_projCoordinates.begin(), m_projCoordinates.end());

    m_loaded = true;
    LOG(INFO) << "Successfully loaded total " << m_allCoordinates.size() 
             << " coordinate systems";
    return true;
}

const std::vector<Coordinate>& UISystemConfig::getGeoCoordinates() const
{
    return m_geoCoordinates;
}

const std::vector<Coordinate>& UISystemConfig::getProjCoordinates() const
{
    return m_projCoordinates;
}

const std::vector<Coordinate>& UISystemConfig::getAllCoordinates() const
{
    return m_allCoordinates;
}

Coordinate UISystemConfig::findByEPSG(int epsg) const
{
    // 在合并列表中查找
    for (const auto& coord : m_allCoordinates) {
        bool ok = false;
        if (coord.EPSG(&ok) == epsg && ok) {
            return coord;
        }
    }

    // 未找到，返回空坐标系
    return Coordinate();
}

std::vector<Coordinate> UISystemConfig::searchByKeyword(const std::string& keyword) const
{
    std::vector<Coordinate> results;

    if (keyword.empty()) {
        return results;
    }

    // 将关键字转换为小写用于比较
    std::string lowerKeyword = keyword;
    std::transform(lowerKeyword.begin(), lowerKeyword.end(),
                   lowerKeyword.begin(), ::tolower);

    // 在所有坐标系中搜索
    for (const auto& coord : m_allCoordinates) {
        // 搜索 EPSG 代码
        bool ok = false;
        int epsg = coord.EPSG(&ok);
        if (ok && std::to_string(epsg).find(keyword) != std::string::npos) {
            results.push_back(coord);
            continue;
        }

        // 搜索坐标系名称
        std::string lowerName = coord.CoordinateName;
        std::transform(lowerName.begin(), lowerName.end(),
                       lowerName.begin(), ::tolower);
        if (lowerName.find(lowerKeyword) != std::string::npos) {
            results.push_back(coord);
            continue;
        }

        // 搜索 EPSG 名称
        std::string lowerEPSGName = coord.EPSGName;
        std::transform(lowerEPSGName.begin(), lowerEPSGName.end(),
                       lowerEPSGName.begin(), ::tolower);
        if (lowerEPSGName.find(lowerKeyword) != std::string::npos) {
            results.push_back(coord);
        }
    }

    return results;
}

}  // namespace ui
}  // namespace insight
