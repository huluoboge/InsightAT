#ifndef INSIGHT_CAMERA_SENSOR_DATABASE_H
#define INSIGHT_CAMERA_SENSOR_DATABASE_H

#include <string>
#include <vector>
#include <map>
#include <memory>

namespace insight {
namespace database {

/**
 * @struct SensorInfo
 * @brief 传感器信息
 */
struct SensorInfo {
    std::string manufacturer;
    std::string model;
    double sensor_width_mm = 0.0;
};

/**
 * @class CameraSensorDatabase
 * @brief 相机传感器数据库单例 - 加载和查询传感器尺寸
 */
class CameraSensorDatabase {
public:
    static CameraSensorDatabase& instance();

    /**
     * @brief 加载数据库文件
     * @param[in] dbPath 数据库文件路径 (camera_sensor_database.txt)
     * @return 成功返回true
     */
    bool load(const std::string& dbPath);

    /**
     * @brief 根据制造商和型号查询传感器宽度
     * @param[in] manufacturer 制造商 (Brand)
     * @param[in] model 型号
     * @param[out] widthMm 传感器宽度 (mm)
     * @return 找到返回true
     */
    bool querySensorWidth(const std::string& manufacturer, const std::string& model, double& widthMm) const;

    /**
     * @brief 获取所有已加载的传感器信息
     */
    const std::vector<SensorInfo>& allSensors() const { return m_sensors; }

private:
    CameraSensorDatabase() = default;
    ~CameraSensorDatabase() = default;
    CameraSensorDatabase(const CameraSensorDatabase&) = delete;
    CameraSensorDatabase& operator=(const CameraSensorDatabase&) = delete;

    std::vector<SensorInfo> m_sensors;
    // 使用 map 加速查询: "manufacturer|model" -> sensor_width
    std::map<std::string, double> m_lookupTable;
    
    std::string normalizeString(const std::string& s) const;
};

} // namespace database
} // namespace insight

#endif // INSIGHT_CAMERA_SENSOR_DATABASE_H
