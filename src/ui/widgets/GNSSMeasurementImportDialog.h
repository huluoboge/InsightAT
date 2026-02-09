#ifndef GNSSMEASUREMENTIMPORTDIALOG_H
#define GNSSMEASUREMENTIMPORTDIALOG_H

#include "GPSPointsWizardDialog.h"
#include "database/database_types.h"
#include <vector>

namespace insight {
namespace ui {
namespace widgets {

/**
 * @class GNSSMeasurementImportDialog
 * @brief GNSS 测量数据导入对话框
 * 
 * 继承自 GPSPointsWizardDialog，专门用于导入 GNSS（GPS/RTK）测量数据
 * 
 * 支持的字段组合：
 * 1. 经纬度格式：Latitude, Longitude, Height
 * 2. 投影坐标格式：X, Y, Z
 * 3. 可选：Uncertainty_X, Uncertainty_Y, Uncertainty_Z（或 Covariance_XX 等）
 * 4. 可选：Omega, Phi, Kappa（旋转角度）
 * 5. 可选：其他 GNSS 特定字段（卫星数、HDOP 等）
 * 
 * 坐标系和旋转的具体含义由项目的坐标系配置决定。
 * 这个对话框只负责"读取数据"，不负责"解释数据的含义"。
 */
class GNSSMeasurementImportDialog : public GPSPointsWizardDialog {
    Q_OBJECT

public:
    explicit GNSSMeasurementImportDialog(QWidget* parent = nullptr);
    ~GNSSMeasurementImportDialog() = default;

    /**
     * @brief 设置导入文件并自动触发预览
     * @param fileFullPath 完整文件路径
     */
    void setFile(const QString& fileFullPath);

    /**
     * @brief 获取导入的 GNSS 测量数据
     * @return GNSS 测量结构体数组，按导入顺序排列
     * 
     * 返回的数据已按向导匹配策略（按名称或按序列）排序，
     * 可直接应用到对应的 Image 对象
     */
    std::vector<database::Measurement::GNSSMeasurement> getGNSSMeasurements();

    /**
     * @brief 设置坐标类型
     * @param useGeographic true 表示使用经纬度，false 表示使用 XYZ 投影坐标
     */
    void setCoordinateType(bool useGeographic);

    /**
     * @brief 设置是否导入旋转数据
     * @param importRotation true 表示导入旋转角度
     */
    void setImportRotation(bool importRotation);

    /**
     * @brief 设置是否使用统一协方差
     * @param useUniform true 表示使用统一协方差值
     */
    void setUseUniformCovariance(bool useUniform);

    /**
     * @brief 设置统一协方差值
     * @param sigma_xy XY方向标准差（米）
     * @param sigma_z Z方向标准差（米）
     */
    void setUniformCovariance(double sigma_xy, double sigma_z);

protected:
    /**
     * @brief 重写字段配置 - GNSS 特定的字段列表
     */
    FieldConfiguration getFieldConfiguration() const override;

    /**
     * @brief 验证字段数据
     */
    bool checkFieldData(int rowFrom, const std::vector<int>& fieldIndex) override;

    /**
     * @brief 获取字段名称
     */
    QList<QString> fieldNames() const override;

private:
    /**
     * @brief 解析单行 GNSS 数据
     */
    database::Measurement::GNSSMeasurement parseLine(const QStringList& fields,
                                                   const std::vector<int>& fieldIndex);

    bool m_useGeographic = true;           ///< true: Lat/Lon, false: X/Y/Z
    bool m_importRotation = false;         ///< 是否导入旋转数据
    bool m_useUniformCovariance = false;   ///< 是否使用统一协方差
    double m_uniformSigmaXY = 1.0;         ///< XY统一标准差（米）
    double m_uniformSigmaZ = 2.0;          ///< Z统一标准差（米）
};

} // namespace widgets
} // namespace ui
} // namespace insight

#endif // GNSSMEASUREMENTIMPORTDIALOG_H
