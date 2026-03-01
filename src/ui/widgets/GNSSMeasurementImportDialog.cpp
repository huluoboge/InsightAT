#include "GNSSMeasurementImportDialog.h"
#include "Common/string_utils.h"
#include <glog/logging.h>
#include <cmath>
#include <QAbstractItemView>

namespace insight {
namespace ui {
namespace widgets {

GNSSMeasurementImportDialog::GNSSMeasurementImportDialog(QWidget* parent)
    : GPSPointsWizardDialog(parent),
      m_useGeographic(true),
      m_importRotation(false)
{
    setWindowTitle("Import GNSS Measurements");
    
    // Set table to single-select mode (ExtendedSelection allows Ctrl/Shift for multi-select)
    ui.tableView->setSelectionMode(QAbstractItemView::ExtendedSelection);
    ui.tableView->setSelectionBehavior(QAbstractItemView::SelectRows);
}

void GNSSMeasurementImportDialog::setFile(const QString& fileFullPath)
{
    // Call base class to load the file
    GPSPointsWizardDialog::setFile(fileFullPath);
    
    // Automatically trigger preview to show data in the table
    // Default to Tab separation if not already checked
    if (!ui.checkBox_tab->isChecked() && !ui.checkBox_semicolon->isChecked() &&
        !ui.checkBox_comma->isChecked() && !ui.checkBox_space->isChecked() &&
        !ui.checkBox_other->isChecked()) {
        ui.checkBox_tab->setChecked(true);
    }
    
    // Trigger preview to populate the table
    checkEnablePreview();
}

FieldConfiguration GNSSMeasurementImportDialog::getFieldConfiguration() const
{
    FieldConfiguration config;

    // 必需字段：位置信息
    if (m_useGeographic) {
        config.requiredFields << "Latitude" << "Longitude" << "Height";
    } else {
        config.requiredFields << "X" << "Y" << "Z";
    }

    // 可选字段
    config.optionalFields << "Uncertainty_X" << "Uncertainty_Y" << "Uncertainty_Z"
                          << "Covariance_XX" << "Covariance_YY" << "Covariance_ZZ"
                          << "Covariance_XY" << "Covariance_XZ" << "Covariance_YZ"
                          << "HDOP" << "VDOP" << "NumSatellites"
                          << "Accuracy_X" << "Accuracy_Y" << "Accuracy_Z";

    // 如果需要导入旋转数据
    if (m_importRotation) {
        config.optionalFields << "Omega" << "Phi" << "Kappa"
                              << "Roll" << "Pitch" << "Yaw";
    }

    return config;
}

QList<QString> GNSSMeasurementImportDialog::fieldNames() const
{
    return getFieldConfiguration().getAllFields();
}

bool GNSSMeasurementImportDialog::checkFieldData(int rowFrom, const std::vector<int>& fieldIndex)
{
    // 验证必需字段是否存在
    FieldConfiguration config = getFieldConfiguration();
    
    for (int i = 0; i < config.requiredFields.size(); ++i) {
        if (fieldIndex[i] == -1) {
            LOG(WARNING) << "Required field not found: " << config.requiredFields[i].toStdString();
            return false;
        }
    }

    // 验证数据有效性
    GPSPointsDocument* doc = getDoc();
    if (!doc || rowFrom >= doc->m_tableData.count()) {
        return false;
    }

    // 检查第一行的必需字段是否为有效数字
    for (int i = 0; i < config.requiredFields.size(); ++i) {
        int col = fieldIndex[i];
        if (col >= 0 && rowFrom < doc->m_tableData.count()) {
            const QStringList& row = doc->m_tableData[rowFrom];
            if (col < row.count()) {
                bool ok;
                row[col].toDouble(&ok);
                if (!ok) {
                    LOG(WARNING) << "Invalid numeric data in field: "
                               << config.requiredFields[i].toStdString();
                    return false;
                }
            }
        }
    }

    return true;
}

std::vector<database::Measurement::GNSSMeasurement> GNSSMeasurementImportDialog::getGNSSMeasurements()
{
    std::vector<database::Measurement::GNSSMeasurement> measurements;

    // 获取向导中的数据
    GPSPointsDocument* doc = getDoc();
    if (!doc || doc->m_tableData.isEmpty()) {
        return measurements;
    }

    // 获取字段索引
    std::vector<int> fieldIndex;
    int rowFrom = 0;
    getFieldIndex(rowFrom, fieldIndex);

    FieldConfiguration config = getFieldConfiguration();
    int requiredFieldCount = config.requiredFields.size();

    // 检查是否至少有一个必需字段被找到
    bool hasRequiredField = false;
    for (int i = 0; i < requiredFieldCount; ++i) {
        if (fieldIndex[i] >= 0) {
            hasRequiredField = true;
            break;
        }
    }

    if (!hasRequiredField) {
        LOG(WARNING) << "No required GNSS fields found in import data";
        return measurements;
    }

    // 解析所有数据行
    for (int rowIdx = rowFrom; rowIdx < doc->m_tableData.count(); ++rowIdx) {
        const QStringList& row = doc->m_tableData[rowIdx];
        if (row.isEmpty()) {
            continue;
        }

        try {
            database::Measurement::GNSSMeasurement gnss = parseLine(row, fieldIndex);
            measurements.push_back(gnss);
        } catch (const std::exception& e) {
            LOG(WARNING) << "Error parsing GNSS data at row " << (rowIdx + 1) 
                        << ": " << e.what();
            continue;
        }
    }

    LOG(INFO) << "Successfully imported " << measurements.size() << " GNSS measurements";
    return measurements;
}

void GNSSMeasurementImportDialog::setCoordinateType(bool useGeographic)
{
    m_useGeographic = useGeographic;
}

void GNSSMeasurementImportDialog::setImportRotation(bool importRotation)
{
    m_importRotation = importRotation;
}

void GNSSMeasurementImportDialog::setUseUniformCovariance(bool useUniform)
{
    m_useUniformCovariance = useUniform;
}

void GNSSMeasurementImportDialog::setUniformCovariance(double sigma_xy, double sigma_z)
{
    m_uniformSigmaXY = sigma_xy;
    m_uniformSigmaZ = sigma_z;
}

database::Measurement::GNSSMeasurement GNSSMeasurementImportDialog::parseLine(const QStringList& fields,
                                                                              const std::vector<int>& fieldIndex)
{
    database::Measurement::GNSSMeasurement gnss;
    
    FieldConfiguration config = getFieldConfiguration();
    
    // 解析位置信息
    if (m_useGeographic) {
        // Latitude, Longitude, Height
        int latIdx = fieldIndex[0];  // Latitude
        int lonIdx = fieldIndex[1];  // Longitude
        int heightIdx = fieldIndex[2];  // Height
        
        if (latIdx >= 0 && lonIdx >= 0 && heightIdx >= 0 &&
            latIdx < fields.count() && lonIdx < fields.count() && heightIdx < fields.count()) {
            gnss.y = fields[latIdx].toDouble();  // Latitude
            gnss.x = fields[lonIdx].toDouble();  // Longitude
            gnss.z = fields[heightIdx].toDouble();  // Height
        }
    } else {
        // X, Y, Z
        int xIdx = fieldIndex[0];  // X
        int yIdx = fieldIndex[1];  // Y
        int zIdx = fieldIndex[2];  // Z
        
        if (xIdx >= 0 && yIdx >= 0 && zIdx >= 0 &&
            xIdx < fields.count() && yIdx < fields.count() && zIdx < fields.count()) {
            gnss.x = fields[xIdx].toDouble();
            gnss.y = fields[yIdx].toDouble();
            gnss.z = fields[zIdx].toDouble();
        }
    }

    // 解析协方差
    if (m_useUniformCovariance) {
        // 使用统一协方差值
        gnss.cov_xx = m_uniformSigmaXY * m_uniformSigmaXY;
        gnss.cov_yy = m_uniformSigmaXY * m_uniformSigmaXY;
        gnss.cov_zz = m_uniformSigmaZ * m_uniformSigmaZ;
        gnss.cov_xy = gnss.cov_xz = gnss.cov_yz = 0.0;
    } else {
        // 从文件读取协方差信息（如果可用）
        gnss.cov_xx = 1.0;
        gnss.cov_yy = 1.0;
        gnss.cov_zz = 2.0;
        gnss.cov_xy = gnss.cov_xz = gnss.cov_yz = 0.0;
    }

    return gnss;
}

} // namespace widgets
} // namespace ui
} // namespace insight
