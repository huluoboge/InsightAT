/**
 * @file ProjectCoordinateWidget.cpp
 * @brief 项目坐标系选择 Widget 实现
 */

#include "ProjectCoordinateWidget.h"
#include "SpatialReferenceDialog.h"
#include "../../Common/Coordinates.h"
#include <QRadioButton>
#include <QLineEdit>
#include <QPushButton>
#include <QMessageBox>
#include <glog/logging.h>

namespace insight {
namespace ui {

ProjectCoordinateWidget::ProjectCoordinateWidget(QWidget* parent)
    : QWidget(parent) {
    ui.setupUi(this);
    initializeUI();
}

ProjectCoordinateWidget::~ProjectCoordinateWidget() {
}

void ProjectCoordinateWidget::initializeUI() {
    // 连接信号槽
    connect(ui.pushButton_gps, &QPushButton::clicked, this, &ProjectCoordinateWidget::onSelectCoord);
    connect(ui.radioButton_5, &QRadioButton::clicked, this, &ProjectCoordinateWidget::onCoordinateSystemModeChanged);
    connect(ui.radioButton_6, &QRadioButton::clicked, this, &ProjectCoordinateWidget::onCoordinateSystemModeChanged);

    // 初始状态：LOCAL 坐标系选中
    updateUIState();
}

void ProjectCoordinateWidget::onSelectCoord() {
    // 打开 SpatialReferenceDialog
    SpatialReferenceDialog dialog(this);
    
    if (dialog.exec() == QDialog::Accepted) {
        const auto& coord = dialog.SelectCoordinate();
        
        if (!coord.WKT.empty()) {
            m_selectedCoordinate.name = coord.CoordinateName;
            m_selectedCoordinate.epsg = coord.EPSGName;
            m_selectedCoordinate.wkt = coord.WKT;
            
            // 更新 UI 显示
            ui.lineEdit_gpsCoordName->setText(QString::fromStdString(m_selectedCoordinate.name));
            ui.lineEdit_gpsCoordEPSG->setText(QString::fromStdString(m_selectedCoordinate.epsg));
            
            LOG(INFO) << "Selected coordinate system: " << m_selectedCoordinate.name
                     << " (" << m_selectedCoordinate.epsg << ")";
        }
    }
}

void ProjectCoordinateWidget::onCoordinateSystemModeChanged() {
    updateUIState();
}

void ProjectCoordinateWidget::updateUIState() {
    bool isLocalMode = ui.radioButton_5->isChecked();
    
    if (isLocalMode) {
        // LOCAL 坐标系模式
        ui.lineEdit_gpsCoordName->setEnabled(false);
        ui.lineEdit_gpsCoordEPSG->setEnabled(false);
        ui.pushButton_gps->setEnabled(false);
    } else {
        // 大地坐标系模式
        ui.lineEdit_gpsCoordName->setEnabled(true);
        ui.lineEdit_gpsCoordEPSG->setEnabled(true);
        ui.pushButton_gps->setEnabled(true);
    }
}

insight::database::CoordinateSystem ProjectCoordinateWidget::GetCoordinateSystem() const {
    insight::database::CoordinateSystem coordSys;

    if (ui.radioButton_5->isChecked()) {
        // LOCAL 坐标系模式
        coordSys.type = insight::database::CoordinateSystem::Type::kLocal;
        
        // 设置参考点和原点信息到 definition 中
        // 格式：LOCAL|lat,lon,alt|x,y,z
        std::string definition = "LOCAL|" +
            std::to_string(m_localParams.ref_lat) + "," +
            std::to_string(m_localParams.ref_lon) + "," +
            std::to_string(m_localParams.ref_alt) + "|" +
            std::to_string(m_localParams.origin_x) + "," +
            std::to_string(m_localParams.origin_y) + "," +
            std::to_string(m_localParams.origin_z);
        
        coordSys.definition = definition;
        
        // 如果有参考点信息，设置到 reference
        if (m_localParams.ref_lat != 0.0 || m_localParams.ref_lon != 0.0) {
            insight::database::CoordinateSystem::ReferencePoint ref;
            ref.lat = m_localParams.ref_lat;
            ref.lon = m_localParams.ref_lon;
            ref.alt = m_localParams.ref_alt;
            coordSys.reference = ref;
        }
        
        // 如果有原点信息，设置到 origin
        if (m_localParams.origin_x != 0.0 || m_localParams.origin_y != 0.0 || m_localParams.origin_z != 0.0) {
            insight::database::CoordinateSystem::Origin orig;
            orig.x = m_localParams.origin_x;
            orig.y = m_localParams.origin_y;
            orig.z = m_localParams.origin_z;
            coordSys.origin = orig;
        }
    } else {
        // 大地坐标系模式
        if (m_selectedCoordinate.wkt.empty()) {
            LOG(WARNING) << "No coordinate system selected";
            coordSys.type = insight::database::CoordinateSystem::Type::kLocal;
            return coordSys;
        }
        
        // 确定坐标系类型
        if (!m_selectedCoordinate.epsg.empty()) {
            // EPSG 代码优先
            coordSys.type = insight::database::CoordinateSystem::Type::kEPSG;
            coordSys.definition = m_selectedCoordinate.epsg;
        } else {
            // 使用 WKT
            coordSys.type = insight::database::CoordinateSystem::Type::kWKT;
            coordSys.definition = m_selectedCoordinate.wkt;
        }
    }

    return coordSys;
}

void ProjectCoordinateWidget::SetCoordinateSystem(const insight::database::CoordinateSystem& coordSys) {
    if (coordSys.type == insight::database::CoordinateSystem::Type::kLocal) {
        // 解析 LOCAL 坐标系
        ui.radioButton_5->setChecked(true);
        
        // 从 definition 解析参数（格式：LOCAL|lat,lon,alt|x,y,z）
        if (!coordSys.definition.empty()) {
            try {
                std::string def = coordSys.definition;
                size_t pos1 = def.find('|');
                size_t pos2 = def.rfind('|');
                
                if (pos1 != std::string::npos && pos2 != std::string::npos && pos1 < pos2) {
                    std::string refPart = def.substr(pos1 + 1, pos2 - pos1 - 1);
                    std::string origPart = def.substr(pos2 + 1);
                    
                    // 解析参考点
                    size_t comma1 = refPart.find(',');
                    size_t comma2 = refPart.rfind(',');
                    if (comma1 != std::string::npos && comma2 != std::string::npos) {
                        m_localParams.ref_lat = std::stod(refPart.substr(0, comma1));
                        m_localParams.ref_lon = std::stod(refPart.substr(comma1 + 1, comma2 - comma1 - 1));
                        m_localParams.ref_alt = std::stod(refPart.substr(comma2 + 1));
                    }
                    
                    // 解析原点
                    comma1 = origPart.find(',');
                    comma2 = origPart.rfind(',');
                    if (comma1 != std::string::npos && comma2 != std::string::npos) {
                        m_localParams.origin_x = std::stod(origPart.substr(0, comma1));
                        m_localParams.origin_y = std::stod(origPart.substr(comma1 + 1, comma2 - comma1 - 1));
                        m_localParams.origin_z = std::stod(origPart.substr(comma2 + 1));
                    }
                }
            } catch (const std::exception& e) {
                LOG(WARNING) << "Failed to parse LOCAL coordinate system: " << e.what();
            }
        }
        
        if (coordSys.origin) {
            m_localParams.origin_x = coordSys.origin->x;
            m_localParams.origin_y = coordSys.origin->y;
            m_localParams.origin_z = coordSys.origin->z;
        }
        
        if (coordSys.reference) {
            m_localParams.ref_lat = coordSys.reference->lat;
            m_localParams.ref_lon = coordSys.reference->lon;
            m_localParams.ref_alt = coordSys.reference->alt;
        }
    } else {
        // 大地坐标系模式
        ui.radioButton_6->setChecked(true);
        
        m_selectedCoordinate.wkt = coordSys.definition;
        
        // 尝试提取 EPSG 代码
        if (coordSys.type == insight::database::CoordinateSystem::Type::kEPSG) {
            m_selectedCoordinate.epsg = coordSys.definition;
        }
        
        // 更新 UI 显示
        ui.lineEdit_gpsCoordName->setText("(loaded)");
        ui.lineEdit_gpsCoordEPSG->setText(QString::fromStdString(m_selectedCoordinate.epsg));
    }
    
    updateUIState();
}

bool ProjectCoordinateWidget::IsValid() const {
    if (ui.radioButton_5->isChecked()) {
        // LOCAL 坐标系总是有效的（使用默认值）
        return true;
    } else {
        // 大地坐标系需要选择
        return !m_selectedCoordinate.wkt.empty();
    }
}

}  // namespace ui
}  // namespace insight
