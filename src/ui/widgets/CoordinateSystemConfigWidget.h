#ifndef INSIGHT_UI_WIDGETS_COORDINATESYSTEMCONFIGWIDGET_H
#define INSIGHT_UI_WIDGETS_COORDINATESYSTEMCONFIGWIDGET_H

#include <QWidget>
#include <QRadioButton>
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QStackedWidget>
#include <QPlainTextEdit>
#include <memory>

#include "database/database_types.h"

namespace insight {
namespace ui {

// Forward declarations
class SpatialReferenceDialog;

/**
 * @class CoordinateSystemConfigWidget
 * @brief 坐标系配置小部件 - 支持 LOCAL/EPSG/ENU/WKT 四种模式
 * 
 * 功能：
 * - 动态 UI 切换（基于选中的坐标系类型）
 * - 实时表单验证（发出 validationChanged 信号）
 * - EPSG/WKT 支持选择对话框（SpatialReferenceDialog）
 * - 用户可编辑 EPSG/WKT 内容
 * - RotationConvention 选择（两选项：摄影测量/航空学）
 */
class CoordinateSystemConfigWidget : public QWidget {
    Q_OBJECT

public:
    explicit CoordinateSystemConfigWidget(QWidget* parent = nullptr);
    ~CoordinateSystemConfigWidget() = default;

    /**
     * 获取用户配置的坐标系对象
     * 
     * @return 完整的 CoordinateSystem 对象
     */
    database::CoordinateSystem GetCoordinateSystem() const;

    /**
     * 加载已有的坐标系配置到小部件
     * 
     * @param[in] coordSys 要加载的坐标系
     */
    void SetCoordinateSystem(const database::CoordinateSystem& coordSys);

    /**
     * 检查当前配置是否有效
     * 
     * @return 有效返回 true
     */
    bool IsValid() const;

signals:
    /**
     * 验证状态变化信号
     * 
     * @param[in] valid 表单是否有效
     */
    void validationChanged(bool valid);

private slots:
    // 坐标系类型选择
    void onCoordinateTypeChanged();
    
    // EPSG 模式
    void onEPSGBrowse();
    void onEPSGTextChanged();
    
    // ENU 模式
    void onENUReferenceChanged();
    void onENUOriginChanged();
    
    // WKT 模式
    void onWKTBrowse();
    void onWKTTextChanged();
    
    // RotationConvention 选择
    void onRotationConventionChanged();

private:
    void initializeUI();
    void connectSignals();
    void updateUIState();
    void validateForm();

    // 验证辅助方法
    bool validateLocalMode() const;
    bool validateEPSGMode() const;
    bool validateENUMode() const;
    bool validateWKTMode() const;

    // 坐标系类型选择
    QRadioButton* m_radioLocal = nullptr;
    QRadioButton* m_radioEPSG = nullptr;
    QRadioButton* m_radioENU = nullptr;
    QRadioButton* m_radioWKT = nullptr;

    // 动态内容区（使用 QStackedWidget）
    QStackedWidget* m_stackedWidget = nullptr;

    // LOCAL 页面（无内容）
    QWidget* m_pageLocal = nullptr;

    // EPSG 页面
    QWidget* m_pageEPSG = nullptr;
    QLineEdit* m_epsgEdit = nullptr;           // 只读显示
    QPushButton* m_epsgBrowseBtn = nullptr;
    QLabel* m_epsgErrorLabel = nullptr;

    // ENU 页面
    QWidget* m_pageENU = nullptr;
    QDoubleSpinBox* m_enuRefLatSpinBox = nullptr;
    QDoubleSpinBox* m_enuRefLonSpinBox = nullptr;
    QDoubleSpinBox* m_enuRefAltSpinBox = nullptr;
    QDoubleSpinBox* m_enuOriginXSpinBox = nullptr;
    QDoubleSpinBox* m_enuOriginYSpinBox = nullptr;
    QDoubleSpinBox* m_enuOriginZSpinBox = nullptr;
    QLabel* m_enuErrorLabel = nullptr;

    // WKT 页面
    QWidget* m_pageWKT = nullptr;
    QPlainTextEdit* m_wktEdit = nullptr;       // 可编辑
    QPushButton* m_wktBrowseBtn = nullptr;
    QLabel* m_wktErrorLabel = nullptr;

    // RotationConvention 选择
    QRadioButton* m_radioPhotogrammetry = nullptr;  // kOmegaPhiKappa
    QRadioButton* m_radioAerospace = nullptr;       // kYawPitchRoll

    // 内部状态
    bool m_isValid = false;
    database::CoordinateSystem::Type m_currentType = database::CoordinateSystem::Type::kLocal;
};

}  // namespace ui
}  // namespace insight

#endif  // INSIGHT_UI_WIDGETS_COORDINATESYSTEMCONFIGWIDGET_H
