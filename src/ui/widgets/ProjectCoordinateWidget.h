/**
 * @file ProjectCoordinateWidget.h
 * @brief 项目坐标系选择 Widget
 * 
 * 用于在项目创建/编辑时选择和配置坐标系
 */

#ifndef UI_WIDGETS_PROJECTCOORDINATEWIDGET_H
#define UI_WIDGETS_PROJECTCOORDINATEWIDGET_H

#include <QWidget>
#include "../../database/database_types.h"
#include "ui_ProjectCoordinateWidget.h"

namespace insight {
namespace database {
struct CoordinateSystem;
}

namespace ui {

/**
 * @class ProjectCoordinateWidget
 * @brief 项目坐标系配置 Widget
 * 
 * 支持两种模式：
 * 1. Local Coordinate System（本地坐标系）
 *    - 需要指定参考点（纬度、经度、高度）
 *    - 需要指定原点（X、Y、Z）
 * 
 * 2. Geodetic Coordinate System（大地坐标系）
 *    - 通过 SpatialReferenceDialog 选择标准坐标系（EPSG/WKT）
 */
class ProjectCoordinateWidget : public QWidget {
    Q_OBJECT

public:
    explicit ProjectCoordinateWidget(QWidget* parent = nullptr);
    ~ProjectCoordinateWidget();

    /**
     * 获取选择的坐标系
     * 
     * @return 转换后的 database::CoordinateSystem 对象
     */
    insight::database::CoordinateSystem GetCoordinateSystem() const;

    /**
     * 设置初始的坐标系值
     * 
     * @param[in] coordSys 坐标系对象
     */
    void SetCoordinateSystem(const insight::database::CoordinateSystem& coordSys);

    /**
     * 验证坐标系配置的有效性
     * 
     * @return 有效返回 true
     */
    bool IsValid() const;

private slots:
    /**
     * 选择大地坐标系（GPS/EPSG）
     */
    void onSelectCoord();

    /**
     * 切换坐标系模式
     */
    void onCoordinateSystemModeChanged();

private:
    /**
     * 初始化 UI（在 setupUi 之后调用）
     */
    void initializeUI();

    /**
     * 更新 UI 状态（根据选择的坐标系模式）
     */
    void updateUIState();

    // UI
    Ui::ProjectCoordinateWidget ui;

    // 当前选择的坐标
    struct {
        std::string name;           ///< 坐标系名称
        std::string epsg;           ///< EPSG 代码
        std::string wkt;            ///< WKT 定义
    } m_selectedCoordinate;

    // LOCAL 坐标系参数
    struct {
        double ref_lat = 0.0;       ///< 参考点纬度
        double ref_lon = 0.0;       ///< 参考点经度
        double ref_alt = 0.0;       ///< 参考点高度

        double origin_x = 0.0;      ///< 原点 X
        double origin_y = 0.0;      ///< 原点 Y
        double origin_z = 0.0;      ///< 原点 Z
    } m_localParams;
};

}  // namespace ui
}  // namespace insight

#endif  // UI_WIDGETS_PROJECTCOORDINATEWIDGET_H
