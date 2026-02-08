/**
 * @file ProjectCoordinateDialog.h
 * @brief 项目坐标系选择对话框
 * 
 * 在项目创建/编辑时使用，提供坐标系选择和配置
 */

#ifndef UI_DIALOGS_PROJECTCOORDINATEDIALOG_H
#define UI_DIALOGS_PROJECTCOORDINATEDIALOG_H

#include <QDialog>

namespace insight {
namespace database {
struct CoordinateSystem;
}

namespace ui {

class ProjectCoordinateWidget;

/**
 * @class ProjectCoordinateDialog
 * @brief 项目坐标系选择对话框
 */
class ProjectCoordinateDialog : public QDialog {
    Q_OBJECT

public:
    explicit ProjectCoordinateDialog(QWidget* parent = nullptr);
    ~ProjectCoordinateDialog();

    /**
     * 获取选择的坐标系
     * 
     * @return database::CoordinateSystem 对象
     */
    insight::database::CoordinateSystem GetCoordinateSystem() const;

    /**
     * 设置初始的坐标系值
     * 
     * @param[in] coordSys 坐标系对象
     */
    void SetCoordinateSystem(const insight::database::CoordinateSystem& coordSys);

private:
    /**
     * 初始化 UI
     */
    void initializeUI();

    ProjectCoordinateWidget* m_coordWidget = nullptr;
};

}  // namespace ui
}  // namespace insight

#endif  // UI_DIALOGS_PROJECTCOORDINATEDIALOG_H
