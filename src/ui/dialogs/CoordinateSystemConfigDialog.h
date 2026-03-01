#ifndef INSIGHT_UI_DIALOGS_COORDINATESYSTEMCONFIGDIALOG_H
#define INSIGHT_UI_DIALOGS_COORDINATESYSTEMCONFIGDIALOG_H

#include <QDialog>
#include "database/database_types.h"

namespace insight {
namespace ui {

// Forward declarations
class CoordinateSystemConfigWidget;

/**
 * @class CoordinateSystemConfigDialog
 * @brief 坐标系配置对话框 - 容器框架
 * 
 * 职责：
 * - 管理对话框框架和 OK/Cancel 按钮
 * - 连接 Widget 的验证信号到 OK 按钮
 * - 初始状态禁用 OK 按钮，直到表单有效
 * - 返回用户配置的 CoordinateSystem 对象
 */
class CoordinateSystemConfigDialog : public QDialog {
    Q_OBJECT

public:
    explicit CoordinateSystemConfigDialog(QWidget* parent = nullptr);
    ~CoordinateSystemConfigDialog() = default;

    /**
     * 获取用户配置的坐标系对象
     * 
     * @return 完整的 CoordinateSystem 对象（仅在 OK 点击后有效）
     */
    database::CoordinateSystem GetCoordinateSystem() const;

    /**
     * 加载已有坐标系配置到对话框
     * 
     * @param[in] coordSys 要加载的坐标系
     */
    void SetCoordinateSystem(const database::CoordinateSystem& coordSys);

private slots:
    /**
     * 验证状态变化槽
     * 控制 OK 按钮的启用/禁用状态
     * 
     * @param[in] valid 表单是否有效
     */
    void onValidationChanged(bool valid);

private:
    void initializeUI();

    CoordinateSystemConfigWidget* m_configWidget = nullptr;
};

}  // namespace ui
}  // namespace insight

#endif  // INSIGHT_UI_DIALOGS_COORDINATESYSTEMCONFIGDIALOG_H
