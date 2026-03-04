/**
 * @file  CoordinateSystemConfigDialog.h
 * @brief 坐标系配置对话框，管理框架与 OK/Cancel，返回 CoordinateSystem。
 */

#ifndef INSIGHT_UI_DIALOGS_COORDINATESYSTEMCONFIGDIALOG_H
#define INSIGHT_UI_DIALOGS_COORDINATESYSTEMCONFIGDIALOG_H

#include "database/database_types.h"

#include <QDialog>

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
  database::CoordinateSystem get_coordinate_system() const;

  /**
   * 加载已有坐标系配置到对话框
   *
   * @param[in] coord_sys 要加载的坐标系
   */
  void set_coordinate_system(const database::CoordinateSystem& coord_sys);

private slots:
  /**
   * 验证状态变化槽
   * 控制 OK 按钮的启用/禁用状态
   *
   * @param[in] valid 表单是否有效
   */
  void on_validation_changed(bool valid);

private:
  void initialize_ui();

  CoordinateSystemConfigWidget* m_configWidget = nullptr;
};

}  // namespace ui
}  // namespace insight

#endif  // INSIGHT_UI_DIALOGS_COORDINATESYSTEMCONFIGDIALOG_H
