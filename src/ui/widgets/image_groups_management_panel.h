/**
 * @file  ImageGroupsManagementPanel.h
 * @brief 图像分组管理面板，显示分组列表与新建/导入/编辑操作。
 */

#pragma once

#include "database/database_types.h"

#include <memory>

#include <QLabel>
#include <QPushButton>
#include <QTableWidget>
#include <QWidget>

namespace insight {
namespace ui {
class ProjectDocument;   // 前向声明，来自 models/ProjectDocument.h
class ImageEditorDialog; // 前向声明

namespace widgets {

/**
 * @brief 图像分组管理面板 - 显示在右侧
 *
 * 功能：
 * - 显示所有分组的列表（QTableWidget）
 * - [New Group] 按钮自动生成分组
 * - [Import Images] 按钮打开图像编辑对话框
 * - [编辑] 按钮打开分组详情编辑对话框
 * - [删除] 按钮删除分组（确认对话框）
 */
class ImageGroupsManagementPanel : public QWidget {
  Q_OBJECT

public:
  explicit ImageGroupsManagementPanel(QWidget* parent = nullptr);
  ~ImageGroupsManagementPanel();

  void set_project_document(ProjectDocument* doc);

  void refresh_group_list();

signals:
  void edit_group_requested(database::ImageGroup* group);

private slots:
  void on_new_group();
  void on_import_images();
  void on_edit_group();
  void on_delete_group();
  void on_project_changed();
  void on_image_group_added(uint32_t group_id);
  void on_image_group_removed(uint32_t group_id);
  void on_image_group_changed(uint32_t group_id);

private:
  void initialize_ui();
  void connect_signals();
  void update_table_row(const database::ImageGroup* group, int row);
  std::string get_next_group_name();

  // ─── UI 组件 ───
  QLabel* m_titleLabel;
  QPushButton* m_newGroupButton;
  QTableWidget* m_groupTable;

  // ─── 对话框 ───
  std::unique_ptr<ImageEditorDialog> m_imageEditorDialog;
  uint32_t m_currentImportGroupId = 0; // 当前导入的分组ID

  // ─── 数据指针 ───
  ProjectDocument* m_projectDocument; // 无所有权
};

}  // namespace widgets
}  // namespace ui
}  // namespace insight
