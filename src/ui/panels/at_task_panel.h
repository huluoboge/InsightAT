/**
 * @file ATTaskPanel.h
 * @brief AT 任务编辑面板
 *
 * 非模态 QWidget，用于编辑 AT Task 的详细信息。包含三个 Tab：
 * - Input Data：显示 InputSnapshot 中的图像和相机数据（嵌入 ImageEditorDialog）
 * - Optimization：相机参数优化配置（CameraOptimizationPanel）
 * - Export：导出选项和按钮
 *
 * 底部有 Save/Export/Close 按钮。
 */

#ifndef UI_PANELS_ATTASKPANEL_H
#define UI_PANELS_ATTASKPANEL_H

#include <QString>
#include <QWidget>
#include <memory>

class QTabWidget;
class QGroupBox;
class QPushButton;
class QLineEdit;
class QLabel;

namespace insight {
namespace ui {

class ProjectDocument;

/**
 * @class ATTaskPanel
 * @brief 非模态 AT Task 编辑面板
 */
class ATTaskPanel : public QWidget {
  Q_OBJECT

public:
  /**
   * 构造函数
   *
   * @param[in] document 项目文档指针
   * @param[in] parent 父部件
   */
  explicit ATTaskPanel(ProjectDocument* document, QWidget* parent = nullptr);

  ~ATTaskPanel();

  /**
   * 加载指定 AT Task 进行编辑
   *
   * @param[in] task_id 任务 UUID
   * @return true 如果成功加载
   */
  bool load_task(const std::string& task_id);

  std::string get_current_task_id() const;

public slots:
  void on_export_clicked();
  void on_run_sift_gpu_clicked();
  void on_task_name_changed();

private:
  void init_ui();
  void refresh_ui();
  void save_task();

private:
  ProjectDocument* m_document;
  std::string m_currentTaskId;

  // UI 组件
  QTabWidget* m_tabWidget;
  QWidget* m_inputDataTab;    ///< Tab 1：Input Data
  QWidget* m_optimizationTab; ///< Tab 2：Optimization
  QWidget* m_exportTab;       ///< Tab 3：Export

  // 顶部信息显示
  QLineEdit* m_taskNameEdit;
  QLabel* m_taskIdLabel; ///< 显示任务的完整 UUID
  QLabel* m_parentTaskLabel;
  QLabel* m_statusLabel;

  // Export Tab 中的按钮
  QPushButton* m_exportButton = nullptr;
  QPushButton* m_siftGPUButton = nullptr;
};

}  // namespace ui
}  // namespace insight

#endif  // UI_PANELS_ATTASKPANEL_H
