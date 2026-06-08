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
#include <QList>

class QProcess;

class QTabWidget;
class QGroupBox;
class QPushButton;
class QLineEdit;
class QLabel;
class QCheckBox;
class QPlainTextEdit;
class QTableWidget;
class QProgressBar;

namespace insight {

class BundlerViewerWindow;
namespace database {
struct ATTask;
struct InputSnapshot;
}

namespace ui {

class ProjectDocument;
namespace dialogs {
class ImageGroupDetailPanel;
}  // namespace dialogs

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
  void on_task_name_changed();

private:
  void init_ui();
  void refresh_ui();
  void save_task();
  void refresh_input_data_tab(const insight::database::ATTask& task);
  void refresh_optimization_tab(const insight::database::ATTask& task);
  void refresh_viewer_tab(const insight::database::ATTask& task);
  void refresh_export_tab(const insight::database::ATTask& task);
  void refresh_sfm_tab(const insight::database::ATTask& task);
  void load_reconstruction_to_viewer(const insight::database::ATTask& task);
  void on_export_colmap_clicked();
  void on_run_sfm_clicked();
  void on_delete_task_clicked();
  void on_sfm_process_finished(int exitCode);
  void on_sfm_process_stdout();
  void on_sfm_process_stderr();
  void on_view_detailed_log_clicked();

  // 编辑输入数据中的 ImageGroup
  void on_edit_input_group_clicked(int row);
  void on_input_group_data_changed(uint32_t group_id);

  // Runner helpers
  void append_log(const QString& text);
  
  // 任务数据导出（Phase 3 核心功能）
  bool exportTaskDataToWorkDir(const insight::database::ATTask& task, const QString& work_dir);
  bool generateImagesAllJson(const insight::database::ATTask& task, const QString& work_dir);
  
  // 工作目录管理：使用项目数据目录结构 (project.iat.data/{uuid}/)
  QString getWorkDirectory(const insight::database::ATTask& task) const;

private:
  ProjectDocument* m_document;
  std::string m_currentTaskId;

  // UI 组件
  QTabWidget* m_tabWidget;
  QWidget* m_inputDataTab;    ///< Tab 1：Input Data
  QWidget* m_optimizationTab; ///< Tab 2：Optimization
  QWidget* m_exportTab;       ///< Tab 3：Export
  QWidget* m_sfmTab;          ///< Tab 4：SfM Runner

  // 顶部信息显示
  QLineEdit* m_taskNameEdit;
  QLabel* m_workDirLabel;    ///< 只读，基于 task_id（固定）
  QLabel* m_taskIdLabel; ///< 显示任务的完整 UUID
  QLabel* m_parentTaskLabel;
  QLabel* m_statusLabel;

  // Input Data tab
  QLabel* m_inputSummaryLabel = nullptr;
  QTableWidget* m_groupTable = nullptr;

  // Optimization tab
  QLabel* m_optimizationConfigLabel = nullptr;
  QLabel* m_optimizationResultLabel = nullptr;

  // Export Tab 中的按钮
  QPushButton* m_exportColmapButton = nullptr;
  QLabel* m_exportStatusLabel = nullptr;

  // SfM Tab
  QLabel* m_sfmWorkDirLabel = nullptr;
  QPushButton* m_runSfmButton = nullptr;
  QPushButton* m_deleteTaskButton = nullptr;  ///< Delete ATTask from project (keep work directory)
  QPushButton* m_viewDetailedLogButton = nullptr; ///< View detailed log file
  QPlainTextEdit* m_sfmLogTextEdit = nullptr;
  QLabel* m_sfmStatusLabel = nullptr;
  QProgressBar* m_sfmProgressBar = nullptr;
  QProcess* m_sfmProcess = nullptr;

  // Viewer (BundlerViewerWindow for 3D visualization)
  insight::BundlerViewerWindow* m_bundlerViewer = nullptr;
  QLabel* m_viewerStatusLabel = nullptr;

  // 编辑输入数据
  dialogs::ImageGroupDetailPanel* m_inputGroupDetailPanel = nullptr;
};

}  // namespace ui
}  // namespace insight

#endif  // UI_PANELS_ATTASKPANEL_H
