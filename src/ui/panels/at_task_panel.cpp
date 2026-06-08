/**
 * @file ATTaskPanel.cpp
 * @brief AT 任务编辑面板实现
 */

#include "at_task_panel.h"
#include "../../database/database_types.h"
#include "../models/project_document.h"
#include "../dialogs/image_group_detail_panel.h"
#include "bundler_viewer_window.h"

#include <cstdlib>
#include <cstring>
#include <cmath>
#include <QAbstractItemView>
#include <QCoreApplication>
#include <QDesktopServices>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QHeaderView>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QProcess>
#include <QProcessEnvironment>
#include <QProgressBar>
#include <QPushButton>
#include <QSignalBlocker>
#include <QTabWidget>
#include <QUrl>
#include <QTableWidget>
#include <QCheckBox>
#include <QPlainTextEdit>
#include <QSplitter>
#include <QVBoxLayout>
#include <glog/logging.h>
#include <cereal/archives/json.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <sstream>
#include <chrono>

namespace {

QString camera_mode_to_string(insight::database::ImageGroup::CameraMode mode) {
  switch (mode) {
    case insight::database::ImageGroup::CameraMode::kGroupLevel:
      return "GroupLevel";
    case insight::database::ImageGroup::CameraMode::kImageLevel:
      return "ImageLevel";
    case insight::database::ImageGroup::CameraMode::kRigBased:
      return "RigBased";
  }
  return "Unknown";
}

QString resolve_work_directory(const insight::database::ATTask& task) {
  if (!task.working_directory.empty()) {
    return QDir::cleanPath(QString::fromStdString(task.working_directory));
  }
  // Fallback: 使用 UUID (task.id)，而不是 task_id (uint32)
  return QDir::home().filePath(QString(".insightat/tasks/%1").arg(QString::fromStdString(task.id)));
}

QString reconstruction_directory_for_task(const insight::database::ATTask& task) {
  return QDir(resolve_work_directory(task)).filePath("incremental_sfm");
}

QString bool_to_yes_no(bool value) { return value ? "Yes" : "No"; }

QString camera_summary_text(const insight::database::CameraModel& camera) {
  return QString("%1x%2, f=%3, cx=%4, cy=%5")
      .arg(camera.width)
      .arg(camera.height)
      .arg(camera.focal_length, 0, 'f', 1)
      .arg(camera.principal_point_x, 0, 'f', 1)
      .arg(camera.principal_point_y, 0, 'f', 1);
}

QString find_isat_sfm_binary() {
  // ─────────────────────────────────────────────────────────────────────────────
  // 按优先级查找 isat_sfm 可执行文件：
  // 1. 环境变量 ISAT_BIN_DIR 指定的目录
  // 2. InsightAT 应用所在的同一目录
  // 3. PATH 中的默认位置
  // ─────────────────────────────────────────────────────────────────────────────

  // 1. 检查环境变量 ISAT_BIN_DIR
  const char* isat_bin_dir_env = std::getenv("ISAT_BIN_DIR");
  if (isat_bin_dir_env && strlen(isat_bin_dir_env) > 0) {
    QString bin_dir = QString::fromStdString(std::string(isat_bin_dir_env));
    QString sfm_path = QDir(bin_dir).absoluteFilePath("isat_sfm");
    if (QFileInfo::exists(sfm_path) && QFileInfo(sfm_path).isExecutable()) {
      LOG(INFO) << "Found isat_sfm via ISAT_BIN_DIR: " << sfm_path.toStdString();
      return sfm_path;
    }
  }

  // 2. 检查 InsightAT 应用所在目录
  QString app_dir = QCoreApplication::applicationDirPath();
  QString sfm_path = QDir(app_dir).absoluteFilePath("isat_sfm");
  LOG(INFO) << "Checking for isat_sfm in app dir: " << sfm_path.toStdString();
  if (QFileInfo::exists(sfm_path)) {
    if (QFileInfo(sfm_path).isExecutable()) {
      LOG(INFO) << "Found isat_sfm in app directory";
      return sfm_path;
    } else {
      LOG(WARNING) << "isat_sfm exists but is not executable: " << sfm_path.toStdString();
    }
  } else {
    LOG(WARNING) << "isat_sfm not found in app directory: " << sfm_path.toStdString();
  }

  // 3. 尝试在 PATH 中查找（使用相对名称）
  // QProcess 会自动在 PATH 中搜索
  LOG(INFO) << "Will attempt to find isat_sfm in PATH";
  return "isat_sfm";
}

} // namespace

namespace insight {
namespace ui {

ATTaskPanel::ATTaskPanel(ProjectDocument* document, QWidget* parent)
    : QWidget(parent), m_document(document), m_currentTaskId(""), m_tabWidget(nullptr),
      m_inputDataTab(nullptr), m_optimizationTab(nullptr), m_exportTab(nullptr), m_sfmTab(nullptr),
  m_taskNameEdit(nullptr), m_workDirLabel(nullptr), m_taskIdLabel(nullptr), m_parentTaskLabel(nullptr),
  m_statusLabel(nullptr), m_bundlerViewer(nullptr), m_viewerStatusLabel(nullptr),
  m_exportStatusLabel(nullptr), m_exportColmapButton(nullptr),
  m_sfmWorkDirLabel(nullptr), m_runSfmButton(nullptr), m_sfmLogTextEdit(nullptr),
  m_sfmStatusLabel(nullptr), m_sfmProgressBar(nullptr), m_sfmProcess(nullptr),
  m_inputGroupDetailPanel(nullptr) {

  setWindowTitle("AT Task Editor");
  init_ui();
}

ATTaskPanel::~ATTaskPanel() = default;

bool ATTaskPanel::load_task(const std::string& task_id) {
  if (!m_document) {
    LOG(ERROR) << "ProjectDocument is null";
    return false;
  }

  const auto& project = m_document->project();
  auto it = std::find_if(project.at_tasks.begin(), project.at_tasks.end(),
                         [&task_id](const auto& t) { return t.id == task_id; });

  if (it == project.at_tasks.end()) {
    LOG(ERROR) << "AT task not found: " << task_id;
    return false;
  }

  m_currentTaskId = task_id;
  refresh_ui();
  return true;
}

std::string ATTaskPanel::get_current_task_id() const { return m_currentTaskId; }

void ATTaskPanel::init_ui() {
  QVBoxLayout* mainLayout = new QVBoxLayout(this);

  // ─── 顶部：任务信息
  QGroupBox* infoGroup = new QGroupBox("Task Information");
  QFormLayout* infoLayout = new QFormLayout(infoGroup);

  m_taskNameEdit = new QLineEdit();
  infoLayout->addRow("Task Name:", m_taskNameEdit);

  m_workDirLabel = new QLabel();
  m_workDirLabel->setStyleSheet("font-family: monospace; font-size: 9pt; color: #666;");
  m_workDirLabel->setWordWrap(true);
  m_workDirLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
  m_workDirLabel->setMinimumHeight(40);
  m_workDirLabel->setText("(Project data directory: project.iat.data/task_N/)");
  infoLayout->addRow("Work Dir:", m_workDirLabel);

  m_taskIdLabel = new QLabel("");
  m_taskIdLabel->setStyleSheet("color: gray; font-family: monospace; font-size: 9pt;");
  infoLayout->addRow("Task ID:", m_taskIdLabel);

  m_parentTaskLabel = new QLabel("");
  infoLayout->addRow("Parent Task:", m_parentTaskLabel);

  m_statusLabel = new QLabel("Ready");
  infoLayout->addRow("Status:", m_statusLabel);

  mainLayout->addWidget(infoGroup);

  // ─── 中间：Tab 部件
  m_tabWidget = new QTabWidget();

  // Tab 1: Input Data
  m_inputDataTab = new QWidget();
  QVBoxLayout* inputLayout = new QVBoxLayout(m_inputDataTab);

    m_inputSummaryLabel = new QLabel();
    m_inputSummaryLabel->setWordWrap(true);
    m_inputSummaryLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    inputLayout->addWidget(m_inputSummaryLabel);

    m_groupTable = new QTableWidget();
    m_groupTable->setColumnCount(6);
    m_groupTable->setHorizontalHeaderLabels(
      {"Group ID", "Group Name", "Camera Mode", "Images", "Camera Summary", "Edit"});
    m_groupTable->horizontalHeader()->setStretchLastSection(false);
    m_groupTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    m_groupTable->setColumnWidth(4, 150);  // Camera Summary column wider
    m_groupTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_groupTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_groupTable->setSelectionMode(QAbstractItemView::SingleSelection);
    inputLayout->addWidget(m_groupTable, 1);

  m_tabWidget->addTab(m_inputDataTab, "Input Data");

  // Tab 2: Optimization
  m_optimizationTab = new QWidget();
  QVBoxLayout* optLayout = new QVBoxLayout(m_optimizationTab);

    m_optimizationConfigLabel = new QLabel();
    m_optimizationConfigLabel->setWordWrap(true);
    m_optimizationConfigLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    optLayout->addWidget(m_optimizationConfigLabel);

    m_optimizationResultLabel = new QLabel();
    m_optimizationResultLabel->setWordWrap(true);
    m_optimizationResultLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    optLayout->addWidget(m_optimizationResultLabel);
    optLayout->addStretch();
  m_tabWidget->addTab(m_optimizationTab, "Optimization");

  // Tab: Viewer (task-scoped viewer, reuse BundlerViewerWindow)
  m_bundlerViewer = new insight::BundlerViewerWindow();
  m_bundlerViewer->setMinimumHeight(400);
  m_viewerStatusLabel = new QLabel();
  m_viewerStatusLabel->setWordWrap(true);
  m_viewerStatusLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
  m_viewerStatusLabel->setMaximumHeight(50);
  
  auto* viewerTabWidget = new QWidget();
  auto* viewerLayout = new QVBoxLayout(viewerTabWidget);
  viewerLayout->addWidget(m_bundlerViewer, 1);
  viewerLayout->addWidget(m_viewerStatusLabel);
  
  m_tabWidget->addTab(viewerTabWidget, "Viewer");
  // Tab 3: Export (only COLMAP)
  m_exportTab = new QWidget();
  QVBoxLayout* exportLayout = new QVBoxLayout(m_exportTab);

  // Export to COLMAP
  m_exportColmapButton = new QPushButton("Export to COLMAP");
  m_exportColmapButton->setMinimumHeight(40);
  connect(m_exportColmapButton, &QPushButton::clicked, this, &ATTaskPanel::on_export_colmap_clicked);
  exportLayout->addWidget(m_exportColmapButton);

  m_exportStatusLabel = new QLabel();
  m_exportStatusLabel->setWordWrap(true);
  m_exportStatusLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
  exportLayout->addWidget(m_exportStatusLabel);
  exportLayout->addStretch();

  m_tabWidget->addTab(m_exportTab, "Export");

  // Tab 4: SfM (one-click SfM pipeline runner)
  m_sfmTab = new QWidget();
  QVBoxLayout* sfmLayout = new QVBoxLayout(m_sfmTab);
  
  // Work directory display
  QGroupBox* sfmControlGroup = new QGroupBox("SfM Pipeline Configuration");
  QFormLayout* sfmControlLayout = new QFormLayout(sfmControlGroup);
  
  m_sfmWorkDirLabel = new QLabel();
  m_sfmWorkDirLabel->setStyleSheet("font-family: monospace; font-size: 9pt;");
  m_sfmWorkDirLabel->setWordWrap(true);
  m_sfmWorkDirLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
  m_sfmWorkDirLabel->setMinimumHeight(40);
  sfmControlLayout->addRow("Work Directory:", m_sfmWorkDirLabel);
  
  // Control buttons layout (Run and Delete side by side)
  QHBoxLayout* sfmButtonLayout = new QHBoxLayout();
  
  // Run button
  m_runSfmButton = new QPushButton("▶ Run SfM Pipeline");
  m_runSfmButton->setMinimumHeight(40);
  m_runSfmButton->setStyleSheet("font-weight: bold; font-size: 11pt;");
  connect(m_runSfmButton, &QPushButton::clicked, this, &ATTaskPanel::on_run_sfm_clicked);
  sfmButtonLayout->addWidget(m_runSfmButton);
  
  // Delete button
  m_deleteTaskButton = new QPushButton("🗑 Delete Task");
  m_deleteTaskButton->setMinimumHeight(40);
  m_deleteTaskButton->setMaximumWidth(150);
  m_deleteTaskButton->setStyleSheet("font-weight: bold; font-size: 11pt;");
  connect(m_deleteTaskButton, &QPushButton::clicked, this, &ATTaskPanel::on_delete_task_clicked);
  sfmButtonLayout->addWidget(m_deleteTaskButton);
  
  sfmControlLayout->addRow(sfmButtonLayout);
  
  // Status label
  m_sfmStatusLabel = new QLabel("Ready");
  m_sfmStatusLabel->setWordWrap(true);
  sfmControlLayout->addRow("Status:", m_sfmStatusLabel);
  
  // Progress bar
  m_sfmProgressBar = new QProgressBar();
  m_sfmProgressBar->setRange(0, 0);  // Indeterminate progress
  m_sfmProgressBar->setVisible(false);
  sfmControlLayout->addRow(m_sfmProgressBar);
  
  sfmLayout->addWidget(sfmControlGroup);
  
  // Log output
  QGroupBox* logGroup = new QGroupBox("Execution Log");
  QVBoxLayout* logLayout = new QVBoxLayout(logGroup);
  
  // Log display area
  m_sfmLogTextEdit = new QPlainTextEdit();
  m_sfmLogTextEdit->setReadOnly(true);
  m_sfmLogTextEdit->setStyleSheet("font-family: monospace; font-size: 9pt;");
  m_sfmLogTextEdit->setMinimumHeight(200);
  logLayout->addWidget(m_sfmLogTextEdit);
  
  // View detailed log button
  QHBoxLayout* logButtonLayout = new QHBoxLayout();
  m_viewDetailedLogButton = new QPushButton("📂 View Detailed Log");
  m_viewDetailedLogButton->setMaximumWidth(150);
  connect(m_viewDetailedLogButton, &QPushButton::clicked, this, &ATTaskPanel::on_view_detailed_log_clicked);
  logButtonLayout->addStretch();
  logButtonLayout->addWidget(m_viewDetailedLogButton);
  logLayout->addLayout(logButtonLayout);
  
  sfmLayout->addWidget(logGroup, 1);
  
  m_tabWidget->addTab(m_sfmTab, "SfM");

  mainLayout->addWidget(m_tabWidget);

  // ─── 自动保存：监听任务名称变更
  connect(m_taskNameEdit, &QLineEdit::textChanged, this, &ATTaskPanel::on_task_name_changed);
}

void ATTaskPanel::refresh_ui() {
  if (m_currentTaskId.empty() || !m_document) {
    m_taskNameEdit->clear();
    m_taskIdLabel->setText("");
    m_parentTaskLabel->setText("");
    m_statusLabel->setText("No task loaded");
    return;
  }

  const auto& project = m_document->project();
  auto it = std::find_if(project.at_tasks.begin(), project.at_tasks.end(),
                         [this](const auto& t) { return t.id == m_currentTaskId; });

  if (it == project.at_tasks.end()) {
    LOG(ERROR) << "Task not found during refresh: " << m_currentTaskId;
    return;
  }

  // 更新任务信息显示
  {
    const QSignalBlocker blocker(m_taskNameEdit);
    m_taskNameEdit->setText(QString::fromStdString(it->task_name));
  }
  m_workDirLabel->setText(getWorkDirectory(*it));
  m_taskIdLabel->setText(QString::fromStdString(m_currentTaskId));

  // 显示父任务信息
  if (it->initialization && it->initialization->prev_task_id != static_cast<uint32_t>(-1)) {
    // 根据 prev_task_id 查找父任务的任务名称
    uint32_t parentTaskIdx = it->initialization->prev_task_id;
    if (parentTaskIdx < project.at_tasks.size()) {
      QString parentName = QString::fromStdString(project.at_tasks[parentTaskIdx].task_name);
      m_parentTaskLabel->setText(parentName);
    } else {
      m_parentTaskLabel->setText("Unknown");
    }
  } else {
    m_parentTaskLabel->setText("None");
  }

  m_statusLabel->setText("Loaded");
  refresh_input_data_tab(*it);
  refresh_optimization_tab(*it);
  refresh_viewer_tab(*it);
  refresh_export_tab(*it);
  refresh_sfm_tab(*it);
}

void ATTaskPanel::refresh_input_data_tab(const insight::database::ATTask& task) {
  if (!m_inputSummaryLabel || !m_groupTable) {
    return;
  }

  size_t total_images = 0;
  for (const auto& group : task.input_snapshot.image_groups) {
    total_images += group.images.size();
  }

  QString summary = QString("Coordinate System: %1\nMeasurements: %2\nImage Groups: %3\n"
                            "Images: %4\nWork Dir: %5")
                        .arg(QString::fromStdString(task.input_snapshot.input_coordinate_system.to_string()))
                        .arg(task.input_snapshot.measurements.size())
                        .arg(task.input_snapshot.image_groups.size())
                        .arg(total_images)
                        .arg(getWorkDirectory(task));
  m_inputSummaryLabel->setText(summary);

  m_groupTable->setRowCount(static_cast<int>(task.input_snapshot.image_groups.size()));
  for (int row = 0; row < static_cast<int>(task.input_snapshot.image_groups.size()); ++row) {
    const auto& group = task.input_snapshot.image_groups[static_cast<size_t>(row)];
    QString camera_summary = group.group_camera.has_value()
                                 ? camera_summary_text(*group.group_camera)
                                 : QStringLiteral("Per-image / rig camera");

    m_groupTable->setItem(row, 0, new QTableWidgetItem(QString::number(group.group_id)));
    m_groupTable->setItem(row, 1, new QTableWidgetItem(QString::fromStdString(group.group_name)));
    m_groupTable->setItem(row, 2, new QTableWidgetItem(camera_mode_to_string(group.camera_mode)));
    m_groupTable->setItem(row, 3,
                          new QTableWidgetItem(QString::number(static_cast<int>(group.images.size()))));
    m_groupTable->setItem(row, 4, new QTableWidgetItem(camera_summary));

    // 添加编辑按钮
    auto editBtn = new QPushButton(tr("✏ Edit"));
    editBtn->setMaximumWidth(100);
    connect(editBtn, &QPushButton::clicked, this, [this, row]() {
      on_edit_input_group_clicked(row);
    });
    m_groupTable->setCellWidget(row, 5, editBtn);
  }
}

void ATTaskPanel::refresh_optimization_tab(const insight::database::ATTask& task) {
  if (!m_optimizationConfigLabel || !m_optimizationResultLabel) {
    return;
  }

  QString config_summary = QString("Configured cameras: %1\nGNSS constraint: %2\nGNSS weight: %3\n"
                                   "Max reprojection error: %4 px\nIn-memory optimized poses: %5")
                               .arg(task.optimization_config.camera_optimization.size())
                               .arg(bool_to_yes_no(task.optimization_config.enable_gnss_constraint))
                               .arg(task.optimization_config.gnss_weight)
                               .arg(task.optimization_config.max_reprojection_error)
                               .arg(task.optimized_poses.size());
  m_optimizationConfigLabel->setText(config_summary);

  const QString reconstruction_dir = reconstruction_directory_for_task(task);
  const QString poses_path = QDir(reconstruction_dir).filePath("poses.json");
  const QString timing_path = QDir(getWorkDirectory(task)).filePath("sfm_timing.json");
  const QString bundle_path = QDir(reconstruction_dir).filePath("bundle.out");
  const QString colmap_sparse_dir = QDir(reconstruction_dir).filePath("colmap/sparse/0");

  int poses_count = -1;
  QFile poses_file(poses_path);
  if (poses_file.open(QIODevice::ReadOnly)) {
    QJsonParseError parse_error;
    const QJsonDocument doc = QJsonDocument::fromJson(poses_file.readAll(), &parse_error);
    if (parse_error.error == QJsonParseError::NoError && doc.isObject()) {
      const QJsonValue poses_value = doc.object().value("poses");
      if (poses_value.isArray()) {
        poses_count = poses_value.toArray().size();
      }
    }
  }

  QString result_summary = QString("Reconstruction Dir: %1\nposes.json: %2\n"
                                   "Optimized poses on disk: %3\nBundle output: %4\n"
                                   "COLMAP sparse dir: %5\nTiming file: %6")
                               .arg(reconstruction_dir)
                               .arg(bool_to_yes_no(QFileInfo::exists(poses_path)))
                               .arg(poses_count >= 0 ? QString::number(poses_count) : QStringLiteral("N/A"))
                               .arg(bool_to_yes_no(QFileInfo::exists(bundle_path)))
                               .arg(bool_to_yes_no(QDir(colmap_sparse_dir).exists()))
                               .arg(bool_to_yes_no(QFileInfo::exists(timing_path)));
  m_optimizationResultLabel->setText(result_summary);
}

void ATTaskPanel::refresh_viewer_tab(const insight::database::ATTask& task) {
  if (!m_viewerStatusLabel || !m_bundlerViewer) {
    return;
  }

  const QString reconstruction_dir = reconstruction_directory_for_task(task);
  const QString bundle_path = QDir(reconstruction_dir).filePath("bundle.out");
  const QString list_path = QDir(reconstruction_dir).filePath("list.txt");
  const QString colmap_sparse_dir = QDir(reconstruction_dir).filePath("colmap/sparse/0");

  bool bundler_exists = QFileInfo::exists(bundle_path) && QFileInfo::exists(list_path);
  bool colmap_exists = QDir(colmap_sparse_dir).exists();

  if (bundler_exists || colmap_exists) {
    load_reconstruction_to_viewer(task);
    m_viewerStatusLabel->setText(
        QString("Reconstruction loaded from: %1").arg(reconstruction_dir));
  } else {
    m_viewerStatusLabel->setText(
        QString("No reconstruction found. Expected Bundler (bundle.out + list.txt) or COLMAP "
                "(colmap/sparse/0) in: %1")
            .arg(reconstruction_dir));
  }
}

void ATTaskPanel::load_reconstruction_to_viewer(const insight::database::ATTask& task) {
  if (!m_bundlerViewer) {
    return;
  }

  const QString reconstruction_dir = reconstruction_directory_for_task(task);
  m_viewerStatusLabel->setText(QString("Loading: %1").arg(reconstruction_dir));
  m_bundlerViewer->open_reconstruction_path(reconstruction_dir);
  m_viewerStatusLabel->setText(QString("Loaded: %1").arg(reconstruction_dir));
}

void ATTaskPanel::refresh_export_tab(const insight::database::ATTask& task) {
  if (!m_exportStatusLabel) {
    return;
  }

  const QString work_dir = getWorkDirectory(task);
  const QString reconstruction_dir = reconstruction_directory_for_task(task);
  const QString colmap_sparse_dir = QDir(reconstruction_dir).filePath("colmap/sparse/0");

  QString export_summary = QString("Task work dir: %1\nReconstruction dir: %2\n"
                                   "Bundler source ready: %3\nCOLMAP sparse source ready: %4\n"
                                   "Export action will reuse this directory contract.")
                               .arg(work_dir)
                               .arg(reconstruction_dir)
                               .arg(bool_to_yes_no(QFileInfo::exists(QDir(reconstruction_dir).filePath("bundle.out"))))
                               .arg(bool_to_yes_no(QDir(colmap_sparse_dir).exists()));
  m_exportStatusLabel->setText(export_summary);
}

void ATTaskPanel::on_task_name_changed() {
  save_task();
}

void ATTaskPanel::on_export_colmap_clicked() {
  if (m_currentTaskId.empty() || !m_document) {
    QMessageBox::warning(this, "Warning", "No task loaded");
    return;
  }

  const auto* task = m_document->getATTaskById(m_currentTaskId);
  if (!task) {
    QMessageBox::critical(this, "Error", "Task not found");
    return;
  }

  const QString reconstruction_dir = reconstruction_directory_for_task(*task);
  const QString colmap_sparse_dir = QDir(reconstruction_dir).filePath("colmap/sparse/0");
  if (!QDir(colmap_sparse_dir).exists()) {
    QMessageBox::warning(this, "Error", 
        QString("COLMAP sparse directory does not exist: %1").arg(colmap_sparse_dir));
    return;
  }

  QString export_dir = QFileDialog::getExistingDirectory(this, "Select COLMAP Export Directory");
  if (export_dir.isEmpty()) {
    return;
  }

  m_statusLabel->setText("Exporting to COLMAP...");
  append_log(QString("Exporting task %1 to COLMAP format: %2")
             .arg(QString::fromStdString(m_currentTaskId), export_dir));

  QDir export_path(export_dir);
  if (!export_path.exists("colmap")) {
    export_path.mkdir("colmap");
  }
  if (!export_path.exists("colmap/sparse")) {
    export_path.mkdir("colmap/sparse");
  }
  if (!export_path.exists("colmap/sparse/0")) {
    export_path.mkdir("colmap/sparse/0");
  }

  const QString export_colmap_dir = export_path.filePath("colmap/sparse/0");
  const QStringList colmap_files = {"cameras.txt", "images.txt", "points3D.txt"};
  bool all_copied = true;

  for (const auto& file : colmap_files) {
    const QString source = QDir(colmap_sparse_dir).filePath(file);
    const QString dest = QDir(export_colmap_dir).filePath(file);
    if (QFileInfo::exists(source)) {
      if (!QFile::copy(source, dest)) {
        all_copied = false;
        append_log(QString("Warning: failed to copy %1").arg(file));
      }
    }
  }

  if (all_copied) {
    m_statusLabel->setText("COLMAP export completed successfully");
    append_log(QString("COLMAP export completed to: %1").arg(export_colmap_dir));
    QMessageBox::information(this, "Success", 
        QString("COLMAP sparse model exported successfully to: %1").arg(export_colmap_dir));
  } else {
    m_statusLabel->setText("COLMAP export completed with warnings");
    append_log("COLMAP export completed with warnings - see details above");
    QMessageBox::warning(this, "Partial Success", 
        QString("COLMAP export completed with some warnings. Check log for details."));
  }
}

void ATTaskPanel::save_task() {
  if (m_currentTaskId.empty() || !m_document) {
    LOG(WARNING) << "No task loaded";
    return;
  }

  // 获取当前任务
  auto* task = m_document->getATTaskById(m_currentTaskId);
  if (!task) {
    LOG(ERROR) << "Task not found: " << m_currentTaskId;
    return;
  }

  // 更新任务名称（如果编辑框中有改动）
  std::string newTaskName = m_taskNameEdit->text().toStdString();
  if (newTaskName != task->task_name) {
    task->task_name = newTaskName;
  }

  // work_dir 是只读的，基于 task_id，不可修改

  // 通过 ProjectDocument 保存更新
  m_document->updateATTask(m_currentTaskId, *task);

  m_statusLabel->setText("Saved");
  LOG(INFO) << "AT task auto-saved: " << m_currentTaskId;
}

void ATTaskPanel::refresh_sfm_tab(const insight::database::ATTask& task) {
  if (!m_sfmWorkDirLabel) {
    return;
  }

  const QString work_dir = getWorkDirectory(task);
  m_sfmWorkDirLabel->setText(work_dir);
  
  // Reset status and log
  m_sfmStatusLabel->setText("Ready");
  m_runSfmButton->setEnabled(true);
}

void ATTaskPanel::on_run_sfm_clicked() {
  if (m_currentTaskId.empty() || !m_document) {
    QMessageBox::warning(this, "Warning", "No task loaded");
    return;
  }

  const auto* task = m_document->getATTaskById(m_currentTaskId);
  if (!task) {
    QMessageBox::critical(this, "Error", "Task not found");
    return;
  }

  const QString work_dir = getWorkDirectory(*task);
  
  // Disable button and show progress
  m_runSfmButton->setEnabled(false);
  m_sfmProgressBar->setVisible(true);
  m_sfmLogTextEdit->clear();
  m_sfmStatusLabel->setText("Preparing SfM pipeline...");

  // ─────────────────────────────────────────────────────────────────────────────
  // 准备工作目录和必要的文件
  // ─────────────────────────────────────────────────────────────────────────────
  QDir work_dir_obj(work_dir);
  if (!work_dir_obj.exists()) {
    if (!work_dir_obj.mkpath(".")) {
      m_runSfmButton->setEnabled(true);
      m_sfmProgressBar->setVisible(false);
      m_sfmStatusLabel->setText("Failed to create work directory");
      m_sfmLogTextEdit->appendPlainText("❌ ERROR: Failed to create work directory");
      m_sfmLogTextEdit->appendPlainText(QString("Path: %1").arg(work_dir));
      QMessageBox::critical(this, "Error", 
          QString("Failed to create work directory:\n%1").arg(work_dir));
      return;
    }
    m_sfmLogTextEdit->appendPlainText(QString("Created work directory: %1").arg(work_dir));
  }
  
  // 创建输出子目录
  QStringList subdirs = {"feat", "feat_retrieval", "match", "geo", "incremental_sfm"};
  for (const auto& subdir : subdirs) {
    QString subdir_path = work_dir_obj.filePath(subdir);
    QDir subdir_obj(subdir_path);
    if (!subdir_obj.exists()) {
      subdir_obj.mkpath(".");
    }
  }
  
  m_sfmStatusLabel->setText("Running isat_sfm...");
  m_sfmLogTextEdit->appendPlainText("");
  m_sfmLogTextEdit->appendPlainText("═══════════════════════════════════════════════════════════");
  m_sfmLogTextEdit->appendPlainText(QString("Starting SfM Pipeline for Task: %1").arg(QString::fromStdString(task->task_name)));
  m_sfmLogTextEdit->appendPlainText(QString("Work directory: %1").arg(work_dir));
  m_sfmLogTextEdit->appendPlainText("═══════════════════════════════════════════════════════════");
  m_sfmLogTextEdit->appendPlainText("");

  // Get isat_sfm binary path (check ISAT_BIN_DIR env var, app directory, or PATH)
  QString sfm_bin = find_isat_sfm_binary();
  
  // Kill previous process if running
  if (m_sfmProcess) {
    m_sfmProcess->kill();
    m_sfmProcess->waitForFinished(3000);
    delete m_sfmProcess;
    m_sfmProcess = nullptr;
  }

  // Create and configure process
  m_sfmProcess = new QProcess(this);
  m_sfmProcess->setWorkingDirectory(work_dir);
  
  // ─────────────────────────────────────────────────────────────────────────────
  // Setup environment variables for isat_sfm
  // ─────────────────────────────────────────────────────────────────────────────
  QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
  
  // Get the build directory from the binary path
  QString bin_dir = QFileInfo(sfm_bin).dir().absolutePath();
  QString ld_library_path = bin_dir;  // Add build directory
  ld_library_path += ":" + QDir(bin_dir).absoluteFilePath("third_party/popsift/Linux-x86_64");
  
  // Append existing LD_LIBRARY_PATH if present
  if (env.contains("LD_LIBRARY_PATH")) {
    ld_library_path += ":" + env.value("LD_LIBRARY_PATH");
  }
  
  env.insert("LD_LIBRARY_PATH", ld_library_path);
  m_sfmProcess->setProcessEnvironment(env);
  
  m_sfmLogTextEdit->appendPlainText(QString("Set LD_LIBRARY_PATH: %1").arg(ld_library_path));
  
  // Prepare arguments for isat_sfm
  // ─────────────────────────────────────────────────────────────────────────────
  // isat_sfm --existing-task workflow:
  //   Runs extract → match → tracks → seed_eval → incremental_sfm
  //   (skips 'create' step automatically, uses existing images_all.json)
  //
  // Step 1: Export latest task data to work directory (ensures images_all.json is current)
  // Step 2: Use --existing-task mode to run the pipeline
  // ─────────────────────────────────────────────────────────────────────────────
  
  // Export latest task data (includes images_all.json)
  m_sfmLogTextEdit->appendPlainText("Exporting task data (images, GNSS, IMU, GCP, coordinate system)...");
  
  if (!exportTaskDataToWorkDir(*task, work_dir)) {
    m_runSfmButton->setEnabled(true);
    m_sfmProgressBar->setVisible(false);
    m_sfmStatusLabel->setText("Failed to export task data");
    m_sfmLogTextEdit->appendPlainText("");
    m_sfmLogTextEdit->appendPlainText("❌ ERROR: Failed to export task data to work directory");
    QMessageBox::critical(this, "Error", 
        "Failed to export task data. Check logs for details.");
    delete m_sfmProcess;
    m_sfmProcess = nullptr;
    return;
  }
  
  m_sfmLogTextEdit->appendPlainText("✓ Task data exported successfully");
  
  // Verify images_all.json exists
  QString images_all_path = work_dir_obj.filePath("images_all.json");
  if (!QFileInfo::exists(images_all_path)) {
    m_runSfmButton->setEnabled(true);
    m_sfmProgressBar->setVisible(false);
    m_sfmStatusLabel->setText("Missing images_all.json");
    m_sfmLogTextEdit->appendPlainText("");
    m_sfmLogTextEdit->appendPlainText("❌ ERROR: images_all.json not found after export");
    QMessageBox::critical(this, "Error", 
        "Failed to create images_all.json. Check export function.");
    delete m_sfmProcess;
    m_sfmProcess = nullptr;
    return;
  }
  
  m_sfmLogTextEdit->appendPlainText(QString("✓ Found images_all.json in work directory"));
  
  // Build command line arguments using --existing-task mode
  QStringList arguments;
  arguments << "--existing-task";
  arguments << "-w" << work_dir;
  arguments << "-v";
  
  // Connect signals
  connect(m_sfmProcess, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
          this, &ATTaskPanel::on_sfm_process_finished);
  connect(m_sfmProcess, &QProcess::readyReadStandardOutput,
          this, &ATTaskPanel::on_sfm_process_stdout);
  connect(m_sfmProcess, &QProcess::readyReadStandardError,
          this, &ATTaskPanel::on_sfm_process_stderr);

  m_sfmLogTextEdit->appendPlainText(QString("Attempting to start: %1").arg(sfm_bin));
  m_sfmLogTextEdit->appendPlainText(QString("Arguments: %1").arg(arguments.join(" ")));
  m_sfmLogTextEdit->appendPlainText(QString("Work directory: %1").arg(work_dir));

  // Start process
  m_sfmProcess->start(sfm_bin, arguments);
  
  if (!m_sfmProcess->waitForStarted(3000)) {
    m_runSfmButton->setEnabled(true);
    m_sfmProgressBar->setVisible(false);
    m_sfmStatusLabel->setText("Failed to start isat_sfm");
    m_sfmLogTextEdit->appendPlainText("");
    m_sfmLogTextEdit->appendPlainText("❌ ERROR: Failed to start isat_sfm binary");
    m_sfmLogTextEdit->appendPlainText("");
    
    // Get QProcess error information
    QProcess::ProcessError error = m_sfmProcess->error();
    QString error_msg;
    switch (error) {
      case QProcess::FailedToStart:
        error_msg = "FailedToStart: The process could not be started";
        break;
      case QProcess::Crashed:
        error_msg = "Crashed: The process crashed";
        break;
      case QProcess::Timedout:
        error_msg = "Timedout: waitForStarted() timed out";
        break;
      case QProcess::WriteError:
        error_msg = "WriteError: An error occurred when attempting to write to the process";
        break;
      case QProcess::ReadError:
        error_msg = "ReadError: An error occurred when attempting to read from the process";
        break;
      default:
        error_msg = "UnknownError";
    }
    m_sfmLogTextEdit->appendPlainText(QString("Error code: %1").arg(error_msg));
    m_sfmLogTextEdit->appendPlainText("");
    
    m_sfmLogTextEdit->appendPlainText("Binary lookup search order:");
    m_sfmLogTextEdit->appendPlainText("1. Environment variable ISAT_BIN_DIR");
    m_sfmLogTextEdit->appendPlainText("2. Same directory as InsightAT application");
    m_sfmLogTextEdit->appendPlainText("3. System PATH");
    m_sfmLogTextEdit->appendPlainText("");
    m_sfmLogTextEdit->appendPlainText(QString("Tried to execute: %1").arg(sfm_bin));
    m_sfmLogTextEdit->appendPlainText("App directory: " + QCoreApplication::applicationDirPath());
    m_sfmLogTextEdit->appendPlainText("");
    m_sfmLogTextEdit->appendPlainText("Solutions:");
    m_sfmLogTextEdit->appendPlainText("✓ Option 1: Set ISAT_BIN_DIR environment variable");
    m_sfmLogTextEdit->appendPlainText("  export ISAT_BIN_DIR=/home/recon/Git/04jones/InsightAT/build-ceres-12.8");
    m_sfmLogTextEdit->appendPlainText("  Then restart InsightAT");
    m_sfmLogTextEdit->appendPlainText("");
    m_sfmLogTextEdit->appendPlainText("✓ Option 2: Create symlink in InsightAT app directory");
    m_sfmLogTextEdit->appendPlainText(QString("  ln -s %1 %2")
        .arg("$(pwd)/build-ceres-12.8/isat_sfm", "$(pwd)/build-ceres-12.8/InsightAT/../isat_sfm"));
    m_sfmLogTextEdit->appendPlainText("");
    m_sfmLogTextEdit->appendPlainText("✓ Option 3: Add build directory to PATH");
    m_sfmLogTextEdit->appendPlainText("  export PATH=/home/recon/Git/04jones/InsightAT/build-ceres-12.8:$PATH");
    
    QMessageBox::critical(this, "Error", 
        QString("Failed to start isat_sfm binary.\n\n"
                "Error: %1\n\n"
                "Please set environment variable:\n"
                "export ISAT_BIN_DIR=/home/recon/Git/04jones/InsightAT/build-ceres-12.8\n\n"
                "Then restart InsightAT.").arg(error_msg));
    delete m_sfmProcess;
    m_sfmProcess = nullptr;
    return;
  }

  m_sfmLogTextEdit->appendPlainText(QString("✓ Started: %1").arg(sfm_bin));
  m_sfmLogTextEdit->appendPlainText(QString("  Work dir: %1").arg(work_dir));
  m_sfmLogTextEdit->appendPlainText("  Detailed logs available in work directory");
  
  LOG(INFO) << "Started isat_sfm process: " << sfm_bin.toStdString();
}

void ATTaskPanel::on_sfm_process_stdout() {
  if (!m_sfmProcess)
    return;
  
  // Discard output - detailed logs are saved in work directory
  m_sfmProcess->readAllStandardOutput();
}

void ATTaskPanel::on_sfm_process_stderr() {
  if (!m_sfmProcess)
    return;
  
  // Discard output - detailed logs are saved in work directory
  m_sfmProcess->readAllStandardError();
}

void ATTaskPanel::on_delete_task_clicked() {
  if (m_currentTaskId.empty() || !m_document) {
    QMessageBox::warning(this, "Warning", "No ATTask selected");
    return;
  }

  // Get task name
  const auto* task = m_document->getATTaskById(m_currentTaskId);
  if (!task) {
    QMessageBox::warning(this, "Warning", "AT Task not found");
    return;
  }

  // Confirmation dialog
  QMessageBox::StandardButton reply = QMessageBox::question(
      this, "Delete ATTask",
      QString("Are you sure you want to delete ATTask '%1' from the project?\n\n"
              "Note: The working directory will be preserved.\n"
              "You can manually delete it later if needed.")
          .arg(QString::fromStdString(task->task_name)),
      QMessageBox::Yes | QMessageBox::No);

  if (reply != QMessageBox::Yes) {
    return;
  }

  // Delete task from project using task_id (uint32_t)
  bool success = m_document->deleteATTask(task->task_id);
  if (success) {
    m_currentTaskId.clear();
    m_sfmWorkDirLabel->setText("");
    m_sfmStatusLabel->setText("Task deleted");
    QMessageBox::information(this, "Success", "ATTask deleted from project successfully.\nWorking directory preserved.");
  } else {
    QMessageBox::critical(this, "Error", "Failed to delete ATTask from project");
  }
}

void ATTaskPanel::on_edit_input_group_clicked(int row) {
  if (!m_document || m_currentTaskId.empty()) {
    QMessageBox::warning(this, "Warning", "No input snapshot available");
    return;
  }

  // 获取当前任务
  auto* task = const_cast<database::ATTask*>(m_document->getATTaskById(m_currentTaskId));
  if (!task) {
    QMessageBox::warning(this, "Warning", "ATTask not found");
    return;
  }

  if (row < 0 || row >= static_cast<int>(task->input_snapshot.image_groups.size())) {
    QMessageBox::warning(this, "Warning", "Invalid group row");
    return;
  }

  // 懒创建对话框
  if (!m_inputGroupDetailPanel) {
    m_inputGroupDetailPanel = new dialogs::ImageGroupDetailPanel(this);
    m_inputGroupDetailPanel->set_project_document(m_document);
    connect(m_inputGroupDetailPanel, &dialogs::ImageGroupDetailPanel::groupDataChanged, this,
            &ATTaskPanel::on_input_group_data_changed);
  }

  // 获取要编辑的 group
  auto& group = task->input_snapshot.image_groups[row];
  m_inputGroupDetailPanel->load_group(&group);
}

void ATTaskPanel::on_input_group_data_changed(uint32_t group_id) {
  Q_UNUSED(group_id);

  if (!m_document || m_currentTaskId.empty()) {
    return;
  }

  // 获取当前任务
  const auto* task = m_document->getATTaskById(m_currentTaskId);
  if (task) {
    refresh_input_data_tab(*task);
    // 直接保存项目（如果已有文件路径）
    if (!m_document->filepath().isEmpty()) {
      m_document->saveProject();
      LOG(INFO) << "Input snapshot modified and saved. "
                << "images_all.json will be regenerated when SfM runs.";
    }
  }
}

void ATTaskPanel::on_sfm_process_finished(int exitCode) {
  m_runSfmButton->setEnabled(true);
  m_sfmProgressBar->setVisible(false);
  
  if (exitCode == 0) {
    m_sfmStatusLabel->setText("✓ SfM completed successfully");
    m_sfmLogTextEdit->appendPlainText("✓ SfM pipeline completed successfully");
    QMessageBox::information(this, "Success", "SfM pipeline completed successfully!");
    
    // Auto-refresh viewer tab to show new reconstruction
    if (m_currentTaskId.empty() || !m_document) {
      return;
    }
    const auto* task = m_document->getATTaskById(m_currentTaskId);
    if (task) {
      refresh_viewer_tab(*task);
    }
  } else {
    m_sfmStatusLabel->setText(QString("✗ SfM failed (exit code %1)").arg(exitCode));
    m_sfmLogTextEdit->appendPlainText(QString("✗ SfM pipeline failed (exit code %1)").arg(exitCode));
    QMessageBox::critical(this, "Error", 
        QString("SfM pipeline failed with exit code %1. Check logs for details.").arg(exitCode));
  }
  
  delete m_sfmProcess;
  m_sfmProcess = nullptr;
}

void ATTaskPanel::append_log(const QString& text) {
  // Log output removed; messages are only printed to glog
  LOG(INFO) << text.toStdString();
}

QString ATTaskPanel::getWorkDirectory(const insight::database::ATTask& task) const {
  // ─────────────────────────────────────────────────────────────────────────────
  // Each ATTask has its own subdirectory: project.iat.data/task_N/
  // Priority:
  // 1. Use task.working_directory if already set
  // 2. Derive from project: project_file.data/task_N/
  // 3. Fallback: ~/.insightat/default_work/task_N/
  // ─────────────────────────────────────────────────────────────────────────────
  
  if (!task.working_directory.empty()) {
    return QDir::cleanPath(QString::fromStdString(task.working_directory));
  }
  
  // Derive from project file and task_id
  if (m_document && !m_document->filepath().isEmpty()) {
    QString project_file = m_document->filepath();
    QString project_data_dir = project_file + ".data";
    return QDir(project_data_dir).filePath(QString("task_%1").arg(task.task_id));
  }
  
  // Fallback to default
  return QDir::home().filePath(QString(".insightat/default_work/task_%1").arg(task.task_id));
}

void ATTaskPanel::on_view_detailed_log_clicked() {
  if (m_currentTaskId.empty() || !m_document) {
    QMessageBox::warning(this, "Warning", "No task loaded");
    return;
  }

  const auto* task = m_document->getATTaskById(m_currentTaskId);
  if (!task) {
    QMessageBox::critical(this, "Error", "Task not found");
    return;
  }

  const QString work_dir = resolve_work_directory(*task);
  const QString logs_dir = QDir(work_dir).filePath("logs");

  // Check if logs directory exists
  if (!QDir(logs_dir).exists()) {
    QMessageBox::information(this, "Info", 
        QString("Log directory not found:\n%1\n\nNo logs have been generated yet.").arg(logs_dir));
    return;
  }

  // Try to open logs directory with system file manager
  QUrl logs_url = QUrl::fromLocalFile(logs_dir);
  bool opened = QDesktopServices::openUrl(logs_url);
  
  if (!opened) {
    QMessageBox::warning(this, "Error", 
        QString("Failed to open log directory:\n%1\n\nPlease open manually.").arg(logs_dir));
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// 任务数据导出函数 (Phase 3 核心功能)
// ─────────────────────────────────────────────────────────────────────────────

bool ATTaskPanel::exportTaskDataToWorkDir(const insight::database::ATTask& task,
                                          const QString& work_dir) {
  // 导出任务的 input_snapshot 到工作目录，使 isat_sfm 能直接使用
  
  m_sfmLogTextEdit->appendPlainText("");
  m_sfmLogTextEdit->appendPlainText("Exporting task data to work directory...");

  // Step 1: 导出任务输入快照为 project_snapshot.json
  // 包含坐标系、测量、图像组等所有必要的项目数据
  QString snapshot_json_path = QDir(work_dir).filePath("project_snapshot.json");
  
  // 创建快照 JSON 对象
  QJsonObject snapshot_root;
  
  // 记录导出信息
  snapshot_root["export_type"] = "task_input_snapshot";
  snapshot_root["task_id"] = QString::fromStdString(task.id);
  snapshot_root["task_name"] = QString::fromStdString(task.task_name);
  snapshot_root["export_timestamp"] = static_cast<qint64>(std::time(nullptr));
  
  // 记录坐标系信息（简化表示）
  QJsonObject coordinate_system;
  coordinate_system["type"] = static_cast<int>(task.input_snapshot.input_coordinate_system.type);
  coordinate_system["rotation_convention"] = 
      static_cast<int>(task.input_snapshot.input_coordinate_system.rotation_convention);
  coordinate_system["definition"] = 
      QString::fromStdString(task.input_snapshot.input_coordinate_system.definition);
  snapshot_root["coordinate_system"] = coordinate_system;
  
  // 记录测量数据个数
  snapshot_root["measurements_count"] = static_cast<int>(task.input_snapshot.measurements.size());
  
  // 记录图像组统计
  QJsonArray image_groups_summary;
  size_t total_images = 0;
  for (const auto& group : task.input_snapshot.image_groups) {
    total_images += group.images.size();
  }
  snapshot_root["total_images"] = static_cast<int>(total_images);
  snapshot_root["image_groups_count"] = static_cast<int>(task.input_snapshot.image_groups.size());
  
  // 写入 project_snapshot.json
  QJsonDocument snapshot_doc(snapshot_root);
  QFile snapshot_file(snapshot_json_path);
  if (!snapshot_file.open(QIODevice::WriteOnly)) {
    LOG(ERROR) << "Failed to open project_snapshot.json for writing";
    return false;
  }
  snapshot_file.write(snapshot_doc.toJson(QJsonDocument::Indented));
  snapshot_file.close();
  
  m_sfmLogTextEdit->appendPlainText(
      QString("✓ Exported project snapshot (%1 images, %2 measurements)")
      .arg(total_images)
      .arg(task.input_snapshot.measurements.size()));
  
  // Step 2: 生成 images_all.json (包含所有图像的路径和元数据)
  if (!generateImagesAllJson(task, work_dir)) {
    m_sfmLogTextEdit->appendPlainText("❌ Failed to generate images_all.json");
    return false;
  }
  
  m_sfmLogTextEdit->appendPlainText("✓ Task data export completed");
  return true;
}

bool ATTaskPanel::generateImagesAllJson(const insight::database::ATTask& task,
                                        const QString& work_dir) {
  // Generate images_all.json in the same format as isat_project extract command
  // This format is consumed by isat_extract, isat_retrieval_match, etc.
  // 
  // NOTE: This regenerates from the current input_snapshot, so any edits to camera
  // parameters in the ATTask will be reflected in the new images_all.json
  
  using json = nlohmann::json;
  
  // ─── Build cameras[] array (one camera per group) ───────────────────────────────
  std::map<uint32_t, int> group_to_camera_index;
  json cameras_arr = json::array();
  
  for (const auto& group : task.input_snapshot.image_groups) {
    // Pick camera from this group
    const insight::database::CameraModel* cam = nullptr;
    
    // Try group-level camera first
    if (group.group_camera.has_value()) {
      cam = &group.group_camera.value();
    } else if (!group.images.empty() && group.images[0].camera.has_value()) {
      // Fallback to first image's camera
      cam = &group.images[0].camera.value();
    }
    
    if (!cam || cam->focal_length <= 0.0)
      continue;
    
    const double fx = cam->focal_length;
    const double fy = cam->focal_length * (cam->aspect_ratio > 0 ? cam->aspect_ratio : 1.0);
    
    json cam_obj;
    cam_obj["fx"] = fx;
    cam_obj["fy"] = fy;
    cam_obj["cx"] = cam->principal_point_x;
    cam_obj["cy"] = cam->principal_point_y;
    cam_obj["width"] = cam->width;
    cam_obj["height"] = cam->height;
    
    if (!cam->camera_name.empty())
      cam_obj["camera_name"] = cam->camera_name;
    if (!cam->make.empty())
      cam_obj["make"] = cam->make;
    if (!cam->model.empty())
      cam_obj["model"] = cam->model;
    
    int cidx = static_cast<int>(cameras_arr.size());
    cameras_arr.push_back(std::move(cam_obj));
    group_to_camera_index[group.group_id] = cidx;
  }
  
  // ─── Build output JSON ───────────────────────────────────────────────────────
  json output;
  output["$schema"] = "InsightAT Image List Format v2.0";
  output["images"] = json::array();
  output["cameras"] = std::move(cameras_arr);
  
  int image_index = 0;
  int gnss_count = 0;
  int imu_count = 0;
  
  for (const auto& group : task.input_snapshot.image_groups) {
    auto it = group_to_camera_index.find(group.group_id);
    int camera_index = (it != group_to_camera_index.end()) ? it->second : 0;
    
    for (const auto& image : group.images) {
      json img;
      img["image_index"] = image_index;
      img["id"] = image.image_id;
      img["path"] = image.filename;
      img["camera_index"] = camera_index;
      img["camera_id"] = group.group_id;
      
      // Add GNSS if available
      if (image.gnss_data.has_value()) {
        const auto& gnss = image.gnss_data.value();
        img["gnss"] = {
          {"x", gnss.x},
          {"y", gnss.y},
          {"z", gnss.z},
          {"cov_xx", gnss.cov_xx},
          {"cov_yy", gnss.cov_yy},
          {"cov_zz", gnss.cov_zz},
          {"cov_xy", gnss.cov_xy},
          {"cov_xz", gnss.cov_xz},
          {"cov_yz", gnss.cov_yz},
          {"num_satellites", gnss.num_satellites},
          {"hdop", gnss.hdop},
          {"vdop", gnss.vdop}
        };
        gnss_count++;
      }
      
      // Add IMU (input_pose rotation) if available
      if (image.input_pose.has_rotation) {
        double omega = image.input_pose.omega;
        double phi = image.input_pose.phi;
        double kappa = image.input_pose.kappa;
        
        // Convert to degrees if needed
        if (image.input_pose.angle_unit == insight::database::InputPose::AngleUnit::kRadians) {
          omega *= 180.0 / M_PI;
          phi *= 180.0 / M_PI;
          kappa *= 180.0 / M_PI;
        }
        
        img["imu"] = {
          {"roll", omega},
          {"pitch", phi},
          {"yaw", kappa},
          {"cov_att_xx", 0.1},
          {"cov_att_yy", 0.1},
          {"cov_att_zz", 0.1}
        };
        imu_count++;
      }
      
      output["images"].push_back(std::move(img));
      ++image_index;
    }
  }
  
  // Add metadata
  output["metadata"] = {
    {"format_version", "2.0"},
    {"exported_from", task.task_name},
    {"task_id", task.task_id},
    {"task_uuid", task.id},
    {"exported_at", std::chrono::system_clock::now().time_since_epoch().count()},
    {"coordinate_system", task.input_snapshot.input_coordinate_system.definition.empty()
                            ? "Unknown"
                            : task.input_snapshot.input_coordinate_system.definition},
    {"num_groups_exported", task.input_snapshot.image_groups.size()},
    {"num_gnss_images", gnss_count},
    {"num_imu_images", imu_count}
  };
  
  // Write to file
  QString images_json_path = QDir(work_dir).filePath("images_all.json");
  std::ofstream out_file(images_json_path.toStdString());
  
  if (!out_file.is_open()) {
    LOG(ERROR) << "Failed to open images_all.json for writing: " << images_json_path.toStdString();
    return false;
  }
  
  out_file << output.dump(2);
  out_file.close();
  
  m_sfmLogTextEdit->appendPlainText(
      QString("✓ Generated images_all.json (%1 images, %2 cameras)")
      .arg(image_index)
      .arg(cameras_arr.size()));
  LOG(INFO) << "Generated images_all.json with " << image_index << " images and "
            << cameras_arr.size() << " cameras";
  
  return true;
}

}  // namespace ui
}  // namespace insight

