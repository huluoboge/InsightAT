/**
 * @file ATTaskPanel.cpp
 * @brief AT 任务编辑面板实现
 */

#include "at_task_panel.h"
#include "../../database/database_types.h"
#include "../models/project_document.h"
#include "bundler_viewer_window.h"

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
  m_sfmStatusLabel(nullptr), m_sfmProgressBar(nullptr), m_sfmProcess(nullptr) {

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
    m_groupTable->setColumnCount(5);
    m_groupTable->setHorizontalHeaderLabels(
      {"Group ID", "Group Name", "Camera Mode", "Images", "Camera Summary"});
    m_groupTable->horizontalHeader()->setStretchLastSection(true);
    m_groupTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
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
  
  // Run button
  m_runSfmButton = new QPushButton("▶ Run SfM Pipeline");
  m_runSfmButton->setMinimumHeight(40);
  m_runSfmButton->setStyleSheet("font-weight: bold; font-size: 11pt;");
  connect(m_runSfmButton, &QPushButton::clicked, this, &ATTaskPanel::on_run_sfm_clicked);
  sfmControlLayout->addRow(m_runSfmButton);
  
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
  m_workDirLabel->setText(resolve_work_directory(*it));
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
                        .arg(resolve_work_directory(task));
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
  const QString timing_path = QDir(resolve_work_directory(task)).filePath("sfm_timing.json");
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

  const QString work_dir = resolve_work_directory(task);
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

  const QString work_dir = resolve_work_directory(task);
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

  const QString work_dir = resolve_work_directory(*task);
  
  // Disable button and show progress
  m_runSfmButton->setEnabled(false);
  m_sfmProgressBar->setVisible(true);
  m_sfmLogTextEdit->clear();
  m_sfmStatusLabel->setText("Running isat_sfm...");

  // Get isat_sfm binary path (assume it's in PATH or same directory as InsightAT)
  QString sfm_bin = "isat_sfm";
  
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
  
  // Prepare arguments: isat_sfm -i <images_dir> -w <work_dir> -v
  // We assume images are in input_snapshot or we just run with defaults
  // For now, we'll just run isat_sfm to process existing work directory
  QStringList arguments;
  arguments << "-w" << work_dir << "-v";
  
  // Connect signals
  connect(m_sfmProcess, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
          this, &ATTaskPanel::on_sfm_process_finished);
  connect(m_sfmProcess, &QProcess::readyReadStandardOutput,
          this, &ATTaskPanel::on_sfm_process_stdout);
  connect(m_sfmProcess, &QProcess::readyReadStandardError,
          this, &ATTaskPanel::on_sfm_process_stderr);

  // Start process
  m_sfmProcess->start(sfm_bin, arguments);
  
  if (!m_sfmProcess->waitForStarted(3000)) {
    m_runSfmButton->setEnabled(true);
    m_sfmProgressBar->setVisible(false);
    m_sfmStatusLabel->setText("Failed to start isat_sfm");
    m_sfmLogTextEdit->appendPlainText("ERROR: Failed to start isat_sfm binary");
    m_sfmLogTextEdit->appendPlainText("Make sure isat_sfm is in PATH or same directory as InsightAT");
    QMessageBox::critical(this, "Error", 
        "Failed to start isat_sfm. Please check if the executable exists in PATH.");
    delete m_sfmProcess;
    m_sfmProcess = nullptr;
    return;
  }

  m_sfmLogTextEdit->appendPlainText(QString("Started: %1").arg(sfm_bin));
  m_sfmLogTextEdit->appendPlainText(QString("Work dir: %1").arg(work_dir));
  m_sfmLogTextEdit->appendPlainText("Detailed logs available in work directory");
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

}  // namespace ui
}  // namespace insight
