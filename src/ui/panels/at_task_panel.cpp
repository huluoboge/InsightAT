/**
 * @file ATTaskPanel.cpp
 * @brief AT 任务编辑面板实现
 */

#include "at_task_panel.h"
#include "../../database/database_types.h"
#include "../models/project_document.h"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QProcess>
#include <QPushButton>
#include <QTabWidget>
#include <QCheckBox>
#include <QPlainTextEdit>
#include <QTextEdit>
#include <QVBoxLayout>
#include <glog/logging.h>
#include <QLabel>

namespace insight {
namespace ui {

ATTaskPanel::ATTaskPanel(ProjectDocument* document, QWidget* parent)
    : QWidget(parent), m_document(document), m_currentTaskId(""), m_tabWidget(nullptr),
      m_inputDataTab(nullptr), m_optimizationTab(nullptr), m_exportTab(nullptr),
  m_taskNameEdit(nullptr), m_taskIdLabel(nullptr), m_parentTaskLabel(nullptr),
  m_statusLabel(nullptr), m_exportButton(nullptr), m_runButton(nullptr),
  m_chkExtract(nullptr), m_chkMatch(nullptr), m_chkIncremental(nullptr), m_logView(nullptr),
  m_viewerWidget(nullptr), m_pendingSteps(), m_currentStepIndex(0), m_currentProcess(nullptr) {

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

  m_taskIdLabel = new QLabel("");
  m_taskIdLabel->setStyleSheet("color: gray; font-family: monospace; font-size: 9pt;");
  infoLayout->addRow("Task ID:", m_taskIdLabel);

  m_parentTaskLabel = new QLabel("");
  infoLayout->addRow("Parent Task:", m_parentTaskLabel);

  m_statusLabel = new QLabel("Ready");
  infoLayout->addRow("Status:", m_statusLabel);

  // Run controls: checkboxes + Run button placed under Task Information
  QHBoxLayout* runControlsLayout = new QHBoxLayout();
  m_chkExtract = new QCheckBox("Feature Extraction");
  m_chkMatch = new QCheckBox("Feature Matching");
  m_chkIncremental = new QCheckBox("Incremental Reconstruction");
  runControlsLayout->addWidget(m_chkExtract);
  runControlsLayout->addWidget(m_chkMatch);
  runControlsLayout->addWidget(m_chkIncremental);

  m_runButton = new QPushButton("Run");
  m_runButton->setMinimumHeight(30);
  runControlsLayout->addWidget(m_runButton);
  infoLayout->addRow(runControlsLayout);

  mainLayout->addWidget(infoGroup);

  // ─── 中间：Tab 部件
  m_tabWidget = new QTabWidget();

  // Tab 1: Input Data
  m_inputDataTab = new QWidget();
  QVBoxLayout* inputLayout = new QVBoxLayout(m_inputDataTab);
  QLabel* inputPlaceholder =
      new QLabel("Input Data Tab - To be implemented\n\n"
                 "This tab will display images and cameras from InputSnapshot.");
  inputPlaceholder->setStyleSheet("color: gray;");
  inputLayout->addWidget(inputPlaceholder);
  inputLayout->addStretch();
  m_tabWidget->addTab(m_inputDataTab, "Input Data");

  // Tab 2: Optimization
  m_optimizationTab = new QWidget();
  QVBoxLayout* optLayout = new QVBoxLayout(m_optimizationTab);
  QLabel* optPlaceholder = new QLabel("Optimization Tab - To be implemented\n\n"
                                      "This tab will display camera optimization parameters.");
  optPlaceholder->setStyleSheet("color: gray;");
  optLayout->addWidget(optPlaceholder);
  optLayout->addStretch();
  m_tabWidget->addTab(m_optimizationTab, "Optimization");

  // Tab: Viewer (task-scoped viewer)
  m_viewerWidget = new QWidget();
  QVBoxLayout* viewerLayout = new QVBoxLayout(m_viewerWidget);
  QLabel* viewerPlaceholder = new QLabel("3D Viewer (placeholder)");
  viewerPlaceholder->setStyleSheet("color: gray;");
  viewerLayout->addWidget(viewerPlaceholder);
  viewerLayout->addStretch();
  m_tabWidget->addTab(m_viewerWidget, "Viewer");

  // Tab 3: Export
  m_exportTab = new QWidget();
  QVBoxLayout* exportLayout = new QVBoxLayout(m_exportTab);

  // SIFT GPU 特征提取按钮
  m_siftGPUButton = new QPushButton("Run SIFT GPU Feature Extraction");
  m_siftGPUButton->setMinimumHeight(40);
  m_siftGPUButton->setStyleSheet(
      "QPushButton { background-color: #4CAF50; color: white; font-weight: bold; }");
  connect(m_siftGPUButton, &QPushButton::clicked, this, &ATTaskPanel::on_run_sift_gpu_clicked);
  exportLayout->addWidget(m_siftGPUButton);

  exportLayout->addSpacing(20);

  // Export 按钮
  m_exportButton = new QPushButton("Export to COLMAP");
  connect(m_exportButton, &QPushButton::clicked, this, &ATTaskPanel::on_export_clicked);
  exportLayout->addWidget(m_exportButton);

  QLabel* exportPlaceholder =
      new QLabel("\nExport Tab - To be implemented\n\n"
                 "This tab will display export options for COLMAP/subprocess.");
  exportPlaceholder->setStyleSheet("color: gray;");
  exportLayout->addWidget(exportPlaceholder);
  exportLayout->addStretch();

  m_tabWidget->addTab(m_exportTab, "Export");

  mainLayout->addWidget(m_tabWidget);

  // Log output below tabs
  m_logView = new QPlainTextEdit();
  m_logView->setReadOnly(true);
  m_logView->setMaximumHeight(200);
  mainLayout->addWidget(m_logView);

  // ─── 自动保存：监听任务名称变更
  connect(m_taskNameEdit, &QLineEdit::textChanged, this, &ATTaskPanel::on_task_name_changed);
  // Run button
  connect(m_runButton, &QPushButton::clicked, this, &ATTaskPanel::on_run_selected_steps_clicked);
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
  m_taskNameEdit->setText(QString::fromStdString(it->task_name));
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
  // Update viewer placeholder (could be replaced with AT3dRenderWidget later)
  if (m_viewerWidget) {
    // no-op for now; real viewer refresh will be added when integrating AT3dRenderWidget
  }
}

void ATTaskPanel::on_export_clicked() {
  if (m_currentTaskId.empty() || !m_document) {
    LOG(WARNING) << "No task loaded";
    return;
  }

  // TODO: 在 Phase 5 中实现，打开导出对话框和 COLMAP 子进程
  LOG(INFO) << "Export button clicked for task: " << m_currentTaskId;
  m_statusLabel->setText("Exporting...");
}

void ATTaskPanel::on_run_sift_gpu_clicked() {
  if (m_currentTaskId.empty() || !m_document) {
    QMessageBox::warning(this, "Warning", "No task loaded");
    return;
  }

  const auto* task = m_document->getATTaskById(m_currentTaskId);
  if (!task) {
    QMessageBox::critical(this, "Error", "Task not found");
    return;
  }

  // 准备工作目录
  QString workDir =
      QDir::homePath() + "/.insightat/tasks/" + QString::fromStdString(m_currentTaskId);
  QDir().mkpath(workDir);

  QString featuresDir = workDir + "/features";
  QString featuresForRetrievalDir = workDir + "/features_for_retrieval";
  QDir().mkpath(featuresDir);
  QDir().mkpath(featuresForRetrievalDir);

  // 生成 image_list.json
  QString imageListPath = workDir + "/image_list.json";
  QJsonObject root;
  QJsonArray images;

  for (const auto& group : task->input_snapshot.image_groups) {
    for (const auto& img : group.images) {
      QJsonObject imgObj;
      imgObj["path"] = QString::fromStdString(img.filename);
      imgObj["camera_id"] = 1; // Default camera ID
      imgObj["id"] = int(img.image_id);
      images.append(imgObj);
    }
  }

  root["images"] = images;

  // 写入 JSON 文件
  QFile file(imageListPath);
  if (!file.open(QIODevice::WriteOnly)) {
    QMessageBox::critical(this, "Error", "Failed to create image list file");
    return;
  }

  file.write(QJsonDocument(root).toJson());
  file.close();

  LOG(INFO) << "Created image list: " << imageListPath.toStdString() << " with " << images.size()
            << " images";

  // 构建 isat_extract 命令
  QString program = QCoreApplication::applicationDirPath() + "/isat_extract";
  QStringList arguments;
  arguments << "-i" << imageListPath << "-o" << featuresDir << "-n"
            << "8000"
            << "--output-retrieval" << featuresForRetrievalDir << "--nms"
            << "--uint8"
            << "-v"; // verbose

  LOG(INFO) << "Running command: " << program.toStdString() << " "
            << arguments.join(" ").toStdString();

  // 禁用按钮并显示进度
  m_siftGPUButton->setEnabled(false);
  m_siftGPUButton->setText("Running SIFT GPU...");
  m_statusLabel->setText("Extracting features...");

  // 创建 QProcess
  QProcess* process = new QProcess(this);
  process->setWorkingDirectory(workDir);

  // 连接输出信号
  connect(process, &QProcess::readyReadStandardOutput, [process, this]() {
    QString output = process->readAllStandardOutput();
    LOG(INFO) << "[SIFT GPU] " << output.toStdString();
  });

  connect(process, &QProcess::readyReadStandardError, [process, this]() {
    QString error = process->readAllStandardError();
    // 解析进度
    if (error.contains("PROGRESS:")) {
      QStringList lines = error.split('\n');
      for (const QString& line : lines) {
        if (line.contains("PROGRESS:")) {
          QString progressStr = line.split("PROGRESS:").last().trimmed();
          bool ok;
          float progress = progressStr.toFloat(&ok);
          if (ok) {
            int percent = static_cast<int>(progress * 100);
            m_statusLabel->setText(QString("Extracting features... %1%").arg(percent));
          }
        }
      }
    }
    LOG(INFO) << "[SIFT GPU] " << error.toStdString();
  });

  // 连接完成信号
  connect(process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
          [this, process, featuresDir](int exitCode, QProcess::ExitStatus exitStatus) {
            m_siftGPUButton->setEnabled(true);
            m_siftGPUButton->setText("Run SIFT GPU Feature Extraction");

            if (exitStatus == QProcess::NormalExit && exitCode == 0) {
              m_statusLabel->setText("Feature extraction completed successfully");

              // 统计生成的特征文件数量
              QDir dir(featuresDir);
              int featureCount = dir.entryList(QStringList() << "*.isat_feat", QDir::Files).count();

              QMessageBox::information(this, "Success",
                                       QString("SIFT GPU feature extraction completed!\n\n"
                                               "Generated %1 feature files in:\n%2")
                                           .arg(featureCount)
                                           .arg(featuresDir));

              LOG(INFO) << "SIFT GPU completed successfully. " << featureCount
                        << " feature files generated.";
            } else {
              m_statusLabel->setText("Feature extraction failed");
              QMessageBox::critical(this, "Error",
                                    QString("SIFT GPU failed with exit code %1").arg(exitCode));
              LOG(ERROR) << "SIFT GPU failed with exit code: " << exitCode;
            }

            process->deleteLater();
          });

  // 启动进程
  process->start(program, arguments);

  if (!process->waitForStarted(3000)) {
    m_siftGPUButton->setEnabled(true);
    m_siftGPUButton->setText("Run SIFT GPU Feature Extraction");
    m_statusLabel->setText("Failed to start");
    QMessageBox::critical(this, "Error",
                          "Failed to start isat_extract. Please check if the executable exists.");
    process->deleteLater();
    return;
  }

  LOG(INFO) << "SIFT GPU process started successfully";
}

void ATTaskPanel::on_run_selected_steps_clicked() {
  if (m_currentTaskId.empty() || !m_document) {
    QMessageBox::warning(this, "Warning", "No task loaded");
    return;
  }

  m_pendingSteps.clear();
  if (m_chkExtract && m_chkExtract->isChecked())
    m_pendingSteps.append(0);
  if (m_chkMatch && m_chkMatch->isChecked())
    m_pendingSteps.append(1);
  if (m_chkIncremental && m_chkIncremental->isChecked())
    m_pendingSteps.append(2);

  if (m_pendingSteps.isEmpty()) {
    QMessageBox::information(this, "No steps selected", "Please select at least one step to run.");
    return;
  }

  // Disable controls while running
  if (m_runButton)
    m_runButton->setEnabled(false);
  if (m_chkExtract)
    m_chkExtract->setEnabled(false);
  if (m_chkMatch)
    m_chkMatch->setEnabled(false);
  if (m_chkIncremental)
    m_chkIncremental->setEnabled(false);

  m_currentStepIndex = 0;
  append_log("Starting run sequence...");
  start_next_selected_step();
}

void ATTaskPanel::start_next_selected_step() {
  if (m_currentStepIndex >= m_pendingSteps.size()) {
    append_log("All selected steps finished.");
    // Re-enable controls
    if (m_runButton)
      m_runButton->setEnabled(true);
    if (m_chkExtract)
      m_chkExtract->setEnabled(true);
    if (m_chkMatch)
      m_chkMatch->setEnabled(true);
    if (m_chkIncremental)
      m_chkIncremental->setEnabled(true);
    m_currentProcess = nullptr;
    m_statusLabel->setText("Ready");
    return;
  }

  int step = m_pendingSteps.at(m_currentStepIndex);
  QString stepName;
  switch (step) {
    case 0:
      stepName = "Feature Extraction";
      break;
    case 1:
      stepName = "Feature Matching";
      break;
    case 2:
      stepName = "Incremental Reconstruction";
      break;
    default:
      stepName = "Unknown";
  }

  append_log(QString("Starting step %1 (%2)").arg(m_currentStepIndex + 1).arg(stepName));
  launch_process_for_step(step);
}

void ATTaskPanel::launch_process_for_step(int step_index) {
  if (m_currentTaskId.empty() || !m_document) {
    append_log("No task loaded for running step");
    return;
  }

  const auto* task = m_document->getATTaskById(m_currentTaskId);
  if (!task) {
    append_log("Task not found when launching step");
    return;
  }

  QString workDir = QDir::homePath() + "/.insightat/tasks/" + QString::fromStdString(m_currentTaskId);
  QDir().mkpath(workDir);

  // Prepare common paths
  QString featuresDir = workDir + "/features";
  QString featuresForRetrievalDir = workDir + "/features_for_retrieval";
  QDir().mkpath(featuresDir);
  QDir().mkpath(featuresForRetrievalDir);

  QString program;
  QStringList arguments;

  if (step_index == 0) {
    // Feature extraction (reuse logic from on_run_sift_gpu_clicked)
    QString imageListPath = workDir + "/image_list.json";
    QJsonObject root;
    QJsonArray images;
    for (const auto& group : task->input_snapshot.image_groups) {
      for (const auto& img : group.images) {
        QJsonObject imgObj;
        imgObj["path"] = QString::fromStdString(img.filename);
        imgObj["camera_id"] = 1;
        imgObj["id"] = int(img.image_id);
        images.append(imgObj);
      }
    }
    root["images"] = images;
    QFile file(imageListPath);
    if (file.open(QIODevice::WriteOnly)) {
      file.write(QJsonDocument(root).toJson());
      file.close();
      append_log(QString("Wrote image list: %1 (%2 images)").arg(imageListPath).arg(images.size()));
    } else {
      append_log("Failed to write image list for extraction");
    }

    program = QCoreApplication::applicationDirPath() + "/isat_extract";
    arguments << "-i" << imageListPath << "-o" << featuresDir << "-n" << "8000"
              << "--output-retrieval" << featuresForRetrievalDir << "--nms" << "--uint8" << "-v";
    m_statusLabel->setText("Extracting features...");
  } else if (step_index == 1) {
    // Feature matching: expects pairs.json in workDir (may be created by retrieval)
    QString pairsPath = workDir + "/pairs.json";
    if (!QFileInfo::exists(pairsPath)) {
      append_log("pairs.json not found; skipping feature matching step. Consider running retrieval first.");
      m_currentStepIndex++;
      start_next_selected_step();
      return;
    }
    QString matchDir = workDir + "/match";
    QDir().mkpath(matchDir);
    program = QCoreApplication::applicationDirPath() + "/isat_match";
    arguments << "-i" << pairsPath << "-f" << featuresDir << "-o" << matchDir << "--match-backend" << "cuda";
    m_statusLabel->setText("Matching features...");
  } else if (step_index == 2) {
    // Incremental SfM: expects tracks and geo/pairs produced earlier
    QString tracksPath = workDir + "/tracks.isat_tracks";
    QString pairsPath = workDir + "/pairs.json";
    QString geoDir = workDir + "/geo";
    QString outputDir = workDir + "/incremental_out";
    if (!QFileInfo::exists(tracksPath) || !QFileInfo::exists(pairsPath) || !QFileInfo::exists(geoDir)) {
      append_log("Missing inputs for incremental SfM (tracks/pairs/geo). Skipping incremental step.");
      m_currentStepIndex++;
      start_next_selected_step();
      return;
    }
    QDir().mkpath(outputDir);
    program = QCoreApplication::applicationDirPath() + "/isat_incremental_sfm";
    arguments << "-t" << tracksPath << "-p" << m_document->filepath() << "-m" << pairsPath << "-g" << geoDir << "-o" << outputDir;
    m_statusLabel->setText("Running incremental SfM...");
  } else {
    append_log("Unknown step index requested");
    m_currentStepIndex++;
    start_next_selected_step();
    return;
  }

  // Start QProcess
  QProcess* process = new QProcess(this);
  process->setWorkingDirectory(workDir);
  m_currentProcess = process;

  connect(process, &QProcess::readyReadStandardOutput, [process, this]() {
    QString out = process->readAllStandardOutput();
    append_log(out);
  });
  connect(process, &QProcess::readyReadStandardError, [process, this]() {
    QString err = process->readAllStandardError();
    append_log(err);
  });

  connect(process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
          [this, process](int exitCode, QProcess::ExitStatus status) {
            if (status == QProcess::NormalExit && exitCode == 0) {
              append_log(QString("Step finished successfully (exit %1)").arg(exitCode));
            } else {
              append_log(QString("Step failed (exit %1)").arg(exitCode));
            }
            process->deleteLater();
            m_currentProcess = nullptr;
            m_currentStepIndex++;
            start_next_selected_step();
          });

  append_log(QString("Launching: %1 %2").arg(program).arg(arguments.join(' ')));
  process->start(program, arguments);
  if (!process->waitForStarted(3000)) {
    append_log("Failed to start process: " + program);
    process->deleteLater();
    m_currentProcess = nullptr;
    m_currentStepIndex++;
    start_next_selected_step();
    return;
  }
}

void ATTaskPanel::append_log(const QString& text) {
  if (!m_logView)
    return;
  m_logView->appendPlainText(text.trimmed());
}

void ATTaskPanel::on_task_name_changed() {
  // 监听任务名称的实时变更，立即保存
  if (m_currentTaskId.empty() || !m_document) {
    return;
  }

  save_task();
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

  // 通过 ProjectDocument 保存更新
  m_document->updateATTask(m_currentTaskId, *task);

  m_statusLabel->setText("Saved");
  LOG(INFO) << "AT task auto-saved: " << m_currentTaskId;
}

}  // namespace ui
}  // namespace insight
