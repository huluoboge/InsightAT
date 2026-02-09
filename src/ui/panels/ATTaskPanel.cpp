/**
 * @file ATTaskPanel.cpp
 * @brief AT 任务编辑面板实现
 */

#include "ATTaskPanel.h"
#include "../models/ProjectDocument.h"
#include "../../database/database_types.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QTabWidget>
#include <QPushButton>
#include <QLineEdit>
#include <QLabel>
#include <QGroupBox>
#include <QTextEdit>
#include <glog/logging.h>

namespace insight {
namespace ui {

ATTaskPanel::ATTaskPanel(ProjectDocument* document, QWidget* parent)
    : QWidget(parent),
      m_document(document),
      m_currentTaskId(""),
      m_tabWidget(nullptr),
      m_inputDataTab(nullptr),
      m_optimizationTab(nullptr),
      m_exportTab(nullptr),
      m_taskNameEdit(nullptr),
      m_taskIdLabel(nullptr),
      m_parentTaskLabel(nullptr),
      m_statusLabel(nullptr),
      m_exportButton(nullptr) {
    
    setWindowTitle("AT Task Editor");
    initUI();
}

ATTaskPanel::~ATTaskPanel() = default;

bool ATTaskPanel::loadTask(const std::string& task_id) {
    if (!m_document) {
        LOG(ERROR) << "ProjectDocument is null";
        return false;
    }
    
    const auto& project = m_document->project();
    auto it = std::find_if(project.at_tasks.begin(),
                          project.at_tasks.end(),
                          [&task_id](const auto& t) { return t.id == task_id; });
    
    if (it == project.at_tasks.end()) {
        LOG(ERROR) << "AT task not found: " << task_id;
        return false;
    }
    
    m_currentTaskId = task_id;
    refreshUI();
    return true;
}

std::string ATTaskPanel::getCurrentTaskId() const {
    return m_currentTaskId;
}

void ATTaskPanel::initUI() {
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
    
    mainLayout->addWidget(infoGroup);
    
    // ─── 中间：Tab 部件
    m_tabWidget = new QTabWidget();
    
    // Tab 1: Input Data
    m_inputDataTab = new QWidget();
    QVBoxLayout* inputLayout = new QVBoxLayout(m_inputDataTab);
    QLabel* inputPlaceholder = new QLabel("Input Data Tab - To be implemented\n\n"
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
    
    // Tab 3: Export
    m_exportTab = new QWidget();
    QVBoxLayout* exportLayout = new QVBoxLayout(m_exportTab);
    
    // 在 Export Tab 内部添加 Export 按钮
    m_exportButton = new QPushButton("Export to COLMAP");
    connect(m_exportButton, &QPushButton::clicked, this, &ATTaskPanel::onExportClicked);
    exportLayout->addWidget(m_exportButton);
    
    QLabel* exportPlaceholder = new QLabel("\nExport Tab - To be implemented\n\n"
                                           "This tab will display export options for COLMAP/subprocess.");
    exportPlaceholder->setStyleSheet("color: gray;");
    exportLayout->addWidget(exportPlaceholder);
    exportLayout->addStretch();
    
    m_tabWidget->addTab(m_exportTab, "Export");
    
    mainLayout->addWidget(m_tabWidget);
    
    // ─── 自动保存：监听任务名称变更
    connect(m_taskNameEdit, &QLineEdit::textChanged, this, &ATTaskPanel::onTaskNameChanged);
}

void ATTaskPanel::refreshUI() {
    if (m_currentTaskId.empty() || !m_document) {
        m_taskNameEdit->clear();
        m_taskIdLabel->setText("");
        m_parentTaskLabel->setText("");
        m_statusLabel->setText("No task loaded");
        return;
    }
    
    const auto& project = m_document->project();
    auto it = std::find_if(project.at_tasks.begin(),
                          project.at_tasks.end(),
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
}

void ATTaskPanel::onExportClicked() {
    if (m_currentTaskId.empty() || !m_document) {
        LOG(WARNING) << "No task loaded";
        return;
    }
    
    // TODO: 在 Phase 5 中实现，打开导出对话框和 COLMAP 子进程
    LOG(INFO) << "Export button clicked for task: " << m_currentTaskId;
    m_statusLabel->setText("Exporting...");
}

void ATTaskPanel::onTaskNameChanged() {
    // 监听任务名称的实时变更，立即保存
    if (m_currentTaskId.empty() || !m_document) {
        return;
    }
    
    saveTask();
}

void ATTaskPanel::saveTask() {
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
