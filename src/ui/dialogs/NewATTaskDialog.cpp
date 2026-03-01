/**
 * @file NewATTaskDialog.cpp
 * @brief 新建空三任务对话框实现
 */

#include "NewATTaskDialog.h"
#include "../models/ProjectDocument.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QLineEdit>
#include <QComboBox>
#include <QPushButton>
#include <QLabel>
#include <QString>
#include <glog/logging.h>

namespace insight {
namespace ui {

NewATTaskDialog::NewATTaskDialog(ProjectDocument* document,
                                 const QString& defaultName,
                                 QWidget* parent)
    : QDialog(parent),
      m_document(document),
      m_taskNameEdit(nullptr),
      m_parentTaskCombo(nullptr),
      m_okButton(nullptr),
      m_cancelButton(nullptr) {
    
    setWindowTitle("New AT Task");
    setMinimumWidth(400);
    setModal(true);
    
    initUI();
    
    // 设置默认名称
    if (!defaultName.isEmpty()) {
        m_taskNameEdit->setText(defaultName);
    }
    
    // 加载父任务列表
    loadParentTasks();
}

NewATTaskDialog::~NewATTaskDialog() = default;

std::string NewATTaskDialog::getTaskName() const {
    return m_taskNameEdit->text().toStdString();
}

uint32_t NewATTaskDialog::getParentTaskIndex() const {
    int index = m_parentTaskCombo->currentIndex();
    if (index <= 0) {  // index 0 is "None"
        return static_cast<uint32_t>(-1);
    }
    
    // m_taskList[0] corresponds to m_parentTaskCombo index 1 (after "None")
    // Return the index which is index - 1
    return static_cast<uint32_t>(index - 1);
}

void NewATTaskDialog::initUI() {
    // 主布局
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    
    // 表单布局
    QFormLayout* formLayout = new QFormLayout();
    
    // 任务名称
    m_taskNameEdit = new QLineEdit();
    m_taskNameEdit->setPlaceholderText("e.g., AT_2");
    formLayout->addRow("Task Name:", m_taskNameEdit);
    
    // 父任务选择
    m_parentTaskCombo = new QComboBox();
    m_parentTaskCombo->addItem("None");  // 索引 0
    formLayout->addRow("Parent Task:", m_parentTaskCombo);
    
    mainLayout->addLayout(formLayout);
    
    // 按钮布局
    QHBoxLayout* buttonLayout = new QHBoxLayout();
    buttonLayout->addStretch();
    
    m_okButton = new QPushButton("OK");
    m_cancelButton = new QPushButton("Cancel");
    
    connect(m_okButton, &QPushButton::clicked, this, &NewATTaskDialog::onOKClicked);
    connect(m_cancelButton, &QPushButton::clicked, this, &NewATTaskDialog::onCancelClicked);
    
    buttonLayout->addWidget(m_okButton);
    buttonLayout->addWidget(m_cancelButton);
    
    mainLayout->addLayout(buttonLayout);
    mainLayout->addStretch();
}

void NewATTaskDialog::loadParentTasks() {
    if (!m_document) {
        return;
    }
    
    const auto& project = m_document->project();
    m_taskList.clear();
    
    // 添加所有现存的 AT Tasks 作为可选父任务
    for (const auto& task : project.at_tasks) {
        QString displayName = QString::fromStdString(task.task_name);
        m_parentTaskCombo->addItem(displayName);
        m_taskList.push_back(std::make_pair(displayName, task.id));
    }
}

void NewATTaskDialog::onOKClicked() {
    if (m_taskNameEdit->text().isEmpty()) {
        LOG(WARNING) << "Task name is empty";
        return;
    }
    
    accept();
}

void NewATTaskDialog::onCancelClicked() {
    reject();
}

}  // namespace ui
}  // namespace insight
