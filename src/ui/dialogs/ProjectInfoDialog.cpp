/**
 * @file ProjectInfoDialog.cpp
 * @brief 项目信息对话框实现
 */

#include "ProjectInfoDialog.h"
#include "../widgets/SpatialReferenceDialog.h"
#include "../../database/database_types.h"
#include "../../Common/string_utils.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QLabel>
#include <QPushButton>
#include <QTabWidget>
#include <QDialogButtonBox>
#include <QDateTime>
#include <chrono>
#include <glog/logging.h>

namespace insight {
namespace ui {

ProjectInfoDialog::ProjectInfoDialog(insight::database::Project* project, QWidget* parent)
    : QDialog(parent), m_project(project)
{
    if (!m_project) {
        return;
    }

    setWindowTitle(QString::fromStdString(m_project->name) + " - Project Information");
    setMinimumWidth(600);
    setMinimumHeight(500);
    initializeUI();
    updateDisplay();
}

ProjectInfoDialog::~ProjectInfoDialog()
{
}

void ProjectInfoDialog::initializeUI()
{
    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    // 创建标签页
    m_tabWidget = new QTabWidget();

    // ─────────────────────────────────────────────────────
    // 基本信息标签页
    // ─────────────────────────────────────────────────────
    QWidget* basicInfoWidget = new QWidget();
    QFormLayout* basicInfoLayout = new QFormLayout(basicInfoWidget);

    // 项目名称
    QHBoxLayout* projectNameLayout = new QHBoxLayout();
    m_projectNameEdit = new QLineEdit();
    m_projectNameEdit->setReadOnly(true);
    QPushButton* editNameButton = new QPushButton(tr("Edit"));
    connect(editNameButton, &QPushButton::clicked, this, &ProjectInfoDialog::onEditProjectName);
    projectNameLayout->addWidget(m_projectNameEdit);
    projectNameLayout->addWidget(editNameButton);
    basicInfoLayout->addRow(tr("Project Name:"), projectNameLayout);

    // 项目路径
    m_projectPathLabel = new QLabel();
    basicInfoLayout->addRow(tr("Project Path:"), m_projectPathLabel);

    // 创建时间
    QLabel* creationTimeLabel = new QLabel();
    basicInfoLayout->addRow(tr("Created:"), creationTimeLabel);

    // 最后修改时间
    QLabel* modifiedTimeLabel = new QLabel();
    basicInfoLayout->addRow(tr("Modified:"), modifiedTimeLabel);

    // 作者
    QLabel* authorLabel = new QLabel();
    basicInfoLayout->addRow(tr("Author:"), authorLabel);

    // 描述
    m_descriptionEdit = new QPlainTextEdit();
    m_descriptionEdit->setReadOnly(true);
    m_descriptionEdit->setMaximumHeight(150);
    QPushButton* editDescButton = new QPushButton(tr("Edit"));
    connect(editDescButton, &QPushButton::clicked, this, &ProjectInfoDialog::onEditDescription);
    QVBoxLayout* descVLayout = new QVBoxLayout();
    descVLayout->addWidget(m_descriptionEdit);
    descVLayout->addWidget(editDescButton);
    basicInfoLayout->addRow(tr("Description:"), descVLayout);

    basicInfoWidget->setLayout(basicInfoLayout);
    m_tabWidget->addTab(basicInfoWidget, tr("Basic Info"));

    // ─────────────────────────────────────────────────────
    // 坐标系标签页
    // ─────────────────────────────────────────────────────
    QWidget* coordWidget = new QWidget();
    QFormLayout* coordLayout = new QFormLayout(coordWidget);

    // 输入坐标系
    QHBoxLayout* inputCoordLayout = new QHBoxLayout();
    m_inputCoordinateSystemEdit = new QLineEdit();
    m_inputCoordinateSystemEdit->setReadOnly(true);
    QPushButton* setCoordButton = new QPushButton(tr("Set..."));
    connect(setCoordButton, &QPushButton::clicked, this, &ProjectInfoDialog::onSetInputCoordinateSystem);
    inputCoordLayout->addWidget(m_inputCoordinateSystemEdit);
    inputCoordLayout->addWidget(setCoordButton);
    coordLayout->addRow(tr("Input Coordinate System:"), inputCoordLayout);

    // 坐标系 WKT 显示
    m_inputCoordinateSystemLabel = new QLabel();
    m_inputCoordinateSystemLabel->setWordWrap(true);
    m_inputCoordinateSystemLabel->setMaximumHeight(200);
    QGroupBox* wktGroup = new QGroupBox(tr("Well-Known Text (WKT)"));
    QVBoxLayout* wktLayout = new QVBoxLayout(wktGroup);
    wktLayout->addWidget(m_inputCoordinateSystemLabel);
    coordLayout->addRow(wktGroup);

    coordWidget->setLayout(coordLayout);
    m_tabWidget->addTab(coordWidget, tr("Coordinate System"));

    mainLayout->addWidget(m_tabWidget);

    // ─────────────────────────────────────────────────────
    // 底部按钮
    // ─────────────────────────────────────────────────────
    QDialogButtonBox* buttonBox = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(buttonBox, &QDialogButtonBox::accepted, this, &ProjectInfoDialog::onApply);
    connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    mainLayout->addWidget(buttonBox);

    setLayout(mainLayout);

    // 仅显示，不可编辑
    m_isEditing = false;
}

void ProjectInfoDialog::updateDisplay()
{
    if (!m_project) {
        return;
    }

    // 更新项目名称
    m_projectNameEdit->setText(QString::fromStdString(m_project->name));

    // 项目路径暂时留空（没有路径信息）
    m_projectPathLabel->setText(tr("(in memory)"));

    // 创建时间
    QLabel* creationTimeLabel = qobject_cast<QLabel*>(
        qobject_cast<QFormLayout*>(qobject_cast<QWidget*>(m_tabWidget->widget(0))->layout())->itemAt(2, QFormLayout::FieldRole)->widget());
    if (creationTimeLabel) {
        QDateTime dt = QDateTime::fromSecsSinceEpoch(m_project->creation_time);
        creationTimeLabel->setText(dt.toString("yyyy-MM-dd hh:mm:ss"));
    }

    // 最后修改时间
    QLabel* modifiedTimeLabel = qobject_cast<QLabel*>(
        qobject_cast<QFormLayout*>(qobject_cast<QWidget*>(m_tabWidget->widget(0))->layout())->itemAt(3, QFormLayout::FieldRole)->widget());
    if (modifiedTimeLabel) {
        QDateTime dt = QDateTime::fromSecsSinceEpoch(m_project->last_modified_time);
        modifiedTimeLabel->setText(dt.toString("yyyy-MM-dd hh:mm:ss"));
    }

    // 作者
    QLabel* authorLabel = qobject_cast<QLabel*>(
        qobject_cast<QFormLayout*>(qobject_cast<QWidget*>(m_tabWidget->widget(0))->layout())->itemAt(4, QFormLayout::FieldRole)->widget());
    if (authorLabel) {
        authorLabel->setText(QString::fromStdString(m_project->author));
    }

    // 更新描述
    m_descriptionEdit->setPlainText(QString::fromStdString(m_project->description));

    // 更新输入坐标系
    const auto& inputCoordSys = m_project->input_coordinate_system;
    QString coordDisplay = tr("(Not set)");
    QString wktDisplay = tr("(Not set)");
    
    if (!inputCoordSys.definition.empty()) {
        coordDisplay = QString::fromStdString(inputCoordSys.definition);
        wktDisplay = QString::fromStdString(inputCoordSys.definition);
    }
    
    m_inputCoordinateSystemEdit->setText(coordDisplay);
    m_inputCoordinateSystemLabel->setText(wktDisplay);
}

void ProjectInfoDialog::onEditProjectName()
{
    m_projectNameEdit->setReadOnly(false);
    m_projectNameEdit->selectAll();
    m_projectNameEdit->setFocus();
}

void ProjectInfoDialog::onEditDescription()
{
    m_descriptionEdit->setReadOnly(false);
    m_descriptionEdit->setFocus();
}

void ProjectInfoDialog::onSetInputCoordinateSystem()
{
    SpatialReferenceDialog dialog(this);
    if (dialog.exec() == QDialog::Accepted) {
        const auto& coord = dialog.SelectCoordinate();
        if (m_project && !coord.WKT.empty()) {
            m_project->input_coordinate_system.definition = coord.WKT;
            m_project->input_coordinate_system.type = insight::database::CoordinateSystem::Type::kWKT;
            updateDisplay();
        }
    }
}

void ProjectInfoDialog::onApply()
{
    if (!m_project) {
        reject();
        return;
    }

    // 更新项目名称（如果被编辑）
    std::string newName = m_projectNameEdit->text().toStdString();
    if (newName != m_project->name && !newName.empty()) {
        m_project->name = newName;
        m_project->last_modified_time = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
    }

    // 更新描述（如果被编辑）
    std::string newDesc = m_descriptionEdit->toPlainText().toStdString();
    if (newDesc != m_project->description) {
        m_project->description = newDesc;
        m_project->last_modified_time = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
    }

    accept();
}

}  // namespace ui
}  // namespace insight
