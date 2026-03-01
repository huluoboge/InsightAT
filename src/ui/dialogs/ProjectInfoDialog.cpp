/**
 * @file ProjectInfoDialog.cpp
 * @brief 项目信息对话框实现
 * 
 * 新设计特点：
 * - 分段式布局，取代了老的标签页设计
 * - 清晰的视觉层次和编辑流程
 * - 更好的空间利用和可读性
 */

#include "ProjectInfoDialog.h"
#include "../../Common/string_utils.h"
#include "../../database/database_types.h"
#include "../widgets/SpatialReferenceDialog.h"
#include "CoordinateSystemConfigDialog.h"

#include <QDateTime>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QVBoxLayout>
#include <QStyle>
#include <QApplication>
#include <chrono>
#include <glog/logging.h>

namespace insight {
namespace ui {

    ProjectInfoDialog::ProjectInfoDialog(insight::database::Project* project, QWidget* parent)
        : QDialog(parent)
        , m_project(project)
    {
        if (!m_project) {
            return;
        }

        setWindowTitle(QString::fromStdString(m_project->name) + " - Project Information");
        setMinimumWidth(700);
        setMinimumHeight(650);
        initializeUI();
        updateDisplay();
    }

    ProjectInfoDialog::~ProjectInfoDialog()
    {
    }

    QWidget* ProjectInfoDialog::createSeparator()
    {
        QWidget* separator = new QWidget();
        separator->setFixedHeight(1);
        separator->setStyleSheet("background-color: #E0E0E0;");
        return separator;
    }

    void ProjectInfoDialog::initializeUI()
    {
        QVBoxLayout* mainLayout = new QVBoxLayout(this);
        mainLayout->setSpacing(0);
        mainLayout->setContentsMargins(0, 0, 0, 0);

        // ─────────────────────────────────────────────────────
        // 可滚动内容区域
        // ─────────────────────────────────────────────────────
        QScrollArea* scrollArea = new QScrollArea();
        scrollArea->setWidgetResizable(true);
        scrollArea->setStyleSheet("QScrollArea { border: none; }");

        QWidget* contentWidget = new QWidget();
        QVBoxLayout* contentLayout = new QVBoxLayout(contentWidget);
        contentLayout->setSpacing(16);
        contentLayout->setContentsMargins(20, 20, 20, 20);

        // ─────────────────────────────────────────────────────
        // 第一段：基本信息
        // ─────────────────────────────────────────────────────
        QGroupBox* basicInfoGroup = new QGroupBox(tr("Basic Information"));
        QFormLayout* basicLayout = new QFormLayout(basicInfoGroup);

        // 项目名称
        QHBoxLayout* nameLayout = new QHBoxLayout();
        m_projectNameLabel = new QLabel();
        m_projectNameLabel->setOpenExternalLinks(false);
        m_projectNameLabel->setWordWrap(true);
        m_projectNameEdit = new QLineEdit();
        m_projectNameEdit->setVisible(false);
        m_editNameButton = new QPushButton(tr("Edit"));
        m_editNameButton->setMaximumWidth(80);
        connect(m_editNameButton, &QPushButton::clicked, this, &ProjectInfoDialog::onEditProjectName);
        nameLayout->addWidget(m_projectNameLabel);
        nameLayout->addWidget(m_projectNameEdit, 1);
        nameLayout->addWidget(m_editNameButton);
        basicLayout->addRow(tr("Project Name:"), nameLayout);

        // 创建时间
        m_creationTimeLabel = new QLabel();
        basicLayout->addRow(tr("Created:"), m_creationTimeLabel);

        // 最后修改时间
        m_modifiedTimeLabel = new QLabel();
        basicLayout->addRow(tr("Modified:"), m_modifiedTimeLabel);

        // 作者
        m_authorLabel = new QLabel();
        basicLayout->addRow(tr("Author:"), m_authorLabel);

        contentLayout->addWidget(basicInfoGroup);

        // ─────────────────────────────────────────────────────
        // 分隔符
        // ─────────────────────────────────────────────────────
        contentLayout->addWidget(createSeparator());

        // ─────────────────────────────────────────────────────
        // 第二段：坐标系设置
        // ─────────────────────────────────────────────────────
        QGroupBox* coordGroup = new QGroupBox(tr("Coordinate System Configuration"));
        QVBoxLayout* coordLayout = new QVBoxLayout(coordGroup);
        coordLayout->setSpacing(12);

        // 坐标系类型显示
        QHBoxLayout* coordTypeLayout = new QHBoxLayout();
        coordTypeLayout->addWidget(new QLabel(tr("Type:")));
        m_inputCoordTypeLabel = new QLabel(tr("(Not set)"));
        m_inputCoordTypeLabel->setStyleSheet("color: #666666; font-weight: bold;");
        coordTypeLayout->addWidget(m_inputCoordTypeLabel);
        coordTypeLayout->addStretch();
        coordLayout->addLayout(coordTypeLayout);

        // 坐标系定义显示
        QLabel* defLabel = new QLabel(tr("Definition:"));
        coordLayout->addWidget(defLabel);
        m_inputCoordDefLabel = new QLabel(tr("(Not set)"));
        m_inputCoordDefLabel->setWordWrap(true);
        m_inputCoordDefLabel->setStyleSheet("padding: 8px; background-color: #F5F5F5; border-radius: 3px;");
        m_inputCoordDefLabel->setMinimumHeight(60);
        coordLayout->addWidget(m_inputCoordDefLabel);

        // 设置/修改按钮
        m_setCoordButton = new QPushButton(tr("Configure Coordinate System"));
        m_setCoordButton->setMinimumHeight(36);
        connect(m_setCoordButton, &QPushButton::clicked, this, &ProjectInfoDialog::onSetInputCoordinateSystem);
        coordLayout->addWidget(m_setCoordButton);

        contentLayout->addWidget(coordGroup);

        // ─────────────────────────────────────────────────────
        // 分隔符
        // ─────────────────────────────────────────────────────
        contentLayout->addWidget(createSeparator());

        // ─────────────────────────────────────────────────────
        // 第三段：描述
        // ─────────────────────────────────────────────────────
        QGroupBox* descGroup = new QGroupBox(tr("Description"));
        QVBoxLayout* descLayout = new QVBoxLayout(descGroup);

        m_descriptionLabel = new QLabel();
        m_descriptionLabel->setWordWrap(true);
        m_descriptionLabel->setVisible(false);
        m_descriptionLabel->setMinimumHeight(80);
        m_descriptionLabel->setStyleSheet("padding: 8px; background-color: #F5F5F5; border-radius: 3px;");

        m_descriptionEdit = new QPlainTextEdit();
        m_descriptionEdit->setMinimumHeight(80);
        m_descriptionEdit->setVisible(false);

        QHBoxLayout* descButtonLayout = new QHBoxLayout();
        m_editDescButton = new QPushButton(tr("Edit"));
        m_editDescButton->setMaximumWidth(80);
        connect(m_editDescButton, &QPushButton::clicked, this, &ProjectInfoDialog::onEditDescription);
        descButtonLayout->addWidget(m_descriptionLabel, 1);
        descButtonLayout->addWidget(m_descriptionEdit, 1);
        descButtonLayout->addWidget(m_editDescButton);

        descLayout->addLayout(descButtonLayout);
        contentLayout->addWidget(descGroup);

        // 添加弹性空间
        contentLayout->addStretch();

        scrollArea->setWidget(contentWidget);
        mainLayout->addWidget(scrollArea);

        // ─────────────────────────────────────────────────────
        // 底部按钮区
        // ─────────────────────────────────────────────────────
        mainLayout->addWidget(createSeparator());

        QHBoxLayout* buttonLayout = new QHBoxLayout();
        buttonLayout->setContentsMargins(20, 12, 20, 12);

        m_saveButton = new QPushButton(tr("Save"));
        m_saveButton->setMinimumWidth(100);
        m_cancelButton = new QPushButton(tr("Close"));
        m_cancelButton->setMinimumWidth(100);

        buttonLayout->addStretch();
        buttonLayout->addWidget(m_saveButton);
        buttonLayout->addWidget(m_cancelButton);

        connect(m_saveButton, &QPushButton::clicked, this, &ProjectInfoDialog::onApply);
        connect(m_cancelButton, &QPushButton::clicked, this, &QDialog::reject);

        mainLayout->addLayout(buttonLayout);

        setLayout(mainLayout);
    }

    void ProjectInfoDialog::updateDisplay()
    {
        if (!m_project) {
            return;
        }

        // ─────────────────────────────────────────────────────
        // 更新基本信息
        // ─────────────────────────────────────────────────────
        m_projectNameLabel->setText(QString::fromStdString(m_project->name));
        m_projectNameEdit->setText(QString::fromStdString(m_project->name));

        // 创建时间
        QDateTime creationTime = QDateTime::fromSecsSinceEpoch(m_project->creation_time);
        m_creationTimeLabel->setText(creationTime.toString("yyyy-MM-dd hh:mm:ss"));

        // 最后修改时间
        QDateTime modifiedTime = QDateTime::fromSecsSinceEpoch(m_project->last_modified_time);
        m_modifiedTimeLabel->setText(modifiedTime.toString("yyyy-MM-dd hh:mm:ss"));

        // 作者
        m_authorLabel->setText(QString::fromStdString(m_project->author));

        // ─────────────────────────────────────────────────────
        // 更新坐标系信息
        // ─────────────────────────────────────────────────────
        const auto& inputCoordSys = m_project->input_coordinate_system;
        
        // 坐标系类型
        QString typeStr = tr("(Not set)");
        switch (inputCoordSys.type) {
            case database::CoordinateSystem::Type::kLocal:
                typeStr = tr("LOCAL");
                break;
            case database::CoordinateSystem::Type::kEPSG:
                typeStr = tr("EPSG");
                break;
            case database::CoordinateSystem::Type::kENU:
                typeStr = tr("ENU");
                break;
            case database::CoordinateSystem::Type::kWKT:
                typeStr = tr("WKT");
                break;
        }
        m_inputCoordTypeLabel->setText(typeStr);

        // 坐标系定义
        QString defStr = tr("(Not set)");
        if (!inputCoordSys.definition.empty()) {
            defStr = QString::fromStdString(inputCoordSys.definition);
            // 如果过长，截断显示
            if (defStr.length() > 200) {
                defStr = defStr.left(200) + "...";
            }
        }
        m_inputCoordDefLabel->setText(defStr);

        // ─────────────────────────────────────────────────────
        // 更新描述
        // ─────────────────────────────────────────────────────
        QString descText = QString::fromStdString(m_project->description);
        if (descText.isEmpty()) {
            descText = tr("(No description)");
        }
        m_descriptionLabel->setText(descText);
        m_descriptionEdit->setPlainText(QString::fromStdString(m_project->description));

        // 设置编辑模式
        setEditingMode(false);
    }

    void ProjectInfoDialog::setEditingMode(bool editing)
    {
        m_isEditing = editing;

        // 项目名称
        m_projectNameLabel->setVisible(!editing);
        m_projectNameEdit->setVisible(editing);
        m_editNameButton->setText(editing ? tr("Cancel") : tr("Edit"));

        // 描述
        m_descriptionLabel->setVisible(!editing);
        m_descriptionEdit->setVisible(editing);
        m_editDescButton->setText(editing ? tr("Cancel") : tr("Edit"));

        // 底部按钮
        m_saveButton->setEnabled(editing);
    }

    void ProjectInfoDialog::onEditProjectName()
    {
        if (!m_isEditing) {
            setEditingMode(true);
            m_projectNameEdit->selectAll();
            m_projectNameEdit->setFocus();
        } else {
            setEditingMode(false);
            // 恢复原始值
            m_projectNameEdit->setText(QString::fromStdString(m_project->name));
        }
    }

    void ProjectInfoDialog::onSaveProjectName()
    {
        std::string newName = m_projectNameEdit->text().toStdString();
        if (!newName.empty() && newName != m_project->name) {
            m_project->name = newName;
            m_project->last_modified_time = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            m_projectNameLabel->setText(QString::fromStdString(m_project->name));
            setWindowTitle(QString::fromStdString(m_project->name) + " - Project Information");
        }
    }

    void ProjectInfoDialog::onEditDescription()
    {
        if (!m_isEditing) {
            setEditingMode(true);
            m_descriptionEdit->setFocus();
        } else {
            setEditingMode(false);
            // 恢复原始值
            m_descriptionEdit->setPlainText(QString::fromStdString(m_project->description));
        }
    }

    void ProjectInfoDialog::onSaveDescription()
    {
        std::string newDesc = m_descriptionEdit->toPlainText().toStdString();
        if (newDesc != m_project->description) {
            m_project->description = newDesc;
            m_project->last_modified_time = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            
            QString descText = QString::fromStdString(m_project->description);
            if (descText.isEmpty()) {
                descText = tr("(No description)");
            }
            m_descriptionLabel->setText(descText);
        }
    }

    void ProjectInfoDialog::onSetInputCoordinateSystem()
    {
        CoordinateSystemConfigDialog dialog(this);
        dialog.SetCoordinateSystem(m_project->input_coordinate_system);
        
        if (dialog.exec() == QDialog::Accepted) {
            auto coordSys = dialog.GetCoordinateSystem();
            m_project->input_coordinate_system = coordSys;
            m_project->last_modified_time = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            updateDisplay();
        }
    }

    void ProjectInfoDialog::onApply()
    {
        if (!m_project) {
            reject();
            return;
        }

        // 保存编辑的项目名称
        onSaveProjectName();
        
        // 保存编辑的描述
        onSaveDescription();

        accept();
    }

    void ProjectInfoDialog::onCancelEdit()
    {
        setEditingMode(false);
    }

} // namespace ui
} // namespace insight
