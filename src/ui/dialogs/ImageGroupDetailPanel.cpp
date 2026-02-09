#include "ImageGroupDetailPanel.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QCloseEvent>
#include <glog/logging.h>

#include "models/ProjectDocument.h"

namespace insight::ui::dialogs {

ImageGroupDetailPanel::ImageGroupDetailPanel(QWidget* parent)
    : QDialog(parent),
      m_projectDocument(nullptr),
      m_currentGroup(nullptr) {
    setWindowTitle(tr("Edit Image Group"));
    setWindowModality(Qt::NonModal);
    setMinimumWidth(600);
    setMinimumHeight(800);
    
    InitializeUI();
    ConnectSignals();
}

ImageGroupDetailPanel::~ImageGroupDetailPanel() = default;

void ImageGroupDetailPanel::SetProjectDocument(ProjectDocument* doc) {
    m_projectDocument = doc;
}

void ImageGroupDetailPanel::LoadGroup(database::ImageGroup* group) {
    if (!group) {
        return;
    }

    m_currentGroup = group;

    // 更新窗口标题
    setWindowTitle(tr("Edit Image Group: %1").arg(QString::fromStdString(group->group_name)));

    // 加载相机参数到编辑器
    if (group->group_camera) {
        m_cameraEditor->LoadCamera(*group->group_camera);
    } else {
        // 创建默认相机模型
        database::CameraModel defaultCamera;
        // 注意: camera_mode 属于 ImageGroup，不属于 CameraModel
        m_cameraEditor->LoadCamera(defaultCamera);
    }

    // 设置编辑器的模式
    m_cameraEditor->SetMode(group->camera_mode);

    // 设置分组名称
    m_cameraEditor->SetGroupName(group->group_name);

    // 显示分组名称字段
    m_cameraEditor->ShowGroupNameField(true);

    // 显示对话框
    show();
    raise();
    activateWindow();
}

void ImageGroupDetailPanel::InitializeUI() {
    auto mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(12, 12, 12, 12);
    mainLayout->setSpacing(8);

    // 标题 label
    auto titleLabel = new QLabel(tr("Group: (no selection)"), this);
    titleLabel->setStyleSheet("font-weight: bold; font-size: 13px;");
    mainLayout->addWidget(titleLabel);

    // 相机参数编辑器
    m_cameraEditor = new widgets::CameraParameterEditorWidget(this);
    mainLayout->addWidget(m_cameraEditor);

    // 底部按钮
    auto btnLayout = new QHBoxLayout();
    btnLayout->addStretch();

    auto closeBtn = new QPushButton(tr("Close"), this);
    closeBtn->setMinimumWidth(80);
    connect(closeBtn, &QPushButton::clicked, this, &QDialog::close);
    btnLayout->addWidget(closeBtn);

    mainLayout->addLayout(btnLayout);

    setLayout(mainLayout);
}

void ImageGroupDetailPanel::ConnectSignals() {
    connect(m_cameraEditor, &widgets::CameraParameterEditorWidget::fieldModified,
            this, &ImageGroupDetailPanel::onCameraParameterModified);
    connect(m_cameraEditor, &widgets::CameraParameterEditorWidget::modeChanged,
            this, &ImageGroupDetailPanel::onCameraParameterModeChanged);
}

void ImageGroupDetailPanel::SaveGroupData() {
    if (!m_currentGroup || !m_projectDocument) {
        return;
    }

    // 保存分组名称
    std::string newName = m_cameraEditor->GetGroupName();
    if (newName != m_currentGroup->group_name) {
        m_currentGroup->group_name = newName;
        // 更新窗口标题
        setWindowTitle(tr("Edit Image Group: %1").arg(QString::fromStdString(newName)));
    }

    // 保存相机参数（仅 GroupLevel 模式）
    auto camera = m_cameraEditor->GetCamera();
    if (m_currentGroup->camera_mode == database::ImageGroup::CameraMode::kGroupLevel) {
        m_currentGroup->group_camera = camera;
    }

    // 更新相机模式（从编辑器获取）
    auto newMode = m_cameraEditor->GetMode();
    m_currentGroup->camera_mode = newMode;

    // 通知 ProjectDocument 分组数据已改变
    // （setModified 是私有的，所以通过信号通知）

    // 发出信号
    emit groupDataChanged(std::to_string(m_currentGroup->group_id));

    LOG(INFO) << "Saved image group: " << m_currentGroup->group_name;
}

// ═══════════════════════════════════════════════════════════════
// 槽函数
// ═══════════════════════════════════════════════════════════════

void ImageGroupDetailPanel::onCameraParameterModified() {
    SaveGroupData();
}

void ImageGroupDetailPanel::onCameraParameterModeChanged(database::ImageGroup::CameraMode mode) {
    if (m_currentGroup) {
        m_currentGroup->camera_mode = mode;
        SaveGroupData();
    }
}

void ImageGroupDetailPanel::onGroupNameModified() {
    SaveGroupData();
}

// ═══════════════════════════════════════════════════════════════
// 事件处理
// ═══════════════════════════════════════════════════════════════

void ImageGroupDetailPanel::closeEvent(QCloseEvent* event) {
    // 关闭时确保数据已保存
    SaveGroupData();
    event->accept();
}

}  // namespace insight::ui::dialogs
