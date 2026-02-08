/**
 * @file ImageGroupDialog.cpp
 * @brief 图像分组对话框 - 实现
 */

#include "ImageGroupDialog.h"
#include "../widgets/CameraModelWidget.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QRadioButton>
#include <QButtonGroup>
#include <QComboBox>
#include <QGroupBox>
#include <QMessageBox>
#include <QUuid>
#include <glog/logging.h>

namespace insight {
namespace ui {

ImageGroupDialog::ImageGroupDialog(
    insight::database::Project* project,
    QWidget* parent)
    : QDialog(parent)
    , m_project(project)
{
    setWindowTitle("创建图像分组");
    setModal(true);
    setMinimumWidth(500);
    setMinimumHeight(600);
    initializeUI();
}

ImageGroupDialog::~ImageGroupDialog() = default;

const insight::database::ImageGroup& ImageGroupDialog::getImageGroup() const
{
    return m_imageGroup;
}

void ImageGroupDialog::initializeUI()
{
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->setSpacing(10);
    mainLayout->setContentsMargins(20, 20, 20, 20);

    // ─────────────────────────────────────────────────────────
    // 分组基本信息
    // ─────────────────────────────────────────────────────────
    
    QGroupBox* basicGroup = new QGroupBox("分组基本信息", this);
    QVBoxLayout* basicLayout = new QVBoxLayout(basicGroup);
    
    // 分组名称
    QLabel* nameLabel = new QLabel("分组名称 *", this);
    m_groupNameEdit = new QLineEdit(this);
    m_groupNameEdit->setPlaceholderText("输入分组名称");
    basicLayout->addWidget(nameLabel);
    basicLayout->addWidget(m_groupNameEdit);
    
    // 分组描述
    QLabel* descLabel = new QLabel("分组描述", this);
    m_descriptionEdit = new QPlainTextEdit(this);
    m_descriptionEdit->setPlaceholderText("输入分组描述（可选）");
    m_descriptionEdit->setMaximumHeight(80);
    basicLayout->addWidget(descLabel);
    basicLayout->addWidget(m_descriptionEdit);
    
    mainLayout->addWidget(basicGroup);

    // ─────────────────────────────────────────────────────────
    // 相机参数模式选择
    // ─────────────────────────────────────────────────────────
    
    QGroupBox* modeGroup = new QGroupBox("相机参数模式", this);
    QVBoxLayout* modeLayout = new QVBoxLayout(modeGroup);
    
    m_cameraModelGroup = new QButtonGroup(this);
    
    // 模式1：组级（所有图像共享一个相机）
    m_groupLevelRadio = new QRadioButton(
        "Group Level - 所有图像共享一个相机参数", this);
    m_groupLevelRadio->setChecked(true);
    m_cameraModelGroup->addButton(m_groupLevelRadio, 0);
    modeLayout->addWidget(m_groupLevelRadio);
    
    // 模式2：图像级（每个图像独立相机）
    m_imageLevelRadio = new QRadioButton(
        "Image Level - 每个图像有独立的相机参数", this);
    m_cameraModelGroup->addButton(m_imageLevelRadio, 1);
    modeLayout->addWidget(m_imageLevelRadio);
    
    // 模式3：Rig 模式（多相机配置）
    m_rigBasedRadio = new QRadioButton(
        "Rig Based - 图像来自多相机配置", this);
    m_cameraModelGroup->addButton(m_rigBasedRadio, 2);
    modeLayout->addWidget(m_rigBasedRadio);
    
    mainLayout->addWidget(modeGroup);

    // ─────────────────────────────────────────────────────────
    // 相机参数编辑（仅在 GroupLevel 显示）
    // ─────────────────────────────────────────────────────────
    
    QGroupBox* cameraGroup = new QGroupBox("相机参数（组级模式）", this);
    QVBoxLayout* cameraLayout = new QVBoxLayout(cameraGroup);
    
    m_cameraWidget = new CameraModelWidget(this);
    cameraLayout->addWidget(m_cameraWidget);
    
    mainLayout->addWidget(cameraGroup);

    // ─────────────────────────────────────────────────────────
    // Rig 配置（仅在 RigBased 显示）
    // ─────────────────────────────────────────────────────────
    
    QGroupBox* rigGroup = new QGroupBox("Rig 配置（Rig 模式）", this);
    QVBoxLayout* rigLayout = new QVBoxLayout(rigGroup);
    rigGroup->setVisible(false);  // 默认隐藏
    
    // Rig 选择
    QLabel* rigLabel = new QLabel("选择 Rig 配置：", this);
    m_rigCombo = new QComboBox(this);
    m_rigCombo->addItem("-- 选择 Rig 配置 --");
    rigLayout->addWidget(rigLabel);
    rigLayout->addWidget(m_rigCombo);
    
    // Rig 内相机选择
    QLabel* mountLabel = new QLabel("选择相机挂载点：", this);
    m_rigMountCombo = new QComboBox(this);
    m_rigMountCombo->addItem("-- 选择挂载点 --");
    rigLayout->addWidget(mountLabel);
    rigLayout->addWidget(m_rigMountCombo);
    
    mainLayout->addWidget(rigGroup);

    // 添加伸缩
    mainLayout->addStretch();

    // ─────────────────────────────────────────────────────────
    // 按钮
    // ─────────────────────────────────────────────────────────
    
    QHBoxLayout* buttonLayout = new QHBoxLayout();
    buttonLayout->setSpacing(10);
    
    m_createButton = new QPushButton("创建分组", this);
    m_cancelButton = new QPushButton("取消", this);
    
    m_createButton->setMinimumWidth(100);
    m_cancelButton->setMinimumWidth(100);
    
    buttonLayout->addStretch();
    buttonLayout->addWidget(m_createButton);
    buttonLayout->addWidget(m_cancelButton);
    
    mainLayout->addLayout(buttonLayout);

    // ─────────────────────────────────────────────────────────
    // 连接信号槽
    // ─────────────────────────────────────────────────────────
    
    connect(m_cameraModelGroup, QOverload<int>::of(&QButtonGroup::idClicked),
            this, &ImageGroupDialog::onCameraModeChanged);
    
    connect(m_createButton, &QPushButton::clicked,
            this, &ImageGroupDialog::onCreateGroup);
    
    connect(m_cancelButton, &QPushButton::clicked,
            this, &QDialog::reject);

    // 默认焦点
    m_groupNameEdit->setFocus();
}

void ImageGroupDialog::onCameraModeChanged()
{
    int modeId = m_cameraModelGroup->checkedId();
    
    // 获取相机配置组件的父组件
    QGroupBox* cameraGroup = qobject_cast<QGroupBox*>(
        m_cameraWidget->parentWidget()->parentWidget());
    QGroupBox* rigGroup = nullptr;
    
    // 查找 Rig 配置组件
    for (int i = 0; i < m_cameraWidget->parentWidget()->parentWidget()
                        ->parentWidget()->findChildren<QGroupBox*>().count(); ++i) {
        QGroupBox* gb = m_cameraWidget->parentWidget()->parentWidget()
                            ->parentWidget()->findChildren<QGroupBox*>()[i];
        if (gb->title() == "Rig 配置（Rig 模式）") {
            rigGroup = gb;
            break;
        }
    }

    // 根据模式显示/隐藏相应的配置区域
    if (cameraGroup) {
        cameraGroup->setVisible(modeId == 0);  // 仅在 GroupLevel 模式显示
    }

    LOG(INFO) << "Camera mode changed to: " << modeId;
}

void ImageGroupDialog::onCreateGroup()
{
    if (!validateInput()) {
        return;
    }

    // 设置分组基本信息
    m_imageGroup.group_id = getNextGroupId();
    m_imageGroup.group_name = m_groupNameEdit->text().trimmed().toStdString();
    m_imageGroup.description = m_descriptionEdit->toPlainText().trimmed().toStdString();
    m_imageGroup.creation_time = std::time(nullptr);

    // 根据模式设置相机参数
    int modeId = m_cameraModelGroup->checkedId();
    
    switch (modeId) {
        case 0:  // GroupLevel
            m_imageGroup.camera_mode = 
                insight::database::ImageGroup::CameraMode::kGroupLevel;
            // 从 CameraModelWidget 获取相机参数
            m_imageGroup.group_camera = m_cameraWidget->getCameraModel();
            break;
            
        case 1:  // ImageLevel
            m_imageGroup.camera_mode = 
                insight::database::ImageGroup::CameraMode::kImageLevel;
            m_imageGroup.group_camera.reset();  // 清空组级相机
            break;
            
        case 2:  // RigBased
            m_imageGroup.camera_mode = 
                insight::database::ImageGroup::CameraMode::kRigBased;
            // 从 combo 框获取 Rig 和 Mount 信息
            m_imageGroup.rig_mount_info = 
                insight::database::ImageGroup::RigMountInfo();
            m_imageGroup.group_camera.reset();
            break;
            
        default:
            break;
    }

    // 发出信号
    emit imageGroupCreated(m_imageGroup);
    
    // 接受对话框
    accept();
}

bool ImageGroupDialog::validateInput()
{
    // 检查分组名称
    if (m_groupNameEdit->text().trimmed().isEmpty()) {
        QMessageBox::warning(this, "输入错误", "分组名称不能为空！");
        m_groupNameEdit->setFocus();
        return false;
    }

    // 检查分组名称长度
    if (m_groupNameEdit->text().length() > 100) {
        QMessageBox::warning(this, "输入错误", "分组名称过长（最多100个字符）！");
        m_groupNameEdit->selectAll();
        m_groupNameEdit->setFocus();
        return false;
    }

    // 如果是 GroupLevel 模式，检查相机参数
    int modeId = m_cameraModelGroup->checkedId();
    if (modeId == 0) {  // GroupLevel
        if (!m_cameraWidget->validateCamera()) {
            QMessageBox::warning(this, "输入错误", "请检查相机参数的有效性！");
            return false;
        }
    }

    return true;
}

uint32_t ImageGroupDialog::getNextGroupId() const
{
    if (!m_project || m_project->image_groups.empty()) {
        return 1;
    }

    uint32_t maxId = 0;
    for (const auto& group : m_project->image_groups) {
        if (group.group_id > maxId) {
            maxId = group.group_id;
        }
    }

    return maxId + 1;
}

}  // namespace ui
}  // namespace insight
