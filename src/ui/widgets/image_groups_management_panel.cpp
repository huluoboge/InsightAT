/**
 * @file  ImageGroupsManagementPanel.cpp
 * @brief 图像分组管理面板实现。
 */

#include "image_groups_management_panel.h"

#include "dialogs/image_editor_dialog.h"
#include "models/project_document.h"

#include <QDateTime>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QMessageBox>
#include <QVBoxLayout>

#include <glog/logging.h>

namespace insight::ui::widgets {

ImageGroupsManagementPanel::ImageGroupsManagementPanel(QWidget* parent)
    : QWidget(parent), m_projectDocument(nullptr) {
  initialize_ui();
  connect_signals();
}

ImageGroupsManagementPanel::~ImageGroupsManagementPanel() = default;

void ImageGroupsManagementPanel::set_project_document(ProjectDocument* doc) {
  if (m_projectDocument) {
    disconnect(m_projectDocument, nullptr, this, nullptr);
  }

  m_projectDocument = doc;

  if (m_projectDocument) {
    connect(m_projectDocument, &ProjectDocument::projectCleared, this,
            &ImageGroupsManagementPanel::on_project_changed);
    connect(m_projectDocument, &ProjectDocument::imageGroupAdded, this,
            &ImageGroupsManagementPanel::on_image_group_added);
    connect(m_projectDocument, &ProjectDocument::imageGroupRemoved, this,
            &ImageGroupsManagementPanel::on_image_group_removed);
    connect(m_projectDocument, &ProjectDocument::imageGroupChanged, this,
            &ImageGroupsManagementPanel::on_image_group_changed);

    refresh_group_list();
  }
}

void ImageGroupsManagementPanel::initialize_ui() {
  auto mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(8, 8, 8, 8);
  mainLayout->setSpacing(8);

  // 标题
  m_titleLabel = new QLabel(tr("Image Groups"), this);
  m_titleLabel->setStyleSheet("font-weight: bold; font-size: 14px;");
  mainLayout->addWidget(m_titleLabel);

  // [New Group] 按钮
  m_newGroupButton = new QPushButton(tr("+ New Group"), this);
  m_newGroupButton->setMaximumWidth(150);
  mainLayout->addWidget(m_newGroupButton);

  // 分组列表表格
  m_groupTable = new QTableWidget(this);
  m_groupTable->setColumnCount(6);
  m_groupTable->setHorizontalHeaderLabels(
      {tr("Group Name"), tr("Mode"), tr("Images"), tr("Import"), tr("Edit"), tr("Delete")});

  // 配置表格
  m_groupTable->setSelectionBehavior(QAbstractItemView::SelectRows);
  m_groupTable->setSelectionMode(QAbstractItemView::SingleSelection);
  m_groupTable->setAlternatingRowColors(true);
  m_groupTable->horizontalHeader()->setStretchLastSection(false);
  m_groupTable->setColumnWidth(0, 200); // Group Name
  m_groupTable->setColumnWidth(1, 120); // Mode
  m_groupTable->setColumnWidth(2, 80);  // Images
  m_groupTable->setColumnWidth(3, 80);  // Import
  m_groupTable->setColumnWidth(4, 80);  // Edit
  m_groupTable->setColumnWidth(5, 80);  // Delete

  mainLayout->addWidget(m_groupTable);

  setLayout(mainLayout);
}

void ImageGroupsManagementPanel::connect_signals() {
  connect(m_newGroupButton, &QPushButton::clicked, this, &ImageGroupsManagementPanel::on_new_group);
}

void ImageGroupsManagementPanel::refresh_group_list() {
  if (!m_projectDocument) {
    m_groupTable->setRowCount(0);
    return;
  }

  auto& project = m_projectDocument->project();
  const auto& groups = project.image_groups;

  m_groupTable->setRowCount(groups.size());

  for (size_t i = 0; i < groups.size(); ++i) {
    update_table_row(&groups[i], static_cast<int>(i));
  }
}

void ImageGroupsManagementPanel::update_table_row(const database::ImageGroup* group, int row) {
  if (!group || row < 0 || row >= m_groupTable->rowCount()) {
    return;
  }

  // 列 0: 分组名称
  auto nameItem = new QTableWidgetItem(QString::fromStdString(group->group_name));
  nameItem->setFlags(nameItem->flags() & ~Qt::ItemIsEditable);
  nameItem->setData(Qt::UserRole, static_cast<uint>(group->group_id));
  m_groupTable->setItem(row, 0, nameItem);

  // 列 1: 相机模式
  QString modeStr;
  switch (group->camera_mode) {
  case database::ImageGroup::CameraMode::kGroupLevel:
    modeStr = "GroupLevel";
    break;
  case database::ImageGroup::CameraMode::kImageLevel:
    modeStr = "ImageLevel";
    break;
  case database::ImageGroup::CameraMode::kRigBased:
    modeStr = "RigBased";
    break;
  }
  auto modeItem = new QTableWidgetItem(modeStr);
  modeItem->setFlags(modeItem->flags() & ~Qt::ItemIsEditable);
  m_groupTable->setItem(row, 1, modeItem);

  // 列 2: 图像数
  auto countItem = new QTableWidgetItem(QString::number(group->images.size()));
  countItem->setFlags(countItem->flags() & ~Qt::ItemIsEditable);
  countItem->setTextAlignment(Qt::AlignCenter);
  m_groupTable->setItem(row, 2, countItem);

  // 列 3: [导入图像] 按钮
  auto importBtn = new QPushButton(tr("Import"), this);
  importBtn->setMaximumWidth(70);
  importBtn->setProperty("group_id", static_cast<uint>(group->group_id));
  connect(importBtn, &QPushButton::clicked, this, &ImageGroupsManagementPanel::on_import_images);
  m_groupTable->setCellWidget(row, 3, importBtn);

  // 列 4: [编辑] 按钮
  auto editBtn = new QPushButton(tr("Edit"), this);
  editBtn->setMaximumWidth(70);
  editBtn->setProperty("group_id", static_cast<uint>(group->group_id));
  connect(editBtn, &QPushButton::clicked, this, &ImageGroupsManagementPanel::on_edit_group);
  m_groupTable->setCellWidget(row, 4, editBtn);

  // 列 5: [删除] 按钮
  auto deleteBtn = new QPushButton(tr("Delete"), this);
  deleteBtn->setMaximumWidth(70);
  deleteBtn->setProperty("group_id", static_cast<uint>(group->group_id));
  deleteBtn->setStyleSheet("QPushButton { background-color: #ffcccc; }");
  connect(deleteBtn, &QPushButton::clicked, this, &ImageGroupsManagementPanel::on_delete_group);
  m_groupTable->setCellWidget(row, 5, deleteBtn);
}

std::string ImageGroupsManagementPanel::get_next_group_name() {
  if (!m_projectDocument) {
    return "photo_group0";
  }

  auto& project = m_projectDocument->project();
  int counter = 0;
  while (true) {
    std::string name = "photo_group" + std::to_string(counter);
    bool found = false;
    for (const auto& group : project.image_groups) {
      if (group.group_name == name) {
        found = true;
        break;
      }
    }
    if (!found) {
      return name;
    }
    counter++;
  }
}

// ═══════════════════════════════════════════════════════════════
// 槽函数
// ═══════════════════════════════════════════════════════════════

void ImageGroupsManagementPanel::on_new_group() {
  if (!m_projectDocument) {
    LOG(WARNING) << "ProjectDocument not set";
    return;
  }

  std::string groupName = get_next_group_name();

  // 创建新分组（通过 ProjectDocument）
  uint32_t groupId = m_projectDocument->createImageGroup(
      QString::fromStdString(groupName), database::ImageGroup::CameraMode::kGroupLevel);

  if (groupId == 0) {
    LOG(WARNING) << "Failed to create image group";
    QMessageBox::warning(this, tr("Error"), tr("Failed to create image group"));
    return;
  }

  // 立即保存项目，确保新分组被持久化
  m_projectDocument->saveProject();

  LOG(INFO) << "Created new image group: " << groupName;
}

void ImageGroupsManagementPanel::on_import_images() {
  // 获取点击的按钮
  auto btn = qobject_cast<QPushButton*>(sender());
  if (!btn) {
    return;
  }

  uint32_t groupId = btn->property("group_id").toUInt();
  if (!m_projectDocument) {
    return;
  }

  // 找到对应的ImageGroup
  auto& project = m_projectDocument->project();
  database::ImageGroup* targetGroup = nullptr;

  for (auto& group : project.image_groups) {
    if (group.group_id == groupId) {
      targetGroup = &group;
      break;
    }
  }

  if (!targetGroup) {
    LOG(WARNING) << "ImageGroup not found: " << groupId;
    return;
  }

  // 创建或重用ImageEditorDialog
  if (!m_imageEditorDialog) {
    m_imageEditorDialog = std::make_unique<ImageEditorDialog>(m_projectDocument, this);
    // 连接 imagesChanged 信号到刷新表格
    connect(m_imageEditorDialog.get(), &ImageEditorDialog::images_changed, this,
            &ImageGroupsManagementPanel::on_image_group_changed);
  }

  m_imageEditorDialog->load_group(targetGroup);
  m_imageEditorDialog->show();
  m_imageEditorDialog->raise();
  m_imageEditorDialog->activateWindow();
}

void ImageGroupsManagementPanel::on_edit_group() {
  // 获取点击的按钮
  auto btn = qobject_cast<QPushButton*>(sender());
  if (!btn) {
    return;
  }

  uint32_t groupId = btn->property("group_id").toUInt();
  if (!m_projectDocument) {
    return;
  }

  auto& project = m_projectDocument->project();
  for (auto& group : project.image_groups) {
    if (group.group_id == groupId) {
      emit edit_group_requested(&group);
      return;
    }
  }
}

void ImageGroupsManagementPanel::on_delete_group() {
  // 获取点击的按钮
  auto btn = qobject_cast<QPushButton*>(sender());
  if (!btn) {
    return;
  }

  uint32_t groupId = btn->property("group_id").toUInt();
  if (!m_projectDocument) {
    return;
  }

  // 查找分组名称用于确认对话框
  auto& project = m_projectDocument->project();
  std::string groupName;
  for (const auto& group : project.image_groups) {
    if (group.group_id == groupId) {
      groupName = group.group_name;
      break;
    }
  }

  // 确认删除
  int ret = QMessageBox::question(
      this, tr("Delete Group"),
      tr("Are you sure you want to delete group \"%1\"?").arg(QString::fromStdString(groupName)),
      QMessageBox::Yes | QMessageBox::No);

  if (ret != QMessageBox::Yes) {
    return;
  }

  // 删除分组
  bool success = m_projectDocument->deleteImageGroup(groupId);
  if (!success) {
    LOG(WARNING) << "Failed to delete image group: " << groupId;
    QMessageBox::warning(this, tr("Error"), tr("Failed to delete group"));
  }
}

void ImageGroupsManagementPanel::on_project_changed() { refresh_group_list(); }

void ImageGroupsManagementPanel::on_image_group_added(uint32_t group_id) {
  Q_UNUSED(group_id);
  refresh_group_list();
}

void ImageGroupsManagementPanel::on_image_group_removed(uint32_t group_id) {
  Q_UNUSED(group_id);
  refresh_group_list();
}

void ImageGroupsManagementPanel::on_image_group_changed(uint32_t group_id) {
  Q_UNUSED(group_id);
  refresh_group_list();
}

}  // namespace insight::ui::widgets
