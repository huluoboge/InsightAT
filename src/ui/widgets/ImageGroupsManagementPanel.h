#pragma once

#include <QWidget>
#include <QTableWidget>
#include <QPushButton>
#include <QLabel>
#include <memory>

#include "database/database_types.h"

namespace insight {
namespace ui {
class ProjectDocument;  // 前向声明，来自 models/ProjectDocument.h
class ImageEditorDialog;  // 前向声明

namespace widgets {

/**
 * @brief 图像分组管理面板 - 显示在右侧
 * 
 * 功能：
 * - 显示所有分组的列表（QTableWidget）
 * - [New Group] 按钮自动生成分组
 * - [Import Images] 按钮打开图像编辑对话框
 * - [编辑] 按钮打开分组详情编辑对话框
 * - [删除] 按钮删除分组（确认对话框）
 */
class ImageGroupsManagementPanel : public QWidget {
    Q_OBJECT

public:
    explicit ImageGroupsManagementPanel(QWidget* parent = nullptr);
    ~ImageGroupsManagementPanel();

    /**
     * @brief 设置项目文档指针，用于访问 Project 数据
     */
    void SetProjectDocument(ProjectDocument* doc);

    /**
     * @brief 从 Project 重新加载分组列表
     */
    void RefreshGroupList();

signals:
    /**
     * @brief 用户点击 [编辑] 按钮时发出
     */
    void editGroupRequested(database::ImageGroup* group);

private slots:
    // ─── UI 事件槽 ───
    void onNewGroup();
    void onImportImages();
    void onEditGroup();
    void onDeleteGroup();

    // ─── 数据变化槽 ───
    void onProjectChanged();
    void onImageGroupAdded(uint32_t group_id);
    void onImageGroupRemoved(uint32_t group_id);
    void onImageGroupChanged(uint32_t group_id);

private:
    // ─── 初始化 ───
    void InitializeUI();
    void ConnectSignals();

    // ─── 工具方法 ───
    void UpdateTableRow(const database::ImageGroup* group, int row);
    std::string GetNextGroupName();

    // ─── UI 组件 ───
    QLabel* m_titleLabel;
    QPushButton* m_newGroupButton;
    QTableWidget* m_groupTable;
    
    // ─── 对话框 ───
    std::unique_ptr<ImageEditorDialog> m_imageEditorDialog;
    uint32_t m_currentImportGroupId = 0;  // 当前导入的分组ID

    // ─── 数据指针 ───
    ProjectDocument* m_projectDocument;  // 无所有权
};

}  // namespace widgets
}  // namespace ui
}  // namespace insight
