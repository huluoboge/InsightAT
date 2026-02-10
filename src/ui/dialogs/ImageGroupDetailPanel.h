#pragma once

#include <QDialog>

#include "database/database_types.h"
#include "../widgets/CameraParameterEditorWidget.h"

namespace insight::ui {

class ProjectDocument;

namespace dialogs {

/**
 * @brief 图像分组详情编辑对话框
 * 
 * 非模态对话框，用户可通过 ImageGroupsManagementPanel 中的 [编辑] 按钮打开
 * 内嵌 CameraParameterEditorWidget 用于编辑相机参数
 * 
 * 任何字段失焦时自动保存到 ProjectDocument
 */
class ImageGroupDetailPanel : public QDialog {
    Q_OBJECT

public:
    explicit ImageGroupDetailPanel(QWidget* parent = nullptr);
    ~ImageGroupDetailPanel();

    /**
     * @brief 设置项目文档指针
     */
    void SetProjectDocument(ProjectDocument* doc);

    /**
     * @brief 加载分组数据到对话框
     */
    void LoadGroup(database::ImageGroup* group);

protected:
    void closeEvent(QCloseEvent* event) override;

signals:
    /**
     * @brief 分组数据被修改并保存时发出
     */
    void groupDataChanged(uint32_t group_id);

private slots:
    // ─── 数据保存槽 ───
    void onCameraParameterModified();
    void onCameraParameterModeChanged(database::ImageGroup::CameraMode mode);
    void onGroupNameModified();
    void onAutoEstimateRequested();

private:
    // ─── 初始化 ───
    void InitializeUI();
    void ConnectSignals();

    // ─── 工具方法 ───
    void SaveGroupData();

    // ─── UI 组件 ───
    widgets::CameraParameterEditorWidget* m_cameraEditor;

    // ─── 数据指针 ───
    ProjectDocument* m_projectDocument;  // 无所有权
    database::ImageGroup* m_currentGroup;  // 无所有权
};

}  // namespace dialogs
}  // namespace insight::ui
