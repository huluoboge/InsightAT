/**
 * @file ATTaskPanel.h
 * @brief AT 任务编辑面板
 * 
 * 非模态 QWidget，用于编辑 AT Task 的详细信息。包含三个 Tab：
 * - Input Data：显示 InputSnapshot 中的图像和相机数据（嵌入 ImageEditorDialog）
 * - Optimization：相机参数优化配置（CameraOptimizationPanel）
 * - Export：导出选项和按钮
 * 
 * 底部有 Save/Export/Close 按钮。
 */

#ifndef UI_PANELS_ATTASKPANEL_H
#define UI_PANELS_ATTASKPANEL_H

#include <QWidget>
#include <QString>
#include <memory>

class QTabWidget;
class QGroupBox;
class QPushButton;
class QLineEdit;
class QLabel;

namespace insight {
namespace ui {

class ProjectDocument;

/**
 * @class ATTaskPanel
 * @brief 非模态 AT Task 编辑面板
 */
class ATTaskPanel : public QWidget {
    Q_OBJECT

public:
    /**
     * 构造函数
     * 
     * @param[in] document 项目文档指针
     * @param[in] parent 父部件
     */
    explicit ATTaskPanel(ProjectDocument* document, QWidget* parent = nullptr);
    
    ~ATTaskPanel();
    
    /**
     * 加载指定 AT Task 进行编辑
     * 
     * @param[in] task_id 任务 UUID
     * @return true 如果成功加载
     */
    bool loadTask(const std::string& task_id);
    
    /**
     * 获取当前加载的任务 ID
     * 
     * @return 当前任务的 UUID 字符串，或空字符串如果没有加载任务
     */
    std::string getCurrentTaskId() const;

public slots:
    /**
     * Export 按钮点击处理（打开导出对话框）
     */
    void onExportClicked();
    
    /**
     * 任务名称改变时自动保存
     */
    void onTaskNameChanged();

private:
    /**
     * 初始化 UI 组件
     */
    void initUI();
    
    /**
     * 刷新面板显示，重新加载当前任务数据
     */
    void refreshUI();
    
    /**
     * 保存当前任务数据
     */
    void saveTask();

private:
    ProjectDocument* m_document;
    std::string m_currentTaskId;
    
    // UI 组件
    QTabWidget* m_tabWidget;
    QWidget* m_inputDataTab;      ///< Tab 1：Input Data
    QWidget* m_optimizationTab;   ///< Tab 2：Optimization
    QWidget* m_exportTab;         ///< Tab 3：Export
    
    // 顶部信息显示
    QLineEdit* m_taskNameEdit;
    QLabel* m_taskIdLabel;        ///< 显示任务的完整 UUID
    QLabel* m_parentTaskLabel;
    QLabel* m_statusLabel;
    
    // Export Tab 中的按钮
    QPushButton* m_exportButton = nullptr;
};

}  // namespace ui
}  // namespace insight

#endif  // UI_PANELS_ATTASKPANEL_H
