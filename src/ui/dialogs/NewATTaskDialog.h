/**
 * @file NewATTaskDialog.h
 * @brief 新建空三任务对话框
 * 
 * 模态对话框，用于创建新的 AT Task。支持：
 * - 任务名称输入（默认 AT_0, AT_1 等）
 * - 父任务选择（支持 None）
 * - OK/Cancel 按钮
 */

#ifndef UI_DIALOGS_NEWATTASKDIALOG_H
#define UI_DIALOGS_NEWATTASKDIALOG_H

#include <QDialog>
#include <QString>
#include <memory>

class QLineEdit;
class QComboBox;
class QPushButton;
class QLabel;

namespace insight {
namespace ui {

class ProjectDocument;

/**
 * @class NewATTaskDialog
 * @brief 创建新 AT Task 的模态对话框
 */
class NewATTaskDialog : public QDialog {
    Q_OBJECT

public:
    /**
     * 构造函数
     * 
     * @param[in] document 项目文档指针
     * @param[in] defaultName 默认任务名称（如 "AT_2"）
     * @param[in] parent 父部件
     */
    explicit NewATTaskDialog(ProjectDocument* document,
                            const QString& defaultName = "",
                            QWidget* parent = nullptr);
    
    ~NewATTaskDialog();
    
    /**
     * 获取输入的任务名称
     * 
     * @return 用户输入的任务名称
     */
    std::string getTaskName() const;
    
    /**
     * 获取选中的父任务索引（在 at_tasks 数组中的位置）
     * 
     * @return 父任务的索引，如果选择 "None" 则返回 -1
     */
    uint32_t getParentTaskIndex() const;

private slots:
    /**
     * OK 按钮点击处理
     */
    void onOKClicked();
    
    /**
     * Cancel 按钮点击处理
     */
    void onCancelClicked();

private:
    /**
     * 初始化 UI 组件
     */
    void initUI();
    
    /**
     * 加载父任务列表到 ComboBox
     */
    void loadParentTasks();

private:
    ProjectDocument* m_document;
    
    // UI 组件
    QLineEdit* m_taskNameEdit;
    QComboBox* m_parentTaskCombo;
    QPushButton* m_okButton;
    QPushButton* m_cancelButton;
    
    // 缓存的任务 ID 和名称映射（用于反向查询）
    std::vector<std::pair<QString, std::string>> m_taskList;  // {displayName, task_id}
};

}  // namespace ui
}  // namespace insight

#endif  // UI_DIALOGS_NEWATTASKDIALOG_H
