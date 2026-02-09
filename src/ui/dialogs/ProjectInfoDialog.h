/**
 * @file ProjectInfoDialog.h
 * @brief 项目信息对话框
 * 
 * 新设计的项目信息对话框：
 * - 分段式布局，更清晰的信息层次
 * - 左侧 panel 弹出式对话框
 * - 支持在线编辑项目名称和描述
 * - 完整的坐标系管理
 */

#ifndef UI_DIALOGS_PROJECTINFODIALOG_H
#define UI_DIALOGS_PROJECTINFODIALOG_H

#include <QDialog>
#include <memory>

class QLineEdit;
class QPlainTextEdit;
class QLabel;
class QPushButton;
class QScrollArea;

namespace insight {
namespace database {
struct Project;
}

namespace ui {

class SpatialReferenceDialog;

/**
 * @class ProjectInfoDialog
 * @brief 项目信息查看编辑对话框
 * 
 * 提供项目的基本信息和坐标系配置的查看和编辑界面
 */
class ProjectInfoDialog : public QDialog {
    Q_OBJECT

public:
    explicit ProjectInfoDialog(insight::database::Project* project, QWidget* parent = nullptr);
    ~ProjectInfoDialog();

private slots:
    /**
     * 编辑/保存 项目名称
     */
    void onEditProjectName();
    void onSaveProjectName();

    /**
     * 编辑/保存 项目描述
     */
    void onEditDescription();
    void onSaveDescription();

    /**
     * 设置输入坐标系
     */
    void onSetInputCoordinateSystem();

    /**
     * 保存对话框
     */
    void onApply();

    /**
     * 取消编辑
     */
    void onCancelEdit();

private:
    /**
     * 初始化 UI
     */
    void initializeUI();

    /**
     * 更新显示信息
     */
    void updateDisplay();

    /**
     * 切换编辑状态
     */
    void setEditingMode(bool editing);

    /**
     * 创建分隔符
     */
    QWidget* createSeparator();

    // 数据
    insight::database::Project* m_project = nullptr;
    bool m_isEditing = false;

    // ─────────────────────────────────────────────────────
    // 基本信息区
    // ─────────────────────────────────────────────────────
    
    // 项目名称
    QLineEdit* m_projectNameEdit = nullptr;
    QPushButton* m_editNameButton = nullptr;
    QLabel* m_projectNameLabel = nullptr;

    // 时间信息
    QLabel* m_creationTimeLabel = nullptr;
    QLabel* m_modifiedTimeLabel = nullptr;
    QLabel* m_authorLabel = nullptr;

    // ─────────────────────────────────────────────────────
    // 坐标系区
    // ─────────────────────────────────────────────────────
    
    // 输入坐标系
    QLabel* m_inputCoordTypeLabel = nullptr;        // 坐标系类型显示
    QLabel* m_inputCoordDefLabel = nullptr;         // 坐标系定义显示
    QPushButton* m_setCoordButton = nullptr;        // 设置按钮

    // ─────────────────────────────────────────────────────
    // 描述区
    // ─────────────────────────────────────────────────────
    
    QPlainTextEdit* m_descriptionEdit = nullptr;
    QPushButton* m_editDescButton = nullptr;
    QLabel* m_descriptionLabel = nullptr;

    // ─────────────────────────────────────────────────────
    // 底部按钮区
    // ─────────────────────────────────────────────────────
    
    QPushButton* m_saveButton = nullptr;
    QPushButton* m_cancelButton = nullptr;
};

}  // namespace ui
}  // namespace insight

#endif  // UI_DIALOGS_PROJECTINFODIALOG_H
