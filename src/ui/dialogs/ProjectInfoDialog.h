/**
 * @file ProjectInfoDialog.h
 * @brief 项目信息对话框
 * 
 * 用于查看和编辑项目的基本信息和坐标系设置
 */

#ifndef UI_DIALOGS_PROJECTINFODIALOG_H
#define UI_DIALOGS_PROJECTINFODIALOG_H

#include <QDialog>
#include <memory>

class QLineEdit;
class QPlainTextEdit;
class QLabel;
class QTabWidget;

namespace insight {
namespace database {
struct Project;
}

namespace ui {

class SpatialReferenceDialog;

/**
 * @class ProjectInfoDialog
 * @brief 项目信息对话框
 */
class ProjectInfoDialog : public QDialog {
    Q_OBJECT

public:
    explicit ProjectInfoDialog(insight::database::Project* project, QWidget* parent = nullptr);
    ~ProjectInfoDialog();

private slots:
    /**
     * 编辑项目名称
     */
    void onEditProjectName();

    /**
     * 编辑项目描述
     */
    void onEditDescription();

    /**
     * 设置输入坐标系
     */
    void onSetInputCoordinateSystem();

    /**
     * 保存项目信息
     */
    void onApply();

private:
    /**
     * 初始化 UI
     */
    void initializeUI();

    /**
     * 更新显示信息
     */
    void updateDisplay();

    // 数据
    insight::database::Project* m_project = nullptr;
    bool m_isEditing = false;

    // UI 组件
    QTabWidget* m_tabWidget = nullptr;

    // 基本信息标签页
    QLineEdit* m_projectNameEdit = nullptr;
    QLabel* m_projectPathLabel = nullptr;
    QPlainTextEdit* m_descriptionEdit = nullptr;
    QLineEdit* m_inputCoordinateSystemEdit = nullptr;
    QLabel* m_inputCoordinateSystemLabel = nullptr;
};

}  // namespace ui
}  // namespace insight

#endif  // UI_DIALOGS_PROJECTINFODIALOG_H
