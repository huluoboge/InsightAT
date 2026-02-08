/**
 * @file NewProjectDialog.h
 * @brief 新建项目对话框
 * 
 * 功能：
 * 1. 输入项目名称、作者、描述
 * 2. 自动生成 UUID 和时间戳
 * 3. 发出 projectCreated 信号
 */

#ifndef UI_NEWPROJECTDIALOG_H
#define UI_NEWPROJECTDIALOG_H

#include <QDialog>
#include <QString>
#include <memory>

class QLineEdit;
class QPlainTextEdit;
class QPushButton;

namespace insight {
namespace ui {

/**
 * @class NewProjectDialog
 * @brief 新建项目对话框
 */
class NewProjectDialog : public QDialog {
    Q_OBJECT

public:
    explicit NewProjectDialog(QWidget* parent = nullptr);
    ~NewProjectDialog();

signals:
    /**
     * 项目创建信号
     * 
     * @param name 项目名称
     * @param author 作者（可为空）
     * @param description 描述（可为空）
     */
    void projectCreated(const QString& name, const QString& author, 
                       const QString& description);

private slots:
    /**
     * 处理确定按钮点击事件
     */
    void onCreateProject();

private:
    /**
     * 初始化UI
     */
    void initializeUI();

    /**
     * 验证输入
     * 
     * @return 有效返回true
     */
    bool validateInput();

    // UI 组件
    QLineEdit* m_projectNameEdit;       ///< 项目名称输入框
    QLineEdit* m_authorEdit;             ///< 作者输入框
    QPlainTextEdit* m_descriptionEdit;   ///< 描述输入框
    QPushButton* m_createButton;         ///< 创建按钮
    QPushButton* m_cancelButton;         ///< 取消按钮
};

}  // namespace ui
}  // namespace insight

#endif  // UI_NEWPROJECTDIALOG_H
