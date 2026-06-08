/**
 * @file NewProjectDialog.h
 * @brief New Project Dialog
 *
 * Features:
 * 1. Input project name, author, description
 * 2. Auto-generate UUID and timestamp
 * 3. Emit projectCreated signal
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
 * @brief New Project Dialog
 */
class NewProjectDialog : public QDialog {
  Q_OBJECT

public:
  explicit NewProjectDialog(QWidget* parent = nullptr);
  ~NewProjectDialog();

signals:
  /**
   * Project creation signal
   *
   * @param name Project name
   * @param author Author (can be empty)
   * @param description Description (can be empty)
   */
  void projectCreated(const QString& name, const QString& author, const QString& description);

private slots:
  /**
   * Handle Create button click event
   */
  void onCreateProject();

private:
  /**
   * Initialize UI
   */
  void initializeUI();

  /**
   * Validate input
   *
   * @return True if valid
   */
  bool validateInput();

  // UI components
  QLineEdit* m_projectNameEdit;      ///< Project name input field
  QLineEdit* m_authorEdit;           ///< Author input field
  QPlainTextEdit* m_descriptionEdit; ///< Description input field
  QPushButton* m_createButton;       ///< Create button
  QPushButton* m_cancelButton;       ///< Cancel button
};

}  // namespace ui
}  // namespace insight

#endif  // UI_NEWPROJECTDIALOG_H
