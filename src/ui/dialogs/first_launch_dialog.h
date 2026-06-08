/**
 * @file FirstLaunchDialog.h
 * @brief Welcome dialog for first-time users to set work directory
 */

#ifndef UI_DIALOGS_FIRST_LAUNCH_DIALOG_H
#define UI_DIALOGS_FIRST_LAUNCH_DIALOG_H

#include <QDialog>

class QLineEdit;
class QPushButton;
class QLabel;

namespace insight {
namespace ui {

/**
 * @class FirstLaunchDialog
 * @brief First-launch welcome dialog
 *
 * Guides users to select a work directory where projects will be stored.
 */
class FirstLaunchDialog : public QDialog {
  Q_OBJECT

public:
  /**
   * Constructor
   * @param parent Parent widget
   */
  explicit FirstLaunchDialog(QWidget* parent = nullptr);

  ~FirstLaunchDialog();

  /**
   * Get the selected work directory
   * @return Work directory path (absolute)
   */
  QString get_selected_work_directory() const;

private slots:
  void on_browse_button_clicked();
  void on_ok_button_clicked();

private:
  QLineEdit* m_workDirLineEdit = nullptr;
  QPushButton* m_browseButton = nullptr;
  QPushButton* m_okButton = nullptr;
  QPushButton* m_cancelButton = nullptr;
  QLabel* m_descriptionLabel = nullptr;
  QString m_selectedDir;

  void init_ui();
  bool validate_selection();
};

}  // namespace ui
}  // namespace insight

#endif  // UI_DIALOGS_FIRST_LAUNCH_DIALOG_H
