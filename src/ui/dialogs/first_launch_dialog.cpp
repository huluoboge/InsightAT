/**
 * @file FirstLaunchDialog.cpp
 * @brief First-launch welcome dialog implementation
 */

#include "first_launch_dialog.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QFileDialog>
#include <QDir>
#include <QStandardPaths>
#include <QMessageBox>
#include <glog/logging.h>

namespace insight {
namespace ui {

FirstLaunchDialog::FirstLaunchDialog(QWidget* parent)
    : QDialog(parent) {
  setWindowTitle("Welcome to InsightAT – Set Up Work Directory");
  setMinimumWidth(600);
  setMinimumHeight(250);
  setModal(true);
  
  init_ui();
  
  // Suggest default directory
  QString default_dir = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation) 
                        + "/InsightAT/Projects";
  m_workDirLineEdit->setText(default_dir);
  m_selectedDir = default_dir;
}

FirstLaunchDialog::~FirstLaunchDialog() = default;

void FirstLaunchDialog::init_ui() {
  QVBoxLayout* mainLayout = new QVBoxLayout(this);

  // Title
  QLabel* titleLabel = new QLabel("Welcome to InsightAT");
  titleLabel->setStyleSheet("font-size: 16pt; font-weight: bold;");
  mainLayout->addWidget(titleLabel);

  // Description
  m_descriptionLabel = new QLabel(
      "Please select a directory where your InsightAT projects will be stored.\n"
      "Each project will be saved as:\n"
      "  • project_name.iat (project file)\n"
      "  • project_name.iat.data/ (working directory)"
  );
  m_descriptionLabel->setWordWrap(true);
  m_descriptionLabel->setStyleSheet("color: #666;");
  mainLayout->addWidget(m_descriptionLabel);

  mainLayout->addSpacing(20);

  // Directory selection
  QHBoxLayout* dirLayout = new QHBoxLayout();
  
  QLabel* dirLabel = new QLabel("Work Directory:");
  dirLabel->setMinimumWidth(100);
  dirLayout->addWidget(dirLabel);
  
  m_workDirLineEdit = new QLineEdit();
  m_workDirLineEdit->setReadOnly(false);
  m_workDirLineEdit->setMinimumHeight(32);
  dirLayout->addWidget(m_workDirLineEdit);
  
  m_browseButton = new QPushButton("Browse...");
  m_browseButton->setMinimumWidth(100);
  m_browseButton->setMinimumHeight(32);
  connect(m_browseButton, &QPushButton::clicked, this, &FirstLaunchDialog::on_browse_button_clicked);
  dirLayout->addWidget(m_browseButton);
  
  mainLayout->addLayout(dirLayout);

  mainLayout->addSpacing(20);

  // Buttons
  QHBoxLayout* buttonLayout = new QHBoxLayout();
  buttonLayout->addStretch();
  
  m_cancelButton = new QPushButton("Cancel");
  m_cancelButton->setMinimumWidth(100);
  m_cancelButton->setMinimumHeight(36);
  connect(m_cancelButton, &QPushButton::clicked, this, &QDialog::reject);
  buttonLayout->addWidget(m_cancelButton);
  
  m_okButton = new QPushButton("Next");
  m_okButton->setMinimumWidth(100);
  m_okButton->setMinimumHeight(36);
  m_okButton->setStyleSheet("font-weight: bold;");
  connect(m_okButton, &QPushButton::clicked, this, &FirstLaunchDialog::on_ok_button_clicked);
  buttonLayout->addWidget(m_okButton);
  
  mainLayout->addLayout(buttonLayout);

  setLayout(mainLayout);
}

void FirstLaunchDialog::on_browse_button_clicked() {
  QString dir = QFileDialog::getExistingDirectory(
      this,
      "Select Work Directory",
      m_workDirLineEdit->text(),
      QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks
  );
  
  if (!dir.isEmpty()) {
    m_workDirLineEdit->setText(dir);
    m_selectedDir = dir;
  }
}

void FirstLaunchDialog::on_ok_button_clicked() {
  if (!validate_selection()) {
    return;
  }
  
  m_selectedDir = QDir::cleanPath(m_workDirLineEdit->text());
  accept();
}

bool FirstLaunchDialog::validate_selection() {
  QString path = QDir::cleanPath(m_workDirLineEdit->text());
  
  if (path.isEmpty()) {
    QMessageBox::warning(this, "Invalid Path", "Please select a work directory.");
    return false;
  }
  
  QDir dir(path);
  
  // Try to create if doesn't exist
  if (!dir.exists()) {
    if (!dir.mkpath(".")) {
      QMessageBox::critical(this, "Cannot Create Directory",
          QString("Cannot create directory: %1").arg(path));
      return false;
    }
    LOG(INFO) << "Created new work directory: " << path.toStdString();
  }
  
  return true;
}

QString FirstLaunchDialog::get_selected_work_directory() const {
  return m_selectedDir;
}

}  // namespace ui
}  // namespace insight
