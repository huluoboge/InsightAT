/**
 * @file NewProjectDialog.cpp
 * @brief New Project Dialog - Implementation
 */

#include "new_project_dialog.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QVBoxLayout>

namespace insight {
namespace ui {

NewProjectDialog::NewProjectDialog(QWidget* parent) : QDialog(parent) {
  setWindowTitle(tr("New Project"));
  setModal(true);
  setMinimumWidth(400);
  initializeUI();
}

NewProjectDialog::~NewProjectDialog() = default;

void NewProjectDialog::initializeUI() {
  // Create main layout
  QVBoxLayout* mainLayout = new QVBoxLayout(this);
  mainLayout->setSpacing(10);
  mainLayout->setContentsMargins(20, 20, 20, 20);

  // Project Name
  QLabel* nameLabel = new QLabel(tr("Project Name *"), this);
  m_projectNameEdit = new QLineEdit(this);
  m_projectNameEdit->setPlaceholderText(tr("Enter project name"));
  mainLayout->addWidget(nameLabel);
  mainLayout->addWidget(m_projectNameEdit);

  // Author (optional)
  QLabel* authorLabel = new QLabel(tr("Author"), this);
  m_authorEdit = new QLineEdit(this);
  m_authorEdit->setPlaceholderText(tr("Enter author name (optional)"));
  mainLayout->addWidget(authorLabel);
  mainLayout->addWidget(m_authorEdit);

  // Description (optional)
  QLabel* descLabel = new QLabel(tr("Project Description"), this);
  m_descriptionEdit = new QPlainTextEdit(this);
  m_descriptionEdit->setPlaceholderText(tr("Enter project description (optional)"));
  m_descriptionEdit->setMaximumHeight(100);
  mainLayout->addWidget(descLabel);
  mainLayout->addWidget(m_descriptionEdit);

  // Add stretch
  mainLayout->addStretch();

  // Button layout
  QHBoxLayout* buttonLayout = new QHBoxLayout();
  buttonLayout->setSpacing(10);

  m_createButton = new QPushButton(tr("Create Project"), this);
  m_cancelButton = new QPushButton(tr("Cancel"), this);

  m_createButton->setMinimumWidth(100);
  m_cancelButton->setMinimumWidth(100);

  buttonLayout->addStretch();
  buttonLayout->addWidget(m_createButton);
  buttonLayout->addWidget(m_cancelButton);

  mainLayout->addLayout(buttonLayout);

  // Connect signals and slots
  connect(m_createButton, &QPushButton::clicked, this, &NewProjectDialog::onCreateProject);
  connect(m_cancelButton, &QPushButton::clicked, this, &QDialog::reject);

  // Set initial focus
  m_projectNameEdit->setFocus();
}

void NewProjectDialog::onCreateProject() {
  if (!validateInput()) {
    return;
  }

  QString name = m_projectNameEdit->text().trimmed();
  QString author = m_authorEdit->text().trimmed();
  QString description = m_descriptionEdit->toPlainText().trimmed();

  // Emit signal
  emit projectCreated(name, author, description);

  // Accept dialog
  accept();
}

bool NewProjectDialog::validateInput() {
  // Check if project name is empty
  if (m_projectNameEdit->text().trimmed().isEmpty()) {
    QMessageBox::warning(this, tr("Input Error"), tr("Project name cannot be empty!"));
    m_projectNameEdit->setFocus();
    return false;
  }

  // Check project name length
  if (m_projectNameEdit->text().length() > 100) {
    QMessageBox::warning(this, tr("Input Error"),
                         tr("Project name is too long (maximum 100 characters)!"));
    m_projectNameEdit->selectAll();
    m_projectNameEdit->setFocus();
    return false;
  }

  return true;
}

}  // namespace ui
}  // namespace insight
