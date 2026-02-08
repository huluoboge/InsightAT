/**
 * @file NewProjectDialog.cpp
 * @brief 新建项目对话框 - 实现
 */

#include "NewProjectDialog.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QMessageBox>

namespace insight {
namespace ui {

NewProjectDialog::NewProjectDialog(QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle("新建项目");
    setModal(true);
    setMinimumWidth(400);
    initializeUI();
}

NewProjectDialog::~NewProjectDialog() = default;

void NewProjectDialog::initializeUI()
{
    // 创建主布局
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->setSpacing(10);
    mainLayout->setContentsMargins(20, 20, 20, 20);

    // 项目名称
    QLabel* nameLabel = new QLabel("项目名称 *", this);
    m_projectNameEdit = new QLineEdit(this);
    m_projectNameEdit->setPlaceholderText("输入项目名称");
    mainLayout->addWidget(nameLabel);
    mainLayout->addWidget(m_projectNameEdit);

    // 作者（可选）
    QLabel* authorLabel = new QLabel("作者", this);
    m_authorEdit = new QLineEdit(this);
    m_authorEdit->setPlaceholderText("输入作者名称（可选）");
    mainLayout->addWidget(authorLabel);
    mainLayout->addWidget(m_authorEdit);

    // 描述（可选）
    QLabel* descLabel = new QLabel("项目描述", this);
    m_descriptionEdit = new QPlainTextEdit(this);
    m_descriptionEdit->setPlaceholderText("输入项目描述（可选）");
    m_descriptionEdit->setMaximumHeight(100);
    mainLayout->addWidget(descLabel);
    mainLayout->addWidget(m_descriptionEdit);

    // 添加伸缩
    mainLayout->addStretch();

    // 按钮布局
    QHBoxLayout* buttonLayout = new QHBoxLayout();
    buttonLayout->setSpacing(10);
    
    m_createButton = new QPushButton("创建项目", this);
    m_cancelButton = new QPushButton("取消", this);
    
    m_createButton->setMinimumWidth(100);
    m_cancelButton->setMinimumWidth(100);
    
    buttonLayout->addStretch();
    buttonLayout->addWidget(m_createButton);
    buttonLayout->addWidget(m_cancelButton);
    
    mainLayout->addLayout(buttonLayout);

    // 连接信号槽
    connect(m_createButton, &QPushButton::clicked, this, &NewProjectDialog::onCreateProject);
    connect(m_cancelButton, &QPushButton::clicked, this, &QDialog::reject);
    
    // 设置初始焦点
    m_projectNameEdit->setFocus();
}

void NewProjectDialog::onCreateProject()
{
    if (!validateInput()) {
        return;
    }

    QString name = m_projectNameEdit->text().trimmed();
    QString author = m_authorEdit->text().trimmed();
    QString description = m_descriptionEdit->toPlainText().trimmed();

    // 发出信号
    emit projectCreated(name, author, description);
    
    // 接受对话框
    accept();
}

bool NewProjectDialog::validateInput()
{
    // 检查项目名称是否为空
    if (m_projectNameEdit->text().trimmed().isEmpty()) {
        QMessageBox::warning(this, "输入错误", "项目名称不能为空！");
        m_projectNameEdit->setFocus();
        return false;
    }

    // 检查项目名称长度
    if (m_projectNameEdit->text().length() > 100) {
        QMessageBox::warning(this, "输入错误", "项目名称过长（最多100个字符）！");
        m_projectNameEdit->selectAll();
        m_projectNameEdit->setFocus();
        return false;
    }

    return true;
}

}  // namespace ui
}  // namespace insight
