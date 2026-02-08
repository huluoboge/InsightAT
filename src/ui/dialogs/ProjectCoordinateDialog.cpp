/**
 * @file ProjectCoordinateDialog.cpp
 * @brief 项目坐标系选择对话框实现
 */

#include "ProjectCoordinateDialog.h"
#include "../widgets/ProjectCoordinateWidget.h"
#include "../../database/database_types.h"

#include <QVBoxLayout>
#include <QDialogButtonBox>
#include <QPushButton>
#include <QMessageBox>

namespace insight {
namespace ui {

ProjectCoordinateDialog::ProjectCoordinateDialog(QWidget* parent)
    : QDialog(parent) {
    setWindowTitle(tr("Select Project Coordinate System"));
    setMinimumWidth(500);
    setMinimumHeight(300);
    initializeUI();
}

ProjectCoordinateDialog::~ProjectCoordinateDialog() {
}

void ProjectCoordinateDialog::initializeUI() {
    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    // 创建坐标系选择 widget
    m_coordWidget = new ProjectCoordinateWidget();
    mainLayout->addWidget(m_coordWidget);

    // 底部按钮
    QDialogButtonBox* buttonBox = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    
    connect(buttonBox, &QDialogButtonBox::accepted, this, [this]() {
        if (m_coordWidget->IsValid()) {
            accept();
        } else {
            QMessageBox::warning(this, tr("Invalid Selection"),
                tr("Please select a valid coordinate system."));
        }
    });
    
    connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    mainLayout->addWidget(buttonBox);

    setLayout(mainLayout);
}

insight::database::CoordinateSystem ProjectCoordinateDialog::GetCoordinateSystem() const {
    if (m_coordWidget) {
        return m_coordWidget->GetCoordinateSystem();
    }
    return insight::database::CoordinateSystem();
}

void ProjectCoordinateDialog::SetCoordinateSystem(const insight::database::CoordinateSystem& coordSys) {
    if (m_coordWidget) {
        m_coordWidget->SetCoordinateSystem(coordSys);
    }
}

}  // namespace ui
}  // namespace insight
