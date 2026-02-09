#include "CoordinateSystemConfigDialog.h"
#include "ui/widgets/CoordinateSystemConfigWidget.h"

#include <QVBoxLayout>
#include <QDialogButtonBox>

namespace insight {
namespace ui {

CoordinateSystemConfigDialog::CoordinateSystemConfigDialog(QWidget* parent)
    : QDialog(parent) {
    setWindowTitle("Configure Coordinate System");
    setModal(true);
    setMinimumWidth(600);
    setMinimumHeight(500);
    initializeUI();
}

void CoordinateSystemConfigDialog::initializeUI() {
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->setSpacing(10);
    mainLayout->setContentsMargins(10, 10, 10, 10);

    // 创建配置小部件
    m_configWidget = new CoordinateSystemConfigWidget(this);
    mainLayout->addWidget(m_configWidget);

    // 创建按钮框
    QDialogButtonBox* buttonBox = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);

    // 获取 OK 按钮并初始状态为禁用
    QPushButton* okBtn = buttonBox->button(QDialogButtonBox::Ok);
    okBtn->setEnabled(m_configWidget->IsValid());

    // 连接验证信号到 OK 按钮启用状态
    connect(m_configWidget, &CoordinateSystemConfigWidget::validationChanged,
            this, &CoordinateSystemConfigDialog::onValidationChanged);

    // 连接 OK/Cancel 按钮
    connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);

    mainLayout->addWidget(buttonBox);
    setLayout(mainLayout);
}

void CoordinateSystemConfigDialog::onValidationChanged(bool valid) {
    QPushButton* okBtn = findChild<QPushButton*>();
    QDialogButtonBox* buttonBox = findChild<QDialogButtonBox*>();
    if (buttonBox) {
        okBtn = buttonBox->button(QDialogButtonBox::Ok);
        if (okBtn) {
            okBtn->setEnabled(valid);
        }
    }
}

database::CoordinateSystem CoordinateSystemConfigDialog::GetCoordinateSystem() const {
    return m_configWidget->GetCoordinateSystem();
}

void CoordinateSystemConfigDialog::SetCoordinateSystem(const database::CoordinateSystem& coordSys) {
    m_configWidget->SetCoordinateSystem(coordSys);
}

}  // namespace ui
}  // namespace insight
