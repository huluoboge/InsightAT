/**
 * @file  CoordinateSystemConfigDialog.cpp
 * @brief 坐标系配置对话框实现。
 */

#include "coordinate_system_config_dialog.h"

#include "ui/widgets/coordinate_system_config_widget.h"

#include <QDialogButtonBox>
#include <QVBoxLayout>

namespace insight {
namespace ui {

CoordinateSystemConfigDialog::CoordinateSystemConfigDialog(QWidget* parent) : QDialog(parent) {
  setWindowTitle("Configure Coordinate System");
  setModal(true);
  setMinimumWidth(600);
  setMinimumHeight(500);
  initialize_ui();
}

void CoordinateSystemConfigDialog::initialize_ui() {
  QVBoxLayout* mainLayout = new QVBoxLayout(this);
  mainLayout->setSpacing(10);
  mainLayout->setContentsMargins(10, 10, 10, 10);

  // 创建配置小部件
  m_configWidget = new CoordinateSystemConfigWidget(this);
  mainLayout->addWidget(m_configWidget);

  // 创建按钮框
  QDialogButtonBox* buttonBox =
      new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);

  // 获取 OK 按钮并初始状态为禁用
  QPushButton* okBtn = buttonBox->button(QDialogButtonBox::Ok);
  okBtn->setEnabled(m_configWidget->is_valid());

  connect(m_configWidget, &CoordinateSystemConfigWidget::validation_changed, this,
          &CoordinateSystemConfigDialog::on_validation_changed);

  // 连接 OK/Cancel 按钮
  connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
  connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);

  mainLayout->addWidget(buttonBox);
  setLayout(mainLayout);
}

void CoordinateSystemConfigDialog::on_validation_changed(bool valid) {
  QPushButton* okBtn = findChild<QPushButton*>();
  QDialogButtonBox* buttonBox = findChild<QDialogButtonBox*>();
  if (buttonBox) {
    okBtn = buttonBox->button(QDialogButtonBox::Ok);
    if (okBtn) {
      okBtn->setEnabled(valid);
    }
  }
}

database::CoordinateSystem CoordinateSystemConfigDialog::get_coordinate_system() const {
  return m_configWidget->get_coordinate_system();
}

void CoordinateSystemConfigDialog::set_coordinate_system(const database::CoordinateSystem& coord_sys) {
  m_configWidget->set_coordinate_system(coord_sys);
}

}  // namespace ui
}  // namespace insight
