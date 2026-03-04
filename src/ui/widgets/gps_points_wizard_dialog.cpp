/**
 * @file  GPSPointsWizardDialog.cpp
 * @brief 控制点/测量向导对话框实现。
 */

#include "gps_points_wizard_dialog.h"

#include "gps_points_wizard_delegate.h"
#include "util/string_utils.h"

#include <cassert>

#include <QFile>
#include <QFileInfo>
#include <QMessageBox>
#include <QTextStream>

namespace insight {

GPSPointsWizardDialog::GPSPointsWizardDialog(QWidget* parent) : QDialog(parent) {
  setWindowFlags(windowFlags() | Qt::WindowMaximizeButtonHint);
  ui.setupUi(this);
  m_model = new GPSPointsWizardModel;
  m_model->set_data_source(&m_document);
  ui.tableView->setModel(m_model);
  GPSPointsWizardDelegate* wizardDele = new GPSPointsWizardDelegate;
  wizardDele->set_database_document(this);
  ui.tableView->setItemDelegate(wizardDele);
}

GPSPointsWizardDialog::~GPSPointsWizardDialog() { delete m_model; }

void GPSPointsWizardDialog::setFile(const QString& fileFullPath) {
  QFile file(fileFullPath);
  assert(file.exists());
  ui.label_fileShow->setText(fileFullPath);
  file.open(QIODevice::ReadOnly);

  QTextStream stream(&file);
  QString txt = stream.readAll();
  ui.textEdit->setText(txt);
  m_document.m_txt = txt;
}

// bool GPSPointsWizardDialog::isLocalSystem() const
// {
// 	return ui.radioButton_local->isChecked();
// }

bool GPSPointsWizardDialog::is_import_by_name() const {
  return ui.radioButton_importByName->isChecked();
}

bool GPSPointsWizardDialog::is_import_rotation() const {
  // 检查是否存在旋转相关的复选框
  // 对应 UI 中的 "Import Rotation" checkbox
  // 暂时这里返回 false，您需要在 UI 中添加相应的 checkbox
  // 或者在子类中重写此方法
  return false; // TODO: 添加 UI checkbox 后修改此处
}

void GPSPointsWizardDialog::update_model() { m_model->update_datas(); }

void GPSPointsWizardDialog::preview() {
  bool isTAb = ui.checkBox_tab->isChecked();
  bool isSemicolon = ui.checkBox_semicolon->isChecked();
  bool isComma = ui.checkBox_comma->isChecked();
  bool isSpace = ui.checkBox_space->isChecked();
  bool isOther = ui.checkBox_other->isChecked();

  QString otherString;
  if (isOther) {
    otherString = ui.lineEdit_other->text();
  }
  bool multiAsSingle = ui.checkBox_multiAsSingle->isChecked();
  m_document.parse(isTAb, isSemicolon, isComma, isSpace, otherString, multiAsSingle);
  m_document.m_rowFrom = ui.spinBox_rowFrom->value() - 1;
  update_model();
}

void GPSPointsWizardDialog::check_enable_preview() {
  bool isTAb = ui.checkBox_tab->isChecked();
  bool isSemicolon = ui.checkBox_semicolon->isChecked();
  bool isComma = ui.checkBox_comma->isChecked();
  bool isSpace = ui.checkBox_space->isChecked();
  bool isOther = ui.checkBox_other->isChecked();

  if (isTAb || isSemicolon || isComma || isSpace || isOther) {
    preview();  // calls update_model internally
  }
}

void GPSPointsWizardDialog::valid_import() {
  if (valid()) {
    if (show_custom_dialog()) {
      accept();
    }
  }
}

void GPSPointsWizardDialog::enable_select_import_option(bool enable) {
  ui.radioButton_importByName->setEnabled(enable);
  ui.radioButton_2->setEnabled(enable);
}

// void GPSPointsWizardDialog::onSelectCoordinate()
// {
// 	WPOSpatialReferenceTool tool(this);
// 	if (tool.exec() == QDialog::Accepted){
// 		m_coord = tool.SelectCoordinate();
// 		ui.lineEdit_EPSG->setText(to_qs(m_coord.EPSGName));
// 	}
// }

bool GPSPointsWizardDialog::valid() {
  std::vector<int> fieldIndex;
  int rowFrom = 0;
  get_field_index(rowFrom, fieldIndex);
  bool ok = check_field_data(rowFrom, fieldIndex);
  return ok;
}

FieldConfiguration GPSPointsWizardDialog::get_field_configuration() const {
  // 默认实现：GPS 数据
  // 必需字段：经纬度或 XYZ
  // 可选字段：旋转角度（如果启用）
  FieldConfiguration config;
  config.requiredFields << "Latitude"
                        << "Longitude"
                        << "Height";
  config.requiredFields << "X"
                        << "Y"
                        << "Z"; // 或者用这些

  // 可选字段：旋转
  config.optionalFields << "Omega"
                        << "Phi"
                        << "Kappa";

  return config;
}

void GPSPointsWizardDialog::get_field_index(int& rowFrom, std::vector<int>& fieldIndex) {
  QList<QString> fields = field_names();
  fieldIndex.resize(fields.size(), -1);

  for (int i = 0; i < m_document.m_fields.count(); ++i) {
    for (int j = 0; j < fields.size(); ++j) {
      if (m_document.m_fields[i] == fields[j]) {
        fieldIndex[j] = i;
      }
    }
  }
  rowFrom = m_document.m_rowFrom;
}

QList<QString> GPSPointsWizardDialog::field_names() const {
  return get_field_configuration().getAllFields();
}

bool GPSPointsWizardDialog::check_field_data(int rowFrom, const std::vector<int>& fieldIndex) {
  FieldConfiguration config = get_field_configuration();
  for (int i = 0; i < config.requiredFields.size(); ++i) {
    if (i >= static_cast<int>(fieldIndex.size()) || fieldIndex[i] == -1) {
      return false;
    }
  }
  return true;
}

}  // namespace insight
