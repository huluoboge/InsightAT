#include "GPSPointsWizardDialog.h"

#include <QFile>
#include <QTextStream>
#include <QMessageBox>
#include <QFileInfo>
#include "Common/string_utils.h"

#include <cassert>

#include "GPSPointsWizardDelegate.h"

namespace insight{

GPSPointsWizardDialog::GPSPointsWizardDialog(QWidget *parent)
	: QDialog(parent)
{
	setWindowFlags(windowFlags() | Qt::WindowMaximizeButtonHint);
	ui.setupUi(this);
	m_model = new GPSPointsWizardModel;
	m_model->setDataSource(&m_document);
	ui.tableView->setModel(m_model);
	GPSPointsWizardDelegate *wizardDele = new GPSPointsWizardDelegate;
	wizardDele->setDataBseDocument(this);
	ui.tableView->setItemDelegate(wizardDele);
}

GPSPointsWizardDialog::~GPSPointsWizardDialog()
{
	delete m_model;
}

void GPSPointsWizardDialog::setFile(const QString &fileFullPath)
{
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

bool GPSPointsWizardDialog::isImportByName() const
{
	return ui.radioButton_importByName->isChecked();
}

bool GPSPointsWizardDialog::isImportRotation() const
{
	// 检查是否存在旋转相关的复选框
	// 对应 UI 中的 "Import Rotation" checkbox
	// 暂时这里返回 false，您需要在 UI 中添加相应的 checkbox
	// 或者在子类中重写此方法
	return false;  // TODO: 添加 UI checkbox 后修改此处
}

void GPSPointsWizardDialog::updateModel()
{
	m_model->updateDatas();
}

void GPSPointsWizardDialog::preview()
{
	bool isTAb = ui.checkBox_tab->isChecked();
	bool isSemicolon = ui.checkBox_semicolon->isChecked();
	bool isComma = ui.checkBox_comma->isChecked();
	bool isSpace = ui.checkBox_space->isChecked();
	bool isOther = ui.checkBox_other->isChecked();

	QString otherString;
	if (isOther)
	{
		otherString = ui.lineEdit_other->text();
	}
	bool multiAsSingle = ui.checkBox_multiAsSingle->isChecked();
	m_document.parse(isTAb, isSemicolon, isComma, isSpace, otherString, multiAsSingle);
	m_document.m_rowFrom = ui.spinBox_rowFrom->value() - 1;
	updateModel();
}

void GPSPointsWizardDialog::checkEnablePreview()
{
	bool isTAb = ui.checkBox_tab->isChecked();
	bool isSemicolon = ui.checkBox_semicolon->isChecked();
	bool isComma = ui.checkBox_comma->isChecked();
	bool isSpace = ui.checkBox_space->isChecked();
	bool isOther = ui.checkBox_other->isChecked();

	if (isTAb || isSemicolon || isComma || isSpace || isOther)
	{
		preview();
	}
}

void GPSPointsWizardDialog::validImport()
{
	if (valid())
	{
		if (showCustemDialog())
		{
			accept();
		}
	}
}

void GPSPointsWizardDialog::enableSelectImportOption(bool enable)
{
	ui.radioButton_importByName->setEnabled(enable);
	ui.radioButton_2->setEnabled(enable);
}

// void GPSPointsWizardDialog::onSelectCoordinate()
// {
// 	WPOSpatialReferenceTool tool(this);
// 	if (tool.exec() == QDialog::Accepted){
// 		m_coord = tool.SelectCoordinate();
// 		ui.lineEdit_EPSG->setText(toqs(m_coord.EPSGName));
// 	}
// }

bool GPSPointsWizardDialog::valid()
{
	std::vector<int> fieldIndex;
	int rowFrom = 0;
	getFieldIndex(rowFrom, fieldIndex);
	bool ok = checkFieldData(rowFrom, fieldIndex);
	return ok;
}

FieldConfiguration GPSPointsWizardDialog::getFieldConfiguration() const
{
	// 默认实现：GPS 数据
	// 必需字段：经纬度或 XYZ
	// 可选字段：旋转角度（如果启用）
	FieldConfiguration config;
	config.requiredFields << "Latitude" << "Longitude" << "Height";
	config.requiredFields << "X" << "Y" << "Z";  // 或者用这些
	
	// 可选字段：旋转
	config.optionalFields << "Omega" << "Phi" << "Kappa";
	
	return config;
}

void GPSPointsWizardDialog::getFieldIndex(int &rowFrom, std::vector<int> &fieldIndex)
{
	QList<QString> fields = fieldNames();
	fieldIndex.resize(fields.size(), -1);

	int maxCol = -1;
	for (int i = 0; i < m_document.m_fields.count(); ++i)
	{
		for (int j = 0; j < fields.size(); ++j)
		{
			if (m_document.m_fields[i] == fields[j])
			{
				fieldIndex[j] = i;
			}
		}
	}
	rowFrom = m_document.m_rowFrom;
}

}//name space insight
