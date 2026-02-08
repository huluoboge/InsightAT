#include "GPSPointsWizardDialog.h"

#include <QFile>
#include <QTextStream>
#include <QMessageBox>
#include <QFileInfo>
#include "SpatialReferenceTool.h"
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

bool GPSPointsWizardDialog::hasOmegaPhiKappa() const
{
	return ui.groupBox_RotationSystem->isChecked();
}

int GPSPointsWizardDialog::angleUnit() const
{
	return ui.radioButton_3->isChecked() ? 0 : 1;
}

int GPSPointsWizardDialog::coordinateSystem() const
{
	return ui.radioButton_5->isChecked() ? 0 : 1;
}

int GPSPointsWizardDialog::eulerAngleSystem() const
{
	return ui.radioButton_7->isChecked() ? 0 : 1;
}

}//name space insight
