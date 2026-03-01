#include "AddPhotosDialog.h"

#include <QString>
#include <cassert>
#include "AT3dRenderWidget.h"
namespace insight{
AddPhotosDialog::AddPhotosDialog(QWidget *parent)
	: QDialog(parent)
	, m_selectIndex(-1)
{
	ui.setupUi(this);
    ui.lineEdit_newGroupName->setVisible(false);

}

AddPhotosDialog::~AddPhotosDialog()
{

}

void AddPhotosDialog::initGroups(const QStringList &groupNames, const QList<int> &groupIds)
{
	m_groupNames = groupNames;
	m_groupIds = groupIds;
	ui.comboBox_groupNames->addItems(m_groupNames);
	if (m_groupNames.empty())
	{
		ui.radioButton_curGroup->setEnabled(false);
	}
}

AddPhotosDialog::AddCameraType AddPhotosDialog::addCameraType() const
{
    if (ui.radioButton_newGroup->isChecked()) return eNew;
    if (ui.radioButton_curGroup->isChecked()) return eSelect;
    if (ui.radioButton_groupByExif->isChecked()) return eByExif;
	
	CHECK(false) << "unknown type";
	return eNew;
}

int AddPhotosDialog::selectCameraId() const
{
	int index = ui.comboBox_groupNames->currentIndex();
	assert(index != -1);
	return m_groupIds[index];
}

void AddPhotosDialog::setNextCameraId(int groupId)
{
	ui.lineEdit_newGroupName->setText(QString("Camera%1").arg(groupId + 1));
}


void AddPhotosDialog::enableSetGroup(bool enable)
{
	ui.groupBox_addPhoto->setEnabled(enable);
}

}//name space insight
