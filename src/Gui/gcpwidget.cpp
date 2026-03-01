#include <QFileDialog>
#include <QTableWidgetItem>
#include <QComboBox>
#include <QRadioButton>

#include "gcpwidget.h"
#include "ImportGPSDialog.h"

#include "Settings.h"
#include "Common/db_types.h"
#include "Common/Project.h"
#include "Document.h"
#include "Common/string_utils.h"


namespace insight{
GCPWidget::GCPWidget(QWidget *parent)
	: SubWidget(parent)
{
	ui.setupUi(this);

	connect(ui.pushButton_import, SIGNAL(clicked()), this, SLOT(onImport()));
	connect(ui.pushButton_clear, SIGNAL(clicked()), this, SLOT(onClear()));
	connect(ui.tableWidget, SIGNAL(itemChanged(QTableWidgetItem*)), this, SLOT(saveDatas()));
}

GCPWidget::~GCPWidget()
{

}

void GCPWidget::refreshDatas()
{
	_refreshing = true;
	QTableWidget *table = ui.tableWidget;
	table->setColumnHidden(0, !_edit);
	table->setColumnHidden(8, project().infomation.coordinate.localSystem);
	table->setColumnHidden(9, project().infomation.coordinate.localSystem);
	table->setColumnHidden(10, project().infomation.coordinate.localSystem);

	ui.tableWidget->clearContents();
	const std::map<uint32_t, DBGCP> &gcpList = project().gcpList.GCP_List();
	ui.tableWidget->setRowCount(gcpList.size());
	int iRow = 0;
	for (auto itr = gcpList.begin(); itr != gcpList.end(); ++itr, ++iRow){
		int col = 0;
		QRadioButton *editButton = new QRadioButton;
		editButton->setChecked(false);
		int gcpId = itr->first;
		connect(editButton, &QRadioButton::toggled, [this, gcpId](bool checked){
			if (checked){
				emit gcpStartEditing(gcpId);
			}
		});
		ui.tableWidget->setCellWidget(iRow, col++, editButton);

		QTableWidgetItem * item = new QTableWidgetItem;
		item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
		if (itr->second.enabled){
			item->setCheckState(Qt::Checked);
		}
		else{
			item->setCheckState(Qt::Unchecked);
		}

		ui.tableWidget->setItem(iRow, col++, item);
		item = new QTableWidgetItem;
		item->setText(QString::number(itr->first));
		ui.tableWidget->setItem(iRow, col++, item);
		item = new QTableWidgetItem;
		item->setText(toqs(itr->second.name));
		ui.tableWidget->setItem(iRow, col++, item);

		QComboBox *box = new QComboBox;
		box->addItem(toqs("Control"));
		box->addItem(toqs("Check"));
		if (itr->second.type == 0){
			box->setCurrentIndex(1);
		}
		else{
			box->setCurrentIndex(0);
		}
		connect(box, SIGNAL(currentIndexChanged(int)), this, SLOT(saveDatas()));
		ui.tableWidget->setCellWidget(iRow, col++, box);
		item = new QTableWidgetItem;
		item->setData(Qt::DisplayRole, QString::number(itr->second.landmark.x,'f',6));
		ui.tableWidget->setItem(iRow, col++, item);
		item = new QTableWidgetItem;
		item->setData(Qt::DisplayRole, QString::number(itr->second.landmark.y,'f',6));
		ui.tableWidget->setItem(iRow, col++, item);
		item = new QTableWidgetItem;
		item->setData(Qt::DisplayRole, QString::number(itr->second.landmark.z,'f',6));
		ui.tableWidget->setItem(iRow, col++, item);
	}
	_refreshing = false;
}

void GCPWidget::saveDatas()
{
	if (_refreshing) return;
	for (int row = 0; row != ui.tableWidget->rowCount(); ++row){
		int col = 0;
		QRadioButton *ra = (QRadioButton*)(ui.tableWidget->cellWidget(row, col++));
		
		bool enabled = ui.tableWidget->item(row, col++)->checkState() == Qt::Checked;
		int gcpID = ui.tableWidget->item(row, col++)->text().toInt();
		std::string name = tos(ui.tableWidget->item(row, col++)->text());
		QComboBox *box = (QComboBox*)(ui.tableWidget->cellWidget(row, col++));
		int type = box->currentIndex() == 0 ? 1 : 0;
		double x = ui.tableWidget->item(row, col++)->text().toDouble();
		double y = ui.tableWidget->item(row, col++)->text().toDouble();
		double z = ui.tableWidget->item(row, col++)->text().toDouble();
		CHECK(project().gcpList.GCP_List().find(gcpID) != project().gcpList.GCP_List().end());
		DBGCP &gcp = project().gcpList.GCP_List().at(gcpID);
		gcp.name = name;
		gcp.type = type;
		gcp.landmark.x = x;
		gcp.landmark.y = y;
		gcp.landmark.z = z;
		gcp.enabled = enabled ? 1 : 0;
	}
	doc().setModify(true);
}

void GCPWidget::onImport(){
	QString file = QFileDialog::getOpenFileName(this, tr("select control points"), settings().recentPath(),
		"ascii format control file (*.*)");
	if (file.isEmpty()) return;
    ImportGPSDialog dlg(this);
	dlg.enableSelectImportOption(false);
	dlg.setFile(file);
	dlg.preview();
	if (QDialog::Accepted == dlg.exec()){
        ImportGPSDialog::Vec_Pose points = dlg.Points();
		Resource *rc = &(project().resource);
		for (int i = 0; i < points.size(); ++i){
			DBGCP gcp;
			gcp.track_id = rc->gcpSeed.generate();
			gcp.name = points[i].name;
			gcp.landmark.x = points[i].x;
			gcp.landmark.y = points[i].y;
			gcp.landmark.z = points[i].z;
			gcp.type = DBGCP::GCP_CONTROL;
			project().gcpList.GCP_List()[gcp.track_id] = gcp;
		}
        //project().updateENUCoord();
		refreshDatas();
		doc().setModify(true);
	}
}

void GCPWidget::onAdd(){

}

void GCPWidget::onClear()
{
	_refreshing = true;
	ui.tableWidget->clearContents();
	ui.tableWidget->setRowCount(0);
	_refreshing = false;
	project().gcpList.GCP_List().clear();
	doc().setModify(true);
	emit gcpCleared();
}

void GCPWidget::onDelete()
{
}

}//name space insight
