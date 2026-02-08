#include "CheckImageWidget.h"
#include "Common/string_utils.h"
#include "Document.h"

using namespace insight;
CheckImageWidget::CheckImageWidget(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);
}

CheckImageWidget::~CheckImageWidget()
{

}

void CheckImageWidget::refreshDatas()
{
	std::vector<int> notValid;
	for (int i = 0; i < _consistency.size(); ++i)
	{
		if (!_consistency[i].isOK()){
			notValid.push_back(i);
		}
	}
	QTableWidget *table = ui.tableWidget;
	table->clearContents();
	table->setRowCount(notValid.size());
	if (notValid.empty()){
        ui.labelResult->setText(tr("OK"));
	}
	else{
        ui.labelResult->setText(tr("Find some erros"));
		for (int r = 0; r < notValid.size(); ++r)
		{
			ImageConsistency &consis = _consistency[notValid[r]];

			QTableWidgetItem *item = new QTableWidgetItem;
			item->setData(Qt::DisplayRole, consis.imageId);
			table->setItem(r, 0, item);
			item = new QTableWidgetItem;
			item->setData(Qt::DisplayRole, consis.cameraId);
			table->setItem(r, 1, item);
			item = new QTableWidgetItem;
			item->setData(Qt::DisplayRole, consis.imageW);
			table->setItem(r, 2, item);
			item = new QTableWidgetItem;
			item->setData(Qt::DisplayRole, consis.cameraW);
			table->setItem(r, 3, item);
			item = new QTableWidgetItem;
			item->setData(Qt::DisplayRole, consis.imageH);
			table->setItem(r, 4, item);
			item = new QTableWidgetItem;
			item->setData(Qt::DisplayRole, consis.cameraH);
			table->setItem(r, 5, item);
			item = new QTableWidgetItem;
			item->setText(consis.imageExist? "Yes": "No");
			table->setItem(r, 6, item);
			item = new QTableWidgetItem;
			item->setText(consis.imageCanRead ? "Yes" : "No");
			table->setItem(r, 7, item);
		}
	}
}
