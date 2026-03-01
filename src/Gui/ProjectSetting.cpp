#include "ProjectSetting.h"

#include "ProjectInfoWidget.h"
#include "ProjectEditWidget.h"
#include "ImageAttributes.h"
#include "Document.h"
#include "ProjectCoordinateWidget.h"
#include "gcpwidget.h"

namespace insight{
ProjectSetting::ProjectSetting(QWidget *parent)
	: SubWidget(parent)
{
	_contentsWidget = new QListWidget;
	_contentsWidget->setViewMode(QListView::IconMode);
	_contentsWidget->setIconSize(QSize(96, 84));
	_contentsWidget->setMovement(QListView::Static);
	_contentsWidget->setMaximumWidth(128);
	_contentsWidget->setSpacing(12);

	_pagesWidget = new QStackedWidget;
	_infoWidget = new ProjectInfoWidget;
	_prjEditWidget = new ProjectEditWidget;
	_imgAttriWidget = new ImageAttributes;
	_projCoordWidget = new ProjectCoordinateWidget;
	_gcpWdiget = new GCPWidget;
	_pagesWidget->addWidget(_infoWidget);
	_pagesWidget->addWidget(_prjEditWidget);
	_pagesWidget->addWidget(_imgAttriWidget);
	_pagesWidget->addWidget(_gcpWdiget);
	_pagesWidget->addWidget(_projCoordWidget);
	_widgetList.push_back(_infoWidget);
	_widgetList.push_back(_prjEditWidget);
	_widgetList.push_back(_imgAttriWidget);
	_widgetList.push_back(_gcpWdiget);
	_widgetList.push_back(_projCoordWidget);
	createIcons();
	_contentsWidget->setCurrentRow(0);

	QHBoxLayout *horizontalLayout = new QHBoxLayout;
	horizontalLayout->addWidget(_contentsWidget);
	horizontalLayout->addWidget(_pagesWidget, 1);
	setLayout(horizontalLayout);

	setWindowTitle(tr("Edit Project"));
}

ProjectSetting::~ProjectSetting()
{

}

void ProjectSetting::init()
{
	refreshDatas();
}

void ProjectSetting::refreshDatas()
{
	_infoWidget->refreshDatas();
	_prjEditWidget->refreshDatas();
	_imgAttriWidget->refreshDatas();
	_projCoordWidget->refreshDatas();
	_gcpWdiget->refreshDatas();
}

void ProjectSetting::createIcons()
{
	QListWidgetItem *infoButton = new QListWidgetItem(_contentsWidget);
	//configButton->setIcon(QIcon(":/images/config.png"));
	infoButton->setText(tr("Information"));
	infoButton->setTextAlignment(Qt::AlignHCenter);
	infoButton->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);

	QListWidgetItem *cameraButton = new QListWidgetItem(_contentsWidget);
	//cameraButton->setIcon(QIcon(":/images/update.png"));
	cameraButton->setText(tr("Camera"));
	cameraButton->setTextAlignment(Qt::AlignHCenter);
	cameraButton->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);

	QListWidgetItem *imageSettingButton = new QListWidgetItem(_contentsWidget);
	//queryButton->setIcon(QIcon(":/images/query.png"));
	imageSettingButton->setText(tr("Image"));
	imageSettingButton->setTextAlignment(Qt::AlignHCenter);
	imageSettingButton->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);

	QListWidgetItem *gcpButton = new QListWidgetItem(_contentsWidget);
	//gcpButton->setIcon(QIcon(":/images/gcp.png"));
	gcpButton->setText(tr("GCP"));
	gcpButton->setTextAlignment(Qt::AlignHCenter);
	gcpButton->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);

	QListWidgetItem *projCoordButton = new QListWidgetItem(_contentsWidget);
	//projCoordButton->setIcon(QIcon(":/images/query.png"));
	projCoordButton->setText(tr("Coordination"));
	projCoordButton->setTextAlignment(Qt::AlignHCenter);
	projCoordButton->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);

	connect(_contentsWidget,
		SIGNAL(currentItemChanged(QListWidgetItem*, QListWidgetItem*)),
		this, SLOT(changePage(QListWidgetItem*, QListWidgetItem*)));
}

void ProjectSetting::changePage(QListWidgetItem *current, QListWidgetItem *previous)
{
	if (previous != NULL){
		//save previous
        //project().updateENUCoord();
		for (int i = 0; i < _contentsWidget->count(); ++i){
			if (_contentsWidget->item(i) == previous){
				save(i);
			}
		}
	}

	if (!current)
		current = previous;
	int row = _contentsWidget->row(current);
	_pagesWidget->setCurrentIndex(row);
	_widgetList[row]->refreshDatas();
}

void ProjectSetting::save()
{
	for (int i = 0; i < _widgetList.size(); ++i){
		save(_widgetList[i]);
	}
    //project().updateENUCoord();
}

void ProjectSetting::save(int row)
{
	SubWidget *curWidget = _widgetList[row];
	save(curWidget);
}

void ProjectSetting::save(SubWidget*w)
{
	if (w == _infoWidget)
	{
		project().infomation.author = tos(_infoWidget->author());
		project().infomation.name = tos(_infoWidget->name());
		project().infomation.description = tos(_infoWidget->description());
		QDateTime dt = _infoWidget->date();
		project().infomation.date = tos(dt.toString("yyyy/MM/dd hh:mm:ss"));
	}
	else if (w == _projCoordWidget)
	{
		project().infomation.averageElevationOfGround = _projCoordWidget->averageElevation();
		project().infomation.relativeFlightAltitude = _projCoordWidget->flyingHeight();
	}
	else if (w == _gcpWdiget){
		_gcpWdiget->saveDatas();
	}
}

}//name space insight
