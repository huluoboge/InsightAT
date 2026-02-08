#include "ProjectEditWidget.h"

#include <QHBoxLayout>

namespace insight{

ProjectEditWidget::ProjectEditWidget(QWidget *parent)
	: SubWidget(parent)
{
	QHBoxLayout *layout = new QHBoxLayout(this);
	_splitter = new QSplitter(Qt::Horizontal,this);
	_camEdit = new CameraEdit;
	_imageAttribute = new ImageAttributes;
	_imageAttribute->setEditCameraEnabled(false);
	_splitter->addWidget(_camEdit);
	_splitter->addWidget(_imageAttribute);
	layout->addWidget(_splitter);

	connect(_camEdit, &CameraEdit::currentCameraChanged, _imageAttribute,
		&ImageAttributes::bindCamera);
	connect(_imageAttribute, &ImageAttributes::projectChanged, this, &ProjectEditWidget::refreshDatas);
	setWindowTitle(tr("Edit Project"));
}

void ProjectEditWidget::setShowTask(const std::string taskId, bool isOrigin)
{
    _camEdit->setShowTask(taskId,isOrigin);
    _imageAttribute->setShowTask(taskId,isOrigin);
}

ProjectEditWidget::~ProjectEditWidget()
{

}

void ProjectEditWidget::init()
{
	_camEdit->init();
	_imageAttribute->init();
}

void ProjectEditWidget::refreshDatas()
{
	_camEdit->refreshDatas();
	_imageAttribute->refreshDatas();
}

}//name space insight
