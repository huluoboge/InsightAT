#include "ProjectWizard.h"

#include "ProjectInfoWidget.h"
#include "ProjectEditWidget.h"
#include "ImageAttributes.h"
#include "Document.h"
#include "ProjectCoordinateWidget.h"
#include "gcpwidget.h"

namespace insight{
ProjectWizard::ProjectWizard(QWidget *parent)
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
//    _imgAttriWidget = new ImageAttributes;
    _projCoordWidget = new ProjectCoordinateWidget;
    _gcpWdiget = new GCPWidget;
    _pagesWidget->addWidget(_infoWidget);
    _pagesWidget->addWidget(_projCoordWidget);

    _pagesWidget->addWidget(_prjEditWidget);
//    _pagesWidget->addWidget(_imgAttriWidget);
    _pagesWidget->addWidget(_gcpWdiget);

    _widgetList.push_back(_infoWidget);
    _widgetList.push_back(_projCoordWidget);
    _widgetList.push_back(_prjEditWidget);
//    _widgetList.push_back(_imgAttriWidget);
    _widgetList.push_back(_gcpWdiget);

    createIcons();
    _contentsWidget->setCurrentRow(0);

	QVBoxLayout *layout = new QVBoxLayout;
    QHBoxLayout *horizontalLayout = new QHBoxLayout;
    horizontalLayout->addWidget(_contentsWidget);
    horizontalLayout->addWidget(_pagesWidget, 1);
	layout->addLayout(horizontalLayout);
	_preButton = new QPushButton;
    _preButton->setText(tr("Previous"));
    _nextButton = new QPushButton;
    _nextButton->setText(tr("Next"));
    QHBoxLayout *buttonLayout = new QHBoxLayout;
    buttonLayout->addStretch();
    buttonLayout->addWidget(_preButton);
    buttonLayout->addWidget(_nextButton);
    layout->addLayout(buttonLayout);
    setLayout(layout);

    setWindowTitle(tr("Project wizard"));
    _preButton->setEnabled(false);
    connect(_preButton, SIGNAL(clicked()), this, SLOT(onPre()));
    connect(_nextButton, SIGNAL(clicked()), this, SLOT(onNext()));
}

ProjectWizard::~ProjectWizard()
{

}

void ProjectWizard::init()
{
    refreshDatas();
}

void ProjectWizard::refreshDatas()
{
    _infoWidget->refreshDatas();
    _projCoordWidget->refreshDatas();
    _prjEditWidget->refreshDatas();
//    _imgAttriWidget->refreshDatas();

    _gcpWdiget->refreshDatas();
}

void ProjectWizard::createIcons()
{
    QListWidgetItem *infoButton = new QListWidgetItem(_contentsWidget);
    //configButton->setIcon(QIcon(":/images/config.png"));
    infoButton->setText(tr("Information"));
    infoButton->setTextAlignment(Qt::AlignHCenter);
    infoButton->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);

    QListWidgetItem *projCoordButton = new QListWidgetItem(_contentsWidget);
    //projCoordButton->setIcon(QIcon(":/images/query.png"));
    projCoordButton->setText(tr("Coordination"));
    projCoordButton->setTextAlignment(Qt::AlignHCenter);
    projCoordButton->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);

    QListWidgetItem *cameraButton = new QListWidgetItem(_contentsWidget);
    //cameraButton->setIcon(QIcon(":/images/update.png"));
    cameraButton->setText(tr("Camera"));
    cameraButton->setTextAlignment(Qt::AlignHCenter);
    cameraButton->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);

//    QListWidgetItem *imageSettingButton = new QListWidgetItem(_contentsWidget);
//    //queryButton->setIcon(QIcon(":/images/query.png"));
//    imageSettingButton->setText(tr("Image"));
//    imageSettingButton->setTextAlignment(Qt::AlignHCenter);
//    imageSettingButton->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);

    QListWidgetItem *gcpButton = new QListWidgetItem(_contentsWidget);
    //gcpButton->setIcon(QIcon(":/images/gcp.png"));
    gcpButton->setText(tr("GCP"));
    gcpButton->setTextAlignment(Qt::AlignHCenter);
    gcpButton->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);



    connect(_contentsWidget,
        SIGNAL(currentItemChanged(QListWidgetItem*, QListWidgetItem*)),
        this, SLOT(changePage(QListWidgetItem*, QListWidgetItem*)));
}

void ProjectWizard::changePage(QListWidgetItem *current, QListWidgetItem *previous)
{
    if (previous != nullptr){
        //save previous
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
    if(_curPage != row){
        _curPage = row;
        updateButtonState();
    }
}

void ProjectWizard::save()
{
    for (int i = 0; i < _widgetList.size(); ++i){
        save(_widgetList[i]);
    }
    //project().updateENUCoord();
}

void ProjectWizard::save(int row)
{
    SubWidget *curWidget = _widgetList[row];
    save(curWidget);
}

void ProjectWizard::save(SubWidget*w)
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

void ProjectWizard::onPre()
{
    --_curPage;
    _contentsWidget->setCurrentRow(_curPage);
    updateButtonState();
}

void ProjectWizard::onNext()
{
    ++_curPage;
    if(_curPage > _contentsWidget->count()-1){
        save();
        emit closeWindow();
        return ;
    }
    _contentsWidget->setCurrentRow(_curPage);
    updateButtonState();
}

void ProjectWizard::updateButtonState()
{
     _preButton->setEnabled(true);
     _nextButton->setEnabled(true);
     _nextButton->setText(tr("Next"));
    if(_curPage == _contentsWidget->count()-1){
//        _nextButton->setEnabled(false);
        _nextButton->setText(tr("Finish"));
    }
    if(_curPage == 0){
        _preButton->setEnabled(false);
    }

}
}//name space insight
