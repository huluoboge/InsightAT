#ifndef PROJECT_WIZARD_H
#define PROJECT_WIZARD_H

#include <QObject>
#include <QWidget>
#include <QStackedWidget>
#include <QListWidget>
#include "SubWidget.h"

namespace insight{

class ProjectInfoWidget;
class ImageAttributes;
class ProjectEditWidget;
class ProjectCoordinateWidget;
class GCPWidget;

class ProjectWizard : public SubWidget
{
    Q_OBJECT

public:
    ProjectWizard(QWidget *parent = nullptr);
    ~ProjectWizard();
    virtual void init();
    virtual void refreshDatas();
    virtual void enable() {}
    virtual void disable() {}
public slots:
    void changePage(QListWidgetItem *current, QListWidgetItem *previous);

    void save();
    void save(int row);
    void save(SubWidget *w);
    void onPre();
    void onNext();
private:
    void createIcons();
    void updateButtonState();
private:
    int _curPage = 0;
    QListWidget *_contentsWidget;
    QStackedWidget *_pagesWidget;
    ProjectInfoWidget *_infoWidget;
    ProjectEditWidget *_prjEditWidget;
//    ImageAttributes *_imgAttriWidget;
    GCPWidget *_gcpWdiget;
    ProjectCoordinateWidget *_projCoordWidget;
    QList<SubWidget *> _widgetList;
	QPushButton *_preButton;
	QPushButton *_nextButton;
};

}//name space insight
#endif // PROJECT_SETTING_H
