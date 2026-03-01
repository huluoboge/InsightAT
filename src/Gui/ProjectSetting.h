#ifndef PROJECT_SETTING_H
#define PROJECT_SETTING_H

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

class ProjectSetting : public SubWidget
{
    Q_OBJECT

public:
    ProjectSetting(QWidget *parent = nullptr);
    ~ProjectSetting();
    virtual void init();
    virtual void refreshDatas();
    virtual void enable() {}
    virtual void disable() {}
public slots:
    void changePage(QListWidgetItem *current, QListWidgetItem *previous);

    void save();
    void save(int row);
    void save(SubWidget *w);

private:
    void createIcons();

private:
    QListWidget *_contentsWidget;
    QStackedWidget *_pagesWidget;
    ProjectInfoWidget *_infoWidget;
    ProjectEditWidget *_prjEditWidget;
    ImageAttributes *_imgAttriWidget;
    GCPWidget *_gcpWdiget;
    ProjectCoordinateWidget *_projCoordWidget;
    QList<SubWidget *> _widgetList;
};

}//name space insight
#endif // PROJECT_SETTING_H
