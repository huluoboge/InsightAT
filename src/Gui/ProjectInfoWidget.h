#ifndef PROJECTINFOWIDGET_H
#define PROJECTINFOWIDGET_H

#include <QWidget>
#include "ui_ProjectInfoWidget.h"

#include <QDateTime>
#include "SubWidget.h"

namespace insight{
class ProjectInfoWidget : public SubWidget
{
	Q_OBJECT

public:
	ProjectInfoWidget(QWidget *parent = 0);
	~ProjectInfoWidget();
	virtual void init();
	virtual void refreshDatas();
    virtual void enable() {};
    virtual void disable(){};
    QString name() const;
	QDateTime date() const;
	QString author() const;
	QString description() const;

private:
	Ui::ProjectInfoWidget ui;
};
}//name space insight
#endif // PROJECTINFOWIDGET_H
