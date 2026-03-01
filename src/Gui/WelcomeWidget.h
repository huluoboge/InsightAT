#ifndef WELCOMEWIDGET_H
#define WELCOMEWIDGET_H

#include <QWidget>

#include "SubWidget.h"

#include "ui_WelcomeWidget.h"

namespace insight{
class WelcomeWidget : public SubWidget
{
	Q_OBJECT

public:
	WelcomeWidget(QWidget *parent = 0);
	~WelcomeWidget();

public:
	void init();
	void refreshDatas();
    void enable();
    void disable();
	public slots:
	void onNewProject();
	void onOpenProject();
	void onShowProjectInfo(QListWidgetItem*);
	void onShowProjectInfo();
	void onOpenRecentProject(QModelIndex);
	void onClearRecentProjects();
private:
	Ui::WelcomeWidget ui;
    QList<QWidget*> actions;
};
}//name space insight
#endif // WELCOMEWIDGET_H
