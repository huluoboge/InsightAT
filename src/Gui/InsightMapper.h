#ifndef INSIGHTMAPPER_H
#define INSIGHTMAPPER_H

#include <QtWidgets/QMainWindow>
#include "ui_InsightMapper.h"
#include <QProgressBar>
#include <QThread>
#include "InsightATGlobal.h"
#include "MainWindowFrame.h"
#include <thread>

class QTreeWidgetItem;


class QLabel;

namespace insight{

class Workspace ;

class InsightMapper : public QMainWindow,
        public MainWindowFrame
{
	Q_OBJECT
	signals:
	void refreshDatas();
    void waitingProcess();
    //MainWindowInterface functions
public:
    virtual void openProject() {
        onOpenProject();
    }
    virtual void newProject() {
        onNewProject();
    }

    virtual void saveProject(){
        onSaveProject();
    }

	virtual void closeAllMdiWindows();

	virtual void closeProject()
	{
		onCloseProject();
	}

	virtual void showWelcomePage();
	
    virtual void openProject(const QString &prj);

     virtual void refreshProject();

    virtual QWidget *widget() {return this;}
public:
	InsightMapper(QWidget *parent = 0);
	~InsightMapper();

public slots:
	void initMdiWindows();
	void addWelcomeWidget();
    void addATConfigWidget(const QString &taskId,const QString &name);
    void addModelConfigWidget(const QString &taskId,const QString &name);
    void addImageAttributesWidget(int camId = -1);


	void onNewProject();
	void onSaveProject();
	void onOpenProject();
	void onCloseProject();
	void onRefresh();
	void onShowAttributes();
	void onSetWorkspaceVisible();
	void onSetLogVisible();
	void onSetWelcomVisible();
	void onEditProject();
	void onWizard();
	void onShowTitle();
	void onEditGCP();
	void refreshWorkspace();
	private slots:
	void onSetBar(float p);
	void onSetMsg(const QString &msg);
	void onSetTitle(const QString &msg);
	void onWaitingProcess();
	void onStartProcess();
	void updateActionState();
	void onRegister();
	
private slots:
    void onWorkspaceDoubleClicked(QTreeWidgetItem *item, int column);
private:
    void checkImageAttributes(QMdiSubWindow *);
	void closeEvent(QCloseEvent *e);
	void addImages(const QStringList &imgs);
//	void toSfmData(SfM_Data &data);
	void del_tree_item(QTreeWidgetItem* node);
	void setWelcomeWidgetEnabled(bool val);
	
	Ui::InsightMapperClass ui;
	QProgressBar *_progressBar;
	std::thread  _progThread;
	QMainWindow *_mapWindow = nullptr;
    QList<QAction*> _progressActions;//disable
    QList<QAction*> _projectOpenActions;//enable

    Workspace *_workspace;


};


}//name space insight
#endif // INSIGHTMAPPER_H
