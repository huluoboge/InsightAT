#include "WelcomeWidget.h"

#include <QDebug>
#include <QFileInfo>
#include <QMessageBox>

#include "Common/db_types.h"
#include "Common/Project.h"

#include "Settings.h"
#include "Document.h"
#include "MainWindowFrame.h"

namespace insight{

WelcomeWidget::WelcomeWidget(QWidget *parent)
    : SubWidget(parent)
{
    ui.setupUi(this);
    actions.push_back(ui.pushButton_open_project);
    actions.push_back(ui.pushButton_new_project);
    actions.push_back(ui.pushButton_clear);
    actions.push_back(ui.listWidget_recent);
    actions.push_back(ui.plainTextEdit_description);
    actions.push_back(ui.lineEdit_projectName);
}

WelcomeWidget::~WelcomeWidget()
{
}

void WelcomeWidget::init()
{
    refreshDatas();
}
void WelcomeWidget::refreshDatas()
{
    QStringList recentFiles = settings().recentProjects();
    qDebug() << recentFiles;
    qDebug() << "Welcome refreshed!";
    ui.listWidget_recent->clear();
    for (int i = 0; i < recentFiles.size(); ++i)
    {
        QFileInfo info(recentFiles[i]);
        QListWidgetItem *item = new QListWidgetItem(recentFiles[i], ui.listWidget_recent);
        if (!info.exists())
        {
            item->setIcon(QIcon(":/InsightMapper/error"));
        }
        item->setData(Qt::UserRole + 1, recentFiles[i]);
    }
    //UPDATE RECENT
    //SystemConfig
    //ui.listWidget_recent->
}

void WelcomeWidget::enable()
{
    for (QWidget *w : actions)
    {
        w->setEnabled(true);
    }
}

void WelcomeWidget::disable()
{
    for (QWidget *w : actions)
    {
        w->setDisabled(true);
    }
}

void WelcomeWidget::onNewProject()
{
    theWindow->newProject();
}

void WelcomeWidget::onOpenProject()
{
    theWindow->openProject();
}

void WelcomeWidget::onShowProjectInfo(QListWidgetItem *item)
{
    if (item != NULL)
    {
        QString prj = item->data(Qt::UserRole + 1).toString();
        Project p;
        if (!p.getProjectInfomation(tos(prj)))
        {
            //int errCode = LastError();
            //qDebug() << ErrorString(errCode);
            return;
        }
        QString name = toqs(p.infomation.name);
        QString desc = toqs(p.infomation.description);
        ui.lineEdit_projectName->setText(name);
        ui.plainTextEdit_description->setPlainText(desc);
    }
}

void WelcomeWidget::onShowProjectInfo()
{
    QListWidgetItem *item = ui.listWidget_recent->currentItem();
    onShowProjectInfo(item);
}

void WelcomeWidget::onOpenRecentProject(QModelIndex modelIndex)
{
    if (modelIndex.isValid())
    {
        int r = modelIndex.row();
        QListWidgetItem *item = ui.listWidget_recent->item(r);
        if (item != NULL)
        {
            QString prj = item->data(Qt::UserRole + 1).toString();
            if (doc().isOpen())
            {
                //message,close or cancel
                int ret = QMessageBox::information(this, tr("promote"), tr("Save current project ?"),
                                                   QMessageBox::Yes, QMessageBox::No, QMessageBox::Cancel);
                if (ret == QMessageBox::Cancel)
                {
                    return;
                }
                else if (ret == QMessageBox::Yes)
                {
                    theWindow->saveProject();
                    theWindow->closeAllMdiWindows();
                    theWindow->showWelcomePage();
                    theWindow->openProject(prj);
                }
                else if (ret == QMessageBox::No)
                {
                    theWindow->closeAllMdiWindows();
                    theWindow->showWelcomePage();
                    theWindow->openProject(prj);
                }
            }
            else
            {
                theWindow->openProject(prj);
            }
        }
    }
}

void WelcomeWidget::onClearRecentProjects()
{
    int btn = QMessageBox::information(this, tr("promt"), tr("Clear recent projects?"), QMessageBox::Yes, QMessageBox::No);
    if (btn == QMessageBox::Yes)
    {
        settings().setRecentProjects(QStringList());
        refreshDatas();
    }
}

}//name space insight
