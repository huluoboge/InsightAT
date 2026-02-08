#include "InsightMapper.h"

#include "Document.h"
#include "InsightATGlobal.h"
#include "Settings.h"
#include <QDateTime>
#include <QDebug>
#include <QDialogButtonBox>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QLabel>
#include <QMdiSubWindow>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QProcess>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>

#include "glog/logging.h"
#include "ogr_spatialref.h"

#include "Common/Coordinates.h"
#include "Common/gps.h"
#include "Common/threading.h"

#include "ATConfigWidget.h"
#include "ModelConfigWidget.h"

#include "ATParamWidget.h"
#include "About.h"
#include "AddPhotosDialog.h"
#include "ImageAttributes.h"
#include "NewProjectDialog.h"
#include "ProgressDialog.h"
#include "ProjectSetting.h"
#include "ProjectWizard.h"
#include "WelcomeWidget.h"
#include "Workspace.h"

namespace insight{

InsightMapper::InsightMapper(QWidget* parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
    QMainWindow::setCorner(Qt::TopLeftCorner, Qt::LeftDockWidgetArea);
    QMainWindow::setCorner(Qt::TopRightCorner, Qt::RightDockWidgetArea);
    QMainWindow::setCorner(Qt::BottomLeftCorner, Qt::LeftDockWidgetArea);
    QMainWindow::setCorner(Qt::BottomRightCorner, Qt::RightDockWidgetArea);

    //    QStatusBar *statusBar = ui.statusBar;
    _progressBar = ui.progressBar;
    _progressBar->setRange(0, 100);

    ui.plainTextEdit_log->setCenterOnScroll(true);
    ui.plainTextEdit_log->setReadOnly(true);
    ui.plainTextEdit_log->setTextInteractionFlags(Qt::TextBrowserInteraction);

    connect(this, SIGNAL(refreshDatas()), this, SLOT(onRefresh()), Qt::QueuedConnection);
    connect(&doc(), SIGNAL(modifying()), this, SLOT(onShowTitle()), Qt::QueuedConnection);
    connect(&doc(), SIGNAL(openStateChanged()), this, SLOT(updateActionState()));
    connect(this, SIGNAL(waitingProcess()), this, SLOT(onWaitingProcess()));
    onShowTitle();

    _progressActions.push_back(ui.actionNew);
    _progressActions.push_back(ui.actionOpen);
    _progressActions.push_back(ui.actionSave);
    _progressActions.push_back(ui.actionProject);
    _progressActions.push_back(ui.actionGCP);

    _projectOpenActions.push_back(ui.actionSave);
    _projectOpenActions.push_back(ui.actionClose);
    _projectOpenActions.push_back(ui.actionProject);
    _projectOpenActions.push_back(ui.actionGCP);

    _workspace = ui.treeWidget;
    updateActionState();
    connect(_workspace, SIGNAL(itemDoubleClicked(QTreeWidgetItem*, int)),
        this, SLOT(onWorkspaceDoubleClicked(QTreeWidgetItem*, int)));

    ui.dockWidget_log->setVisible(false);
    //    ui.mdiArea->setTabsClosable(false);
}

InsightMapper::~InsightMapper()
{
    if (_progThread.joinable()) {
        _progThread.join();
    }
}

void InsightMapper::closeAllMdiWindows()
{
    ui.mdiArea->closeAllSubWindows();
}

void InsightMapper::showWelcomePage()
{
    addWelcomeWidget();
}

void InsightMapper::initMdiWindows()
{
    addWelcomeWidget();
}

void InsightMapper::addWelcomeWidget()
{
    {
        QList<QMdiSubWindow*> subWidgetList = ui.mdiArea->subWindowList();
        for (int i = 0; i < subWidgetList.size(); ++i) {
            if (subWidgetList[i]->objectName() == "Welcome") {
                subWidgetList[i]->showMaximized();
                ui.mdiArea->setActiveSubWindow(subWidgetList[i]);
                return;
            }
        }
    }

    WelcomeWidget* widget = new WelcomeWidget(this);
    widget->init();
    QMdiSubWindow* subWindow = ui.mdiArea->addSubWindow(widget);
    subWindow->setObjectName("Welcome");
    subWindow->showMaximized();
    ui.mdiArea->setActiveSubWindow(subWindow);
}

void InsightMapper::addATConfigWidget(const QString& taskId, const QString& name)
{
    QList<QMdiSubWindow*> subWidgetList = ui.mdiArea->subWindowList();
    for (int i = 0; i < subWidgetList.size(); ++i) {
        if (subWidgetList[i]->objectName() == taskId) {
            subWidgetList[i]->showMaximized();
            ui.mdiArea->setActiveSubWindow(subWidgetList[i]);
            return;
        }
    }
    ATConfigWidget* widget = new ATConfigWidget(this);
    std::string stdTaskId = tos(taskId);
    widget->setTask(stdTaskId);
    QString title = QString("AT-%1").arg(name);
    widget->setWindowTitle(title);
    widget->init();
    MdiSubWindow* subWindow = new MdiSubWindow;
    //����Widget
    subWindow->setCloseFunction([stdTaskId, this]() -> bool {
        if (SingleEngine::instance().isRunning() && SingleEngine::instance().currentTaskName() == stdTaskId) {
            QMessageBox::information(this, tr("Warnning"), tr("Processing is running..."));
            return false;
        }
        return true;
    });

    subWindow->setWidget(widget);
    subWindow->setAttribute(Qt::WA_DeleteOnClose);
    subWindow->setObjectName(taskId);
    ui.mdiArea->addSubWindow(subWindow);
    subWindow->showMaximized();
    ui.mdiArea->setActiveSubWindow(subWindow);
}

void InsightMapper::addModelConfigWidget(const QString& taskId, const QString& name)
{
    QList<QMdiSubWindow*> subWidgetList = ui.mdiArea->subWindowList();
    for (int i = 0; i < subWidgetList.size(); ++i) {
        if (subWidgetList[i]->objectName() == taskId) {
            subWidgetList[i]->showMaximized();
            ui.mdiArea->setActiveSubWindow(subWidgetList[i]);
            return;
        }
    }
    ModelConfigWidget* widget = new ModelConfigWidget(this);
    std::string stdTaskId = tos(taskId);
    widget->setTask(stdTaskId);
    QString title = QString("Model-%1").arg(name);
    widget->setWindowTitle(title);
    widget->init();
    MdiSubWindow* subWindow = new MdiSubWindow;
    subWindow->setCloseFunction([stdTaskId, this]() -> bool {
        if (SingleEngine::instance().isRunning() && SingleEngine::instance().currentTaskName() == stdTaskId) {
            QMessageBox::information(this, tr("Warnning"), tr("Processing is running..."));
            return false;
        }
        return true;
    });

    subWindow->setWidget(widget);
    subWindow->setAttribute(Qt::WA_DeleteOnClose);
    subWindow->setObjectName(taskId);
    ui.mdiArea->addSubWindow(subWindow);
    subWindow->showMaximized();
    ui.mdiArea->setActiveSubWindow(subWindow);
}

void InsightMapper::addImageAttributesWidget(int camId)
{
    QList<QMdiSubWindow*> subWidgetList = ui.mdiArea->subWindowList();
    for (int i = 0; i < subWidgetList.size(); ++i) {
        if (subWidgetList[i]->objectName() == "ImageAttributes") {
            subWidgetList[i]->showMaximized();
            ImageAttributes* widget = (ImageAttributes*)subWidgetList[i]->widget();
            if (camId != -1) {
                widget->setEditCameraEnabled(false);
                widget->bindCamera(camId);
                widget->refreshDatas();
            }
            ui.mdiArea->setActiveSubWindow(subWidgetList[i]);
            return;
        }
    }
    ImageAttributes* widget = new ImageAttributes(this);
    if (camId != -1) {
        widget->setEditCameraEnabled(false);
        widget->bindCamera(camId);
    }
    widget->init();
    QMdiSubWindow* subWindow = ui.mdiArea->addSubWindow(widget);
    subWindow->setObjectName("ImageAttributes");
    subWindow->showMaximized();
    ui.mdiArea->setActiveSubWindow(subWindow);
}
void InsightMapper::openProject(const QString& file)
{
    _progThread = std::thread([this, file]() {
        LOG(INFO) << "Opening project... " << tos(file);
        Project p;
        if (p.openProject(tos(file))) {
            QFileInfo info(file);
            settings().setRecentProjectPath(info.absolutePath());
            settings().addProjectToRecent(file);
            project() = p;
            doc().setOpen(true);
            doc().setModify(false);
            LOG(INFO) << "Project opened";
            emit waitingProcess();
            emit refreshDatas();
        } else {
            LOG(INFO) << "Can't open project " << tos(file);
        }
    });
}

void InsightMapper::refreshProject()
{
    onRefresh();
}

void InsightMapper::onNewProject()
{
    //remider save or close;
    if (doc().isOpen() && doc().isModified()) {
        //message,close or cancel
        int ret = QMessageBox::information(this, tr("promote"), tr("Save current project ?"),
            QMessageBox::Yes, QMessageBox::No, QMessageBox::Cancel);
        if (ret == QMessageBox::Yes) {
            onSaveProject();
        } else if (ret == QMessageBox::No) {
        } else if (ret == QMessageBox::Cancel) {
            return;
        }
    }

    Project prj;

    NewProjectDialog dlg(this);
    if (dlg.exec() == QDialog::Accepted) {
        //close the project first
        project() = Project();
        doc().setOpen(false);
        onRefresh();

        ProjectInfomation info;
        info.name = tos(dlg.name());
        info.description = tos(dlg.description());
        info.date = tos(QDateTime::currentDateTime().toString("yyyy/MM/dd hh:mm:ss"));
        info.author = tos(dlg.author());
        QString prjFile = dlg.location() + "/" + dlg.name() + toqs(Document::projectExt);
        if (!prj.createProject(info, tos(prjFile)) || !prj.saveProject()) {
            LOG(INFO) << "Create file " << tos(prjFile) << " failed";
            return;
        }

        project() = prj;
        doc().setOpen(true);
        doc().setModify(false);
        QDir prjPathDir(dlg.location());
        settings().setRecentProjectPath(prjPathDir.absolutePath());
        settings().addProjectToRecent(prjFile);
        ProjectWizard* wizard = new ProjectWizard(this);
        wizard->init();
        wizard->onNext();
        showSubWidget(this, wizard);
        project().saveProject();
    }
    onRefresh();
}

void InsightMapper::onSaveProject()
{
    if (!project().saveProject()) {
        LOG(ERROR) << "Save project failed";
    } else {
        doc().setModify(false);
        LOG(INFO) << "Project saved";
    }
}

void InsightMapper::onOpenProject()
{
    if (doc().isOpen() && doc().isModified()) {
        //message,close or cancel
        int ret = QMessageBox::information(this, tr("promote"), tr("Save current project ?"),
            QMessageBox::Yes, QMessageBox::No, QMessageBox::Cancel);
        if (ret == QMessageBox::Cancel) {
            return;
        }
        if (ret == QMessageBox::Yes) {
            onSaveProject();
        }
        ui.mdiArea->closeAllSubWindows();
    }
    QString file = QFileDialog::getOpenFileName(this, tr("Open project"), settings().recentProjectPath(),
        tr("insight AT project(*") + toqs(Document::projectExt) + ")");
    if (file.isEmpty())
        return;
    openProject(file);
}

void InsightMapper::onCloseProject()
{
    if (doc().isOpen()) {
        //message,close or cancel
        int ret = QMessageBox::information(this, tr("promote"), tr("Save current project ?"),
            QMessageBox::Yes, QMessageBox::No, QMessageBox::Cancel);
        if (ret == QMessageBox::Cancel) {
            return;
        }
        if (ret == QMessageBox::Yes) {
            onSaveProject();
        }
        project() = Project();
        doc().setOpen(false);
        onRefresh();
    }
}

void InsightMapper::checkImageAttributes(QMdiSubWindow* window)
{
    if (window->objectName() == "ImageAttributes") {
        ImageAttributes* widget = (ImageAttributes*)window->widget();
        if (widget->bindedCamera() != UndefinedKey) {
            KeyType camId = widget->bindedCamera();
            if (project().cameraList.Camera_list().find(camId)
                == project().cameraList.Camera_list().end()) {
                ui.mdiArea->setActiveSubWindow(window);
                ui.mdiArea->closeActiveSubWindow();
            }
        }
    }
}
void InsightMapper::onRefresh()
{
    refreshWorkspace();
    if (doc().isOpen()) {
        QList<QMdiSubWindow*> subWindows = ui.mdiArea->subWindowList();
        QMdiSubWindow* imageAttWindow = nullptr;
        for (QMdiSubWindow* window : subWindows) {
            QWidget* w = window->widget();
            SubWidget* subWidget = qobject_cast<SubWidget*>(w);
            if (subWidget != nullptr) {
                subWidget->refreshDatas();
                if (window->objectName() == "ImageAttributes") {
                    imageAttWindow = window;
                }
            }
        }
        if (imageAttWindow != nullptr)
            checkImageAttributes(imageAttWindow);
        onShowTitle();
    } else {
        ui.mdiArea->closeAllSubWindows();
        addWelcomeWidget();
        onShowTitle();
    }
}

void InsightMapper::onShowAttributes()
{

    ImageAttributes* widget = new ImageAttributes;
    showSubWidget(this, widget);
    if (widget->changedProject) {
        refreshWorkspace();
    }
}

void InsightMapper::onSetWorkspaceVisible()
{
    ui.dockWidget_workspace->setVisible(true);
}

void InsightMapper::onSetLogVisible()
{
    ui.dockWidget_log->setVisible(true);
}

void InsightMapper::onSetWelcomVisible()
{
    addWelcomeWidget();
}

void InsightMapper::onEditProject()
{

    QDialog dlg(this);
    ProjectSetting* widget = new ProjectSetting(&dlg);
    widget->init();
    QHBoxLayout* layout = new QHBoxLayout(&dlg);
    layout->addWidget(widget);
    dlg.setWindowTitle("Edit Project");
    dlg.setWindowFlags(dlg.windowFlags() & ~Qt::WindowContextHelpButtonHint | Qt::WindowMaximizeButtonHint);
    dlg.showMaximized();
    dlg.exec();
    widget->save();
    doc().setModify(true);
    emit refreshDatas();
    refreshWorkspace();
}

void InsightMapper::onWizard()
{
    //	addWizardWidget();
}

void InsightMapper::onShowTitle()
{

    if (doc().isOpen()) {
        QString prjFile = doc().currentFile();
        QString title = QString("Insight AT %1").arg(prjFile);
        if (doc().isModified()) {
            title += "*";
        }
        setWindowTitle(title);
    } else {
        setWindowTitle(QString("Insight AT"));
    }
}

void InsightMapper::onEditGCP()
{
    if (doc().isOpen()) {
#ifdef WIN32
        QString program = qApp->applicationDirPath() + "/ControlEdit.exe";
#else
        QString program = qApp->applicationDirPath() + "/ControlEdit";
#endif
        QStringList args;
        args << toqs(project().projectDataDir)
             << "0"; //disable open action
        qDebug() << args;
        QProcess::execute(program, args);
    }
}

void InsightMapper::onSetBar(float p)
{

    //	_progressBar->setValue(p * 100);
}

void InsightMapper::onSetMsg(const QString& msg)
{
    QString content = QString("<font color = \"blue\"> %1%2</font>")
                          .arg(QDateTime::currentDateTime().toString("hh:mm:ss "))
                          .arg(msg);
    ui.plainTextEdit_log->appendHtml(content);
}

void InsightMapper::onSetTitle(const QString& msg)
{
    ui.label_log_title->setText(QString("Processing:") + msg);
}

void InsightMapper::onWaitingProcess()
{
    LOG(INFO) << "Waiting process...";
    if (_progThread.joinable())
        _progThread.join();

    for (QAction* a : _progressActions) {
        a->setEnabled(true);
    }
    setWelcomeWidgetEnabled(true);
}

void InsightMapper::onStartProcess()
{
    // 	for (QAction *a : _progressActions){
    // 		a->setEnabled(false);
    // 	}
    // 	setWelcomeWidgetEnabled(false);
}

void InsightMapper::closeEvent(QCloseEvent* e)
{
    // 	if (_progThread.joinable()){
    // 		int btn = QMessageBox::information(this, toqs("promote"),
    // 			toqs("is processing, stop now?"),
    // 			QDialogButtonBox::Yes, QDialogButtonBox::Cancel);
    // 		if (btn == QDialogButtonBox::Cancel){
    // 			e->ignore();  //�����˳��źţ������������
    // 			return;
    // 		}
    // 		else if (btn == QDialogButtonBox::Yes){
    // 			LOG(INFO) << "Is stopping..." ;
    // 			//prog.setProgress(NULL);
    // 			//prog.Stop();
    // 			QPProgressDialog dlg(this);
    // 			struct TempThread : public Thread
    // 			{
    // 				std::thread *_t;
    // 				TempThread(std::thread *t){ _t = t; }
    // 			private:
    // 				void Run(){
    // 					_t->join();
    // 				}
    // 			};
    //  			TempThread tt(&_progThread);
    //  			dlg.setProcessThread(&tt);
    // 			dlg.exec();
    // 		}
    // 	}
    // 	if (doc().haveFile() && doc().isModified()){
    // 		int btn = QMessageBox::information(this, toqs("Promote"),toqs("Project has been changed, save first?"),
    // 			QDialogButtonBox::Yes, QDialogButtonBox::No, QDialogButtonBox::Cancel);
    // 		if (btn == QDialogButtonBox::Cancel){
    // 			e->ignore();  //�����˳��źţ������������
    // 			return;
    // 		}
    // 		else if (btn == QDialogButtonBox::Yes){
    // 			doc().save();
    // 			LOG(INFO) << "Project saved" ;
    // 		}
    // 	}
    // 	e->accept();
}

void InsightMapper::addImages(const QStringList& fileList)
{
#if 0
    if (fileList.empty()) return;

    std::set<std::string> imgs;
    for (int i = 0; i < fileList.size(); ++i)
    {
        imgs.insert(tos(fileList[i]));
    }
    QList<int> ids;
    QStringList groupNames;
    for (auto itr = project().cameraList.Camera_list().begin();
        itr != project().cameraList.Camera_list().end(); ++itr)
    {
        ids.push_back(itr->first);
        groupNames << toqs(itr->second.camera_name);
    }

    AddPhotosDialog optionDialog(this);
    optionDialog.initGroups(groupNames, ids);
    optionDialog.setNextGroupId(project().resource.cameraSeed.seed);
    if (optionDialog.exec() == QDialog::Accepted)
    {
        AddPhotosDialog::AddGroupType groupType = optionDialog.addGroupType();
        KeyType camera_key = 0;
        if (groupType == AddPhotosDialog::eNewGroup)// new camera
        {
            camera_key = project().resource.cameraSeed.generate();
            project().cameraList.Camera_list()[camera_key].id = camera_key;
            project().cameraList.Camera_list()[camera_key].camera_name = tos(optionDialog.newGroupName());
        }
        else if (groupType == AddPhotosDialog::eSelectGroup)//
        {
            camera_key = optionDialog.selectGroupId();
        }
        else if (groupType == AddPhotosDialog::eEXIF)//by exif
        {
            camera_key = UndefinedKey;
        }

        _progThread = std::thread([this, imgs, camera_key, groupType](){
            std::vector<int> imageAdded = project().imageListGen.importImages(imgs, camera_key, &project().resource);
            if (imageAdded.size() > 0){
                if (groupType == AddPhotosDialog::eEXIF){
                    project().generateCameraByExif(imageAdded);
                }
                if (groupType == AddPhotosDialog::eNewGroup) {
                    project().completeCameraByExif(imageAdded,camera_key);
                }
                doc().setModify(true);
            }
            emit waitingProcess();
            emit refreshDatas();

        });
    }
#endif
}

void InsightMapper::refreshWorkspace()
{
    Project& p = project();
    _workspace->update(p);
}

void InsightMapper::del_tree_item(QTreeWidgetItem* node)
{
    if (node->childCount() > 0) {
        for (int i = 0; i < node->childCount(); i++) {
            del_tree_item(node->child(i));
        }
    } else {
        delete node;
    }
}

void InsightMapper::setWelcomeWidgetEnabled(bool val)
{
    QList<QMdiSubWindow*> subWidgetList = ui.mdiArea->subWindowList();
    for (int i = 0; i < subWidgetList.size(); ++i) {
        if (subWidgetList[i]->objectName() == "Welcome") {
            SubWidget* widget = (SubWidget*)(subWidgetList[i]->widget());
            val ? widget->enable() : widget->disable();
            return;
        }
    }
}

void InsightMapper::updateActionState()
{
    bool enabled = (doc().isOpen());

    for (QAction* act : _projectOpenActions) {
        act->setEnabled(enabled);
    }
}

void InsightMapper::onRegister()
{
    About aboutDialog(this);
    aboutDialog.exec();
}

void InsightMapper::onWorkspaceDoubleClicked(
    QTreeWidgetItem* item, int column)
{
    if (column != 0)
        return;
    int type = item->data(0, Qt::UserRole).toInt();
    if (type == Workspace::eCamera) {
        int camId = item->data(0, Qt::UserRole + 1).toInt();
        addImageAttributesWidget(camId);
    } else if (type == Workspace::eAT) {
        // printf("AT clicked\n");
        QString name = item->text(0);
        QString taskId = item->data(0, Qt::UserRole + 1).toString();
        qDebug() << name << taskId;
        addATConfigWidget(taskId, name);
    } else if (type == Workspace::eModel) {
        QString name = item->text(0);
        QString taskId = item->data(0, Qt::UserRole + 1).toString();
        qDebug() << name << taskId;
        addModelConfigWidget(taskId, name);
    } else if (type == Workspace::eCameras) {
        // printf("cameras clicked\n");
    } else if (type == Workspace::eATs) {
        // printf("ATs clicked\n");
    }
}

// void ProgressHelper::setInfinite(bool va)
// {
// 	if (!va){
// 		emit setRange(0, 100);
// 	}
// 	else{
// 		emit setRange(0,0);
// 	}
// }
//
// void ProgressHelper::starting()
// {
// 	emit start();
// }
//
// void ProgressHelper::finished()
// {
// 	emit finish();
// }

}//name space insight
