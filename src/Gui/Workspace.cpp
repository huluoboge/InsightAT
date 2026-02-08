#include "Workspace.h"
#include "Document.h"
#include "MainWindowFrame.h"
#include "cameraedit.h"
#include "ui_Workspace.h"
#include <QStringList>

static void del_tree_item(QTreeWidgetItem* node)
{
    if (node->childCount() > 0) {
        for (int i = 0; i < node->childCount(); i++) {
            del_tree_item(node->child(i));
        }
    } else {
        delete node;
    }
}

namespace insight{
Workspace::Workspace(QWidget* parent)
    : QTreeWidget(parent)
    , ui(new Ui::Workspace)
{
    ui->setupUi(this);

    _cameras = new QTreeWidgetItem(this, QStringList(QString("Cameras")));
    _ATs = new QTreeWidgetItem(this, QStringList(QString("AT")));
    _Models = new QTreeWidgetItem(this, QStringList(QString("Model")));

    _cameras->setData(0, Qt::UserRole, Workspace::eCameras);
    _ATs->setData(0, Qt::UserRole, Workspace::eATs);
    _Models->setData(0, Qt::UserRole, Workspace::eModels);

    _cameraMenu = new QMenu(this);
    QAction* cameraManagement = new QAction(tr("Set camera"), this);
    _cameraMenu->addAction(cameraManagement);

    _ATMenu = new QMenu(this);
    QAction* newAT = new QAction(tr("New AT"), this);
    _ATMenu->addAction(newAT);

    _ModelMenu = new QMenu(this);
    QAction* newModel = new QAction(tr("New Model"), this);
    _ModelMenu->addAction(newModel);

    connect(cameraManagement, SIGNAL(triggered()), this, SLOT(onSetCamera()));
    connect(newAT, SIGNAL(triggered()), this, SLOT(onNewAT()));
    connect(newModel, SIGNAL(triggered()), this, SLOT(onNewModel()));
}

Workspace::~Workspace()
{
    delete ui;
}

void Workspace::contextMenuEvent(QContextMenuEvent* event)
{
    if (doc().isOpen()) {
        //_menu->exec(mapToGlobal(event->pos()));
        QTreeWidgetItem* pItem = this->itemAt(event->pos());
        if (pItem != nullptr) {
            if (_cameras == pItem) {
                _cameraMenu->exec(event->globalPos());
            } else if (_ATs == pItem) {
                _ATMenu->exec(event->globalPos());
            } else if (pItem->parent() == _ATs) {
                _currentItem = pItem;
                _ModelMenu->exec(event->globalPos());
            }
        }
    }
}
void Workspace::onSetCamera()
{
    CameraEdit* camEdit = new CameraEdit(this);
    showSubWidget(theWindow->widget(), camEdit);
    theWindow->saveProject();
    theWindow->refreshProject();
}

static QString makeCommand(const QString& path, const QString& program)
{
#ifdef WIN32
    const QString ext = ".exe";
#else
    const QString ext = "";
#endif
    return path + "/" + program + ext;
}

void Workspace::onNewAT()
{
    std::string id = project().newAT();
    int idx = project().findATTask(id);

    ATTask& task = project().atTaskList.at(idx);
    try {
        QString program = makeCommand(qApp->applicationDirPath(), "main_check_project");
        QStringList args;
        args << "-i" << toqs(task.taskDir);
        qDebug() << args;
        int result = QProcess::execute(program, args);
        if (result != EXIT_SUCCESS) {
            LOG(ERROR) << "AT check failed";
        } else {
            task.readRefined();
            task.readOriginMapCoord();
        }
    } catch (std::exception e) {
        LOG(ERROR) << "Exception information:" << e.what();
        LOG(ERROR) << "AT check failed";
    } catch (...) {
        LOG(ERROR) << "AT check failed";
    }

    theWindow->saveProject();
    theWindow->refreshProject();
}

void Workspace::onNewModel()
{
    std::string atId = tos(_currentItem->data(0, Qt::UserRole + 1).toString());

    int atIdx = project().findATTask(atId);
    ATTask& task = project().atTaskList.at(atIdx);
    std::string modelId = project().newModel(atId);
    theWindow->saveProject();
    theWindow->refreshProject();
}

void Workspace::update(Project& project)
{
    QList<QTreeWidgetItem*> items = _cameras->takeChildren();
    for (auto item : items) {
        del_tree_item(item);
    }
    items = _ATs->takeChildren();
    for (auto item : items) {
        del_tree_item(item);
    }

    items = _Models->takeChildren();
    for (auto item : items) {
        del_tree_item(item);
    }
    items.clear();

    const auto& camList = project.cameraList.Camera_list();

    const std::map<uint32_t, DBImage>& imageList = project.imageListGen.imageList.Image_list();
    std::map<uint32_t, int> groupImageCount;
    for (auto itr = imageList.begin(); itr != imageList.end(); ++itr) {
        ++groupImageCount[itr->second.camera_id];
    }

    for (auto itr = camList.begin(); itr != camList.end(); ++itr) {
        QTreeWidgetItem* item = new QTreeWidgetItem(_cameras);
        int n = groupImageCount[itr->first];
        QString name = QString("%1(%2)").arg(toqs(itr->second.camera_name)).arg(n);
        item->setText(0, name);
        item->setData(0, Qt::UserRole, Workspace::eCamera);
        item->setData(0, Qt::UserRole + 1, itr->first);
    }

    ATTaskList& atList = project.atTaskList;
    for (size_t i = 0; i < atList.size(); ++i) {
        QTreeWidgetItem* item = new QTreeWidgetItem(_ATs);
        QString name = QString("%1").arg(toqs(atList[i].name));
        QString taskId = QString("%1").arg(toqs(atList[i].id));
        if (name.isEmpty()) {
            name = taskId;
        }
        item->setText(0, name);
        item->setData(0, Qt::UserRole, Workspace::eAT);
        item->setData(0, Qt::UserRole + 1, taskId);
    }
    ModelTaskList& modelList = project.modelTaskList;

    for (int j = 0; j < modelList.size(); ++j) {
        QTreeWidgetItem* modelItem = new QTreeWidgetItem(_Models);
        QString name = QString("%1").arg(toqs(modelList[j].name));
        QString taskId = QString("%1").arg(toqs(modelList[j].id));
        if (name.isEmpty()) {
            name = taskId;
        }
        modelItem->setText(0, name);
        modelItem->setData(0, Qt::UserRole, Workspace::eModel);
        modelItem->setData(0, Qt::UserRole + 1, taskId);
    }
}

}//name space insight
