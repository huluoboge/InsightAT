#ifndef WORKSPACE_H
#define WORKSPACE_H

#include "InsightATGlobal.h"
#include <QContextMenuEvent>
#include <QMenu>
#include <QTreeWidget>
#include <QWidget>

#include "Common/Project.h"

namespace Ui {
class Workspace;
}

class QTreeWidgetItem;

namespace insight{

class Workspace : public QTreeWidget {
    Q_OBJECT

public:
    enum WorkspaceItemType {
        eCameras,
        eCamera,
        eATs,
        eAT,
        eModels,
        eModel,
    };

    explicit Workspace(QWidget* parent = nullptr);
    ~Workspace();

    void update(Project &project);

    void contextMenuEvent(QContextMenuEvent* event);
public slots:
    void onSetCamera();
    void onNewAT();
    void onNewModel();

private:
    Ui::Workspace* ui;
    QMenu* _cameraMenu;
    QMenu* _ATMenu;
    QMenu* _ModelMenu;
    QTreeWidgetItem* _cameras;
    QTreeWidgetItem* _ATs;
    QTreeWidgetItem* _Models;

    QTreeWidgetItem* _currentItem = nullptr;
    
};

}//name space insight
#endif // WORKSPACE_H
