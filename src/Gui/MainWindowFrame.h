#ifndef MAINWINDOWINTERFACE_H
#define MAINWINDOWINTERFACE_H


#include "InsightATGlobal.h"

#include <QString>
#include <QWidget>

namespace insight{

class MainWindowFrame
{
public:
    MainWindowFrame() = default;
    virtual ~MainWindowFrame() = default;

    virtual void openProject() = 0;
    virtual void newProject() = 0;

    virtual void saveProject() = 0;

    virtual void openProject(const QString &prj) = 0;

    virtual void closeProject() = 0;


    virtual void closeAllMdiWindows() = 0;

    virtual void showWelcomePage() = 0;
    
    virtual void refreshProject() = 0;

    virtual QWidget *widget() = 0;


};


}//name space insight

extern insight::MainWindowFrame *theWindow;
#endif // MAINWINDOWINTERFACE_H
