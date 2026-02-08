#ifndef AIMAGEATTRIBUTES_H
#define AIMAGEATTRIBUTES_H

#include <QWidget>
#include "ui_ImageAttributes.h"
#include <cstdlib>
#include <cstdint>
#include "Common/db_types.h"
#include "Common/Project.h"
#include "SubWidget.h"

class QDialog;

namespace insight{

class ImageAttributes : public SubWidget
{
    Q_OBJECT

signals:
    void projectChanged();
public:
    ImageAttributes(QWidget *parent = 0);
    ~ImageAttributes();

    bool changedProject = false;
    void init();
    void refreshDatas(){ refresh(); }

    void setShowTask(const std::string &taskId,bool origin);
    void enable(){}
    void disable(){}

    void setEditCameraEnabled(bool enable);
public slots:
    void importImages();
    void importPose();
    void bindCamera(uint32_t cameraId);
    uint32_t bindedCamera() const {return _currentCameraId;}
    void setCamera();
    void setPath();
    void check();
    void refresh();
    void removeImages();

private:
    ATTask *getTask();
    uint32_t _currentCameraId = insight::UndefinedKey;
    Ui::ImageAttributes ui;
    QDialog *_checkDlg = nullptr;
    std::string _taskId;
    bool _showTask;
    bool _taskOrigin;
    bool _refreshing;
};

}//name space insight
#endif // AIMAGEATTRIBUTES_H
