#ifndef CAMERAEDIT_H
#define CAMERAEDIT_H

#include <QWidget>
#include "ui_cameraedit.h"

#include "SubWidget.h"
#include <cstdint>

#include "Common/db_types.h"
#include "Common/Project.h"

namespace insight{
class CameraEdit : public SubWidget
{
	Q_OBJECT

signals:
	void currentCameraChanged(uint32_t came_id);
public:
	CameraEdit(QWidget *parent = 0);
	~CameraEdit();

    void setShowTask(const std::string &taskId,bool origin);
	void init();
	void refreshDatas();
    void enable(){}
    void disable(){}

	int currentCamera();
	public slots:
	void addCamera();
	void removeCamera();
	void showCurCamera();
	void camerraItemEdit(QTableWidgetItem* item);
	void getCameraFromExif();
	void getCameraFromImageWH();
    void setEditable(bool editable);
    void updateEditable();
private:
    ATTask *getTask();
private:
	Ui::CameraEdit ui;
    std::string _taskId;
    bool _showTask;
    bool _taskOrigin;
    bool _refreshing;
    bool _cameraEditable;
};

}//name space insight
#endif // CAMERAEDIT_H
