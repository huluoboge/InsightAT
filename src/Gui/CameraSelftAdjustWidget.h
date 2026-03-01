#ifndef CAMERASELFTADJUSTWIDGET_H
#define CAMERASELFTADJUSTWIDGET_H

#include <QWidget>
#include "ui_CameraSelftAdjustWidget.h"
#include "InsightATGlobal.h"
#include "Common/Project.h"

namespace insight{

class CameraSelftAdjustWidget : public QWidget
{
	Q_OBJECT

public:
	CameraSelftAdjustWidget(QWidget *parent = 0);
	~CameraSelftAdjustWidget();

	public slots:

	void readCameras();

	void showCurCamera();

	void saveCameras();

    void setTaskId(const std::string &task){_taskId = task;}


private:
    ATTask *getTask();
    std::string _taskId;
	int _update_flag = 0;
	Ui::CameraSelftAdjustWidget ui;
};

}//name space insight
#endif // CAMERASELFTADJUSTWIDGET_H
