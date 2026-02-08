#include "CameraSelftAdjustWidget.h"
#include "Common/Project.h"
#include "Document.h"

using namespace insight;

namespace insight{
CameraSelftAdjustWidget::CameraSelftAdjustWidget(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);

}

CameraSelftAdjustWidget::~CameraSelftAdjustWidget()
{

}

void CameraSelftAdjustWidget::readCameras()
{

    ATTask *task = getTask();
    if(task == nullptr) return;

	QListWidget *camera_list = ui.listWidget_camera;
	camera_list->clear();

    std::map<uint32_t, DBCamera> &camlist = task->originCameraList.Camera_list();
	for (auto itr = camlist.begin(); itr != camlist.end(); ++itr){
		QString camName = toqs(itr->second.camera_name);
		QListWidgetItem *item = new QListWidgetItem(camName, camera_list);
		item->setData(Qt::UserRole, itr->first);
	}
	if (!camlist.empty()){
		camera_list->setCurrentRow(0);
		showCurCamera();
	}
}

void CameraSelftAdjustWidget::showCurCamera()
{
    ATTask *task = getTask();
    if(task == nullptr) return;


	QListWidget *camera_list = ui.listWidget_camera;
	QTableWidget *init = ui.tableWidget_initial;
	QListWidgetItem *item = camera_list->currentItem();
	if (item != NULL){
		uint32_t camId = item->data(Qt::UserRole).toInt();
		_update_flag = 1;
        std::map<uint32_t, DBCamera> &camlist =task->originCameraList.Camera_list();
		DBCamera &cam = camlist.at(camId);
		Qt::CheckState state;
		state = cam.adjustFlag.f ? Qt::Checked : Qt::Unchecked;
		init->item(0, 1)->setCheckState(state);
		state = cam.adjustFlag.ppxy ? Qt::Checked : Qt::Unchecked;
		init->item(1, 1)->setCheckState(state);
		state = cam.adjustFlag.k1 ? Qt::Checked : Qt::Unchecked;
		init->item(2, 1)->setCheckState(state);
		state = cam.adjustFlag.p1 ? Qt::Checked : Qt::Unchecked;
		init->item(3, 1)->setCheckState(state);
		state = cam.adjustFlag.b1 ? Qt::Checked : Qt::Unchecked;
		init->item(4, 1)->setCheckState(state);
		_update_flag = 0;
	}
}

void CameraSelftAdjustWidget::saveCameras()
{
	if (_update_flag == 1) return;
	QListWidget *camera_list = ui.listWidget_camera;
	QTableWidget *init = ui.tableWidget_initial;
	QListWidgetItem *item = camera_list->currentItem();
    ATTask *task = getTask();
    if(task == nullptr) return;

    if (item != nullptr){
		uint32_t camId = item->data(Qt::UserRole).toInt();
        std::map<uint32_t, DBCamera> &camlist = task->originCameraList.Camera_list();
		DBCamera &cam = camlist.at(camId);

		Qt::CheckState state;
		bool adjust = false;
		state = init->item(0, 1)->checkState();
		adjust = (state == Qt::Checked);
		cam.adjustFlag.f = adjust;
		state = init->item(1, 1)->checkState();
		adjust = (state == Qt::Checked);
		cam.adjustFlag.ppxy = adjust;

		state = init->item(2, 1)->checkState();
		adjust = (state == Qt::Checked);
		cam.adjustFlag.k1 = adjust;
		cam.adjustFlag.k2 = adjust;
		cam.adjustFlag.k3 = adjust;

		state = init->item(3, 1)->checkState();
		adjust = (state == Qt::Checked);
		cam.adjustFlag.p1 = adjust;
		cam.adjustFlag.p2 = adjust;

		state = init->item(4, 1)->checkState();
		adjust = (state == Qt::Checked);
		cam.adjustFlag.b1 = adjust;
		cam.adjustFlag.b2 = adjust;
	}
}

ATTask *CameraSelftAdjustWidget::getTask()
{
    for(auto &t : project().atTaskList)
    {
        if(t.id == _taskId){
            return &t;
        }
    }
    return nullptr;
}


}//name space insight
