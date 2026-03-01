#include "cameraedit.h"

#include "Document.h"

#include <QTableWidget>

namespace insight{
CameraEdit::CameraEdit(QWidget *parent)
    : SubWidget(parent)
{
    ui.setupUi(this);
    ui.pushButton_exif->setVisible(false);
    ui.pushButton_reset->setVisible(false);
    _showTask = false;
    _taskOrigin = true;
    _refreshing = false;
    _cameraEditable = true;
}

CameraEdit::~CameraEdit()
{

}
void CameraEdit::setShowTask(const std::string &taskId,bool origin)
{
    _taskId = taskId;
    _showTask = true;
    _taskOrigin = origin;
    ui.pushButton->setVisible(false);
    ui.pushButton_2->setVisible(false);

}
void CameraEdit::init()
{
    refreshDatas();
}

ATTask *CameraEdit::getTask()
{
    for(auto &t : project().atTaskList)
    {
        if(t.id == _taskId){
            return &t;
        }
    }
    return nullptr;
}
void CameraEdit::refreshDatas()
{
    _refreshing = true;
    QListWidget *camera_list = ui.listWidget_camera;
    camera_list->clear();

    if(!_showTask)
    {
        std::map<uint32_t, DBCamera> &camlist = project().cameraList.Camera_list();
        for (auto itr = camlist.begin(); itr != camlist.end(); ++itr){
            QString camName = toqs(itr->second.camera_name);
            QListWidgetItem *item = new QListWidgetItem(camName, camera_list);
            item->setData(Qt::UserRole, itr->first);
        }
        if (!camlist.empty()){
            camera_list->setCurrentRow(0);
        }
        showCurCamera();
    }else{
        ATTask *task = getTask();
        if(task == nullptr){
            LOG(ERROR)<<"Logic error,task is NULL";
            return ;
        }
        std::map<uint32_t, DBCamera> camlist;
        if(_taskOrigin)
        {
            camlist = task->originCameraList.Camera_list();
        }else{
            camlist = task->refinedCameraList.Camera_list();
        }
        for (auto itr = camlist.begin(); itr != camlist.end(); ++itr){
            QString camName = toqs(itr->second.camera_name);
            QListWidgetItem *item = new QListWidgetItem(camName, camera_list);
            item->setData(Qt::UserRole, itr->first);
        }
        if (!camlist.empty()){
            camera_list->setCurrentRow(0);
        }
        showCurCamera();
    }
    updateEditable();
    _refreshing = false;
}

int CameraEdit::currentCamera()
{
    QListWidget *camera_list = ui.listWidget_camera;
    QListWidgetItem *listItem = camera_list->currentItem();
    if (nullptr == listItem) return -1;
    int cam_id = listItem->data(Qt::UserRole).toInt();
    return cam_id;
}

void CameraEdit::addCamera()
{
    uint32_t camid = project().resource.cameraSeed.generate();
    std::map<uint32_t, DBCamera> &camlist = project().cameraList.Camera_list();
    camlist[camid].id = camid;
    QString camName = tr("New Camera");
    camlist[camid].camera_name = tos(camName);

    QListWidget *camera_list = ui.listWidget_camera;
    QListWidgetItem *newItem = new QListWidgetItem(camName, camera_list);
    newItem->setData(Qt::UserRole, camid);
}

void CameraEdit::removeCamera()
{
    QListWidget *camera_list = ui.listWidget_camera;
    QListWidgetItem *item = camera_list->currentItem();
    if (item != NULL){
        int camId = item->data(Qt::UserRole).toInt();
        std::map<uint32_t, DBCamera> &camlist = project().cameraList.Camera_list();
        camlist.erase(camId);
        std::map<uint32_t, DBImage> &imglist = project().imageListGen.imageList.Image_list();
        std::vector<KeyType> imageDelete;
        for (auto itr = imglist.begin(); itr != imglist.end(); ++itr){
            if (itr->second.camera_id == camId){
                imageDelete.push_back(itr->first);
                //itr->second.camera_id = UndefinedKey;
            }
        }
        for(KeyType imgId: imageDelete){
            imglist.erase(imgId);
        }
        delete item;
        if (camera_list->count() > 0){
            camera_list->setCurrentRow(0);
            QListWidgetItem *item = camera_list->currentItem();
            int camId = item->data(Qt::UserRole).toInt();
            emit currentCameraChanged(camId);
        }
        else{
            emit currentCameraChanged(UndefinedKey);
        }
    }
}

void CameraEdit::showCurCamera()
{
    QListWidget *camera_list = ui.listWidget_camera;
    QTableWidget *init = ui.tableWidget_initial;
    QListWidgetItem *item = camera_list->currentItem();
    if (item != nullptr){
        ui.tableWidget_initial->setEnabled(true);
        uint32_t camId = item->data(Qt::UserRole).toInt();
        DBCamera cam;
        if(!_showTask){
            const auto &camlist = project().cameraList.Camera_list();
            cam = camlist.at(camId);
        }else{
            ATTask *t = getTask();
            if(t == nullptr) return;
            if(_taskOrigin){
                const auto &camlist = t->originCameraList.Camera_list();
                cam = camlist.at(camId);
            }else{
                const auto &camlist = t->refinedCameraList.Camera_list();
                cam = camlist.at(camId);
            }

        }
        QTableWidgetItem *item = new QTableWidgetItem(toqs(cam.camera_name));
        init->setItem(0, 1, item);

        item = new QTableWidgetItem(QString::number(cam.w));
        init->setItem(1, 1, item);

        item = new QTableWidgetItem(QString::number(cam.h));
        init->setItem(2, 1, item);

        item = new QTableWidgetItem(QString::number(cam.ppx, 'f', 6));
        init->setItem(3, 1, item);
        item = new QTableWidgetItem(QString::number(cam.ppy, 'f', 6));
        init->setItem(4, 1, item);

        item = new QTableWidgetItem(QString::number(cam.focalmm, 'f', 6));
        init->setItem(5, 1, item);
        item = new QTableWidgetItem(QString::number(cam.focalpx, 'f', 6));
        init->setItem(6, 1, item);

        item = new QTableWidgetItem(QString::number(cam.sensor_size_x, 'f', 6));
        init->setItem(7, 1, item);
        item = new QTableWidgetItem(QString::number(cam.sensor_size_y, 'f', 6));
        init->setItem(8, 1, item);
        item = new QTableWidgetItem(QString::number(cam.k1, 'f', 10));
        init->setItem(9, 1, item);
        item = new QTableWidgetItem(QString::number(cam.k2, 'f', 10));
        init->setItem(10, 1, item);
        item = new QTableWidgetItem(QString::number(cam.k3, 'f', 10));
        init->setItem(11, 1, item);
        item = new QTableWidgetItem(QString::number(cam.p1, 'f', 10));
        init->setItem(12, 1, item);
        item = new QTableWidgetItem(QString::number(cam.p2, 'f', 10));
        init->setItem(13, 1, item);
        item = new QTableWidgetItem(QString::number(cam.b1, 'f', 10));
        init->setItem(14, 1, item);
        item = new QTableWidgetItem(QString::number(cam.b2, 'f', 10));
        init->setItem(15, 1, item);
        updateEditable();
        emit currentCameraChanged(cam.id);
    }
    else{
        ui.tableWidget_initial->setEnabled(false);
    }
}

void CameraEdit::camerraItemEdit(QTableWidgetItem* item)
{
    if(_refreshing) return;
    if(!_cameraEditable) return;
    int col = item->column();
    if (col != 1) return;

    QListWidget *camera_list = ui.listWidget_camera;
    QListWidgetItem *listItem = camera_list->currentItem();
    if (listItem == NULL) return;
    int cam_id = listItem->data(Qt::UserRole).toInt();
    QTableWidget *initWidget = ui.tableWidget_initial;

    int row = item->row();
    DBCamera cam;
    if(!_showTask){
        std::map<uint32_t, DBCamera> &camlist = project().cameraList.Camera_list();
        cam = camlist.at(cam_id);
    }
    else{
        ATTask *t = getTask();
        if(_taskOrigin)
        {
            std::map<uint32_t, DBCamera> &camlist = t->originCameraList.Camera_list();
            cam = camlist.at(cam_id);
        }else{
            std::map<uint32_t, DBCamera> &camlist = t->refinedCameraList.Camera_list();
            cam = camlist.at(cam_id);
        }
    }
    if (row == 0){
        QString camName = item->text();
        listItem->setText(camName);
        cam.camera_name = tos(camName);
    }
    else if (row == 1){
        int val = item->text().toInt();
        cam.w = val;
    }
    else if (row == 2){
        int val = item->text().toInt();
        cam.h = val;
    }
    else{
        double val = item->text().toDouble();
        switch (row){
        case 3:
            cam.ppx = val;
            break;
        case 4:
            cam.ppy = val;
            break;
        case 5:
            cam.focalmm = val;
            break;
        case 6:
            cam.focalpx = val;
            break;
        case 7:
            cam.sensor_size_x = val;
            break;
        case 8:
            cam.sensor_size_y = val;
            break;
        case 9:
            cam.k1 = val;
            break;
        case 10:
            cam.k2 = val;
            break;
        case 11:
            cam.k3 = val;
            break;
        case 12:
            cam.p1 = val;
            break;
        case 13:
            cam.p2 = val;
            break;
        case 14:
            cam.b1 = val;
            break;
        case 15:
            cam.b2 = val;
            break;
        }
    }

    //save
    if(!_showTask){
        std::map<uint32_t, DBCamera> &camlist = project().cameraList.Camera_list();
        camlist.at(cam_id) = cam;
    }
    else{
        ATTask *t = getTask();
        if(_taskOrigin)
        {
            std::map<uint32_t, DBCamera> &camlist = t->originCameraList.Camera_list();
            camlist.at(cam_id) = cam;
        }else{
            std::map<uint32_t, DBCamera> &camlist = t->refinedCameraList.Camera_list();
            camlist.at(cam_id) = cam;
        }
    }
}

void CameraEdit::getCameraFromExif()
{
    project().generateCameraByExif();
    init();
}

void CameraEdit::getCameraFromImageWH()
{
    project().getCameraFromImageWH();
    refreshDatas();
}

void CameraEdit::updateEditable()
{
    QTableWidget *initTable = ui.tableWidget_initial;
    for (int i = 0; i < initTable->rowCount(); ++i)
    {
        QTableWidgetItem *item = initTable->item(i, 0);
        item->setFlags(item->flags() & (~Qt::ItemIsEditable));
		if (initTable->columnCount() == 2)
		{
			item = initTable->item(i, 1);
			if (item != nullptr)
			{
				if (_cameraEditable) {
					item->setFlags(item->flags() | (Qt::ItemIsEditable));
				}
				else {
					item->setFlags(item->flags() & (~Qt::ItemIsEditable));
				}
			}
	
		}
    }
}
void CameraEdit::setEditable(bool editable)
{
    _cameraEditable = editable;
}
}//name space insight
