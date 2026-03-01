#include "ImageAttributes.h"
#include <QFileDialog>
#include <QMessageBox>

#include "AddPhotosDialog.h"
#include "Common/string_utils.h"
#include "Document.h"
#include "GPSPointsWizardDialog.h"
#include "ImportGPSDialog.h"
#include "Settings.h"
#include <set>
#include <string>

#include "CheckImageWidget.h"
#include "MainWindowFrame.h"
#include "ProgressDialog.h"
#include "cameraedit.h"
namespace insight{
ImageAttributes::ImageAttributes(QWidget* parent)
    : SubWidget(parent)
{
    ui.setupUi(this);
    _showTask = false;
    _taskOrigin = true;
    _refreshing = false;
}

ImageAttributes::~ImageAttributes()
{
}

void ImageAttributes::init()
{
    refreshDatas();
}

void ImageAttributes::setShowTask(const std::string& taskId, bool origin)
{
    _taskId = taskId;
    _showTask = true;
    _taskOrigin = origin;
    ui.pushButton->setVisible(false);
    ui.pushButton_2->setVisible(false);
    ui.pushButton_3->setVisible(false);
    ui.pushButton_4->setVisible(false);
    ui.pushButton_5->setVisible(false);
    ui.pushButton_setCamera->setVisible(false);
}

void ImageAttributes::setEditCameraEnabled(bool enable)
{
    ui.pushButton_setCamera->setVisible(enable);
}

void ImageAttributes::importImages()
{
    QStringList fileList = QFileDialog::getOpenFileNames(this, tr("add images..."), settings().recentPath(),
        "Image files(*.jpg *.JPG *.tif *.TIF *.png *.PNG *.bmp *.BMP);;"
        "Jpg files(*.jpg *.JPG);;"
        "Tif files(*.tif *.TIF);;"
        "Png files(*.png *.PNG);;"
        "Bmp files(*.bmp *.BMP);;"
        "All files(*.*)");
    if (!fileList.isEmpty()) {
        QFileInfo info(fileList[0]);
        settings().setRecentPath(info.absolutePath());

        std::set<std::string> imgs;
        for (int i = 0; i < fileList.size(); ++i) {
            imgs.insert(tos(fileList[i]));
        }
        QList<int> ids;
        QStringList groupNames;
        for (auto itr = project().cameraList.Camera_list().begin();
             itr != project().cameraList.Camera_list().end(); ++itr) {
            ids.push_back(itr->first);
            groupNames << toqs(itr->second.camera_name);
        }
        KeyType camera_key;
        if (_currentCameraId == UndefinedKey) {
            AddPhotosDialog optionDialog(this);
            optionDialog.initGroups(groupNames, ids);
            optionDialog.setNextCameraId(project().resource.cameraSeed.seed);
            if (optionDialog.exec() == QDialog::Accepted) {
                AddPhotosDialog::AddCameraType camType = optionDialog.addCameraType();
                if (camType == AddPhotosDialog::eNew) // new camera
                {
                    camera_key = project().resource.cameraSeed.generate();
                    project().cameraList.Camera_list()[camera_key].id = camera_key;
                    project().cameraList.Camera_list()[camera_key].camera_name = tos(optionDialog.newCameraName());
                } else if (camType == AddPhotosDialog::eSelect) //
                {
                    camera_key = optionDialog.selectCameraId();
                } else if (camType == AddPhotosDialog::eByExif) {
                    camera_key = UndefinedKey;
                }
            } else {
                return;
            }
        } else {
            camera_key = _currentCameraId;
        }
        int n = 0;
        // CHECK(camera_key != UndefinedKey);
        ProgressDialog* dlg = new ProgressDialog(this);
        std::thread addImageThread([imgs, camera_key, &n, dlg]() {
            std::vector<int> imported = project().imageListGen.importImages(
                imgs, camera_key, &project().resource);
            n = imported.size();
            if (n > 0) {
                if (camera_key == UndefinedKey) {
                    project().generateCameraByExif(imported);
                } else {
                    project().getCameraFromImageWH(camera_key);
                    project().generateCameraByExif(camera_key);
                }
                doc().setModify(true);
            }
            // prog.Finish();
            dlg->exit();
        });
        dlg->exec();
        addImageThread.join();
        refreshDatas();
        if (n > 0) {
            project().saveProject();
            changedProject = true;
            emit projectChanged();
            refresh();
            theWindow->refreshProject();
        }
    }
}

void ImageAttributes::importPose()
{
    QString file = QFileDialog::getOpenFileName(this, tr("Import pose data..."), settings().recentPath(),
        tr("Coord Text File(*.txt);;All files(*.*)"));
    if (file.isEmpty())
        return;
    settings().setRecentPath(QFileInfo(file).absolutePath());

    ImportGPSDialog wizard(this);
    wizard.setFile(file);
    wizard.checkEnablePreview();
    if (QDialog::Accepted == wizard.exec()) {
        ImportGPSDialog::Vec_Pose poses = wizard.Points();
        bool hasIMU = false;
        int angleUnitMode = 0;
        int coordSysMode = 0;
        int eulerSysMode = 0;
        if (wizard.hasOmegaPhiKappa()) {
            hasIMU = true;
            angleUnitMode = wizard.angleUnit();
            coordSysMode = wizard.coordinateSystem();
            eulerSysMode = wizard.eulerAngleSystem();
        }

        // std::map<uint32_t, DBPose> &poselist = project().imageListGen.poseList.Pose_list();
        std::map<uint32_t, DBImage>& imageList = project().imageListGen.imageList.Image_list();
        int nImported = 0;
        if (wizard.isImportByName()) {
            std::map<std::string, KeyType> imageNameList;
            for (auto itr = imageList.begin(); itr != imageList.end(); ++itr) {
                std::string name = tos(QFileInfo(toqs(itr->second.image_name)).baseName().toLower());
                DBImage& img = itr->second;
                if (_currentCameraId != UndefinedKey && img.camera_id != _currentCameraId) {
                    continue; // only set with camera id
                }
                if (imageNameList.find(name) != imageNameList.end()) {
                    LOG(INFO) << "Find images with same name within same camera!";
                } else {
                    imageNameList[name] = itr->first;
                }
            }

            for (auto itr = poses.begin(); itr != poses.end(); ++itr) {

                std::string poseName = tos(QFileInfo(toqs(itr->name)).baseName().toLower());

                auto findItr = imageNameList.find(poseName);
                if (findItr != imageNameList.end()) {
                    DBImage& img = imageList.at(findItr->second);
                    DBPose& dbpose = img.pose;
                    auto& pose = *itr;
                    dbpose.x = pose.x;
                    dbpose.y = pose.y;
                    dbpose.z = pose.z;
                    if (hasIMU) {
                        dbpose.omega = pose.omega;
                        dbpose.phi = pose.phi;
                        dbpose.kappa = pose.kappa;
                        dbpose.angleUnit = angleUnitMode;
                        dbpose.coordinate = coordSysMode;
                        dbpose.eulerAngle = eulerSysMode;
                    }
                    // img.init_pose_center_valid = true;
                    // img.init_pose_rotation_valid = hasIMU;
                    ++nImported;
                }
            }
        } else {
            // auto inPose = imageList.begin();
            std::vector<KeyType> imageIdList;
            for (auto itr = imageList.begin(); itr != imageList.end(); ++itr) {
                std::string name = tos(QFileInfo(toqs(itr->second.image_name)).baseName().toLower());
                DBImage& img = itr->second;
                if (_currentCameraId != UndefinedKey && img.camera_id != _currentCameraId) {
                    continue; // only set with camera id
                }
                imageIdList.push_back(itr->first);
            }

            auto itImageId = imageIdList.begin();
            for (auto itr = poses.begin(); itr != poses.end() && itImageId != imageIdList.end(); ++itr, ++itImageId) {
                auto& pose = *itr;
                DBPose& dbpose = imageList.at(*itImageId).pose;
                dbpose.x = pose.x;
                dbpose.y = pose.y;
                dbpose.z = pose.z;
                if (hasIMU) {
                    dbpose.omega = pose.omega;
                    dbpose.phi = pose.phi;
                    dbpose.kappa = pose.kappa;
                    dbpose.angleUnit = angleUnitMode;
                    dbpose.coordinate = coordSysMode;
                    dbpose.eulerAngle = eulerSysMode;
                }
                // inPose->second.init_pose_center_valid = true;
                //  inPose->second.init_pose_rotation_valid = hasIMU;
                ++nImported;
            }
        }
        changedProject = true;
        LOG(INFO) << "Add " << nImported << " GPSs";
        // project().updateENUCoord();
        refresh();
        theWindow->refreshProject();
    } else {
        LOG(INFO) << "Import poses canceled";
    }
}

void ImageAttributes::bindCamera(uint32_t camera_id)
{
    _currentCameraId = camera_id;
    refreshDatas();
}

void ImageAttributes::setCamera()
{
    QList<QTableWidgetItem*> items = ui.tableWidget_attribute->selectedItems();
    if (items.empty()) {
        QMessageBox::information(this, tr("Error"), tr("No image selected"));
        return;
    }
    std::set<int> imgIds;
    for (int i = 0; i < items.size(); ++i) {
        imgIds.insert(ui.tableWidget_attribute->item(items[i]->row(), 0)->text().toInt());
    }

    QDialog dlg(this);
    CameraEdit* edit = new CameraEdit;
    dlg.setWindowTitle(tr("Select camera"));
    edit->init();
    QVBoxLayout* layout = new QVBoxLayout(&dlg);
    layout->addWidget(edit);
    QDialogButtonBox* bbox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(bbox, SIGNAL(accepted()), &dlg, SLOT(accept()));
    connect(bbox, SIGNAL(rejected()), &dlg, SLOT(reject()));
    layout->addWidget(bbox);
    if (dlg.exec() == QDialog::Accepted) {
        int came_id = edit->currentCamera();
        if (came_id == -1) {
            QMessageBox::information(this, tr("Error"), tr("No camera selected"));
        } else {
            for (auto it = imgIds.begin(); it != imgIds.end(); ++it) {
                DBImage& dbimage = project().imageListGen.imageList.Image_list().at(*it);
                dbimage.camera_id = came_id;
            }
            project().saveProject();
            refresh();
        }
    }
}

void ImageAttributes::setPath()
{
    QList<QTableWidgetItem*> items = ui.tableWidget_attribute->selectedItems();
    if (items.empty()) {
        QMessageBox::information(this, tr("Error"), tr("No image selected"));
        return;
    }
    std::set<int> imgIds;
    for (int i = 0; i < items.size(); ++i) {
        imgIds.insert(ui.tableWidget_attribute->item(items[i]->row(), 0)->text().toInt());
    }
    QString path = QFileDialog::getExistingDirectory(this, tr("Select image path"), settings().recentPath());
    if (!path.isEmpty()) {
        for (auto itr = imgIds.begin(); itr != imgIds.end(); ++itr) {
            DBImage& img = project().imageListGen.imageList.Image_list().at(*itr);
            img.image_full_path = tos(path) + "/" + img.image_name;
        }
        refreshDatas();
    }
}

void ImageAttributes::check()
{
    std::vector<insight::ImageConsistency> vecConsis;
    project().checkConsistency(vecConsis);
    if (!_checkDlg) {
        _checkDlg = new QDialog(this);
        QHBoxLayout* layout = new QHBoxLayout(_checkDlg);
        CheckImageWidget* widget = new CheckImageWidget(_checkDlg);
        widget->setObjectName("CHECK");
        layout->addWidget(widget);
        layout->setSpacing(1);
    }
    CheckImageWidget* widget = (CheckImageWidget*)(_checkDlg->findChild<CheckImageWidget*>("CHECK"));
    widget->setConsistency(vecConsis);
    widget->refreshDatas();

    _checkDlg->resize(_checkDlg->sizeHint());
    _checkDlg->show();
    _checkDlg->activateWindow();
    _checkDlg->raise();
}

ATTask* ImageAttributes::getTask()
{
    for (auto& t : project().atTaskList) {
        if (t.id == _taskId) {
            return &t;
        }
    }
    return nullptr;
}

void ImageAttributes::refresh()
{
    if (_refreshing)
        return;
    QTableWidget* table = ui.tableWidget_attribute;
    table->clearContents();
    std::map<uint32_t, DBImage> imagelist;
    if (!_showTask) {
        imagelist = project().imageListGen.imageList.Image_list();
    } else {
        ATTask* t = getTask();
        if (_taskOrigin) {
            imagelist = t->originImageListGen.imageList.Image_list();
        } else {
            imagelist = t->refinedImageListGen.imageList.Image_list();
        }
    }
    int r = 0;
    int totalCount = 0;
    for (auto itr = imagelist.begin(); itr != imagelist.end(); ++itr) {
        DBImage& img = itr->second;
        if (_currentCameraId != UndefinedKey && img.camera_id != _currentCameraId) {
            continue; // only show image with camera id
        }
        ++totalCount;
    }
    table->setRowCount(totalCount);
    // int columnCount = table->columnCount();

    for (auto itr = imagelist.begin(); itr != imagelist.end(); ++itr) {
        DBImage& img = itr->second;
        if (_currentCameraId != UndefinedKey && img.camera_id != _currentCameraId) {
            continue; // only show image with camera id
        }

        QTableWidgetItem* item = new QTableWidgetItem;
        item->setData(Qt::DisplayRole, itr->first);
        // item->setText(QString("%1").arg(itr->first));
        table->setItem(r, 0, item);
        item = new QTableWidgetItem;
        if (img.camera_id != UndefinedKey) {
            item->setText(toqs(project().cameraList.Camera_list().at(img.camera_id).camera_name));
        } else {
            item->setText(tr("Undefined"));
        }
        table->setItem(r, 1, item);

        item = new QTableWidgetItem;
        item->setText(toqs(img.image_name));
        table->setItem(r, 2, item);

        item = new QTableWidgetItem;

        const DBPose& pose = img.pose;
        item->setText(QString::number(pose.x, 'f', 6));
        table->setItem(r, 3, item);

        item = new QTableWidgetItem;
        item->setText(QString::number(pose.y, 'f', 6));
        table->setItem(r, 4, item);

        item = new QTableWidgetItem;
        item->setText(QString::number(pose.z, 'f', 6));
        table->setItem(r, 5, item);

        item = new QTableWidgetItem;
        item->setText(QString::number(pose.omega, 'f', 10));
        table->setItem(r, 6, item);

        item = new QTableWidgetItem;
        item->setText(QString::number(pose.phi, 'f', 10));
        table->setItem(r, 7, item);

        item = new QTableWidgetItem;
        item->setText(QString::number(pose.kappa, 'f', 10));
        table->setItem(r, 8, item);

        item = new QTableWidgetItem;

        QFileInfo info(toqs(img.image_full_path));
        if (!info.exists()) {
            item->setText(QString("image not exist!:") + toqs(img.image_full_path));
            // item->setTextColor(QColor(255, 0, 0));
            item->setForeground(QColor(255, 0, 0));
            item->setToolTip(QString("image not exist!:") + toqs(img.image_full_path));
        } else {
            item->setText(toqs(img.image_full_path));
            item->setToolTip(toqs(img.image_full_path));
        }

        table->setItem(r, 9, item);
        ++r;
    }

    table->resizeColumnsToContents();
    // table->resizeRowsToContents();
}

void ImageAttributes::removeImages()
{
    if (!doc().isOpen())
        return;
    QList<QTableWidgetItem*> items = ui.tableWidget_attribute->selectedItems();
    if (items.empty()) {
        QMessageBox::information(this, tr("Error"), tr("No image selected"));
        return;
    }
    std::set<KeyType> imgIds;
    for (int i = 0; i < items.size(); ++i) {
        imgIds.insert(ui.tableWidget_attribute->item(items[i]->row(), 0)->text().toInt());
    }
    project().imageListGen.removeImages(imgIds);
    refreshDatas();
}

}//name space insight
