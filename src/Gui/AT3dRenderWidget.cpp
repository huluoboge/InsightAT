#include "AT3dRenderWidget.h"
// #include "MultiView/Pose.h"
#include "Utils.h"
#include "render/render_simple_objects.h"
#include "render/render_tracks.h"
#include "ui_AT3dRenderWidget.h"
#include <QDebug>
#include <QHBoxLayout>

namespace insight{
AT3dRenderWidget::AT3dRenderWidget(QWidget* parent)
    : QWidget(parent)
    , ui(new Ui::AT3dRenderWidget)
{
    _setGridFunction = &gridFunction;
    // _center.fill(0);
    ui->setupUi(this);
    _renderWidget = ui->widget;
    render::RenderTracks* tracks = new render::RenderTracks;
    _renderWidget->dataRoot()->renderObjects().push_back(tracks);
    _tracks = tracks;
    _renderWidget->setPivotVisible(ui->pushButton_showBall->isChecked());
}

AT3dRenderWidget::~AT3dRenderWidget()
{
    printf("~AT3dRenderWidget\n");
    delete ui;
}

void AT3dRenderWidget::on_pushButton_cameraSmaller_clicked()
{
    _tracks->photoSmaller();
    _renderWidget->updateGL();
}

void AT3dRenderWidget::on_pushButton_cameraBigger_clicked()
{
    _tracks->photoLarger();
    _renderWidget->updateGL();
}

void AT3dRenderWidget::on_pushButton_showCamera_clicked(bool visible)
{
    _tracks->setPhotoVisible(visible);
    _renderWidget->updateGL();
}
void AT3dRenderWidget::on_pushButton_showVertex_clicked(bool visible)
{
    _tracks->setVertexVisible(visible);
    _renderWidget->updateGL();
}

void AT3dRenderWidget::on_pushButton_showBall_clicked(bool visible)
{
    _renderWidget->setPivotVisible(visible);
    _renderWidget->updateGL();
}

void AT3dRenderWidget::on_pushButton_vertexSmaller_clicked()
{
    _tracks->vertexSmaller();
    _renderWidget->updateGL();
}
void AT3dRenderWidget::on_pushButton_vertexBigger_clicked()
{
    _tracks->vertexLarge();
    _renderWidget->updateGL();
}

void AT3dRenderWidget::on_pushButton_home_clicked()
{
    _renderWidget->root()->identityAll();
    _renderWidget->updateGL();
}

void AT3dRenderWidget::refreshDatas(const ATTask& task)
{
    const auto& imgList = task.originImageListMapCoordGen.imageList.Image_list();
    const auto& camList = task.originCameraList.Camera_list();
    const auto& refineImgList = task.refinedImageListGen.imageList.Image_list();
    render::RenderTracks::Photos photos;
    size_t totalPts = 0;
    double meanX = 0;
    double meanY = 0;
    double meanZ = 0;
    std::vector<uint32_t> images;
    for (auto itr = imgList.begin(); itr != imgList.end(); ++itr) {
        const DBImage& img = itr->second;
        if (img.pose.centerValid()) {
            meanX += img.pose.x;
            meanY += img.pose.y;
            meanZ += img.pose.z;
            images.push_back(itr->first);
            ++totalPts;
        }
    }
    if (totalPts == 0) {
        for (auto itr = refineImgList.begin(); itr != refineImgList.end(); ++itr) {
            const DBImage& img = itr->second;
            if (img.pose.centerValid()) {
                meanX += img.pose.x;
                meanY += img.pose.y;
                meanZ += img.pose.z;
                images.push_back(itr->first);
                ++totalPts;
            }
        }
        if (totalPts == 0)
            return;
    }
    meanX /= totalPts;
    meanY /= totalPts;
    meanZ /= totalPts;

    for (size_t i = 0; i < images.size(); ++i) {
        render::RenderTracks::Photo p;

        auto itr = imgList.find(images[i]);
        auto refineItr = refineImgList.find(images[i]);

        if (itr != imgList.end()) {
            const DBImage& img = itr->second;
            const auto& cam = camList.at(img.camera_id);
            p.w = cam.w;
            p.h = cam.h;
            p.focal = cam.focalpx;
            if (p.focal == 0.f)
                p.focal = (p.w + p.h) * 0.5f;
            p.name = toqs(img.image_name);

            p.initPose.centerValid = img.pose.centerValid();
            p.initPose.rotationValid = img.pose.rotationValid();
            p.initPose.data[0] = img.pose.x - meanX;
            p.initPose.data[1] = img.pose.y - meanY;
            p.initPose.data[2] = img.pose.z - meanZ;
            p.initPose.data[3] = img.pose.omega;
            p.initPose.data[4] = img.pose.phi;
            p.initPose.data[5] = img.pose.kappa;
            p.initPose.openglMat.setIdentity();

            if (p.initPose.rotationValid) {
                // insight::Pose pose;
                // pose.dbPose.omega = img.pose.omega;
                // pose.dbPose.phi = img.pose.phi;
                // pose.dbPose.kappa = img.pose.kappa;
                // pose.dbPose.x = 0;
                // pose.dbPose.y = 0;
                // pose.dbPose.z = 0;
                // pose.updateRt();
                // p.initPose.openglMat = pose.toOpenGLModelView();
            }
        }

        p.refinedPose.openglMat.setIdentity();
        p.refinedPose.color = Vec3(1, 1, 0);

        if (refineItr == refineImgList.end()) {
            p.refinedPose.centerValid = false;
            p.refinedPose.rotationValid = false;
        } else {
            const DBImage& refineImg = refineItr->second;
            p.refinedPose.centerValid = refineImg.pose_valid;
            p.refinedPose.rotationValid = refineImg.pose_valid;
            p.refinedPose.data[0] = refineImg.pose.x - meanX;
            p.refinedPose.data[1] = refineImg.pose.y - meanY;
            p.refinedPose.data[2] = refineImg.pose.z - meanZ;
            p.refinedPose.data[3] = refineImg.pose.omega;
            p.refinedPose.data[4] = refineImg.pose.phi;
            p.refinedPose.data[5] = refineImg.pose.kappa;
            if (p.refinedPose.rotationValid) {
                // insight::Pose pose;
                // pose.dbPose = refineImg.pose;
                // pose.dbPose.x = 0;
                // pose.dbPose.y = 0;
                // pose.dbPose.z = 0;
                // pose.updateRt();
                // p.refinedPose.openglMat = pose.toOpenGLModelView();
            }
        }
        photos.push_back(p);
    }
    qDebug() << "Photos count=" << photos.size();
    _tracks->setPhotos(photos);

    auto& trackList = task.trackList.TrackList();
    qDebug() << "Track count=" << trackList.size();
    render::RenderTracks::Tracks tracks;
    for (auto itr = trackList.begin(); itr != trackList.end(); ++itr) {
        render::RenderTracks::Track t;
        t.x = itr->second.landmark.x - meanX;
        t.y = itr->second.landmark.y - meanY;
        t.z = itr->second.landmark.z - meanZ;
        t.color.x() = itr->second.landmark.r / 255.0;
        t.color.y() = itr->second.landmark.g / 255.0;
        t.color.z() = itr->second.landmark.b / 255.0;
        t.trackId = itr->first;
        for (size_t i = 0; i < itr->second.views.size(); ++i) {
            render::RenderTracks::Observe obv;
            obv.featX = itr->second.views[i].u;
            obv.featY = itr->second.views[i].v;
            obv.photoId = itr->second.views[i].image_id;
            t.obs.push_back(obv);
        }
        tracks.push_back(t);
    }
    _tracks->setTracks(tracks);

    auto& gcpList = task.gcpList.GCP_List();
    render::RenderTracks::Tracks gcps;
    for (auto itr = gcpList.begin(); itr != gcpList.end(); ++itr) {
        render::RenderTracks::Track t;
        t.x = itr->second.landmark.x - meanX;
        t.y = itr->second.landmark.y - meanY;
        t.z = itr->second.landmark.z - meanZ;
        t.color.x() = 255.0;
        t.color.y() = 0.0;
        t.color.z() = 255.0;
        t.trackId = itr->first;
        gcps.push_back(t);
    }
    _tracks->setGCPs(gcps);
    render::RenderTracks::RenderOptions opt;
    _tracks->setRenderOptions(opt);
    update();
}

void AT3dRenderWidget::refreshDatas(const ModelTask& task)
{
    const ImageListGenerator& gen = task.atResultGen;
    const auto& imgList = gen.imageList.Image_list();
    const auto& camList = task.atResultCameraList.Camera_list();

    auto& trackList = task.atResultTrackList.TrackList();

    render::RenderTracks::Tracks tracks;
    for (auto itr = trackList.begin(); itr != trackList.end(); ++itr) {
        render::RenderTracks::Track t;
        t.x = itr->second.landmark.x - task.grid.centerx;
        t.y = itr->second.landmark.y - task.grid.centery;
        t.z = itr->second.landmark.z - task.grid.centerz;
        t.color.x() = itr->second.landmark.r / 255.0;
        t.color.y() = itr->second.landmark.g / 255.0;
        t.color.z() = itr->second.landmark.b / 255.0;
        t.trackId = itr->first;
        for (size_t i = 0; i < itr->second.views.size(); ++i) {
            render::RenderTracks::Observe obv;
            obv.featX = itr->second.views[i].u;
            obv.featY = itr->second.views[i].v;
            obv.photoId = itr->second.views[i].image_id;
            t.obs.push_back(obv);
        }
        tracks.push_back(t);
    }
    _tracks->setTracks(tracks);
    render::RenderTracks::Grid grid;
    grid.minx = task.grid.minx;
    grid.miny = task.grid.miny;
    grid.minz = task.grid.minz;
    grid.maxx = task.grid.maxx;
    grid.maxy = task.grid.maxy;
    grid.maxz = task.grid.maxz;
    grid.xcount = task.grid.xcount;
    grid.ycount = task.grid.ycount;
    grid.zcount = task.grid.zcount;
    grid.generateDatas();
    _tracks->setGrid(grid);
    _tracks->setGridVisible(true);
    render::RenderTracks::Photos photos;
    std::vector<uint32_t> images;
    for (auto itr = imgList.begin(); itr != imgList.end(); ++itr) {
        const DBImage& img = itr->second;
        if (img.pose.centerValid()) {
            images.push_back(itr->first);
        }
    }

    for (size_t i = 0; i < images.size(); ++i) {
        render::RenderTracks::Photo p;
        auto itr = imgList.find(images[i]);
        if (itr != imgList.end()) {
            const DBImage& img = itr->second;
            const auto& cam = camList.at(img.camera_id);
            p.w = cam.w;
            p.h = cam.h;
            p.focal = cam.focalpx;
            if (p.focal == 0.f)
                p.focal = (p.w + p.h) * 0.5f;
            p.name = toqs(img.image_name);

            p.initPose.centerValid = false;
            p.initPose.rotationValid = false;
            p.refinedPose.openglMat.setIdentity();
            p.refinedPose.color = Vec3(1, 1, 0);
            p.refinedPose.centerValid = true;
            p.refinedPose.rotationValid = true;
            p.refinedPose.data[0] = img.pose.x - task.grid.centerx;
            p.refinedPose.data[1] = img.pose.y - task.grid.centery;
            p.refinedPose.data[2] = img.pose.z - task.grid.centerz;
            p.refinedPose.data[3] = img.pose.omega;
            p.refinedPose.data[4] = img.pose.phi;
            p.refinedPose.data[5] = img.pose.kappa;
            // insight::Pose pose;
            // pose.dbPose = img.pose;
            // pose.dbPose.x = 0;
            // pose.dbPose.y = 0;
            // pose.dbPose.z = 0;
            // pose.updateRt();
            // p.refinedPose.openglMat = pose.toOpenGLModelView();
        } else {
            p.refinedPose.centerValid = false;
            p.refinedPose.rotationValid = false;
        }
        photos.push_back(p);
    }
    qDebug() << "Photos count=" << photos.size();
    _tracks->setPhotos(photos);
    render::RenderTracks::RenderOptions opt;
    _tracks->setRenderOptions(opt);
    update();
}

void AT3dRenderWidget::onSetGridCount(int xcount, int ycount, int zcount)
{
    _setGridFunction(xcount, ycount, zcount);
    auto grid = _tracks->getGrid();
    grid.xcount = xcount;
    grid.ycount = ycount;
    grid.zcount = zcount;
    grid.generateDatas();
    _tracks->setGrid(grid);
    _renderWidget->updateGL();
    update();
}

}//name space insight
