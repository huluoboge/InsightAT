#include "Project.h"

#include "Coordinates.h"
#include "ImageIO/gdal_utils.h"
#include "data_sheet.h"
#include "gps.h"
#include <cereal/archives/json.hpp>
#include <fstream>
#include <glog/logging.h>
#include <iostream>
#include <stlplus3/filesystemSimplified/file_system.hpp>
#include <string>

#include "ogr_spatialref.h"
#ifdef WIN32
#define NOMINMAX
#include <Windows.h>
#endif
namespace insight{

Project::Project()
{
}

bool Project::createProject(const ProjectInfomation& info, const std::string& path)
{
    infomation = info;
    projectFile = path;
    resource.reset();
    completeDirs();
    return saveProject();
}

bool readATList(const std::string& atListFile,
    std::vector<ATTask>& taskList)
{
    std::ifstream ifs(atListFile);
    if (!ifs) {
        LOG(ERROR) << "Can't read AT list file: " << atListFile;
        return false;
    }

    int version;
    std::string versionStr;
    ifs >> versionStr;
    // TODO(Jones) check the return result
    sscanf(versionStr.c_str(), "version=%d", &version);
    int nAT = 0;
    ifs >> nAT;
    for (int i = 0; i < nAT; ++i) {
        std::string id;
        std::string name;
        ifs >> id >> name;
        ATTask task;
        task.id = id;
        task.name = name;
        taskList.push_back(task);
    }

    return true;
}

bool writeATList(const std::string& atListFile,
    const std::vector<ATTask>& taskList)
{
    std::ofstream ofs(atListFile);
    if (!ofs) {
        LOG(ERROR) << "Can't write AT list file :" << atListFile;
        return false;
    }
    int version = INSIGHT_TASK_VERSION;
    ofs << "version=" << version << std::endl;
    int nAT = int(taskList.size());
    ofs << nAT << std::endl;
    for (int i = 0; i < nAT; ++i) {
        ofs << taskList[i].id << " " << taskList[i].name << std::endl;
    }
    return true;
}

bool writeModelList(const std::string& modelListFile,
    const ModelTaskList& taskList)
{
    std::ofstream ofs(modelListFile);
    if (!ofs) {
        LOG(ERROR) << "Can't write Model list file :" << modelListFile;
        return false;
    }
    int version = INSIGHT_TASK_MODEL_VERSION;
    ofs << "version=" << version << std::endl;
    int n = int(taskList.size());
    ofs << n << std::endl;
    for (int i = 0; i < n; ++i) {
        ofs << taskList[i].id << " " << taskList[i].name << " " << taskList[i].atId << std::endl;
    }
    return true;
}

bool readModelList(const std::string& modelListFile,
    ModelTaskList& taskList)
{
    std::ifstream ifs(modelListFile);
    if (!ifs) {
        LOG(ERROR) << "Can't read AT list file: " << modelListFile;
        return false;
    }

    int version;
    std::string versionStr;
    ifs >> versionStr;
    // TODO(Jones) check the return result
    sscanf(versionStr.c_str(), "version=%d", &version);
    int n = 0;
    ifs >> n;
    for (int i = 0; i < n; ++i) {
        std::string id;
        std::string name;
        std::string atid;
        ifs >> id >> name >> atid;
        ModelTask task;
        task.id = id;
        task.name = name;
        task.atId = atid;
        taskList.push_back(task);
    }

    return true;
}

bool Project::openProject(const std::string& projectFile)
{
    try {
        imageListGen.clear();

        this->projectFile = projectFile;

        std::ifstream ifs((projectFile));
        if (!ifs.is_open()) {
            LOG(ERROR) << "Can't open project file :" << projectFile;
            return false;
        }
        cereal::JSONInputArchive ar(ifs);
        ar(cereal::make_nvp("InsightProject", *this));
        completeDirs();

        if (!imageListGen.imageList.readFromASCIIFile(projectDataDir + "/image_list.txt")) {
            LOG(ERROR) << "Can't open image list file :" << projectDataDir + "/image_list.txt";
            return false;
        }

        DBPoseList poseList;
        if (!poseList.readFromASCIIFile(projectDataDir + "/pose_list.txt")) {
            LOG(ERROR) << "Can't open pose list file :" << projectDataDir + "/pose_list.txt";
            return false;
        }
        CHECK(poseList.Pose_list().size() == imageListGen.imageList.Image_list().size());
        auto& imgList = imageListGen.imageList.Image_list();
        auto& pList = poseList.Pose_list();

        for (auto itr = imgList.begin();
             itr != imgList.end(); ++itr) {
            CHECK(pList.find(itr->first) != pList.end());
            itr->second.pose = pList.at(itr->first);
        }
        gcpList.readFromAsciiFile(projectDataDir + "/gcp_list.txt");
        cameraList.readFromJson(projectDataDir + "/camera_list.txt");
        std::string atListFile = projectDataDir + "/at_task_list.txt";
        std::string modelListFile = projectDataDir + "/model_task_list.txt";
        atTaskList.clear();
        readATList(atListFile, atTaskList);
        for (size_t i = 0; i < atTaskList.size(); ++i) {
            ATTask& t = atTaskList[i];
            t.completeDirs(projectDataDir);
            t.readDatas();
        }
        modelTaskList.clear();
        readModelList(modelListFile, modelTaskList);
        for (size_t i = 0; i < modelTaskList.size(); ++i) {
            ModelTask& t = modelTaskList[i];
            t.completeDirs(projectDataDir);
            t.readDatas();
        }
        // updateENUCoord();
        return true;
    } catch (std::runtime_error err) {
        LOG(ERROR) << err.what();
        std::string info = std::string("Can't open file ") + projectFile;
        LOG(ERROR) << info;
        return false;
    }
}

bool Project::getProjectInfomation(const std::string& projectFile)
{
    this->projectFile = projectFile;

    std::ifstream ifs((projectFile));
    if (!ifs.is_open()) {
        LOG(ERROR) << "Can't open project file: " << projectFile;
        return false;
    }

    cereal::JSONInputArchive ar(ifs);
    ar(cereal::make_nvp("InsightProject", *this));
    return true;
}

bool Project::saveProject(SaveFlag flag)
{
    std::ofstream ofs((projectFile));
    if (!ofs.is_open()) {
        LOG(ERROR) << "Can't save project file:" << projectFile;
        return false;
    }
    cereal::JSONOutputArchive ar(ofs);
    ar(cereal::make_nvp("InsightProject", *this));

    if (flag & eSaveImageAndCameras) {
        imageListGen.imageList.saveToASCIIFile((projectDataDir + "/image_list.txt"));
        cameraList.saveToJson((projectDataDir + "/camera_list.txt"));
    }

    if (flag & eSavePose) {
        DBPoseList poseList;
        auto& imgList = imageListGen.imageList.Image_list();
        auto& pList = poseList.Pose_list();

        for (auto itr = imgList.begin();
             itr != imgList.end(); ++itr) {
            pList[itr->first] = itr->second.pose;
        }
        poseList.saveToASCIIFile(projectDataDir + "/pose_list.txt");
    }

    if (flag & eSaveGCP) {
        gcpList.saveToAsciiFile((projectDataDir + ("/gcp_list.txt")));
    }

    std::string atListFile = projectDataDir + "/at_task_list.txt";
    std::string modelListFile = projectDataDir + "/model_task_list.txt";

    writeATList(atListFile, atTaskList);

    for (size_t i = 0; i < atTaskList.size(); ++i) {
        atTaskList[i].writeDatas();
    }

    writeModelList(modelListFile, modelTaskList);
    for (size_t i = 0; i < modelTaskList.size(); ++i) {
        modelTaskList[i].writeDatas();
    }
    return true;
}

void Project::generateCameraByExif()
{
    std::map<uint32_t, std::vector<int>> camera_images;
    std::map<uint32_t, DBImage>& imagelist = imageListGen.imageList.Image_list();
    // std::map<uint32_t, uint32_t> mapCameraImage;
    for (auto itr = imagelist.begin(); itr != imagelist.end(); ++itr) {
        DBImage& img = itr->second;
        camera_images[img.camera_id].push_back(itr->first);
    }
    for (auto itr = camera_images.begin(); itr != camera_images.end(); ++itr) {
        uint32_t camera_id = itr->first;

        auto& imgs = itr->second;
        if (imgs.size() > 0) {
            DBImage& img = imagelist.at(imgs[0]);
            img.readExif();
            if (img.exif_valid) {
                DBCamera cam;
                cam.camera_name = cameraList.Camera_list().at(camera_id).camera_name;
                cam.id = camera_id;
                if (img.getCameraByExif(cam)) {
                    cameraList.Camera_list().at(camera_id) = cam;
                }
            }
        }
    }
}

void Project::generateCameraByExif(uint32_t camera_id)
{
    std::vector<int> camera_images;
    std::map<uint32_t, DBImage>& imagelist = imageListGen.imageList.Image_list();
    // std::map<uint32_t, uint32_t> mapCameraImage;
    for (auto itr = imagelist.begin(); itr != imagelist.end(); ++itr) {
        DBImage& img = itr->second;
        if (img.camera_id == camera_id) {
            camera_images.push_back(itr->first);
        }
    }
    if (camera_images.size() > 0) {
        DBImage& img = imagelist.at(camera_images[0]);
        img.readExif();
        if (img.exif_valid) {
            DBCamera cam;
            cam.id = camera_id;
            cam.camera_name = cameraList.Camera_list().at(camera_id).camera_name;
            if (img.getCameraByExif(cam)) {
                cameraList.Camera_list().at(camera_id) = cam;
            }
        }
    }
}

KeyType Project::findCameraByCameraHashCode(const size_t& hashCode)
{
    for (auto camItr = cameraList.Camera_list().begin(); camItr != cameraList.Camera_list().end();
         ++camItr) {
        const DBCamera& cam = camItr->second;
        if (cam.exif_hash_code == hashCode) {
            return cam.id;
        }
    }
    return UndefinedKey;
}

void Project::generateCameraByExif(const std::vector<int>& imageIds)
{
    std::map<uint32_t, DBImage>& imagelist = imageListGen.imageList.Image_list();

    for (size_t i = 0; i < imageIds.size(); ++i) {
        printf(".");
        int id = imageIds[i];
        auto itr = imagelist.find(id);
        if (itr == imagelist.end())
            continue;
        DBImage& img = itr->second;
        img.readExif();
        if (img.exif_valid) {
            size_t hash_code = img.cameraHashCode();
            KeyType key = findCameraByCameraHashCode(hash_code);
            if (key != UndefinedKey) {
                img.camera_id = key;
            } else {
                // exif valid and no camera,generate one
                KeyType cam_id = resource.cameraSeed.generate();
                DBCamera camera;
                camera.exif_hash_code = hash_code;
                img.getCameraByExif(camera);
                camera.id = cam_id;
                cameraList.Camera_list()[cam_id] = camera;
                img.camera_id = camera.id;
            }
        } else {
            // generate by wh
            DBCamera camera;
            img.getCameraByWH(camera);
            camera.generateHashCode();
            KeyType key = findCameraByCameraHashCode(camera.exif_hash_code);
            if (key != UndefinedKey) {
                img.camera_id = key;
            } else {
                // no camera,generate one
                KeyType cam_id = resource.cameraSeed.generate();
                camera.id = cam_id;
                cameraList.Camera_list()[cam_id] = camera;
                img.camera_id = camera.id;
            }
        }
    }
}

void Project::completeCameraByExif(const std::vector<int>& imageIds, KeyType camId)
{
    DBCamera& camera = cameraList.Camera_list().at(camId);

    std::map<uint32_t, DBImage>& imagelist = imageListGen.imageList.Image_list();

    bool setCamera = false;
    for (size_t i = 0; i < imageIds.size(); ++i) {
        printf(".");
        int id = imageIds[i];
        auto itr = imagelist.find(id);
        if (itr == imagelist.end())
            continue;
        DBImage& img = itr->second;
        if (img.exif_valid && !setCamera) {
            size_t hash_code = img.cameraHashCode();
            camera.exif_hash_code = hash_code;
            img.getCameraByExif(camera);
            setCamera = true;
        } else if (!setCamera) {
            setCamera = true;
            // generate by wh
            img.getCameraByWH(camera);
            camera.generateHashCode();
        }
        img.camera_id = camera.id;
    }
}

void Project::getCameraFromImageWH()
{
    std::map<uint32_t, std::vector<int>> camera_images;
    std::map<uint32_t, DBImage>& imagelist = imageListGen.imageList.Image_list();
    for (auto itr = imagelist.begin(); itr != imagelist.end(); ++itr) {
        DBImage& img = itr->second;
        camera_images[img.camera_id].push_back(itr->first);
    }
    GdalUtils::InitGDAL();
    for (auto itr = camera_images.begin(); itr != camera_images.end(); ++itr) {
        uint32_t camera_id = itr->first;

        auto& imgs = itr->second;
        if (imgs.size() > 0) {
            DBImage& img = imagelist.at(imgs[0]);
            int w = 0, h = 0;
            GdalUtils::GetWidthHeightPixel(img.image_full_path.c_str(), w, h);
            DBCamera& dbcam = cameraList.Camera_list().at(camera_id);
            dbcam.w = w;
            dbcam.h = h;
            dbcam.ppx = (w - 1.0) / 2.0;
            dbcam.ppy = (h - 1.0) / 2.0;
            dbcam.focalpx = 0;
        }
    }
}

void Project::getCameraFromImageWH(uint32_t camera_id)
{
    std::vector<int> camera_images;
    std::map<uint32_t, DBImage>& imagelist = imageListGen.imageList.Image_list();
    for (auto itr = imagelist.begin(); itr != imagelist.end(); ++itr) {
        DBImage& img = itr->second;
        if (img.camera_id == camera_id) {
            camera_images.push_back(itr->first);
        }
    }
    GdalUtils::InitGDAL();
    if (camera_images.size() > 0) {
        DBImage& img = imagelist.at(camera_images[0]);
        int w = 0, h = 0;
        GdalUtils::GetWidthHeightPixel(img.image_full_path.c_str(), w, h);
        DBCamera& dbcam = cameraList.Camera_list().at(camera_id);
        dbcam.w = w;
        dbcam.h = h;
        dbcam.ppx = (w - 1.0) / 2.0;
        dbcam.ppy = (h - 1.0) / 2.0;
        dbcam.focalpx = 0;
    }
}

void Project::getCameraFromImageWH(const std::vector<int>& imgIds)
{
    GdalUtils::InitGDAL();
    std::map<uint32_t, DBImage>& imagelist = imageListGen.imageList.Image_list();

    for (int imgId : imgIds) {
        DBImage& img = imagelist.at(imgId);
        int w = 0, h = 0;
        GdalUtils::GetWidthHeightPixel(img.image_full_path.c_str(), w, h);
        DBCamera& dbcam = cameraList.Camera_list().at(img.camera_id);
        dbcam.w = w;
        dbcam.h = h;
        dbcam.ppx = (w - 1.0) / 2.0;
        dbcam.ppy = (h - 1.0) / 2.0;
        dbcam.focalpx = 0;
    }
}

void Project::checkConsistency(std::vector<ImageConsistency>& result)
{
    GdalUtils::InitGDAL();
    std::map<uint32_t, DBImage>& imgList = imageListGen.imageList.Image_list();
    LOG(INFO) << "Checking pho";
    for (auto itr = imgList.begin(); itr != imgList.end(); ++itr) {
        ImageConsistency consis;
        consis.imageId = itr->first;
        DBImage& img = itr->second;
        consis.cameraId = img.camera_id;
        if (consis.cameraId != UndefinedKey) {
            auto camItr = cameraList.Camera_list().find(consis.cameraId);
            if (camItr != cameraList.Camera_list().end()) {
                DBCamera cam = camItr->second;
                consis.cameraW = cam.w;
                consis.cameraH = cam.h;
            }
        }
        if (stlplus::file_exists(img.image_full_path)) {
            consis.imageExist = true;
            int w = 0, h = 0;
            if (GdalUtils::GetWidthHeightPixel(img.image_full_path.c_str(), w, h)) {
                consis.imageW = w;
                consis.imageH = h;
                consis.imageCanRead = true;
            } else {
                consis.imageCanRead = false;
            }
        }
        result.push_back(consis);
    }
}

#if 0
bool Project::updateENUCoord()
{
    std::map<uint32_t, DBImage> &images = imageListGen.imageList.Image_list();
    std::map<uint32_t, DBGCP> &gcps = gcpList.GCP_List();

    if (infomation.coordinate.localSystem){
        for (auto itr = images.begin(); itr != images.end(); ++itr)
        {
            DBPose &dbpose = itr->second.pose;
            dbpose.enuX = dbpose.x;
            dbpose.enuY = dbpose.y;
            dbpose.enuZ = dbpose.z;
        }
        for (auto itr = gcps.begin(); itr != gcps.end(); ++itr)
        {
            DBGCP &dbgcp = itr->second;
            dbgcp.enuX = dbgcp.landmark.x;
            dbgcp.enuY = dbgcp.landmark.y;
            dbgcp.enuZ = dbgcp.landmark.z;
        }
        return true;
    }

    //non-local coordinate
    Coordinate targetCoord;
    targetCoord.WKT = infomation.coordinate.wkt;
    targetCoord.EPSGName = infomation.coordinate.epsg;
    targetCoord.CoordinateName = infomation.coordinate.name;


    OGRSpatialReference targetSr;
    bool	ok = Coordinate::coordToSR(targetCoord, targetSr);
    if (!ok) {
        LOG(INFO) << "Can't parse map coordinate" ;
        return false;
    }

    bool local = false;

    Coordinate gpsCoord;
    gpsCoord.WKT = infomation.gpsCoordinate.wkt;
    gpsCoord.EPSGName = infomation.gpsCoordinate.epsg;
    gpsCoord.CoordinateName = infomation.gpsCoordinate.name;
    OGRSpatialReference gpsSR;
    bool	gps_ok = Coordinate::coordToSR(gpsCoord, gpsSR);

    if (gps_ok)
    {
        CoordTransform transform;
        transform.setFrom(&gpsSR);
        transform.setTo(&targetSr);
        //transform pose
        std::vector<double >xs, ys, zs;
        for (auto itr = poses.begin(); itr != poses.end(); ++itr)
        {
            DBPose &dbpose = itr->second;
            xs.push_back(dbpose.x);
            ys.push_back(dbpose.y);
            zs.push_back(dbpose.z);
        }
        int n = xs.size();
        if (n > 0) {
            transform.beginTransfrom();
            transform.transform(n, xs.data(), ys.data(), zs.data());
            transform.endTransfrom();
            int ipose = 0;
            std::vector<Vec3> ell;//ע����γ�Ⱦ��ȸ̵߳�˳��
            for (auto itr = poses.begin(); itr != poses.end(); ++itr, ++ipose)
            {
                DBPose &dbpose = itr->second;
                dbpose.enuX = xs[ipose];
                dbpose.enuY = ys[ipose];
                dbpose.enuZ = zs[ipose];
            }
        }

    }
    else {
        LOG(ERROR) << "Can't transform EPSG/WKT to Spatial reference" ;
        return false;
    }

    Coordinate gcpCoord;
    gcpCoord.WKT = infomation.gcpCoordinate.wkt;
    gcpCoord.EPSGName = infomation.gcpCoordinate.epsg;
    gcpCoord.CoordinateName = infomation.gcpCoordinate.name;
    OGRSpatialReference gcpSR;

    bool gcp_ok = Coordinate::coordToSR(gcpCoord, gcpSR);


    if (gcp_ok){
        CoordTransform transform;
        transform.setFrom(&gcpSR);
        transform.setTo(&targetSr);
        //transform pose
        std::vector<double >xs, ys, zs;
        for (auto itr = gcps.begin(); itr != gcps.end(); ++itr)
        {
            xs.push_back(itr->second.landmark.x);
            ys.push_back(itr->second.landmark.y);
            zs.push_back(itr->second.landmark.z);
        }
        int n = xs.size();
        if (n > 0) {
            transform.beginTransfrom();
            transform.transform(n, xs.data(), ys.data(), zs.data());
            transform.endTransfrom();
            int igcp = 0;
            for (auto itr = gcps.begin(); itr != gcps.end(); ++itr,++igcp)
            {
                DBGCP &dbgcp = itr->second;
                dbgcp.enuX = xs[igcp];
                dbgcp.enuY = ys[igcp];
                dbgcp.enuZ = zs[igcp];
            }
        }
    }
    else{
        LOG(ERROR) << "Can't transform EPSG/WKT to Spatial reference" ;
        return false;
    }
    return true;
}

#endif
template <>
void Project::serialize(cereal::JSONInputArchive& ar, std::uint32_t const version)
{
    ar(CEREAL_NVP(infomation));
    ar(CEREAL_NVP(resource));
    if (version == 1) {
        ar(CEREAL_NVP(configData));
        _saveBinFlag |= eSaveTrackBin;
    }
}

template <>
void Project::serialize(cereal::JSONOutputArchive& ar, std::uint32_t const version)
{
    ar(CEREAL_NVP(infomation));
    ar(CEREAL_NVP(resource));
    if (version == 1) {
        ar(CEREAL_NVP(configData));
        _saveBinFlag |= eSaveTrackBin;
    }
}

void Project::completeDirs()
{
    projectDir = stlplus::folder_part(projectFile);
    std::string name = stlplus::basename_part(projectFile);
    projectDataDir = projectDir + "/" + name + ".datas";

    if (!stlplus::folder_exists(projectDataDir) && !stlplus::folder_create(projectDataDir)) {
        LOG(ERROR) << "Can't create project folder : " << projectDataDir;
    }
}

SystemConfig* global_system_config = nullptr;
SystemConfig& SystemConfig::instance()
{
    if (global_system_config == nullptr) {
        global_system_config = new SystemConfig;
    }
    return *global_system_config;
}
void SystemConfig::exit()
{
    if (global_system_config)
        delete global_system_config;
}

void SystemConfig::setExePath(const std::string& path)
{
    _exePath = path;
}

void SystemConfig::setConfigPath(const std::string& path)
{
    _configPath = path;
}

std::string SystemConfig::exePath() const
{
#ifdef WIN32
    char buffer[MAX_PATH];
    GetModuleFileNameA(NULL, buffer, MAX_PATH);
    std::string::size_type pos = std::string(buffer).find_last_of("\\/");
    return std::string(buffer).substr(0, pos);
#else
    return _exePath;
#endif
}

std::string SystemConfig::configPath() const
{
    return _configPath;
    // return stlplus::create_filespec(exePath(), "Config");
}

bool SystemConfig::readSensorDatabase()
{
    std::string sensorDB = stlplus::create_filespec(configPath(), "camera_sensor_database.txt");
    if (!stlplus::file_exists(sensorDB)) {
        LOG(ERROR) << "Can't read camera sensor DB" << sensorDB;
        return false;
    }
    return parseDatabase(sensorDB, sensorDatabase);
}

bool SystemConfig::readCoordinate()
{
    std::string geoCoord = stlplus::create_filespec(configPath(), "GEOGCS_Database.csv");
    std::string prjCoord = stlplus::create_filespec(configPath(), "PROJCS_Database.csv");
    ProjCoordinate.clear();
    GeoCoordinate.clear();
    if (!stlplus::file_exists(geoCoord)) {
        LOG(ERROR) << "Can't read project DB: " << geoCoord;
        return false;
    }
    if (!stlplus::file_exists(prjCoord)) {
        LOG(ERROR) << "Can't read project DB: " << prjCoord;
        return false;
    }
    if (parseCoordinates(ProjCoordinate, prjCoord) && parseCoordinates(GeoCoordinate, geoCoord)) {
        return true;
    } else {
        return false;
    }
}

bool CoordInfomation::isProject(bool* ok) const
{
    Coordinate coord;
    coord.EPSGName = epsg;
    coord.WKT = wkt;
    return coord.isProject(ok);
}

bool ProjectInfomation::read(std::ifstream& ifs, ProjectInfomation& projectInfo)
{
    if (!ifs.is_open()) {
        return false;
    }

    cereal::JSONInputArchive ar(ifs);
    ar(cereal::make_nvp("projectInfo", projectInfo));
    return true;
}

bool ProjectInfomation::write(std::ofstream& ofs, const ProjectInfomation& projectInfo)
{
    if (!ofs.is_open()) {
        return false;
    }

    cereal::JSONOutputArchive ar(ofs);
    ar(cereal::make_nvp("projectInfo", projectInfo));
    return true;
}

/////////////////////////////////////////////////////////////////////////

void ModelTask::completeDirs(const std::string& parentFolder)
{
    parentDir = parentFolder;
    taskDir = parentDir + "/" + id;
}

void ModelTask::createDirs()
{
    LOG(INFO) << "Create dir " << taskDir;
    if (!stlplus::folder_exists(taskDir) && !stlplus::folder_create(taskDir)) {
        LOG(ERROR) << "Can't create project folder : " << taskDir;
        return;
    }
}

bool ModelTask::updateGrid()
{
    auto& trackList = atResultTrackList.TrackList();
    {
        size_t totalPts = 0;
        double meanX = 0;
        double meanY = 0;
        double meanZ = 0;
        for (auto itr = trackList.begin(); itr != trackList.end(); ++itr) {
            meanX += itr->second.landmark.x;
            meanY += itr->second.landmark.y;
            meanZ += itr->second.landmark.z;
            ++totalPts;
        }
        meanX /= totalPts;
        meanY /= totalPts;
        meanZ /= totalPts;
        grid.centerx = meanX;
        grid.centery = meanY;
        grid.centerz = meanZ;
    }

    double minx, miny, minz, maxx, maxy, maxz;
    minx = miny = minz = 1e10;
    maxx = maxy = maxz = -1e10;
    for (auto itr = trackList.begin(); itr != trackList.end(); ++itr) {
        double x = itr->second.landmark.x - grid.centerx;
        double y = itr->second.landmark.y - grid.centery;
        double z = itr->second.landmark.z - grid.centerz;
        minx = std::min(x, minx);
        miny = std::min(y, miny);
        minz = std::min(z, minz);
        maxx = std::max(x, maxx);
        maxy = std::max(y, maxy);
        maxz = std::max(z, maxz);
    }
    grid.minx = minx;
    grid.miny = miny;
    grid.minz = minz;
    grid.maxx = maxx;
    grid.maxy = maxy;
    grid.maxz = maxz;
    grid.xcount = 1;
    grid.ycount = 1;
    grid.zcount = 1;
    return true;
}
bool ModelTask::writeDatas()
{
    std::string refinedImgFile = taskDir + "/image_list.txt";
    std::string refinedCamFile = taskDir + "/camera_list.txt";
    std::string refinedPoseFile = taskDir + "/pose_list.txt";
    std::string trackFile = taskDir + "/track_list.bin";

    bool ok = false;
    ok = atResultGen.imageList.saveToASCIIFile(refinedImgFile);
    if (!ok) {
        LOG(ERROR) << "Can't write file" << refinedImgFile;
        return false;
    }
    ok = atResultCameraList.saveToJson(refinedCamFile);
    if (!ok) {
        LOG(ERROR) << "Can't write file" << refinedCamFile;
        return false;
    }
    DBPoseList poseList;
    auto& imgList = atResultGen.imageList.Image_list();
    auto& pList = poseList.Pose_list();

    for (auto itr = imgList.begin();
         itr != imgList.end(); ++itr) {
        pList[itr->first] = itr->second.pose;
    }
    ok = poseList.saveToASCIIFile(refinedPoseFile);
    if (!ok) {
        LOG(ERROR) << "Can't write file" << refinedPoseFile;
        return false;
    }
    ok = atResultTrackList.saveToBinFile(trackFile);
    if (!ok) {
        LOG(ERROR) << "Can't write file" << trackFile;
        return false;
    }

    std::string gridFile = taskDir + "/grid.txt";

    std::ofstream ofs(gridFile);
    if (!ofs.is_open()) {
        LOG(ERROR) << "Can't save grid file:" << gridFile;
        return false;
    }
    cereal::JSONOutputArchive ar(ofs);
    ar(cereal::make_nvp("Grid", grid));
    return true;
}

bool ModelTask::readDatas()
{
    std::string refinedImgFile = taskDir + "/image_list.txt";
    std::string refinedCamFile = taskDir + "/camera_list.txt";
    std::string refinedPoseFile = taskDir + "/pose_list.txt";
    std::string trackFile = taskDir + "/track_list.bin";
    if (stlplus::file_exists(refinedImgFile) && stlplus::file_exists(refinedCamFile) && stlplus::file_exists(refinedPoseFile) && stlplus::file_exists(trackFile)) {
        bool ok = atResultGen.imageList.readFromASCIIFile(refinedImgFile);
        if (!ok) {
            return false;
        }
        ok = atResultCameraList.readFromJson(refinedCamFile);
        if (!ok) {
            return false;
        }
        DBPoseList refinedPoseList;
        ok = refinedPoseList.readFromASCIIFile(taskDir + "/pose_list.txt");
        if (!ok) {
            return false;
        }
        CHECK(refinedPoseList.Pose_list().size() == atResultGen.imageList.Image_list().size());
        atResultGen.mergePoseList(refinedPoseList);
        ok = atResultTrackList.readFromBinFile(trackFile);
        if (!ok) {
            return false;
        }

        std::string gridFile = taskDir + "/grid.txt";
        std::ifstream ifs(gridFile);
        if (!ifs.is_open()) {
            LOG(ERROR) << "Can't read grid file:" << gridFile;
            return false;
        }
        cereal::JSONInputArchive ar(ifs);
        ar(cereal::make_nvp("Grid", grid));
        return true;
    } else {
        return false;
    }
    return true;
}
//////////////////////////////////////////////////////////////////////

void ATTask::completeDirs(const std::string& parentFolder)
{
    parentDir = parentFolder;
    taskDir = parentDir + "/" + id;
    featsDir = taskDir + "/F";
    matchDir = taskDir + "/M";
    atDir = taskDir + "/A";
}

void ATTask::createDirs()
{
    if (!stlplus::folder_exists(taskDir) && !stlplus::folder_create(taskDir)) {
        LOG(ERROR) << "Can't create project folder : " << taskDir;
        return;
    }
    if (!stlplus::folder_exists(featsDir) && !stlplus::folder_create(featsDir)) {
        LOG(ERROR) << "Can't create project folder : " << featsDir;
    }
    if (!stlplus::folder_exists(matchDir) && !stlplus::folder_create(matchDir)) {
        LOG(ERROR) << "Can't create project folder : " << matchDir;
    }
    if (!stlplus::folder_exists(atDir) && !stlplus::folder_create(atDir)) {
        LOG(ERROR) << "Can't create project folder : " << atDir;
        return;
    }
}

bool ATTask::readOrigin()
{
    if (!originImageListGen.imageList.readFromASCIIFile(taskDir + "/image_list.txt")) {
        LOG(ERROR) << "Can't read " << taskDir + "/image_list.txt";
        return false;
    }
    if (!originCameraList.readFromJson(taskDir + "/camera_list.txt")) {
        LOG(ERROR) << "Can't read " << taskDir + "/camera_list.txt";
        return false;
    }
    DBPoseList originPoseList;
    if (!originPoseList.readFromASCIIFile(taskDir + "/pose_list.txt")) {
        LOG(ERROR) << "Can't read " << taskDir + "/pose_list.txt";
        return false;
    }
    CHECK(originPoseList.Pose_list().size() == originImageListGen.imageList.Image_list().size());
    originImageListGen.mergePoseList(originPoseList);

    if (!gcpList.readFromAsciiFile(taskDir + "/gcp_list.txt")) {
        LOG(ERROR) << "Can't read " << taskDir + "/gcp_list.txt";
        return false;
    }
    return true;
}

bool ATTask::refreshGCPList()
{
    if (!gcpList.readFromAsciiFile(taskDir + "/gcp_list.txt")) {
        LOG(ERROR) << "Can't read " << taskDir + "/gcp_list.txt";
        return false;
    }
    return true;
}

bool ATTask::writeOrigin()
{
    bool ok = originImageListGen.imageList.saveToASCIIFile(taskDir + "/image_list.txt");
    if (!ok) {
        LOG(ERROR) << "Can't write " << taskDir + "/image_list.txt";
        return false;
    }
    ok = originCameraList.saveToJson(taskDir + "/camera_list.txt");
    if (!ok) {
        LOG(ERROR) << "Can't write " << taskDir + "/camera_list.txt";
        return false;
    }
    DBPoseList poseList;

    auto& imgList = originImageListGen.imageList.Image_list();
    auto& pList = poseList.Pose_list();

    for (auto itr = imgList.begin();
         itr != imgList.end(); ++itr) {
        pList[itr->first] = itr->second.pose;
    }
    ok = poseList.saveToASCIIFile(taskDir + "/pose_list.txt");
    if (!ok) {
        LOG(ERROR) << "Can't write " << taskDir + "/pose_list.txt";
        return false;
    }
    ok = gcpList.saveToAsciiFile(taskDir + "/gcp_list.txt");
    if (!ok) {
        LOG(ERROR) << "Can't write " << taskDir + "/gcp_list.txt";
        return false;
    }
    return true;
}

bool ATTask::readRefined()
{
    std::string refinedImgFile = atDir + "/image_list.txt";
    std::string refinedCamFile = atDir + "/camera_list.txt";
    std::string refinedPoseFile = atDir + "/pose_list.txt";
    std::string trackFile = atDir + "/track_list.bin";
    if (stlplus::file_exists(refinedImgFile) && stlplus::file_exists(refinedCamFile) && stlplus::file_exists(refinedPoseFile) && stlplus::file_exists(trackFile)) {
        bool ok = refinedImageListGen.imageList.readFromASCIIFile(atDir + "/image_list.txt");
        if (!ok) {
            info.ATStatus = eNotCommit;
            return false;
        }
        ok = refinedCameraList.readFromJson(atDir + "/camera_list.txt");
        if (!ok) {
            info.ATStatus = eNotCommit;
            return false;
        }
        DBPoseList refinedPoseList;
        ok = refinedPoseList.readFromASCIIFile(atDir + "/pose_list.txt");
        if (!ok) {
            info.ATStatus = eNotCommit;
            return false;
        }
        CHECK(refinedPoseList.Pose_list().size() == refinedImageListGen.imageList.Image_list().size());
        refinedImageListGen.mergePoseList(refinedPoseList);
        ok = trackList.readFromBinFile(trackFile);
        if (!ok) {
            info.ATStatus = eNotCommit;
            LOG(INFO) << "NO track result";
            return false;
        }
    } else {
        info.ATStatus = eNotCommit;
    }
    return true;
}

bool ATTask::readOriginMapCoord()
{
    if (!originImageListMapCoordGen.imageList.readFromASCIIFile(taskDir + "/image_list.txt")) {
        LOG(ERROR) << "Can't read " << taskDir + "/image_list.txt";
        return false;
    }
    DBPoseList originMapPoseList;
    if (!originMapPoseList.readFromASCIIFile(taskDir + "/pose_map_coord.txt")) {
        LOG(ERROR) << "Can't read " << taskDir + "/pose_list.txt";
        return false;
    }
    CHECK(originMapPoseList.Pose_list().size() == originImageListGen.imageList.Image_list().size());
    originImageListMapCoordGen.mergePoseList(originMapPoseList);

    return true;
}

bool ATTask::writeRefined()
{
    std::string refinedImgFile = atDir + "/image_list.txt";
    std::string refinedCamFile = atDir + "/camera_list.txt";
    std::string refinedPoseFile = atDir + "/pose_list.txt";
    std::string trackFile = atDir + "/track_list.bin";

    if (info.ATStatus == eFinished) {
        bool ok = false;
        ok = refinedImageListGen.imageList.saveToASCIIFile(refinedImgFile);
        if (!ok) {
            LOG(ERROR) << "Can't write file" << refinedImgFile;
            return false;
        }
        ok = refinedCameraList.saveToJson(refinedCamFile);
        if (!ok) {
            LOG(ERROR) << "Can't write file" << refinedCamFile;
            return false;
        }
        DBPoseList poseList;

        auto& imgList = refinedImageListGen.imageList.Image_list();
        auto& pList = poseList.Pose_list();

        for (auto itr = imgList.begin();
             itr != imgList.end(); ++itr) {
            pList[itr->first] = itr->second.pose;
        }
        ok = poseList.saveToASCIIFile(refinedPoseFile);
        if (!ok) {
            LOG(ERROR) << "Can't write file" << refinedPoseFile;
            return false;
        }
        ok = trackList.saveToBinFile(trackFile);
        if (!ok) {
            LOG(ERROR) << "Can't write file" << trackFile;
            return false;
        }
        return true;
    }
    return false;
}

bool ATTask::writeInfos()
{
    int version = INSIGHT_TASK_PARAM_VERSION;
    std::ofstream ofs(taskDir + "/task_param.txt");
    ofs << "version=" << version << std::endl;
    ofs << "model=" << info.mode << std::endl;
    ofs << "status=" << info.ATStatus << std::endl;
    ofs << "gps_precision=" << info.gpsPrecision << std::endl;
    ofs << "gps_max_error=" << info.gpsMaxError << std::endl;
    ofs << "gnss_ba=" << int(info.enableGNSSBA) << std::endl;
    ofs << "max_reproject_error=" << info.maxReprojectError << std::endl;
    ofs << "max_link_features=" << info.maxLinkFeatures << std::endl;
    return true;
}

bool ATTask::readInfos()
{
    std::ifstream ifs(taskDir + "/task_param.txt");
    if (!ifs)
        return false;
    std::string s;
    std::getline(ifs, s);
    int version = 1;
    sscanf(s.c_str(), "version=%d", &version);
    if (version == 1 || version == 2 || version == 3) {
        std::getline(ifs, s);
        int mode = 0;
        int status = 0;
        sscanf(s.c_str(), "mode=%d", &mode);
        info.mode = mode;

        std::getline(ifs, s);
        sscanf(s.c_str(), "status=%d", &status);
        info.ATStatus = status;

        std::getline(ifs, s);
        float precision = 1.0;
        sscanf(s.c_str(), "gps_precision=%f", &precision);
        info.gpsPrecision = precision;

        std::getline(ifs, s);
        float maxError = 1.0;
        sscanf(s.c_str(), "gps_max_error=%f", &maxError);
        info.gpsMaxError = maxError;
    }
    if (version == 2 || version == 3) {
        std::getline(ifs, s);
        int gnssba = 0;
        sscanf(s.c_str(), "gnss_ba=%d", &gnssba);
        info.enableGNSSBA = bool(gnssba);
    }
    if (version >= 3) {
        std::getline(ifs, s);
        float maxReproj;
        int maxLinkFeatures;
        sscanf(s.c_str(), "max_reproject_error=%f", &maxReproj);
        info.maxReprojectError = maxReproj;
        std::getline(ifs, s);
        sscanf(s.c_str(), "max_link_features=%d", &maxLinkFeatures);
        info.maxLinkFeatures = maxLinkFeatures;
    }
    return true;
}

bool ATTask::writeProjectInformation()
{
    const std::string projectInfoFile = taskDir + "/project_information.json";
    std::ofstream ofs((projectInfoFile));
    if (!ofs.is_open()) {
        LOG(ERROR) << "Can't save file :" << projectInfoFile;
        return false;
    }

    cereal::JSONOutputArchive ar(ofs);
    ar(cereal::make_nvp("projectInfo", projectInfo));
    return true;
}

bool ATTask::readProjectInfomation()
{
    const std::string projectInfoFile = taskDir + "/project_information.json";
    std::ifstream ifs(projectInfoFile);
    if (!ifs.is_open()) {
        LOG(ERROR) << "Can't open project file :" << projectInfoFile;
        return false;
    }

    cereal::JSONInputArchive ar(ifs);
    ar(cereal::make_nvp("projectInfo", projectInfo));
    return true;
}

bool ATTask::writeCoordInfo()
{
    std::ofstream ofs(taskDir + "/coord_info.txt");
    if (!ofs)
        return false;
    int version = INSIGHT_TASK_COORDINFO_VERSION;
    ofs << "version=" << version << std::endl;
    ofs << gpsCoordinate.name << std::endl;
    ofs << gpsCoordinate.epsg << std::endl;
    ofs << gpsCoordinate.wkt << std::endl;
    ofs << int(gpsCoordinate.localSystem) << std::endl;
    ofs << coordinate.name << std::endl;
    ofs << coordinate.epsg << std::endl;
    ofs << coordinate.wkt << std::endl;
    ofs << int(coordinate.localSystem) << std::endl;

    return true;
}

bool ATTask::readCoordInfo()
{
    std::ifstream ifs(taskDir + "/coord_info.txt");
    if (!ifs)
        return false;
    std::string s;
    std::getline(ifs, s);
    int version = 1;
    sscanf(s.c_str(), "version=%d", &version);
    if (version == 1) {

        std::getline(ifs, s);
        gpsCoordinate.name = s;
        std::getline(ifs, s);
        gpsCoordinate.epsg = s;
        std::getline(ifs, s);
        gpsCoordinate.wkt = s;
        std::getline(ifs, s);
        int localSystem = atoi(s.c_str());
        gpsCoordinate.localSystem = (localSystem == 1);
        std::getline(ifs, s);
        coordinate.name = s;
        std::getline(ifs, s);
        coordinate.epsg = s;
        std::getline(ifs, s);
        coordinate.wkt = s;
        std::getline(ifs, s);
        localSystem = atoi(s.c_str());
        coordinate.localSystem = (localSystem == 1);
    }

    return true;
}

void ATTask::readDatas()
{
    readOrigin();
    readRefined();
    readInfos();
    readCoordInfo();
    readOriginMapCoord(); // read pose on map-coord-system
    readProjectInfomation();
}

void ATTask::writeDatas()
{
    writeOrigin();
    writeRefined();
    writeInfos();
    writeCoordInfo();
    writeProjectInformation();
}

/////////////////////////////////////////////////////////////////////
std::string Project::newAT()
{
    ATTask task;
    char name[128];
    int id = resource.taskSeed.generate();
    sprintf(name, "AT_%d", id);
    task.name = name;
    task.completeDirs(projectDataDir);
    task.createDirs();

    task.originImageListGen = imageListGen;
    task.originCameraList = cameraList;
    task.gcpList = gcpList;
    task.info.ATStatus = ATTask::eNotCommit;
    task.projectInfo = infomation;
    task.gpsCoordinate = infomation.gpsCoordinate;
    task.coordinate = infomation.coordinate;
    task.writeDatas();

    atTaskList.push_back(task);
    saveProject();
    return task.id;
}

std::string Project::newModel(const std::string& atId)
{
    int idx = findATTask(atId);
    CHECK_NE(idx, -1);
    ATTask& attask = atTaskList[idx];
    ModelTask task;
    char name[128];
    int id = resource.modelSeed.generate();
    sprintf(name, "Model_%d", id);
    task.name = name;
    task.atId = atId;
    task.completeDirs(projectDataDir);
    task.createDirs();
    // copy at result
    task.atResultGen = attask.refinedImageListGen;
    task.atResultCameraList = attask.refinedCameraList;
    task.atResultTrackList = attask.trackList;
    //
    task.updateGrid();
    task.writeDatas();
    modelTaskList.push_back(task);
    saveProject();
    return task.id;
}

 void computeSharedPoints(const DBTrackList& trackList, std::map<int, std::map<int, int>>& shared_points)
{
    shared_points.clear();

    for (auto trackItr = trackList.TrackList().begin();
         trackItr != trackList.TrackList().end(); ++trackItr) {
        const auto& views = trackItr->second.views;
        for (size_t i = 0; i < views.size(); ++i) {
            KeyType image_id1 = views[i].image_id;
            for (size_t j = 0; j < i; ++j) {
                const KeyType image_id2 = views[j].image_id;
                if (image_id1 != image_id2) {
                    shared_points[image_id1][image_id2] += 1;
                    shared_points[image_id2][image_id1] += 1;
                }
            }
        }
    }
}

 void generateNeighbors(const DBImageList& imageList,
    const DBTrackList& trackList,
    std::map<int, std::vector<int>>& imageNeighbors)
{
    std::map<int, std::map<int, int>> shared_points;
    computeSharedPoints(trackList, shared_points);

    for (auto itr = imageList.Image_list().begin(); itr != imageList.Image_list().end(); ++itr) {
        std::vector<int>& neighbors = imageNeighbors[itr->first];
        auto findItr = shared_points.find(itr->first);
        if (findItr == shared_points.end())
            continue;
        const auto& overlapping_images = shared_points.at(itr->first);
        std::vector<std::pair<int, int>> src_images;
        src_images.reserve(overlapping_images.size());
        for (const auto& image : overlapping_images) {
            src_images.emplace_back(image.first, image.second);
        }

        std::sort(src_images.begin(), src_images.end(), [](const std::pair<int, int> image1, const std::pair<int, int> image2) {
            return image1.second > image2.second;
        });

        for (size_t ii = 0; ii < src_images.size(); ++ii) {
            neighbors.push_back(src_images[ii].first);
        }
        if (neighbors.empty()) {
            LOG(WARNING) << itr->first << " Neighbors is empty";
        }
    }
}

int Project::findATTask(const std::string& taskId)
{
    for (int i = 0; i < atTaskList.size(); ++i) {
        if (atTaskList[i].id == taskId)
            return i;
    }
    return -1;
}

int Project::findModelTask(const std::string& taskId)
{
    for (int i = 0; i < modelTaskList.size(); ++i) {
        if (modelTaskList[i].id == taskId)
            return i;
    }
    return -1;
}
}//name space insight
