

#ifndef Project_h
#define Project_h

#include <cereal/cereal.hpp>
#include <cstdint>
#include <string>

#include "Coordinates.h"
#include "common_global.h"
#include "data_sheet.h"
#include "db_types.h"
#include "string_utils.h"

//////////////////////////////////////////////////////////////////////////
namespace insight{

struct  CoordInfomation {
    CoordInfomation()
    {
        //        ENUCenter = Vec3(0, 0, 0);
    }
    std::string name;
    std::string epsg;
    std::string wkt;

    bool localSystem = true; //

    template <class Archive>
    void serialize(Archive& ar, std::uint32_t const version)
    {
        ar(CEREAL_NVP(name),
            CEREAL_NVP(epsg),
            CEREAL_NVP(wkt));
        ar(CEREAL_NVP(localSystem));
    }

    bool isProject(bool* ok = nullptr) const;
};

struct  ProjectInfomation {
    enum {
        UNKNOWN_ALTITUTE = 0
    };
    enum {
        Aerial = 0,
        Object = 1,
    };
    std::string name;
    std::string description;
    std::string date;
    std::string author;
    int type = Aerial;
    float relativeFlightAltitude = UNKNOWN_ALTITUTE;
    float averageElevationOfGround = UNKNOWN_ALTITUTE;
    CoordInfomation gpsCoordinate; // gps
    CoordInfomation coordinate; // map coordinate

    // static method
    static bool read(std::ifstream& ifs, ProjectInfomation& info);
    static bool write(std::ofstream& ofs, const ProjectInfomation& info);

    template <class Archive>
    void serialize(Archive& ar, std::uint32_t const version)
    {
        // You can choose different behaviors depending on the version
        // This is useful if you need to support older variants of your codebase
        // interacting with newer ones
        if (version == 1) {
            ar(CEREAL_NVP(name));
            ar(CEREAL_NVP(description));
            ar(CEREAL_NVP(type));
            ar(CEREAL_NVP(date));
            ar(CEREAL_NVP(author));
            ar(CEREAL_NVP(relativeFlightAltitude));
            ar(CEREAL_NVP(averageElevationOfGround));
            ar(CEREAL_NVP(coordinate));
            ar(CEREAL_NVP(gpsCoordinate));
        }
    }
};
/*
 */
struct  ProjectConfigData {
    ProjectConfigData()
    {
        // invalid box initialized
        boxMin = Vec3(1, 1, 1);
        boxMax = Vec3(-1, -1, -1);
    }
    float sfmScale = 1.0f;
    // scene box
    Vec3 boxMin;
    Vec3 boxMax;
    template <class Archive>
    void serialize(Archive& ar, std::uint32_t const version)
    {
        ar(CEREAL_NVP(sfmScale));
        // version 2 add
        if (version == 1) {
            ar(cereal::make_nvp("minX", boxMin.x()));
            ar(cereal::make_nvp("minY", boxMin.y()));
            ar(cereal::make_nvp("minZ", boxMin.z()));
            ar(cereal::make_nvp("maxX", boxMax.x()));
            ar(cereal::make_nvp("maxY", boxMax.y()));
            ar(cereal::make_nvp("maxZ", boxMax.z()));
        }
    }
};

struct  ImageConsistency {
    KeyType imageId = UndefinedKey;
    KeyType cameraId = UndefinedKey;
    int imageW = 0;
    int imageH = 0;
    int cameraW = 0;
    int cameraH = 0;
    bool imageExist = false;
    bool imageCanRead = false;
    inline bool isOK() const
    {
        if (imageId != UndefinedKey && cameraId != UndefinedKey && imageW != 0 && imageH != 0 && imageW == cameraW && imageH == cameraH && imageExist && imageCanRead) {
            return true;
        }
        return false;
    }
};

struct  Task {
    enum Type {
        eAT,
        eModel,
    };

    std::string id;
    Type type;

    void generateID()
    {
        id = GetUUID();
    }
};

struct  ATTask : public Task {
    ATTask()
    {
        type = eAT;
        generateID();
    }

    void completeDirs(const std::string& parentFolder);

    void createDirs();

    void readDatas();
    void writeDatas();

    bool readOrigin();
    bool writeOrigin();

    bool readRefined();
    bool writeRefined();

    // read pose in map-coordinate-system
    bool readOriginMapCoord();

    bool writeInfos();
    bool readInfos();

    bool writeProjectInformation();
    bool readProjectInfomation();

    bool writeCoordInfo();
    bool readCoordInfo();

    bool refreshGCPList();

    // version=1
    // 0=not commit,1=commited,2=finished
    enum enumATStatus {
        eNotCommit = 0,
        eCommitted = 1,
        eFinished = 2
    };
    struct Infos {
        int mode = 1; // 0 highest,1 hight ;2 mid; 3 low 4 lowest
        int ATStatus = eNotCommit;
        double gpsPrecision = 1.0; // 1 meter
        double gpsMaxError = 20.0;
        int maxLinkFeatures = 1000;
        double maxReprojectError = 2.0;
        bool enableGNSSBA = true;
    };

    Infos info;
    ProjectInfomation projectInfo;
    std::string name;
    ImageListGenerator originImageListGen;
    ImageListGenerator originImageListMapCoordGen; // if origin coordinate is geo-system, project the coords
    ImageListGenerator refinedImageListGen;
    DBCameraList originCameraList;
    DBCameraList refinedCameraList;

    CoordInfomation gpsCoordinate; // gps
    CoordInfomation coordinate; // map coordinate

    DBTrackList trackList;
    DBGCPList gcpList;

    std::string parentDir;
    std::string taskDir;
    std::string featsDir; // = projectDataDir + "/feats";
    std::string matchDir; // = projectDataDir + "/match";
    std::string atDir; // = projectDataDir + "/AT";
};

struct ModelGrid {
    ModelGrid()
        : minx(0)
        , miny(0)
        , minz(0)
        , maxx(1)
        , maxy(1)
        , maxz(1)
    {
    }
    int xcount = 1;
    int ycount = 1;
    int zcount = 1;
    double centerx = 0;
    double centery = 0;
    double centerz = 0;

    double minx, miny, minz;
    double maxx, maxy, maxz;
    std::vector<double> xs;
    std::vector<double> ys;
    std::vector<double> zs;
    void generate(double low, double high, int count, std::vector<double>& datas)
    {
        if (count == 1) {
            datas.push_back(low);
            datas.push_back(high);
        } else if (count == 2) {
            double mid = (low + high) * 0.5;
            datas.emplace_back(low);
            datas.emplace_back(mid);
            datas.emplace_back(high);
        } else {
            int n = count - 1;
            double space = (high - low) / n;
            double half = space * 0.5;
            datas.emplace_back(low);
            double start = low + half;
            for (int i = 0; i < n; ++i) {
                double d = start + i * space;
                datas.emplace_back(d);
            }
            datas.emplace_back(high);
        }
    }

    void generateDatas()
    {
        //两边都是0.5，中间都是1
        xs.clear();
        ys.clear();
        zs.clear();
        generate(minx, maxx, xcount, xs);
        generate(miny, maxy, ycount, ys);
        generate(minz, maxz, zcount, zs);
    }

    void updateCenter(double cx, double cy, double cz, bool updateMinMax = true)
    {
        if (updateMinMax) {
            minx += centerx;
            miny += centery;
            minz += centerz;
            maxx += centerx;
            maxy += centery;
            maxz += centerz;
        }
        centerx = cx;
        centery = cy;
        centerz = cz;
        if (updateMinMax) {
            minx -= centerx;
            miny -= centery;
            minz -= centerz;
            maxx -= centerx;
            maxy -= centery;
            maxz -= centerz;
        }
    }
    template <class Archive>
    void serialize(Archive& ar, std::uint32_t const version)
    {
        ar(cereal::make_nvp("minX", minx));
        ar(cereal::make_nvp("minY", miny));
        ar(cereal::make_nvp("minZ", minz));
        ar(cereal::make_nvp("maxX", maxx));
        ar(cereal::make_nvp("maxY", maxy));
        ar(cereal::make_nvp("maxZ", maxz));

        ar(cereal::make_nvp("xcount", xcount));
        ar(cereal::make_nvp("ycount", ycount));
        ar(cereal::make_nvp("zcount", zcount));

        ar(cereal::make_nvp("centerx", centerx));
        ar(cereal::make_nvp("centery", centery));
        ar(cereal::make_nvp("centerz", centerz));
    }
};

struct  ModelTask : public Task {
    ModelTask()
    {
        type = eModel,
        generateID();
    }

    void completeDirs(const std::string& parentFolder);

    void createDirs();

    bool readDatas();
    bool writeDatas();

    // update grid datas by track
    bool updateGrid();

    ImageListGenerator atResultGen;
    DBCameraList atResultCameraList;
    DBTrackList atResultTrackList;

    std::string parentDir; // atdir
    std::string taskDir;
    std::string name;
    std::string atId; // AT task uuid

    ModelGrid grid;
};

typedef std::vector<ATTask> ATTaskList;
typedef std::vector<ModelTask> ModelTaskList;

class  Project {
public:
    enum SaveFlag {
        eSaveImageAndCameras = 0x01,
        eSavePose = 0x02,
        eSaveATResult = 0x04,
        eSaveGCP = 0x08,
        eSaveAll = eSaveImageAndCameras | eSavePose | eSaveATResult | eSaveGCP,
    };
    enum SaveBinFlag {
        eSaveTrackBin = 0x01,
    };

    Project();
    bool openProject(const std::string& projectFile);
    bool getProjectInfomation(const std::string& projectFile);
    bool createProject(const ProjectInfomation& info, const std::string& path);
    bool saveProject(SaveFlag flag = eSaveAll);

    //**obsolete,will delete this function later
    void generateCameraByExif();

    void generateCameraByExif(uint32_t camera_id);

    void generateCameraByExif(const std::vector<int>& imageIds);

    void completeCameraByExif(const std::vector<int>& imageIds, KeyType camId);

    //**obsolete,will delete this function later
    void getCameraFromImageWH();

    void getCameraFromImageWH(uint32_t camera_id);
    
    void getCameraFromImageWH(const std::vector<int>& imgIds);

    void checkConsistency(std::vector<ImageConsistency>& result);

    // return the AT id;
    std::string newAT();
    std::string newModel(const std::string& atId);
    int findATTask(const std::string& taskId); // return the index of atTaskList; if not exist , return -1
    int findModelTask(const std::string& taskId);

#if 0
    bool updateENUCoord();
#endif
    void setSaveBinFlag(int flag)
    {
        _saveBinFlag = flag;
    }
    ImageListGenerator imageListGen;
    DBCameraList cameraList;
    DBGCPList gcpList; //
    ATTaskList atTaskList;
    ModelTaskList modelTaskList;

    ProjectInfomation infomation;
    std::string projectFile; //
    std::string projectDir; //
    std::string projectDataDir; //
    Resource resource; //
    ProjectConfigData configData;

    template <class Archive>
    void serialize(Archive& ar, std::uint32_t const version);

private:
    KeyType findCameraByCameraHashCode(const size_t& hashCode);
    void completeDirs(); //

    int _saveBinFlag = 0;
};

class  SystemConfig {
public:
    static SystemConfig& instance();
    static void exit();

    std::string exePath() const;
    std::string configPath() const;
    void setExePath(const std::string& exepath);
    void setConfigPath(const std::string& config);
    bool readSensorDatabase();
    bool readCoordinate();
    std::vector<Datasheet> sensorDatabase;
    std::vector<Coordinate> ProjCoordinate;
    std::vector<Coordinate> GeoCoordinate;
    int maxImages = 1000;

private:
    SystemConfig() { }
    SystemConfig(SystemConfig&);
    SystemConfig& operator=(SystemConfig&);
    std::string _exePath;
    std::string _configPath;
};

 void computeSharedPoints(
    const DBTrackList& trackList,
    std::map<int, std::map<int, int>>& shared_points);

 float computeGSD(
    const DBImageList& imageList,
    const DBCameraList& cameraList,
    const DBTrackList& trackList);

 void generateNeighbors(
    const DBImageList& imageList,
    const DBTrackList& trackList,
    std::map<int, std::vector<int>>& imageNeighbors);

}//name space insight

CEREAL_CLASS_VERSION(insight::ProjectInfomation, 1);
CEREAL_CLASS_VERSION(insight::CoordInfomation, 1);
CEREAL_CLASS_VERSION(insight::ProjectConfigData, 1);
CEREAL_CLASS_VERSION(insight::ModelGrid, 1);
CEREAL_CLASS_VERSION(insight::Project, 1);

#define INSIGHT_TASK_VERSION 2
#define INSIGHT_TASK_MODEL_VERSION 1
#define INSIGHT_TASK_COORDINFO_VERSION 1
#define INSIGHT_TASK_PARAM_VERSION 3
#endif // Project_h__
