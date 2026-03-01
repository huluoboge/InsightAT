#include "db_types.h"

#include <fstream>
#include <glog/logging.h>
#include <cereal/archives/json.hpp>
#include <sstream>
#include "exif_IO_EasyExif.hpp"
#include "exif_IO.hpp"
#include "hash.h"
#include "Project.h"

#include <iomanip>
#include <algorithm>
#include <ImageIO/gdal_utils.h>

#ifndef IMAGE_NEIGHBOR_FILE_VERSION
#define IMAGE_NEIGHBOR_FILE_VERSION 1
#endif

using namespace cereal;
using namespace std;

namespace insight{

bool DBImageList::saveToASCIIFile(const std::string &file) const
{
    ofstream ofs(file);
    if (!ofs.is_open())
        return false;
    ofs << "#Insight image list file." << std::endl
        << "#Version 1.1. Generate by Task" << std::endl
        << "#Format : " << endl
        << "#image_id;camera_id;image_name_with_ext;image_full_path_name;pose_valid;pose_t_valid,pose_r_valid" << endl;
    ofs << "1.1" << std::endl; //version
    for (auto itr = Image_list().begin(); itr != Image_list().end(); ++itr)
    {
        ofs << itr->first << ";"
            << itr->second.camera_id << ";"
            << itr->second.image_name << ";"
            << itr->second.image_full_path << ";"
            << itr->second.pose_valid << ";"
            << 0 << ";"
            << 0 << std::endl;
        //            << itr->second.init_pose_center_valid << ";"
        //            << itr->second.init_pose_rotation_valid<< std::endl;
    }
    return true;
}

//version 1.1
bool DBImageList::readFromASCIIFile(const std::string &file)
{
    ifstream ifs(file);
    if (!ifs)
    {
        return false;
    }
    std::string s;
    int line = 0;
    Image_list().clear();
    bool readVersion = false;
    while (std::getline(ifs, s))
    {
        ++line;
        if (s.empty())
            continue;
        if (s[0] == '#')
            continue; //ignore #
        if (!readVersion)
        {
            readVersion = true;
            if (s != "1.1")
            {
                LOG(ERROR) << "need version 1.1 at file " << file << ".line:" << line;
                return false;
            }
            continue;
        }
        vector<string> datas;
        split(s, ';', datas);
        if (datas.size() != 7)
        {
            LOG(ERROR) << "Must 7 datas but " << datas.size() << " at file " << file << ".line:" << line;
            return false;
        }
        for (int ii = 0; ii < 7; ++ii)
            datas[ii] = trim(datas[ii]);
        DBImage image;

        stringstream imagess(datas[0]);
        stringstream camerass(datas[1]);
        imagess >> image.id;
        camerass >> image.camera_id;
        image.image_name = datas[2];
        image.image_full_path = datas[3];
        stringstream ss(datas[4]);
        ss >> image.pose_valid;
        ss.clear();
        //        ss.str(datas[5]); 	ss >> image.init_pose_center_valid; ss.clear();
        //        ss.str(datas[6]); 	ss >> image.init_pose_rotation_valid; ss.clear();
        Image_list()[image.id] = image;
    }
    LOG(INFO) << file << " loaded";
    return true;
}

bool DBImageList::hasImage(uint32_t image_id) const
{
    return Image_list().find(image_id) != Image_list().end();
}

bool DBImageList::hasImage(const std::string &fullPathName) const
{
    return _imagePathCacheList.find(fullPathName) != _imagePathCacheList.end();
}

void DBImageList::addImage(const DBImage &dbimage)
{
    Image_list()[dbimage.id] = dbimage;
}

void DBImageList::resetUnregisted()
{
    for (auto itr = _image_list.begin(); itr != _image_list.end(); ++itr)
    {
        itr->second.pose_valid = false;
    }
}

void insight::DBImageList::buildPatchCache()
{
    _imagePathCacheList.clear();
    for (auto itr = Image_list().begin(); itr != Image_list().end(); ++itr)
    {
        _imagePathCacheList.insert(itr->second.image_full_path);
    }
}

bool DBCameraList::saveToJson(const std::string &file) const
{
    std::ofstream ofs(file);
    if (!ofs)
    {
        return false;
    }
    JSONOutputArchive ar(ofs);
    ar(cereal::make_nvp("CameraList", _head));
    for (auto itr = _camera_list.begin(); itr != _camera_list.end(); ++itr)
    {
        stringstream ss;
        ss << itr->first;
        ar(cereal::make_nvp(ss.str(), itr->second));
    }
    return ofs.is_open();
}

bool DBCameraList::readFromJson(const std::string &file)
{
    std::ifstream ifs(file);
    if (!ifs)
    {
        return false;
    }
    try
    {
        JSONInputArchive ar(ifs);
        _camera_list.clear();
        ar(cereal::make_nvp("CameraList", _head));

        auto hint = _camera_list.begin();
        while (true)
        {
            const auto namePtr = ar.getNodeName();

            if (!namePtr)
                break;
            stringstream ss;
            ss.str((std::string(namePtr)));
            uint32_t cam_id;
            DBCamera cam;
            ss >> cam_id;
            ar(cam);
            hint = _camera_list.emplace_hint(hint, cam_id, std::move(cam));
        }
        LOG(INFO)<<"Camera list loaded";
        return true;
    }
    catch (...)
    {
        LOG(INFO) << "Can't read file " << file;
        return false;
    }
}

void DBImage::readExif()
{
    std::unique_ptr<Exif_IO_EasyExif> exifReader(new Exif_IO_EasyExif());
    exifReader->open(image_full_path);
    if (exifReader->doesHaveExifInfo())
    {
        exif_header = SimpleExifHeader(exifReader->exifInfo());
        exif_valid = true;
    }
    else
    {
        exif_valid = false;
    }
}

void DBImage::readExif(const std::vector<unsigned char> &buf)
{
    std::unique_ptr<Exif_IO_EasyExif> exifReader(new Exif_IO_EasyExif());
    exifReader->open(buf);
    if (exifReader->doesHaveExifInfo())
    {
        exif_header = SimpleExifHeader(exifReader->exifInfo());
        exif_valid = true;
    }
    else
    {
        exif_valid = false;
    }
}

bool DBImage::getCameraByExif(DBCamera &camera) const
{
    if (camera.camera_name == "")
    {
        camera.camera_name = exif_header.Make + " " + exif_header.Model + " " +
                             std::to_string(exif_header.FocalLength) + "_" + std::to_string(exif_header.FocalLengthIn35mm);
    }

    camera.w = exif_header.width;
    camera.h = exif_header.height;
    if (camera.w == 0 || camera.h == 0)
    {
        GdalUtils::InitGDAL();
        GdalUtils::GetWidthHeightPixel(image_full_path.c_str(), camera.w, camera.h);
    }
    camera.ppx = (camera.w - 1) * 0.5;
    camera.ppy = (camera.h - 1) * 0.5;
    camera.focalmm = exif_header.FocalLength;
    camera.focal35mm = exif_header.FocalLengthIn35mm;
    float w = camera.w;
    float h = camera.h;
    if (exif_header.FocalLengthIn35mm != 0)
    {
        //35mm size: 36 * 24 (mm)
        //�Խ��ߣ�43.26661530556787mm
        double scale = exif_header.FocalLengthIn35mm / 43.26661530556787;
        double a = sqrt(w * w + h * h);
        camera.focalpx = scale * a;
        if (exif_header.FocalLength != 0.0)
        {
            float pix_size = exif_header.FocalLength / camera.focalpx;
            double sensor_x = pix_size * w;
            double sensor_y = pix_size * h;
            camera.sensor_size_x = sensor_x;
            camera.sensor_size_y = sensor_y;
        }
        else
        {
            //����sensor size��focalpx�Ѿ����Ƴ�
            Datasheet ds;
            if (getInfo(exif_header.Make, exif_header.Model,
                        SystemConfig::instance().sensorDatabase, ds))
            {
                float pix_size = ds._sensorSize / std::max(camera.w, camera.h);
                double sensor_x = pix_size * w;
                double sensor_y = pix_size * h;
                camera.sensor_size_x = sensor_x;
                camera.sensor_size_y = sensor_y;
            }
        }
        return true;
    }
    else
    {
        Datasheet ds;
        if (getInfo(exif_header.Make, exif_header.Model, SystemConfig::instance().sensorDatabase,
                    ds))
        {
            float pix_size = ds._sensorSize / std::max(camera.w, camera.h);
            double sensor_x = pix_size * w;
            double sensor_y = pix_size * h;
            camera.sensor_size_x = sensor_x;
            camera.sensor_size_y = sensor_y;
            if (camera.focalmm != 0.0)
            {
                camera.focalpx = camera.focalmm / pix_size;
            }
            return true;
        }
    }
    return true;
}

bool DBImage::getCameraByWH(DBCamera &camera) const
{
    std::string Make = "UnknownMaker";
    std::string Model = "UnknownModel";
    GdalUtils::InitGDAL();
    int w = 0, h = 0;
    GdalUtils::GetWidthHeightPixel(image_full_path.c_str(), w, h);
    camera.w = w;
    camera.h = h;
    camera.ppx = (w - 1.0) / 2.0;
    camera.ppy = (h - 1.0) / 2.0;
    camera.focalpx = 0;
    float FocalLength = 0;
    float FocalLengthIn35mm = 0;
    camera.camera_name = Make + " " + Model + " " +
                         std::to_string(FocalLength) + "_" + std::to_string(FocalLengthIn35mm);
    return true;
}
size_t DBImage::cameraHashCode() const
{
    size_t seed = 0;
    hash_combine(seed, exif_header.width);
    hash_combine(seed, exif_header.height);
    hash_combine(seed, exif_header.FocalLength);
    hash_combine(seed, exif_header.FocalLengthIn35mm);
    hash_combine(seed, exif_header.Make);
    hash_combine(seed, exif_header.Model);
    return seed;
}

bool DBPoseList::saveToASCIIFile(const std::string &file) const
{
    ofstream ofs(file);
    ofs << std::fixed << std::setprecision(10);
    if (!ofs.is_open())
        return false;
    ofs << "#Insight pose list file." << std::endl
        << "#Version 2. Generate by InsightAT" << std::endl
        << "#Format : " << endl
        << "#image_id;x;y;z;omega;phi;kappa;weight_x;weight_y;weight_z;angleUnit;coordinate;eulerAngle" << std::endl
        << "#angleUnit:0=deg,1=rad" << std::endl
        << "#coordinate: 0=x-right,y-down,z-forward; 1=x-right,y-up,z-backward" << std::endl
        << "#eulerAngle: 0=OmegaPhiKappa,1=PhiOmegaKappa" << std::endl;
    ofs << "version;" << VERSION << std::endl;
    for (auto itr = _pose_list.begin(); itr != _pose_list.end(); ++itr)
    {
        ofs << itr->first << ";"
            << itr->second.x << ";"
            << itr->second.y << ";"
            << itr->second.z << ";"
            << itr->second.omega << ";"
            << itr->second.phi << ";"
            << itr->second.kappa << ";"
            << itr->second.weight_x << ";"
            << itr->second.weight_y << ";"
            << itr->second.weight_z << ";"
            << itr->second.angleUnit << ";"
            << itr->second.coordinate << ";"
            << itr->second.eulerAngle << std::endl;
    }
    return true;
}

bool DBPoseList::readFromASCIIFile(const std::string &file)
{
    ifstream ifs(file);
    if (!ifs)
    {
        return false;
    }
    std::string s;
    int line = 0;
    _pose_list.clear();
    int version = -1;
    while (std::getline(ifs, s))
    {
        ++line;
        if (s.empty())
            continue;
        if (s[0] == '#')
            continue; //ignore #

        vector<string> datas;
        split(s, ';', datas);
        if (version == -1)
        {
            if (datas.size() != 2)
            {
                LOG(ERROR) << "Must 2 datas but " << datas.size() << " at file " << file << ".line:" << line;
                return false;
            }
            for (int ii = 0; ii < 2; ++ii)
                datas[ii] = trim(datas[ii]);
            if (datas[0] != "version")
            {
                LOG(ERROR) << "Can't find version " << datas.size() << " at file " << file << ".line:" << line;
                return false;
            }
            stringstream ss(datas[1]);
            ss >> version;
            continue;
        }
        if (version == 0)
        {
            if (datas.size() != 7)
            {
                LOG(ERROR) << "Must 7 datas but " << datas.size() << " at file " << file << ".line:" << line;
                return false;
            }
        }
        else if (version == 1)
        {
            if (datas.size() != 10)
            {
                LOG(ERROR) << "Must 10 datas but " << datas.size() << " at file " << file << ".line:" << line;
                return false;
            }
        }
        else if (version == 2)
        {
            if (datas.size() != 13)
            {
                LOG(ERROR) << "Must 13 datas but " << datas.size() << " at file " << file << ".line:" << line;
                return false;
            }
        }
        else
        {
            LOG(ERROR) << "Unknown version of file " << file << line;
            return false;
        }

        for (int ii = 0; ii < datas.size(); ++ii)
            datas[ii] = trim(datas[ii]);
        DBPose pose;

        stringstream image_id(datas[0]);
        stringstream x(datas[1]);
        stringstream y(datas[2]);
        stringstream z(datas[3]);
        stringstream rx(datas[4]);
        stringstream ry(datas[5]);
        stringstream rz(datas[6]);

        image_id >> pose.image_id;
        x >> pose.x;
        y >> pose.y;
        z >> pose.z;
        rx >> pose.omega;
        ry >> pose.phi;
        rz >> pose.kappa;
        if (version == 1)
        {
            stringstream wx(datas[7]);
            stringstream wy(datas[8]);
            stringstream wz(datas[9]);
            wx >> pose.weight_x;
            wy >> pose.weight_y;
            wz >> pose.weight_z;
        }
        else if (version == 2)
        {
            stringstream a(datas[10]);
            stringstream b(datas[11]);
            stringstream c(datas[12]);
            a >> pose.angleUnit;
            b >> pose.coordinate;
            c >> pose.eulerAngle;
        }
        _pose_list[pose.image_id] = pose;
    }
    LOG(INFO)<<"Pose loaded";
    return true;
}

bool DBTrackList::saveToAsciiFile(const std::string &file) const
{
    ofstream ofs(file);
    ofs << std::fixed << std::setprecision(10);
    if (!ofs.is_open())
        return false;
    ofs << "#Insight track list file." << std::endl
        << "#Version 1.0. Generate by Task" << std::endl
        << "#Format : " << endl
        << "#track_id;x;y;z" << endl
        << "#view_id;u;v;scale;view_id;u;v;scale..." << endl;
    ofs << "version;" << VERSION << std::endl;

    for (auto itr = _track_lsit.begin(); itr != _track_lsit.end(); ++itr)
    {
        const DBTrack &track = itr->second;
        ofs << track.track_id << ";" << track.landmark.x << ";" << track.landmark.y << ";"
            << track.landmark.z;
        if (VERSION >= 1)
        {
            ofs << ";" << (int)track.landmark.r
                << ";" << (int)track.landmark.g
                << ";" << (int)track.landmark.b << std::endl;
        }
        else
        {
            ofs << std::endl;
        }
        for (int i = 0; i < track.views.size(); ++i)
        {
            ofs << track.views[i].image_id << ";" << track.views[i].u << ";" << track.views[i].v;
            if(VERSION >=2){
                ofs<<";"<<track.views[i].scale;
            }
            if (i != int(track.views.size()) - 1)
            {
                ofs << ";";
            }
        }
        ofs << std::endl;
    }
    return true;
}

bool DBTrackList::readFromAsciiFile(const std::string &file)
{
    ifstream ifs(file);
    std::string s;
    int line = 0;
    _track_lsit.clear();
    int version = -1;
    while (std::getline(ifs, s))
    {
        ++line;
        if (s.empty())
            continue;
        if (s[0] == '#')
            continue; //ignore #
        if (version == -1)
        {
            vector<string> data;
            split(s, ';', data);
            if (data.size() != 2)
            {
                LOG(ERROR) << "Must 2 datas but " << data.size() << " at file " << file << ".line:" << line;
                return false;
            }
            for (int ii = 0; ii < 2; ++ii)
                data[ii] = trim(data[ii]);
            if (data[0] != "version")
            {
                LOG(ERROR) << "No version find at file " << file << ".line:" << line;
                return false;
            }
            stringstream ss;
            ss << data[1];
            ss >> version;
            continue;
        }
        std::string sviews;
        std::getline(ifs, sviews);

        vector<string> Xs;
        vector<string> Vs;
        split(s, ';', Xs);
        split(sviews, ';', Vs);
        if (version >= 1 && Xs.size() != 7)
        {
            LOG(ERROR) << "Must 7 datas but " << Xs.size() << " at file " << file << ".line:" << line;
            return false;
        }
        if(version == 1){
            if (Vs.size() % 3 != 0)
            {
                LOG(ERROR) << "Must 3 data per view but  at file " << file << ".line:" << line;
                return false;
            }
        }else if(version == 2){
            if (Vs.size() % 4 != 0)
            {
                LOG(ERROR) << "Must 4 data per view but  at file " << file << ".line:" << line;
                return false;
            }
        }
        for (int ii = 0; ii < Xs.size(); ++ii)
            Xs[ii] = trim(Xs[ii]);
        for (int ii = 0; ii < Vs.size(); ++ii)
            Vs[ii] = trim(Vs[ii]);
        DBTrack track;

        stringstream ss;
        ss.str(Xs[0]);
        ss >> track.track_id;
        ss.clear();
        ss.str(Xs[1]);
        ss >> track.landmark.x;
        ss.clear();
        ss.str(Xs[2]);
        ss >> track.landmark.y;
        ss.clear();
        ss.str(Xs[3]);
        ss >> track.landmark.z;
        ss.clear();
        if (version >= 1)
        {
            int r = 0, g = 0, b = 0;
            ss.str(Xs[4]);
            ss >> r;
            ss.clear();
            ss.str(Xs[5]);
            ss >> g;
            ss.clear();
            ss.str(Xs[6]);
            ss >> b;
            ss.clear();
            track.landmark.r = r;
            track.landmark.g = g;
            track.landmark.b = b;
        }
        
        int viewCount = 0;
        int base = 3;
        if(version == 1){
            viewCount = Vs.size() / 3;
        }
        if(version == 2){
            viewCount = Vs.size() / 4;
            base = 4;
        }

        track.views.resize(viewCount);
        for (int ii = 0; ii < viewCount; ++ii)
        {
            DBTrack::V &v = track.views[ii];
            ss.str(Vs[base * ii]);
            ss >> v.image_id;
            ss.clear();
            ss.str(Vs[base * ii + 1]);
            ss >> v.u;
            ss.clear();
            ss.str(Vs[base * ii + 2]);
            ss >> v.v;
            if(version == 2){
                ss>>v.scale;
            }
            ss.clear();
        }
        _track_lsit[track.track_id] = track;
    }
    return true;
}

bool DBTrackList::readFromBinFile(const std::string &file)
{
    ifstream ifs(file, std::ios::in | std::ios::binary);
    if (!ifs.is_open())
        return false;

    _track_lsit.clear();
    int version = -1;
    ifs.read((char *)(&version), sizeof(int));
    int nTrack = 0;
    ifs.read((char *)(&nTrack), sizeof(int));

    for (int i = 0; i < nTrack; ++i)
    {
        DBTrack track;
        ifs.read((char *)&track.track_id, sizeof(int));
        ifs.read((char *)&track.landmark.x, sizeof(double));
        ifs.read((char *)&track.landmark.y, sizeof(double));
        ifs.read((char *)&track.landmark.z, sizeof(double));
        if (version >= 1)
        {
            int R = 0; //(int)
            int G = 0; //(int);
            int B = 0; //(int);

            ifs.read((char *)&R, sizeof(int));
            ifs.read((char *)&G, sizeof(int));
            ifs.read((char *)&B, sizeof(int));
            track.landmark.r = R;
            track.landmark.g = G;
            track.landmark.b = B;
        }
        int nView = 0;
        ifs.read((char *)&nView, sizeof(int));

        track.views.resize(nView);
        for (int i = 0; i < track.views.size(); ++i)
        {
            ifs.read((char *)&track.views[i].image_id, sizeof(int));
            ifs.read((char *)&track.views[i].u, sizeof(float));
            ifs.read((char *)&track.views[i].v, sizeof(float));
            if(version == 2){
                ifs.read((char *)&track.views[i].scale, sizeof(float));
            }
        }
        _track_lsit[track.track_id] = track;
    }

    LOG(INFO)<<"Track loaded";
    return true;
}

bool insight::DBTrackList::saveToBinFile(const std::string &file) const
{
    ofstream ofs(file, std::ios::out | std::ios::binary | std::ios::trunc);
    ofs << std::fixed << std::setprecision(10);
    if (!ofs.is_open())
        return false;

    int version = VERSION;
    ofs.write((char *)(&version), sizeof(int));
    int nTrack = _track_lsit.size();
    ofs.write((char *)(&nTrack), sizeof(int));

    for (auto itr = _track_lsit.begin(); itr != _track_lsit.end(); ++itr)
    {
        const DBTrack &track = itr->second;
        ofs.write((char *)&track.track_id, sizeof(int));
        ofs.write((char *)&track.landmark.x, sizeof(double));
        ofs.write((char *)&track.landmark.y, sizeof(double));
        ofs.write((char *)&track.landmark.z, sizeof(double));

        if (VERSION >= 1)
        {
            int R = (int)track.landmark.r;
            int G = (int)track.landmark.g;
            int B = (int)track.landmark.b;

            ofs.write((char *)&R, sizeof(int));
            ofs.write((char *)&G, sizeof(int));
            ofs.write((char *)&B, sizeof(int));
        }

        int nView = track.views.size();
        ofs.write((char *)&nView, sizeof(int));

        for (int i = 0; i < track.views.size(); ++i)
        {
            ofs.write((char *)&track.views[i].image_id, sizeof(int));
            ofs.write((char *)&track.views[i].u, sizeof(float));
            ofs.write((char *)&track.views[i].v, sizeof(float));
            if(version >=2){
                ofs.write((char *)&track.views[i].scale, sizeof(float));
            }
        }
    }
    return true;
}

bool DBGCPList::saveToAsciiFile(const std::string &file) const
{
    ofstream ofs(file);
    ofs << std::fixed << std::setprecision(10);
    if (!ofs.is_open())
        return false;
    ofs << "#Insight gcp list file." << std::endl
        << "#Generate by InsightAT" << std::endl
        << "#Format : " << endl
        << "#name;type;enabled;track_id;x;y;z;" << endl
        << "#view_id;u;v;reprojectU;reprojectV;enabled;view_id;u;v;reprojectU;reprojectV;enabled..." << endl;
    ofs << "VERSION=" << DBGCPListVersion << std::endl;
    for (auto itr = _gcp_lsit.begin(); itr != _gcp_lsit.end(); ++itr)
    {
        const DBGCP &track = itr->second;
        ofs << track.name << ";" << track.type << ";" << track.enabled << ";"
            << track.track_id << "; " << track.landmark.x << "; " << track.landmark.y << "; "
            << track.landmark.z << std::endl;
        int i = 0;
        for (auto itr = track.views.begin(); itr != track.views.end(); ++itr, ++i)
        {
            ofs << itr->second.image_id << ";" << itr->second.u << ";" << itr->second.v << ";" << itr->second.ru << ";" << itr->second.rv << ";" << itr->second.enabled;
            if (i != int(track.views.size()) - 1)
            {
                ofs << ";";
            }
        }
        ofs << std::endl;
    }
    return true;
}

bool DBGCPList::readFromAsciiFile(const std::string &file)
{
    ifstream ifs(file);
    if(!ifs){
        LOG(INFO)<<"Can't read file "<<file;
        return false;
    }
    std::string s;
    int line = 0;
    _gcp_lsit.clear();
    bool readVersion = false;
    int version = 1;
    while (std::getline(ifs, s))
    {
        ++line;
        if (s.empty())
            continue;
        if (s[0] == '#')
            continue; //ignore #
        if (!readVersion && (sscanf(s.c_str(), "VERSION=%d", &version) == 1))
        {
            readVersion = true;
            continue;
        }
        std::string sviews;
        std::getline(ifs, sviews);

        vector<string> Xs;
        vector<string> Vs;
        split(s, ';', Xs);
        split(sviews, ';', Vs);
        if (Xs.size() != 7)
        {
            LOG(ERROR) << "Must 7 datas but " << Xs.size() << " at file " << file << ".line:" << line;
            return false;
        }
        if (Vs.size() % 6 != 0)
        {
            LOG(ERROR) << "Must 6 data per view but  at file " << file << ".line:" << line;
            return false;
        }
        for (int ii = 0; ii < Xs.size(); ++ii)
            Xs[ii] = trim(Xs[ii]);
        for (int ii = 0; ii < Vs.size(); ++ii)
            Vs[ii] = trim(Vs[ii]);
        DBGCP track;

        stringstream ss;
        track.name = Xs[0];
        ss.str(Xs[1]);
        ss >> track.type;
        ss.clear();
        ss.str(Xs[2]);
        ss >> track.enabled;
        ss.clear();
        ss.str(Xs[3]);
        ss >> track.track_id;
        ss.clear();
        ss.str(Xs[4]);
        ss >> track.landmark.x;
        ss.clear();
        ss.str(Xs[5]);
        ss >> track.landmark.y;
        ss.clear();
        ss.str(Xs[6]);
        ss >> track.landmark.z;
        ss.clear();
        int viewCount = Vs.size() / 6;
        for (int ii = 0; ii < viewCount; ++ii)
        {
            DBGCP::V v;
            ss.str(Vs[6 * ii]);
            ss >> v.image_id;
            ss.clear();
            ss.str(Vs[6 * ii + 1]);
            ss >> v.u;
            ss.clear();
            ss.str(Vs[6 * ii + 2]);
            ss >> v.v;
            ss.clear();
            ss.str(Vs[6 * ii + 3]);
            ss >> v.ru;
            ss.clear();
            ss.str(Vs[6 * ii + 4]);
            ss >> v.rv;
            ss.clear();
            ss.str(Vs[6 * ii + 5]);
            ss >> v.enabled;
            ss.clear();
            track.views[v.image_id] = v;
        }
        _gcp_lsit[track.track_id] = track;
    }
    return true;
}

std::vector<int> ImageListGenerator::importImages(
    const std::set<std::string> &imageFiles,
    const KeyType &cameraKey,
    Resource *rc, progress_fun progress)
{
    int nHave = imageList.Image_list().size();
    int imageCount = SystemConfig::instance().maxImages;
    int n = 0;
    std::vector<std::string> vecImages;
    vecImages.reserve(imageFiles.size());
    for (auto itr = imageFiles.begin(); itr != imageFiles.end(); ++itr)
    {
        vecImages.push_back(*itr);
    }
    imageList.buildPatchCache();

    if (progress)
    {
        progress(0, "Add images");
    }
    std::vector<int> addImageIds;
    int lastPercent = 0;
    int nTotalTask = vecImages.size();
    //std::vector<DBImage> dbImages;
    for (int i = 0; i < (int)vecImages.size(); ++i)
    {
        const std::string &img = vecImages[i];

        if (n + nHave > imageCount)
            break;
        if (imageList.hasImage(img))
            continue;
        DBImage image;
        image.id = rc->imageSeed.generate();
        image.image_full_path = img;
        image.camera_id = cameraKey;
        image.image_name = stlplus::filename_part(img);
        imageList.addImage(image);
        addImageIds.push_back(image.id);
        ++n;
        if (progress)
        {
            float percent = float(n) / float(nTotalTask);
            int curPercent = int(percent * 100);
            if (curPercent - lastPercent > 1)
            {
                curPercent = lastPercent;
                progress(percent, "Add images...");
            }
        }
    }
    return addImageIds;
}

void ImageListGenerator::mergePoseList(const DBPoseList &poseList)
{
    CHECK(poseList.Pose_list().size() == imageList.Image_list().size());
    auto &imgList = imageList.Image_list();
    auto &pList = poseList.Pose_list();
    for (auto itr = imgList.begin();
         itr != imgList.end(); ++itr)
    {
        CHECK(pList.find(itr->first) != pList.end());
        itr->second.pose = pList.at(itr->first);
    }
}

void DBCamera::generateHashCode()
{
    size_t seed = 0;
    hash_combine(seed, w);
    hash_combine(seed, h);
    hash_combine(seed, focalmm);
    hash_combine(seed, focal35mm);
    hash_combine(seed, Make);
    hash_combine(seed, Model);
    exif_hash_code = seed;
}

 bool saveImageNeighbors(
    const std::map<int, std::vector<int>> &neighbors,
    const std::string &file)
{
    std::ofstream ofs(file);
    if (!ofs)
    {
        LOG(ERROR) << "Can't write file " << file;
        return false;
    }
    ofs << "version=" << IMAGE_NEIGHBOR_FILE_VERSION << std::endl;
    int nImage = int(neighbors.size());
    ofs << nImage << std::endl;
    for (auto itr = neighbors.begin(); itr != neighbors.end(); ++itr)
    {
        ofs << itr->first << " " << int(itr->second.size()) << std::endl;
        for (int id : itr->second)
        {
            ofs << id << " ";
        }
        ofs << std::endl;
    }
    return true;
}

 bool readImageNeighbors(
    std::map<int, std::vector<int>> &neighbors,
    const std::string &file)
{
    std::ifstream ifs(file);
    if (!ifs)
    {
        LOG(ERROR) << "Can't read file " << file;
        return false;
    }

    std::string versionHead;
    ifs >> versionHead;
    int version = 0;
    sscanf(versionHead.c_str(), "version=%d", &version);
    if (version == 1)
    {
        int nCount = 0;
        ifs >> nCount;
        for (int i = 0; i < nCount; ++i)
        {
            int imageId, nNeighbor;
            ifs >> imageId >> nNeighbor;
            std::vector<int> ns; //neighbors
            for (int j = 0; j < nNeighbor; ++j)
            {
                int iN = 0;
                ifs >> iN;
                ns.push_back(iN);
            }
            neighbors[imageId].swap(ns);
        }
    }
    else
    {
        LOG(ERROR) << "Can't read version of" << version << " with file "
                   << file;
        return false;
    }
    return true;
}

}//name space insight
