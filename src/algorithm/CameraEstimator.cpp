#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <cmath>
#include <memory>
#include <glog/logging.h>
#include <cereal/archives/json.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/string.hpp>

#include "Common/exif_IO_EasyExif.hpp"
#include "ImageIO/gdal_utils.h"
#include "database/CameraSensorDatabase.h"

using namespace insight;
using namespace insight::database;

// ─────────────────────────────────────────────────────────────
// JSON IPC 数据结构
// ─────────────────────────────────────────────────────────────

struct EstimatorInput {
    std::vector<std::string> image_paths;
    std::string sensor_db_path;
    std::string log_dir;
    
    template<class Archive>
    void serialize(Archive& ar) {
        ar(cereal::make_nvp("image_paths", image_paths),
           cereal::make_nvp("sensor_db_path", sensor_db_path),
           cereal::make_nvp("log_dir", log_dir));
    }
};

struct CameraResult {
    std::string make;
    std::string model;
    int width = 0;
    int height = 0;
    double sensor_width_mm = 0.0;
    double focal_length_px = 0.0;
    double focal_length_35mm = 0.0;

    template<class Archive>
    void serialize(Archive& ar) {
        ar(cereal::make_nvp("make", make),
           cereal::make_nvp("model", model),
           cereal::make_nvp("width", width),
           cereal::make_nvp("height", height),
           cereal::make_nvp("sensor_width_mm", sensor_width_mm),
           cereal::make_nvp("focal_length_px", focal_length_px),
           cereal::make_nvp("focal_length_35mm", focal_length_35mm));
    }
};

struct GroupResult {
    CameraResult camera;
    std::vector<int> image_indices;

    template<class Archive>
    void serialize(Archive& ar) {
        ar(cereal::make_nvp("camera", camera),
           cereal::make_nvp("image_indices", image_indices));
    }
};

struct EstimatorOutput {
    std::vector<GroupResult> groups;

    template<class Archive>
    void serialize(Archive& ar) {
        ar(cereal::make_nvp("groups", groups));
    }
};

// ─────────────────────────────────────────────────────────────
// 辅助函数
// ─────────────────────────────────────────────────────────────

struct ImageMetadata {
    std::string make;
    std::string model;
    int width = 0;
    int height = 0;
    float focal_length = 0.0f;
    float focal_35mm = 0.0f;
};

// 分组键：Brand + Model + Width + Height
struct GroupKey {
    std::string make;
    std::string model;
    int width;
    int height;

    bool operator<(const GroupKey& other) const {
        if (make != other.make) return make < other.make;
        if (model != other.model) return model < other.model;
        if (width != other.width) return width < other.width;
        return height < other.height;
    }
};

int main(int argc, char* argv[]) {
    // 强制输出到 stderr，因为 stdout 用于 JSON IPC
    FLAGS_logtostderr = true;
    google::InitGoogleLogging(argv[0]);

    // 从 stdin 读取输入 JSON
    EstimatorInput input;
    try {
        cereal::JSONInputArchive archive(std::cin);
        archive(cereal::make_nvp("estimator_input", input));
    } catch (const std::exception& e) {
        LOG(ERROR) << "Failed to parse input JSON: " << e.what();
        return 1;
    }

    if (!input.log_dir.empty()) {
        FLAGS_log_dir = input.log_dir;
        FLAGS_logtostderr = false;
        FLAGS_alsologtostderr = true;
    }

    LOG(INFO) << "Starting CameraEstimator for " << input.image_paths.size() << " images";

    // 加载传感器数据库
    if (!input.sensor_db_path.empty()) {
        CameraSensorDatabase::instance().load(input.sensor_db_path);
    }

    GdalUtils::InitGDAL();

    std::map<GroupKey, std::vector<int>> groupedImages;
    std::map<GroupKey, ImageMetadata> groupMetadata;

    for (int i = 0; i < (int)input.image_paths.size(); ++i) {
        const std::string& path = input.image_paths[i];
        ImageMetadata meta;
        
        // 1. 获取宽和高 (GDAL)
        if (!GdalUtils::GetWidthHeightPixel(path.c_str(), meta.width, meta.height)) {
            LOG(WARNING) << "Failed to get image size via GDAL: " << path;
        }

        // 2. 获取 EXIF 信息
        Exif_IO_EasyExif exif(path);
        meta.make = exif.getBrand();
        meta.model = exif.getModel();
        meta.focal_length = exif.getFocal();
        meta.focal_35mm = exif.getFocal35mm(); 

        // 纠正 Make/Model 中的空格
        auto trim = [](std::string& s) {
            s.erase(0, s.find_first_not_of(" "));
            s.erase(s.find_last_not_of(" ") + 1);
        };
        trim(meta.make);
        trim(meta.model);

        GroupKey key{meta.make, meta.model, meta.width, meta.height};
        groupedImages[key].push_back(i);
        
        // 记录该组的一个代表性 metadata
        if (groupMetadata.find(key) == groupMetadata.end()) {
            groupMetadata[key] = meta;
        }
    }

    EstimatorOutput output;

    for (auto const& [key, indices] : groupedImages) {
        const auto& meta = groupMetadata[key];
        GroupResult group;
        group.image_indices = indices;
        
        CameraResult& cam = group.camera;
        cam.make = key.make;
        cam.model = key.model;
        cam.width = key.width;
        cam.height = key.height;

        // 检索传感器数据库
        double sensor_width = 0.0;
        bool foundInDb = CameraSensorDatabase::instance().querySensorWidth(cam.make, cam.model, sensor_width);
        if (foundInDb) {
            cam.sensor_width_mm = sensor_width;
        }

        // 估计内参 (仅 Focal Pixel)
        // 使用 Diagonal FOV 估算法
        // 1. 如果有 35mm 等效焦距
        // f_px = f_35mm * sqrt(w^2 + h^2) / 43.2666
        if (meta.focal_35mm > 0.1f) {
            cam.focal_length_35mm = meta.focal_35mm;
            double diagonal_px = std::sqrt((double)cam.width * cam.width + (double)cam.height * cam.height);
            cam.focal_length_px = cam.focal_length_35mm * diagonal_px / 43.26661;
        } 
        // 2. 否则如果有传感器宽度和物理焦距
        // f_px = f_mm * width_px / sensor_width_mm
        else if (sensor_width > 0.1 && meta.focal_length > 0.1f) {
            cam.focal_length_px = (double)meta.focal_length * cam.width / sensor_width;
            // 估计 35mm 等效焦距用于显示
            cam.focal_length_35mm = (double)meta.focal_length * 36.0 / sensor_width;
        }
        // 3. 保底方案 (默认 35mm 等效为 35mm)
        else {
            cam.focal_length_35mm = 35.0;
            double diagonal_px = std::sqrt((double)cam.width * cam.width + (double)cam.height * cam.height);
            cam.focal_length_px = 35.0 * diagonal_px / 43.26661;
        }

        output.groups.push_back(group);
        LOG(INFO) << "Estimated for group " << cam.make << " " << cam.model << ": f=" << cam.focal_length_px << "px";
    }

    // 输出 JSON 到 stdout
    try {
        cereal::JSONOutputArchive archive(std::cout);
        archive(cereal::make_nvp("estimator_output", output));
    } catch (const std::exception& e) {
        LOG(ERROR) << "Failed to generate output JSON: " << e.what();
        return 1;
    }

    GdalUtils::DestoryGDAL();
    return 0;
}
