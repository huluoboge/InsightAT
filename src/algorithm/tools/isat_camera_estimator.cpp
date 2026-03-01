/**
 * isat_camera_estimator.cpp
 * InsightAT – Per-Group Camera Intrinsics Estimator
 *
 * Reads a .iat project file, samples EXIF metadata from each group's images,
 * estimates fx / fy / cx / cy via:
 *   1. 35mm-equivalent focal length from EXIF  →  f_px = f35 * diag_px / 43.2666
 *   2. Physical focal length + sensor DB       →  f_px = f_mm * width_px / sw_mm
 *   3. Fallback: assume 35mm focal = 35 mm
 *
 * For each group the estimated CameraModel is written back into the project
 * file immediately (equivalent to isat_project set-camera).
 *
 * Usage:
 *   isat_camera_estimator -p project.iat -a
 *   isat_camera_estimator -p project.iat -g 0
 *   isat_camera_estimator -p project.iat -a -d /path/to/sensor_db.txt --max-sample 10
 */

#include <algorithm>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <glog/logging.h>
#include <nlohmann/json.hpp>

#include <cereal/archives/json.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/optional.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>

#include "Common/exif_IO_EasyExif.hpp"
#include "ImageIO/gdal_utils.h"
#include "cmdLine/cmdLine.h"
#include "cli_logging.h"
#include "database/CameraSensorDatabase.h"
#include "database/database_types.h"

namespace fs = std::filesystem;
using json   = nlohmann::json;
using namespace insight;
using namespace insight::database;

static constexpr const char* kEventPrefix = "ISAT_EVENT ";

static void printEvent(const json& j) {
    std::cout << kEventPrefix << j.dump() << "\n";
    std::cout.flush();
}

// ─────────────────────────────────────────────────────────────────────────────
// Project I/O helpers (same pattern as isat_project.cpp)
// ─────────────────────────────────────────────────────────────────────────────

static bool loadProject(const std::string& path, Project& project) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        LOG(ERROR) << "Cannot open project file: " << path;
        return false;
    }
    try {
        cereal::JSONInputArchive ar(ifs);
        ar(cereal::make_nvp("project", project));
        return true;
    } catch (const std::exception& e) {
        LOG(ERROR) << "Failed to deserialize project: " << e.what();
        return false;
    }
}

static bool saveProject(const std::string& path, const Project& project) {
    std::ofstream ofs(path);
    if (!ofs.is_open()) {
        LOG(ERROR) << "Cannot write project file: " << path;
        return false;
    }
    try {
        cereal::JSONOutputArchive ar(ofs);
        ar(cereal::make_nvp("project", project));
        return true;
    } catch (const std::exception& e) {
        LOG(ERROR) << "Failed to serialize project: " << e.what();
        return false;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Per-image EXIF data
// ─────────────────────────────────────────────────────────────────────────────

struct ImageExif {
    std::string make;
    std::string model;
    int         width        = 0;
    int         height       = 0;
    float       focal_mm     = 0.0f;   ///< physical focal length (mm)
    float       focal_35mm   = 0.0f;   ///< 35mm-equivalent focal length
    bool        valid        = false;
};

static ImageExif readExif(const std::string& path) {
    ImageExif e;

    // Image dimensions via GDAL (more reliable than EXIF for UAS imagery)
    int w = 0, h = 0;
    if (GdalUtils::GetWidthHeightPixel(path.c_str(), w, h)) {
        e.width  = w;
        e.height = h;
    } else {
        LOG(WARNING) << "GDAL: failed to read dimensions: " << path;
    }

    Exif_IO_EasyExif exif(path);
    e.make      = exif.getBrand();
    e.model     = exif.getModel();
    e.focal_mm  = exif.getFocal();
    e.focal_35mm = exif.getFocal35mm();

    // Trim surrounding whitespace
    auto trim = [](std::string& s) {
        auto first = s.find_first_not_of(" \t");
        auto last  = s.find_last_not_of(" \t");
        s = (first == std::string::npos) ? "" : s.substr(first, last - first + 1);
    };
    trim(e.make);
    trim(e.model);

    e.valid = (e.width > 0 && e.height > 0);
    return e;
}

// ─────────────────────────────────────────────────────────────────────────────
// Focal-pixel estimation from one representative EXIF sample
// ─────────────────────────────────────────────────────────────────────────────

struct EstimateResult {
    double fx          = 0.0;
    double fy          = 0.0;
    double cx          = 0.0;
    double cy          = 0.0;
    double sensor_width_mm  = 0.0;
    double focal_length_35mm = 0.0;
    std::string source;   ///< "exif_focal35mm" | "sensor_db" | "fallback"
};

static EstimateResult estimateFocal(const ImageExif& e) {
    EstimateResult r;
    if (!e.valid) return r;

    r.cx = e.width  * 0.5;
    r.cy = e.height * 0.5;

    // ── Method 1: 35mm-equivalent focal from EXIF ─────────────────────────
    if (e.focal_35mm > 0.1f) {
        double diag_px = std::sqrt((double)e.width * e.width + (double)e.height * e.height);
        r.focal_length_35mm = e.focal_35mm;
        r.fx     = r.focal_length_35mm * diag_px / 43.26661;
        r.fy     = r.fx;
        r.source = "exif_focal35mm";
        LOG(INFO) << "  focal estimated via 35mm-equiv: f35=" << e.focal_35mm
                  << "mm → fx=" << r.fx << "px";
        return r;
    }

    // ── Method 2: physical focal length + sensor DB ───────────────────────
    double sw = 0.0;
    bool in_db = CameraSensorDatabase::instance().querySensorWidth(e.make, e.model, sw);
    if (in_db && sw > 0.1 && e.focal_mm > 0.1f) {
        r.sensor_width_mm   = sw;
        r.fx                = (double)e.focal_mm * e.width / sw;
        r.fy                = r.fx;
        r.focal_length_35mm = (double)e.focal_mm * 36.0 / sw;
        r.source            = "sensor_db";
        LOG(INFO) << "  focal estimated via sensor DB: f_mm=" << e.focal_mm
                  << "mm, sw=" << sw << "mm → fx=" << r.fx << "px";
        return r;
    }

    // ── Method 3: fallback – assume 35mm focal = 35 mm ────────────────────
    double diag_px = std::sqrt((double)e.width * e.width + (double)e.height * e.height);
    r.focal_length_35mm = 35.0;
    r.fx     = 35.0 * diag_px / 43.26661;
    r.fy     = r.fx;
    r.source = "fallback";
    LOG(WARNING) << "  focal fallback (f35=35mm) → fx=" << r.fx << "px  "
                 << "[make=" << e.make << " model=" << e.model << "]";
    return r;
}

// ─────────────────────────────────────────────────────────────────────────────
// Process a single ImageGroup
// ─────────────────────────────────────────────────────────────────────────────

static void processGroup(Project& project, ImageGroup& group, int max_sample) {
    const uint32_t gid = group.group_id;
    LOG(INFO) << "Processing group " << gid << " (" << group.group_name << ") with "
              << group.images.size() << " images, sampling up to " << max_sample;

    if (group.images.empty()) {
        printEvent({{"type", "camera_estimator.estimate"},
                    {"ok",   false},
                    {"data", {{"group_id", gid}, {"group_name", group.group_name},
                              {"error", "group has no images"}}}});
        return;
    }

    // ── Sample up to max_sample images ────────────────────────────────────
    const int n_sample = std::min((int)group.images.size(), max_sample);
    ImageExif representative;
    int sampled = 0;

    for (int i = 0; i < (int)group.images.size() && sampled < n_sample; ++i) {
        const std::string& path = group.images[i].filename;
        if (!fs::exists(path)) {
            LOG(WARNING) << "Image not found (skipping): " << path;
            continue;
        }
        ImageExif e = readExif(path);
        if (!e.valid) {
            LOG(WARNING) << "Invalid EXIF/dimensions (skipping): " << path;
            continue;
        }
        // Use first valid sample as representative
        if (sampled == 0) representative = e;
        sampled++;
    }

    if (sampled == 0) {
        printEvent({{"type", "camera_estimator.estimate"},
                    {"ok",   false},
                    {"data", {{"group_id", gid}, {"group_name", group.group_name},
                              {"error", "no readable images in group"}}}});
        return;
    }

    // ── Estimate focal from representative sample ─────────────────────────
    EstimateResult est = estimateFocal(representative);

    if (est.fx <= 0.0 || representative.width == 0) {
        printEvent({{"type", "camera_estimator.estimate"},
                    {"ok",   false},
                    {"data", {{"group_id", gid}, {"group_name", group.group_name},
                              {"error", "focal estimation produced invalid result"}}}});
        return;
    }

    // ── Write back to group CameraModel ───────────────────────────────────
    CameraModel cam;
    cam.type              = CameraModel::Type::kBrownConrady;
    cam.width             = static_cast<uint32_t>(representative.width);
    cam.height            = static_cast<uint32_t>(representative.height);
    cam.focal_length      = est.fx;
    cam.aspect_ratio      = (est.fy > 0.0) ? (est.fy / est.fx) : 1.0;
    cam.principal_point_x = est.cx;
    cam.principal_point_y = est.cy;
    cam.sensor_width_mm   = est.sensor_width_mm;
    cam.focal_length_35mm = est.focal_length_35mm;
    cam.make              = representative.make;
    cam.model             = representative.model;
    cam.camera_name       = representative.make + " " + representative.model;

    group.camera_mode  = ImageGroup::CameraMode::kGroupLevel;
    group.group_camera = cam;

    // Unused: surpress warning for project param
    (void)project;

    printEvent({{"type", "camera_estimator.estimate"},
                {"ok",   true},
                {"data", {{"group_id",         gid},
                          {"group_name",        group.group_name},
                          {"images_sampled",    sampled},
                          {"width",             cam.width},
                          {"height",            cam.height},
                          {"fx",                est.fx},
                          {"fy",                est.fy},
                          {"cx",                est.cx},
                          {"cy",                est.cy},
                          {"source",            est.source},
                          {"make",              cam.make},
                          {"model",             cam.model},
                          {"sensor_width_mm",   est.sensor_width_mm},
                          {"focal_length_35mm", est.focal_length_35mm},
                          {"written_to_project",true}}}});
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    FLAGS_logtostderr = true;
    google::InitGoogleLogging(argv[0]);

    CmdLine cmd("Estimate per-group camera intrinsics from EXIF and write back to .iat project.");
    std::string project_file;
    std::string sensor_db;
    uint32_t group_id  = static_cast<uint32_t>(-1);
    int      max_sample = 5;

    cmd.add(make_option('p', project_file, "project")
        .doc("Project file (.iat)  [required]"));
    cmd.add(make_option('g', group_id, "group-id")
        .doc("Process a single group ID (mutually exclusive with -a)"));
    cmd.add(make_switch('a', "all-groups")
        .doc("Process all groups in the project"));
    cmd.add(make_option('d', sensor_db, "sensor-db")
        .doc("Path to camera sensor database (overrides built-in)"));
    cmd.add(make_option(0, max_sample, "max-sample")
        .doc("Max images to sample per group for EXIF (default: 5)"));
    std::string log_level;
    cmd.add(make_option(0, log_level, "log-level").doc("Log level: error|warn|info|debug"));
    cmd.add(make_switch('v', "verbose")
        .doc("Verbose logging (INFO level)"));
    cmd.add(make_switch('q', "quiet")
        .doc("Quiet mode (ERROR level only)"));
    cmd.add(make_switch('h', "help")
        .doc("Show this help message"));

    try { cmd.process(argc, argv); }
    catch (const std::string& s) {
        std::cerr << "Error: " << s << "\n\n";
        cmd.printHelp(std::cerr, argv[0]);
        return 2;
    }
    if (cmd.checkHelp(argv[0])) return 0;

    // ── Validate arguments ─────────────────────────────────────────────────
    const bool all_groups = cmd.used('a');
    const bool one_group  = (group_id != static_cast<uint32_t>(-1));

    if (project_file.empty()) {
        std::cerr << "Error: -p/--project is required\n\n";
        cmd.printHelp(std::cerr, argv[0]);
        return 2;
    }
    if (!all_groups && !one_group) {
        std::cerr << "Error: specify -a/--all-groups or -g/--group-id\n\n";
        cmd.printHelp(std::cerr, argv[0]);
        return 2;
    }
    if (all_groups && one_group) {
        std::cerr << "Error: -a and -g are mutually exclusive\n\n";
        return 2;
    }

    // ── Configure logging ──────────────────────────────────────────────────
    insight::tools::ApplyLogLevel(cmd.used('v'), cmd.used('q'), log_level);

    // ── Load sensor DB ─────────────────────────────────────────────────────
    if (!sensor_db.empty()) {
        CameraSensorDatabase::instance().load(sensor_db);
    }

    GdalUtils::InitGDAL();

    // ── Load project ───────────────────────────────────────────────────────
    Project project;
    if (!loadProject(project_file, project)) {
        printEvent({{"type","camera_estimator.estimate"},
                    {"ok",false},{"error","failed to load project"}});
        GdalUtils::DestoryGDAL();
        return 1;
    }

    // ── Process groups ─────────────────────────────────────────────────────
    int ok_count   = 0;
    int fail_count = 0;

    if (all_groups) {
        for (auto& grp : project.image_groups) {
            processGroup(project, grp, max_sample);
            // Count based on last event printed (side effect: always increments)
            ok_count++;
        }
    } else {
        // Locate the specified group
        ImageGroup* target = nullptr;
        for (auto& grp : project.image_groups) {
            if (grp.group_id == group_id) { target = &grp; break; }
        }
        if (!target) {
            printEvent({{"type","camera_estimator.estimate"},
                        {"ok",false},
                        {"data",{{"group_id",group_id},
                                 {"error","group not found"}}}});
            GdalUtils::DestoryGDAL();
            return 1;
        }
        processGroup(project, *target, max_sample);
        ok_count = 1;
    }

    // ── Write project back ─────────────────────────────────────────────────
    project.last_modified_time = static_cast<int64_t>(std::time(nullptr));
    if (!saveProject(project_file, project)) {
        printEvent({{"type","camera_estimator.estimate"},
                    {"ok",false},{"error","failed to write project"}});
        GdalUtils::DestoryGDAL();
        return 1;
    }

    LOG(INFO) << "camera_estimator: " << ok_count << " group(s) processed, "
              << fail_count << " failed. Project saved: " << project_file;

    GdalUtils::DestoryGDAL();
    return 0;
}
