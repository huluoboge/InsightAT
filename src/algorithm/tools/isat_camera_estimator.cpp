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
#include <atomic>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <glog/logging.h>
#include <nlohmann/json.hpp>

#include <cereal/archives/json.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/optional.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>

#include "ImageIO/gdal_utils.h"
#include "cli_logging.h"
#include "cmdLine/cmdLine.h"
#include "database/camera_sensor_database.h"
#include "database/database_types.h"
#include "io/exif/exif_IO_EasyExif.hpp"
#include "task_queue/task_queue.hpp"

namespace fs = std::filesystem;
using json = nlohmann::json;
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
// GroupKey: identifies a unique camera model for auto-split grouping.
// focal is intentionally excluded — same model may have slight EXIF focal drift.
// ─────────────────────────────────────────────────────────────────────────────

struct GroupKey {
  std::string make;
  std::string model;
  int width  = 0;
  int height = 0;

  bool operator==(const GroupKey& o) const {
    return make == o.make && model == o.model &&
           width == o.width && height == o.height;
  }
};

struct GroupKeyHash {
  size_t operator()(const GroupKey& k) const {
    // boost::hash_combine style
    auto hc = [](size_t seed, size_t v) {
      return seed ^ (v + 0x9e3779b9u + (seed << 6) + (seed >> 2));
    };
    size_t h = std::hash<std::string>{}(k.make);
    h = hc(h, std::hash<std::string>{}(k.model));
    h = hc(h, std::hash<int>{}(k.width));
    h = hc(h, std::hash<int>{}(k.height));
    return h;
  }
};

using ExifBuckets = std::unordered_map<GroupKey, std::vector<size_t>, GroupKeyHash>;

// ─────────────────────────────────────────────────────────────────────────────
// Per-image EXIF data
// ─────────────────────────────────────────────────────────────────────────────

struct ImageExif {
  std::string make;
  std::string model;
  int width = 0;
  int height = 0;
  float focal_mm = 0.0f;   ///< physical focal length (mm)
  float focal_35mm = 0.0f; ///< 35mm-equivalent focal length
  bool valid = false;
};

static ImageExif readExif(const std::string& path) {
  ImageExif e;

  // Image dimensions via GDAL (more reliable than EXIF for UAS imagery)
  int w = 0, h = 0;
  if (GdalUtils::GetWidthHeightPixel(path.c_str(), w, h)) {
    e.width = w;
    e.height = h;
  } else {
    LOG(WARNING) << "GDAL: failed to read dimensions: " << path;
  }

  Exif_IO_EasyExif exif(path);
  e.make = exif.getBrand();
  e.model = exif.getModel();
  e.focal_mm = exif.getFocal();
  e.focal_35mm = exif.getFocal35mm();

  // Trim surrounding whitespace
  auto trim = [](std::string& s) {
    auto first = s.find_first_not_of(" \t");
    auto last = s.find_last_not_of(" \t");
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
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  double sensor_width_mm = 0.0;
  double focal_length_35mm = 0.0;
  std::string source; ///< "exif_focal35mm" | "sensor_db" | "fallback"
};

static EstimateResult estimateFocal(const ImageExif& e) {
  EstimateResult r;
  if (!e.valid)
    return r;

  r.cx = e.width * 0.5;
  r.cy = e.height * 0.5;

  // ── Method 1: 35mm-equivalent focal from EXIF ─────────────────────────
  if (e.focal_35mm > 0.1f) {
    double diag_px = std::sqrt((double)e.width * e.width + (double)e.height * e.height);
    r.focal_length_35mm = e.focal_35mm;
    r.fx = r.focal_length_35mm * diag_px / 43.26661;
    r.fy = r.fx;
    r.source = "exif_focal35mm";
    LOG(INFO) << "  focal estimated via 35mm-equiv: f35=" << e.focal_35mm << "mm → fx=" << r.fx
              << "px";
    return r;
  }

  // ── Method 2: physical focal length + sensor DB ───────────────────────
  double sw = 0.0;
  bool in_db = CameraSensorDatabase::instance().query_sensor_width(e.make, e.model, sw);
  if (in_db && sw > 0.1 && e.focal_mm > 0.1f) {
    r.sensor_width_mm = sw;
    r.fx = (double)e.focal_mm * e.width / sw;
    r.fy = r.fx;
    r.focal_length_35mm = (double)e.focal_mm * 36.0 / sw;
    r.source = "sensor_db";
    LOG(INFO) << "  focal estimated via sensor DB: f_mm=" << e.focal_mm << "mm, sw=" << sw
              << "mm → fx=" << r.fx << "px";
    return r;
  }

  // ── Method 3: fallback – assume 35mm focal = 35 mm ────────────────────
  double diag_px = std::sqrt((double)e.width * e.width + (double)e.height * e.height);
  r.focal_length_35mm = 35.0;
  r.fx = 35.0 * diag_px / 43.26661;
  r.fy = r.fx;
  r.source = "fallback";
  LOG(WARNING) << "  focal fallback (f35=35mm) → fx=" << r.fx << "px  "
               << "[make=" << e.make << " model=" << e.model << "]";
  return r;
}

// ─────────────────────────────────────────────────────────────────────────────
// Per-image EXIF scan task (Stage pipeline)
// ─────────────────────────────────────────────────────────────────────────────

struct ExifScanTask {
  size_t image_idx = 0;
  std::string path;
  ImageExif   exif;
  bool        exif_valid = false;
};

// ─────────────────────────────────────────────────────────────────────────────
// scanGroupExif – 3-stage async pipeline (模仿 isat_extract)
//
//   Stage 1 [multi-thread I/O]  : readExif() per image  → ExifScanTask.exif
//   StageCurrent [main thread]  : aggregate → ExifBuckets
//
// 磁盘 IO 在后台线程池并发，主线程做无锁汇总（StageCurrent 保证单线程）。
// ─────────────────────────────────────────────────────────────────────────────

static ExifBuckets scanGroupExif(const ImageGroup& group, int num_threads = 0) {
  const int n = static_cast<int>(group.images.size());
  if (n == 0) return {};

  if (num_threads <= 0)
    num_threads = std::max(1, std::min(8,
        static_cast<int>(std::thread::hardware_concurrency())));

  // Pre-allocate task array; Stage accesses by index (no shared mutable state).
  std::vector<ExifScanTask> tasks(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    tasks[static_cast<size_t>(i)].image_idx = static_cast<size_t>(i);
    tasks[static_cast<size_t>(i)].path      = group.images[static_cast<size_t>(i)].filename;
  }

  ExifBuckets buckets;

  const int IO_QUEUE_SIZE  = 32;
  const int AGG_QUEUE_SIZE = 32;

  // Stage 1: multi-thread EXIF read (I/O bound)
  Stage readExifStage(
      "ReadExif", num_threads, IO_QUEUE_SIZE,
      [&tasks](int idx) {
        auto& t = tasks[static_cast<size_t>(idx)];
        if (!fs::exists(t.path)) {
          LOG(WARNING) << "[auto-split] image not found (skip): " << t.path;
          return;
        }
        t.exif       = readExif(t.path);
        t.exif_valid = t.exif.valid;
        if (!t.exif_valid)
          LOG(WARNING) << "[auto-split] invalid EXIF (skip): " << t.path;
      });

  // Stage 2 (StageCurrent = main thread): aggregate into buckets, no lock needed.
  StageCurrent aggregateStage(
      "AggregateExif", 1, AGG_QUEUE_SIZE,
      [&tasks, &buckets](int idx) {
        const auto& t = tasks[static_cast<size_t>(idx)];
        if (!t.exif_valid) return;
        GroupKey key;
        key.make   = t.exif.make;
        key.model  = t.exif.model;
        key.width  = t.exif.width;
        key.height = t.exif.height;
        buckets[key].push_back(t.image_idx);
      });

  chain(readExifStage, aggregateStage);
  readExifStage.setTaskCount(n);
  aggregateStage.setTaskCount(n);

  // Push indices in background; GPU/aggregate stage runs on main thread.
  std::thread push_thread([&]() {
    for (int i = 0; i < n; ++i)
      readExifStage.push(i);
  });

  aggregateStage.run(); // main thread drives Stage 2

  push_thread.join();
  readExifStage.wait();

  LOG(INFO) << "[auto-split] scanned " << n << " images in group '"
            << group.group_name << "' → " << buckets.size()
            << " distinct camera model(s)";
  return buckets;
}

// ─────────────────────────────────────────────────────────────────────────────
// splitGroup – split an ImageGroup into per-camera-model sub-groups.
// The largest bucket KEEPS the original group_id (primary camera).
// All other buckets are appended as NEW groups to project.image_groups.
// Returns the list of newly created sub-groups (does NOT append them itself).
// ─────────────────────────────────────────────────────────────────────────────

static std::vector<ImageGroup> splitGroup(Project& project,
                                          ImageGroup& group,
                                          const ExifBuckets& buckets) {
  if (buckets.size() <= 1) return {};

  // Find the largest bucket → stays as the primary group.
  const ExifBuckets::value_type* largest = nullptr;
  for (const auto& kv : buckets)
    if (!largest || kv.second.size() > largest->second.size())
      largest = &kv;

  std::vector<ImageGroup> new_groups;
  int sub_idx = 0;

  for (const auto& kv : buckets) {
    const bool is_primary = (&kv == largest);

    std::vector<Image> sub_images;
    sub_images.reserve(kv.second.size());
    for (size_t gi : kv.second)
      sub_images.push_back(group.images[gi]);

    if (is_primary) {
      group.images = sub_images;
      LOG(INFO) << "  [split] sub[" << sub_idx << "] PRIMARY "
                << kv.first.make << " " << kv.first.model
                << " " << kv.first.width << "x" << kv.first.height
                << " → " << sub_images.size() << " images"
                << " (retained as group " << group.group_id << ")";
    } else {
      ImageGroup ng     = group; // copy: inherit flags/settings
      ng.group_id       = project.next_image_group_id++;
      ng.group_name     = group.group_name + "_sub" + std::to_string(sub_idx);
      ng.images         = sub_images;
      ng.group_camera   = CameraModel{}; // reset; processGroup will estimate
      LOG(INFO) << "  [split] sub[" << sub_idx << "] "
                << kv.first.make << " " << kv.first.model
                << " " << kv.first.width << "x" << kv.first.height
                << " → " << sub_images.size() << " images"
                << " (new group " << ng.group_id << ")";
      new_groups.push_back(std::move(ng));
    }
    ++sub_idx;
  }
  return new_groups;
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
                {"ok", false},
                {"data",
                 {{"group_id", gid},
                  {"group_name", group.group_name},
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
    if (sampled == 0)
      representative = e;
    sampled++;
  }

  if (sampled == 0) {
    printEvent({{"type", "camera_estimator.estimate"},
                {"ok", false},
                {"data",
                 {{"group_id", gid},
                  {"group_name", group.group_name},
                  {"error", "no readable images in group"}}}});
    return;
  }

  // ── Estimate focal from representative sample ─────────────────────────
  EstimateResult est = estimateFocal(representative);

  if (est.fx <= 0.0 || representative.width == 0) {
    printEvent({{"type", "camera_estimator.estimate"},
                {"ok", false},
                {"data",
                 {{"group_id", gid},
                  {"group_name", group.group_name},
                  {"error", "focal estimation produced invalid result"}}}});
    return;
  }

  // ── Write back to group CameraModel ───────────────────────────────────
  CameraModel cam;
  cam.type = CameraModel::Type::kBrownConrady;
  cam.width = static_cast<uint32_t>(representative.width);
  cam.height = static_cast<uint32_t>(representative.height);
  cam.focal_length = est.fx;
  cam.aspect_ratio = (est.fy > 0.0) ? (est.fy / est.fx) : 1.0;
  cam.principal_point_x = est.cx;
  cam.principal_point_y = est.cy;
  cam.sensor_width_mm = est.sensor_width_mm;
  cam.focal_length_35mm = est.focal_length_35mm;
  cam.make = representative.make;
  cam.model = representative.model;
  cam.camera_name = representative.make + " " + representative.model;

  group.camera_mode = ImageGroup::CameraMode::kGroupLevel;
  group.group_camera = cam;

  // Unused: surpress warning for project param
  (void)project;

  printEvent({{"type", "camera_estimator.estimate"},
              {"ok", true},
              {"data",
               {{"group_id", gid},
                {"group_name", group.group_name},
                {"images_sampled", sampled},
                {"width", cam.width},
                {"height", cam.height},
                {"fx", est.fx},
                {"fy", est.fy},
                {"cx", est.cx},
                {"cy", est.cy},
                {"source", est.source},
                {"make", cam.make},
                {"model", cam.model},
                {"sensor_width_mm", est.sensor_width_mm},
                {"focal_length_35mm", est.focal_length_35mm},
                {"written_to_project", true}}}});
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
  uint32_t group_id = static_cast<uint32_t>(-1);
  int max_sample = 5;

  cmd.add(make_option('p', project_file, "project").doc("Project file (.iat)  [required]"));
  cmd.add(make_option('g', group_id, "group-id")
              .doc("Process a single group ID (mutually exclusive with -a)"));
  cmd.add(make_switch('a', "all-groups").doc("Process all groups in the project"));
  cmd.add(make_option('d', sensor_db, "sensor-db")
              .doc("Path to camera sensor database (overrides built-in)"));
  cmd.add(make_option(0, max_sample, "max-sample")
              .doc("Max images to sample per group for EXIF (default: 5)"));
  cmd.add(make_switch(0, "auto-split")
              .doc("Scan all images in each group; split into sub-groups when multiple camera "
                   "models (make/model/w/h) are detected"));
  int split_threads = 0;
  cmd.add(make_option(0, split_threads, "split-threads")
              .doc("Thread count for parallel EXIF scan in --auto-split (0 = auto, max 8)"));
  std::string log_level;
  cmd.add(make_option(0, log_level, "log-level").doc("Log level: error|warn|info|debug"));
  cmd.add(make_switch('v', "verbose").doc("Verbose logging (INFO level)"));
  cmd.add(make_switch('q', "quiet").doc("Quiet mode (ERROR level only)"));
  cmd.add(make_switch('h', "help").doc("Show this help message"));

  try {
    cmd.process(argc, argv);
  } catch (const std::string& s) {
    std::cerr << "Error: " << s << "\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 2;
  }
  if (cmd.checkHelp(argv[0]))
    return 0;

  // ── Validate arguments ─────────────────────────────────────────────────
  const bool all_groups  = cmd.used('a');
  const bool one_group   = (group_id != static_cast<uint32_t>(-1));
  const bool auto_split  = cmd.used("auto-split");

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
  insight::tools::apply_log_level(cmd.used('v'), cmd.used('q'), log_level);

  // ── Load sensor DB ─────────────────────────────────────────────────────
  if (!sensor_db.empty()) {
    CameraSensorDatabase::instance().load(sensor_db);
  }

  GdalUtils::InitGDAL();

  // ── Load project ───────────────────────────────────────────────────────
  Project project;
  if (!loadProject(project_file, project)) {
    printEvent({{"type", "camera_estimator.estimate"},
                {"ok", false},
                {"error", "failed to load project"}});
    GdalUtils::DestoryGDAL();
    return 1;
  }

  // ── Process groups ─────────────────────────────────────────────────────
  // Collect original target indices first (auto-split may append new groups).
  std::vector<size_t> target_indices;
  if (all_groups) {
    for (size_t i = 0; i < project.image_groups.size(); ++i)
      target_indices.push_back(i);
  } else {
    for (size_t i = 0; i < project.image_groups.size(); ++i) {
      if (project.image_groups[i].group_id == group_id) {
        target_indices.push_back(i);
        break;
      }
    }
    if (target_indices.empty()) {
      printEvent({{"type", "camera_estimator.estimate"},
                  {"ok", false},
                  {"data", {{"group_id", group_id}, {"error", "group not found"}}}});
      GdalUtils::DestoryGDAL();
      return 1;
    }
  }

  int ok_count   = 0;
  int fail_count = 0;

  for (size_t gi : target_indices) {
    // Use index not pointer — splitGroup push_backs and would invalidate refs.
    ImageGroup& grp = project.image_groups[gi];

    // ── auto-split: Stage pipeline scans all images, buckets by camera model ──
    if (auto_split && grp.images.size() > 1) {
      ExifBuckets buckets = scanGroupExif(grp, split_threads);

      if (buckets.size() > 1) {
        std::vector<ImageGroup> new_grps = splitGroup(project, grp, buckets);

        json sub_arr = json::array();
        for (const auto& ng : new_grps)
          sub_arr.push_back({{"group_id",   ng.group_id},
                             {"group_name", ng.group_name},
                             {"n_images",   ng.images.size()}});

        printEvent({{"type", "camera_estimator.split"},
                    {"ok",   true},
                    {"data",
                     {{"source_group_id",    grp.group_id},
                      {"source_group_name",  grp.group_name},
                      {"sub_groups_created", new_grps.size()},
                      {"new_groups",         sub_arr}}}});

        // Append new sub-groups; they are processed after this loop.
        for (auto& ng : new_grps)
          project.image_groups.push_back(std::move(ng));

      } else {
        LOG(INFO) << "[auto-split] only 1 camera model in group " << grp.group_id
                  << " — no split needed";
      }
    }

    // Estimate intrinsics for the (possibly trimmed) primary group.
    processGroup(project, grp, max_sample);
    ++ok_count;
  }

  // Process newly created sub-groups (always appended to the end).
  if (auto_split && !target_indices.empty()) {
    const size_t first_new = *std::max_element(target_indices.begin(), target_indices.end()) + 1;
    for (size_t i = first_new; i < project.image_groups.size(); ++i) {
      processGroup(project, project.image_groups[i], max_sample);
      ++ok_count;
    }
  }

  // ── Write project back ─────────────────────────────────────────────────
  project.last_modified_time = static_cast<int64_t>(std::time(nullptr));
  if (!saveProject(project_file, project)) {
    printEvent({{"type", "camera_estimator.estimate"},
                {"ok", false},
                {"error", "failed to write project"}});
    GdalUtils::DestoryGDAL();
    return 1;
  }

  LOG(INFO) << "camera_estimator: " << ok_count << " group(s) processed, " << fail_count
            << " failed. Project saved: " << project_file;

  GdalUtils::DestoryGDAL();
  return 0;
}
