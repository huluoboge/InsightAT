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
  double latitude = 0.0;   ///< WGS84 latitude  (degrees, NaN if unavailable)
  double longitude = 0.0;  ///< WGS84 longitude (degrees, NaN if unavailable)
  double altitude = 0.0;   ///< altitude above ellipsoid (meters, NaN if unavailable)
  bool has_gps = false;    ///< true when GPS fields are valid
  bool valid = false;
};

// Each bucket: image indices + one representative ImageExif (first valid in bucket).
// The representative is used directly by processGroupFromExif, avoiding disk re-reads.
struct BucketData {
  std::vector<size_t> indices;
  ImageExif           representative;
};
using ExifBuckets = std::unordered_map<GroupKey, BucketData, GroupKeyHash>;

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

  if (exif.doesHaveExifInfo()) {
    const auto& geo = exif.exifInfo().GeoLocation;
    // EasyExif sets Latitude/Longitude to 0 when absent; treat non-zero as valid.
    // Altitude can legitimately be 0, so we only require lat/lon to be non-zero.
    if (geo.Latitude != 0.0 || geo.Longitude != 0.0) {
      e.latitude  = geo.Latitude;
      e.longitude = geo.Longitude;
      e.altitude  = geo.Altitude;
      e.has_gps   = true;
    }
  }

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
    // Supplement sensor_width_mm without querying the DB when possible.
    // Priority: (1) derive from EXIF focal_mm (O(1), no DB scan)
    //           (2) DB loose query (O(N) linear scan) only as last resort
    {
      if (e.focal_mm > 0.1f) {
        // sw = focal_mm * width_px / f_px  — exact, no DB needed
        r.sensor_width_mm = (double)e.focal_mm * e.width / r.fx;
        LOG(INFO) << "  sensor_width_mm derived from focal_mm=" << e.focal_mm
                  << "mm: " << r.sensor_width_mm << "mm";
      } else {
        // No physical focal in EXIF → fall back to DB (O(N) scan, rare path)
        double sw_sup = 0.0;
        std::string mk_sup;
        if (CameraSensorDatabase::instance().query_sensor_width_loose(
                e.make, e.model, sw_sup, &mk_sup) && sw_sup > 0.1) {
          r.sensor_width_mm = sw_sup;
          LOG(INFO) << "  sensor_width_mm supplemented from DB: " << sw_sup
                    << "mm (matched key='" << mk_sup << "')";
        } else {
          LOG(WARNING) << "  sensor_width_mm unknown for " << e.make << " " << e.model
                       << " (no focal_mm in EXIF, DB miss)";
        }
      }
    }
    return r;
  }

  // ── Method 2: physical focal length + sensor DB ───────────────────────
  double sw = 0.0;
  std::string matched_key;
  bool in_db = CameraSensorDatabase::instance().query_sensor_width_loose(
      e.make, e.model, sw, &matched_key);
  if (!in_db) {
    LOG(WARNING) << "  sensor DB miss: make='" << e.make << "' model='" << e.model
                 << "'  (DB loaded=" << CameraSensorDatabase::instance().loaded() << ")";
  }
  if (in_db && sw > 0.1 && e.focal_mm > 0.1f) {
    r.sensor_width_mm = sw;
    r.fx = (double)e.focal_mm * e.width / sw;
    r.fy = r.fx;
    r.focal_length_35mm = (double)e.focal_mm * 36.0 / sw;
    r.source = "sensor_db";
    LOG(INFO) << "  focal estimated via sensor DB: f_mm=" << e.focal_mm << "mm, sw=" << sw
              << "mm (matched key='" << matched_key << "') → fx=" << r.fx << "px";
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
// scanGroupExif – two-phase pipeline optimised for million-scale groups
//
//   Phase 1 [multi-thread I/O, all cores]  : readExif() per image → tasks[]
//   Phase 2 [main thread, single pass]      : aggregate → ExifBuckets
//
// StageCurrent was removed:  at large N the chain queue (depth 32) caused
// Stage-1 threads to stall waiting for the single-consumer queue to drain.
// Instead we let Stage-1 run fully parallel, then do one tight serial pass
// over the filled task array.  Aggregation into a handful of camera-model
// buckets is O(N) with negligible per-item work (< 100 ms for 1 M images).
// ─────────────────────────────────────────────────────────────────────────────

static ExifBuckets scanGroupExif(const ImageGroup& group, int num_threads = 0) {
  const int n = static_cast<int>(group.images.size());
  if (n == 0) return {};

  // Use all available cores; caller can override with --split-threads.
  if (num_threads <= 0)
    num_threads = std::max(1, static_cast<int>(std::thread::hardware_concurrency()));

  // Pre-allocate task array; Stage workers access disjoint indices — no locks.
  std::vector<ExifScanTask> tasks(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    tasks[static_cast<size_t>(i)].image_idx = static_cast<size_t>(i);
    tasks[static_cast<size_t>(i)].path      = group.images[static_cast<size_t>(i)].filename;
  }

  // Queue depth: large enough that producers never stall on a full queue.
  // 4096 caps memory while giving ample headroom for num_threads workers.
  const int queue_depth = std::min(n, 4096);

  // Phase 1: parallel EXIF read — pure I/O, no downstream chain.
  Stage readExifStage(
      "ReadExif", num_threads, queue_depth,
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

  readExifStage.setTaskCount(n);

  // Push all indices from a background thread so main thread can call wait().
  std::thread push_thread([&]() {
    for (int i = 0; i < n; ++i)
      readExifStage.push(i);
  });
  readExifStage.wait(); // blocks until every worker has finished
  push_thread.join();

  // Phase 2: single-pass aggregation — O(N), no queue/lock overhead.
  // Camera models are typically < 10 distinct keys even at million scale.
  ExifBuckets buckets;
  buckets.reserve(16);
  for (int i = 0; i < n; ++i) {
    const auto& t = tasks[static_cast<size_t>(i)];
    if (!t.exif_valid) continue;
    GroupKey key;
    key.make   = t.exif.make;
    key.model  = t.exif.model;
    key.width  = t.exif.width;
    key.height = t.exif.height;
    auto& bd = buckets[key];
    bd.indices.push_back(t.image_idx);
    // Keep first valid sample as representative (focal_mm / focal_35mm already read).
    if (!bd.representative.valid)
      bd.representative = t.exif;
  }

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

// Returns new (non-primary) sub-groups each paired with their in-memory representative ImageExif.
// Also sets primary_rep_out to the representative of the largest bucket (the primary group).
static std::vector<std::pair<ImageGroup, ImageExif>>
splitGroup(Project& project,
           ImageGroup& group,
           const ExifBuckets& buckets,
           ImageExif& primary_rep_out) {
  if (buckets.size() <= 1) return {};

  // Find the largest bucket → stays as the primary group.
  const ExifBuckets::value_type* largest = nullptr;
  for (const auto& kv : buckets)
    if (!largest || kv.second.indices.size() > largest->second.indices.size())
      largest = &kv;

  std::vector<std::pair<ImageGroup, ImageExif>> new_groups;
  int sub_idx = 0;

  for (const auto& kv : buckets) {
    const bool is_primary = (&kv == largest);

    std::vector<Image> sub_images;
    sub_images.reserve(kv.second.indices.size());
    for (size_t gi : kv.second.indices)
      sub_images.push_back(group.images[gi]);

    if (is_primary) {
      group.images  = sub_images;
      primary_rep_out = kv.second.representative;
      LOG(INFO) << "  [split] sub[" << sub_idx << "] PRIMARY "
                << kv.first.make << " " << kv.first.model
                << " " << kv.first.width << "x" << kv.first.height
                << " → " << sub_images.size() << " images"
                << " (retained as group " << group.group_id << ")";
    } else {
      ImageGroup ng   = group; // copy: inherit flags/settings
      ng.group_id     = project.next_image_group_id++;
      ng.group_name   = group.group_name + "_sub" + std::to_string(sub_idx);
      ng.images       = sub_images;
      ng.group_camera = CameraModel{}; // reset; will be estimated from representative
      LOG(INFO) << "  [split] sub[" << sub_idx << "] "
                << kv.first.make << " " << kv.first.model
                << " " << kv.first.width << "x" << kv.first.height
                << " → " << sub_images.size() << " images"
                << " (new group " << ng.group_id << ")";
      new_groups.emplace_back(std::move(ng), kv.second.representative);
    }
    ++sub_idx;
  }
  return new_groups;
}

// ─────────────────────────────────────────────────────────────────────────────
// applyEstimateToGroup – write an EstimateResult into a group's CameraModel
// and emit the camera_estimator.estimate event.
// ─────────────────────────────────────────────────────────────────────────────

static void applyEstimateToGroup(ImageGroup& group,
                                 const ImageExif& representative,
                                 const EstimateResult& est,
                                 int images_sampled) {
  const uint32_t gid = group.group_id;

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

  printEvent({{"type", "camera_estimator.estimate"},
              {"ok", true},
              {"data",
               {{"group_id", gid},
                {"group_name", group.group_name},
                {"images_sampled", images_sampled},
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
// processGroupFromExif – estimate + write-back using a pre-scanned ImageExif.
// Used by the auto-split path to avoid re-reading images from disk.
// ─────────────────────────────────────────────────────────────────────────────

static void processGroupFromExif(ImageGroup& group,
                                 const ImageExif& representative,
                                 int n_images) {
  const uint32_t gid = group.group_id;
  LOG(INFO) << "Processing group " << gid << " ('" << group.group_name
            << "') from in-memory EXIF (" << n_images << " images)";

  if (!representative.valid) {
    printEvent({{"type", "camera_estimator.estimate"},
                {"ok", false},
                {"data",
                 {{"group_id", gid},
                  {"group_name", group.group_name},
                  {"error", "no valid representative EXIF for group"}}}});
    return;
  }

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

  applyEstimateToGroup(group, representative, est, n_images);
}

// ─────────────────────────────────────────────────────────────────────────────
// processGroup – estimate + write-back by sampling images from disk.
// Used when no pre-scanned EXIF is available (non-auto-split path).
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

  // ── Write back via shared helper ──────────────────────────────────────
  (void)project;
  applyEstimateToGroup(group, representative, est, sampled);
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
  // Resolve path: explicit -d > default build/data/config path > fatal error.
  if (sensor_db.empty()) {
    // Mirror what project_utils.py does: look for build-time default.
    const std::string default_rel = "data/config/camera_sensor_database.txt";
    // Search relative to the executable and to CWD.
    std::vector<std::string> candidates;
    {
      fs::path exe = fs::canonical("/proc/self/exe").parent_path();
      candidates.push_back((exe / default_rel).string());
      candidates.push_back((exe.parent_path() / default_rel).string());
      candidates.push_back(default_rel); // CWD fallback
    }
    for (const auto& c : candidates) {
      if (fs::exists(c)) { sensor_db = c; break; }
    }
  }
  if (sensor_db.empty()) {
    LOG(ERROR) << "camera_sensor_database not found. "
                  "Use -d /path/to/camera_sensor_database.txt";
    printEvent({{"type", "camera_estimator.estimate"}, {"ok", false},
                {"error", "sensor_db not found"}});
    GdalUtils::DestoryGDAL();
    return 1;
  }
  if (!CameraSensorDatabase::instance().load(sensor_db)) {
    LOG(ERROR) << "Failed to load sensor DB: " << sensor_db;
    printEvent({{"type", "camera_estimator.estimate"}, {"ok", false},
                {"error", "sensor_db load failed: " + sensor_db}});
    GdalUtils::DestoryGDAL();
    return 1;
  }
  LOG(INFO) << "Loaded sensor DB: " << sensor_db
            << " (" << CameraSensorDatabase::instance().all_sensors().size() << " entries)";

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
        // splitGroup trims grp.images to the primary bucket and fills primary_rep.
        ImageExif primary_rep;
        auto new_grps = splitGroup(project, grp, buckets, primary_rep);

        json sub_arr = json::array();
        for (const auto& [ng, _rep] : new_grps)
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

        // Estimate primary group from in-memory EXIF — no disk re-read.
        processGroupFromExif(grp, primary_rep,
                             static_cast<int>(grp.images.size()));
        ++ok_count;

        // Append new sub-groups and estimate each immediately from in-memory EXIF.
        for (auto& [ng, rep] : new_grps) {
          int n_img = static_cast<int>(ng.images.size());
          project.image_groups.push_back(std::move(ng));
          processGroupFromExif(project.image_groups.back(), rep, n_img);
          ++ok_count;
        }

      } else {
        // Single camera model — use the one bucket's representative directly.
        LOG(INFO) << "[auto-split] only 1 camera model in group " << grp.group_id
                  << " — no split needed";
        const ImageExif& rep = buckets.begin()->second.representative;
        processGroupFromExif(grp, rep, static_cast<int>(grp.images.size()));
        ++ok_count;
      }

    } else {
      // No auto-split: fall back to disk sampling.
      processGroup(project, grp, max_sample);
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
