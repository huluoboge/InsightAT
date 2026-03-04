/**
 * isat_project.cpp
 * InsightAT Project CLI – create and manage .iat projects and AT tasks
 *
 * Subcommands: create, add-group, set-cs, inspect, create-at-task, etc.
 * Operates on project files (.iat) and optionally exports camera/intrinsics
 * JSON for use with isat_geo -k / isat_twoview -k.
 *
 * Usage:
 *   isat_project create -p project.iat
 *   isat_project add-group -p project.iat -n "Group1" --image-list images.txt
 *   isat_project inspect -p project.iat
 *   isat_project create-at-task -p project.iat -t 1 -o task.json
 */

#include <algorithm>
#include <cereal/archives/json.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/optional.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <glog/logging.h>
#include <iostream>
#include <nlohmann/json.hpp>
#include <random>
#include <regex>
#include <sstream>
#include <unordered_set>

#include "../database/database_types.h"
#include "cli_logging.h"
#include "cmdLine/cmdLine.h"

namespace fs = std::filesystem;
using json = nlohmann::json;
using namespace insight::database;

static constexpr const char* kEventPrefix = "ISAT_EVENT ";

static void printEvent(const json& j) {
  // Machine-readable, single-line JSON on stdout.
  std::cout << kEventPrefix << j.dump() << "\n";
  std::cout.flush();
}

static std::string generateUUIDv4() {
  // Minimal UUIDv4 generator (no Qt/Boost dependency in algorithm tools).
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<int> dis(0, 15);
  std::uniform_int_distribution<int> dis_y(8, 11); // 8..b

  auto hex = [](int v) -> char { return "0123456789abcdef"[v & 0xF]; };
  std::string s = "xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx";
  for (char& c : s) {
    if (c == 'x')
      c = hex(dis(gen));
    else if (c == 'y')
      c = hex(dis_y(gen));
  }
  return s;
}

static bool loadProjectFromFile(const std::string& filepath, Project& project) {
  std::ifstream ifs(filepath); // Text mode for JSON
  if (!ifs.is_open()) {
    LOG(ERROR) << "Failed to open project file: " << filepath;
    return false;
  }

  try {
    cereal::JSONInputArchive archive(ifs);
    archive(cereal::make_nvp("project", project));
    return true;
  } catch (const std::exception& e) {
    LOG(ERROR) << "Failed to deserialize project: " << e.what();
    return false;
  }
}

static bool saveProjectToFile(const std::string& filepath, const Project& project) {
  std::ofstream ofs(filepath); // Text mode for JSON (consistent with UI ProjectDocument)
  if (!ofs.is_open()) {
    LOG(ERROR) << "Failed to write project file: " << filepath;
    return false;
  }
  try {
    cereal::JSONOutputArchive archive(ofs);
    archive(cereal::make_nvp("project", project));
    return true;
  } catch (const std::exception& e) {
    LOG(ERROR) << "Failed to serialize project: " << e.what();
    return false;
  }
}

static void printTopLevelHelp(const char* argv0) {
  std::cerr
      << "InsightAT Project Tool\n\n"
      << "Usage:\n"
      << "  " << argv0 << " create     [options]\n"
      << "  " << argv0 << " add-group  [options]\n"
      << "  " << argv0 << " add-images [options]\n"
      << "  " << argv0 << " set-camera [options]\n"
      << "  " << argv0 << " set-cs     [options]\n"
      << "  " << argv0 << " inspect    [options]\n"
      << "  " << argv0 << " ls         [options]\n"
      << "  " << argv0 << " create-at-task [options]\n"
      << "  " << argv0 << " delete-at-task [options]\n"
      << "  " << argv0 << " extract    [options]\n"
      << "  " << argv0 << " intrinsics [options]\n\n"
      << "Commands:\n"
      << "  create         Create a new project (.iat) with default Local coordinate system\n"
      << "  add-group      Add an image group to the project (prints group_id)\n"
      << "  add-images     Import images into a group (prints counts)\n"
      << "  set-camera     Set group camera intrinsics / resolution\n"
      << "  set-cs         Set project input coordinate system\n"
      << "  inspect        Print project/task JSON to stdout\n"
      << "  ls             List all AT tasks (task_id / uuid / name)\n"
      << "  create-at-task Create an AT task (snapshot of current project; supports nesting)\n"
      << "  delete-at-task Delete an AT task by task_id\n"
      << "  extract        Export image list JSON for isat_extract\n"
      << "  intrinsics     Export intrinsics JSON for isat_geo\n\n"
      << "Run each command with -h/--help for detailed options.\n";
}

static const ATTask* findTaskById(const Project& project, uint32_t task_id) {
  for (const auto& task : project.at_tasks) {
    if (task.task_id == task_id)
      return &task;
  }
  return nullptr;
}

static std::vector<const ImageGroup*> selectGroups(const std::vector<ImageGroup>& groups,
                                                   bool all_groups, int target_group_id) {
  std::vector<const ImageGroup*> selected;

  if (all_groups) {
    for (const auto& group : groups)
      selected.push_back(&group);
    return selected;
  }

  if (target_group_id >= 0) {
    for (const auto& group : groups) {
      if (group.group_id == static_cast<uint32_t>(target_group_id)) {
        selected.push_back(&group);
        break;
      }
    }
    return selected;
  }

  if (!groups.empty())
    selected.push_back(&groups[0]);
  return selected;
}

static json taskToSummaryJson(const ATTask& task) {
  json out;
  out["task_id"] = task.task_id;
  out["uuid"] = task.id;
  out["task_name"] = task.task_name;
  out["image_groups"] = task.input_snapshot.image_groups.size();

  size_t total_images = 0;
  for (const auto& g : task.input_snapshot.image_groups) {
    total_images += g.images.size();
  }
  out["images"] = total_images;
  out["measurements"] = task.input_snapshot.measurements.size();
  out["has_initialization"] = task.initialization.has_value();
  out["optimized_poses"] = task.optimized_poses.size();
  return out;
}

static ImageGroup* findProjectGroup(Project& project, uint32_t group_id) {
  for (auto& g : project.image_groups) {
    if (g.group_id == group_id)
      return &g;
  }
  return nullptr;
}

static int runCreate(int argc, char* argv[]) {
  CmdLine cmd("Create a new project (.iat) with default Local coordinate system");
  std::string project_file;
  std::string name;
  std::string author = "unknown";
  std::string description;

  cmd.add(make_option('p', project_file, "project").doc("Output project file (.iat)"));
  cmd.add(make_option('n', name, "name").doc("Project name (optional)"));
  cmd.add(make_option('a', author, "author").doc("Author/operator (optional, default: unknown)"));
  cmd.add(make_option('d', description, "description").doc("Description (optional)"));
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
  if (project_file.empty()) {
    std::cerr << "Error: -p/--project is required\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 2;
  }

  insight::tools::apply_log_level(cmd.used('v'), cmd.used('q'), log_level);

  Project project;
  project.name = name.empty() ? fs::path(project_file).stem().string() : name;
  project.author = author;
  project.description = description;
  project.uuid = generateUUIDv4();
  const auto now = static_cast<int64_t>(std::time(nullptr));
  project.creation_time = now;
  project.last_modified_time = now;
  project.project_version = "1.0";

  project.input_coordinate_system.type = CoordinateSystem::Type::kLocal;
  project.input_coordinate_system.definition = "Local";

  // Note: We intentionally do NOT enforce Project::is_valid here, because groups/cameras
  // may be added later and the validation policy may be stricter than the creation workflow.

  if (!saveProjectToFile(project_file, project)) {
    printEvent({{"type", "project.create"}, {"ok", false}, {"error", "failed to write project"}});
    return 1;
  }

  printEvent({{"type", "project.create"},
              {"ok", true},
              {"data",
               {{"project_path", project_file},
                {"name", project.name},
                {"uuid", project.uuid},
                {"coordinate_system", "Local"}}}});
  return 0;
}

static int runAddGroup(int argc, char* argv[]) {
  CmdLine cmd("Add an image group to the project (.iat). Prints group_id.");
  std::string project_file;
  std::string group_name;
  std::string camera_mode = "group"; // group|image

  cmd.add(make_option('p', project_file, "project").doc("Project file (.iat)"));
  cmd.add(make_option('n', group_name, "name").doc("Group name"));
  cmd.add(
      make_option(0, camera_mode, "camera-mode").doc("Camera mode: group|image (default: group)"));
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
  if (project_file.empty() || group_name.empty()) {
    std::cerr << "Error: -p/--project and -n/--name are required\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 2;
  }

  insight::tools::apply_log_level(cmd.used('v'), cmd.used('q'), log_level);

  Project project;
  if (!loadProjectFromFile(project_file, project)) {
    printEvent({{"type", "project.add_group"}, {"ok", false}, {"error", "failed to load project"}});
    return 1;
  }

  ImageGroup group;
  group.group_id = project.next_image_group_id++;
  group.group_name = group_name;
  group.creation_time = static_cast<int64_t>(std::time(nullptr));
  if (camera_mode == "image")
    group.camera_mode = ImageGroup::CameraMode::kImageLevel;
  else
    group.camera_mode = ImageGroup::CameraMode::kGroupLevel;

  project.image_groups.push_back(group);
  project.last_modified_time = static_cast<int64_t>(std::time(nullptr));

  if (!saveProjectToFile(project_file, project)) {
    printEvent(
        {{"type", "project.add_group"}, {"ok", false}, {"error", "failed to write project"}});
    return 1;
  }

  printEvent({{"type", "project.add_group"},
              {"ok", true},
              {"data",
               {{"group_id", group.group_id},
                {"group_name", group.group_name},
                {"camera_mode", camera_mode}}}});
  return 0;
}

// Build a set of canonical absolute paths for all images already in the project.
// Used for project-wide deduplication when importing new images.
static std::unordered_set<std::string> buildExistingPathSet(const Project& project) {
  std::unordered_set<std::string> s;
  for (const auto& grp : project.image_groups) {
    for (const auto& img : grp.images) {
      std::error_code ec;
      auto canon = fs::canonical(fs::path(img.filename), ec);
      s.insert(ec ? fs::absolute(img.filename).string() : canon.string());
    }
  }
  return s;
}

static int runAddImages(int argc, char* argv[]) {
  CmdLine cmd("Import images into a project group. Writes filenames only (no image decoding).");
  std::string project_file;
  uint32_t group_id = static_cast<uint32_t>(-1);
  std::string input_dir;
  std::string ext = ".jpg";
  bool recursive = false;

  cmd.add(make_option('p', project_file, "project").doc("Project file (.iat)"));
  cmd.add(make_option('g', group_id, "group-id").doc("Target group ID"));
  cmd.add(make_option('i', input_dir, "input").doc("Input directory containing images"));
  cmd.add(make_option(0, ext, "ext").doc("File extension filter (default: .jpg)"));
  cmd.add(make_switch('r', "recursive").doc("Recursively scan input directory"));
  cmd.add(make_switch(0, "no-dedup").doc("Do not deduplicate by filename"));
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
  if (project_file.empty() || input_dir.empty() || group_id == static_cast<uint32_t>(-1)) {
    std::cerr << "Error: -p/--project, -g/--group-id and -i/--input are required\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 2;
  }

  insight::tools::apply_log_level(cmd.used('v'), cmd.used('q'), log_level);
  recursive = cmd.used('r');
  const bool dedup = !cmd.used("no-dedup");

  if (!fs::is_directory(input_dir)) {
    printEvent(
        {{"type", "project.add_images"}, {"ok", false}, {"error", "input directory not found"}});
    return 1;
  }

  Project project;
  if (!loadProjectFromFile(project_file, project)) {
    printEvent(
        {{"type", "project.add_images"}, {"ok", false}, {"error", "failed to load project"}});
    return 1;
  }

  ImageGroup* group = findProjectGroup(project, group_id);
  if (!group) {
    printEvent({{"type", "project.add_images"}, {"ok", false}, {"error", "group not found"}});
    return 1;
  }

  int scanned = 0;
  int added = 0;
  int skipped = 0; // dedup skips

  // ── Pre-process --ext into a matcher ─────────────────────────────────
  // Supported forms:
  //   Single extension  : .jpg
  //   Comma-separated   : .jpg,.tif,.png
  //   Regex pattern     : \.(jpg|tif|png|tiff)$   (detected by special chars)
  auto toLower = [](std::string s) -> std::string {
    for (auto& c : s)
      c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
  };

  // Detect regex: contains any of ( | * + [ { ^ $
  const bool ext_is_regex = (ext.find_first_of("(|*+[{^$") != std::string::npos);

  std::regex ext_re;
  std::vector<std::string> ext_list; // lower-cased

  if (ext_is_regex) {
    try {
      ext_re = std::regex(ext, std::regex_constants::icase);
    } catch (const std::regex_error& e) {
      std::cerr << "Error: --ext regex is invalid: " << e.what() << "\n";
      return 2;
    }
  } else {
    // Split by comma
    std::istringstream iss(ext);
    std::string tok;
    while (std::getline(iss, tok, ',')) {
      // trim surrounding whitespace
      auto first = tok.find_first_not_of(" \t");
      auto last = tok.find_last_not_of(" \t");
      if (first == std::string::npos)
        continue;
      ext_list.push_back(toLower(tok.substr(first, last - first + 1)));
    }
    if (ext_list.empty()) {
      std::cerr << "Error: --ext value is empty\n";
      return 2;
    }
  }

  auto accept = [&](const fs::path& p) -> bool {
    if (ext_is_regex) {
      return std::regex_search(toLower(p.filename().string()), ext_re);
    }
    if (!p.has_extension())
      return false;
    const std::string e = toLower(p.extension().string());
    for (const auto& ex : ext_list)
      if (e == ex)
        return true;
    return false;
  };

  // ── Build project-wide dedup set (canonical absolute paths) ──────────
  std::unordered_set<std::string> existing_paths;
  if (dedup) {
    existing_paths = buildExistingPathSet(project);
  }

  auto addOne = [&](const fs::path& p) {
    scanned++;
    std::error_code ec;
    const auto abs_path = fs::absolute(p);
    const auto canon_path = fs::canonical(p, ec);
    const std::string key = ec ? abs_path.string() : canon_path.string();
    const std::string fn = abs_path.string(); // store as absolute path

    if (dedup && existing_paths.count(key)) {
      skipped++;
      return;
    }

    Image img;
    img.image_id = project.next_image_id++;
    img.filename = fn;
    group->images.push_back(std::move(img));
    if (dedup)
      existing_paths.insert(key); // intra-batch dedup too
    added++;
  };

  if (recursive) {
    for (auto it = fs::recursive_directory_iterator(input_dir);
         it != fs::recursive_directory_iterator(); ++it) {
      if (!it->is_regular_file())
        continue;
      if (!accept(it->path()))
        continue;
      addOne(it->path());
    }
  } else {
    for (auto it = fs::directory_iterator(input_dir); it != fs::directory_iterator(); ++it) {
      if (!it->is_regular_file())
        continue;
      if (!accept(it->path()))
        continue;
      addOne(it->path());
    }
  }

  project.last_modified_time = static_cast<int64_t>(std::time(nullptr));
  if (!saveProjectToFile(project_file, project)) {
    printEvent(
        {{"type", "project.add_images"}, {"ok", false}, {"error", "failed to write project"}});
    return 1;
  }

  printEvent({{"type", "project.add_images"},
              {"ok", true},
              {"data",
               {{"group_id", group_id},
                {"scanned", scanned},
                {"added", added},
                {"skipped", skipped},
                {"total_in_group", static_cast<int>(group->images.size())}}}});
  return 0;
}

static int runSetCamera(int argc, char* argv[]) {
  CmdLine cmd("Set group camera (resolution + intrinsics) for GroupLevel mode.");
  std::string project_file;
  std::string from_k_file; // --from-k: load fx/fy/cx/cy from isat_calibrate K.json
  uint32_t group_id = static_cast<uint32_t>(-1);
  uint32_t width = 0, height = 0;
  double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
  double aspect_ratio = 1.0;
  std::string camera_name;

  cmd.add(make_option('p', project_file, "project").doc("Project file (.iat)"));
  cmd.add(make_option('g', group_id, "group-id").doc("Target group ID"));
  cmd.add(make_option(0, from_k_file, "from-k")
              .doc("Load fx/fy/cx/cy from isat_calibrate K.json (replaces "
                   "--fx/--fy/--cx/--cy/--width/--height)"));
  cmd.add(make_option(0, width, "width").doc("Image width (pixels)"));
  cmd.add(make_option(0, height, "height").doc("Image height (pixels)"));
  cmd.add(make_option(0, fx, "fx").doc("Focal length fx (pixels)"));
  cmd.add(make_option(0, fy, "fy").doc("Focal length fy (pixels). If 0, uses fx."));
  cmd.add(make_option(0, cx, "cx").doc("Principal point cx (pixels)"));
  cmd.add(make_option(0, cy, "cy").doc("Principal point cy (pixels)"));
  cmd.add(make_option(0, aspect_ratio, "aspect").doc("Aspect ratio fy/fx (optional, default 1.0)"));
  cmd.add(make_option(0, camera_name, "camera-name").doc("Camera name (optional)"));
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
  if (project_file.empty() || group_id == static_cast<uint32_t>(-1)) {
    std::cerr << "Error: -p/--project and -g/--group-id are required\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 2;
  }

  // ── Load intrinsics from K.json if --from-k is given ──────────────────
  if (!from_k_file.empty()) {
    std::ifstream kfs(from_k_file);
    if (!kfs.is_open()) {
      std::cerr << "Error: cannot open --from-k file: " << from_k_file << "\n";
      return 2;
    }
    try {
      json kj;
      kfs >> kj;
      fx = kj.value("fx", 0.0);
      fy = kj.value("fy", 0.0);
      cx = kj.value("cx", 0.0);
      cy = kj.value("cy", 0.0);
      if (kj.contains("width") && width == 0)
        width = kj["width"];
      if (kj.contains("height") && height == 0)
        height = kj["height"];
    } catch (const std::exception& e) {
      std::cerr << "Error: failed to parse --from-k JSON: " << e.what() << "\n";
      return 2;
    }
    if (fx <= 0.0) {
      std::cerr << "Error: --from-k file contains invalid fx (<= 0)\n";
      return 2;
    }
  }

  if (width == 0 || height == 0 || fx <= 0.0) {
    std::cerr << "Error: --width, --height, --fx are required (or use --from-k)\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 2;
  }

  insight::tools::apply_log_level(cmd.used('v'), cmd.used('q'), log_level);

  Project project;
  if (!loadProjectFromFile(project_file, project)) {
    printEvent(
        {{"type", "project.set_camera"}, {"ok", false}, {"error", "failed to load project"}});
    return 1;
  }

  ImageGroup* group = findProjectGroup(project, group_id);
  if (!group) {
    printEvent({{"type", "project.set_camera"}, {"ok", false}, {"error", "group not found"}});
    return 1;
  }

  // Preserve existing CameraModel metadata (make/model/sensor_width etc.)
  // when only updating intrinsics via --from-k.
  CameraModel cam = group->group_camera.value_or(CameraModel{});
  cam.width = width;
  cam.height = height;
  cam.focal_length = fx;
  cam.aspect_ratio = (fy > 0.0) ? (fy / fx) : aspect_ratio;
  cam.principal_point_x = cx;
  cam.principal_point_y = cy;
  if (!camera_name.empty())
    cam.camera_name = camera_name;

  group->camera_mode = ImageGroup::CameraMode::kGroupLevel;
  group->group_camera = cam;

  project.last_modified_time = static_cast<int64_t>(std::time(nullptr));
  if (!saveProjectToFile(project_file, project)) {
    printEvent(
        {{"type", "project.set_camera"}, {"ok", false}, {"error", "failed to write project"}});
    return 1;
  }

  printEvent({{"type", "project.set_camera"},
              {"ok", true},
              {"data",
               {{"group_id", group_id},
                {"width", width},
                {"height", height},
                {"fx", fx},
                {"fy", fx * cam.aspect_ratio},
                {"cx", cx},
                {"cy", cy},
                {"from_k", !from_k_file.empty()}}}});
  return 0;
}

static int runSetCS(int argc, char* argv[]) {
  CmdLine cmd("Set project input coordinate system");
  std::string project_file;
  std::string type = "local"; // local|enu|epsg|wkt
  int epsg = 0;
  std::string wkt;
  double lat = 0.0, lon = 0.0, alt = 0.0;
  bool have_ref = false;

  cmd.add(make_option('p', project_file, "project").doc("Project file (.iat)"));
  cmd.add(make_option(0, type, "type")
              .doc("Coordinate system type: local|enu|epsg|wkt (default: local)"));
  cmd.add(make_option(0, epsg, "epsg").doc("EPSG code (when type=epsg)"));
  cmd.add(make_option(0, wkt, "wkt").doc("WKT string (when type=wkt)"));
  cmd.add(make_option(0, lat, "lat").doc("ENU reference latitude (when type=enu)"));
  cmd.add(make_option(0, lon, "lon").doc("ENU reference longitude (when type=enu)"));
  cmd.add(make_option(0, alt, "alt").doc("ENU reference altitude (when type=enu)"));
  cmd.add(make_switch(0, "have-ref").doc("Set when lat/lon/alt are provided (for enu)"));
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
  if (project_file.empty()) {
    std::cerr << "Error: -p/--project is required\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 2;
  }

  insight::tools::apply_log_level(cmd.used('v'), cmd.used('q'), log_level);
  have_ref = cmd.used("have-ref");

  Project project;
  if (!loadProjectFromFile(project_file, project)) {
    printEvent({{"type", "project.set_cs"}, {"ok", false}, {"error", "failed to load project"}});
    return 1;
  }

  CoordinateSystem cs;
  if (type == "enu") {
    cs.type = CoordinateSystem::Type::kENU;
    cs.definition = "ENU";
    if (have_ref) {
      CoordinateSystem::ReferencePoint ref;
      ref.lat = lat;
      ref.lon = lon;
      ref.alt = alt;
      cs.reference = ref;
    }
  } else if (type == "epsg") {
    cs.type = CoordinateSystem::Type::kEPSG;
    cs.definition = (epsg > 0) ? ("EPSG:" + std::to_string(epsg)) : "EPSG";
  } else if (type == "wkt") {
    cs.type = CoordinateSystem::Type::kWKT;
    cs.definition = wkt.empty() ? "WKT" : wkt;
  } else {
    cs.type = CoordinateSystem::Type::kLocal;
    cs.definition = "Local";
  }

  project.input_coordinate_system = cs;
  project.last_modified_time = static_cast<int64_t>(std::time(nullptr));

  if (!saveProjectToFile(project_file, project)) {
    printEvent({{"type", "project.set_cs"}, {"ok", false}, {"error", "failed to write project"}});
    return 1;
  }

  printEvent(
      {{"type", "project.set_cs"},
       {"ok", true},
       {"data",
        {{"project_path", project_file}, {"cs_type", type}, {"definition", cs.definition}}}});
  return 0;
}

static int runInspect(int argc, char* argv[]) {
  CmdLine cmd("Inspect project (.iat) and print JSON to stdout");
  std::string project_file;
  uint32_t task_id = static_cast<uint32_t>(-1);

  cmd.add(make_option('p', project_file, "project").doc("Input project file (.iat)"));
  cmd.add(make_option('t', task_id, "task-id").doc("Optional: inspect specific AT task_id only"));
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
    return 1;
  }

  if (cmd.checkHelp(argv[0]))
    return 0;
  if (project_file.empty()) {
    std::cerr << "Error: -p/--project is required\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 1;
  }

  insight::tools::apply_log_level(cmd.used('v'), cmd.used('q'), log_level);

  Project project;
  if (!loadProjectFromFile(project_file, project))
    return 1;

  json out;
  out["project"]["name"] = project.name;
  out["project"]["uuid"] = project.uuid;
  out["project"]["description"] = project.description;
  out["project"]["author"] = project.author;

  if (task_id != static_cast<uint32_t>(-1)) {
    const ATTask* task = findTaskById(project, task_id);
    if (!task) {
      LOG(ERROR) << "ATTask not found, task_id=" << task_id;
      return 1;
    }

    std::stringstream ss;
    cereal::JSONOutputArchive archive(ss);
    archive(cereal::make_nvp("task", *task));
    out["task_detail"] = json::parse(ss.str());
  } else {
    out["project"]["image_groups"] = project.image_groups.size();
    out["project"]["images"] = project.get_total_image_count();
    out["project"]["measurements"] = project.measurements.size();
    out["project"]["tasks"] = project.at_tasks.size();
    out["tasks"] = json::array();
    for (const auto& task : project.at_tasks) {
      out["tasks"].push_back(taskToSummaryJson(task));
    }
  }

  std::cout << out.dump(2) << std::endl;
  return 0;
}

static int runLs(int argc, char* argv[]) {
  CmdLine cmd("List AT tasks in project (.iat)");
  std::string project_file;

  cmd.add(make_option('p', project_file, "project").doc("Input project file (.iat)"));
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
    return 1;
  }

  if (cmd.checkHelp(argv[0]))
    return 0;
  if (project_file.empty()) {
    std::cerr << "Error: -p/--project is required\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 1;
  }

  insight::tools::apply_log_level(cmd.used('v'), cmd.used('q'), log_level);

  Project project;
  if (!loadProjectFromFile(project_file, project))
    return 1;

  json out;
  out["project_name"] = project.name;
  out["tasks"] = json::array();
  for (const auto& task : project.at_tasks) {
    out["tasks"].push_back(taskToSummaryJson(task));
  }

  std::cout << out.dump(2) << std::endl;
  return 0;
}

static int runCreateATTask(int argc, char* argv[]) {
  CmdLine cmd(
      "Create an AT task with a snapshot of current project (image groups, measurements, CS). "
      "Can be nested under another task via --parent-task-id.");
  std::string project_file;
  std::string task_name;
  uint32_t parent_task_id = static_cast<uint32_t>(-1);

  cmd.add(make_option('p', project_file, "project").doc("Project file (.iat)"));
  cmd.add(make_option('n', task_name, "name").doc("Task name (optional; default AT_0, AT_1, ...)"));
  cmd.add(make_option(0, parent_task_id, "parent-task-id")
              .doc("Optional: parent task_id for nesting (create sub-task)"));
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
  if (project_file.empty()) {
    std::cerr << "Error: -p/--project is required\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 2;
  }

  insight::tools::apply_log_level(cmd.used('v'), cmd.used('q'), log_level);

  Project project;
  if (!loadProjectFromFile(project_file, project)) {
    printEvent(
        {{"type", "project.create_at_task"}, {"ok", false}, {"error", "failed to load project"}});
    return 1;
  }

  if (parent_task_id != static_cast<uint32_t>(-1)) {
    const ATTask* parent = findTaskById(project, parent_task_id);
    if (!parent) {
      LOG(ERROR) << "Parent ATTask not found, task_id=" << parent_task_id;
      printEvent(
          {{"type", "project.create_at_task"}, {"ok", false}, {"error", "parent task not found"}});
      return 1;
    }
  }

  uint32_t task_id = project.next_at_task_id++;
  std::string id = generateUUIDv4();
  std::string name = task_name.empty() ? ("AT_" + std::to_string(task_id)) : task_name;

  ATTask task;
  task.id = id;
  task.task_id = task_id;
  task.task_name = name;
  task.input_snapshot.image_groups = project.image_groups;
  task.input_snapshot.measurements = project.measurements;
  task.input_snapshot.input_coordinate_system = project.input_coordinate_system;
  if (parent_task_id != static_cast<uint32_t>(-1)) {
    task.initialization = ATTask::Initialization();
    task.initialization->prev_task_id = parent_task_id;
  }
  task.optimization_config.enable_gnss_constraint = true;
  task.optimization_config.gnss_weight = 1.0;
  task.optimization_config.max_reprojection_error = 10.0;

  project.at_tasks.push_back(std::move(task));
  project.last_modified_time = static_cast<int64_t>(std::time(nullptr));

  if (!saveProjectToFile(project_file, project)) {
    printEvent(
        {{"type", "project.create_at_task"}, {"ok", false}, {"error", "failed to write project"}});
    return 1;
  }

  printEvent({{"type", "project.create_at_task"},
              {"ok", true},
              {"data", {{"task_id", task_id}, {"uuid", id}, {"task_name", name}}}});
  return 0;
}

static int runDeleteATTask(int argc, char* argv[]) {
  CmdLine cmd("Delete an AT task by task_id.");
  std::string project_file;
  uint32_t task_id = static_cast<uint32_t>(-1);

  cmd.add(make_option('p', project_file, "project").doc("Project file (.iat)"));
  cmd.add(make_option('t', task_id, "task-id").doc("AT task_id to delete (required)"));
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
  if (project_file.empty() || task_id == static_cast<uint32_t>(-1)) {
    std::cerr << "Error: -p/--project and -t/--task-id are required\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 2;
  }

  insight::tools::apply_log_level(cmd.used('v'), cmd.used('q'), log_level);

  Project project;
  if (!loadProjectFromFile(project_file, project)) {
    printEvent(
        {{"type", "project.delete_at_task"}, {"ok", false}, {"error", "failed to load project"}});
    return 1;
  }

  auto it = std::find_if(project.at_tasks.begin(), project.at_tasks.end(),
                         [task_id](const ATTask& t) { return t.task_id == task_id; });
  if (it == project.at_tasks.end()) {
    LOG(ERROR) << "ATTask not found, task_id=" << task_id;
    printEvent({{"type", "project.delete_at_task"}, {"ok", false}, {"error", "task not found"}});
    return 1;
  }

  project.at_tasks.erase(it);
  project.last_modified_time = static_cast<int64_t>(std::time(nullptr));

  if (!saveProjectToFile(project_file, project)) {
    printEvent(
        {{"type", "project.delete_at_task"}, {"ok", false}, {"error", "failed to write project"}});
    return 1;
  }

  printEvent({{"type", "project.delete_at_task"}, {"ok", true}, {"data", {{"task_id", task_id}}}});
  return 0;
}

static int runExtract(int argc, char* argv[]) {
  CmdLine cmd("Export image list for isat_extract from ATTask input snapshot");

  std::string project_file;
  std::string output_file;
  std::string image_root;
  uint32_t task_id = static_cast<uint32_t>(-1);
  int target_group_id = -1;
  bool all_groups = false;
  bool export_gnss = true;
  bool export_imu = true;

  cmd.add(make_option('p', project_file, "project").doc("Input project file (.iat)"));
  cmd.add(make_option('o', output_file, "output").doc("Output JSON for isat_extract input"));
  cmd.add(make_option('t', task_id, "task-id").doc("AT task_id (required)"));
  cmd.add(make_option('r', image_root, "root")
              .doc("Optional root path: rewrite image path as <root>/<filename>"));
  cmd.add(make_option('g', target_group_id, "group-id")
              .doc("Export specific group ID (default: first group)"));
  cmd.add(make_switch('a', "all-groups").doc("Export all groups"));
  cmd.add(make_switch(0, "no-gnss").doc("Exclude GNSS fields"));
  cmd.add(make_switch(0, "no-imu").doc("Exclude IMU fields"));
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
    return 1;
  }

  if (cmd.checkHelp(argv[0]))
    return 0;
  if (project_file.empty() || output_file.empty() || task_id == static_cast<uint32_t>(-1)) {
    std::cerr << "Error: -p/--project, -o/--output and -t/--task-id are required\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 1;
  }

  insight::tools::apply_log_level(cmd.used('v'), cmd.used('q'), log_level);

  all_groups = cmd.used('a');
  export_gnss = !cmd.used("no-gnss");
  export_imu = !cmd.used("no-imu");

  Project project;
  if (!loadProjectFromFile(project_file, project))
    return 1;

  const ATTask* task = findTaskById(project, task_id);
  if (!task) {
    LOG(ERROR) << "ATTask not found, task_id=" << task_id;
    return 1;
  }

  const std::vector<ImageGroup>& groups = task->input_snapshot.image_groups;
  std::vector<const ImageGroup*> selected = selectGroups(groups, all_groups, target_group_id);
  if (selected.empty()) {
    LOG(ERROR) << "No image groups selected from task input snapshot";
    return 1;
  }

  json output;
  output["$schema"] = "InsightAT Image List Format v2.0";
  output["images"] = json::array();

  int gnss_count = 0;
  int imu_count = 0;

  for (const auto* group : selected) {
    for (const auto& image : group->images) {
      json img;
      img["id"] = image.image_id;

      std::string image_path = image.filename;
      if (!image_root.empty()) {
        fs::path full_path = fs::path(image_root) / fs::path(image.filename).filename();
        image_path = full_path.string();
      }
      img["path"] = image_path;
      img["camera_id"] = group->group_id;

      if (export_gnss && image.gnss_data.has_value()) {
        const auto& gnss = image.gnss_data.value();
        img["gnss"] = {{"x", gnss.x},           {"y", gnss.y},
                       {"z", gnss.z},           {"cov_xx", gnss.cov_xx},
                       {"cov_yy", gnss.cov_yy}, {"cov_zz", gnss.cov_zz},
                       {"cov_xy", gnss.cov_xy}, {"cov_xz", gnss.cov_xz},
                       {"cov_yz", gnss.cov_yz}, {"num_satellites", gnss.num_satellites},
                       {"hdop", gnss.hdop},     {"vdop", gnss.vdop}};
        gnss_count++;
      }

      if (export_imu && image.input_pose.has_rotation) {
        double omega = image.input_pose.omega;
        double phi = image.input_pose.phi;
        double kappa = image.input_pose.kappa;
        if (image.input_pose.angle_unit == InputPose::AngleUnit::kRadians) {
          omega *= 180.0 / M_PI;
          phi *= 180.0 / M_PI;
          kappa *= 180.0 / M_PI;
        }

        img["imu"] = {{"roll", omega},     {"pitch", phi},      {"yaw", kappa},
                      {"cov_att_xx", 0.1}, {"cov_att_yy", 0.1}, {"cov_att_zz", 0.1}};
        imu_count++;
      }

      output["images"].push_back(std::move(img));
    }
  }

  output["metadata"] = {
      {"format_version", "2.0"},
      {"exported_from", task->task_name},
      {"task_id", task->task_id},
      {"task_uuid", task->id},
      {"exported_at", std::chrono::system_clock::now().time_since_epoch().count()},
      {"coordinate_system", task->input_snapshot.input_coordinate_system.definition.empty()
                                ? "Unknown"
                                : task->input_snapshot.input_coordinate_system.definition},
      {"num_groups_exported", selected.size()}};

  std::ofstream out_file(output_file);
  if (!out_file.is_open()) {
    LOG(ERROR) << "Failed to open output file: " << output_file;
    return 1;
  }
  out_file << output.dump(2);

  LOG(INFO) << "Exported " << output["images"].size() << " images to " << output_file;
  LOG(INFO) << "  GNSS: " << gnss_count << "  IMU: " << imu_count;
  return 0;
}

static const CameraModel* pickCameraFromTask(const ATTask& task, int group_id, int image_id,
                                             const ImageGroup** picked_group,
                                             const Image** picked_image) {
  if (picked_group)
    *picked_group = nullptr;
  if (picked_image)
    *picked_image = nullptr;

  const auto& groups = task.input_snapshot.image_groups;

  // 1) If image_id specified, search exact image first.
  if (image_id >= 0) {
    for (const auto& group : groups) {
      if (group_id >= 0 && group.group_id != static_cast<uint32_t>(group_id))
        continue;
      for (const auto& image : group.images) {
        if (image.image_id == static_cast<uint32_t>(image_id)) {
          if (image.camera.has_value()) {
            if (picked_group)
              *picked_group = &group;
            if (picked_image)
              *picked_image = &image;
            return &image.camera.value();
          }
          if (group.group_camera.has_value()) {
            if (picked_group)
              *picked_group = &group;
            if (picked_image)
              *picked_image = &image;
            return &group.group_camera.value();
          }
          return nullptr;
        }
      }
    }
    return nullptr;
  }

  // 2) If group_id specified, prefer group camera then first image camera.
  if (group_id >= 0) {
    for (const auto& group : groups) {
      if (group.group_id != static_cast<uint32_t>(group_id))
        continue;
      if (group.group_camera.has_value()) {
        if (picked_group)
          *picked_group = &group;
        return &group.group_camera.value();
      }
      for (const auto& image : group.images) {
        if (image.camera.has_value()) {
          if (picked_group)
            *picked_group = &group;
          if (picked_image)
            *picked_image = &image;
          return &image.camera.value();
        }
      }
      return nullptr;
    }
    return nullptr;
  }

  // 3) Default: first available camera in task snapshot.
  for (const auto& group : groups) {
    if (group.group_camera.has_value()) {
      if (picked_group)
        *picked_group = &group;
      return &group.group_camera.value();
    }
    for (const auto& image : group.images) {
      if (image.camera.has_value()) {
        if (picked_group)
          *picked_group = &group;
        if (picked_image)
          *picked_image = &image;
        return &image.camera.value();
      }
    }
  }

  return nullptr;
}

static int runIntrinsics(int argc, char* argv[]) {
  CmdLine cmd(
      "Export intrinsics JSON for isat_geo / isat_twoview from ATTask input snapshot.\n"
      "\n"
      "Single-camera mode (default):\n"
      "  Picks one camera from the snapshot (optionally filtered by -g/-i).\n"
      "  Output: { \"fx\":..., \"fy\":..., \"cx\":..., \"cy\":... }\n"
      "\n"
      "Multi-camera mode (--all):\n"
      "  Exports ALL cameras from every group in the snapshot.\n"
      "  Output: { \"schema\":\"multi_camera_v1\",\n"
      "            \"cameras\": { \"<group_id>\": {fx,fy,cx,cy,width,height}, ... } }\n"
      "  Use together with  isat_project extract  (which embeds camera_id=group_id\n"
      "  in the image list) and pass both files to  isat_geo -k cameras.json -l images.json.");

  std::string project_file;
  std::string output_file;
  uint32_t task_id = static_cast<uint32_t>(-1);
  int target_group_id = -1;
  int target_image_id = -1;

  cmd.add(make_option('p', project_file, "project").doc("Input project file (.iat)"));
  cmd.add(make_option('o', output_file, "output").doc("Output intrinsics JSON"));
  cmd.add(make_option('t', task_id, "task-id").doc("AT task_id (required)"));
  cmd.add(make_option('g', target_group_id, "group-id")
              .doc("Single-camera mode: pick camera from specific group (optional)"));
  cmd.add(make_option('i', target_image_id, "image-id")
              .doc("Single-camera mode: pick camera from specific image (optional)"));
  cmd.add(make_switch('a', "all")
              .doc("Multi-camera mode: export all group cameras (multi_camera_v1 format).\n"
                   "  Ignores -g/-i.  The resulting file is consumed by isat_geo/isat_twoview\n"
                   "  together with the image list (-l) to resolve per-pair intrinsics."));
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
    return 1;
  }

  if (cmd.checkHelp(argv[0]))
    return 0;
  if (project_file.empty() || output_file.empty() || task_id == static_cast<uint32_t>(-1)) {
    std::cerr << "Error: -p/--project, -o/--output and -t/--task-id are required\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 1;
  }

  insight::tools::apply_log_level(cmd.used('v'), cmd.used('q'), log_level);

  Project project;
  if (!loadProjectFromFile(project_file, project))
    return 1;

  const ATTask* task = findTaskById(project, task_id);
  if (!task) {
    LOG(ERROR) << "ATTask not found, task_id=" << task_id;
    return 1;
  }

  // ── Multi-camera mode: export all groups ─────────────────────────────────
  if (cmd.used('a')) {
    const auto& groups = task->input_snapshot.image_groups;
    if (groups.empty()) {
      LOG(ERROR) << "No image groups in task snapshot";
      return 1;
    }

    json cameras_obj = json::object();
    int exported = 0;

    for (const auto& group : groups) {
      // Resolve camera for this group
      const ImageGroup* pg = nullptr;
      const Image* pi = nullptr;
      const CameraModel* cam = pickCameraFromTask(*task, static_cast<int>(group.group_id),
                                                  -1, // no image filter
                                                  &pg, &pi);

      if (!cam) {
        LOG(WARNING) << "Group " << group.group_id << " (" << group.group_name
                     << "): no camera model, skipping";
        continue;
      }

      const double fx = cam->focal_length;
      const double fy = cam->focal_length * cam->aspect_ratio;
      if (fx <= 0.0 || fy <= 0.0) {
        LOG(WARNING) << "Group " << group.group_id << ": invalid focal, skipping";
        continue;
      }

      json cam_obj;
      cam_obj["fx"] = fx;
      cam_obj["fy"] = fy;
      cam_obj["cx"] = cam->principal_point_x;
      cam_obj["cy"] = cam->principal_point_y;
      cam_obj["width"] = cam->width;
      cam_obj["height"] = cam->height;
      // Optional metadata
      if (!cam->camera_name.empty())
        cam_obj["camera_name"] = cam->camera_name;
      if (!cam->make.empty())
        cam_obj["make"] = cam->make;
      if (!cam->model.empty())
        cam_obj["model"] = cam->model;

      cameras_obj[std::to_string(group.group_id)] = cam_obj;
      ++exported;

      LOG(INFO) << "  Group " << group.group_id << " (" << group.group_name << ")"
                << "  fx=" << fx << " fy=" << fy << " cx=" << cam->principal_point_x
                << " cy=" << cam->principal_point_y;
    }

    if (exported == 0) {
      LOG(ERROR) << "No valid cameras found in task snapshot";
      return 1;
    }

    json out;
    out["schema"] = "multi_camera_v1";
    out["cameras"] = cameras_obj;
    out["source"]["task_id"] = task->task_id;
    out["source"]["task_uuid"] = task->id;
    out["source"]["task_name"] = task->task_name;

    std::ofstream ofs(output_file);
    if (!ofs.is_open()) {
      LOG(ERROR) << "Failed to open output file: " << output_file;
      return 1;
    }
    ofs << out.dump(2);

    LOG(WARNING) << "Exported " << exported << " camera(s) to " << output_file
                 << "  (multi_camera_v1 format)";
    return 0;
  }

  // ── Single-camera mode (original behaviour) ───────────────────────────────
  const ImageGroup* picked_group = nullptr;
  const Image* picked_image = nullptr;
  const CameraModel* camera =
      pickCameraFromTask(*task, target_group_id, target_image_id, &picked_group, &picked_image);
  if (!camera) {
    LOG(ERROR) << "No camera model found in task input snapshot with current filters";
    return 1;
  }

  const double fx = camera->focal_length;
  const double fy = camera->focal_length * camera->aspect_ratio;
  const double cx = camera->principal_point_x;
  const double cy = camera->principal_point_y;
  if (fx <= 0.0 || fy <= 0.0) {
    LOG(ERROR) << "Invalid camera intrinsics: focal_length/aspect_ratio produce non-positive fx/fy";
    return 1;
  }

  json out;
  out["fx"] = fx;
  out["fy"] = fy;
  out["cx"] = cx;
  out["cy"] = cy;
  if (camera->width > 0)
    out["width"] = camera->width;
  if (camera->height > 0)
    out["height"] = camera->height;
  out["source"]["task_id"] = task->task_id;
  out["source"]["task_uuid"] = task->id;
  out["source"]["task_name"] = task->task_name;
  if (picked_group)
    out["source"]["group_id"] = picked_group->group_id;
  if (picked_image)
    out["source"]["image_id"] = picked_image->image_id;
  out["source"]["camera_type"] = static_cast<int>(camera->type);

  std::ofstream ofs(output_file);
  if (!ofs.is_open()) {
    LOG(ERROR) << "Failed to open output file: " << output_file;
    return 1;
  }
  ofs << out.dump(2);

  LOG(INFO) << "Exported intrinsics to " << output_file << "  fx=" << fx << " fy=" << fy
            << " cx=" << cx << " cy=" << cy;
  return 0;
}

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;

  if (argc < 2) {
    printTopLevelHelp(argv[0]);
    return 1;
  }

  const std::string command = argv[1];

  if (command == "create") {
    return runCreate(argc - 1, argv + 1);
  }
  if (command == "add-group") {
    return runAddGroup(argc - 1, argv + 1);
  }
  if (command == "add-images") {
    return runAddImages(argc - 1, argv + 1);
  }
  if (command == "set-camera") {
    return runSetCamera(argc - 1, argv + 1);
  }
  if (command == "set-cs") {
    return runSetCS(argc - 1, argv + 1);
  }

  if (command == "inspect") {
    return runInspect(argc - 1, argv + 1);
  }

  if (command == "ls") {
    return runLs(argc - 1, argv + 1);
  }
  if (command == "create-at-task") {
    return runCreateATTask(argc - 1, argv + 1);
  }
  if (command == "delete-at-task") {
    return runDeleteATTask(argc - 1, argv + 1);
  }

  if (command == "extract") {
    return runExtract(argc - 1, argv + 1);
  }

  if (command == "intrinsics") {
    return runIntrinsics(argc - 1, argv + 1);
  }

  std::cerr << "Error: unknown command: " << command << "\n\n";
  printTopLevelHelp(argv[0]);
  return 1;
}
