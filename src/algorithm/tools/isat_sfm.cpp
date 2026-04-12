/**
 * isat_sfm.cpp
 * InsightAT SfM Pipeline Driver – single binary, end-to-end SfM.
 *
 * Orchestrates the full pipeline by invoking sibling CLI tools as subprocesses:
 *   1. create            – project setup, add images, camera estimation, export image list
 *   2. extract           – dual feature extraction (matching + retrieval)
 *   3. match             – retrieval-by-matching (exhaustive low-res) → full-res match → geo
 *   4. tracks            – build tracks from matches + geometry
 *   5. incremental_sfm   – incremental SfM (resection + BA)
 *
 * Usage:
 *   isat_sfm -i /photos -w work/                              # run all steps
 *   isat_sfm -i /photos -w work/ --steps create,extract       # run only create + extract
 *   isat_sfm -i /photos -w work/ --steps tracks,incremental_sfm  # resume from tracks
 *
 * The binary locates sibling tools relative to its own path (same directory).
 */

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <glog/logging.h>
#include <nlohmann/json.hpp>

#include "cli_logging.h"
#include "cmdLine/cmdLine.h"

namespace fs = std::filesystem;
using json = nlohmann::json;

// ─────────────────────────────────────────────────────────────────────────────
// Subprocess helper
// ─────────────────────────────────────────────────────────────────────────────

static std::string g_bin_dir;

static std::string tool_path(const std::string& name) {
  fs::path p = fs::path(g_bin_dir) / name;
  if (fs::exists(p)) return p.string();
  return name;
}

static std::string build_cmd(const std::vector<std::string>& args) {
  std::string cmd;
  for (size_t i = 0; i < args.size(); ++i) {
    if (i > 0) cmd += ' ';
    std::string a = args[i];
    bool needs_quote = a.find(' ') != std::string::npos || a.find('\'') != std::string::npos;
    if (needs_quote) {
      std::string escaped;
      for (char c : a) {
        if (c == '\'') escaped += "'\\''";
        else escaped += c;
      }
      cmd += '\'' + escaped + '\'';
    } else {
      cmd += a;
    }
  }
  return cmd;
}

/// Run subprocess, stream output to console. Returns exit code.
static int run(const std::vector<std::string>& args) {
  std::string cmd = build_cmd(args);
  LOG(INFO) << "RUN: " << cmd;
  int rc = std::system(cmd.c_str());
#ifdef _WIN32
  return rc;
#else
  return WIFEXITED(rc) ? WEXITSTATUS(rc) : 1;
#endif
}

/// Run subprocess, capture stdout lines, parse ISAT_EVENT JSON. Returns exit code.
/// Captured events are appended to `events`. Stderr still goes to console.
static int run_capture(const std::vector<std::string>& args, std::vector<json>& events) {
  // Redirect stdout to pipe, keep stderr on console
  std::string cmd = build_cmd(args) + " 2>&1";
  LOG(INFO) << "RUN: " << build_cmd(args);
  FILE* pipe = popen(cmd.c_str(), "r");
  if (!pipe) {
    LOG(ERROR) << "popen failed";
    return 1;
  }
  char buf[4096];
  static const std::string prefix = "ISAT_EVENT ";
  while (fgets(buf, sizeof(buf), pipe)) {
    std::string line(buf);
    // Remove trailing newline
    while (!line.empty() && (line.back() == '\n' || line.back() == '\r'))
      line.pop_back();
    // Parse ISAT_EVENT lines
    if (line.compare(0, prefix.size(), prefix) == 0) {
      try {
        events.push_back(json::parse(line.substr(prefix.size())));
      } catch (...) {}
    }
    // Echo everything to stderr so user sees output
    std::cerr << line << "\n";
  }
  int status = pclose(pipe);
#ifdef _WIN32
  return status;
#else
  return WIFEXITED(status) ? WEXITSTATUS(status) : 1;
#endif
}

static void run_or_die(const std::string& step, const std::vector<std::string>& args) {
  auto t0 = std::chrono::steady_clock::now();
  int rc = run(args);
  double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  if (rc != 0) {
    LOG(ERROR) << "Step [" << step << "] failed (exit code " << rc << ")";
    std::exit(1);
  }
  LOG(INFO) << "Step [" << step << "] completed in " << secs << "s";
}

/// Run and capture ISAT_EVENT; abort on failure. Returns parsed events.
static std::vector<json> run_capture_or_die(const std::string& step,
                                             const std::vector<std::string>& args) {
  std::vector<json> events;
  auto t0 = std::chrono::steady_clock::now();
  int rc = run_capture(args, events);
  double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  if (rc != 0) {
    LOG(ERROR) << "Step [" << step << "] failed (exit code " << rc << ")";
    std::exit(1);
  }
  LOG(INFO) << "Step [" << step << "] completed in " << secs << "s";
  return events;
}

// ─────────────────────────────────────────────────────────────────────────────
// Steps parsing
// ─────────────────────────────────────────────────────────────────────────────

static const std::vector<std::string> ALL_STEPS = {
    "create", "extract", "match", "tracks", "incremental_sfm"};

static std::set<std::string> parse_steps(const std::string& steps_str) {
  std::set<std::string> result;
  std::istringstream ss(steps_str);
  std::string token;
  while (std::getline(ss, token, ',')) {
    // trim whitespace
    token.erase(0, token.find_first_not_of(" \t"));
    token.erase(token.find_last_not_of(" \t") + 1);
    if (token.empty()) continue;
    if (std::find(ALL_STEPS.begin(), ALL_STEPS.end(), token) == ALL_STEPS.end()) {
      LOG(ERROR) << "Unknown step: '" << token << "'";
      LOG(ERROR) << "Valid steps: create, extract, match, tracks, incremental_sfm";
      std::exit(2);
    }
    result.insert(token);
  }
  return result;
}

// ─────────────────────────────────────────────────────────────────────────────
// Directory scanning
// ─────────────────────────────────────────────────────────────────────────────

/// Normalize extension string: ensure each token has a leading dot and is lowercase.
/// Input: "JPG,.tif,png" → ".jpg,.tif,.png"
static std::string normalize_exts(const std::string& raw) {
  std::string result;
  std::istringstream ss(raw);
  std::string token;
  while (std::getline(ss, token, ',')) {
    token.erase(0, token.find_first_not_of(" \t"));
    token.erase(token.find_last_not_of(" \t") + 1);
    if (token.empty()) continue;
    if (token[0] != '.') token = "." + token;
    std::transform(token.begin(), token.end(), token.begin(), ::tolower);
    if (!result.empty()) result += ',';
    result += token;
  }
  return result;
}

static bool has_images(const fs::path& dir, const std::string& exts) {
  for (const auto& entry : fs::directory_iterator(dir)) {
    if (!entry.is_regular_file()) continue;
    std::string ext = entry.path().extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    if (exts.find(ext) != std::string::npos) return true;
  }
  return false;
}

static std::vector<std::pair<std::string, fs::path>> scan_groups(const fs::path& root,
                                                                  const std::string& exts) {
  std::vector<std::pair<std::string, fs::path>> groups;
  for (const auto& entry : fs::recursive_directory_iterator(root)) {
    if (entry.is_directory() && has_images(entry.path(), exts)) {
      std::string name = fs::relative(entry.path(), root).string();
      std::replace(name.begin(), name.end(), '/', '_');
      std::replace(name.begin(), name.end(), '\\', '_');
      groups.emplace_back(name, entry.path());
    }
  }
  std::sort(groups.begin(), groups.end());
  if (groups.empty() && has_images(root, exts))
    groups.emplace_back(root.filename().string(), root);
  return groups;
}

// ─────────────────────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;

  g_bin_dir = fs::path(argv[0]).parent_path().string();
  if (g_bin_dir.empty()) g_bin_dir = ".";

  // ── CLI options ──────────────────────────────────────────────────────────
  std::string input_dir;
  std::string work_dir;
  std::string steps_str = "create,extract,match,tracks,incremental_sfm";
  std::string extract_backend = "cuda";
  std::string match_backend = "cuda";
  std::string ext = ".jpg,.tif,.png";
  std::string log_level;
  int max_sample = 5;

  CmdLine cmd("InsightAT SfM Pipeline – end-to-end incremental SfM");
  cmd.add(make_option('i', input_dir, "input").doc("Input directory containing images (required)"));
  cmd.add(make_option('w', work_dir, "work-dir").doc("Working directory for all outputs (required)"));
  cmd.add(make_option('s', steps_str, "steps")
              .doc("Comma-separated steps to run (default: create,extract,match,tracks,incremental_sfm)"));
  cmd.add(make_option(0, extract_backend, "extract-backend").doc("Feature extraction backend: cuda or glsl (default: cuda)"));
  cmd.add(make_option(0, match_backend, "match-backend").doc("Matching backend: cuda or glsl (default: cuda)"));
  cmd.add(make_option(0, ext, "ext").doc("Image extensions, comma-separated (default: .jpg,.tif,.png)"));
  cmd.add(make_option(0, max_sample, "max-sample").doc("Max images sampled for focal estimation (default: 5)"));
  cmd.add(make_switch(0, "fix-intrinsics").doc("Hold camera intrinsics fixed during BA"));
  cmd.add(make_option(0, log_level, "log-level").doc("Log level: error|warn|info|debug"));
  cmd.add(make_switch('v', "verbose").doc("Verbose (INFO)"));
  cmd.add(make_switch('q', "quiet").doc("Quiet (ERROR only)"));
  cmd.add(make_switch('h', "help").doc("Show help"));

  try { cmd.process(argc, argv); }
  catch (const std::string& s) {
    std::cerr << "Error: " << s << "\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 1;
  }
  if (cmd.checkHelp(argv[0])) return 0;
  if (input_dir.empty() || work_dir.empty()) {
    std::cerr << "Error: --input and --work-dir are required\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 1;
  }
  insight::tools::apply_log_level(cmd.used('v'), cmd.used('q'), log_level);
  bool fix_intrinsics = cmd.used("fix-intrinsics");

  // Normalize extensions: "JPG" → ".jpg", "tif" → ".tif", etc.
  ext = normalize_exts(ext);
  LOG(INFO) << "Image extensions: " << ext;

  auto active_steps = parse_steps(steps_str);
  {
    std::string active_list;
    for (const auto& s : ALL_STEPS)
      if (active_steps.count(s)) active_list += (active_list.empty() ? "" : ", ") + s;
    LOG(INFO) << "Active steps: " << active_list;
  }

  fs::path input_path = fs::absolute(input_dir);
  fs::path work_path  = fs::absolute(work_dir);
  if (!fs::is_directory(input_path)) {
    LOG(ERROR) << "Input directory does not exist: " << input_path;
    return 1;
  }

  // ── Derived paths ────────────────────────────────────────────────────────
  fs::path project_path     = work_path / "project.iat";
  fs::path images_all       = work_path / "images_all.json";
  fs::path feat_dir         = work_path / "feat";
  fs::path feat_ret_dir     = work_path / "feat_retrieval";
  fs::path pairs_retrieve   = work_path / "pairs_retrieve.json";
  fs::path match_dir_path   = work_path / "match";
  fs::path geo_dir          = work_path / "geo";
  fs::path pairs_json       = geo_dir   / "pairs.json";
  fs::path tracks_path      = work_path / "tracks.isat_tracks";
  fs::path sfm_out          = work_path / "incremental_sfm";

  // Create work directory (sub-dirs created per step as needed)
  fs::create_directories(work_path);

  // Sensor DB: look next to binary
  std::string sensor_db;
  {
    fs::path candidate = fs::path(g_bin_dir) / "data" / "config" / "camera_sensor_database.txt";
    if (fs::exists(candidate)) sensor_db = candidate.string();
  }

  auto pipeline_start = std::chrono::steady_clock::now();
  int step_num = 0;
  int total_steps = static_cast<int>(active_steps.size());

  // ════════════════════════════════════════════════════════════════════════
  // Step: CREATE
  // ════════════════════════════════════════════════════════════════════════
  if (active_steps.count("create")) {
    ++step_num;
    LOG(INFO) << "=== Step " << step_num << "/" << total_steps << ": Create project ===";

    run_or_die("create-project",
      {tool_path("isat_project"), "create", "-p", project_path.string()});

    auto groups = scan_groups(input_path, ext);
    if (groups.empty()) {
      LOG(ERROR) << "No image groups found under " << input_path;
      return 1;
    }
    LOG(INFO) << "Found " << groups.size() << " image group(s)";

    // Add groups and capture actual group_ids from ISAT_EVENT
    std::vector<int> group_ids;
    for (const auto& [name, path] : groups) {
      auto events = run_capture_or_die("add-group",
        {tool_path("isat_project"), "add-group", "-p", project_path.string(), "-n", name});
      int gid = -1;
      for (const auto& ev : events) {
        if (ev.contains("data") && ev["data"].contains("group_id"))
          gid = ev["data"]["group_id"].get<int>();
      }
      if (gid < 0) {
        LOG(ERROR) << "Failed to get group_id for group '" << name << "'";
        return 1;
      }
      group_ids.push_back(gid);
      LOG(INFO) << "Group '" << name << "' → group_id=" << gid;
    }

    // Add images to each group using actual group_id
    for (size_t gi = 0; gi < groups.size(); ++gi) {
      run_or_die("add-images",
        {tool_path("isat_project"), "add-images", "-p", project_path.string(),
         "-g", std::to_string(group_ids[gi]), "-i", groups[gi].second.string(), "--ext", ext});
    }

    {
      std::vector<std::string> cam_cmd = {
        tool_path("isat_camera_estimator"), "-p", project_path.string(),
        "-a", "--max-sample", std::to_string(max_sample), "--auto-split"};
      if (!sensor_db.empty()) { cam_cmd.push_back("-d"); cam_cmd.push_back(sensor_db); }
      run_or_die("camera-estimate", cam_cmd);
    }

    run_or_die("create-at-task",
      {tool_path("isat_project"), "create-at-task", "-p", project_path.string()});

    run_or_die("export-images",
      {tool_path("isat_project"), "extract", "-p", project_path.string(),
       "-t", "0", "-o", images_all.string(), "-a"});
  }

  // ════════════════════════════════════════════════════════════════════════
  // Step: EXTRACT
  // ════════════════════════════════════════════════════════════════════════
  if (active_steps.count("extract")) {
    ++step_num;
    LOG(INFO) << "=== Step " << step_num << "/" << total_steps << ": Feature extraction ===";
    fs::create_directories(feat_dir);
    fs::create_directories(feat_ret_dir);

    run_or_die("extract",
      {tool_path("isat_extract"),
       "-i", images_all.string(),
       "-o", feat_dir.string(),
       "--output-retrieval", feat_ret_dir.string(),
       "--nfeatures-retrieval", "1500",
       "--resize-retrieval", "1024",
       "--extract-backend", extract_backend,
       "--nfeatures", "40000",
       "--threshold", "0.02",
       "--octaves", "-1",
       "--levels", "3",
       "--norm", "l1root",
       "--no-adapt", "--nms", "--uint8"});
  }

  // ════════════════════════════════════════════════════════════════════════
  // Step: MATCH (retrieval-by-matching → match → geo)
  // ════════════════════════════════════════════════════════════════════════
  if (active_steps.count("match")) {
    ++step_num;
    LOG(INFO) << "=== Step " << step_num << "/" << total_steps << ": Retrieval-by-matching + Geometry ===";
    fs::create_directories(match_dir_path);
    fs::create_directories(geo_dir);

    // Use isat_retrieval_match: exhaustive low-res matching → F verification → supplement weak images
    // This replaces the VLAD train → retrieve → match → geo pipeline with a more robust approach.
    // Output: pairs_retrieve.json (verified + supplemented pairs)
    fs::path retrieval_work = work_path / "retrieval_match_work";
    fs::create_directories(retrieval_work);

    run_or_die("retrieval-match",
      {tool_path("isat_retrieval_match"),
       "-l", images_all.string(),
       "-f", feat_dir.string(),
       "-w", retrieval_work.string(),
       "-o", pairs_retrieve.string(),
       "--match-backend", match_backend,
       "--max-features", "4096",
       "--threads", "4"});

    // Now run full-resolution matching on the discovered pairs
    run_or_die("match",
      {tool_path("isat_match"),
       "-i", pairs_retrieve.string(),
       "-f", feat_dir.string(),
       "-o", match_dir_path.string(),
       "--match-backend", match_backend,
       "--max-features", "-1",
       "--threads", "4"});

    // Full geometric verification on full-resolution matches
    run_or_die("geo",
      {tool_path("isat_geo"),
       "-i", pairs_retrieve.string(),
       "-m", match_dir_path.string(),
       "-o", geo_dir.string(),
       "--image-list", images_all.string(),
       "--min-inliers", "15",
       "--backend", "gpu",
       "--estimate-h", "--twoview", "--vis"});
  }

  // ════════════════════════════════════════════════════════════════════════
  // Step: TRACKS
  // ════════════════════════════════════════════════════════════════════════
  if (active_steps.count("tracks")) {
    ++step_num;
    LOG(INFO) << "=== Step " << step_num << "/" << total_steps << ": Track building ===";

    run_or_die("tracks",
      {tool_path("isat_tracks"),
       "-i", pairs_json.string(),
       "-m", match_dir_path.string(),
       "-g", geo_dir.string(),
       "-l", images_all.string(),
       "-o", tracks_path.string(),
       "--min-track-length", "3"});
  }

  // ════════════════════════════════════════════════════════════════════════
  // Step: INCREMENTAL_SFM
  // ════════════════════════════════════════════════════════════════════════
  if (active_steps.count("incremental_sfm")) {
    ++step_num;
    LOG(INFO) << "=== Step " << step_num << "/" << total_steps << ": Incremental SfM ===";
    fs::create_directories(sfm_out);

    std::vector<std::string> sfm_cmd = {
      tool_path("isat_incremental_sfm"),
      "-t", tracks_path.string(),
      "-p", images_all.string(),
      "-m", pairs_json.string(),
      "-g", geo_dir.string(),
      "-o", sfm_out.string()};
    if (fix_intrinsics) sfm_cmd.push_back("--fix-intrinsics");
    run_or_die("incremental-sfm", sfm_cmd);
  }

  // ════════════════════════════════════════════════════════════════════════
  // Done
  // ════════════════════════════════════════════════════════════════════════
  auto pipeline_end = std::chrono::steady_clock::now();
  double total_secs = std::chrono::duration<double>(pipeline_end - pipeline_start).count();

  LOG(INFO) << "========================================";
  LOG(INFO) << "Pipeline complete in " << total_secs << "s";
  if (active_steps.count("incremental_sfm")) {
    LOG(INFO) << "  Poses:  " << (sfm_out / "poses.json").string();
    LOG(INFO) << "  Bundle: " << (sfm_out / "bundle.out").string();
    LOG(INFO) << "View results:";
    LOG(INFO) << "  " << tool_path("at_bundler_viewer") << " "
              << (sfm_out / "bundle.out").string() << " "
              << (sfm_out / "list.txt").string();
  }
  LOG(INFO) << "========================================";

  return 0;
}
