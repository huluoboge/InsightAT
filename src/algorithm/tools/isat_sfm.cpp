/**
 * isat_sfm.cpp
 * InsightAT SfM Pipeline Driver – single binary, end-to-end SfM.
 *
 * Orchestrates the full pipeline by invoking sibling CLI tools as subprocesses:
 *   1. create            – project setup, add images, camera estimation, export image list
 *   2. extract           – dual feature extraction (matching + retrieval)
 *   3. match             – 默认：检索穷举 → 全分辨率 match → geo；图像数 < --auto-exhaustive-max-images
 *                          时自动改全穷举；若有 matching_extract_meta.json 中的 low_peak 图，则与
 *                          检索 pairs 并集后再匹配；--exhaustive-match 强制全穷举
 *   4. tracks            – build tracks from matches + geometry
 *   5. incremental_sfm   – incremental SfM (resection + BA)
 *
 * Usage:
 *   isat_sfm -i /photos -w work/                              # run all steps
 *   isat_sfm -i /photos -w work/ --exhaustive-match           # 跳过检索，全图像对全分辨率匹配
 *   isat_sfm -i /photos -w work/ --no-grid                    # 提取不传 --nms（关空间网格）
 *   isat_sfm -i /photos -w work/ --geo-thresh-f 3             # 更严 F 内点（像素，见 --help）
 *   isat_sfm -i /photos -w work/ --io-threads 8 --ba-threads 8  # I/O 与 Ceres 线程数
 *   isat_sfm -i /photos -w work/ --steps create,extract       # 只跑 create + extract
 *   isat_sfm -i /photos -w work/ --steps tracks,incremental_sfm  # 从 tracks 续跑
 *
 * The binary locates sibling tools relative to its own path (same directory).
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
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

static void printEvent(const json& j) {
  std::cout << "ISAT_EVENT " << j.dump() << "\n";
  std::cout.flush();
}

// ─────────────────────────────────────────────────────────────────────────────
// Log-file sink: duplicate isat_sfm's own glog output to a file
// ─────────────────────────────────────────────────────────────────────────────

class FileSink : public google::LogSink {
public:
  explicit FileSink(const std::string& path) {
    f_ = fopen(path.c_str(), "a");
    if (!f_)
      fprintf(stderr, "[isat_sfm] WARNING: cannot open log file: %s\n", path.c_str());
  }
  ~FileSink() override {
    if (f_)
      fclose(f_);
  }
  bool ok() const { return f_ != nullptr; }
  void send(google::LogSeverity sev, const char* /*full_filename*/, const char* base_filename,
            int line, const struct ::tm* tm_time, const char* message,
            size_t message_len) override {
    if (!f_)
      return;
    static const char kCodes[] = "IWEF";
    char ts[24];
    strftime(ts, sizeof(ts), "%m%d %H:%M:%S", tm_time);
    fprintf(f_, "%c%s %s:%d] %.*s\n", kCodes[std::min((int)sev, 3)], ts, base_filename, line,
            (int)message_len, message);
    fflush(f_);
  }

private:
  FILE* f_ = nullptr;
};

// ─────────────────────────────────────────────────────────────────────────────
// Subprocess helper
// ─────────────────────────────────────────────────────────────────────────────

static std::string g_bin_dir;

/// Path of the shared pipeline log file (empty = no file logging).
static std::string g_log_file;

// ─────────────────────────────────────────────────────────────────────────────
// Step timing registry
// ─────────────────────────────────────────────────────────────────────────────

struct StepTiming {
  std::string label; ///< human-readable step name
  double secs;       ///< wall-clock seconds
};
static std::vector<StepTiming> g_step_timings;

/// Verbosity flags to forward to every child process (e.g. {"-v"} or {"--log-level","debug"}).
static std::vector<std::string> g_verbosity_args;

static std::string tool_path(const std::string& name) {
  fs::path p = fs::path(g_bin_dir) / name;
  if (fs::exists(p))
    return p.string();
  return name;
}

static std::string build_cmd(const std::vector<std::string>& args) {
  std::string cmd;
  for (size_t i = 0; i < args.size(); ++i) {
    if (i > 0)
      cmd += ' ';
    std::string a = args[i];
    bool needs_quote = a.find(' ') != std::string::npos || a.find('\'') != std::string::npos;
    if (needs_quote) {
      std::string escaped;
      for (char c : a) {
        if (c == '\'')
          escaped += "'\\''";
        else
          escaped += c;
      }
      cmd += '\'' + escaped + '\'';
    } else {
      cmd += a;
    }
  }
  return cmd;
}

/// Append g_verbosity_args to an args list.
static std::vector<std::string> with_verbosity(std::vector<std::string> args) {
  for (const auto& a : g_verbosity_args)
    args.push_back(a);
  return args;
}

/// Run subprocess, stream output to console (and to g_log_file when set). Returns exit code.
static int run(const std::vector<std::string>& args) {
  auto full_args = with_verbosity(args);
  std::string cmd = build_cmd(full_args);
  LOG(INFO) << "RUN: " << cmd;
  std::string shell_cmd = cmd;
  if (!g_log_file.empty())
    shell_cmd += " >> \"" + g_log_file + "\" 2>&1";
  int rc = std::system(shell_cmd.c_str());
#ifdef _WIN32
  return rc;
#else
  return WIFEXITED(rc) ? WEXITSTATUS(rc) : 1;
#endif
}

/// Run subprocess, capture stdout+stderr lines, parse ISAT_EVENT JSON. Returns exit code.
/// Captured events are appended to `events`. Lines echoed to stderr + g_log_file.
static int run_capture(const std::vector<std::string>& args, std::vector<json>& events) {
  auto full_args = with_verbosity(args);
  std::string cmd = build_cmd(full_args) + " 2>&1";
  LOG(INFO) << "RUN: " << build_cmd(full_args);
  FILE* pipe = popen(cmd.c_str(), "r");
  if (!pipe) {
    LOG(ERROR) << "popen failed";
    return 1;
  }
  // Open log file for appending captured output
  FILE* lf = nullptr;
  if (!g_log_file.empty())
    lf = fopen(g_log_file.c_str(), "a");
  char buf[4096];
  static const std::string prefix = "ISAT_EVENT ";
  while (fgets(buf, sizeof(buf), pipe)) {
    std::string line(buf);
    while (!line.empty() && (line.back() == '\n' || line.back() == '\r'))
      line.pop_back();
    if (line.compare(0, prefix.size(), prefix) == 0) {
      try {
        events.push_back(json::parse(line.substr(prefix.size())));
      } catch (...) {
      }
    }
    std::cerr << line << "\n";
    if (lf) {
      fprintf(lf, "%s\n", line.c_str());
      fflush(lf);
    }
  }
  if (lf)
    fclose(lf);
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
  g_step_timings.push_back({step, secs});
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
  g_step_timings.push_back({step, secs});
  return events;
}

// ─────────────────────────────────────────────────────────────────────────────
// Steps parsing
// ─────────────────────────────────────────────────────────────────────────────

static const std::vector<std::string> ALL_STEPS = {"create", "extract", "match", "tracks",
                                                   "incremental_sfm"};

static std::set<std::string> parse_steps(const std::string& steps_str) {
  std::set<std::string> result;
  std::istringstream ss(steps_str);
  std::string token;
  while (std::getline(ss, token, ',')) {
    // trim whitespace
    token.erase(0, token.find_first_not_of(" \t"));
    token.erase(token.find_last_not_of(" \t") + 1);
    if (token.empty())
      continue;
    if (std::find(ALL_STEPS.begin(), ALL_STEPS.end(), token) == ALL_STEPS.end()) {
      LOG(ERROR) << "Unknown step: '" << token << "'";
      LOG(ERROR) << "Valid steps: create, extract, match, tracks, incremental_sfm";
      std::exit(2);
    }
    result.insert(token);
  }
  return result;
}

/// images_all.json → 图像数量（与 isat_retrieval_match 一致）。
static int count_images_in_images_all_json(const fs::path& images_all_path) {
  std::ifstream f(images_all_path);
  if (!f)
    return 0;
  json j;
  try {
    f >> j;
  } catch (...) {
    return 0;
  }
  if (j.contains("images") && j["images"].is_array())
    return static_cast<int>(j["images"].size());
  return 0;
}

/// 写入全穷举图像对 JSON（image1_index < image2_index），供 isat_match / isat_geo 使用。
static bool write_exhaustive_pairs_json(const fs::path& output_path, int n_images,
                                        std::string* err_out) {
  if (n_images < 2) {
    if (err_out)
      *err_out = "need at least 2 images for exhaustive pairs";
    return false;
  }
  json pairs_arr = json::array();
  for (int i = 0; i < n_images; ++i) {
    for (int j = i + 1; j < n_images; ++j)
      pairs_arr.push_back({{"image1_index", i}, {"image2_index", j}});
  }
  json out = {{"pairs", pairs_arr}};
  std::ofstream f(output_path);
  if (!f) {
    if (err_out)
      *err_out = "cannot write " + output_path.string();
    return false;
  }
  f << out.dump(2) << "\n";
  LOG(INFO) << "Exhaustive pairs: " << pairs_arr.size() << " → " << output_path.string();
  return true;
}

/// 读取 isat_extract 写入的 matching_extract_meta.json，返回 low_peak_matching==true 的 image_index。
static std::vector<int> read_low_peak_matching_indices(const fs::path& meta_path) {
  std::vector<int> out;
  std::ifstream f(meta_path);
  if (!f)
    return out;
  json j;
  try {
    f >> j;
  } catch (...) {
    return out;
  }
  if (!j.contains("images") || !j["images"].is_array())
    return out;
  for (const auto& im : j["images"]) {
    if (!im.contains("image_index"))
      continue;
    const int idx = im["image_index"].get<int>();
    if (im.value("low_peak_matching", false))
      out.push_back(idx);
  }
  return out;
}

/// 将检索得到的 pairs 与「所有涉及 boost 图像的穷举对」取并集，写回 pairs_path（同一 JSON 格式）。
static bool merge_retrieval_pairs_with_low_peak_boost(const fs::path& pairs_path, int n_images,
                                                      const std::vector<int>& boost_indices,
                                                      std::string* err_out) {
  if (n_images < 2) {
    if (err_out)
      *err_out = "merge pairs: n_images < 2";
    return false;
  }
  std::set<std::pair<int, int>> seen;
  std::ifstream pf(pairs_path);
  if (!pf) {
    if (err_out)
      *err_out = "cannot read " + pairs_path.string();
    return false;
  }
  json jp;
  try {
    pf >> jp;
  } catch (...) {
    if (err_out)
      *err_out = "invalid JSON " + pairs_path.string();
    return false;
  }
  if (jp.contains("pairs") && jp["pairs"].is_array()) {
    for (const auto& p : jp["pairs"]) {
      int a = p.value("image1_index", -1);
      int b = p.value("image2_index", -1);
      if (a < 0 || b < 0)
        continue;
      if (a > b)
        std::swap(a, b);
      seen.insert({a, b});
    }
  }
  const size_t from_retrieval = seen.size();
  for (int bi : boost_indices) {
    if (bi < 0 || bi >= n_images)
      continue;
    for (int j = 0; j < n_images; ++j) {
      if (j == bi)
        continue;
      int a = bi, b = j;
      if (a > b)
        std::swap(a, b);
      seen.insert({a, b});
    }
  }
  json pairs_arr = json::array();
  for (const auto& pr : seen)
    pairs_arr.push_back({{"image1_index", pr.first}, {"image2_index", pr.second}});
  json out = {{"pairs", pairs_arr}};
  std::ofstream of(pairs_path);
  if (!of) {
    if (err_out)
      *err_out = "cannot write " + pairs_path.string();
    return false;
  }
  of << out.dump(2) << "\n";
  LOG(INFO) << "Adaptive pairs: retrieval " << from_retrieval << " unique → merged "
            << pairs_arr.size() << " (added exhaustive links for " << boost_indices.size()
            << " low-peak image(s)) → " << pairs_path.string();
  return true;
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
    if (token.empty())
      continue;
    if (token[0] != '.')
      token = "." + token;
    std::transform(token.begin(), token.end(), token.begin(), ::tolower);
    if (!result.empty())
      result += ',';
    result += token;
  }
  return result;
}

static bool has_images(const fs::path& dir, const std::string& exts) {
  for (const auto& entry : fs::directory_iterator(dir)) {
    if (!entry.is_regular_file())
      continue;
    std::string ext = entry.path().extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    if (exts.find(ext) != std::string::npos)
      return true;
  }
  return false;
}

static std::vector<std::pair<std::string, fs::path>> scan_groups(const fs::path& root_in,
                                                                 const std::string& exts) {
  // Strip trailing separator at string level (lexically_normal() is unreliable on some GCC builds)
  std::string root_str = root_in.string();
  while (!root_str.empty() && (root_str.back() == '/' || root_str.back() == '\\'))
    root_str.pop_back();
  const fs::path root(root_str);
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
    groups.emplace_back(root.lexically_normal().filename().string(), root);
  return groups;
}

// ─────────────────────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;

  g_bin_dir = fs::path(argv[0]).parent_path().string();
  if (g_bin_dir.empty())
    g_bin_dir = ".";

  // ── CLI options ──────────────────────────────────────────────────────────
  std::string input_dir;
  std::string work_dir;
  std::string steps_str = "create,extract,match,tracks,incremental_sfm";
  std::string extract_backend = "cuda";
  std::string match_backend = "cuda";
  std::string ext = ".jpg,.tif,.png";
  std::string log_level;
  int max_sample = 5;
  bool exhaustive_match = false;
  /// When not using --exhaustive-match: if image count < this, skip retrieval and use full
  /// exhaustive pairs (same as manual exhaustive for small sets).
  int auto_exhaustive_max_images = 60;
  /// Passed to isat_geo --min-inliers (F/E/H RANSAC inlier count gate for pairs.json).
  int geo_min_inliers = 10;
  /// Passed to isat_geo -t / --thresh-f (F 内点 Sampson 门限，单位像素；越大越宽松).
  double geo_thresh_f = 16.0;
  /// isat_extract / isat_retrieval_match / isat_match CPU I/O worker count (GPU 仍单线程上下文).
  int io_threads = 4;
  /// isat_incremental_sfm Ceres BA threads (0 = 子进程默认，按硬件并发).
  int ba_threads = 0;

  CmdLine cmd("InsightAT SfM Pipeline – end-to-end incremental SfM");
  cmd.add(make_option('i', input_dir, "input").doc("Input directory containing images (required)"));
  cmd.add(
      make_option('w', work_dir, "work-dir").doc("Working directory for all outputs (required)"));
  cmd.add(make_option('s', steps_str, "steps")
              .doc("Comma-separated steps to run (default: "
                   "create,extract,match,tracks,incremental_sfm)"));
  cmd.add(make_option(0, extract_backend, "extract-backend")
              .doc("Feature extraction backend: cuda or glsl (default: cuda)"));
  cmd.add(make_option(0, match_backend, "match-backend")
              .doc("Matching backend: cuda or glsl (default: cuda)"));
  cmd.add(make_option(0, ext, "ext")
              .doc("Image extensions, comma-separated (default: .jpg,.tif,.png)"));
  cmd.add(make_option(0, max_sample, "max-sample")
              .doc("Max images sampled for focal estimation (default: 5)"));
  cmd.add(make_switch(0, "fix-intrinsics").doc("Hold camera intrinsics fixed during BA"));
  cmd.add(make_switch(0, "exhaustive-match")
              .doc("Skip isat_retrieval_match; generate all image pairs and run full-resolution "
                   "matching only (O(n²) pairs, heavy for large sets)"));
  cmd.add(make_option(0, auto_exhaustive_max_images, "auto-exhaustive-max-images")
              .doc("When --exhaustive-match is off: if image count is below this, automatically "
                   "use full exhaustive pairs (default: 60). Set 0 to disable."));
  cmd.add(make_switch(0, "no-grid")
              .doc("Feature extract: do not pass --nms to isat_extract (disable spatial grid on "
                   "matching + retrieval features; temporary A/B vs dense SIFT)"));
  cmd.add(make_option(0, geo_min_inliers, "geo-min-inliers")
              .doc("isat_geo --min-inliers: min F/E/H inliers to accept a pair (default: 10). "
                   "Stricter scenes: try 12–15."));
  cmd.add(
      make_option(0, geo_thresh_f, "geo-thresh-f")
          .doc("isat_geo -t: F inlier threshold in pixels (default: 16.0). "
               "Larger tolerates calibration / distortion / noise; too large admits bad pairs."));
  cmd.add(make_option(0, log_level, "log-level").doc("Log level: error|warn|info|debug"));
  cmd.add(make_switch('v', "verbose").doc("Verbose (INFO); also forwarded to all sub-tools"));
  cmd.add(make_switch('q', "quiet").doc("Quiet (ERROR only); also forwarded to all sub-tools"));
  cmd.add(make_switch('h', "help").doc("Show help"));
  cmd.add(make_switch(0, "no-log-file").doc("Disable automatic log file in <work-dir>/logs/"));
  cmd.add(make_option(0, io_threads, "io-threads")
              .doc("CPU threads for extract + retrieval + match I/O stages (default: 4). "
                   "Forwarded as -j/--threads to those tools."));
  cmd.add(make_option(0, ba_threads, "ba-threads")
              .doc("Ceres thread count for incremental SfM bundle adjustment (default: 0 = use "
                   "isat_incremental_sfm hardware default). Set >0 to cap or fix parallelism."));

  try {
    cmd.process(argc, argv);
  } catch (const std::string& s) {
    std::cerr << "Error: " << s << "\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 1;
  }
  if (cmd.checkHelp(argv[0]))
    return 0;
  if (input_dir.empty() || work_dir.empty()) {
    std::cerr << "Error: --input and --work-dir are required\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 1;
  }
  if (geo_min_inliers < 1) {
    std::cerr << "Error: --geo-min-inliers must be >= 1\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 1;
  }
  if (!(geo_thresh_f > 0.0)) {
    std::cerr << "Error: --geo-thresh-f must be > 0\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 1;
  }
  if (auto_exhaustive_max_images < 0) {
    std::cerr << "Error: --auto-exhaustive-max-images must be >= 0 (0 disables)\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 1;
  }
  if (io_threads < 1) {
    std::cerr << "Error: --io-threads must be >= 1\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 1;
  }
  if (ba_threads < 0) {
    std::cerr << "Error: --ba-threads must be >= 0\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 1;
  }
  insight::tools::apply_log_level(cmd.used('v'), cmd.used('q'), log_level);
  bool fix_intrinsics = cmd.used("fix-intrinsics");
  exhaustive_match = cmd.used("exhaustive-match");
  const bool no_grid = cmd.used("no-grid");

  // ── Build verbosity args to forward to all sub-tools ────────────────────
  if (cmd.used('v'))
    g_verbosity_args.push_back("-v");
  else if (cmd.used('q'))
    g_verbosity_args.push_back("-q");
  else if (!log_level.empty()) {
    g_verbosity_args.push_back("--log-level");
    g_verbosity_args.push_back(log_level);
  }

  // ── Set up dated log file in <work-dir>/logs/ ────────────────────────────
  if (!cmd.used("no-log-file")) {
    std::string wd = work_dir;
    while (!wd.empty() && (wd.back() == '/' || wd.back() == '\\'))
      wd.pop_back();
    fs::path logs_dir = fs::absolute(wd) / "logs";
    fs::create_directories(logs_dir);
    // Generate filename: sfm_YYYYMMDD_HHMMSS.log
    {
      std::time_t now = std::time(nullptr);
      std::tm* lt = std::localtime(&now);
      char ts[32];
      std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", lt);
      g_log_file = (logs_dir / (std::string("sfm_") + ts + ".log")).string();
    }
    // Add sink so isat_sfm's own glog lines also appear in the file
    static FileSink* s_sink = nullptr;
    s_sink = new FileSink(g_log_file); // lives for process lifetime
    if (s_sink->ok()) {
      google::AddLogSink(s_sink);
      LOG(INFO) << "Pipeline log file: " << g_log_file;
    } else {
      delete s_sink;
      s_sink = nullptr;
      g_log_file.clear();
    }
  }

  // Normalize extensions: "JPG" → ".jpg", "tif" → ".tif", etc.
  ext = normalize_exts(ext);
  LOG(INFO) << "Image extensions: " << ext;
  LOG(INFO) << "Threading: io_threads=" << io_threads << "  ba_threads=" << ba_threads
            << (ba_threads > 0 ? "" : " (BA: auto)");

  auto active_steps = parse_steps(steps_str);
  {
    std::string active_list;
    for (const auto& s : ALL_STEPS)
      if (active_steps.count(s))
        active_list += (active_list.empty() ? "" : ", ") + s;
    LOG(INFO) << "Active steps: " << active_list;
  }

  // Strip trailing directory separators before constructing paths.
  // lexically_normal() does not reliably remove trailing '/' on all GCC/libstdc++ versions.
  auto strip_trailing_sep = [](std::string s) -> std::string {
    while (!s.empty() && (s.back() == '/' || s.back() == '\\'))
      s.pop_back();
    return s;
  };
  fs::path input_path = fs::absolute(strip_trailing_sep(input_dir));
  fs::path work_path = fs::absolute(strip_trailing_sep(work_dir));
  if (!fs::is_directory(input_path)) {
    LOG(ERROR) << "Input directory does not exist: " << input_path;
    return 1;
  }

  // ── Derived paths ────────────────────────────────────────────────────────
  fs::path project_path = work_path / "project.iat";
  fs::path images_all = work_path / "images_all.json";
  fs::path feat_dir = work_path / "feat";
  fs::path feat_ret_dir = work_path / "feat_retrieval";
  fs::path pairs_retrieve = work_path / "pairs_retrieve.json";
  fs::path match_dir_path = work_path / "match";
  fs::path geo_dir = work_path / "geo";
  fs::path pairs_json = geo_dir / "pairs.json";
  fs::path tracks_path = work_path / "tracks.isat_tracks";
  fs::path sfm_out = work_path / "incremental_sfm";

  // Create work directory (sub-dirs created per step as needed)
  fs::create_directories(work_path);

  // Sensor DB: look next to binary
  std::string sensor_db;
  {
    fs::path candidate = fs::path(g_bin_dir) / "data" / "config" / "camera_sensor_database.txt";
    if (fs::exists(candidate))
      sensor_db = candidate.string();
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
      auto events = run_capture_or_die("add-group", {tool_path("isat_project"), "add-group", "-p",
                                                     project_path.string(), "-n", name});
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
                 {tool_path("isat_project"), "add-images", "-p", project_path.string(), "-g",
                  std::to_string(group_ids[gi]), "-i", groups[gi].second.string(), "--ext", ext});
    }

    {
      std::vector<std::string> cam_cmd = {tool_path("isat_camera_estimator"),
                                          "-p",
                                          project_path.string(),
                                          "-a",
                                          "--max-sample",
                                          std::to_string(max_sample),
                                          "--auto-split"};
      if (!sensor_db.empty()) {
        cam_cmd.push_back("-d");
        cam_cmd.push_back(sensor_db);
      }
      run_or_die("camera-estimate", cam_cmd);
    }

    run_or_die("create-at-task",
               {tool_path("isat_project"), "create-at-task", "-p", project_path.string()});

    run_or_die("export-images", {tool_path("isat_project"), "extract", "-p", project_path.string(),
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

    const int sift_levels = 3;
    //           << sift_levels << ")";
    if (no_grid) {
      LOG(INFO) << "isat_extract: spatial grid disabled (--no-grid, omitting --nms)";
    }
    if (exhaustive_match) {
      LOG(INFO) << "Exhaustive full-res matching + Geometry";
      std::vector<std::string> extract_cmd = {tool_path("isat_extract"),
                                              "-i",
                                              images_all.string(),
                                              "-o",
                                              feat_dir.string(),
                                              "--extract-backend",
                                              extract_backend,
                                              "--nfeatures",
                                              "10000",
                                              "--threshold",
                                              "0.02",
                                              "--octaves",
                                              "-1",
                                              "--levels",
                                              std::to_string(sift_levels),
                                              "--norm",
                                              "l1root",
                                              "--no-adapt",
                                              "--uint8",
                                              "-j",
                                              std::to_string(io_threads)};
      if (!no_grid)
        extract_cmd.push_back("--nms");
      run_or_die("extract", extract_cmd);
    } else {
      std::vector<std::string> extract_cmd = {tool_path("isat_extract"),
                                              "-i",
                                              images_all.string(),
                                              "-o",
                                              feat_dir.string(),
                                              "--output-retrieval",
                                              feat_ret_dir.string(),
                                              "--nfeatures-retrieval",
                                              "1500",
                                              "--resize-retrieval",
                                              "1024",
                                              "--extract-backend",
                                              extract_backend,
                                              "--nfeatures",
                                              "10000",
                                              "--threshold",
                                              "0.02",
                                              "--octaves",
                                              "-1",
                                              "--levels",
                                              std::to_string(sift_levels),
                                              "--norm",
                                              "l1root",
                                              "--no-adapt",
                                              "--uint8",
                                              "-j",
                                              std::to_string(io_threads)};
      if (!no_grid)
        extract_cmd.push_back("--nms");
      run_or_die("extract", extract_cmd);
    }
  }

  // ════════════════════════════════════════════════════════════════════════
  // Step: MATCH (retrieval-by-matching → match → geo) 或 全穷举全分辨率匹配
  // ════════════════════════════════════════════════════════════════════════
  if (active_steps.count("match")) {
    ++step_num;

    const int n_img = count_images_in_images_all_json(images_all);
    if (n_img < 2) {
      LOG(ERROR) << "match step: need at least 2 images in " << images_all << " (got " << n_img
                 << ")";
      return 1;
    }

    const bool manual_exhaustive = exhaustive_match;
    const bool auto_exhaustive =
        !manual_exhaustive && auto_exhaustive_max_images > 0 && (n_img < auto_exhaustive_max_images);
    const bool use_exhaustive_pairs = manual_exhaustive || auto_exhaustive;

    fs::create_directories(match_dir_path);
    fs::create_directories(geo_dir);

    LOG(INFO) << "=== Step " << step_num << "/" << total_steps << ": Match + Geometry ===";
    LOG(INFO) << "  Images: " << n_img << "  manual_exhaustive=" << (manual_exhaustive ? "yes" : "no")
              << "  auto_exhaustive=" << (auto_exhaustive ? "yes" : "no") << " (threshold "
              << auto_exhaustive_max_images << ")";

    if (use_exhaustive_pairs) {
      const long long n_pairs = static_cast<long long>(n_img) * (n_img - 1) / 2;
      if (manual_exhaustive) {
        LOG(INFO) << "Exhaustive pairs (--exhaustive-match): " << n_img << " images → " << n_pairs
                  << " pairs (skipped isat_retrieval_match)";
      } else {
        LOG(INFO) << "Auto exhaustive pairs (image count " << n_img << " < " << auto_exhaustive_max_images
                  << "): " << n_pairs << " pairs (skipped isat_retrieval_match)";
      }
      if (n_img > 80) {
        LOG(WARNING) << "Very large image count; exhaustive matching may be extremely slow and "
                        "memory-heavy.";
      }
      std::string werr;
      if (!write_exhaustive_pairs_json(pairs_retrieve, n_img, &werr)) {
        LOG(ERROR) << werr;
        return 1;
      }
    } else {
      fs::path retrieval_work = work_path / "retrieval_match_work";
      fs::create_directories(retrieval_work);

      run_or_die("retrieval-match",
                 {tool_path("isat_retrieval_match"), "-l", images_all.string(), "-f",
                  feat_ret_dir.string(), "-w", retrieval_work.string(), "-o",
                  pairs_retrieve.string(), "--match-backend", match_backend, "--max-features", "4096",
                  "--threads", std::to_string(io_threads)});

      const fs::path meta_path = feat_dir / "matching_extract_meta.json";
      const std::vector<int> boost_idx = read_low_peak_matching_indices(meta_path);
      if (!boost_idx.empty()) {
        std::string werr;
        if (!merge_retrieval_pairs_with_low_peak_boost(pairs_retrieve, n_img, boost_idx, &werr)) {
          LOG(ERROR) << werr;
          return 1;
        }
      } else {
        LOG(INFO) << "No low-peak boost (missing or empty " << meta_path.string()
                  << "); pairs = retrieval output only.";
      }
    }

    run_or_die("match", {tool_path("isat_match"), "-i", pairs_retrieve.string(), "-f",
                         feat_dir.string(), "-o", match_dir_path.string(), "--match-backend",
                         match_backend, "--max-features", "10000", "--threads",
                         std::to_string(io_threads)});

    // Full geometric verification on full-resolution matches
    char geo_tf_buf[64];
    std::snprintf(geo_tf_buf, sizeof(geo_tf_buf), "%.9g", geo_thresh_f);
    LOG(INFO) << "isat_geo thresholds: min-inliers=" << geo_min_inliers
              << "  thresh-f(px)=" << geo_tf_buf;
    run_or_die("geo",
               {tool_path("isat_geo"), "-i", pairs_retrieve.string(), "-m", match_dir_path.string(),
                "-o", geo_dir.string(), "--image-list", images_all.string(), "-t",
                std::string(geo_tf_buf), "--min-inliers", std::to_string(geo_min_inliers),
                "--backend", "gpu", "--estimate-h", "--twoview", "--vis"});
  }

  // ════════════════════════════════════════════════════════════════════════
  // Step: TRACKS
  // ════════════════════════════════════════════════════════════════════════
  if (active_steps.count("tracks")) {
    ++step_num;
    LOG(INFO) << "=== Step " << step_num << "/" << total_steps << ": Track building ===";

    run_or_die("tracks",
               {tool_path("isat_tracks"), "-i", pairs_json.string(), "-m", match_dir_path.string(),
                "-g", geo_dir.string(), "-l", images_all.string(), "-o", tracks_path.string(),
                "--min-track-length", "2"});
  }

  // ════════════════════════════════════════════════════════════════════════
  // Step: INCREMENTAL_SFM
  // ════════════════════════════════════════════════════════════════════════
  if (active_steps.count("incremental_sfm")) {
    ++step_num;
    LOG(INFO) << "=== Step " << step_num << "/" << total_steps << ": Incremental SfM ===";
    fs::create_directories(sfm_out);

    std::vector<std::string> sfm_cmd = {tool_path("isat_incremental_sfm"),
                                        "-t",
                                        tracks_path.string(),
                                        "-p",
                                        images_all.string(),
                                        "-m",
                                        pairs_json.string(),
                                        "-g",
                                        geo_dir.string(),
                                        "-o",
                                        sfm_out.string()};
    if (fix_intrinsics)
      sfm_cmd.push_back("--fix-intrinsics");
    if (ba_threads > 0) {
      sfm_cmd.push_back("--ba-threads");
      sfm_cmd.push_back(std::to_string(ba_threads));
    }
    run_or_die("incremental-sfm", sfm_cmd);
  }

  // ════════════════════════════════════════════════════════════════════════
  // Done
  // ════════════════════════════════════════════════════════════════════════
  auto pipeline_end = std::chrono::steady_clock::now();
  double total_secs = std::chrono::duration<double>(pipeline_end - pipeline_start).count();

  // ── Per-step timing table (stderr, human-readable) ──────────────────────
  auto fmt_secs = [](double s) -> std::string {
    char buf[32];
    if (s < 60.0)
      snprintf(buf, sizeof(buf), "%.1fs", s);
    else if (s < 3600.0)
      snprintf(buf, sizeof(buf), "%.0fm%.0fs", std::floor(s / 60), std::fmod(s, 60));
    else
      snprintf(buf, sizeof(buf), "%.0fh%.0fm", std::floor(s / 3600),
               std::floor(std::fmod(s, 3600) / 60));
    return buf;
  };

  // Determine column widths
  size_t max_label = 5; // "Step"
  for (const auto& t : g_step_timings)
    max_label = std::max(max_label, t.label.size());

  auto sep = std::string(max_label + 2, '-') + "+" + std::string(10, '-');
  auto hdr_line =
      std::string(" ") + std::string("Step") + std::string(max_label - 4, ' ') + " │ Time";

  LOG(INFO) << "========================================";
  LOG(INFO) << "Pipeline complete in " << fmt_secs(total_secs);
  LOG(INFO) << "";
  LOG(INFO) << hdr_line;
  LOG(INFO) << sep;
  for (const auto& t : g_step_timings) {
    std::string padding(max_label - t.label.size(), ' ');
    LOG(INFO) << " " << t.label << padding << " │ " << fmt_secs(t.secs);
  }
  LOG(INFO) << sep;
  {
    std::string total_label("Total");
    std::string padding(max_label - total_label.size(), ' ');
    LOG(INFO) << " " << total_label << padding << " │ " << fmt_secs(total_secs);
  }
  LOG(INFO) << "";
  if (active_steps.count("incremental_sfm")) {
    LOG(INFO) << "  Poses:  " << (sfm_out / "poses.json").string();
    LOG(INFO) << "  Bundle: " << (sfm_out / "bundle.out").string();
    LOG(INFO) << "View results:";
    LOG(INFO) << "  " << tool_path("at_bundler_viewer") << " " << (sfm_out / "bundle.out").string()
              << " " << (sfm_out / "list.txt").string();
  }
  LOG(INFO) << "========================================";

  // ── Build timing JSON (shared by file output + ISAT_EVENT) ─────────────
  json timing_json;
  {
    json steps_arr = json::array();
    for (const auto& t : g_step_timings)
      steps_arr.push_back({{"step", t.label}, {"elapsed_s", t.secs}});

    // ISO-8601 timestamp
    std::time_t now = std::time(nullptr);
    std::tm* lt = std::localtime(&now);
    char ts_buf[32];
    std::strftime(ts_buf, sizeof(ts_buf), "%Y-%m-%dT%H:%M:%S", lt);

    timing_json = {{"total_s", total_secs},
                   {"steps", steps_arr},
                   {"timestamp", ts_buf},
                   {"work_dir", work_path.string()}};
  }

  // ── Write sfm_timing.json into work_dir ──────────────────────────────────
  {
    fs::path timing_path = work_path / "sfm_timing.json";
    std::ofstream f(timing_path);
    if (f.is_open()) {
      f << timing_json.dump(2) << "\n";
      LOG(INFO) << "Timing data: " << timing_path;
    } else {
      LOG(WARNING) << "Failed to write timing file: " << timing_path;
    }
  }

  // ── ISAT_EVENT: pipeline timing (stdout, machine-readable) ───────────────
  printEvent({{"type", "sfm.pipeline_timing"}, {"ok", true}, {"data", timing_json}});

  return 0;
}
