/**
 * isat_seed_eval.cpp
 *
 * Run short-window incremental SfM for one or more seed strategies,
 * aggregate metrics, and choose the best strategy.
 */

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <glog/logging.h>
#include <nlohmann/json.hpp>

#include "cli_logging.h"
#include "cmdLine/cmdLine.h"
#include "seed_eval_common.h"

namespace fs = std::filesystem;
using json = nlohmann::json;
using insight::tools::SeedEvalMetrics;
using insight::tools::SeedStrategyProfile;

namespace {

static void printEvent(const json& j) {
  std::cout << "ISAT_EVENT " << j.dump() << "\n";
  std::cout.flush();
}

static std::string quote_arg(const std::string& a) {
  if (a.find(' ') == std::string::npos && a.find('\'') == std::string::npos)
    return a;
  std::string out = "'";
  for (char c : a) {
    if (c == '\'')
      out += "'\\''";
    else
      out += c;
  }
  out += "'";
  return out;
}

static std::string build_cmd(const std::vector<std::string>& args) {
  std::string cmd;
  for (size_t i = 0; i < args.size(); ++i) {
    if (i)
      cmd += ' ';
    cmd += quote_arg(args[i]);
  }
  return cmd;
}

static int run_cmd(const std::vector<std::string>& args, const std::string& log_path,
                   const std::vector<std::string>& verbosity_args) {
  std::vector<std::string> full = args;
  full.insert(full.end(), verbosity_args.begin(), verbosity_args.end());
  std::string cmd = build_cmd(full);
  if (!log_path.empty())
    cmd += " > " + quote_arg(log_path) + " 2>&1";
  LOG(INFO) << "RUN: " << cmd;
  const int rc = std::system(cmd.c_str());
#ifdef _WIN32
  return rc;
#else
  return WIFEXITED(rc) ? WEXITSTATUS(rc) : 1;
#endif
}

static std::optional<int> read_registered_images_from_poses(const fs::path& poses_json_path) {
  std::ifstream f(poses_json_path);
  if (!f)
    return std::nullopt;
  json j;
  try {
    f >> j;
  } catch (...) {
    return std::nullopt;
  }
  if (!j.contains("poses") || !j["poses"].is_array())
    return std::nullopt;
  return static_cast<int>(j["poses"].size());
}

static std::optional<int> read_triangulated_points_from_bundle(const fs::path& bundle_path) {
  std::ifstream f(bundle_path);
  if (!f)
    return std::nullopt;
  std::string line1, line2;
  if (!std::getline(f, line1) || !std::getline(f, line2))
    return std::nullopt;
  std::istringstream iss(line2);
  int cams = 0, pts = 0;
  if (!(iss >> cams >> pts))
    return std::nullopt;
  (void)cams;
  return pts;
}

static const SeedStrategyProfile* find_profile(const std::vector<SeedStrategyProfile>& profiles,
                                               const std::string& name) {
  for (const auto& p : profiles)
    if (p.name == name)
      return &p;
  return nullptr;
}

static json profile_to_json(const SeedStrategyProfile& p) {
  return {
      {"id", p.id},
      {"name", p.name},
      {"init_min_inliers", p.init_min_inliers},
      {"init_max_forward_motion", p.init_max_forward_motion},
      {"init_min_angle_deg", p.init_min_angle_deg},
      {"init_min_median_angle_deg", p.init_min_median_angle_deg},
      {"resection_min_inliers", p.resection_min_inliers},
  };
}

static std::string resolve_incremental_sfm_bin(const std::string& requested,
                                               const fs::path& self_exe_path) {
  const fs::path requested_path(requested);

  if (requested_path.has_parent_path()) {
    std::error_code ec;
    const fs::path abs_requested = fs::absolute(requested_path, ec);
    if (!ec && fs::exists(abs_requested))
      return abs_requested.string();

    const fs::path sibling = self_exe_path.parent_path() / requested_path.filename();
    if (fs::exists(sibling))
      return sibling.string();

    return requested;
  }

  const fs::path sibling = self_exe_path.parent_path() / requested_path;
  if (fs::exists(sibling))
    return sibling.string();
  return requested;
}

} // namespace

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;

  std::string tracks_path;
  std::string project_path;
  std::string pairs_path;
  std::string geo_dir;
  std::string output_dir;
  std::string strategy_name;
  std::string log_level;
  std::string isat_incremental_sfm_bin = "isat_incremental_sfm";
  int max_eval_images = 6;

  CmdLine cmd("Seed evaluation: short-window SfM across strategy profiles");
  cmd.add(make_option('t', tracks_path, "tracks").doc("Path to .isat_tracks"));
  cmd.add(make_option('p', project_path, "project").doc("Path to project JSON (images_all.json)"));
  cmd.add(make_option('m', pairs_path, "pairs").doc("Path to pairs JSON"));
  cmd.add(make_option('g', geo_dir, "geo").doc("Path to geo directory"));
  cmd.add(make_option('o', output_dir, "output").doc("Output directory"));
  cmd.add(make_option(0, strategy_name, "strategy")
              .doc("Single strategy name to run: balanced|wide_baseline|support_first|conservative"));
  cmd.add(make_option(0, max_eval_images, "max-eval-images")
              .doc("Early-stop registration cap for each strategy (default: 6)"));
  cmd.add(make_option(0, isat_incremental_sfm_bin, "incremental-sfm-bin")
              .doc("Path to isat_incremental_sfm binary (default: isat_incremental_sfm in PATH)"));
  cmd.add(make_option(0, log_level, "log-level").doc("Log level: error|warn|info|debug"));
  cmd.add(make_switch('v', "verbose").doc("Verbose (INFO)"));
  cmd.add(make_switch('q', "quiet").doc("Quiet (ERROR only)"));
  cmd.add(make_switch('h', "help").doc("Show help"));

  try {
    cmd.process(argc, argv);
  } catch (const std::string& s) {
    std::cerr << "Error: " << s << "\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 2;
  }
  if (cmd.checkHelp(argv[0]))
    return 0;

  if (tracks_path.empty() || project_path.empty() || pairs_path.empty() || geo_dir.empty() ||
      output_dir.empty()) {
    std::cerr << "Error: -t, -p, -m, -g, -o are required\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 2;
  }
  if (max_eval_images < 2) {
    std::cerr << "Error: --max-eval-images must be >= 2\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 2;
  }

  insight::tools::apply_log_level(cmd.used('v'), cmd.used('q'), log_level);

  std::vector<std::string> verbosity_args;
  if (cmd.used('v'))
    verbosity_args.push_back("-v");
  else if (cmd.used('q'))
    verbosity_args.push_back("-q");
  else if (!log_level.empty()) {
    verbosity_args.push_back("--log-level");
    verbosity_args.push_back(log_level);
  }

  std::vector<SeedStrategyProfile> profiles = insight::tools::default_seed_profiles();
  if (!strategy_name.empty()) {
    const SeedStrategyProfile* p = find_profile(profiles, strategy_name);
    if (!p) {
      std::cerr << "Error: unknown --strategy '" << strategy_name << "'\n";
      return 2;
    }
    profiles = {*p};
  }

  fs::create_directories(output_dir);

  const std::string resolved_incremental_sfm_bin =
      resolve_incremental_sfm_bin(isat_incremental_sfm_bin, fs::absolute(argv[0]));
  LOG(INFO) << "Resolved incremental_sfm binary: " << resolved_incremental_sfm_bin;

  struct Row {
    SeedStrategyProfile profile;
    SeedEvalMetrics metrics;
    fs::path out_dir;
    fs::path log_path;
    int return_code = 0;
    std::string status = "ok";
  };
  std::vector<Row> rows;

  for (const auto& p : profiles) {
    Row row;
    row.profile = p;
    row.out_dir = fs::path(output_dir) / ("strategy_" + p.id);
    row.log_path = row.out_dir / "run.log";
    fs::create_directories(row.out_dir);

    std::vector<std::string> eval_cmd = {
      resolved_incremental_sfm_bin,
        "-t", tracks_path,
        "-p", project_path,
        "-m", pairs_path,
        "-g", geo_dir,
        "-o", row.out_dir.string(),
        "--max-registered-images", std::to_string(max_eval_images),
        "--init-min-inliers", std::to_string(p.init_min_inliers),
        "--init-max-forward-motion", std::to_string(p.init_max_forward_motion),
        "--init-min-angle-deg", std::to_string(p.init_min_angle_deg),
        "--init-min-median-angle-deg", std::to_string(p.init_min_median_angle_deg),
        "--resection-min-inliers", std::to_string(p.resection_min_inliers),
        "--fix-intrinsics"};

    const auto t0 = std::chrono::steady_clock::now();
    const int rc = run_cmd(eval_cmd, row.log_path.string(), verbosity_args);
    const auto t1 = std::chrono::steady_clock::now();
    row.metrics.runtime_sec = std::chrono::duration<double>(t1 - t0).count();
    row.return_code = rc;

    if (rc != 0) {
      LOG(WARNING) << "Strategy failed: " << p.name << " rc=" << rc;
      row.metrics.score = -1e9;
      row.status = "subprocess_failed";
      rows.push_back(std::move(row));
      continue;
    }

    const auto reg = read_registered_images_from_poses(row.out_dir / "poses.json");
    const auto tri = read_triangulated_points_from_bundle(row.out_dir / "bundle.out");
    row.metrics.registered_images = reg.value_or(0);
    row.metrics.triangulated_points = tri.value_or(0);
    row.metrics.points_per_image = row.metrics.registered_images > 0
                                       ? static_cast<double>(row.metrics.triangulated_points) /
                                             static_cast<double>(row.metrics.registered_images)
                                       : 0.0;
    row.metrics.score = insight::tools::compute_seed_eval_score(
        row.metrics.registered_images, row.metrics.triangulated_points);

    // Write per-strategy summary
    {
      json one = {
          {"strategy", profile_to_json(row.profile)},
          {"metrics",
           {{"registered_images", row.metrics.registered_images},
            {"triangulated_points", row.metrics.triangulated_points},
            {"points_per_image", row.metrics.points_per_image},
            {"runtime_sec", row.metrics.runtime_sec},
            {"score", row.metrics.score}}},
          {"paths",
           {{"output_dir", row.out_dir.string()},
            {"log", row.log_path.string()},
            {"poses_json", (row.out_dir / "poses.json").string()},
            {"bundle_out", (row.out_dir / "bundle.out").string()}}},
      };
      std::ofstream f(row.out_dir / "summary.json");
      f << one.dump(2) << "\n";
    }

    rows.push_back(std::move(row));
  }

  std::sort(rows.begin(), rows.end(), [](const Row& a, const Row& b) {
    return a.metrics.score > b.metrics.score;
  });

  json results = json::array();
  for (size_t i = 0; i < rows.size(); ++i) {
    const auto& r = rows[i];
    results.push_back({
        {"rank", static_cast<int>(i + 1)},
        {"strategy", profile_to_json(r.profile)},
        {"metrics",
         {{"registered_images", r.metrics.registered_images},
          {"triangulated_points", r.metrics.triangulated_points},
          {"points_per_image", r.metrics.points_per_image},
          {"runtime_sec", r.metrics.runtime_sec},
          {"score", r.metrics.score}}},
        {"status", r.status},
        {"return_code", r.return_code},
        {"output_dir", r.out_dir.string()},
        {"log", r.log_path.string()},
    });
  }

  const std::string best = rows.empty() ? std::string() : rows.front().profile.name;

  json report = {
      {"type", "seed_eval.report"},
      {"ok", !rows.empty()},
      {"config",
       {{"max_eval_images", max_eval_images},
        {"tracks", tracks_path},
        {"project", project_path},
        {"pairs", pairs_path},
        {"geo", geo_dir},
        {"incremental_sfm_bin", resolved_incremental_sfm_bin}}},
      {"results", results},
      {"best_strategy", best},
  };

  {
    std::ofstream f(fs::path(output_dir) / "report.json");
    f << report.dump(2) << "\n";
  }
  {
    json best_json;
    if (!rows.empty()) {
      const auto& r = rows.front();
      best_json = {
          {"best_strategy", profile_to_json(r.profile)},
          {"metrics",
           {{"registered_images", r.metrics.registered_images},
            {"triangulated_points", r.metrics.triangulated_points},
            {"points_per_image", r.metrics.points_per_image},
            {"runtime_sec", r.metrics.runtime_sec},
            {"score", r.metrics.score}}},
          {"output_dir", r.out_dir.string()},
          {"log", r.log_path.string()},
      };
    }
    std::ofstream f(fs::path(output_dir) / "best_seed.json");
    f << best_json.dump(2) << "\n";
  }

  printEvent({{"type", "seed_eval.complete"},
              {"ok", true},
              {"best_strategy", best},
              {"evaluated", static_cast<int>(rows.size())}});
  return 0;
}
