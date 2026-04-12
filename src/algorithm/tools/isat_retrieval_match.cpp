/**
 * isat_retrieval_match.cpp
 * InsightAT Retrieval-by-Matching – discover image pairs via exhaustive low-res matching.
 *
 * Strategy (Wu, VisualSFM style):
 *   1. Generate exhaustive pairs from the image list
 *   2. Run isat_match on low-resolution retrieval features (GPU)
 *   3. Run isat_geo with --min-inliers 1 (F-matrix only, lenient)
 *   4. Read geo output pairs.json, count neighbours per image
 *   5. Images with < K neighbours (default 5): add exhaustive pairs to all other images
 *   6. Merge and output final pairs.json
 *
 * Usage:
 *   isat_retrieval_match -l images_all.json -f feat_retrieval/ -w retrieval_work/ -o pairs.json
 *
 * The tool invokes sibling isat_match and isat_geo as subprocesses.
 */

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>

#include <glog/logging.h>
#include <nlohmann/json.hpp>

#include "cli_logging.h"
#include "cmdLine/cmdLine.h"

namespace fs = std::filesystem;
using json = nlohmann::json;

// ─────────────────────────────────────────────────────────────────────────────
// Subprocess
// ─────────────────────────────────────────────────────────────────────────────

static std::string g_bin_dir;

static std::string tool_path(const std::string& name) {
  fs::path p = fs::path(g_bin_dir) / name;
  if (fs::exists(p)) return p.string();
  return name;
}

static int run(const std::vector<std::string>& args) {
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
  LOG(INFO) << "RUN: " << cmd;
  int rc = std::system(cmd.c_str());
#ifdef _WIN32
  return rc;
#else
  return WIFEXITED(rc) ? WEXITSTATUS(rc) : 1;
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

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Read image count from images_all.json
static int count_images(const std::string& image_list_path) {
  std::ifstream f(image_list_path);
  if (!f.is_open()) return 0;
  json j;
  try { f >> j; } catch (...) { return 0; }
  if (j.contains("images") && j["images"].is_array())
    return static_cast<int>(j["images"].size());
  return 0;
}

/// Generate exhaustive pairs JSON: all (i,j) where i < j
static void write_exhaustive_pairs(const std::string& output_path, int n_images) {
  json pairs_arr = json::array();
  for (int i = 0; i < n_images; ++i) {
    for (int j = i + 1; j < n_images; ++j) {
      pairs_arr.push_back({{"image1_index", i}, {"image2_index", j}});
    }
  }
  json out = {{"pairs", pairs_arr}};
  std::ofstream f(output_path);
  f << out.dump(2) << "\n";
  LOG(INFO) << "Wrote " << pairs_arr.size() << " exhaustive pairs to " << output_path;
}

/// Read geo output pairs.json → set of (min,max) index pairs + per-image neighbour count
static void read_geo_pairs(const std::string& pairs_path,
                           std::set<std::pair<int,int>>& verified_pairs,
                           std::map<int,int>& neighbour_count,
                           int n_images) {
  // Init counts to 0
  for (int i = 0; i < n_images; ++i) neighbour_count[i] = 0;

  std::ifstream f(pairs_path);
  if (!f.is_open()) {
    LOG(WARNING) << "Cannot open geo pairs: " << pairs_path;
    return;
  }
  json j;
  try { f >> j; } catch (...) { return; }
  if (!j.contains("pairs") || !j["pairs"].is_array()) return;

  for (const auto& p : j["pairs"]) {
    int i1 = p.value("image1_index", -1);
    int i2 = p.value("image2_index", -1);
    if (i1 < 0 || i2 < 0) continue;
    int lo = std::min(i1, i2), hi = std::max(i1, i2);
    if (verified_pairs.insert({lo, hi}).second) {
      neighbour_count[lo]++;
      neighbour_count[hi]++;
    }
  }
  LOG(INFO) << "Read " << verified_pairs.size() << " verified pairs from " << pairs_path;
}

/// Write final merged pairs JSON
static void write_final_pairs(const std::string& output_path,
                              const std::set<std::pair<int,int>>& pairs) {
  json pairs_arr = json::array();
  for (const auto& [i, j] : pairs) {
    pairs_arr.push_back({{"image1_index", i}, {"image2_index", j}});
  }
  json out = {{"pairs", pairs_arr}};
  std::ofstream f(output_path);
  f << out.dump(2) << "\n";
  LOG(INFO) << "Wrote " << pairs_arr.size() << " final pairs to " << output_path;
}

// ─────────────────────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;

  g_bin_dir = fs::path(argv[0]).parent_path().string();
  if (g_bin_dir.empty()) g_bin_dir = ".";

  std::string image_list;
  std::string feat_dir;
  std::string work_dir;
  std::string output_path;
  std::string match_backend = "cuda";
  std::string log_level;
  int min_neighbours = 5;
  int match_threads = 4;
  int max_features = 4096;

  CmdLine cmd("InsightAT Retrieval-by-Matching – discover pairs via exhaustive low-res matching + F verification");
  cmd.add(make_option('l', image_list, "image-list").doc("Image list JSON (images_all.json)"));
  cmd.add(make_option('f', feat_dir, "feat-dir").doc("Feature directory (.isat_feat files)"));
  cmd.add(make_option('w', work_dir, "work-dir").doc("Working directory for intermediate match/geo results"));
  cmd.add(make_option('o', output_path, "output").doc("Output pairs JSON path"));
  cmd.add(make_option(0, match_backend, "match-backend").doc("Matching backend: cuda or glsl (default: cuda)"));
  cmd.add(make_option('k', min_neighbours, "min-neighbours").doc("Min neighbours; images below this get exhaustive pairs added (default: 5)"));
  cmd.add(make_option(0, max_features, "max-features").doc("Max features per image for matching (default: 4096)"));
  cmd.add(make_option(0, match_threads, "threads").doc("Matching threads (default: 4)"));
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
  if (image_list.empty() || feat_dir.empty() || work_dir.empty() || output_path.empty()) {
    std::cerr << "Error: -l, -f, -w, -o are all required\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 1;
  }
  insight::tools::apply_log_level(cmd.used('v'), cmd.used('q'), log_level);

  auto t_start = std::chrono::steady_clock::now();

  int n_images = count_images(image_list);
  if (n_images < 2) {
    LOG(ERROR) << "Need at least 2 images, got " << n_images;
    return 1;
  }
  int n_exhaustive = n_images * (n_images - 1) / 2;
  LOG(INFO) << "Images: " << n_images << ", exhaustive pairs: " << n_exhaustive;

  // ── Paths ────────────────────────────────────────────────────────────────
  fs::path wp = fs::absolute(work_dir);
  fs::path match_dir = wp / "match";
  fs::path geo_dir   = wp / "geo";
  fs::path exhaustive_pairs = wp / "exhaustive_pairs.json";
  fs::create_directories(match_dir);
  fs::create_directories(geo_dir);

  // ── Step 1: Generate exhaustive pairs ────────────────────────────────────
  LOG(INFO) << "=== Step 1/3: Generate exhaustive pairs ===";
  write_exhaustive_pairs(exhaustive_pairs.string(), n_images);

  // ── Step 2: Match (low-res features, GPU) ────────────────────────────────
  LOG(INFO) << "=== Step 2/3: Exhaustive matching (low-res) ===";
  run_or_die("match",
    {tool_path("isat_match"),
     "-i", exhaustive_pairs.string(),
     "-f", feat_dir,
     "-o", match_dir.string(),
     "--match-backend", match_backend,
     "--max-features", "-1",
     "--threads", std::to_string(match_threads)});

  // ── Step 3: Geometric verification (F only, lenient) ─────────────────────
  LOG(INFO) << "=== Step 3/3: Geometric verification (F, min-inliers=1) ===";
  run_or_die("geo",
    {tool_path("isat_geo"),
     "-i", exhaustive_pairs.string(),
     "-m", match_dir.string(),
     "-o", geo_dir.string(),
     "--image-list", image_list,
     "--min-inliers", "1",
     "--backend", "poselib"});

  // ── Analyse results ──────────────────────────────────────────────────────
  std::set<std::pair<int,int>> verified_pairs;
  std::map<int,int> neighbour_count;
  read_geo_pairs((geo_dir / "pairs.json").string(), verified_pairs, neighbour_count, n_images);

  LOG(INFO) << "Verified pairs: " << verified_pairs.size() << " / " << n_exhaustive;

  // ── Supplement weak images ───────────────────────────────────────────────
  int weak_count = 0;
  int added_pairs = 0;
  for (int i = 0; i < n_images; ++i) {
    if (neighbour_count[i] < min_neighbours) {
      weak_count++;
      // Add pairs to ALL other images (unverified, "better too many than too few")
      for (int j = 0; j < n_images; ++j) {
        if (i == j) continue;
        int lo = std::min(i, j), hi = std::max(i, j);
        if (verified_pairs.insert({lo, hi}).second)
          added_pairs++;
      }
    }
  }
  if (weak_count > 0) {
    LOG(INFO) << "Weak images (neighbours < " << min_neighbours << "): " << weak_count
              << ", added " << added_pairs << " exhaustive pairs for them";
  } else {
    LOG(INFO) << "All images have >= " << min_neighbours << " neighbours, no supplement needed";
  }

  // ── Write final output ───────────────────────────────────────────────────
  write_final_pairs(output_path, verified_pairs);

  double total_secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
  LOG(INFO) << "Retrieval-by-matching complete in " << total_secs << "s";
  LOG(INFO) << "  Verified: " << (verified_pairs.size() - added_pairs)
            << ", supplemented: " << added_pairs
            << ", total: " << verified_pairs.size();

  return 0;
}
