/**
 * isat_calibrate.cpp
 * InsightAT – Focal-Length Calibration Aggregation Tool
 *
 * Reads all .isat_twoview files produced by isat_twoview, collects per-pair
 * focal-length estimates, runs a global 1-D optimisation to find the single
 * best focal length that minimises the total cross-pair reprojection error,
 * and writes a refined K.json file ready to feed back into isat_geo -k.
 *
 * Pipeline:
 *   1. Scan --twoview-dir for *.isat_twoview files.
 *   2. For each file load: R, t, focal_estimate, n_points, rmse, quality.
 *      (Load .isat_match pixel coords + points3d only for Phase-2 BA pairs.)
 *   3. Phase 1 – Weighted median  over focal estimates to get f₀.
 *   4. Phase 2 – 1-D golden-section search over a tight range [f₀/1.2, f₀*1.2]:
 *        For each candidate f, re-triangulate the inlier correspondences
 *        for every "good" pair (given fixed R from Phase-1 result) and
 *        compute mean reprojection error.  Choose f* that minimises it.
 *   5. Write K.json:
 *        { "fx": f*, "fy": f*, "cx": cx, "cy": cy,
 *          "rmse_px": ..., "num_pairs": ..., "method": "isat_calibrate" }
 *
 * Usage:
 *   isat_calibrate -t twoview_dir/ -m match_dir/ \
 *                  -o K.json --image-width 3840 --image-height 2160
 *
 * After this step, re-run:
 *   isat_geo -i pairs.json -m match_dir/ -o geo2_dir/ -k K.json
 * to obtain a better Essential matrix, then re-run isat_twoview with -k K.json
 * for the final reconstruction.
 */

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

#include <glog/logging.h>
#include <nlohmann/json.hpp>

#include "../io/idc_reader.h"
#include "../modules/sfm/two_view_reconstruction.h"
#include "cli_logging.h"
#include "cmdLine/cmdLine.h"
#include "pair_json_utils.h"

namespace fs = std::filesystem;
using json = nlohmann::json;
using namespace insight::io;
using namespace insight::sfm;

static constexpr const char* kEventPrefix = "ISAT_EVENT ";

static void print_event(const json& j) {
  std::cout << kEventPrefix << j.dump() << "\n";
  std::cout.flush();
}

// ─────────────────────────────────────────────────────────────────────────────
// Per-pair aggregation record
// ─────────────────────────────────────────────────────────────────────────────

struct CalibEntry {
  uint32_t image1_id = 0, image2_id = 0;
  std::string twoview_path;
  std::string match_path;

  double focal_estimate = 0.0;
  double reprojection_rmse = 0.0;
  int n_points = 0;
  std::string quality; // "good" / "poor"

  Eigen::Matrix3d R;
  Eigen::Vector3d t;

  // Pixel correspondences (loaded on demand for Phase-2)
  std::vector<Eigen::Vector2d> pts1_px, pts2_px;
  std::vector<Eigen::Vector3d> points3d;
  bool loaded = false;
};

// ─────────────────────────────────────────────────────────────────────────────
// Load summary from a .isat_twoview file  (no blobs)
// ─────────────────────────────────────────────────────────────────────────────

static bool load_tv_header(CalibEntry& e) {
  IDCReader rdr(e.twoview_path);
  if (!rdr.is_valid())
    return false;
  const auto& meta = rdr.get_metadata();
  if (!meta.value("success", false))
    return false;

  e.focal_estimate = meta.value("focal_length_px", 0.0);
  e.reprojection_rmse = meta.value("reprojection_rmse_px", 1e9);
  e.n_points = meta.value("num_points_3d", 0);
  e.quality = meta.value("quality", "poor");
  e.image1_id = insight::tools::get_image_id_from_pair(meta["image_pair"], "image1_id");
  e.image2_id = insight::tools::get_image_id_from_pair(meta["image_pair"], "image2_id");

  const auto& pose = meta["pose"];
  const auto& Rv = pose["R"];
  const auto& tv = pose["t"];
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      e.R(i, j) = Rv[i * 3 + j].template get<double>();
  for (int i = 0; i < 3; ++i)
    e.t[i] = tv[i].template get<double>();
  return (e.focal_estimate > 0.0 && e.n_points > 0);
}

// Load full pixel correspondences and 3-D points for Phase-2 evaluation
static bool load_tv_full(CalibEntry& e) {
  if (e.loaded)
    return true;

  // Load points3d from .isat_twoview
  IDCReader tv_rdr(e.twoview_path);
  if (!tv_rdr.is_valid())
    return false;
  auto pts_blob = tv_rdr.read_blob<double>("points3d");
  const int Np = (int)pts_blob.size() / 3;
  if (Np == 0)
    return false;
  e.points3d.resize(Np);
  for (int i = 0; i < Np; ++i)
    e.points3d[i] = {pts_blob[i * 3], pts_blob[i * 3 + 1], pts_blob[i * 3 + 2]};

  auto ids_blob = tv_rdr.read_blob<int32_t>("inlier_ids");
  if (ids_blob.size() != (size_t)Np)
    return false;

  // Load pixel coords from .isat_match
  if (e.match_path.empty())
    return false;
  IDCReader match_rdr(e.match_path);
  if (!match_rdr.is_valid())
    return false;
  auto coords = match_rdr.read_blob<float>("coords_pixel");
  const int N_all = (int)coords.size() / 4;

  e.pts1_px.resize(Np);
  e.pts2_px.resize(Np);
  for (int i = 0; i < Np; ++i) {
    const int idx = ids_blob[i];
    if (idx < 0 || idx >= N_all)
      return false;
    e.pts1_px[i] = {coords[idx * 4 + 0], coords[idx * 4 + 1]};
    e.pts2_px[i] = {coords[idx * 4 + 2], coords[idx * 4 + 3]};
  }
  e.loaded = true;
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Evaluate a candidate focal length: mean reprojection error over all pairs
// ─────────────────────────────────────────────────────────────────────────────

static double eval_focal(std::vector<CalibEntry>& entries, double f, double cx, double cy,
                        int min_pts) {
  double sq_sum = 0.0;
  int count = 0;

  for (auto& e : entries) {
    if (e.quality != "good")
      continue;
    if (!e.loaded)
      continue;

    // Re-triangulate with this focal
    std::vector<Eigen::Vector2d> pts1_n(e.pts1_px.size());
    std::vector<Eigen::Vector2d> pts2_n(e.pts1_px.size());
    for (int i = 0; i < (int)e.pts1_px.size(); ++i) {
      pts1_n[i] = {(e.pts1_px[i][0] - cx) / f, (e.pts1_px[i][1] - cy) / f};
      pts2_n[i] = {(e.pts2_px[i][0] - cx) / f, (e.pts2_px[i][1] - cy) / f};
    }
    auto X_new = insight::sfm::triangulate_points(pts1_n, pts2_n, e.R, e.t);

    for (int i = 0; i < (int)X_new.size(); ++i) {
      const auto& X = X_new[i];
      if (!std::isfinite(X[0]))
        continue;
      // Project into cam1
      if (X[2] < 1e-4)
        continue;
      double u1 = f * X[0] / X[2] + cx;
      double v1 = f * X[1] / X[2] + cy;
      double du1 = u1 - e.pts1_px[i][0], dv1 = v1 - e.pts1_px[i][1];
      // Project into cam2
      Eigen::Vector3d Xc2 = e.R * X + e.t;
      if (Xc2[2] < 1e-4)
        continue;
      double u2 = f * Xc2[0] / Xc2[2] + cx;
      double v2 = f * Xc2[1] / Xc2[2] + cy;
      double du2 = u2 - e.pts2_px[i][0], dv2 = v2 - e.pts2_px[i][1];
      sq_sum += du1 * du1 + dv1 * dv1 + du2 * du2 + dv2 * dv2;
      ++count;
    }
  }
  if (count < min_pts)
    return 1e9;
  return std::sqrt(sq_sum / (2.0 * count));
}

// ─────────────────────────────────────────────────────────────────────────────
// Weighted median (O(N log N))
// ─────────────────────────────────────────────────────────────────────────────

static double weighted_median(const std::vector<double>& vals, const std::vector<double>& weights) {
  assert(vals.size() == weights.size());
  if (vals.empty())
    return 0.0;
  // Create index-sorted by value
  std::vector<int> idx(vals.size());
  std::iota(idx.begin(), idx.end(), 0);
  std::sort(idx.begin(), idx.end(), [&](int a, int b) { return vals[a] < vals[b]; });
  double total_w = 0.0;
  for (double w : weights)
    total_w += w;
  double half = total_w * 0.5;
  double cum = 0.0;
  for (int i : idx) {
    cum += weights[i];
    if (cum >= half)
      return vals[i];
  }
  return vals[idx.back()];
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;

  CmdLine cmd("InsightAT Focal-Length Calibration\n"
              "  Aggregates per-pair focal estimates from isat_twoview,\n"
              "  runs a global 1-D optimisation, and writes K.json for isat_geo.");

  std::string twoview_dir, match_dir, output_json;
  int image_width = 0;
  int image_height = 0;
  double cx_override = 0.0;
  double cy_override = 0.0;
  int min_pairs = 5;
  int min_pts = 20;
  double rmse_thresh = 3.0; // discard pairs with initial rmse > this

  cmd.add(make_option('t', twoview_dir, "twoview-dir")
              .doc("Directory of .isat_twoview files (output of isat_twoview)"));
  cmd.add(make_option('m', match_dir, "match-dir")
              .doc("Directory of .isat_match files (for Phase-2 re-triangulation)"));
  cmd.add(make_option('o', output_json, "output")
              .doc("Output path for K.json. Example: --output K.json"));
  cmd.add(make_option(0, image_width, "image-width")
              .doc("Image width  (px) – sets cx = W/2 when --cx not given"));
  cmd.add(make_option(0, image_height, "image-height")
              .doc("Image height (px) – sets cy = H/2 when --cy not given"));
  cmd.add(make_option(0, cx_override, "cx")
              .doc("Principal-point x override (px). Overrides --image-width / 2."));
  cmd.add(make_option(0, cy_override, "cy")
              .doc("Principal-point y override (px). Overrides --image-height / 2."));
  cmd.add(make_option(0, min_pairs, "min-pairs")
              .doc("Minimum number of good pairs required. Default: 5"));
  cmd.add(make_option(0, rmse_thresh, "rmse-thresh")
              .doc("Exclude pairs with per-pair BA rmse > this (pixels). Default: 3.0"));
  std::string log_level;
  cmd.add(make_option(0, log_level, "log-level").doc("Log level: error|warn|info|debug"));
  cmd.add(make_switch('v', "verbose").doc("Verbose logging"));
  cmd.add(make_switch('q', "quiet").doc("Quiet (ERROR only)"));
  cmd.add(make_switch('h', "help").doc("Show this help"));

  try {
    cmd.process(argc, argv);
  } catch (const std::string& s) {
    std::cerr << "Error: " << s << "\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 1;
  }
  if (cmd.checkHelp(argv[0]))
    return 0;

  if (twoview_dir.empty() || output_json.empty()) {
    std::cerr << "Error: -t/--twoview-dir and -o/--output are required\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 1;
  }

  insight::tools::apply_log_level(cmd.used('v'), cmd.used('q'), log_level);

  double cx = (cx_override > 0.0) ? cx_override : (image_width * 0.5);
  double cy = (cy_override > 0.0) ? cy_override : (image_height * 0.5);
  if (cx <= 0.0 || cy <= 0.0)
    LOG(WARNING) << "Principal point (cx,cy) is (0,0); supply --image-width/--image-height";

  // ── Scan .isat_twoview files ─────────────────────────────────────────────
  std::vector<CalibEntry> entries;
  for (const auto& de : fs::directory_iterator(twoview_dir)) {
    if (de.path().extension() != ".isat_twoview")
      continue;
    CalibEntry e;
    e.twoview_path = de.path().string();
    if (!load_tv_header(e))
      continue;
    if (e.reprojection_rmse > rmse_thresh)
      continue;
    if (!match_dir.empty()) {
      e.match_path = match_dir + "/" + std::to_string(e.image1_id) + "_" +
                     std::to_string(e.image2_id) + ".isat_match";
    }
    entries.push_back(std::move(e));
  }

  LOG(WARNING) << "Found " << entries.size() << " usable pairs (rmse < " << rmse_thresh << " px)";

  // ── Phase 1: weighted median focal estimate ────────────────────────────
  std::vector<double> focals, weights;
  for (const auto& e : entries) {
    if (e.focal_estimate <= 0.0)
      continue;
    // Weight by quality proxy: more points + lower rmse → higher weight
    double w = static_cast<double>(e.n_points) / (1.0 + e.reprojection_rmse * e.reprojection_rmse);
    // Quality flag
    if (e.quality == "good")
      w *= 2.0;
    focals.push_back(e.focal_estimate);
    weights.push_back(w);
  }

  if ((int)focals.size() < min_pairs) {
    LOG(ERROR) << "Not enough good pairs (" << focals.size() << " < " << min_pairs << "). "
               << "Try lowering --rmse-thresh or running more pairs.";
    return 1;
  }

  const double f0 = weighted_median(focals, weights);
  LOG(WARNING) << "Phase-1 weighted-median focal = " << f0 << " px  (" << focals.size()
               << " pairs)";

  // ── Phase 2: global 1-D golden-section search ──────────────────────────
  // Load pixel correspondences for "good" pairs only
  int n_loaded = 0;
  for (auto& e : entries) {
    if (e.quality != "good")
      continue;
    if (load_tv_full(e))
      ++n_loaded;
  }
  LOG(WARNING) << "Loaded full correspondences for " << n_loaded << " pairs";

  double f_best = f0;
  double rmse_best = 1e9;

  if (n_loaded >= min_pairs) {
    // Golden-section in [f0/1.2, f0*1.2]
    const double phi = (std::sqrt(5.0) - 1.0) * 0.5;
    double lo = f0 / 1.2, hi = f0 * 1.2;
    double x1 = hi - phi * (hi - lo);
    double x2 = lo + phi * (hi - lo);
    double r1 = eval_focal(entries, x1, cx, cy, min_pts);
    double r2 = eval_focal(entries, x2, cx, cy, min_pts);
    for (int iter = 0; iter < 60; ++iter) {
      if (r1 < r2) {
        hi = x2;
        x2 = x1;
        r2 = r1;
        x1 = hi - phi * (hi - lo);
        r1 = eval_focal(entries, x1, cx, cy, min_pts);
      } else {
        lo = x1;
        x1 = x2;
        r1 = r2;
        x2 = lo + phi * (hi - lo);
        r2 = eval_focal(entries, x2, cx, cy, min_pts);
      }
      if ((hi - lo) < 0.5)
        break;
    }
    f_best = (lo + hi) * 0.5;
    rmse_best = eval_focal(entries, f_best, cx, cy, min_pts);
    LOG(WARNING) << "Phase-2 refined focal = " << f_best << " px  (rmse=" << rmse_best << " px)";
  } else {
    LOG(WARNING) << "Skipping Phase-2 (not enough loaded pairs). Using Phase-1 estimate.";
    rmse_best = eval_focal(entries, f_best, cx, cy, min_pts);
  }

  // ── Write K.json ──────────────────────────────────────────────────────────
  json K;
  K["fx"] = f_best;
  K["fy"] = f_best;
  K["cx"] = cx;
  K["cy"] = cy;
  K["rmse_px"] = rmse_best;
  K["num_pairs_used"] = n_loaded;
  K["method"] = "isat_calibrate";
  K["phase1_median_focal"] = f0;

  std::ofstream out_file(output_json);
  if (!out_file.is_open()) {
    LOG(ERROR) << "Cannot open output: " << output_json;
    print_event({{"type", "calibrate.complete"},
                {"ok", false},
                {"error", "cannot open output: " + output_json}});
    return 1;
  }
  out_file << K.dump(2) << "\n";
  print_event({{"type", "calibrate.complete"},
              {"ok", true},
              {"data",
               {{"output_json", output_json},
                {"fx", f_best},
                {"fy", f_best},
                {"cx", cx},
                {"cy", cy},
                {"rmse_px", rmse_best}}}});
  return 0;
}
