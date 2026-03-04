/**
 * isat_twoview.cpp
 * InsightAT – Two-View Geometric Reconstruction Tool
 *
 * Reads .isat_geo files (F, optional E, optional H from GPU RANSAC) and
 * degeneracy metadata, together with .isat_match pixel coordinates.
 * For every image pair:
 *
 *   1. Load F (always), E (if isat_geo had -k), H and degeneracy (if --estimate-h).
 *   2. Geometry correctness: use E, F, H and degeneracy to decide if pair is
 *      suitable for triangulation. If degeneracy indicates planar (H preferred),
 *      skip triangulation and mark degenerate.
 *   3. If intrinsics not supplied (-k): estimate focal f from F (Hartley σ₁≈σ₂).
 *   4. Build K; compute or use E; enforce essential constraint; decompose E→(R,t).
 *   5. GPU-triangulate inlier correspondences (DLT, Cholesky IPI).
 *   6. Evaluate Huber-weighted reprojection residuals on GPU; optionally compute
 *      stability metrics (parallax, depth/baseline) to assess geometry stability.
 *   7. Discard pairs with < --min-points valid 3-D points or unstable geometry.
 *   8. Write .isat_twoview (IDC): JSON (pose, focal, rmse, quality) + points3d, inlier_ids.
 *
 * Intrinsics from isat_project-exported JSON (-k). When known, focal is fixed.
 *
 * Pipeline (Stage/chain as isat_geo):
 *   Stage 1  [multi-thread I/O]   Load .isat_geo + .isat_match
 *   Stage 2  [main thread, EGL]   Geometry check → GPU triangulate + residuals (+ stability)
 *   Stage 3  [multi-thread I/O]   Write .isat_twoview
 *
 * Usage:
 *   isat_twoview -i pairs.json -g geo_dir/ -m match_dir/ -o twoview_dir/
 *   isat_twoview -i pairs.json -g geo_dir/ -m match_dir/ -o twoview_dir/ -k camera.json
 */

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <glog/logging.h>
#include <nlohmann/json.hpp>

#include "../io/idc_reader.h"
#include "../io/idc_writer.h"
#include "../modules/sfm/gpu_twoview_sfm.h"
#include "../modules/sfm/two_view_reconstruction.h"
#include "cli_logging.h"
#include "cmdLine/cmdLine.h"
#include "isat_intrinsics.h"
#include "pair_json_utils.h"
#include "task_queue/task_queue.hpp"

namespace fs = std::filesystem;
using json = nlohmann::json;
using namespace insight::io;
using namespace insight::sfm;

static constexpr const char* kEventPrefix = "ISAT_EVENT ";

static void printEvent(const json& j) {
  std::cout << kEventPrefix << j.dump() << "\n";
  std::cout.flush();
}

// ─────────────────────────────────────────────────────────────────────────────
// Per-pair task
// ─────────────────────────────────────────────────────────────────────────────

struct TwoViewTask {
  uint32_t image1_id = 0;
  uint32_t image2_id = 0;
  std::string geo_file;   // .isat_geo path
  std::string match_file; // .isat_match path
  int index = 0;
  uint32_t camera1_id = 1; ///< group_id of image1 (from image list)
  uint32_t camera2_id = 1; ///< group_id of image2 (from image list)

  // Stage 1 output (loaded from disk)
  float F[9] = {};
  float E[9] = {};
  float H[9] = {}; // from isat_geo when --estimate-h
  int F_inliers = 0;
  int E_inliers = 0;
  int H_inliers = 0;
  bool F_ok = false;
  bool E_ok = false;
  bool H_ok = false;

  /// F vs H degeneracy from isat_geo (or computed here when H loaded but not in meta)
  DegeneracyResult degeneracy;

  std::vector<float> coords_pixel; // [x1,y1,x2,y2] × n_matches
  std::vector<uint8_t> F_mask;
  std::vector<uint8_t> E_mask;
  std::vector<uint8_t> H_mask;
  int num_matches = 0;

  // Stage 2 output (reconstruction)
  TwoViewResult recon;
  bool processed = false;
};

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

static std::vector<TwoViewTask>
loadPairs(const std::string& json_path, const std::string& geo_dir, const std::string& match_dir,
          const std::unordered_map<uint32_t, uint32_t>& img_cam_map) {
  std::ifstream file(json_path);
  if (!file.is_open())
    LOG(FATAL) << "Cannot open pairs file: " << json_path;
  json j;
  file >> j;

  std::vector<TwoViewTask> tasks;
  int idx = 0;
  for (const auto& pair : j["pairs"]) {
    TwoViewTask t;
    t.image1_id = insight::tools::get_image_id_from_pair(pair, "image1_id");
    t.image2_id = insight::tools::get_image_id_from_pair(pair, "image2_id");
    t.geo_file = geo_dir + "/" + std::to_string(t.image1_id) + "_" + std::to_string(t.image2_id) +
                 ".isat_geo";
    t.match_file = match_dir + "/" + std::to_string(t.image1_id) + "_" +
                   std::to_string(t.image2_id) + ".isat_match";
    t.index = idx++;
    auto it1 = img_cam_map.find(t.image1_id);
    if (it1 != img_cam_map.end())
      t.camera1_id = it1->second;
    auto it2 = img_cam_map.find(t.image2_id);
    if (it2 != img_cam_map.end())
      t.camera2_id = it2->second;
    tasks.push_back(std::move(t));
  }
  LOG(INFO) << "Loaded " << tasks.size() << " pairs from " << json_path;
  return tasks;
}

static void loadGeoAndMatch(TwoViewTask& task) {
  // Load .isat_geo
  IDCReader geo(task.geo_file);
  if (!geo.is_valid()) {
    LOG(WARNING) << "Invalid .isat_geo: " << task.geo_file;
    return;
  }
  const auto& meta = geo.get_metadata();
  const auto& gm = meta["geometry"];

  task.F_ok = gm["F"]["estimated"].get<bool>();
  task.E_ok = gm["E"]["estimated"].get<bool>();
  task.F_inliers = task.F_ok ? gm["F"]["num_inliers"].get<int>() : 0;
  task.E_inliers = task.E_ok ? gm["E"]["num_inliers"].get<int>() : 0;

  if (!task.F_ok) {
    LOG(WARNING) << "No F in " << task.geo_file << ", skipping pair";
    return;
  }

  // Read camera IDs stored by isat_geo (present when isat_geo had -l)
  if (meta["image_pair"].contains("camera1_id"))
    task.camera1_id = meta["image_pair"]["camera1_id"].get<uint32_t>();
  if (meta["image_pair"].contains("camera2_id"))
    task.camera2_id = meta["image_pair"]["camera2_id"].get<uint32_t>();

  auto F_blob = geo.read_blob<float>("F_matrix");
  if (F_blob.size() == 9)
    std::copy(F_blob.begin(), F_blob.end(), task.F);
  else {
    task.F_ok = false;
    return;
  }

  auto Fm_blob = geo.read_blob<uint8_t>("F_inliers");
  task.F_mask = Fm_blob;

  if (task.E_ok) {
    auto E_blob = geo.read_blob<float>("E_matrix");
    if (E_blob.size() == 9)
      std::copy(E_blob.begin(), E_blob.end(), task.E);
    else
      task.E_ok = false;

    auto Em_blob = geo.read_blob<uint8_t>("E_inliers");
    task.E_mask = Em_blob;
  }

  // H and degeneracy (when isat_geo was run with --estimate-h)
  if (gm.contains("H")) {
    task.H_ok = gm["H"]["estimated"].get<bool>();
    task.H_inliers = task.H_ok ? gm["H"]["num_inliers"].get<int>() : 0;
    if (task.H_ok) {
      auto H_blob = geo.read_blob<float>("H_matrix");
      if (H_blob.size() == 9)
        std::copy(H_blob.begin(), H_blob.end(), task.H);
      else
        task.H_ok = false;
      if (task.H_ok) {
        auto Hm_blob = geo.read_blob<uint8_t>("H_inliers");
        task.H_mask = Hm_blob;
      }
    }
  }
  if (gm.contains("degeneracy")) {
    const auto& dg = gm["degeneracy"];
    task.degeneracy.is_degenerate = dg["is_degenerate"].get<bool>();
    task.degeneracy.model_preferred = dg["model_preferred"].get<int>();
    task.degeneracy.h_over_f_ratio = dg["h_over_f_ratio"].get<double>();
  } else if (task.H_ok && task.F_ok) {
    // Old .isat_geo without degeneracy in meta: compute from F/H inliers
    task.degeneracy = insight::sfm::detect_degeneracy(task.F_inliers, task.H_inliers, task.num_matches);
  }

  // Load .isat_match (pixel coords)
  IDCReader match(task.match_file);
  if (!match.is_valid()) {
    LOG(WARNING) << "Invalid .isat_match: " << task.match_file;
    task.F_ok = false;
    return;
  }
  task.coords_pixel = match.read_blob<float>("coords_pixel");
  task.num_matches = (int)task.coords_pixel.size() / 4;
}

static void writeTwoView(const TwoViewTask& task, const std::string& output_dir) {
  const TwoViewResult& r = task.recon;
  std::string out = output_dir + "/" + std::to_string(task.image1_id) + "_" +
                    std::to_string(task.image2_id) + ".isat_twoview";

  json meta;
  meta["schema_version"] = "1.0";
  meta["task_type"] = "two_view_reconstruction";
  meta["image_pair"]["image1_id"] = task.image1_id;
  meta["image_pair"]["image2_id"] = task.image2_id;
  meta["image_pair"]["camera1_id"] = task.camera1_id;
  meta["image_pair"]["camera2_id"] = task.camera2_id;
  meta["num_matches_input"] = task.num_matches;
  meta["num_points_3d"] = r.num_valid_points;
  meta["focal_length_px"] = r.focal_refined;
  meta["reprojection_rmse_px"] = r.reprojection_rmse;
  meta["quality"] = r.quality;
  meta["success"] = r.success;

  if (r.success) {
    // Store R as 9 doubles and t as 3 doubles in JSON for easy downstream use
    json pose;
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        pose["R"].push_back(r.R(i, j));
    for (int i = 0; i < 3; ++i)
      pose["t"].push_back(r.t[i]);
    meta["pose"] = pose;
  }

  IDCWriter writer(out);
  writer.set_metadata(meta);

  if (r.success && !r.points3d.empty()) {
    // Collect valid (inlier) points and their source indices
    std::vector<double> pts_flat;
    std::vector<int32_t> ids;
    for (int i = 0; i < (int)r.points3d.size(); ++i) {
      if (!std::isfinite(r.points3d[i][0]))
        continue;
      pts_flat.push_back(r.points3d[i][0]);
      pts_flat.push_back(r.points3d[i][1]);
      pts_flat.push_back(r.points3d[i][2]);
      ids.push_back(i < (int)r.inlier_ids.size() ? r.inlier_ids[i] : i);
    }
    if (!pts_flat.empty()) {
      const int Np = (int)pts_flat.size() / 3;
      writer.add_blob("points3d", pts_flat.data(), pts_flat.size() * sizeof(double), "float64",
                     {Np, 3});
      writer.add_blob("inlier_ids", ids.data(), ids.size() * sizeof(int32_t), "int32", {Np});
    }
  }

  if (!writer.write())
    LOG(ERROR) << "Failed to write: " << out;
  else
    VLOG(1) << "Wrote " << out;
}

// ─────────────────────────────────────────────────────────────────────────────
// Core reconstruction for one pair
// ─────────────────────────────────────────────────────────────────────────────

static TwoViewResult
reconstructPair(TwoViewTask& task, const CameraIntrinsics& K1, const CameraIntrinsics& K2,
                double f_hint, // 0 = auto-estimate from F (only when !K1.valid())
                int min_points, float huber_k) {
  TwoViewResult result;
  result.quality = "degenerate";

  // ── Step 0: geometry correctness – skip planar/degenerate pairs ─────────
  // When isat_geo estimated H and degeneracy says H preferred (planar scene),
  // F/E are unreliable; do not triangulate.
  if (task.degeneracy.is_degenerate && task.degeneracy.model_preferred == 1) {
    VLOG(1) << "  Pair " << task.image1_id << "-" << task.image2_id
            << ": degenerate (planar, H preferred), skip triangulation";
    return result;
  }

  const bool have_K = K1.valid() && K2.valid();
  const Eigen::Matrix3d F_mat = insight::sfm::float_array_to_matrix3d(task.F);

  // ── Step 1: use F inliers (consistent with E = K^T F K when K present) ──
  const std::vector<uint8_t>& inlier_mask_src = task.F_mask;
  const int n_inliers_src = task.F_inliers;

  if (n_inliers_src < min_points) {
    VLOG(1) << "  Pair " << task.image1_id << "-" << task.image2_id << ": only " << n_inliers_src
            << " inliers, skip";
    return result;
  }

  // Collect inlier pixel coordinates
  std::vector<Eigen::Vector2d> pts1_px_inl, pts2_px_inl;
  std::vector<int> inlier_ids;
  for (int i = 0; i < task.num_matches; ++i) {
    if (i >= (int)inlier_mask_src.size() || !inlier_mask_src[i])
      continue;
    pts1_px_inl.push_back({task.coords_pixel[i * 4 + 0], task.coords_pixel[i * 4 + 1]});
    pts2_px_inl.push_back({task.coords_pixel[i * 4 + 2], task.coords_pixel[i * 4 + 3]});
    inlier_ids.push_back(i);
  }
  if ((int)pts1_px_inl.size() < min_points) {
    VLOG(1) << "  Pair " << task.image1_id << "-" << task.image2_id << ": collected "
            << pts1_px_inl.size() << " inliers (< " << min_points << ")";
    return result;
  }

  // ── Step 2: determine focal length ────────────────────────────────────
  double f = f_hint;
  double cx = 0.0, cy = 0.0;

  if (have_K) {
    // Use K1 as reference focal (for CPU BA when K1==K2)
    f = K1.fx;
    cx = K1.cx;
    cy = K1.cy;
  } else if (f <= 0.0) {
    // Auto-estimate from F; use zero pp as fallback
    f = insight::sfm::focal_from_fundamental(F_mat, 0.0, 0.0);
    if (f <= 0.0) {
      LOG(WARNING) << "  focal_from_fundamental failed for pair " << task.image1_id << "-"
                   << task.image2_id;
      return result;
    }
    VLOG(1) << "  Estimated focal from F: f=" << f << " px";
  }

  // ── Step 3: compute E. Use F decomposition when K is available ───────
  // (When intrinsics are inaccurate, direct RANSAC E is worse; F then E = K^T F K is more robust.)
  Eigen::Matrix3d E_mat;
  if (have_K) {
    Eigen::Matrix3d K1m, K2m;
    K1m << K1.fx, 0, K1.cx, 0, K1.fy, K1.cy, 0, 0, 1;
    K2m << K2.fx, 0, K2.cx, 0, K2.fy, K2.cy, 0, 0, 1;
    E_mat = K1m.transpose() * F_mat * K2m;
    E_mat = insight::sfm::enforce_essential(E_mat);
  } else {
    Eigen::Matrix3d K;
    K << f, 0, cx, 0, f, cy, 0, 0, 1;
    E_mat = K.transpose() * F_mat * K;
    E_mat = insight::sfm::enforce_essential(E_mat);
  }

  // ── Step 4: normalise pixel coords (K1 for img1, K2 for img2) ────────
  const int Np_all = (int)pts1_px_inl.size();
  std::vector<Eigen::Vector2d> pts1_n(Np_all), pts2_n(Np_all);
  if (have_K) {
    for (int i = 0; i < Np_all; ++i) {
      pts1_n[i] = {(pts1_px_inl[i][0] - K1.cx) / K1.fx, (pts1_px_inl[i][1] - K1.cy) / K1.fy};
      pts2_n[i] = {(pts2_px_inl[i][0] - K2.cx) / K2.fx, (pts2_px_inl[i][1] - K2.cy) / K2.fy};
    }
  } else {
    for (int i = 0; i < Np_all; ++i) {
      pts1_n[i] = {(pts1_px_inl[i][0] - cx) / f, (pts1_px_inl[i][1] - cy) / f};
      pts2_n[i] = {(pts2_px_inl[i][0] - cx) / f, (pts2_px_inl[i][1] - cy) / f};
    }
  }

  // ── Step 5: decompose E → (R, t) ──────────────────────────────────────
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  const int n_cheirality = insight::sfm::decompose_essential(E_mat, pts1_n, pts2_n, R, t);
  if (n_cheirality < 4) {
    result.quality = "degenerate";
    return result;
  }

  // ── Step 6: GPU triangulation (normalised coords) ─────────────────────
  const int Np = Np_all;
  std::vector<float> pts_n_flat(4 * Np);
  for (int k = 0; k < Np; ++k) {
    pts_n_flat[k * 4 + 0] = (float)pts1_n[k][0];
    pts_n_flat[k * 4 + 1] = (float)pts1_n[k][1];
    pts_n_flat[k * 4 + 2] = (float)pts2_n[k][0];
    pts_n_flat[k * 4 + 3] = (float)pts2_n[k][1];
  }
  float R_f[9], t_f[3];
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      R_f[r * 3 + c] = (float)R(r, c);
  t_f[0] = (float)t[0];
  t_f[1] = (float)t[1];
  t_f[2] = (float)t[2];

  std::vector<float> X_gpu(3 * Np);
  const int n_tri = gpu_triangulate(pts_n_flat.data(), Np, R_f, t_f, X_gpu.data());
  if (n_tri < min_points) {
    result.quality = "poor";
    return result;
  }
  std::vector<Eigen::Vector3d> X_init(Np);
  for (int k = 0; k < Np; ++k)
    X_init[k] = {X_gpu[k * 3 + 0], X_gpu[k * 3 + 1], X_gpu[k * 3 + 2]};

  // ── Step 7: GPU residuals for quality evaluation (all pairs) ────────────
  // CPU LM-BA removed — GPU is the only BA path.
  // Use K1 focal for residual evaluation (exact for same-camera, approximation for cross-K).
  std::vector<float> pts_px_flat(4 * Np);
  for (int k = 0; k < Np; ++k) {
    pts_px_flat[k * 4 + 0] = (float)pts1_px_inl[k][0];
    pts_px_flat[k * 4 + 1] = (float)pts1_px_inl[k][1];
    pts_px_flat[k * 4 + 2] = (float)pts2_px_inl[k][0];
    pts_px_flat[k * 4 + 3] = (float)pts2_px_inl[k][1];
  }
  float wrss = 0.0f;
  int valid = 0;
  gpu_ba_residuals(pts_px_flat.data(), X_gpu.data(), Np, R_f, t_f, static_cast<float>(f),
                   static_cast<float>(cx), static_cast<float>(cy), huber_k, nullptr, &wrss, &valid);
  if (valid < min_points) {
    result.quality = "poor";
    return result;
  }
  result.success = true;
  result.R = R;
  result.t = t;
  result.focal_refined = f;
  result.reprojection_rmse = std::sqrt(wrss);
  result.num_valid_points = valid;
  result.inlier_ids = inlier_ids;
  result.points3d = X_init;

  // ── Step 8: stability (parallax, depth/baseline) ───────────────────────
  const StabilityMetrics stability =
      insight::sfm::compute_stability_metrics(X_init, pts1_n, pts2_n, R, t, 2.0, 2.0, 500.0);
  result.quality = (result.reprojection_rmse < 1.5 && stability.is_stable) ? "good" : "poor";
  return result;
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;

  CmdLine cmd("InsightAT Two-View Geometric Reconstruction\n"
              "  Reads .isat_geo (F/E from GPU RANSAC) + .isat_match pixel coords.\n"
              "  Loads camera intrinsics from isat_project-exported JSON (-k).\n"
              "  GPU-triangulates inlier correspondences + evaluates per-pair\n"
              "  Huber-weighted reprojection residuals on GPU.\n"
              "  Writes .isat_twoview: (R, t, 3-D points, focal estimate).\n"
              "\n"
              "  Feed the output into isat_calibrate to produce refined K.json,\n"
              "  then re-run isat_geo -k refined_K.json for better E matrices.");

  std::string pairs_json;
  std::string geo_dir, match_dir, output_dir;
  std::string intrinsics_json;
  std::string image_list_json;
  int min_points = 50;
  int num_threads = 4;
  double ba_loss_k = 4.0;
  double f_min = 100.0;
  double f_max = 50000.0;

  cmd.add(make_option('i', pairs_json, "input")
              .doc("Input pairs JSON (same schema as isat_retrieve / isat_geo output)"));
  cmd.add(make_option('g', geo_dir, "geo-dir").doc("Directory of .isat_geo files (from isat_geo)"));
  cmd.add(make_option('m', match_dir, "match-dir")
              .doc("Directory of .isat_match files (from isat_match)"));
  cmd.add(make_option('o', output_dir, "output").doc("Output directory for .isat_twoview files"));
  cmd.add(
      make_option('k', intrinsics_json, "intrinsics")
          .doc("Camera intrinsics JSON exported by isat_project.\n"
               "  Single : {\"fx\":f,\"fy\":f,\"cx\":cx,\"cy\":cy}\n"
               "  Multi  : {\"schema\":\"multi_camera_v1\",\"cameras\":{\"1\":{...},\"2\":{...}}}\n"
               "  When provided, GPU triangulation + Huber residuals are used.\n"
               "  -l/--image-list is required alongside -k."));
  cmd.add(make_option('l', image_list_json, "image-list")
              .doc("Image list JSON (from isat_project extract).\n"
                   "  Required when -k is given. Maps image_id to camera_id\n"
                   "  so that per-pair K1/K2 can be resolved."));
  cmd.add(make_option(0, min_points, "min-points")
              .doc("Minimum 3-D point count to accept a reconstruction. Default: 50"));
  cmd.add(make_option(0, ba_loss_k, "ba-loss-k")
              .doc("Huber loss threshold (pixels) for GPU residual evaluation. Default: 4.0"));
  cmd.add(make_option(0, f_min, "focal-min")
              .doc("Lower bound for focal search when -k not given. Default: 100 px"));
  cmd.add(make_option(0, f_max, "focal-max")
              .doc("Upper bound for focal search when -k not given. Default: 50000 px"));
  cmd.add(make_option('j', num_threads, "threads").doc("CPU threads for I/O stages. Default: 4"));
  std::string log_level;
  cmd.add(make_option(0, log_level, "log-level").doc("Log level: error|warn|info|debug"));
  cmd.add(make_switch('v', "verbose").doc("Verbose logging"));
  cmd.add(make_switch('q', "quiet").doc("Quiet (ERROR only)"));
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

  if (pairs_json.empty() || geo_dir.empty() || match_dir.empty() || output_dir.empty()) {
    std::cerr << "Error: -i, -g, -m, -o are required\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 1;
  }
  if (!intrinsics_json.empty() && image_list_json.empty()) {
    std::cerr << "Error: -l/--image-list is required when -k/--intrinsics is provided\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 1;
  }

  insight::tools::apply_log_level(cmd.used('v'), cmd.used('q'), log_level);

  // ── Load intrinsics map + image→camera map ───────────────────────────────────
  CameraIntrinsicsMap cam_intrinsics = load_intrinsics_map(intrinsics_json);
  const bool have_K = !cam_intrinsics.empty();

  std::unordered_map<uint32_t, uint32_t> img_cam_map;
  if (have_K) {
    img_cam_map = build_image_camera_map(image_list_json);
  }

  if (!have_K)
    LOG(WARNING) << "No intrinsics provided: focal length will be estimated per "
                    "pair from F.  For best results supply -k camera.json "
                    "(-a/--all for multi-camera) + -l image_list.json.";
  else
    LOG(INFO) << "Loaded " << cam_intrinsics.size() << " camera(s), " << img_cam_map.size()
              << " images mapped.";

  // ── Setup ────────────────────────────────────────────────────────────────
  if (!fs::is_directory(geo_dir)) {
    LOG(ERROR) << "geo-dir not found: " << geo_dir;
    return 1;
  }
  if (!fs::is_directory(match_dir)) {
    LOG(ERROR) << "match-dir not found: " << match_dir;
    return 1;
  }
  fs::create_directories(output_dir);

  auto tasks = loadPairs(pairs_json, geo_dir, match_dir, img_cam_map);
  const int total = (int)tasks.size();
  if (total == 0) {
    LOG(ERROR) << "No pairs to process";
    return 1;
  }

  LOG(WARNING) << "=== isat_twoview ===";
  LOG(WARNING) << "  Pairs:      " << total;
  LOG(WARNING) << "  Min points: " << min_points;
  LOG(WARNING) << "  Huber k:    " << ba_loss_k;
  LOG(WARNING) << "  Intrinsics: "
               << (have_K ? (std::to_string(cam_intrinsics.size()) + " camera(s) loaded")
                          : "auto-estimated from F");

  // ── Initialise GPU (EGL surfaceless + OpenGL 4.3 compute shaders) ────────
  if (gpu_twoview_init() != 0) {
    LOG(FATAL) << "Failed to initialise GPU two-view SfM "
                  "(EGL + OpenGL 4.3 required)";
    return 1;
  }

  // ── Stage 1: load geo + match (multi-thread I/O) ─────────────────────────
  const int IO_Q = 12;
  const int GPU_Q = 4;

  Stage loadStage("LoadGeoMatch", num_threads, IO_Q, [&](int i) { loadGeoAndMatch(tasks[i]); });

  // ── Stage 2: GPU triangulate + residuals  (main thread, EGL context) ─────────
  StageCurrent reconStage("ReconTV", 1, GPU_Q, [&](int i) {
    TwoViewTask& task = tasks[i];
    if (!task.F_ok)
      return;

    // Resolve per-pair intrinsics
    CameraIntrinsics K1, K2;
    if (have_K) {
      const CameraIntrinsics* pK1 = lookup_camera(cam_intrinsics, task.camera1_id);
      const CameraIntrinsics* pK2 = lookup_camera(cam_intrinsics, task.camera2_id);
      if (pK1)
        K1 = *pK1;
      if (pK2)
        K2 = *pK2;
    }

    const double f_hint = (!have_K) ? 0.0 : K1.fx;
    auto t0 = std::chrono::high_resolution_clock::now();
    task.recon = reconstructPair(task, K1, K2, f_hint, min_points, static_cast<float>(ba_loss_k));
    task.processed = true;

    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  std::chrono::high_resolution_clock::now() - t0)
                  .count();
    LOG(INFO) << "Recon [" << i << "/" << total << "] " << task.image1_id << "–" << task.image2_id
              << (task.recon.success ? ("  f=" + std::to_string((int)task.recon.focal_refined) +
                                        "  pts=" + std::to_string(task.recon.num_valid_points) +
                                        "  rmse=" + std::to_string(task.recon.reprojection_rmse) +
                                        "px  " + task.recon.quality)
                                     : "  FAILED")
              << "  " << ms << "ms";
  });

  // ── Stage 3: write .isat_twoview (multi-thread I/O) ─────────────────────
  Stage writeStage("WriteTV", num_threads, IO_Q, [&](int i) {
    if (!tasks[i].processed)
      return;
    writeTwoView(tasks[i], output_dir);
    // Free heavy memory
    tasks[i].coords_pixel.clear();
    tasks[i].coords_pixel.shrink_to_fit();
    float prog = static_cast<float>(i + 1) / total;
    std::cerr << "PROGRESS: " << prog << "\n";
  });

  // ── Chain and run (same as isat_geo) ─────────────────────────────────────
  chain(loadStage, reconStage);
  chain(reconStage, writeStage);
  loadStage.setTaskCount(total);
  reconStage.setTaskCount(total);
  writeStage.setTaskCount(total);

  auto t_start = std::chrono::high_resolution_clock::now();

  std::thread push_thread([&]() {
    for (int i = 0; i < total; ++i)
      loadStage.push(i);
  });

  // Main thread drives the GPU stage (EGL context is current here)
  reconStage.run();

  push_thread.join();
  loadStage.wait();
  writeStage.wait();

  auto total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::high_resolution_clock::now() - t_start)
                      .count();

  gpu_twoview_shutdown();

  // ── Summary (machine-readable stdout) ────────────────────────────────────
  int n_good = 0, n_poor = 0, n_fail = 0;
  double f_sum = 0.0;
  int f_cnt = 0;
  for (const auto& t : tasks) {
    if (!t.processed || !t.recon.success) {
      ++n_fail;
      continue;
    }
    if (t.recon.quality == "good")
      ++n_good;
    else
      ++n_poor;
    if (t.recon.focal_refined > 0.0) {
      f_sum += t.recon.focal_refined;
      ++f_cnt;
    }
  }

  json data = {{"total", total},   {"n_good", n_good},     {"n_poor", n_poor},
               {"n_fail", n_fail}, {"total_ms", total_ms}, {"output_dir", output_dir}};
  if (f_cnt > 0)
    data["mean_focal_px"] = f_sum / f_cnt;
  printEvent({{"type", "twoview.complete"}, {"ok", true}, {"data", data}});

  return 0;
}
