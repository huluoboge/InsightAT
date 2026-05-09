/**
 * isat_geo_cuda.cpp
 * InsightAT Two-View Geometry Estimation — Pure CUDA Pipeline
 *
 * A CUDA-accelerated alternative to isat_geo that keeps the ENTIRE algorithm
 * on the GPU: F RANSAC → E RANSAC → E decomposition → triangulation.
 * CPU handles only async I/O (Stage 1 read / Stage 3 write) and lightweight
 * post-processing (mask computation, degeneracy detection, stability metrics).
 *
 * Pipeline:
 *   Stage 1  [N CPU threads]   Read .isat_match → coords into GeoTask batch
 *   Stage 2  [main thread]     GPU batch:
 *                                F RANSAC (cuda_ransac_F_batch)
 *                                E RANSAC (cuda_ransac_E_batch, K-normalised)
 *                                H RANSAC (cuda_ransac_H_batch, degeneracy)
 *                                E decompose (cuda_decompose_E_batch)   ← NEW
 *                                triangulate (cuda_triangulate_batch)   ← NEW
 *   Stage 3  [N CPU threads]   Write .isat_geo
 *
 * Usage:
 *   isat_geo_cuda -i pairs.json -m match_dir/ -o geo_dir/ -l image_list.json
 *
 * Same output format (.isat_geo IDC) as isat_geo --twoview --backend gpu.
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>

#include <glog/logging.h>
#include <iostream>
#include <nlohmann/json.hpp>

#include "../io/idc_reader.h"
#include "../io/idc_writer.h"
#include "cli_logging.h"
#include "cmdLine/cmdLine.h"
#include "task_queue/task_queue.hpp"

#include "../modules/camera/camera_types.h"
#include "../modules/geometry/cuda_geo_ransac.h"
#include "../modules/sfm/two_view_reconstruction.h"
#include "pair_json_utils.h"
#include "tools/project_loader.h"

namespace fs = std::filesystem;
using json = nlohmann::json;
using namespace insight::io;
using namespace insight::sfm;

static constexpr const char* kEventPrefix = "ISAT_EVENT ";
static constexpr int kMinInliersForTrack = 4;
static constexpr int kMinCheir = 8; // minimum positive-depth count to accept (R,t)

static void print_event(const json& j) {
  std::cout << kEventPrefix << j.dump() << "\n";
  std::cout.flush();
}

// ─────────────────────────────────────────────────────────────────────────────
// Per-pair task (same fields as isat_geo.cpp)
// ─────────────────────────────────────────────────────────────────────────────

struct GeoTask {
  uint32_t image1_index = 0;
  uint32_t image2_index = 0;
  std::string match_file;
  int index = 0;

  // Stage 1 output
  std::vector<float> coords; // num_matches * 4 : [x1,y1,x2,y2] per match
  int num_matches = 0;

  // Stage 2 output
  float F[9] = {};
  float E[9] = {};
  float H[9] = {};
  int F_inliers = 0;
  int E_inliers = 0;
  int H_inliers = 0;
  std::vector<uint8_t> F_mask;
  std::vector<uint8_t> E_mask;
  std::vector<uint8_t> H_mask;
  bool F_ok = false;
  bool E_ok = false;
  bool H_ok = false;

  // Two-view pipeline output
  DegeneracyResult degeneracy;
  float R[9] = {};
  float t[3] = {};
  std::vector<float> points3d; // float[3 * num_valid_points]
  int num_valid_points = 0;
  StabilityMetrics stability;
  bool twoview_ok = false;

  // Scoring helpers
  double score_prelim = 0.0;
  double median_pixel_disp = 0.0;
  double inlier_ratio = 0.0;
};

// ─────────────────────────────────────────────────────────────────────────────
// Pairs loading
// ─────────────────────────────────────────────────────────────────────────────

static std::vector<GeoTask> load_pairs(const std::string& json_path,
                                        const std::string& match_dir) {
  std::ifstream file(json_path);
  if (!file.is_open())
    LOG(FATAL) << "Cannot open pairs file: " << json_path;
  json j;
  file >> j;

  std::vector<GeoTask> tasks;
  int idx = 0;
  for (const auto& pair : j["pairs"]) {
    GeoTask t;
    t.image1_index = insight::tools::get_image_index_from_pair(pair, "image1_index");
    t.image2_index = insight::tools::get_image_index_from_pair(pair, "image2_index");
    t.match_file = match_dir + "/" + std::to_string(t.image1_index) + "_" +
                   std::to_string(t.image2_index) + ".isat_match";
    t.index = idx++;
    tasks.push_back(std::move(t));
  }
  LOG(INFO) << "Loaded " << tasks.size() << " pairs from " << json_path;
  return tasks;
}

// ─────────────────────────────────────────────────────────────────────────────
// Stage 1: read .isat_match
// ─────────────────────────────────────────────────────────────────────────────

static void read_match_coords(GeoTask& task) {
  IDCReader reader(task.match_file);
  if (!reader.is_valid()) {
    LOG(WARNING) << "Invalid .isat_match: " << task.match_file;
    return;
  }
  task.coords = reader.read_blob<float>("coords_pixel");
  if (task.coords.empty()) {
    LOG(WARNING) << "Empty coords_pixel: " << task.match_file;
    return;
  }
  task.num_matches = static_cast<int>(task.coords.size() / 4);
}

// ─────────────────────────────────────────────────────────────────────────────
// CPU inlier mask helpers
// ─────────────────────────────────────────────────────────────────────────────

static float sampson_sq(const float F[9], float x1, float y1, float x2, float y2) {
  float Fx1x = F[0]*x1 + F[1]*y1 + F[2];
  float Fx1y = F[3]*x1 + F[4]*y1 + F[5];
  float Ftx2x = F[0]*x2 + F[3]*y2 + F[6];
  float Ftx2y = F[1]*x2 + F[4]*y2 + F[7];
  float r = x2*Fx1x + y2*Fx1y + (F[6]*x1 + F[7]*y1 + F[8]);
  float den = Fx1x*Fx1x + Fx1y*Fx1y + Ftx2x*Ftx2x + Ftx2y*Ftx2y;
  return (den < 1e-18f) ? 1e9f : (r * r) / den;
}

static float transfer_sq(const float H[9], float x1, float y1, float x2, float y2) {
  float hz = H[6]*x1 + H[7]*y1 + H[8];
  if (fabsf(hz) < 1e-9f) return 1e9f;
  float hx = (H[0]*x1 + H[1]*y1 + H[2]) / hz;
  float hy = (H[3]*x1 + H[4]*y1 + H[5]) / hz;
  float dx = hx - x2, dy = hy - y2;
  return dx*dx + dy*dy;
}

static std::vector<uint8_t> compute_f_mask(const float F[9], const std::vector<float>& coords,
                                            int n, float thresh_sq) {
  std::vector<uint8_t> m(static_cast<size_t>(n), 0);
  for (int i = 0; i < n; i++) {
    float x1 = coords[i*4+0], y1 = coords[i*4+1], x2 = coords[i*4+2], y2 = coords[i*4+3];
    m[static_cast<size_t>(i)] = (sampson_sq(F, x1, y1, x2, y2) < thresh_sq) ? 1u : 0u;
  }
  return m;
}

static std::vector<uint8_t> compute_e_mask(const float E[9], const std::vector<float>& coords,
                                            int n, const insight::camera::Intrinsics& K1,
                                            const insight::camera::Intrinsics& K2,
                                            float thresh_sq_norm) {
  std::vector<uint8_t> m(static_cast<size_t>(n), 0);
  for (int i = 0; i < n; i++) {
    float nx1 = static_cast<float>((coords[i*4+0] - K1.cx) / K1.fx);
    float ny1 = static_cast<float>((coords[i*4+1] - K1.cy) / K1.fy);
    float nx2 = static_cast<float>((coords[i*4+2] - K2.cx) / K2.fx);
    float ny2 = static_cast<float>((coords[i*4+3] - K2.cy) / K2.fy);
    m[static_cast<size_t>(i)] = (sampson_sq(E, nx1, ny1, nx2, ny2) < thresh_sq_norm) ? 1u : 0u;
  }
  return m;
}

static std::vector<uint8_t> compute_h_mask(const float H[9], const std::vector<float>& coords,
                                            int n, float thresh_sq) {
  std::vector<uint8_t> m(static_cast<size_t>(n), 0);
  for (int i = 0; i < n; i++) {
    float x1 = coords[i*4+0], y1 = coords[i*4+1], x2 = coords[i*4+2], y2 = coords[i*4+3];
    m[static_cast<size_t>(i)] = (transfer_sq(H, x1, y1, x2, y2) < thresh_sq) ? 1u : 0u;
  }
  return m;
}

static int count_inliers(const std::vector<uint8_t>& mask) {
  return static_cast<int>(std::accumulate(mask.begin(), mask.end(), 0));
}

// ─────────────────────────────────────────────────────────────────────────────
// Stage 3: write .isat_geo
// ─────────────────────────────────────────────────────────────────────────────

static void write_geo(const GeoTask& task, const std::string& output_dir, int ransac_iter) {
  const std::string out = output_dir + "/" + std::to_string(task.image1_index) + "_" +
                          std::to_string(task.image2_index) + ".isat_geo";

  json meta;
  meta["schema_version"] = "1.0";
  meta["task_type"] = "two_view_geometry";
  meta["algorithm"]["name"] = "isat_geo_cuda";
  meta["algorithm"]["solver"] = "Cholesky_IPI";
  meta["algorithm"]["fundamental_backend"] = "cuda";
  meta["algorithm"]["essential_backend"] = "cuda";
  meta["algorithm"]["homography_backend"] = "cuda";
  meta["algorithm"]["iterations"] = ransac_iter;
  meta["image_pair"]["image1_index"] = task.image1_index;
  meta["image_pair"]["image2_index"] = task.image2_index;
  meta["num_matches_input"] = task.num_matches;

  auto& gm = meta["geometry"];
  gm["F"]["estimated"] = task.F_ok;
  gm["F"]["num_inliers"] = task.F_inliers;
  gm["F"]["inlier_ratio"] =
      (task.num_matches > 0) ? static_cast<float>(task.F_inliers) / task.num_matches : 0.0f;
  gm["E"]["estimated"] = task.E_ok;
  gm["E"]["num_inliers"] = task.E_inliers;
  gm["E"]["inlier_ratio"] =
      (task.num_matches > 0) ? static_cast<float>(task.E_inliers) / task.num_matches : 0.0f;
  gm["H"]["estimated"] = task.H_ok;
  gm["H"]["num_inliers"] = task.H_inliers;
  gm["H"]["inlier_ratio"] =
      (task.num_matches > 0) ? static_cast<float>(task.H_inliers) / task.num_matches : 0.0f;
  gm["degeneracy"]["is_degenerate"] = task.degeneracy.is_degenerate;
  gm["degeneracy"]["model_preferred"] = task.degeneracy.model_preferred;
  gm["degeneracy"]["h_over_f_ratio"] = task.degeneracy.h_over_f_ratio;
  gm["score_prelim"] = task.score_prelim;
  gm["median_pixel_disp"] = task.median_pixel_disp;

  if (task.twoview_ok) {
    auto& tv = meta["twoview"];
    tv["stable"] = task.stability.is_stable;
    tv["num_valid_points"] = task.num_valid_points;
    tv["median_parallax_deg"] = task.stability.median_parallax_deg;
    tv["median_depth_baseline"] = task.stability.median_depth_baseline;
  }

  IDCWriter writer(out);
  writer.set_metadata(meta);

  if (task.F_ok)
    writer.add_blob("F_matrix", task.F, 9 * sizeof(float), "float32", {3, 3});
  if (static_cast<int>(task.F_mask.size()) == task.num_matches)
    writer.add_blob("F_inliers", task.F_mask.data(), task.num_matches, "uint8", {task.num_matches});
  if (task.E_ok)
    writer.add_blob("E_matrix", task.E, 9 * sizeof(float), "float32", {3, 3});
  if (static_cast<int>(task.E_mask.size()) == task.num_matches)
    writer.add_blob("E_inliers", task.E_mask.data(), task.num_matches, "uint8", {task.num_matches});
  if (task.H_ok)
    writer.add_blob("H_matrix", task.H, 9 * sizeof(float), "float32", {3, 3});
  if (static_cast<int>(task.H_mask.size()) == task.num_matches)
    writer.add_blob("H_inliers", task.H_mask.data(), task.num_matches, "uint8", {task.num_matches});
  if (task.twoview_ok) {
    writer.add_blob("R_matrix", task.R, 9 * sizeof(float), "float32", {3, 3});
    writer.add_blob("t_vector", task.t, 3 * sizeof(float), "float32", {3});
    writer.add_blob("points3d", task.points3d.data(),
                    task.points3d.size() * sizeof(float), "float32", {task.num_valid_points, 3});
  }
  if (!writer.write())
    LOG(ERROR) << "Failed to write: " << out;
  else
    VLOG(1) << "Wrote " << out;
}

// ─────────────────────────────────────────────────────────────────────────────
// Score computation (same formula as isat_geo.cpp)
// ─────────────────────────────────────────────────────────────────────────────

static void compute_score_prelim(GeoTask& task, bool estimate_E) {
  // Median pixel displacement
  std::vector<float> disps;
  disps.reserve(static_cast<size_t>(task.num_matches));
  for (int k = 0; k < task.num_matches; ++k) {
    float dx = task.coords[k*4+2] - task.coords[k*4+0];
    float dy = task.coords[k*4+3] - task.coords[k*4+1];
    disps.push_back(std::sqrt(dx*dx + dy*dy));
  }
  if (!disps.empty()) {
    std::nth_element(disps.begin(), disps.begin() + disps.size()/2, disps.end());
    task.median_pixel_disp = disps[disps.size()/2];
  }

  int best_inliers = task.F_inliers;
  if (estimate_E && task.E_inliers > best_inliers) best_inliers = task.E_inliers;
  task.inlier_ratio = task.num_matches > 0
      ? static_cast<double>(best_inliers) / task.num_matches : 0.0;

  const double w_f = 1.0, w_e = 1.0, w_tv = 2.0, w_st = 1.0, w_pt = 0.5, w_par = 2.0, w_ir = 1.0;
  double tin  = std::log(1.0 + static_cast<double>(std::max(0, best_inliers)));
  double tpt  = std::log(1.0 + static_cast<double>(task.num_valid_points));
  double tpar = (task.twoview_ok && task.stability.median_parallax_deg > 0.0)
                    ? std::tanh(task.stability.median_parallax_deg / 5.0)
                    : std::tanh(task.median_pixel_disp / 200.0);
  double tir  = std::min(1.0, task.inlier_ratio * 3.0);
  double flag_bonus = (task.E_ok ? 1.0 : 0.0) + (task.H_ok ? 0.5 : 0.0);
  task.score_prelim = w_f * tin + w_e * (task.E_ok ? 1.0 : 0.0) +
                      w_tv * (task.twoview_ok ? 1.0 : 0.0) +
                      w_st * (task.stability.is_stable ? 1.0 : 0.0) +
                      w_pt * tpt + w_par * tpar + w_ir * tir + 0.5 * flag_bonus;
}

// ─────────────────────────────────────────────────────────────────────────────
// GPU batch processing (Stage 2 core)
// ─────────────────────────────────────────────────────────────────────────────

// Wrapper that transparently sub-chunks any RANSAC batch call to stay within
// the CUDA kernel's compile-time limit (INSIGHTAT_CUDA_GEO_BATCH_MAX_PAIRS).
// This lets --batch control the I/O grouping granularity independently of the
// GPU kernel limit.  mat_out and inliers_out must already be sized for P items.
using RansacBatchFn = int (*)(const CudaGeoBatchItem*, int, float*, int*);
static int ransac_batch_chunked(RansacBatchFn fn, const CudaGeoBatchItem* items, int P,
                                float* mat_out, int* inliers_out) {
  constexpr int kMax = INSIGHTAT_CUDA_GEO_BATCH_MAX_PAIRS;
  for (int off = 0; off < P; off += kMax) {
    const int chunk = std::min(kMax, P - off);
    if (fn(items + off, chunk, mat_out + off * 9, inliers_out + off) != 0)
      return -1;
  }
  return 0;
}

static void process_gpu_batch(std::vector<GeoTask>& tasks, int batch_start, int batch_end,
                              const std::vector<insight::camera::Intrinsics>& img_K,
                              float thresh_f, float thresh_h, int min_inliers,
                              int min_points_twoview) {
  const int B = batch_end - batch_start;
  if (B <= 0) return;

  const float thresh_f_sq = thresh_f * thresh_f;
  const float thresh_h_sq = thresh_h * thresh_h;
  const bool estimate_E = !img_K.empty();

  // ── F RANSAC batch (compact: only pairs with num_matches >= 8) ──────────────
  {
    std::vector<int> f_batch_j;
    std::vector<std::vector<Match2D>> f_pts;
    std::vector<CudaGeoBatchItem> f_items;
    f_batch_j.reserve(static_cast<size_t>(B));
    f_pts.reserve(static_cast<size_t>(B));
    f_items.reserve(static_cast<size_t>(B));

    for (int j = 0; j < B; j++) {
      GeoTask& task = tasks[static_cast<size_t>(batch_start + j)];
      if (task.num_matches < 8) continue;
      f_pts.emplace_back(static_cast<size_t>(task.num_matches));
      for (int k = 0; k < task.num_matches; k++) {
        f_pts.back()[static_cast<size_t>(k)].x1 = task.coords[k*4+0];
        f_pts.back()[static_cast<size_t>(k)].y1 = task.coords[k*4+1];
        f_pts.back()[static_cast<size_t>(k)].x2 = task.coords[k*4+2];
        f_pts.back()[static_cast<size_t>(k)].y2 = task.coords[k*4+3];
      }
      CudaGeoBatchItem it{};
      it.matches = f_pts.back().data();
      it.n = task.num_matches;
      it.thresh_sq = thresh_f_sq;
      f_items.push_back(it);
      f_batch_j.push_back(j);
    }

    const int Pf = static_cast<int>(f_batch_j.size());
    if (Pf > 0) {
      std::vector<float> f_mats(static_cast<size_t>(Pf) * 9u);
      std::vector<int> f_inls(static_cast<size_t>(Pf));
      if (ransac_batch_chunked(cuda_ransac_F_batch, f_items.data(), Pf, f_mats.data(), f_inls.data()) != 0)
        LOG(FATAL) << "cuda_ransac_F_batch failed";    // Pf auto-chunked to <= INSIGHTAT_CUDA_GEO_BATCH_MAX_PAIRS
      for (int p = 0; p < Pf; p++) {
        const int j = f_batch_j[static_cast<size_t>(p)];
        GeoTask& task = tasks[static_cast<size_t>(batch_start + j)];
        for (int c = 0; c < 9; c++) task.F[c] = f_mats[static_cast<size_t>(p)*9u + c];
        task.F_inliers = f_inls[static_cast<size_t>(p)];
        task.F_ok = (task.F_inliers >= min_inliers);
        if (task.F_ok || task.F_inliers >= kMinInliersForTrack)
          task.F_mask = compute_f_mask(task.F, task.coords, task.num_matches, thresh_f_sq);
      }
    }
  }

  // ── E RANSAC batch (K-normalised coords) ────────────────────────────────────
  // Collect per-pair K and normalised pts for each pair that has valid K
  auto intrinsic_valid = [](const insight::camera::Intrinsics& K) {
    return K.fx > 1e-6 && K.fy > 1e-6;
  };

  std::vector<int> e_batch_j;        // batch-local indices with valid K
  std::vector<std::vector<Match2D>> e_norm(static_cast<size_t>(B)); // normalised pts per pair
  std::vector<float> e_thresh_sq(static_cast<size_t>(B), 0.0f);
  std::vector<CudaGeoBatchItem> e_items;
  std::vector<const insight::camera::Intrinsics*> e_K1(static_cast<size_t>(B), nullptr);
  std::vector<const insight::camera::Intrinsics*> e_K2(static_cast<size_t>(B), nullptr);

  if (estimate_E) {
    for (int j = 0; j < B; j++) {
      GeoTask& task = tasks[static_cast<size_t>(batch_start + j)];
      if (task.num_matches < 5) continue;
      const int i1 = static_cast<int>(task.image1_index);
      const int i2 = static_cast<int>(task.image2_index);
      if (i1 < 0 || i1 >= static_cast<int>(img_K.size()) ||
          i2 < 0 || i2 >= static_cast<int>(img_K.size())) continue;
      const insight::camera::Intrinsics& K1 = img_K[static_cast<size_t>(i1)];
      const insight::camera::Intrinsics& K2 = img_K[static_cast<size_t>(i2)];
      if (!intrinsic_valid(K1) || !intrinsic_valid(K2)) continue;

      e_K1[static_cast<size_t>(j)] = &K1;
      e_K2[static_cast<size_t>(j)] = &K2;

      e_norm[static_cast<size_t>(j)].resize(static_cast<size_t>(task.num_matches));
      for (int k = 0; k < task.num_matches; k++) {
        e_norm[static_cast<size_t>(j)][static_cast<size_t>(k)].x1 =
            static_cast<float>((task.coords[k*4+0] - K1.cx) / K1.fx);
        e_norm[static_cast<size_t>(j)][static_cast<size_t>(k)].y1 =
            static_cast<float>((task.coords[k*4+1] - K1.cy) / K1.fy);
        e_norm[static_cast<size_t>(j)][static_cast<size_t>(k)].x2 =
            static_cast<float>((task.coords[k*4+2] - K2.cx) / K2.fx);
        e_norm[static_cast<size_t>(j)][static_cast<size_t>(k)].y2 =
            static_cast<float>((task.coords[k*4+3] - K2.cy) / K2.fy);
      }

      const double f_avg = std::sqrt(K1.fx * K1.fy * K2.fx * K2.fy);
      const float te_norm = static_cast<float>(thresh_f / std::sqrt(f_avg));
      e_thresh_sq[static_cast<size_t>(j)] = te_norm * te_norm;

      e_batch_j.push_back(j);
      CudaGeoBatchItem it{};
      it.matches = e_norm[static_cast<size_t>(j)].data();
      it.n = task.num_matches;
      it.thresh_sq = e_thresh_sq[static_cast<size_t>(j)];
      e_items.push_back(it);
    }

    const int Pe = static_cast<int>(e_batch_j.size());
    if (Pe > 0) {
      std::vector<float> e_mats(static_cast<size_t>(Pe) * 9u);
      std::vector<int> e_inls(static_cast<size_t>(Pe));
      if (ransac_batch_chunked(cuda_ransac_E_batch, e_items.data(), Pe, e_mats.data(), e_inls.data()) != 0)
        LOG(FATAL) << "cuda_ransac_E_batch failed";    // Pe auto-chunked
      for (int p = 0; p < Pe; p++) {
        const int j = e_batch_j[static_cast<size_t>(p)];
        GeoTask& task = tasks[static_cast<size_t>(batch_start + j)];
        for (int c = 0; c < 9; c++) task.E[c] = e_mats[static_cast<size_t>(p)*9u + c];
        task.E_inliers = e_inls[static_cast<size_t>(p)];
        task.E_ok = (task.E_inliers >= min_inliers);
        if ((task.E_ok || task.E_inliers >= kMinInliersForTrack) && e_K1[static_cast<size_t>(j)])
          task.E_mask = compute_e_mask(task.E, task.coords, task.num_matches,
                                       *e_K1[static_cast<size_t>(j)],
                                       *e_K2[static_cast<size_t>(j)],
                                       e_thresh_sq[static_cast<size_t>(j)]);
      }
    }
  }

  // ── H RANSAC batch (for degeneracy detection) ───────────────────────────────
  {
    std::vector<CudaGeoBatchItem> h_items;
    std::vector<std::vector<Match2D>> h_pts(static_cast<size_t>(B));
    std::vector<int> h_batch_j;
    for (int j = 0; j < B; j++) {
      GeoTask& task = tasks[static_cast<size_t>(batch_start + j)];
      if (task.num_matches < 4) continue;
      h_pts[static_cast<size_t>(j)].resize(static_cast<size_t>(task.num_matches));
      for (int k = 0; k < task.num_matches; k++) {
        h_pts[static_cast<size_t>(j)][static_cast<size_t>(k)].x1 = task.coords[k*4+0];
        h_pts[static_cast<size_t>(j)][static_cast<size_t>(k)].y1 = task.coords[k*4+1];
        h_pts[static_cast<size_t>(j)][static_cast<size_t>(k)].x2 = task.coords[k*4+2];
        h_pts[static_cast<size_t>(j)][static_cast<size_t>(k)].y2 = task.coords[k*4+3];
      }
      h_batch_j.push_back(j);
      CudaGeoBatchItem it{};
      it.matches = h_pts[static_cast<size_t>(j)].data();
      it.n = task.num_matches;
      it.thresh_sq = thresh_h_sq;
      h_items.push_back(it);
    }
    const int Ph = static_cast<int>(h_batch_j.size());
    if (Ph > 0) {
      std::vector<float> h_mats(static_cast<size_t>(Ph) * 9u);
      std::vector<int> h_inls(static_cast<size_t>(Ph));
      if (ransac_batch_chunked(cuda_ransac_H_batch, h_items.data(), Ph, h_mats.data(), h_inls.data()) != 0)
        LOG(FATAL) << "cuda_ransac_H_batch failed";    // Ph auto-chunked
      for (int p = 0; p < Ph; p++) {
        const int j = h_batch_j[static_cast<size_t>(p)];
        GeoTask& task = tasks[static_cast<size_t>(batch_start + j)];
        for (int c = 0; c < 9; c++) task.H[c] = h_mats[static_cast<size_t>(p)*9u + c];
        task.H_inliers = h_inls[static_cast<size_t>(p)];
        task.H_ok = (task.H_inliers >= min_inliers);
        if (task.H_ok || task.H_inliers >= kMinInliersForTrack)
          task.H_mask = compute_h_mask(task.H, task.coords, task.num_matches, thresh_h_sq);
      }
    }
  }

  // ── E decompose + triangulate (GPU) ───────────────────────────────────────
  // Collect pairs eligible: E_ok with valid K, non-degenerate (checked after degeneracy step)
  // Build flat arrays for the batch CUDA calls

  // First: degeneracy detection (CPU, fast) so we can skip degenerate pairs
  for (int j = 0; j < B; j++) {
    GeoTask& task = tasks[static_cast<size_t>(batch_start + j)];
    task.degeneracy = detect_degeneracy(task.F_inliers, task.H_inliers, task.num_matches);
  }

  // Now build decomp batch: only E_ok + non-degenerate + has K
  std::vector<int> decomp_j; // batch-local indices eligible for E decompose
  std::vector<float> d_E_mats;
  std::vector<float> d_pts_flat; // sum_n * 4 floats (normalised, float4 layout)
  std::vector<uint8_t> d_masks_flat;
  std::vector<int> d_pair_off; // prefix sums, B+1 elements
  std::vector<int> d_pair_n;

  d_pair_off.push_back(0);

  for (int j = 0; j < B; j++) {
    GeoTask& task = tasks[static_cast<size_t>(batch_start + j)];
    if (!task.E_ok || task.degeneracy.is_degenerate) continue;
    if (!e_K1[static_cast<size_t>(j)]) continue;
    if (static_cast<int>(task.E_mask.size()) != task.num_matches) continue;

    decomp_j.push_back(j);
    d_E_mats.insert(d_E_mats.end(), task.E, task.E + 9);

    // Append normalised pts (float4 layout: x1,y1,x2,y2)
    const auto& en = e_norm[static_cast<size_t>(j)];
    for (int k = 0; k < task.num_matches; k++) {
      d_pts_flat.push_back(en[static_cast<size_t>(k)].x1);
      d_pts_flat.push_back(en[static_cast<size_t>(k)].y1);
      d_pts_flat.push_back(en[static_cast<size_t>(k)].x2);
      d_pts_flat.push_back(en[static_cast<size_t>(k)].y2);
    }
    d_masks_flat.insert(d_masks_flat.end(), task.E_mask.begin(), task.E_mask.end());
    d_pair_n.push_back(task.num_matches);
    d_pair_off.push_back(d_pair_off.back() + task.num_matches);
  }

  const int Pd = static_cast<int>(decomp_j.size());
  if (Pd > 0) {
    std::vector<float> R_batch(static_cast<size_t>(Pd) * 9u);
    std::vector<float> t_batch(static_cast<size_t>(Pd) * 3u);
    std::vector<int>   cheir_batch(static_cast<size_t>(Pd));

    if (cuda_decompose_E_batch(d_E_mats.data(), Pd, d_pts_flat.data(), d_pair_off.data(),
                                d_pair_n.data(), d_masks_flat.data(),
                                R_batch.data(), t_batch.data(), cheir_batch.data()) != 0)
      LOG(FATAL) << "cuda_decompose_E_batch failed";

    // Assign R, t to tasks with enough positive-depth support
    for (int p = 0; p < Pd; p++) {
      const int j = decomp_j[static_cast<size_t>(p)];
      GeoTask& task = tasks[static_cast<size_t>(batch_start + j)];
      if (cheir_batch[static_cast<size_t>(p)] >= kMinCheir) {
        for (int c = 0; c < 9; c++) task.R[c] = R_batch[static_cast<size_t>(p)*9u + c];
        task.t[0] = t_batch[static_cast<size_t>(p)*3u + 0];
        task.t[1] = t_batch[static_cast<size_t>(p)*3u + 1];
        task.t[2] = t_batch[static_cast<size_t>(p)*3u + 2];
      } else {
        // Remove from triangulation: mark pair not eligible
        // We'll check task.R validity via a flag instead of removing from batch
        decomp_j[static_cast<size_t>(p)] = -1; // mark invalid
      }
    }

    // Triangulate all eligible pairs
    const int sum_n = d_pair_off.back();
    std::vector<float> X_flat(static_cast<size_t>(sum_n) * 3u, 0.0f);
    std::vector<int>   valid_cnt(static_cast<size_t>(Pd), 0);

    if (cuda_triangulate_batch(R_batch.data(), t_batch.data(), Pd, d_pts_flat.data(),
                                d_pair_off.data(), d_pair_n.data(), d_masks_flat.data(),
                                X_flat.data(), valid_cnt.data()) != 0)
      LOG(FATAL) << "cuda_triangulate_batch failed";

    // Collect 3D points into tasks + compute stability metrics (CPU)
    for (int p = 0; p < Pd; p++) {
      const int j = decomp_j[static_cast<size_t>(p)];
      if (j < 0) continue; // invalid (not enough cheir)
      GeoTask& task = tasks[static_cast<size_t>(batch_start + j)];

      const int off = d_pair_off[static_cast<size_t>(p)];
      const int n   = d_pair_n[static_cast<size_t>(p)];

      // Collect valid 3D points and corresponding normalised 2D pts for stability
      std::vector<Eigen::Vector3d> pts3d;
      std::vector<Eigen::Vector2d> pts1_n_valid, pts2_n_valid;
      pts3d.reserve(static_cast<size_t>(valid_cnt[static_cast<size_t>(p)]));
      pts1_n_valid.reserve(pts3d.capacity());
      pts2_n_valid.reserve(pts3d.capacity());

      const auto& en = e_norm[static_cast<size_t>(j)];
      for (int k = 0; k < n; k++) {
        const float x = X_flat[static_cast<size_t>(off + k) * 3u + 0];
        const float y = X_flat[static_cast<size_t>(off + k) * 3u + 1];
        const float z = X_flat[static_cast<size_t>(off + k) * 3u + 2];
        if (!std::isfinite(x)) continue;
        pts3d.emplace_back(static_cast<double>(x), static_cast<double>(y), static_cast<double>(z));
        pts1_n_valid.emplace_back(static_cast<double>(en[static_cast<size_t>(k)].x1),
                                   static_cast<double>(en[static_cast<size_t>(k)].y1));
        pts2_n_valid.emplace_back(static_cast<double>(en[static_cast<size_t>(k)].x2),
                                   static_cast<double>(en[static_cast<size_t>(k)].y2));
      }

      task.num_valid_points = static_cast<int>(pts3d.size());
      if (task.num_valid_points < 10 || task.num_valid_points < min_points_twoview) continue;

      // Stability metrics on CPU (needs Eigen points)
      Eigen::Matrix3d Rm;
      Eigen::Vector3d tv;
      for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++)
        Rm(r, c) = static_cast<double>(task.R[r*3+c]);
      tv << static_cast<double>(task.t[0]),
            static_cast<double>(task.t[1]),
            static_cast<double>(task.t[2]);

      task.stability = compute_stability_metrics(pts3d, pts1_n_valid, pts2_n_valid, Rm, tv);

      if (task.stability.is_stable && task.num_valid_points >= min_points_twoview) {
        task.twoview_ok = true;
        task.points3d.resize(static_cast<size_t>(task.num_valid_points) * 3u);
        for (int k = 0; k < task.num_valid_points; k++) {
          task.points3d[static_cast<size_t>(k)*3u+0] = static_cast<float>(pts3d[static_cast<size_t>(k)].x());
          task.points3d[static_cast<size_t>(k)*3u+1] = static_cast<float>(pts3d[static_cast<size_t>(k)].y());
          task.points3d[static_cast<size_t>(k)*3u+2] = static_cast<float>(pts3d[static_cast<size_t>(k)].z());
        }
        VLOG(1) << "TwoView [" << task.index << "] " << task.image1_index << "–"
                << task.image2_index << "  pts=" << task.num_valid_points
                << "  parallax=" << task.stability.median_parallax_deg << "°";
      }
    }
  }

  // ── Score and cleanup ───────────────────────────────────────────────────────
  for (int j = 0; j < B; j++) {
    GeoTask& task = tasks[static_cast<size_t>(batch_start + j)];
    compute_score_prelim(task, estimate_E);
    // Free match coords (no longer needed after GPU processing)
    task.coords.clear();
    task.coords.shrink_to_fit();
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;

  // ── CLI ──────────────────────────────────────────────────────────────────
  CmdLine cmd("InsightAT Two-View Geometry Estimation (Pure CUDA Pipeline)\n"
              "  F RANSAC → E RANSAC → E decompose → triangulate — all on GPU.\n"
              "  Output format identical to: isat_geo --backend gpu --twoview");

  std::string pairs_json, match_dir, output_dir, image_list_json;
  float thresh_f = 2.0f;
  float thresh_h = 2.25f;
  int min_inliers = 20;
  int min_points = 50;
  int ransac_iter = 2000;
  int num_threads = 4;
  int batch_size  = 10000;  // I/O grouping granularity; CUDA kernel auto-chunks at INSIGHTAT_CUDA_GEO_BATCH_MAX_PAIRS
  int cuda_device = 0;

  cmd.add(make_option('i', pairs_json, "input").doc("Input pairs JSON"));
  cmd.add(make_option('m', match_dir, "match-dir").doc("Directory with .isat_match files"));
  cmd.add(make_option('o', output_dir, "output").doc("Output directory for .isat_geo files"));
  cmd.add(make_option('l', image_list_json, "image-list")
              .doc("Image list JSON (cameras + images[].camera_index) for per-image K"));
  cmd.add(make_option('t', thresh_f, "thresh").doc("Inlier threshold pixels (F & E). Default: 2.0"));
  cmd.add(make_option('T', thresh_h, "thresh-h").doc("Inlier threshold for H. Default: 2.25"));
  cmd.add(make_option(0, min_inliers, "min-inliers").doc("Min inliers to accept geometry. Default: 20"));
  cmd.add(make_option(0, min_points, "min-points").doc("Min 3D points for twoview output. Default: 50"));
  cmd.add(make_option('n', ransac_iter, "iterations").doc("RANSAC iterations per pair. Default: 2000"));
  cmd.add(make_option('j', num_threads, "threads").doc("CPU I/O threads. Default: 4"));
  cmd.add(make_option(0, batch_size, "batch")
              .doc("I/O grouping batch size (pairs loaded per round). Default: 10000. "
                   "CUDA kernel auto-chunks internally at INSIGHTAT_CUDA_GEO_BATCH_MAX_PAIRS "
                   "so any value is safe; larger values use more RAM but reduce load overhead."));
  cmd.add(make_option(0, cuda_device, "cuda-device").doc("CUDA device id. Default: 0"));
  std::string log_level;
  cmd.add(make_option(0, log_level, "log-level").doc("Log level: error|warn|info|debug"));
  cmd.add(make_switch('v', "verbose").doc("Verbose (INFO)"));
  cmd.add(make_switch('q', "quiet").doc("Quiet (ERROR only)"));
  cmd.add(make_switch('h', "help").doc("Show help"));

  try { cmd.process(argc, argv); }
  catch (const std::string& s) {
    std::cerr << "Error: " << s << "\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 2;
  }
  if (cmd.checkHelp(argv[0])) return 0;
  cmd.print(std::cerr);

  if (pairs_json.empty() || match_dir.empty() || output_dir.empty()) {
    std::cerr << "Error: -i, -m, -o are required\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 2;
  }

  insight::tools::apply_log_level(cmd.used('v'), cmd.used('q'), log_level);

  // ── Load intrinsics by image index ──────────────────────────────────────
  std::vector<insight::camera::Intrinsics> image_index_K;
  if (!image_list_json.empty()) {
    insight::tools::ProjectData proj;
    if (insight::tools::load_project_data(image_list_json, &proj) && proj.num_images() > 0) {
      image_index_K.resize(static_cast<size_t>(proj.num_images()));
      for (int i = 0; i < proj.num_images(); ++i) {
        int c = proj.image_to_camera_index[static_cast<size_t>(i)];
        if (c >= 0 && c < proj.num_cameras())
          image_index_K[static_cast<size_t>(i)] = proj.cameras[static_cast<size_t>(c)];
      }
    }
  }
  const bool estimate_E = !image_index_K.empty();

  LOG(INFO) << "=== isat_geo_cuda configuration ===";
  LOG(INFO) << "  Pairs JSON   : " << pairs_json;
  LOG(INFO) << "  Match dir    : " << match_dir;
  LOG(INFO) << "  Output dir   : " << output_dir;
  LOG(INFO) << "  RANSAC iter  : " << ransac_iter;
  LOG(INFO) << "  Threshold F  : " << thresh_f << " px";
  LOG(INFO) << "  Estimate E   : " << (estimate_E ? "yes" : "no (pass -l for K)");
  LOG(INFO) << "  Batch size   : " << batch_size;
  LOG(INFO) << "  CPU threads  : " << num_threads;
  LOG(INFO) << "  CUDA device  : " << cuda_device;

  if (!fs::is_directory(match_dir)) { LOG(ERROR) << "Match dir not found: " << match_dir; return 1; }
  fs::create_directories(output_dir);

  std::vector<GeoTask> tasks = load_pairs(pairs_json, match_dir);
  const int total = static_cast<int>(tasks.size());
  if (total == 0) { LOG(ERROR) << "No pairs to process"; return 1; }
  const int num_batches_preview = (total + batch_size - 1) / batch_size;
  LOG(INFO) << "  Total pairs  : " << total
            << "  batches=" << num_batches_preview
            << "  (batch_size=" << batch_size << ")";

  // ── Init CUDA ────────────────────────────────────────────────────────────
  cuda_geo_set_device(cuda_device);
  GeoRansacConfig cfg;
  cfg.num_iterations = ransac_iter;
  cfg.local_size_x = 256;
  if (cuda_geo_init(&cfg) != 0) {
    LOG(FATAL) << "cuda_geo_init failed";
    return 1;
  }
  cuda_geo_set_solver(1); // Cholesky IPI

  // ── Three-stage pipeline, scheduling unit = batch_idx ────────────────────
  //
  //  LoadStage [1 worker, uses inner Stage for parallel I/O per batch]
  //       ↓ chain  (bounded queue depth 2 → up to 2 pre-loaded batches queued)
  //  GpuStage  [main thread, StageCurrent — CUDA context pinned here]
  //       ↓ chain  (bounded queue depth 4)
  //  WriteStage [num_threads workers — disk I/O overlaps with next load+GPU]
  //
  // This mirrors the pattern in isat_gpu_cascade_hashing_match:
  //   - LoadStage completes loading an entire batch before handing it to GpuStage.
  //   - GpuStage never sees a partially-loaded batch.
  //   - WriteStage and the next LoadStage run concurrently.

  const int num_batches = (total + batch_size - 1) / batch_size;
  std::atomic<int> batch_done{0};

  // ── Stage 1: LoadStage ────────────────────────────────────────────────────
  // One outer worker serialises job dispatch; an inner Stage parallelises file
  // I/O across all tasks within the batch.  The outer worker blocks on
  // inner_load.wait() → acts as a barrier before LoadStage marks the job done
  // and chain() pushes it to GpuStage.
  Stage load_stage(
      "GeoLoadBatch", 1, /*capacity=*/2,
      [&tasks, &batch_size, &total, &num_threads](int batch_idx) {
        const int bs = batch_idx * batch_size;
        const int be = std::min(bs + batch_size, total);
        const int n  = be - bs;
        Stage inner_load(
            "GeoInnerLoad", num_threads, /*capacity=*/n + 4,
            [&tasks, bs](int j) {
              read_match_coords(tasks[static_cast<size_t>(bs + j)]);
            });
        inner_load.setTaskCount(n);
        for (int j = 0; j < n; ++j) inner_load.push(j);
        inner_load.wait(); // barrier: entire batch loaded before returning
      });

  // ── Stage 2: GpuStage (main thread) ──────────────────────────────────────
  StageCurrent gpu_stage(
      "GeoGPU", 1, /*capacity=*/2,
      [&tasks, &batch_size, &total, &num_batches, &batch_done, &image_index_K,
       thresh_f, thresh_h, min_inliers, min_points](int batch_idx) {
        const int bs = batch_idx * batch_size;
        const int be = std::min(bs + batch_size, total);
        const int cur = batch_done.fetch_add(1, std::memory_order_relaxed) + 1;
        LOG(INFO) << "[batch " << cur << "/" << num_batches << "] GPU processing "
                  << "pairs [" << bs << ".." << (be - 1) << "]  count=" << (be - bs);
        const auto t0 = std::chrono::high_resolution_clock::now();
        process_gpu_batch(tasks, bs, be, image_index_K, thresh_f, thresh_h,
                          min_inliers, min_points);
        const long ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::high_resolution_clock::now() - t0).count();
        LOG(INFO) << "[batch " << cur << "/" << num_batches << "] done  " << ms << " ms";
      });

  // ── Stage 3: WriteStage ───────────────────────────────────────────────────
  // Iterates all pairs within the batch; with num_threads workers and queue
  // depth 4, up to 4 completed batches can be queued for writing, keeping
  // disk I/O alive while GpuStage + LoadStage work on the next batch.
  Stage write_stage(
      "GeoWrite", num_threads, /*capacity=*/4,
      [&tasks, &batch_size, &total, &output_dir, ransac_iter](int batch_idx) {
        const int bs = batch_idx * batch_size;
        const int be = std::min(bs + batch_size, total);
        for (int i = bs; i < be; ++i) {
          const GeoTask& task = tasks[static_cast<size_t>(i)];
          if (!task.F_ok && !task.E_ok && !task.H_ok) continue;
          write_geo(task, output_dir, ransac_iter);
        }
      });

  chain(load_stage, gpu_stage);
  chain(gpu_stage, write_stage);

  load_stage.setTaskCount(num_batches);
  gpu_stage.setTaskCount(num_batches);
  write_stage.setTaskCount(num_batches);

  auto t_start = std::chrono::high_resolution_clock::now();

  // Push batch indices from a background thread so the main thread can run
  // GpuStage (CUDA context must stay on the main thread).
  std::thread push_thread([&]() {
    for (int bi = 0; bi < num_batches; ++bi)
      load_stage.push(bi);
  });

  gpu_stage.run(); // blocks until all batches are GPU-processed

  push_thread.join();
  load_stage.wait();
  write_stage.wait();

  cuda_geo_shutdown();

  auto t_end = std::chrono::high_resolution_clock::now();
  const long total_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();

  // ── Summary ──────────────────────────────────────────────────────────────
  int pairs_F = 0, pairs_E = 0, pairs_H = 0, pairs_tv = 0;
  int sum_F = 0, sum_E = 0, sum_H = 0;
  for (const auto& t : tasks) {
    if (t.F_ok) { pairs_F++; sum_F += t.F_inliers; }
    if (t.E_ok) { pairs_E++; sum_E += t.E_inliers; }
    if (t.H_ok) { pairs_H++; sum_H += t.H_inliers; }
    if (t.twoview_ok) pairs_tv++;
  }

  json ev_done;
  ev_done["type"] = "geo_cuda.done";
  ev_done["ok"] = true;
  ev_done["data"]["total_pairs"] = total;
  ev_done["data"]["pairs_F"] = pairs_F;
  ev_done["data"]["pairs_E"] = pairs_E;
  ev_done["data"]["pairs_H"] = pairs_H;
  ev_done["data"]["pairs_twoview"] = pairs_tv;
  ev_done["data"]["avg_F_inliers"] = (pairs_F > 0) ? sum_F / pairs_F : 0;
  ev_done["data"]["avg_E_inliers"] = (pairs_E > 0) ? sum_E / pairs_E : 0;
  ev_done["data"]["elapsed_ms"] = total_ms;
  print_event(ev_done);

  LOG(INFO) << "=== Done ===  " << total_ms << " ms";
  LOG(INFO) << "  F success: " << pairs_F << " / " << total;
  LOG(INFO) << "  E success: " << pairs_E << " / " << total;
  LOG(INFO) << "  H success: " << pairs_H << " / " << total;
  LOG(INFO) << "  Twoview  : " << pairs_tv << " / " << total;
  return 0;
}
