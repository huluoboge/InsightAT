/**
 * isat_geo.cpp
 * InsightAT Two-View Geometry Estimation Tool
 *
 * Reads .isat_match files produced by isat_match, runs GPU RANSAC for
 * Fundamental matrix (always), Essential matrix (if -k intrinsics are given),
 * and optionally Homography (--estimate-h), then writes .isat_geo files.
 *
 * Pipeline (CPU–GPU async, same Stage/chain pattern as isat_match):
 *   Stage 1  [multi-thread I/O]  Read .isat_match → coords_pixel
 *   Stage 2  [main thread, EGL]  GPU RANSAC F [E] [H]
 *   Stage 3  [multi-thread I/O]  Write .isat_geo
 *
 * Output .isat_geo (IDC format):
 *   JSON metadata    : schema, algorithm, pair IDs, per-model inlier stats
 *   F_matrix         : float32[9]   always present
 *   F_inliers        : uint8[n]     per-match inlier mask
 *   E_matrix         : float32[9]   when -k provided
 *   E_inliers        : uint8[n]     when -k provided
 *   H_matrix         : float32[9]   when --estimate-h
 *   H_inliers        : uint8[n]     when --estimate-h
 *
 * Adjacency matrix visualization (--vis):
 *   match_graph_F.png  inlier heatmap for Fundamental matrix (always)
 *   match_graph_E.png  inlier heatmap for Essential matrix  (if -k given)
 *   match_graph_H.png  inlier heatmap for Homography        (if --estimate-h)
 *
 * Usage:
 *   isat_geo -i pairs.json -m match_dir/ -o geo_dir/
 *   isat_geo -i pairs.json -m match_dir/ -o geo_dir/ -k camera.json
 *   isat_geo -i pairs.json -m match_dir/ -o geo_dir/ --estimate-h
 */

#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <numeric>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <PoseLib/robust.h>

#include <glog/logging.h>
#include <iostream>
#include <nlohmann/json.hpp>

#include "../io/idc_reader.h"
#include "../io/idc_writer.h"
#include "cli_logging.h"
#include "cmdLine/cmdLine.h"
#include "task_queue/task_queue.hpp"

#include "../modules/camera/camera_types.h"
#include "../modules/geometry/gpu_geo_ransac.h"
#include "../modules/sfm/gpu_twoview_sfm.h"
#include "../modules/sfm/two_view_reconstruction.h"
#include "pair_json_utils.h"
#include "tools/project_loader.h"

namespace fs = std::filesystem;
using json = nlohmann::json;
using namespace insight::io;
using namespace insight::sfm;

static constexpr const char* kEventPrefix = "ISAT_EVENT ";

enum class FundamentalBackend {
  kGpu,
  kPoseLib,
};

enum class EssentialBackend {
  kGpu,
  kPoseLib,
};

enum class HomographyBackend {
  kGpu,
  kPoseLib,
};

static const char* fundamental_backend_name(FundamentalBackend backend) {
  switch (backend) {
  case FundamentalBackend::kGpu:
    return "gpu";
  case FundamentalBackend::kPoseLib:
    return "poselib";
  }
  return "unknown";
}

static const char* essential_backend_name(EssentialBackend backend) {
  switch (backend) {
  case EssentialBackend::kGpu:
    return "gpu";
  case EssentialBackend::kPoseLib:
    return "poselib";
  }
  return "unknown";
}

static const char* homography_backend_name(HomographyBackend backend) {
  switch (backend) {
  case HomographyBackend::kGpu:
    return "gpu";
  case HomographyBackend::kPoseLib:
    return "poselib";
  }
  return "unknown";
}

static void print_event(const json& j) {
  std::cout << kEventPrefix << j.dump() << "\n";
  std::cout.flush();
}

// ─────────────────────────────────────────────────────────────────────────────
// Per-pair task
// ─────────────────────────────────────────────────────────────────────────────

struct GeoTask {
  uint32_t image1_index = 0;
  uint32_t image2_index = 0;
  std::string match_file;
  int index = 0;

  // Stage 1 output ─────────────────────────────────────────────────────────
  // Flat [x1,y1,x2,y2] per match (coords_pixel blob)
  std::vector<float> coords; // size = num_matches * 4
  int num_matches = 0;

  // Stage 2 output ─────────────────────────────────────────────────────────
  float F[9] = {};
  float E[9] = {};
  float H[9] = {};
  int F_inliers = 0;
  int E_inliers = 0;
  int H_inliers = 0;
  std::vector<uint8_t> F_mask; // per-match: 1=inlier
  std::vector<uint8_t> E_mask;
  std::vector<uint8_t> H_mask;
  bool F_ok = false;
  bool E_ok = false;
  bool H_ok = false;

  // Two-view pipeline (--twoview): degeneracy, E→R,t, triangulation, stability
  DegeneracyResult degeneracy;
  float R[9] = {};
  float t[3] = {};
  std::vector<float> points3d; // float[3*n] for valid points
  int num_valid_points = 0;
  StabilityMetrics stability;
  bool twoview_ok = false;
};

/// Minimum inlier count to still write inlier mask to IDC for track building (degenerate pairs keep
/// inlier data)
static constexpr int kMinInliersForTrack = 4;

// ─────────────────────────────────────────────────────────────────────────────
// Pairs loading (reuses same JSON schema as isat_match / isat_retrieve)
// ─────────────────────────────────────────────────────────────────────────────

static std::vector<GeoTask> load_pairs(const std::string& json_path, const std::string& match_dir) {
  std::ifstream file(json_path);
  if (!file.is_open()) {
    LOG(FATAL) << "Cannot open pairs file: " << json_path;
  }
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
// Stage 1 helper – read coords_pixel from .isat_match
// ─────────────────────────────────────────────────────────────────────────────

static void read_match_coords(GeoTask& task) {
  IDCReader reader(task.match_file);
  if (!reader.is_valid()) {
    LOG(WARNING) << "Invalid .isat_match file: " << task.match_file;
    return;
  }

  // coords_pixel blob: float32[N, 4]  →  [x1, y1, x2, y2] per row
  task.coords = reader.read_blob<float>("coords_pixel");
  if (task.coords.empty()) {
    LOG(WARNING) << "Empty coords_pixel in: " << task.match_file;
    return;
  }
  task.num_matches = static_cast<int>(task.coords.size() / 4);
  VLOG(1) << "Read " << task.num_matches << " matches from " << task.match_file;
}

// ─────────────────────────────────────────────────────────────────────────────
// CPU inlier mask helpers  (applied after GPU returns best matrix)
// ─────────────────────────────────────────────────────────────────────────────

// Squared Sampson distance for F/E:  x2ᵀ F x1 = 0
static float sampson_sq(const float F[9], float x1, float y1, float x2, float y2) {
  // Fx1
  float fx = F[0] * x1 + F[1] * y1 + F[2];
  float fy = F[3] * x1 + F[4] * y1 + F[5];
  // Fᵀx2
  float ftx = F[0] * x2 + F[3] * y2 + F[6];
  float fty = F[1] * x2 + F[4] * y2 + F[7];

  float num = F[6] * x1 * x2 + F[7] * y1 * x2 + F[8] * x2 + F[3] * x1 * y2 + F[4] * y1 * y2 +
              F[5] * y2 + F[0] * x1 + F[1] * y1 + F[2];
  // Use direct formulation: num = (x2, y2, 1) · F · (x1, y1, 1)
  float r = (F[0] * x1 + F[1] * y1 + F[2]) * x2 + (F[3] * x1 + F[4] * y1 + F[5]) * y2 +
            (F[6] * x1 + F[7] * y1 + F[8]);
  float den = fx * fx + fy * fy + ftx * ftx + fty * fty;
  (void)num;
  return (den < 1e-18f) ? 1e9f : (r * r) / den;
}

// Squared forward-transfer error for H:  H·x1 ≈ x2
static float transfer_sq(const float H[9], float x1, float y1, float x2, float y2) {
  float hz = H[6] * x1 + H[7] * y1 + H[8];
  if (fabsf(hz) < 1e-9f)
    return 1e9f;
  float hx = (H[0] * x1 + H[1] * y1 + H[2]) / hz;
  float hy = (H[3] * x1 + H[4] * y1 + H[5]) / hz;
  float dx = hx - x2, dy = hy - y2;
  return dx * dx + dy * dy;
}

static std::vector<uint8_t> compute_f_mask(const float F[9], const std::vector<float>& coords,
                                           int n, float thresh_sq) {
  std::vector<uint8_t> mask(n, 0);
  for (int i = 0; i < n; i++) {
    float x1 = coords[i * 4 + 0], y1 = coords[i * 4 + 1];
    float x2 = coords[i * 4 + 2], y2 = coords[i * 4 + 3];
    mask[i] = (sampson_sq(F, x1, y1, x2, y2) < thresh_sq) ? 1 : 0;
  }
  return mask;
}

static std::vector<uint8_t> compute_h_mask(const float H[9], const std::vector<float>& coords,
                                           int n, float thresh_sq) {
  std::vector<uint8_t> mask(n, 0);
  for (int i = 0; i < n; i++) {
    float x1 = coords[i * 4 + 0], y1 = coords[i * 4 + 1];
    float x2 = coords[i * 4 + 2], y2 = coords[i * 4 + 3];
    mask[i] = (transfer_sq(H, x1, y1, x2, y2) < thresh_sq) ? 1 : 0;
  }
  return mask;
}

// For E: Sampson on K-normalised coordinates (K1 for image1, K2 for image2)
static std::vector<uint8_t> compute_e_mask(const float E[9], const std::vector<float>& coords,
                                           int n, const insight::camera::Intrinsics& K1,
                                           const insight::camera::Intrinsics& K2,
                                           float thresh_sq_norm) {
  std::vector<uint8_t> mask(n, 0);
  for (int i = 0; i < n; i++) {
    float x1 = static_cast<float>((coords[i * 4 + 0] - K1.cx) / K1.fx);
    float y1 = static_cast<float>((coords[i * 4 + 1] - K1.cy) / K1.fy);
    float x2 = static_cast<float>((coords[i * 4 + 2] - K2.cx) / K2.fx);
    float y2 = static_cast<float>((coords[i * 4 + 3] - K2.cy) / K2.fy);
    mask[i] = (sampson_sq(E, x1, y1, x2, y2) < thresh_sq_norm) ? 1 : 0;
  }
  return mask;
}

static int count_inliers(const std::vector<uint8_t>& mask) {
  return static_cast<int>(std::accumulate(mask.begin(), mask.end(), 0));
}

static void matrix3d_to_float9(const Eigen::Matrix3d& M, float out[9]) {
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      out[r * 3 + c] = static_cast<float>(M(r, c));
}

static Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d& t) {
  Eigen::Matrix3d tx;
  tx << 0.0, -t.z(), t.y(), t.z(), 0.0, -t.x(), -t.y(), t.x(), 0.0;
  return tx;
}

static bool estimate_fundamental_poselib(const std::vector<float>& coords, int num_matches,
                                         float thresh_px, int ransac_iter, float F_out[9],
                                         std::vector<uint8_t>* mask_out, int* inliers_out) {
  if (!F_out || !mask_out || !inliers_out || num_matches < 8 ||
      static_cast<int>(coords.size()) < num_matches * 4) {
    return false;
  }

  std::vector<poselib::Point2D> x1;
  std::vector<poselib::Point2D> x2;
  x1.reserve(static_cast<size_t>(num_matches));
  x2.reserve(static_cast<size_t>(num_matches));
  for (int i = 0; i < num_matches; ++i) {
    x1.emplace_back(static_cast<double>(coords[static_cast<size_t>(i) * 4 + 0]),
                    static_cast<double>(coords[static_cast<size_t>(i) * 4 + 1]));
    x2.emplace_back(static_cast<double>(coords[static_cast<size_t>(i) * 4 + 2]),
                    static_cast<double>(coords[static_cast<size_t>(i) * 4 + 3]));
  }

  poselib::RelativePoseOptions opt;
  opt.max_error = std::max(0.25f, thresh_px);
  opt.ransac.max_iterations = static_cast<size_t>(std::max(100, ransac_iter));
  opt.ransac.min_iterations =
      static_cast<size_t>(std::min(std::max(50, ransac_iter / 4), std::max(100, ransac_iter)));
  opt.bundle.max_iterations = 25;
  opt.bundle.loss_type = poselib::BundleOptions::CAUCHY;
  opt.bundle.loss_scale = std::max(1.0f, thresh_px);

  Eigen::Matrix3d F = Eigen::Matrix3d::Zero();
  std::vector<char> poselib_inliers;
  const poselib::RansacStats stats =
      poselib::estimate_fundamental(x1, x2, opt, &F, &poselib_inliers);
  (void)stats;

  if (!F.allFinite())
    return false;

  matrix3d_to_float9(F, F_out);
  *mask_out = compute_f_mask(F_out, coords, num_matches, thresh_px * thresh_px);
  *inliers_out = count_inliers(*mask_out);
  return *inliers_out > 0;
}

static bool estimate_essential_poselib(const std::vector<float>& coords, int num_matches,
                                       const insight::camera::Intrinsics& K1,
                                       const insight::camera::Intrinsics& K2, float thresh_px,
                                       int ransac_iter, float E_out[9],
                                       std::vector<uint8_t>* mask_out, int* inliers_out) {
  if (!E_out || !mask_out || !inliers_out || num_matches < 5 ||
      static_cast<int>(coords.size()) < num_matches * 4) {
    return false;
  }

  std::vector<poselib::Point2D> x1;
  std::vector<poselib::Point2D> x2;
  x1.reserve(static_cast<size_t>(num_matches));
  x2.reserve(static_cast<size_t>(num_matches));
  for (int i = 0; i < num_matches; ++i) {
    x1.emplace_back(static_cast<double>(coords[static_cast<size_t>(i) * 4 + 0]),
                    static_cast<double>(coords[static_cast<size_t>(i) * 4 + 1]));
    x2.emplace_back(static_cast<double>(coords[static_cast<size_t>(i) * 4 + 2]),
                    static_cast<double>(coords[static_cast<size_t>(i) * 4 + 3]));
  }

  poselib::RelativePoseOptions opt;
  opt.max_error = std::max(0.25f, thresh_px);
  opt.ransac.max_iterations = static_cast<size_t>(std::max(100, ransac_iter));
  opt.ransac.min_iterations =
      static_cast<size_t>(std::min(std::max(50, ransac_iter / 4), std::max(100, ransac_iter)));
  opt.bundle.max_iterations = 25;
  opt.bundle.loss_type = poselib::BundleOptions::CAUCHY;
  opt.bundle.loss_scale = std::max(1.0f, thresh_px);

  poselib::Camera camera1(poselib::CameraModelId::PINHOLE, {K1.fx, K1.fy, K1.cx, K1.cy});
  poselib::Camera camera2(poselib::CameraModelId::PINHOLE, {K2.fx, K2.fy, K2.cx, K2.cy});
  poselib::CameraPose pose;
  std::vector<char> poselib_inliers;
  const poselib::RansacStats stats =
      poselib::estimate_relative_pose(x1, x2, camera1, camera2, opt, &pose, &poselib_inliers);
  (void)stats;

  const Eigen::Matrix3d E = skew_symmetric(pose.t) * pose.R();
  if (!E.allFinite())
    return false;

  matrix3d_to_float9(E, E_out);
  const double f_avg = std::sqrt(K1.fx * K1.fy * K2.fx * K2.fy);
  const float thresh_e_norm = static_cast<float>(thresh_px / std::sqrt(f_avg));
  const float thresh_e_sq = thresh_e_norm * thresh_e_norm;
  *mask_out = compute_e_mask(E_out, coords, num_matches, K1, K2, thresh_e_sq);
  *inliers_out = count_inliers(*mask_out);
  return *inliers_out > 0;
}

static bool estimate_homography_poselib(const std::vector<float>& coords, int num_matches,
                                        float thresh_px, int ransac_iter, float H_out[9],
                                        std::vector<uint8_t>* mask_out, int* inliers_out) {
  if (!H_out || !mask_out || !inliers_out || num_matches < 4 ||
      static_cast<int>(coords.size()) < num_matches * 4) {
    return false;
  }

  std::vector<poselib::Point2D> x1;
  std::vector<poselib::Point2D> x2;
  x1.reserve(static_cast<size_t>(num_matches));
  x2.reserve(static_cast<size_t>(num_matches));
  for (int i = 0; i < num_matches; ++i) {
    x1.emplace_back(static_cast<double>(coords[static_cast<size_t>(i) * 4 + 0]),
                    static_cast<double>(coords[static_cast<size_t>(i) * 4 + 1]));
    x2.emplace_back(static_cast<double>(coords[static_cast<size_t>(i) * 4 + 2]),
                    static_cast<double>(coords[static_cast<size_t>(i) * 4 + 3]));
  }

  poselib::HomographyOptions opt;
  opt.max_error = std::max(0.25f, thresh_px);
  opt.ransac.max_iterations = static_cast<size_t>(std::max(100, ransac_iter));
  opt.ransac.min_iterations =
      static_cast<size_t>(std::min(std::max(50, ransac_iter / 4), std::max(100, ransac_iter)));
  opt.bundle.max_iterations = 25;
  opt.bundle.loss_type = poselib::BundleOptions::CAUCHY;
  opt.bundle.loss_scale = std::max(1.0f, thresh_px);

  Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
  std::vector<char> poselib_inliers;
  const poselib::RansacStats stats =
      poselib::estimate_homography(x1, x2, opt, &H, &poselib_inliers);
  (void)stats;

  if (!H.allFinite())
    return false;

  matrix3d_to_float9(H, H_out);
  *mask_out = compute_h_mask(H_out, coords, num_matches, thresh_px * thresh_px);
  *inliers_out = count_inliers(*mask_out);
  return *inliers_out > 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// Stage 3 helper – write .isat_geo
// ─────────────────────────────────────────────────────────────────────────────

static void write_geo(const GeoTask& task, const std::string& output_dir, int ransac_iter,
                      FundamentalBackend f_backend, EssentialBackend e_backend,
                      HomographyBackend h_backend) {
  std::string out = output_dir + "/" + std::to_string(task.image1_index) + "_" +
                    std::to_string(task.image2_index) + ".isat_geo";

  // ── JSON metadata ──────────────────────────────────────────────────────
  json meta;
  meta["schema_version"] = "1.0";
  meta["task_type"] = "two_view_geometry";
  meta["algorithm"]["name"] = "isat_geo";
  meta["algorithm"]["solver"] = "Cholesky_IPI";
  meta["algorithm"]["fundamental_backend"] = fundamental_backend_name(f_backend);
  meta["algorithm"]["essential_backend"] = essential_backend_name(e_backend);
  meta["algorithm"]["homography_backend"] = homography_backend_name(h_backend);
  meta["algorithm"]["iterations"] = ransac_iter;
  meta["image_pair"]["image1_index"] = task.image1_index;
  meta["image_pair"]["image2_index"] = task.image2_index;
  meta["num_matches_input"] = task.num_matches;

  auto& gm = meta["geometry"];
  gm["F"]["estimated"] = task.F_ok;
  gm["F"]["num_inliers"] = task.F_inliers; // always actual count (degenerate is only a flag)
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
    writer.add_blob("points3d", task.points3d.data(), task.points3d.size() * sizeof(float),
                    "float32", {task.num_valid_points, 3});
  }

  if (!writer.write()) {
    LOG(ERROR) << "Failed to write: " << out;
  } else {
    VLOG(1) << "Wrote " << out;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Visualization: match connectivity graph as a heatmap image
// ─────────────────────────────────────────────────────────────────────────────

// inlier_fn(task) → {ok, inlier_count}
using InlierFn = std::function<std::pair<bool, int>(const GeoTask&)>;

static void write_match_matrix(const std::vector<GeoTask>& tasks, const std::string& output_path,
                               const std::string& model_label, InlierFn inlier_fn) {
  // Collect all unique image IDs
  std::set<uint32_t> id_set;
  for (const auto& t : tasks) {
    id_set.insert(t.image1_index);
    id_set.insert(t.image2_index);
  }
  std::vector<uint32_t> ids(id_set.begin(), id_set.end());
  std::sort(ids.begin(), ids.end());

  const int N = static_cast<int>(ids.size());
  if (N == 0)
    return;

  std::unordered_map<uint32_t, int> id_idx;
  for (int i = 0; i < N; i++)
    id_idx[ids[static_cast<size_t>(i)]] = i;

  // Build symmetric inlier-count matrix using the provided extractor
  cv::Mat mat(N, N, CV_32F, cv::Scalar(0.0f));
  int max_inliers = 0;
  int pairs_ok = 0;
  for (const auto& t : tasks) {
    auto [ok, count] = inlier_fn(t);
    if (!ok)
      continue;
    int i = id_idx.at(t.image1_index);
    int j = id_idx.at(t.image2_index);
    float val = static_cast<float>(count);
    mat.at<float>(i, j) = val;
    mat.at<float>(j, i) = val;
    if (count > max_inliers)
      max_inliers = count;
    pairs_ok++;
  }

  if (pairs_ok == 0) {
    LOG(WARNING) << "No valid " << model_label << " pairs to visualise, skipping " << output_path;
    return;
  }

  // Normalise to [0, 255], then apply VIRIDIS colormap
  cv::Mat norm8;
  mat.convertTo(norm8, CV_8U, max_inliers > 0 ? 255.0 / max_inliers : 0.0);

  cv::Mat colored;
  cv::applyColorMap(norm8, colored, cv::COLORMAP_VIRIDIS);

  // Force zero-inlier cells to black (distinguishable from low counts)
  colored.setTo(cv::Scalar(0, 0, 0), (norm8 == 0));

  // Scale up so each cell is visible (target ~1000 px wide)
  const int cell = std::max(1, std::min(16, 1000 / N));
  cv::Mat big;
  cv::resize(colored, big, cv::Size(N * cell, N * cell), 0, 0, cv::INTER_NEAREST);

  // Draw a simple colorbar + label strip on the right side
  const int cb_w = 40;
  const int cb_h = big.rows;
  cv::Mat colorbar(cb_h, cb_w, CV_8UC3);
  for (int row = 0; row < cb_h; row++) {
    uint8_t v = static_cast<uint8_t>(255 - row * 255 / cb_h);
    cv::Mat strip = colorbar.row(row);
    strip.setTo(cv::Scalar(v, v, v));
  }
  cv::applyColorMap(colorbar, colorbar, cv::COLORMAP_VIRIDIS);
  cv::Mat legend(cb_h, cb_w + 60, CV_8UC3, cv::Scalar(30, 30, 30));
  colorbar.copyTo(legend(cv::Rect(0, 0, cb_w, cb_h)));
  cv::putText(legend, std::to_string(max_inliers), {cb_w + 4, 14}, cv::FONT_HERSHEY_SIMPLEX, 0.4,
              cv::Scalar(200, 200, 200), 1);
  cv::putText(legend, "0", {cb_w + 4, cb_h - 4}, cv::FONT_HERSHEY_SIMPLEX, 0.4,
              cv::Scalar(200, 200, 200), 1);
  // Model label at top
  cv::putText(legend, model_label, {cb_w + 4, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.4,
              cv::Scalar(180, 255, 180), 1);

  cv::Mat canvas;
  cv::hconcat(big, legend, canvas);

  if (!cv::imwrite(output_path, canvas)) {
    LOG(ERROR) << "Failed to write match matrix image: " << output_path;
  } else {
    LOG(WARNING) << "Match graph [" << model_label << "] saved: " << output_path << "  (" << N
                 << "x" << N << " images"
                 << ", pairs=" << pairs_ok << ", max_inliers=" << max_inliers << ")";
  }
}

// Derive output path: replace/append model suffix before extension
static std::string make_vis_path(const std::string& base_path, const std::string& suffix) {
  fs::path p(base_path);
  std::string stem = p.stem().string();     // e.g. "match_graph"
  std::string ext = p.extension().string(); // e.g. ".png"
  return (p.parent_path() / (stem + "_" + suffix + ext)).string();
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;

  // ── Command-line options ─────────────────────────────────────────────────
  CmdLine cmd("InsightAT Two-View Geometry Estimation\n"
              "  Estimates F (always), E (if -k given), and optionally H.\n"
              "  Input .isat_match files are read from -m dir; output .isat_geo\n"
              "  files are written to -o dir.");

  std::string pairs_json;
  std::string match_dir;
  std::string output_dir;
  std::string intrinsics_json;
  std::string image_list_json;
  float thresh_f = 2.0f;
  float thresh_h = 2.25f;
  // int min_inliers = 8;
  int min_inliers = 20;
  int min_points_twoview = 50;
  int ransac_iter = 1000;
  int num_threads = 4;
  std::string backend_str = "poselib";

  cmd.add(make_option('i', pairs_json, "input")
              .doc("Input pairs JSON (same format as isat_retrieve / isat_match output)"));
  cmd.add(make_option('m', match_dir, "match-dir")
              .doc("Directory containing .isat_match files from isat_match.\n"
                   "  File names are derived as {dir}/{id1}_{id2}.isat_match"));
  cmd.add(make_option('o', output_dir, "output").doc("Output directory for .isat_geo files"));
  cmd.add(
      make_option('k', intrinsics_json, "intrinsics")
          .doc("Camera intrinsics JSON. Supports two formats:\n"
               "  Single : {\"fx\":f, \"fy\":f, \"cx\":cx, \"cy\":cy}\n"
               "  Multi  : {\"schema\":\"multi_camera_v1\",\"cameras\":{\"1\":{...},\"2\":{...}}}\n"
               "  When provided (together with -l), E matrix is estimated per pair.\n"
               "  See: isat_project intrinsics [-a] for export."));
  cmd.add(make_option('l', image_list_json, "image-list")
              .doc("Image list JSON from isat_project extract (must have 'cameras' and "
                   "images[].camera_index).\n"
                   "  Used to resolve per-pair K by image index for E estimation."));
  cmd.add(make_option('t', thresh_f, "thresh-f")
              .doc("Inlier threshold for F (squared Sampson, pixels). Default: 2.0"));
  cmd.add(make_option('T', thresh_h, "thresh-h")
              .doc("Inlier threshold for H (forward transfer, pixels). Default: 2.25.\n"
                   "  Only used when --estimate-h is set."));
  cmd.add(make_option(0, min_inliers, "min-inliers")
              .doc("Minimum inlier count required to write a result. Default: 8"));
  cmd.add(make_option('n', ransac_iter, "iterations")
              .doc("RANSAC iterations per pair (GPU). Default: 1000"));
  cmd.add(make_option('j', num_threads, "threads")
              .doc("CPU I/O threads for read/write stages. Default: 4"));
  cmd.add(make_option(0, backend_str, "backend")
              .doc("Geometry backend for F/E/H together: gpu|poselib|cpu. Default: gpu\n"
                   "  cpu is an alias of poselib."));
  cmd.add(make_switch(0, "estimate-h")
              .doc("Also estimate Homography H (disabled by default).\n"
                   "  Useful for planar scenes or pure-rotation image pairs."));
  cmd.add(make_switch(0, "twoview")
              .doc("Run two-view reconstruction: degeneracy check, E→R,t, GPU triangulation,\n"
                   "  stability metrics. Requires -k/-l. Implies --estimate-h for degeneracy.\n"
                   "  Outputs R, t, points3d when stable and num_valid_points >= --min-points."));
  cmd.add(make_option(0, min_points_twoview, "min-points")
              .doc("Min valid 3D points for --twoview to store. Default: 50"));
  std::string log_level;
  cmd.add(make_option(0, log_level, "log-level").doc("Log level: error|warn|info|debug"));
  cmd.add(make_switch('v', "verbose").doc("Verbose logging (INFO level)"));
  cmd.add(make_switch('q', "quiet").doc("Quiet mode (ERROR level only)"));
  cmd.add(make_switch(0, "vis").doc(
      "Generate adjacency matrix heatmap images (PNG) per model.\n"
      "  Outputs: match_graph_F.png [match_graph_E.png] [match_graph_H.png]"));
  std::string vis_output;
  cmd.add(make_option(0, vis_output, "vis-output")
              .doc("Base path for adjacency matrix images.\n"
                   "  Default: <output_dir>/match_graph.png\n"
                   "  Actual files: <stem>_F.png, <stem>_E.png, <stem>_H.png"));
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

  // ── Validate required arguments ──────────────────────────────────────────
  if (pairs_json.empty() || match_dir.empty() || output_dir.empty()) {
    std::cerr << "Error: -i/--input, -m/--match-dir and -o/--output are required\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 1;
  }
  if (!intrinsics_json.empty() && image_list_json.empty()) {
    std::cerr << "Error: -l/--image-list is required when -k/--intrinsics is provided\n"
                 "  (needed to map each image_id to its camera_id).\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 1;
  }

  bool estimate_H = cmd.used("estimate-h");
  const bool run_twoview = cmd.used("twoview");
  FundamentalBackend f_backend = FundamentalBackend::kGpu;
  EssentialBackend e_backend = EssentialBackend::kGpu;
  HomographyBackend h_backend = HomographyBackend::kGpu;
  if (backend_str == "gpu") {
    f_backend = FundamentalBackend::kGpu;
    e_backend = EssentialBackend::kGpu;
    h_backend = HomographyBackend::kGpu;
  } else if (backend_str == "poselib" || backend_str == "cpu") {
    f_backend = FundamentalBackend::kPoseLib;
    e_backend = EssentialBackend::kPoseLib;
    h_backend = HomographyBackend::kPoseLib;
  } else {
    std::cerr << "Error: --backend must be gpu, poselib, or cpu\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 1;
  }
  if (run_twoview) {
    estimate_H = true; // need H for degeneracy detection
    if (image_list_json.empty()) {
      std::cerr
          << "Error: --twoview requires -l (image list; list may include cameras or use -k)\n\n";
      cmd.printHelp(std::cerr, argv[0]);
      return 1;
    }
  }

  // ── Logging level ────────────────────────────────────────────────────────
  insight::tools::apply_log_level(cmd.used('v'), cmd.used('q'), log_level);

  // ── Load intrinsics by image index (project/image-list JSON: cameras + images[].camera_index) ─
  std::vector<insight::camera::Intrinsics> image_index_intrinsics;
  if (!image_list_json.empty()) {
    insight::tools::ProjectData proj;
    if (insight::tools::load_project_data(image_list_json, &proj) && proj.num_images() > 0) {
      image_index_intrinsics.resize(static_cast<size_t>(proj.num_images()));
      for (int i = 0; i < proj.num_images(); ++i) {
        int c = proj.image_to_camera_index[static_cast<size_t>(i)];
        if (c >= 0 && c < proj.num_cameras())
          image_index_intrinsics[static_cast<size_t>(i)] = proj.cameras[static_cast<size_t>(c)];
      }
    }
  }
  const bool estimate_E = !image_index_intrinsics.empty();

  LOG(INFO) << "=== isat_geo configuration ===";
  LOG(INFO) << "  Pairs JSON   : " << pairs_json;
  LOG(INFO) << "  Match dir    : " << match_dir;
  LOG(INFO) << "  Output dir   : " << output_dir;
  LOG(INFO) << "  RANSAC iter  : " << ransac_iter;
  LOG(INFO) << "  Threshold F  : " << thresh_f << " px";
  LOG(INFO) << "  Estimate E   : " << (estimate_E ? "yes" : "no")
            << (estimate_E
                    ? " (" + std::to_string(image_index_intrinsics.size()) + " images by index)"
                    : "; use -l with export JSON (cameras + camera_index)");
  if (estimate_E)
    LOG(INFO) << "  Image list   : " << image_list_json;
  LOG(INFO) << "  Estimate H   : " << (estimate_H ? "yes (--estimate-h)" : "no");
  if (estimate_H)
    LOG(INFO) << "    Threshold H: " << thresh_h << " px";
  LOG(INFO) << "  F backend    : " << fundamental_backend_name(f_backend);
  LOG(INFO) << "  E backend    : " << essential_backend_name(e_backend);
  LOG(INFO) << "  H backend    : " << homography_backend_name(h_backend);
  LOG(INFO) << "  Min inliers  : " << min_inliers;
  LOG(INFO) << "  CPU threads  : " << num_threads;

  // ── Validate paths ───────────────────────────────────────────────────────
  if (!fs::is_directory(match_dir)) {
    LOG(ERROR) << "Match directory not found: " << match_dir;
    return 1;
  }
  fs::create_directories(output_dir);

  std::vector<GeoTask> tasks = load_pairs(pairs_json, match_dir);
  const int total = static_cast<int>(tasks.size());
  if (total == 0) {
    LOG(ERROR) << "No pairs to process";
    return 1;
  }

  const bool need_gpu_geo = (f_backend == FundamentalBackend::kGpu) ||
                            (estimate_E && e_backend == EssentialBackend::kGpu) ||
                            (estimate_H && h_backend == HomographyBackend::kGpu);

  // ── Initialise GPU RANSAC ────────────────────────────────────────────────
  if (need_gpu_geo) {
    GeoRansacConfig cfg;
    cfg.num_iterations = ransac_iter;
    cfg.local_size_x = 32;
    if (gpu_geo_init(&cfg) != 0) {
      LOG(FATAL) << "Failed to initialise GPU RANSAC (EGL + OpenGL 4.3 required)";
      return 1;
    }
    gpu_geo_set_solver(0); // Cholesky IPI: ~1ms/pair, ~45× faster than Jacobi
    // gpu_geo_set_solver(0); // Cholesky IPI: ~1ms/pair, ~45× faster than Jacobi
  }

  const float thresh_f_sq = thresh_f * thresh_f;
  const float thresh_h_sq = thresh_h * thresh_h;
  // thresh_e_sq is computed per-pair based on K1/K2

  // ── Stage 1: read match coords (multi-thread I/O) ────────────────────────
  const int IO_Q = 12;
  const int GPU_Q = 4;

  Stage loadStage("LoadMatches", num_threads, IO_Q, [&tasks](int i) {
    auto t0 = std::chrono::high_resolution_clock::now();
    read_match_coords(tasks[i]);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  std::chrono::high_resolution_clock::now() - t0)
                  .count();
    VLOG(2) << "  [load] pair " << i << " → " << tasks[i].num_matches << " matches  " << ms << "ms";
  });

  // ── Stage 3: write .isat_geo (multi-thread I/O) ──────────────────────────
  Stage writeStage("WriteGeo", num_threads, IO_Q, [&](int i) {
    const GeoTask& task = tasks[i];

    if (!task.F_ok && !task.E_ok && !task.H_ok) {
      VLOG(1) << "Pair [" << i << "] no valid geometry, skip write";
      return;
    }

    auto t0 = std::chrono::high_resolution_clock::now();
    write_geo(task, output_dir, ransac_iter, f_backend, e_backend, h_backend);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  std::chrono::high_resolution_clock::now() - t0)
                  .count();
    VLOG(2) << "  [write] pair " << i << "  " << ms << "ms";

    float prog = static_cast<float>(i + 1) / total;
    std::cerr << "PROGRESS: " << prog << "\n";
  });

  // ── Stage 2: GPU RANSAC (single thread = main thread, EGL context) ───────
  auto estimate_function = [&](int i) {
    GeoTask& task = tasks[i];

    if (task.num_matches < 8) {
      LOG(WARNING) << "Pair [" << i << "] " << task.image1_index << "–" << task.image2_index
                   << ": too few matches (" << task.num_matches << " < 8), skip";
      task.coords.clear();
      return;
    }

    // Build Match2D array for GPU RANSAC
    std::vector<Match2D> pts(task.num_matches);
    for (int k = 0; k < task.num_matches; k++) {
      pts[k].x1 = task.coords[k * 4 + 0];
      pts[k].y1 = task.coords[k * 4 + 1];
      pts[k].x2 = task.coords[k * 4 + 2];
      pts[k].y2 = task.coords[k * 4 + 3];
    }

    auto t0 = std::chrono::high_resolution_clock::now();

    // ── Estimate F ───────────────────────────────────────────────────
    if (f_backend == FundamentalBackend::kGpu) {
      task.F_inliers = gpu_ransac_F(pts.data(), task.num_matches, task.F, thresh_f_sq);
      task.F_ok = (task.F_inliers >= min_inliers);
      if (task.F_ok || task.F_inliers >= kMinInliersForTrack)
        task.F_mask = compute_f_mask(task.F, task.coords, task.num_matches, thresh_f_sq);
    } else {
      if (estimate_fundamental_poselib(task.coords, task.num_matches, thresh_f, ransac_iter, task.F,
                                       &task.F_mask, &task.F_inliers)) {
        task.F_ok = (task.F_inliers >= min_inliers);
        if (!task.F_ok && task.F_inliers < kMinInliersForTrack)
          task.F_mask.clear();
      }
    }

    // ── Estimate E (intrinsics by image index from list) ────────────────────
    if (estimate_E) {
      const int i1 = static_cast<int>(task.image1_index);
      const int i2 = static_cast<int>(task.image2_index);
      const insight::camera::Intrinsics* pK1 = nullptr;
      const insight::camera::Intrinsics* pK2 = nullptr;
      auto intrinsic_valid = [](const insight::camera::Intrinsics& K) {
        return K.fx > 1e-6 && K.fy > 1e-6;
      };
      if (i1 >= 0 && i1 < (int)image_index_intrinsics.size() && i2 >= 0 &&
          i2 < (int)image_index_intrinsics.size() &&
          intrinsic_valid(image_index_intrinsics[static_cast<size_t>(i1)]) &&
          intrinsic_valid(image_index_intrinsics[static_cast<size_t>(i2)])) {
        pK1 = &image_index_intrinsics[static_cast<size_t>(i1)];
        pK2 = &image_index_intrinsics[static_cast<size_t>(i2)];
      }

      if (!pK1 || !pK2) {
        LOG(WARNING) << "Pair [" << i << "] " << task.image1_index << "–" << task.image2_index
                     << ": cannot find K, skipping E";
      } else {
        const insight::camera::Intrinsics& K1 = *pK1;
        const insight::camera::Intrinsics& K2 = *pK2;

        // K-normalise using per-image camera (K1 for img1, K2 for img2)
        std::vector<Match2D> pts_norm(task.num_matches);
        for (int k = 0; k < task.num_matches; k++) {
          pts_norm[k].x1 = static_cast<float>((pts[k].x1 - K1.cx) / K1.fx);
          pts_norm[k].y1 = static_cast<float>((pts[k].y1 - K1.cy) / K1.fy);
          pts_norm[k].x2 = static_cast<float>((pts[k].x2 - K2.cx) / K2.fx);
          pts_norm[k].y2 = static_cast<float>((pts[k].y2 - K2.cy) / K2.fy);
        }

        // Threshold in normalised coords: use geometric mean of both cameras
        const double f_avg = std::sqrt(K1.fx * K1.fy * K2.fx * K2.fy);
        const float thresh_e_norm = static_cast<float>(thresh_f / std::sqrt(f_avg));
        const float thresh_e_sq = thresh_e_norm * thresh_e_norm;

        if (e_backend == EssentialBackend::kGpu) {
          task.E_inliers = gpu_ransac_E(pts_norm.data(), task.num_matches, task.E, thresh_e_sq);
          task.E_ok = (task.E_inliers >= min_inliers);
          if (task.E_ok || task.E_inliers >= kMinInliersForTrack)
            task.E_mask =
                compute_e_mask(task.E, task.coords, task.num_matches, K1, K2, thresh_e_sq);
        } else {
          if (estimate_essential_poselib(task.coords, task.num_matches, K1, K2, thresh_f,
                                         ransac_iter, task.E, &task.E_mask, &task.E_inliers)) {
            task.E_ok = (task.E_inliers >= min_inliers);
            if (!task.E_ok && task.E_inliers < kMinInliersForTrack)
              task.E_mask.clear();
          }
        }
      }
    }

    // ── Estimate H (if --estimate-h) ─────────────────────────────────
    if (estimate_H) {
      if (h_backend == HomographyBackend::kGpu) {
        task.H_inliers = gpu_ransac_H(pts.data(), task.num_matches, task.H, thresh_h_sq);
        task.H_ok = (task.H_inliers >= min_inliers);
        if (task.H_ok || task.H_inliers >= kMinInliersForTrack)
          task.H_mask = compute_h_mask(task.H, task.coords, task.num_matches, thresh_h_sq);
      } else {
        if (estimate_homography_poselib(task.coords, task.num_matches, thresh_h, ransac_iter,
                                        task.H, &task.H_mask, &task.H_inliers)) {
          task.H_ok = (task.H_inliers >= min_inliers);
          if (!task.H_ok && task.H_inliers < kMinInliersForTrack)
            task.H_mask.clear();
        }
      }
    }

    // ── Degeneracy detection (F vs H) ──────────────────────────────────
    task.degeneracy = insight::sfm::detect_degeneracy(
        task.F_inliers, task.H_ok ? task.H_inliers : 0, task.num_matches);

    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  std::chrono::high_resolution_clock::now() - t0)
                  .count();

    LOG(INFO) << "Geo [" << i << "/" << total << "] " << task.image1_index << "–"
              << task.image2_index << "  n=" << task.num_matches << "  F=" << task.F_inliers
              << (estimate_E ? ("  E=" + std::to_string(task.E_inliers)) : "")
              << (estimate_H ? ("  H=" + std::to_string(task.H_inliers)) : "")
              << (task.degeneracy.is_degenerate ? "  [DEG]" : "") << "  " << ms << "ms";

    // Keep coords for --twoview triangulation; else free
    if (!run_twoview) {
      task.coords.clear();
      task.coords.shrink_to_fit();
    }
  };

  StageCurrent* geoStage;
  Stage* cpuGeoStage;
  if (backend_str == "gpu") {
    geoStage = new StageCurrent("GeoEstimate", 1, GPU_Q, estimate_function);
    // ── Chain and run ────────────────────────────────────────────────────────
    chain(loadStage, *geoStage);
    if (!run_twoview) {
      chain(*geoStage, writeStage);
    }
    geoStage->setTaskCount(total);
  } else {
    cpuGeoStage = new Stage("GeoEstimateCPU", num_threads, GPU_Q * 2, estimate_function);
    // ── Chain and run ────────────────────────────────────────────────────────
    chain(loadStage, *cpuGeoStage);
    if (!run_twoview) {
      chain(*cpuGeoStage, writeStage);
    }
    cpuGeoStage->setTaskCount(total);
  }
  loadStage.setTaskCount(total);
  writeStage.setTaskCount(total);

  auto t_start = std::chrono::high_resolution_clock::now();

  // Push tasks from background thread; GPU runs on main thread (EGL context)
  std::thread push_thread([&]() {
    for (int i = 0; i < total; ++i)
      loadStage.push(i);
  });

  if(backend_str == "gpu") {
    geoStage->run(); // blocks until all GPU work done
  } 

  push_thread.join();
  loadStage.wait();

  // ── Two-view batch (when --twoview): E→R,t, GPU triangulate, stability ───
  if (run_twoview) {
    if (need_gpu_geo)
      gpu_geo_shutdown();

    if (gpu_twoview_init() != 0) {
      LOG(ERROR) << "gpu_twoview_init failed, skipping two-view reconstruction";
    } else {
      for (int i = 0; i < total; ++i) {
        GeoTask& task = tasks[i];
        if (!task.F_ok || task.degeneracy.is_degenerate || task.num_matches < 8)
          continue;

        int i1 = static_cast<int>(task.image1_index), i2 = static_cast<int>(task.image2_index);
        const insight::camera::Intrinsics* pK1 = nullptr;
        const insight::camera::Intrinsics* pK2 = nullptr;
        auto intrinsic_valid = [](const insight::camera::Intrinsics& K) {
          return K.fx > 1e-6 && K.fy > 1e-6;
        };
        if (i1 >= 0 && i1 < (int)image_index_intrinsics.size() && i2 >= 0 &&
            i2 < (int)image_index_intrinsics.size() &&
            intrinsic_valid(image_index_intrinsics[static_cast<size_t>(i1)]) &&
            intrinsic_valid(image_index_intrinsics[static_cast<size_t>(i2)])) {
          pK1 = &image_index_intrinsics[static_cast<size_t>(i1)];
          pK2 = &image_index_intrinsics[static_cast<size_t>(i2)];
        }
        if (!pK1 || !pK2)
          continue;

        const insight::camera::Intrinsics& K1 = *pK1;
        const insight::camera::Intrinsics& K2 = *pK2;

        // Two-view: always use F decomposition (E = K2^T F K1).
        // When intrinsics are inaccurate, direct RANSAC E bakes K into inlier selection
        // and is worse; F is estimated without K, then we apply K only for E decomposition.
        Eigen::Matrix3d K1m, K2m, F_mat;
        F_mat = insight::sfm::float_array_to_matrix3d(task.F);
        K1m << K1.fx, 0, K1.cx, 0, K1.fy, K1.cy, 0, 0, 1;
        K2m << K2.fx, 0, K2.cx, 0, K2.fy, K2.cy, 0, 0, 1;
        Eigen::Matrix3d E_mat = K2m.transpose() * F_mat * K1m;
        E_mat = insight::sfm::enforce_essential(E_mat);

        // Use F inliers for triangulation (consistent with F-derived E)
        const std::vector<uint8_t>& inl_mask = task.F_mask;
        std::vector<Eigen::Vector2d> pts1_n, pts2_n;
        for (int k = 0; k < task.num_matches; k++) {
          if (inl_mask[k] == 0)
            continue;
          pts1_n.emplace_back((task.coords[k * 4 + 0] - K1.cx) / K1.fx,
                              (task.coords[k * 4 + 1] - K1.cy) / K1.fy);
          pts2_n.emplace_back((task.coords[k * 4 + 2] - K2.cx) / K2.fx,
                              (task.coords[k * 4 + 3] - K2.cy) / K2.fy);
        }
        if ((int)pts1_n.size() < 8)
          continue;

        Eigen::Matrix3d R_mat;
        Eigen::Vector3d t_vec;
        int cheir = insight::sfm::decompose_essential(E_mat, pts1_n, pts2_n, R_mat, t_vec);
        if (cheir < 8)
          continue;

        // GPU triangulate
        const int n_pts = static_cast<int>(pts1_n.size());
        std::vector<float> pts_n_flat(n_pts * 4);
        for (int k = 0; k < n_pts; k++) {
          pts_n_flat[k * 4 + 0] = static_cast<float>(pts1_n[k].x());
          pts_n_flat[k * 4 + 1] = static_cast<float>(pts1_n[k].y());
          pts_n_flat[k * 4 + 2] = static_cast<float>(pts2_n[k].x());
          pts_n_flat[k * 4 + 3] = static_cast<float>(pts2_n[k].y());
        }
        std::vector<float> X_out(n_pts * 3);
        float Rf[9], tf[3];
        for (int r = 0; r < 3; r++)
          for (int c = 0; c < 3; c++)
            Rf[r * 3 + c] = static_cast<float>(R_mat(r, c));
        tf[0] = static_cast<float>(t_vec.x());
        tf[1] = static_cast<float>(t_vec.y());
        tf[2] = static_cast<float>(t_vec.z());

        gpu_triangulate(pts_n_flat.data(), n_pts, Rf, tf, X_out.data());

        std::vector<Eigen::Vector3d> points3d_eigen;
        std::vector<Eigen::Vector2d> pts1_valid, pts2_valid;
        for (int k = 0; k < n_pts; k++) {
          float x = X_out[k * 3 + 0], y = X_out[k * 3 + 1], z = X_out[k * 3 + 2];
          if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || z <= 1e-9f)
            continue;
          points3d_eigen.emplace_back(x, y, z);
          pts1_valid.push_back(pts1_n[k]);
          pts2_valid.push_back(pts2_n[k]);
        }
        const int n_valid = static_cast<int>(points3d_eigen.size());
        if (n_valid < 10)
          continue;

        task.stability = insight::sfm::compute_stability_metrics(points3d_eigen, pts1_valid,
                                                                 pts2_valid, R_mat, t_vec);

        if (task.stability.is_stable && n_valid >= min_points_twoview) {
          task.twoview_ok = true;
          task.num_valid_points = n_valid;
          insight::sfm::matrix3d_to_float_array(R_mat, task.R);
          task.t[0] = static_cast<float>(t_vec.x());
          task.t[1] = static_cast<float>(t_vec.y());
          task.t[2] = static_cast<float>(t_vec.z());
          task.points3d.resize(n_valid * 3);
          for (int k = 0; k < n_valid; k++) {
            task.points3d[k * 3 + 0] = static_cast<float>(points3d_eigen[k].x());
            task.points3d[k * 3 + 1] = static_cast<float>(points3d_eigen[k].y());
            task.points3d[k * 3 + 2] = static_cast<float>(points3d_eigen[k].z());
          }
          LOG(INFO) << "TwoView [" << i << "] " << task.image1_index << "–" << task.image2_index
                    << "  pts=" << n_valid << "  parallax=" << task.stability.median_parallax_deg
                    << "°"
                    << "  d/b=" << task.stability.median_depth_baseline;
        }
      }
      gpu_twoview_shutdown();
    }

    // Push all to write stage
    for (int i = 0; i < total; ++i)
      writeStage.push(i);
  }

  writeStage.wait();

  auto t_end = std::chrono::high_resolution_clock::now();
  auto total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();

  // ── Summary ──────────────────────────────────────────────────────────────
  int pairs_F = 0, pairs_E = 0, pairs_H = 0;
  int sum_F = 0, sum_E = 0, sum_H = 0;
  for (const auto& t : tasks) {
    if (t.F_ok) {
      pairs_F++;
      sum_F += t.F_inliers;
    }
    if (t.E_ok) {
      pairs_E++;
      sum_E += t.E_inliers;
    }
    if (t.H_ok) {
      pairs_H++;
      sum_H += t.H_inliers;
    }
  }

  int pairs_tv = 0;
  if (run_twoview) {
    for (const auto& t : tasks)
      if (t.twoview_ok)
        pairs_tv++;
  }

  // ── Output adjacency summary JSON and geometry-filtered pairs.json ───────
  json summary_pairs = json::array();
  json filtered_pairs = json::array();
  for (const auto& t : tasks) {
    summary_pairs.push_back({{"image1_index", t.image1_index},
                             {"image2_index", t.image2_index},
                             {"F_inliers", t.F_inliers},
                             {"E_inliers", t.E_inliers},
                             {"H_inliers", t.H_inliers},
                             {"degenerate", t.degeneracy.is_degenerate}});
    if (t.F_ok) {
      filtered_pairs.push_back(
          {{"image1_index", t.image1_index}, {"image2_index", t.image2_index}});
    }
  }
  json summary_json = {{"pairs", summary_pairs}};
  std::string adjacency_path = output_dir + "/adjacency.json";
  std::ofstream adj_out(adjacency_path);
  if (adj_out)
    adj_out << summary_json.dump(2) << "\n";
  else
    LOG(WARNING) << "Failed to write " << adjacency_path;

  json pairs_out_json = {{"pairs", filtered_pairs}};
  std::string pairs_path = output_dir + "/pairs.json";
  std::ofstream pairs_out(pairs_path);
  if (pairs_out)
    pairs_out << pairs_out_json.dump(2) << "\n";
  else
    LOG(WARNING) << "Failed to write " << pairs_path;

  json data = {{"total_pairs", total},
               {"pairs_with_F", pairs_F},
               {"total_time_ms", total_ms},
               {"total_time_s", total_ms / 1000.0},
               {"output_dir", output_dir}};
  if (estimate_E)
    data["pairs_with_E"] = pairs_E;
  if (estimate_H)
    data["pairs_with_H"] = pairs_H;
  if (run_twoview)
    data["pairs_twoview_ok"] = pairs_tv;
  data["adjacency_json"] = adjacency_path;
  data["pairs_json"] = pairs_path;
  print_event({{"type", "geo.complete"}, {"ok", true}, {"data", data}});

  // ── Optional: match-graph matrix visualization ───────────────────────────
  if (cmd.used("vis")) {
    const std::string vis_base =
        vis_output.empty() ? (output_dir + "/match_graph.png") : vis_output;

    // F – always estimated
    write_match_matrix(
        tasks, make_vis_path(vis_base, "F"), "F",
        [](const GeoTask& t) -> std::pair<bool, int> { return {t.F_ok, t.F_inliers}; });

    // E – only when intrinsics were provided
    if (estimate_E) {
      write_match_matrix(
          tasks, make_vis_path(vis_base, "E"), "E",
          [](const GeoTask& t) -> std::pair<bool, int> { return {t.E_ok, t.E_inliers}; });
    }

    // H – only when --estimate-h was set
    if (estimate_H) {
      write_match_matrix(
          tasks, make_vis_path(vis_base, "H"), "H",
          [](const GeoTask& t) -> std::pair<bool, int> { return {t.H_ok, t.H_inliers}; });
    }
  }

  if (!run_twoview && need_gpu_geo)
    gpu_geo_shutdown();
  return 0;
}
