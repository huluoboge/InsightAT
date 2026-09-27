/**
 * isat_focal_from_geo.cpp
 *
 * Estimate shared (or per-camera) focal length from geo F matrices when EXIF
 * is missing / unreliable.  Writes updated fx/fy into --project JSON
 * (images_all.json style) and optionally prints ISAT_EVENT.
 *
 * Typical use (after isat_geo, before incremental SfM):
 *   isat_focal_from_geo -p images_all.json -g geo/ -o images_all.json -v
 *
 * I/O strategy (matches isat_tracks geopack path):
 *   - Prefer GeoPackIndex → group by pack file
 *   - One sequential fread of each pack payload, then zero-copy blob lookups
 *   - Cap to top --max-pairs per camera by F/H/parallax/disp quality
 *   - Hartley search range from image size; reject boundary minima
 *   - OpenMP across packs (and across legacy .isat_geo files)
 */
#include "cli_logging.h"
#include "cmdLine/cmdLine.h"

#include "algorithm/io/geopack_index.h"
#include "algorithm/io/idc_reader.h"
#include "algorithm/modules/sfm/focal_from_view_graph.h"
#include "project_loader.h"

#include <Eigen/Core>
#include <glog/logging.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

using json = nlohmann::json;
namespace fs = std::filesystem;
using insight::tools::ProjectData;
using insight::sfm::FocalFromViewGraphOptions;
using insight::sfm::FocalPairConstraint;
using insight::sfm::estimate_focals_from_view_graph;

static bool F_from_payload(const insight::io::IDCReader& rd, const std::vector<uint8_t>& payload,
                           const std::string& blob_name, Eigen::Matrix3d* F) {
  size_t nbytes = 0;
  const uint8_t* ptr = rd.get_blob_from_payload(blob_name, payload, &nbytes);
  if (!ptr || nbytes != 9 * sizeof(float))
    return false;
  const float* raw = reinterpret_cast<const float*>(ptr);
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      (*F)(r, c) = static_cast<double>(raw[r * 3 + c]);
  return true;
}

static FocalPairConstraint make_constraint(const ProjectData& project, uint32_t i, uint32_t j,
                                           const Eigen::Matrix3d& F, int weight,
                                           bool degenerate) {
  FocalPairConstraint c;
  c.F = F;
  const int ci = project.image_to_camera_index[i];
  const int cj = project.image_to_camera_index[j];
  const auto& Ki = project.cameras[static_cast<size_t>(ci)];
  const auto& Kj = project.cameras[static_cast<size_t>(cj)];
  c.cx1 = Ki.cx;
  c.cy1 = Ki.cy;
  c.cx2 = Kj.cx;
  c.cy2 = Kj.cy;
  c.camera_id1 = ci;
  c.camera_id2 = cj;
  c.weight = std::max(1, weight);
  c.is_degenerate = degenerate;
  return c;
}

/// Quality score for selecting pairs useful for Hartley focal-from-F.
/// Uses geo signals already stored in GeoPackIndex (F/H/E, degeneracy, parallax, disp).
/// Returns <=0 if the pair should be hard-rejected.
static double pair_focal_quality(const insight::io::GeoPackPairEntry& e, int min_inliers,
                                 double min_pixel_disp, double min_parallax_deg,
                                 double max_parallax_deg, bool require_stable) {
  if (!e.F_ok || e.is_degenerate || e.F_inliers < min_inliers)
    return -1.0;

  // Prefer geometrically stable two-view pairs when the geo stage marked them.
  if (require_stable && !e.stable)
    return -1.0;

  // H explains the matches as well as (or better than) F → planar / pure-rotation.
  if (e.H_ok && e.H_inliers > 0 && e.H_inliers >= e.F_inliers)
    return -1.0;
  // Stricter: H catching up to F is already a bad focal constraint.
  if (e.H_ok && e.H_inliers > 0 && e.F_inliers > 0) {
    const double h_over_f = static_cast<double>(e.H_inliers) / static_cast<double>(e.F_inliers);
    if (h_over_f >= 0.85)
      return -1.0;
  }

  if (e.median_pixel_disp > 0.0 && e.median_pixel_disp < min_pixel_disp)
    return -1.0;

  // Prefer moderate triangulation parallax (observability without wild outliers).
  if (e.median_parallax_deg > 0.0) {
    if (e.median_parallax_deg < min_parallax_deg || e.median_parallax_deg > max_parallax_deg)
      return -1.0;
  } else if (require_stable) {
    // Stable flag without parallax is rare; keep but don't prefer.
  }

  double score = static_cast<double>(e.F_inliers);
  if (e.F_inlier_ratio > 0.0)
    score *= (0.25 + 0.75 * std::min(1.0, static_cast<double>(e.F_inlier_ratio)));

  if (e.E_ok)
    score *= 1.25;
  if (e.twoview_ok)
    score *= 1.25;
  if (e.stable)
    score *= 2.0; // strong preference — these survived cheirality/parallax checks

  if (e.median_parallax_deg > 0.0) {
    // Peak preference around ~5–15° (good baseline without grazing rays).
    const double p = e.median_parallax_deg;
    const double pref =
        std::exp(-0.5 * std::pow((std::log(std::max(1e-3, p)) - std::log(8.0)) / 0.9, 2));
    score *= (0.4 + 0.6 * pref);
  } else if (e.median_pixel_disp > 0.0) {
    score *= (0.5 + 0.5 * std::min(1.0, e.median_pixel_disp / 60.0));
  }

  if (e.num_valid_points > 0)
    score *= (0.7 + 0.3 * std::min(1.0, e.num_valid_points / 80.0));

  return score;
}

/// Fast path: GeoPackIndex → quality filter/rank → per-pack full payload.
static bool collect_from_geopack_index(const std::string& geo_dir, const ProjectData& project,
                                       int min_inliers, int max_pairs, int num_threads,
                                       double min_pixel_disp, double min_parallax_deg,
                                       double max_parallax_deg,
                                       std::vector<FocalPairConstraint>* out) {
  insight::io::GeoPackIndex index;
  if (!index.load_from_dir(geo_dir) || index.entries().empty())
    return false;

  struct Cand {
    const insight::io::GeoPackPairEntry* e = nullptr;
    double score = 0.0;
  };

  auto collect_cands = [&](bool require_stable) {
    std::vector<Cand> out;
    out.reserve(index.entries().size());
    int n_f_ok = 0, n_hard_reject = 0;
    for (const auto& kv : index.entries()) {
      const auto& e = kv.second;
      if (!e.F_ok)
        continue;
      ++n_f_ok;
      if (e.image1_index >= project.num_images() || e.image2_index >= project.num_images()) {
        ++n_hard_reject;
        continue;
      }
      if (e.pack_path.empty()) {
        ++n_hard_reject;
        continue;
      }
      const double score = pair_focal_quality(e, min_inliers, min_pixel_disp, min_parallax_deg,
                                             max_parallax_deg, require_stable);
      if (!(score > 0.0)) {
        ++n_hard_reject;
        continue;
      }
      out.push_back(Cand{&e, score});
    }
    return std::make_tuple(std::move(out), n_f_ok, n_hard_reject);
  };

  // Prefer stable two-view pairs when geo provided enough of them.
  auto [cands, n_f_ok, n_hard_reject] = collect_cands(/*require_stable=*/true);
  const int min_stable_keep = std::max(50, min_inliers * 5);
  if (static_cast<int>(cands.size()) < min_stable_keep) {
    LOG(INFO) << "Focal-from-geo: only " << cands.size()
              << " stable pairs; falling back to non-stable quality filter";
    std::tie(cands, n_f_ok, n_hard_reject) = collect_cands(/*require_stable=*/false);
  } else {
    LOG(INFO) << "Focal-from-geo: using stable-only pairs (" << cands.size() << ")";
  }
  if (cands.empty()) {
    LOG(WARNING) << "GeoPack: no quality-passed pairs (F_ok=" << n_f_ok
                 << " rejected=" << n_hard_reject << " min_inliers=" << min_inliers
                 << " min_disp=" << min_pixel_disp << " parallax∈[" << min_parallax_deg << ","
                 << max_parallax_deg << "])";
    return false;
  }

  // Cap per camera (same-camera pairs) + separate budget for cross-camera pairs.
  // Avoid O(n²) blow-up on large multi-cam / 3000-image sets.
  const int before_cap = static_cast<int>(cands.size());
  if (max_pairs > 0) {
    std::map<int, std::vector<Cand>> by_cam;
    std::vector<Cand> cross;
    for (const auto& c : cands) {
      const int ci = project.image_to_camera_index[c.e->image1_index];
      const int cj = project.image_to_camera_index[c.e->image2_index];
      if (ci == cj)
        by_cam[ci].push_back(c);
      else
        cross.push_back(c);
    }
    cands.clear();
    for (auto& kv : by_cam) {
      auto& v = kv.second;
      std::sort(v.begin(), v.end(),
                [](const Cand& a, const Cand& b) { return a.score > b.score; });
      if (static_cast<int>(v.size()) > max_pairs)
        v.resize(static_cast<size_t>(max_pairs));
      cands.insert(cands.end(), v.begin(), v.end());
    }
    std::sort(cross.begin(), cross.end(),
              [](const Cand& a, const Cand& b) { return a.score > b.score; });
    if (static_cast<int>(cross.size()) > max_pairs)
      cross.resize(static_cast<size_t>(max_pairs));
    cands.insert(cands.end(), cross.begin(), cross.end());

    LOG(INFO) << "Focal-from-geo: capped " << before_cap << " → " << cands.size()
              << " (max " << max_pairs << " same-cam pairs / camera, max " << max_pairs
              << " cross-cam)";
  } else {
    std::sort(cands.begin(), cands.end(),
              [](const Cand& a, const Cand& b) { return a.score > b.score; });
  }

  if (cands.empty()) {
    LOG(WARNING) << "GeoPack: empty after per-camera cap";
    return false;
  }

  std::sort(cands.begin(), cands.end(),
            [](const Cand& a, const Cand& b) { return a.score > b.score; });

  LOG(INFO) << "Focal-from-geo selection: kept=" << cands.size() << " F_ok=" << n_f_ok
            << " rejected=" << n_hard_reject << " top_score=" << cands.front().score
            << " top_F_inl=" << cands.front().e->F_inliers
            << " top_H_inl=" << cands.front().e->H_inliers
            << " top_par=" << cands.front().e->median_parallax_deg
            << "deg top_disp=" << cands.front().e->median_pixel_disp << "px"
            << (cands.front().e->is_degenerate ? " [DEG]" : "")
            << (cands.front().e->stable ? " [stable]" : "");

  std::map<std::string, std::vector<Cand>> by_pack;
  for (const auto& c : cands)
    by_pack[c.e->pack_path].push_back(c);

  std::vector<std::string> pack_paths;
  pack_paths.reserve(by_pack.size());
  std::vector<std::vector<Cand>> pack_cands_list;
  pack_cands_list.reserve(by_pack.size());
  for (auto& kv : by_pack) {
    pack_paths.push_back(kv.first);
    pack_cands_list.push_back(std::move(kv.second));
  }

  LOG(INFO) << "Focal-from-geo geopack: " << cands.size() << " pairs across "
            << pack_paths.size() << " blocks (threads=" << num_threads << ")";

  std::vector<std::vector<FocalPairConstraint>> per_pack(pack_paths.size());

#ifdef _OPENMP
  omp_set_num_threads(std::max(1, num_threads));
#pragma omp parallel for schedule(dynamic)
#endif
  for (int pi = 0; pi < static_cast<int>(pack_paths.size()); ++pi) {
    const std::string& pack_path = pack_paths[static_cast<size_t>(pi)];
    const auto& pack_cands = pack_cands_list[static_cast<size_t>(pi)];
    insight::io::IDCReader rd(pack_path);
    if (!rd.is_valid()) {
      LOG(WARNING) << "Unreadable geopack: " << pack_path;
      continue;
    }
    std::vector<uint8_t> payload = rd.read_full_payload();
    if (payload.empty()) {
      LOG(WARNING) << "Empty geopack payload: " << pack_path;
      continue;
    }

    auto& local = per_pack[static_cast<size_t>(pi)];
    local.reserve(pack_cands.size());
    for (const auto& cand : pack_cands) {
      const auto& e = *cand.e;
      const std::string blob =
          "pair/" + std::to_string(e.image1_index) + "_" + std::to_string(e.image2_index) +
          "/F_matrix";
      Eigen::Matrix3d F;
      if (!F_from_payload(rd, payload, blob, &F))
        continue;
      // Weight = inliers × quality (quality already folds H/parallax/E/…).
      const int w = std::max(1, static_cast<int>(std::lround(cand.score)));
      local.push_back(make_constraint(project, e.image1_index, e.image2_index, F, w,
                                      e.is_degenerate));
    }
    VLOG(1) << "geopack " << fs::path(pack_path).filename().string() << ": loaded "
            << local.size() << "/" << pack_cands.size() << " F matrices";
  }

  out->clear();
  size_t total = 0;
  for (const auto& v : per_pack)
    total += v.size();
  out->reserve(total);
  for (auto& v : per_pack) {
    out->insert(out->end(), std::make_move_iterator(v.begin()),
                std::make_move_iterator(v.end()));
  }
  return !out->empty();
}

/// Legacy per-pair .isat_geo (OMP open + payload).
static bool collect_from_legacy_isat_geo(const std::string& geo_dir, const ProjectData& project,
                                         int min_inliers, int max_pairs, int num_threads,
                                         std::vector<FocalPairConstraint>* out) {
  std::vector<fs::path> files;
  for (const auto& entry : fs::directory_iterator(geo_dir)) {
    if (!entry.is_regular_file())
      continue;
    const std::string name = entry.path().filename().string();
    if (name.find(".isat_geo") == std::string::npos)
      continue;
    files.push_back(entry.path());
  }
  if (files.empty())
    return false;

  std::vector<FocalPairConstraint> local(files.size());
  std::vector<char> ok(files.size(), 0);

#ifdef _OPENMP
  omp_set_num_threads(std::max(1, num_threads));
#pragma omp parallel for schedule(dynamic, 32)
#endif
  for (int fi = 0; fi < static_cast<int>(files.size()); ++fi) {
    const auto& path = files[static_cast<size_t>(fi)];
    const std::string name = path.filename().string();
    const auto us = name.find('_');
    const auto dot = name.find('.');
    if (us == std::string::npos || dot == std::string::npos || us > dot)
      continue;
    uint32_t i = 0, j = 0;
    try {
      i = static_cast<uint32_t>(std::stoul(name.substr(0, us)));
      j = static_cast<uint32_t>(std::stoul(name.substr(us + 1, dot - us - 1)));
    } catch (...) {
      continue;
    }
    if (i >= project.num_images() || j >= project.num_images())
      continue;

    insight::io::IDCReader rd(path.string());
    if (!rd.is_valid())
      continue;
    std::vector<uint8_t> payload;
    rd.read_full_payload_into(payload);
    Eigen::Matrix3d F;
    if (!F_from_payload(rd, payload, "F_matrix", &F))
      continue;
    // Legacy files lack H/parallax summary in the filename path.
    local[static_cast<size_t>(fi)] =
        make_constraint(project, i, j, F, std::max(min_inliers, 1), false);
    ok[static_cast<size_t>(fi)] = 1;
  }

  out->clear();
  out->reserve(files.size());
  for (size_t k = 0; k < files.size(); ++k) {
    if (ok[k])
      out->push_back(std::move(local[k]));
  }
  if (max_pairs > 0 && static_cast<int>(out->size()) > max_pairs) {
    std::sort(out->begin(), out->end(),
              [](const FocalPairConstraint& a, const FocalPairConstraint& b) {
                return a.weight > b.weight;
              });
    out->resize(static_cast<size_t>(max_pairs));
  }
  return !out->empty();
}

static bool collect_pairs_from_geo(const std::string& geo_dir, const ProjectData& project,
                                   int min_inliers, int max_pairs, int num_threads,
                                   double min_pixel_disp, double min_parallax_deg,
                                   double max_parallax_deg,
                                   std::vector<FocalPairConstraint>* out) {
  if (collect_from_geopack_index(geo_dir, project, min_inliers, max_pairs, num_threads,
                                 min_pixel_disp, min_parallax_deg, max_parallax_deg, out))
    return true;
  LOG(INFO) << "No usable geopack F; trying legacy .isat_geo";
  return collect_from_legacy_isat_geo(geo_dir, project, min_inliers, max_pairs, num_threads, out);
}

int main(int argc, char** argv) {
  FLAGS_logtostderr = true;
  google::InitGoogleLogging(argv[0]);

  CmdLine cmd("Estimate focal from geo F matrices (no-EXIF / unreliable prior path).");
  std::string project_path, geo_dir, output_path;
  int min_inliers = 30;
  int min_pairs = 3;
  int max_pairs = 5000; // per camera (same-cam); also caps cross-cam budget
  int num_threads = -1;
  int no_ceres = 0;
  double min_pixel_disp = 15.0;
  double min_parallax_deg = 1.0;
  double max_parallax_deg = 60.0;
  std::string log_level = "info";

  cmd.add(make_option('p', project_path, "project").doc("images_all.json / project JSON"));
  cmd.add(make_option('g', geo_dir, "geo").doc("Geo directory (.isat_geo / geopack)"));
  cmd.add(make_option('o', output_path, "output")
              .doc("Output JSON (default: overwrite -p)"));
  cmd.add(make_option(0, min_inliers, "min-inliers").doc("Min F inliers per pair (default 30)"));
  cmd.add(make_option(0, min_pairs, "min-pairs").doc("Min pairs per camera (default 3)"));
  cmd.add(make_option(0, max_pairs, "max-pairs")
              .doc("Max quality-ranked pairs per camera for same-camera edges, and max "
                   "cross-camera edges overall (default 5000; 0 = no cap)"));
  cmd.add(make_option(0, min_pixel_disp, "min-pixel-disp")
              .doc("Min median match displacement in px (default 15)"));
  cmd.add(make_option(0, min_parallax_deg, "min-parallax-deg")
              .doc("Min median triangulation parallax in deg when available (default 1)"));
  cmd.add(make_option(0, max_parallax_deg, "max-parallax-deg")
              .doc("Max median triangulation parallax in deg when available (default 60)"));
  cmd.add(make_option('j', num_threads, "threads")
              .doc("OpenMP threads for pack I/O + Hartley (default: hardware)"));
  cmd.add(make_option(0, no_ceres, "no-ceres").doc("Skip Ceres refine (1=skip)"));
  cmd.add(make_option(0, log_level, "log-level").doc("Log level"));
  cmd.add(make_switch('v', "verbose").doc("Verbose (INFO)"));
  cmd.add(make_switch('q', "quiet").doc("Quiet (ERROR only)"));
  cmd.add(make_switch('h', "help").doc("Show help"));

  try {
    cmd.process(argc, argv);
  } catch (const std::string& s) {
    std::cerr << "Error: " << s << "\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 1;
  }
  if (cmd.used('h') || project_path.empty() || geo_dir.empty()) {
    cmd.printHelp(std::cerr, argv[0]);
    return cmd.used('h') ? 0 : 1;
  }
  if (output_path.empty())
    output_path = project_path;

  insight::tools::apply_log_level(cmd.used('v'), cmd.used('q'), log_level);

  if (num_threads <= 0)
    num_threads = static_cast<int>(std::max(1u, std::thread::hardware_concurrency()));

  ProjectData project;
  if (!insight::tools::load_project_data(project_path, &project)) {
    LOG(ERROR) << "Failed to load project " << project_path;
    return 1;
  }

  std::vector<FocalPairConstraint> pairs;
  if (!collect_pairs_from_geo(geo_dir, project, min_inliers, max_pairs, num_threads, min_pixel_disp,
                              min_parallax_deg, max_parallax_deg, &pairs)) {
    LOG(ERROR) << "No F constraints loaded from " << geo_dir;
    return 1;
  }
  LOG(INFO) << "Loaded " << pairs.size() << " F constraints from " << geo_dir;

  FocalFromViewGraphOptions opts;
  opts.min_inliers = 1; // weights already encode inlier quality; don't double-filter
  opts.min_pairs_per_camera = min_pairs;
  opts.refine_with_ceres = (no_ceres == 0);
  opts.use_cross_camera_pairs = true;
  opts.ceres_num_threads = num_threads;
  opts.hartley_num_threads = num_threads;

  // Per-camera search range + soft prior from each camera's image size.
  opts.camera_priors.resize(static_cast<size_t>(std::max(0, project.num_cameras())));
  for (int ci = 0; ci < project.num_cameras(); ++ci) {
    const auto& K = project.cameras[static_cast<size_t>(ci)];
    auto& pr = opts.camera_priors[static_cast<size_t>(ci)];
    pr.width = K.width;
    pr.height = K.height;
    insight::sfm::focal_bounds_from_image_size(K.width, K.height, &pr.f_min, &pr.f_max,
                                               &pr.soft_prior);
    LOG(INFO) << "Camera " << ci << " " << K.width << "x" << K.height << ": f∈[" << pr.f_min
              << ", " << pr.f_max << "] soft_prior=" << pr.soft_prior
              << " (ignore fx=" << K.fx << ")";
  }
  if (!opts.camera_priors.empty()) {
    opts.f_min = opts.camera_priors[0].f_min;
    opts.f_max = opts.camera_priors[0].f_max;
    opts.soft_prior_focal = opts.camera_priors[0].soft_prior;
  }

  const auto res = estimate_focals_from_view_graph(pairs, opts);
  if (!res.ok) {
    LOG(ERROR) << "Focal estimation failed (pairs_used=" << res.num_pairs_used
               << " skipped=" << res.num_pairs_skipped
               << " cross=" << res.num_cross_pairs_used << ")";
    return 1;
  }

  int n_updated = 0;
  for (const auto& cam_est : res.cameras) {
    if (!cam_est.ok)
      continue;
    if (cam_est.camera_id < 0 ||
        cam_est.camera_id >= static_cast<int>(project.cameras.size()))
      continue;
    auto& K = project.cameras[static_cast<size_t>(cam_est.camera_id)];
    LOG(INFO) << "Update camera " << cam_est.camera_id << " fx: " << K.fx << " → "
              << cam_est.focal << " (" << res.method << ", same_pairs=" << cam_est.num_pairs
              << ", cross_pairs=" << cam_est.num_cross_pairs << ")";
    K.fx = cam_est.focal;
    K.fy = cam_est.focal;
    ++n_updated;
  }
  if (n_updated == 0) {
    LOG(ERROR) << "No camera focal updated";
    return 1;
  }

  // Rewrite JSON: load original, patch cameras[].fx/fy.
  std::ifstream ifs(project_path);
  json root = json::parse(ifs);
  ifs.close();
  if (root.contains("cameras") && root["cameras"].is_array()) {
    for (const auto& cam_est : res.cameras) {
      if (!cam_est.ok)
        continue;
      if (cam_est.camera_id < 0 ||
          cam_est.camera_id >= static_cast<int>(root["cameras"].size()))
        continue;
      root["cameras"][cam_est.camera_id]["fx"] = cam_est.focal;
      root["cameras"][cam_est.camera_id]["fy"] = cam_est.focal;
    }
  }
  std::ofstream ofs(output_path);
  ofs << root.dump(2) << "\n";
  ofs.close();
  LOG(INFO) << "Wrote " << output_path << " (updated " << n_updated << " cameras)";

  json cams_ev = json::array();
  for (const auto& c : res.cameras) {
    cams_ev.push_back({{"camera_id", c.camera_id},
                       {"focal", c.focal},
                       {"median_focal", c.median_focal},
                       {"num_pairs", c.num_pairs},
                       {"num_cross_pairs", c.num_cross_pairs},
                       {"ok", c.ok}});
  }
  json ev = {{"type", "focal_from_geo.estimate"},
             {"ok", true},
             {"method", res.method},
             {"num_pairs_used", res.num_pairs_used},
             {"num_pairs_skipped", res.num_pairs_skipped},
             {"num_cross_pairs_used", res.num_cross_pairs_used},
             {"shared_focal", res.shared_focal},
             {"num_cameras_updated", n_updated},
             {"cameras", cams_ev}};
  std::cout << "ISAT_EVENT " << ev.dump() << "\n";
  return 0;
}
