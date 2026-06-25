/**
 * @file  bundler_loader.cpp
 * @brief Bundler v0.3 解析与 RenderTracks 填充。
 */

#include "bundler_loader.h"

#include "colmap_loader.h"

#include "ImageIO/gdal_utils.h"

#include <glog/logging.h>

#include <QString>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <sstream>
#include <unordered_map>

namespace insight {
namespace render {

bool load_reconstruction_directory(const std::string& dir, BundlerScene* scene,
                                   std::string* error_message,
                                   ReconstructionLoadProgress progress) {
  namespace fs = std::filesystem;
  std::error_code ec;
  const fs::path root = fs::absolute(dir, ec);
  if (ec || !fs::is_directory(root)) {
    *error_message = "not a directory: " + dir;
    return false;
  }
  const bool has_colmap_text =
      fs::exists(root / "cameras.txt") && fs::exists(root / "images.txt") &&
      fs::exists(root / "points3D.txt");
  const bool has_colmap_bin =
      fs::exists(root / "cameras.bin") && fs::exists(root / "images.bin") &&
      fs::exists(root / "points3D.bin");
  if (has_colmap_text)
    return load_colmap_text_directory(dir, scene, error_message, std::move(progress));
  if (has_colmap_bin)
    return load_colmap_binary_directory(dir, scene, error_message, std::move(progress));
  return load_bundler_directory(dir, scene, error_message, std::move(progress));
}

namespace {

// ── Image dimension cache ────────────────────────────────────────────────────
// Image files don't change between SfM iterations; cache w/h so GDAL is only
// invoked once per unique path for the lifetime of the process.
std::mutex g_dim_cache_mutex;
std::unordered_map<std::string, std::pair<int, int>> g_dim_cache;

bool get_image_dimensions(const std::string& path, int& w, int& h) {
  {
    std::lock_guard<std::mutex> lk(g_dim_cache_mutex);
    auto it = g_dim_cache.find(path);
    if (it != g_dim_cache.end()) {
      w = it->second.first;
      h = it->second.second;
      return true;
    }
  }
  if (!GdalUtils::GetWidthHeightPixel(path.c_str(), w, h))
    return false;
  {
    std::lock_guard<std::mutex> lk(g_dim_cache_mutex);
    g_dim_cache[path] = {w, h};
  }
  return true;
}

/// 假定主点在图像中心：w、h 取为 2*round(cx)、2*round(cy)。无主点信息时用 round(focal) 同时作 cx、cy 的整数近似。
void estimate_image_wh_from_principal_integers(const BundlerCamera& c, int* w, int* h) {
  int cx_i;
  int cy_i;
  if (c.principal_cx > 0.0 && c.principal_cy > 0.0) {
    cx_i = std::max(1, static_cast<int>(std::lround(c.principal_cx)));
    cy_i = std::max(1, static_cast<int>(std::lround(c.principal_cy)));
  } else {
    const int d = std::max(1, static_cast<int>(std::lround(std::max(c.focal, 1.0))));
    cx_i = d;
    cy_i = d;
  }
  *w = std::clamp(2 * cx_i, 2, 32768);
  *h = std::clamp(2 * cy_i, 2, 32768);
}

std::string normalize_path_sep(const std::string& p) {
  std::string s = p;
  for (char& c : s) {
    if (c == '\\')
      c = '/';
  }
  return s;
}

bool read_list_file(const std::filesystem::path& list_path, std::vector<std::string>* rel_paths,
                    std::string* err) {
  std::ifstream in(list_path);
  if (!in) {
    *err = "cannot open list file: " + list_path.string();
    return false;
  }
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty())
      continue;
    std::istringstream iss(line);
    std::string token;
    iss >> token;
    if (token.empty())
      continue;
    rel_paths->push_back(normalize_path_sep(token));
  }
  if (rel_paths->empty()) {
    *err = "list file is empty: " + list_path.string();
    return false;
  }
  return true;
}

bool read_bundle_cameras_points(std::istream& in, int num_cameras, int num_points,
                                std::vector<BundlerCamera>* cameras,
                                std::vector<BundlerPoint>* points, std::string* err,
                                const ReconstructionLoadProgress* progress) {
  const int64_t total = static_cast<int64_t>(num_cameras) + static_cast<int64_t>(num_points);
  auto report = [&](int64_t cur) {
    if (progress && *progress)
      (*progress)(cur, total, "bundle");
  };
  report(0);

  cameras->resize(num_cameras);
  for (int i = 0; i < num_cameras; ++i) {
    BundlerCamera& c = (*cameras)[i];
    in >> c.focal >> c.k1 >> c.k2;
    for (int row = 0; row < 3; ++row)
      for (int col = 0; col < 3; ++col)
        in >> c.R(row, col);
    in >> c.t(0) >> c.t(1) >> c.t(2);
    if (!in) {
      *err = "unexpected EOF while reading camera " + std::to_string(i);
      return false;
    }
    if (progress && *progress &&
        (((i + 1) % 16 == 0) || (i + 1 == num_cameras)))
      report(static_cast<int64_t>(i + 1));
  }

  constexpr int kPointStride = 4096;
  points->resize(num_points);
  for (int i = 0; i < num_points; ++i) {
    BundlerPoint& p = (*points)[i];
    in >> p.xyz(0) >> p.xyz(1) >> p.xyz(2);
    in >> p.rgb(0) >> p.rgb(1) >> p.rgb(2);
    int nvis = 0;
    in >> nvis;
    if (!in) {
      *err = "unexpected EOF while reading point " + std::to_string(i);
      return false;
    }
    p.observations.reserve(nvis);
    for (int k = 0; k < nvis; ++k) {
      BundlerObservation obs;
      in >> obs.cam_idx >> obs.key_idx >> obs.u >> obs.v;
      if (!in) {
        *err = "unexpected EOF while reading point " + std::to_string(i) + " view " +
               std::to_string(k);
        return false;
      }
      p.observations.push_back(obs);
    }
    if (progress && *progress &&
        ((((i + 1) % kPointStride) == 0) || (i + 1 == num_points)))
      report(static_cast<int64_t>(num_cameras) + static_cast<int64_t>(i + 1));
  }
  report(total);
  return true;
}

bool read_bundle_file(const std::filesystem::path& bundle_path, std::vector<BundlerCamera>* cameras,
                      std::vector<BundlerPoint>* points, std::string* err,
                      const ReconstructionLoadProgress* progress) {
  std::ifstream in(bundle_path);
  if (!in) {
    *err = "cannot open bundle file: " + bundle_path.string();
    return false;
  }
  std::string line;
  int num_cameras = 0;
  int num_points = 0;
  bool header_ok = false;
  while (std::getline(in, line)) {
    if (line.empty())
      continue;
    if (line[0] == '#')
      continue;
    std::istringstream iss(line);
    if (iss >> num_cameras >> num_points) {
      header_ok = true;
      break;
    }
  }
  if (!header_ok) {
    *err = "bundle file missing camera/point counts: " + bundle_path.string();
    return false;
  }

  return read_bundle_cameras_points(in, num_cameras, num_points, cameras, points, err, progress);
}

} // namespace

float compute_initial_photo_scale_from_scene(double avg_observation_depth,
                                           float mean_max_pixel_extent) {
  if (!std::isfinite(avg_observation_depth) || avg_observation_depth <= 0.0)
    return 1.f;
  if (!std::isfinite(mean_max_pixel_extent) || mean_max_pixel_extent <= 1e-6f)
    return 1.f;
  // render_tracks::render_photo uses local scale 1/50 on pixel-sized frustum; target world
  // extent ~ avg_depth/10  =>  photoScale * L / 50 ≈ avg_depth/10
  constexpr double kLocalFrustumScale = 50.0;
  const float ps = static_cast<float>((avg_observation_depth / 10.0) *
                                      (kLocalFrustumScale / static_cast<double>(mean_max_pixel_extent)));
  return std::clamp(ps, 1e-3f, 500.f);
}

bool load_bundler_directory(const std::string& bundle_dir, BundlerScene* scene,
                            std::string* error_message, ReconstructionLoadProgress progress) {
  GdalUtils::InitGDAL();

  std::error_code ec;
  std::filesystem::path root = std::filesystem::absolute(bundle_dir, ec);
  if (ec || !std::filesystem::is_directory(root)) {
    *error_message = "not a directory: " + bundle_dir;
    return false;
  }

  const std::filesystem::path list_path = root / "list.txt";
  std::vector<std::string> rel;
  if (!read_list_file(list_path, &rel, error_message))
    return false;

  std::filesystem::path bundle_path = root / "bundle.out";
  if (!std::filesystem::exists(bundle_path))
    bundle_path = root / "bundle.r.out";
  if (!std::filesystem::exists(bundle_path)) {
    *error_message = "neither bundle.out nor bundle.r.out found in " + root.string();
    return false;
  }

  std::vector<BundlerCamera> cameras;
  std::vector<BundlerPoint> points;
  const ReconstructionLoadProgress* prog_ptr = progress ? &progress : nullptr;
  if (!read_bundle_file(bundle_path, &cameras, &points, error_message, prog_ptr))
    return false;

  if (static_cast<int>(rel.size()) != static_cast<int>(cameras.size())) {
    *error_message = "list.txt count (" + std::to_string(rel.size()) + ") != bundle cameras (" +
                     std::to_string(cameras.size()) + ")";
    LOG(ERROR) << *error_message;
    return false;
  }

  scene->cameras = std::move(cameras);
  scene->points = std::move(points);
  scene->image_paths.clear();
  scene->image_paths.reserve(rel.size());

  for (const std::string& r : rel) {
    std::filesystem::path p = std::filesystem::path(r);
    if (p.is_relative())
      p = root / p;
    scene->image_paths.push_back(p.string());
  }

  return true;
}

void fill_render_tracks_from_bundler(RenderTracks* tracks, const BundlerScene& scene,
                                     BundlerFillProgress progress) {
  if (!tracks)
    return;

  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  int n = 0;

  for (const auto& c : scene.cameras) {
    Eigen::Matrix3d Rt = c.R.transpose();
    Eigen::Vector3d center = -Rt * c.t;
    mean += center;
    ++n;
  }
  for (const auto& p : scene.points) {
    mean += p.xyz;
    ++n;
  }
  if (n > 0)
    mean /= static_cast<double>(n);

  double sum_dist = 0.0;
  int n_dist = 0;
  for (const auto& c : scene.cameras) {
    Eigen::Matrix3d Rwc = c.R.transpose();
    Eigen::Vector3d center = -Rwc * c.t;
    sum_dist += (center - mean).norm();
    ++n_dist;
  }
  for (const auto& p : scene.points) {
    sum_dist += (p.xyz - mean).norm();
    ++n_dist;
  }
  const double avg_observation_depth =
      n_dist > 0 ? sum_dist / static_cast<double>(n_dist) : 1.0;

  const int n_cam = static_cast<int>(scene.cameras.size());
  RenderTracks::Photos photos;
  photos.reserve(scene.cameras.size());

  float sum_max_pixel = 0.f;
  int n_pixel = 0;

  for (size_t i = 0; i < scene.cameras.size(); ++i) {
    if (progress) {
      progress(static_cast<int>(i) + 1, n_cam, "dimensions");
    }

    const BundlerCamera& c = scene.cameras[i];
    RenderTracks::Photo photo;
    photo.id = static_cast<int>(i);

    int w = (c.image_width > 0) ? c.image_width : 0;
    int h = (c.image_height > 0) ? c.image_height : 0;
    const std::string& ip = scene.image_paths[i];

    if (w > 0 && h > 0) {
      // COLMAP（或已写入尺寸的模型）：完全使用 cameras.txt / 内参里的宽高，不调 GDAL。
    } else {
      if (!get_image_dimensions(ip, w, h)) {
        estimate_image_wh_from_principal_integers(c, &w, &h);
        LOG(WARNING) << "GetWidthHeightPixel failed for " << ip << ", using 2*cx x 2*cy estimate "
                     << w << "x" << h;
      }
    }
    photo.w = static_cast<float>(w);
    photo.h = static_cast<float>(h);
    photo.focal = static_cast<float>(c.focal);
    if (photo.focal <= 0.f)
      photo.focal = 0.5f * (photo.w + photo.h);

    const float L =
        std::max(photo.focal, std::max(photo.w, photo.h));
    sum_max_pixel += L;
    ++n_pixel;

    photo.name = QString::fromStdString(ip);

    Eigen::Matrix3d Rwc = c.R.transpose();
    Eigen::Vector3d center = -Rwc * c.t;

    photo.initPose.centerValid = true;
    photo.initPose.rotationValid = true;
    photo.initPose.data[0] = center.x() - mean.x();
    photo.initPose.data[1] = center.y() - mean.y();
    photo.initPose.data[2] = center.z() - mean.z();
    photo.initPose.color.setOnes();

    Mat4 M = Mat4::Identity();
    M.block<3, 3>(0, 0) = Rwc;
    photo.initPose.openglMat = M;

    photo.refinedPose = photo.initPose;
    photos.push_back(photo);
  }

  const float mean_max_pixel =
      n_pixel > 0 ? (sum_max_pixel / static_cast<float>(n_pixel)) : 1.f;

  RenderTracks::RenderOptions opt;
  opt.photoScale = compute_initial_photo_scale_from_scene(avg_observation_depth, mean_max_pixel);
  // glPointSize: scale with scene extent so sparse points / camera centers stay visible
  opt.vetexSize = static_cast<float>(
      std::clamp(avg_observation_depth * 0.08, 1.0, 12.0));
  opt.poseSize =
      static_cast<float>(std::clamp(avg_observation_depth * 0.1, 2.0, 16.0));

  RenderTracks::Tracks tracks_data;
  tracks_data.reserve(scene.points.size());
  for (size_t i = 0; i < scene.points.size(); ++i) {
    const BundlerPoint& p = scene.points[i];
    RenderTracks::Track t;
    t.trackId = static_cast<int>(i);
    t.x = p.xyz.x() - mean.x();
    t.y = p.xyz.y() - mean.y();
    t.z = p.xyz.z() - mean.z();
    t.color.x() = p.rgb.x() / 255.0;
    t.color.y() = p.rgb.y() / 255.0;
    t.color.z() = p.rgb.z() / 255.0;
    for (const auto& o : p.observations) {
      RenderTracks::Observe obs;
      obs.photoId = o.cam_idx;
      obs.featX   = o.u;
      obs.featY   = o.v;
      t.obs.push_back(obs);
    }
    tracks_data.push_back(t);
  }

  tracks->clear();
  tracks->set_render_options(opt);
  tracks->set_photos(photos);
  tracks->set_tracks(tracks_data);
  tracks->set_center(mean.x(), mean.y(), mean.z());
}

} // namespace render
} // namespace insight
