/**
 * @file  colmap_loader.cpp
 * @brief COLMAP sparse text model → BundlerScene（world→camera: Xc = R*Xw + t）。
 *
 * COLMAP 相机轴为 CV 惯例（+X 右、+Y 下、+Z 前）；渲染里 draw_camera_image_frame 按 OpenGL
 * 视锥（+Y 上、沿 -Z 看）绘制。导入时在保持光心 C = -R^T t 不变的前提下左乘对角(1,-1,-1)，
 * 使 COLMAP 与 Bundler 在可视化里朝向一致。
 */

#include "colmap_loader.h"

#include "bundler_loader.h"

#include "ImageIO/gdal_utils.h"

#include <glog/logging.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <vector>

namespace insight {
namespace render {

namespace {

namespace fs = std::filesystem;

// COLMAP / OpenCV 相机系 → 与 render_tracks 中视锥一致（与 bundle.out 中常见 Bundler 视姿对齐）
static const Eigen::Matrix3d kColmapCvCameraToBundlerLikeCamera =
    (Eigen::Matrix3d() << 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0).finished();

static std::string trim_copy(std::string_view sv) {
  while (!sv.empty() && std::isspace(static_cast<unsigned char>(sv.front())))
    sv.remove_prefix(1);
  while (!sv.empty() && std::isspace(static_cast<unsigned char>(sv.back())))
    sv.remove_suffix(1);
  return std::string(sv);
}

static bool is_comment_or_empty(std::string_view line) {
  const std::string t = trim_copy(line);
  return t.empty() || t[0] == '#';
}

struct ColmapCameraRow {
  int id = 0;
  std::string model;
  int width = 0;
  int height = 0;
  std::vector<double> params;
};

static bool parse_camera_intrinsics(const ColmapCameraRow& row, double* focal, double* k1,
                                  double* k2) {
  const std::string& m = row.model;
  const auto& p = row.params;
  *k1 = *k2 = 0.0;
  if (m == "SIMPLE_PINHOLE" && p.size() >= 3) {
    *focal = p[0];
    return true;
  }
  if (m == "PINHOLE" && p.size() >= 4) {
    *focal = 0.5 * (p[0] + p[1]);
    return true;
  }
  if (m == "SIMPLE_RADIAL" && p.size() >= 4) {
    *focal = p[0];
    *k1 = p[3];
    return true;
  }
  if (m == "RADIAL" && p.size() >= 5) {
    *focal = p[0];
    *k1 = p[3];
    *k2 = p[4];
    return true;
  }
  if (m == "OPENCV" && p.size() >= 8) {
    *focal = 0.5 * (p[0] + p[1]);
    *k1 = p[4];
    *k2 = p[5];
    return true;
  }
  if (m == "FULL_OPENCV" && p.size() >= 12) {
    *focal = 0.5 * (p[0] + p[1]);
    *k1 = p[4];
    *k2 = p[5];
    return true;
  }
  if (m == "SIMPLE_RADIAL_FISHEYE" && p.size() >= 4) {
    *focal = p[0];
    *k1 = p[3];
    return true;
  }
  if (m == "RADIAL_FISHEYE" && p.size() >= 5) {
    *focal = p[0];
    *k1 = p[3];
    *k2 = p[4];
    return true;
  }
  // Fallback: first parameter often focal
  if (!p.empty() && p[0] > 1e-6) {
    *focal = p[0];
    LOG(WARNING) << "colmap: unknown camera model \"" << m << "\", using params[0] as focal";
    return true;
  }
  return false;
}

static std::string resolve_image_path(const fs::path& sparse_dir, const std::string& name) {
  fs::path n(name);
  if (n.is_absolute() && fs::exists(n))
    return n.string();

  const fs::path try1 = sparse_dir / n;
  if (fs::exists(try1))
    return fs::weakly_canonical(try1).string();

  const fs::path base = n.filename();
  const fs::path images_sibling = sparse_dir.parent_path().parent_path() / "images" / base;
  if (fs::exists(images_sibling))
    return fs::weakly_canonical(images_sibling).string();

  const fs::path images_up = sparse_dir / ".." / "images" / base;
  if (fs::exists(images_up))
    return fs::weakly_canonical(images_up).string();

  // ETH3D 等：images.txt 常为另一台机器上的绝对路径 .../gt/dslr_images/xxx.JPG；在本机数据根下
  // 向上查找存在的 gt/dslr_images/<basename>。
  fs::path walk = sparse_dir;
  for (int depth = 0; depth < 8; ++depth) {
    const fs::path gt_dslr = walk / "gt" / "dslr_images" / base;
    if (fs::exists(gt_dslr))
      return fs::weakly_canonical(gt_dslr).string();
    if (walk.has_parent_path())
      walk = walk.parent_path();
    else
      break;
  }

  if (n.is_absolute())
    return n.string();
  return try1.string();
}

struct ImageBlock {
  int image_id = 0;
  int camera_id = 0;
  std::string name;
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  double focal = 0;
  double k1 = 0;
  double k2 = 0;
  int width_hint = 0;
  int height_hint = 0;
  std::vector<std::pair<float, float>> uv_by_idx; // POINT2D_IDX → (u,v)
};

static bool read_cameras_txt(const fs::path& path, std::unordered_map<int, ColmapCameraRow>* out,
                             std::string* err) {
  std::ifstream in(path);
  if (!in) {
    *err = "cannot open " + path.string();
    return false;
  }
  std::string line;
  while (std::getline(in, line)) {
    if (is_comment_or_empty(line))
      continue;
    std::istringstream iss(line);
    ColmapCameraRow row;
    iss >> row.id >> row.model >> row.width >> row.height;
    if (!iss) {
      *err = "bad cameras.txt line: " + line;
      return false;
    }
    double v;
    while (iss >> v)
      row.params.push_back(v);
    (*out)[row.id] = std::move(row);
  }
  if (out->empty()) {
    *err = "no cameras in " + path.string();
    return false;
  }
  return true;
}

static bool parse_points2d_line(const std::string& line,
                                std::vector<std::pair<float, float>>* uv_by_idx) {
  uv_by_idx->clear();
  std::istringstream iss(line);
  float x, y;
  long long pid = 0;
  while (iss >> x >> y >> pid) {
    (void)pid;
    uv_by_idx->push_back({x, y});
  }
  return !uv_by_idx->empty();
}

static bool read_images_txt(const fs::path& path,
                            const std::unordered_map<int, ColmapCameraRow>& cam_rows,
                            std::vector<ImageBlock>* images_out, std::string* err) {
  std::ifstream in(path);
  if (!in) {
    *err = "cannot open " + path.string();
    return false;
  }
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(in, line))
    lines.push_back(std::move(line));

  size_t i = 0;
  while (i < lines.size()) {
    while (i < lines.size() && is_comment_or_empty(lines[i]))
      ++i;
    if (i >= lines.size())
      break;

    std::istringstream iss(lines[i]);
    ImageBlock blk;
    double qw, qx, qy, qz;
    iss >> blk.image_id >> qw >> qx >> qy >> qz >> blk.t(0) >> blk.t(1) >> blk.t(2) >>
        blk.camera_id;
    if (!iss) {
      ++i;
      continue;
    }
    std::string name_token;
    iss >> name_token;
    if (name_token.empty()) {
      *err = "images.txt: missing NAME on line " + std::to_string(i + 1);
      return false;
    }
    // Remainder of line may be image path with spaces — rare; join tokens.
    std::string rest;
    std::getline(iss, rest);
    blk.name = name_token + rest;

    Eigen::Quaterniond q(qw, qx, qy, qz);
    q.normalize();
    blk.R = q.toRotationMatrix();

    auto cit = cam_rows.find(blk.camera_id);
    if (cit == cam_rows.end()) {
      *err = "images.txt: unknown CAMERA_ID " + std::to_string(blk.camera_id) + " for IMAGE_ID " +
             std::to_string(blk.image_id);
      return false;
    }
    blk.width_hint = cit->second.width;
    blk.height_hint = cit->second.height;
    if (!parse_camera_intrinsics(cit->second, &blk.focal, &blk.k1, &blk.k2)) {
      blk.focal = 0.5 * static_cast<double>(blk.width_hint + blk.height_hint);
      LOG(WARNING) << "colmap: could not parse intrinsics for CAMERA_ID " << blk.camera_id
                     << ", using heuristic focal";
    }

    ++i;
    if (i >= lines.size()) {
      *err = "images.txt: missing POINTS2D line after image " + std::to_string(blk.image_id);
      return false;
    }
    if (!parse_points2d_line(lines[i], &blk.uv_by_idx)) {
      // COLMAP may output empty second line when no keypoints
      blk.uv_by_idx.clear();
    }
    ++i;
    images_out->push_back(std::move(blk));
  }

  if (images_out->empty()) {
    *err = "no images in " + path.string();
    return false;
  }
  std::sort(images_out->begin(), images_out->end(),
            [](const ImageBlock& a, const ImageBlock& b) { return a.image_id < b.image_id; });
  return true;
}

static bool read_points3d_txt(
    const fs::path& path, const std::unordered_map<int, int>& image_id_to_cam_idx,
    const std::vector<ImageBlock>& images_by_sorted_idx, std::vector<BundlerPoint>* points_out,
    std::string* err) {
  std::ifstream in(path);
  if (!in) {
    *err = "cannot open " + path.string();
    return false;
  }

  // cam_idx → ImageBlock reference by sorted order (images_by_sorted_idx index == cam_idx)
  auto uv_lookup = [&](int image_id, int p2d_idx, float* u, float* v) -> bool {
    auto it = image_id_to_cam_idx.find(image_id);
    if (it == image_id_to_cam_idx.end())
      return false;
    const int cam_idx = it->second;
    if (cam_idx < 0 || cam_idx >= static_cast<int>(images_by_sorted_idx.size()))
      return false;
    const auto& im = images_by_sorted_idx[static_cast<size_t>(cam_idx)];
    if (p2d_idx < 0 || p2d_idx >= static_cast<int>(im.uv_by_idx.size()))
      return false;
    *u = im.uv_by_idx[static_cast<size_t>(p2d_idx)].first;
    *v = im.uv_by_idx[static_cast<size_t>(p2d_idx)].second;
    return true;
  };

  std::string line;
  while (std::getline(in, line)) {
    if (is_comment_or_empty(line))
      continue;
    std::istringstream iss(line);
    long long pid = 0;
    double x, y, z;
    int r = 0, g = 0, b = 0;
    double err_r = 0;
    iss >> pid >> x >> y >> z >> r >> g >> b >> err_r;
    if (!iss) {
      *err = "bad points3D.txt line";
      return false;
    }
    BundlerPoint p;
    p.xyz = Eigen::Vector3d(x, y, z);
    p.rgb = Eigen::Vector3d(static_cast<double>(r), static_cast<double>(g), static_cast<double>(b));

    int img_id = 0;
    int p2d_idx = 0;
    while (iss >> img_id >> p2d_idx) {
      float u = 0, v = 0;
      if (!uv_lookup(img_id, p2d_idx, &u, &v))
        continue;
      auto it = image_id_to_cam_idx.find(img_id);
      if (it == image_id_to_cam_idx.end())
        continue;
      BundlerObservation o;
      o.cam_idx = it->second;
      o.key_idx = p2d_idx;
      o.u = u;
      o.v = v;
      p.observations.push_back(o);
    }
    points_out->push_back(std::move(p));
  }
  return true;
}

} // namespace

bool load_colmap_text_directory(const std::string& colmap_sparse_dir, BundlerScene* scene,
                                std::string* error_message) {
  if (!scene || !error_message)
    return false;

  // Same as load_bundler_directory: required before GetWidthHeightPixel in fill_render_tracks_from_bundler.
  GdalUtils::InitGDAL();

  std::error_code ec;
  const fs::path root = fs::absolute(colmap_sparse_dir, ec);
  if (ec || !fs::is_directory(root)) {
    *error_message = "not a directory: " + colmap_sparse_dir;
    return false;
  }

  const fs::path cam_path = root / "cameras.txt";
  const fs::path img_path = root / "images.txt";
  const fs::path pts_path = root / "points3D.txt";
  if (!fs::exists(cam_path) || !fs::exists(img_path) || !fs::exists(pts_path)) {
    *error_message =
        "COLMAP text model requires cameras.txt, images.txt, points3D.txt in " + root.string();
    return false;
  }

  std::unordered_map<int, ColmapCameraRow> cam_rows;
  if (!read_cameras_txt(cam_path, &cam_rows, error_message))
    return false;

  std::vector<ImageBlock> blocks;
  if (!read_images_txt(img_path, cam_rows, &blocks, error_message))
    return false;

  scene->image_paths.clear();
  scene->cameras.clear();
  scene->points.clear();

  std::unordered_map<int, int> image_id_to_cam_idx;
  for (size_t i = 0; i < blocks.size(); ++i) {
    image_id_to_cam_idx[blocks[i].image_id] = static_cast<int>(i);
  }

  scene->image_paths.reserve(blocks.size());
  scene->cameras.reserve(blocks.size());

  for (const ImageBlock& blk : blocks) {
    BundlerCamera c;
    c.focal = blk.focal;
    c.k1 = blk.k1;
    c.k2 = blk.k2;
    c.R = kColmapCvCameraToBundlerLikeCamera * blk.R;
    c.t = kColmapCvCameraToBundlerLikeCamera * blk.t;
    c.image_width = blk.width_hint;
    c.image_height = blk.height_hint;
    scene->cameras.push_back(c);
    scene->image_paths.push_back(resolve_image_path(root, blk.name));
  }

  if (!read_points3d_txt(pts_path, image_id_to_cam_idx, blocks, &scene->points, error_message))
    return false;

  LOG(INFO) << "colmap text: " << scene->cameras.size() << " cameras, " << scene->points.size()
            << " points from " << root.string();
  return true;
}

} // namespace render
} // namespace insight
