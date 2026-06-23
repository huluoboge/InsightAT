/**
 * @file  colmap_loader.cpp
 * @brief COLMAP sparse text model → BundlerScene（world→camera: Xc = R*Xw + t）。
 *
 * COLMAP 相机轴为 CV 惯例（+X 右、+Y 下、+Z 前）；渲染里 draw_camera_image_frame 按 OpenGL
 * 视锥（+Y 上、沿 -Z 看）绘制。导入时在保持光心 C = -R^T t 不变的前提下左乘对角(1,-1,-1)，
 * 使 COLMAP 与 Bundler 在可视化里朝向一致。
 */

#include "colmap_loader.h"

#include "ImageIO/gdal_utils.h"

#include <glog/logging.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cctype>
#include <cstdint>
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

/// 写入 BundlerCamera 的像素主点：优先 cameras.txt 内参里的 cx,cy；否则假定主点在图像中心 (W/2,H/2)。
static void fill_bundler_camera_principal_from_colmap_row(const ColmapCameraRow& row,
                                                        BundlerCamera* c) {
  const auto& p = row.params;
  const double def_cx = 0.5 * static_cast<double>(row.width);
  const double def_cy = 0.5 * static_cast<double>(row.height);
  const std::string& m = row.model;
  bool have = false;
  if (m == "SIMPLE_PINHOLE" && p.size() >= 3) {
    c->principal_cx = p[1];
    c->principal_cy = p[2];
    have = true;
  } else if (m == "PINHOLE" && p.size() >= 4) {
    c->principal_cx = p[2];
    c->principal_cy = p[3];
    have = true;
  } else if (m == "SIMPLE_RADIAL" && p.size() >= 3) {
    c->principal_cx = p[1];
    c->principal_cy = p[2];
    have = true;
  } else if (m == "RADIAL" && p.size() >= 3) {
    c->principal_cx = p[1];
    c->principal_cy = p[2];
    have = true;
  } else if (m == "OPENCV" && p.size() >= 4) {
    c->principal_cx = p[2];
    c->principal_cy = p[3];
    have = true;
  } else if (m == "FULL_OPENCV" && p.size() >= 4) {
    c->principal_cx = p[2];
    c->principal_cy = p[3];
    have = true;
  } else if (m == "SIMPLE_RADIAL_FISHEYE" && p.size() >= 3) {
    c->principal_cx = p[1];
    c->principal_cy = p[2];
    have = true;
  } else if (m == "RADIAL_FISHEYE" && p.size() >= 3) {
    c->principal_cx = p[1];
    c->principal_cy = p[2];
    have = true;
  }
  if (!have) {
    c->principal_cx = def_cx;
    c->principal_cy = def_cy;
  }
}

/// Generate alternate filenames from common COLMAP naming conventions.
/// E.g. "image_08749.jpg" → {"00008749.jpg", "08749.jpg", "8749.jpg"}
static std::vector<std::string> alternate_names(const std::string& name) {
  std::vector<std::string> alts;
  const std::string stem = fs::path(name).stem().string();
  const std::string ext  = fs::path(name).extension().string();

  // Strip known prefixes: "image_", "img_", "frame_"
  for (const char* prefix : {"image_", "img_", "frame_"}) {
    if (stem.size() > strlen(prefix) && stem.compare(0, strlen(prefix), prefix) == 0) {
      std::string num = stem.substr(strlen(prefix));
      // Try zero-padding to 4, 5, 6, 8 digits (common COLMAP formats).
      for (int width : {8, 6, 5, 4}) {
        if (num.size() < static_cast<size_t>(width)) {
          std::string padded(width - num.size(), '0');
          alts.push_back(padded + num + ext);
        }
      }
      // Also try without padding.
      alts.push_back(num + ext);
      break;
    }
  }
  return alts;
}

/// Try to find `candidate` at each of the standard search directories.
/// Returns the resolved path on first hit, empty string if not found.
static std::string try_resolve(const fs::path& sparse_dir, const std::string& candidate) {
  const fs::path base = fs::path(candidate).filename();

  // Same-directory
  fs::path p = sparse_dir / base;
  if (fs::exists(p))
    return fs::weakly_canonical(p).string();

  // ../../images/
  p = sparse_dir.parent_path().parent_path() / "images" / base;
  if (fs::exists(p))
    return fs::weakly_canonical(p).string();

  // ../images/
  p = sparse_dir / ".." / "images" / base;
  if (fs::exists(p))
    return fs::weakly_canonical(p).string();

  // ../../images/
  p = sparse_dir / ".." / ".." / "images" / base;
  if (fs::exists(p))
    return fs::weakly_canonical(p).string();

  // ../../../images/
  p = sparse_dir / ".." / ".." / ".." / "images" / base;
  if (fs::exists(p))
    return fs::weakly_canonical(p).string();

  // Walk up looking for images/<base>
  fs::path walk = sparse_dir;
  for (int depth = 0; depth < 10; ++depth) {
    p = walk / "images" / base;
    if (fs::exists(p))
      return fs::weakly_canonical(p).string();
    p = walk / "gt" / "dslr_images" / base;
    if (fs::exists(p))
      return fs::weakly_canonical(p).string();
    if (walk.has_parent_path())
      walk = walk.parent_path();
    else
      break;
  }
  return {};
}

static std::string resolve_image_path(const fs::path& sparse_dir, const std::string& name) {
  fs::path n(name);

  // Absolute path that already exists.
  if (n.is_absolute() && fs::exists(n))
    return n.string();

  // 1) Try the original name.
  std::string found = try_resolve(sparse_dir, name);
  if (!found.empty())
    return found;

  // 2) Try alternate namings (image_08749.jpg → 00008749.jpg etc.)
  for (const auto& alt : alternate_names(name)) {
    found = try_resolve(sparse_dir, alt);
    if (!found.empty()) {
      LOG(INFO) << "resolve_image_path: mapped '" << name << "' -> '" << alt
                << "' -> " << found;
      return found;
    }
  }

  // 3) Absolute-but-missing path, or guarded fallback.
  if (n.is_absolute())
    return n.string();
  return (sparse_dir / n).string();
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
                            std::vector<ImageBlock>* images_out, std::string* err,
                            const ReconstructionLoadProgress* progress) {
  std::ifstream in(path);
  if (!in) {
    *err = "cannot open " + path.string();
    return false;
  }
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(in, line))
    lines.push_back(std::move(line));

  if (progress && *progress)
    (*progress)(0, -1, "images");

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
    if (progress && *progress &&
        (images_out->size() % 256u == 0u ||
         images_out->size() == 1u))
      (*progress)(static_cast<int64_t>(images_out->size()), -1, "images");
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
    std::string* err, const ReconstructionLoadProgress* progress) {
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

  if (progress && *progress)
    (*progress)(0, -1, "points3D");

  std::string line;
  int64_t n_pts = 0;
  constexpr int64_t kPtsStride = 2048;
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
    ++n_pts;
    if (progress && *progress &&
        (n_pts == 1 || (n_pts % kPtsStride == 0)))
      (*progress)(n_pts, -1, "points3D");
  }
  if (progress && *progress && n_pts > 0 && (n_pts % kPtsStride != 0))
    (*progress)(n_pts, -1, "points3D");
  return true;
}

} // namespace

bool load_colmap_text_directory(const std::string& colmap_sparse_dir, BundlerScene* scene,
                                std::string* error_message, ReconstructionLoadProgress progress) {
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

  const ReconstructionLoadProgress* prog_ptr = progress ? &progress : nullptr;

  std::vector<ImageBlock> blocks;
  if (!read_images_txt(img_path, cam_rows, &blocks, error_message, prog_ptr))
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
    auto crow = cam_rows.find(blk.camera_id);
    if (crow != cam_rows.end())
      fill_bundler_camera_principal_from_colmap_row(crow->second, &c);
    else {
      c.principal_cx = 0.5 * static_cast<double>(blk.width_hint);
      c.principal_cy = 0.5 * static_cast<double>(blk.height_hint);
    }
    scene->cameras.push_back(c);
    scene->image_paths.push_back(resolve_image_path(root, blk.name));
  }

  if (!read_points3d_txt(pts_path, image_id_to_cam_idx, blocks, &scene->points, error_message,
                         prog_ptr))
    return false;

  LOG(INFO) << "colmap text: " << scene->cameras.size() << " cameras, " << scene->points.size()
            << " points from " << root.string();
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Binary COLMAP reader  (cameras.bin / images.bin / points3D.bin)
// ─────────────────────────────────────────────────────────────────────────────

namespace {

/// Map COLMAP camera model_id to number of double parameters.
/// See https://colmap.github.io/format.html#output-format
static int colmap_model_num_params(int model_id) {
  switch (model_id) {
    case 0:  return 3;   // SIMPLE_PINHOLE: f, cx, cy
    case 1:  return 4;   // PINHOLE: fx, fy, cx, cy
    case 2:  return 4;   // SIMPLE_RADIAL: f, cx, cy, k
    case 3:  return 5;   // RADIAL: f, cx, cy, k1, k2
    case 4:  return 8;   // OPENCV: fx, fy, cx, cy, k1, k2, p1, p2
    case 5:  return 8;   // OPENCV_FISHEYE: fx, fy, cx, cy, k1, k2, k3, k4
    case 6:  return 12;  // FULL_OPENCV: fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, k5, k6
    case 7:  return 5;   // FOV: fx, fy, cx, cy, omega
    case 8:  return 4;   // SIMPLE_RADIAL_FISHEYE: f, cx, cy, k
    case 9:  return 5;   // RADIAL_FISHEYE: f, cx, cy, k1, k2
    case 10: return 12;  // THIN_PRISM_FISHEYE: fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, sx1, sy1
    default: return -1;  // unknown
  }
}

/// Convert binary model_id to the text-format model name string (for reuse of parse_camera_intrinsics).
static std::string colmap_model_name(int model_id) {
  switch (model_id) {
    case 0:  return "SIMPLE_PINHOLE";
    case 1:  return "PINHOLE";
    case 2:  return "SIMPLE_RADIAL";
    case 3:  return "RADIAL";
    case 4:  return "OPENCV";
    case 5:  return "OPENCV_FISHEYE";
    case 6:  return "FULL_OPENCV";
    case 7:  return "FOV";
    case 8:  return "SIMPLE_RADIAL_FISHEYE";
    case 9:  return "RADIAL_FISHEYE";
    case 10: return "THIN_PRISM_FISHEYE";
    default: return "PINHOLE";
  }
}

/// Read cameras.bin.  Layout (per COLMAP spec, all little-endian):
///   uint64_t      num_cameras
///   for each:
///     uint32_t    camera_id
///     int32_t     model_id
///     uint64_t    width
///     uint64_t    height
///     double[N]   params  (N = colmap_model_num_params(model_id))
static bool read_cameras_bin(const fs::path& path, std::unordered_map<int, ColmapCameraRow>* out,
                             std::string* err) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    *err = "cannot open " + path.string();
    return false;
  }

  uint64_t num_cams = 0;
  in.read(reinterpret_cast<char*>(&num_cams), sizeof(uint64_t));
  if (!in || num_cams == 0) {
    *err = "cameras.bin: empty or missing count";
    return false;
  }
  if (num_cams > 100'000'000ULL) {
    *err = "cameras.bin: unreasonably large camera count " + std::to_string(num_cams);
    return false;
  }

  out->reserve(static_cast<size_t>(num_cams));
  for (uint64_t i = 0; i < num_cams; ++i) {
    uint32_t cam_id = 0;
    int32_t  model_id = 0;
    uint64_t w = 0, h = 0;
    in.read(reinterpret_cast<char*>(&cam_id), sizeof(uint32_t));
    in.read(reinterpret_cast<char*>(&model_id), sizeof(int32_t));
    in.read(reinterpret_cast<char*>(&w), sizeof(uint64_t));
    in.read(reinterpret_cast<char*>(&h), sizeof(uint64_t));

    const int np = colmap_model_num_params(model_id);
    if (np < 0) {
      *err = "cameras.bin: camera " + std::to_string(cam_id) +
             " has unknown model_id " + std::to_string(model_id);
      return false;
    }

    ColmapCameraRow row;
    row.id = static_cast<int>(cam_id);
    row.model = colmap_model_name(model_id);
    row.width = static_cast<int>(w);
    row.height = static_cast<int>(h);
    row.params.resize(static_cast<size_t>(np));

    in.read(reinterpret_cast<char*>(row.params.data()), static_cast<std::streamsize>(np * sizeof(double)));
    if (!in) {
      *err = "cameras.bin: unexpected EOF at camera " + std::to_string(cam_id);
      return false;
    }
    (*out)[row.id] = std::move(row);
  }
  return true;
}

/// Read images.bin.  Layout:
///   uint64_t        num_images
///   for each registered image:
///     uint32_t      image_id
///     double[4]     qvec  (qw, qx, qy, qz) — world-to-camera rotation
///     double[3]     tvec  (tx, ty, tz)
///     uint32_t      camera_id
///     char[]        name  (null-terminated)
///     uint64_t      num_points2D
///     for each point2D:
///       double      x
///       double      y
///       uint64_t    point3D_id  (UINT64_MAX = no association)
static bool read_images_bin(const fs::path& path,
                            const std::unordered_map<int, ColmapCameraRow>& cam_rows,
                            std::vector<ImageBlock>* images_out, std::string* err,
                            const ReconstructionLoadProgress* progress) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    *err = "cannot open " + path.string();
    return false;
  }

  uint64_t num_imgs = 0;
  in.read(reinterpret_cast<char*>(&num_imgs), sizeof(uint64_t));
  if (!in || num_imgs == 0) {
    *err = "images.bin: empty or missing count";
    return false;
  }
  if (num_imgs > 10'000'000ULL) {
    *err = "images.bin: unreasonably large image count " + std::to_string(num_imgs);
    return false;
  }

  if (progress && *progress)
    (*progress)(0, -1, "images");

  images_out->reserve(static_cast<size_t>(num_imgs));

  for (uint64_t i = 0; i < num_imgs; ++i) {
    uint32_t image_id = 0;
    double qvec[4] = {0, 0, 0, 0};
    double tvec[3] = {0, 0, 0};
    uint32_t camera_id = 0;

    in.read(reinterpret_cast<char*>(&image_id), sizeof(uint32_t));
    in.read(reinterpret_cast<char*>(qvec), 4 * sizeof(double));
    in.read(reinterpret_cast<char*>(tvec), 3 * sizeof(double));
    in.read(reinterpret_cast<char*>(&camera_id), sizeof(uint32_t));

    // null-terminated name
    std::string name;
    std::getline(in, name, '\0');
    if (!in) {
      *err = "images.bin: unexpected EOF at image " + std::to_string(image_id);
      return false;
    }

    uint64_t num_pts2d = 0;
    in.read(reinterpret_cast<char*>(&num_pts2d), sizeof(uint64_t));
    if (!in) {
      *err = "images.bin: unexpected EOF reading num_points2D for image " + std::to_string(image_id);
      return false;
    }

    ImageBlock blk;
    blk.image_id = static_cast<int>(image_id);
    blk.camera_id = static_cast<int>(camera_id);
    blk.name = name;

    // Quaternion → rotation
    Eigen::Quaterniond q(qvec[0], qvec[1], qvec[2], qvec[3]);
    q.normalize();
    blk.R = q.toRotationMatrix();
    blk.t = Eigen::Vector3d(tvec[0], tvec[1], tvec[2]);

    auto cit = cam_rows.find(blk.camera_id);
    if (cit == cam_rows.end()) {
      *err = "images.bin: unknown CAMERA_ID " + std::to_string(blk.camera_id) +
             " for IMAGE_ID " + std::to_string(blk.image_id);
      return false;
    }
    blk.width_hint = cit->second.width;
    blk.height_hint = cit->second.height;
    if (!parse_camera_intrinsics(cit->second, &blk.focal, &blk.k1, &blk.k2)) {
      blk.focal = 0.5 * static_cast<double>(blk.width_hint + blk.height_hint);
      LOG(WARNING) << "colmap bin: could not parse intrinsics for CAMERA_ID " << blk.camera_id
                     << ", using heuristic focal";
    }

    // Read 2D points
    if (num_pts2d > 0) {
      blk.uv_by_idx.reserve(static_cast<size_t>(num_pts2d));
      // Batch-read to reduce I/O calls
      constexpr size_t kBufSize = 8192;
      struct { double x, y; uint64_t pt3d_id; } buf[kBufSize];
      uint64_t remaining = num_pts2d;
      while (remaining > 0) {
        const size_t chunk = static_cast<size_t>(std::min(remaining, uint64_t{kBufSize}));
        in.read(reinterpret_cast<char*>(buf),
                static_cast<std::streamsize>(chunk * (2 * sizeof(double) + sizeof(uint64_t))));
        if (!in) {
          *err = "images.bin: unexpected EOF reading point2D for image " + std::to_string(image_id);
          return false;
        }
        for (size_t j = 0; j < chunk; ++j)
          blk.uv_by_idx.emplace_back(static_cast<float>(buf[j].x), static_cast<float>(buf[j].y));
        remaining -= static_cast<uint64_t>(chunk);
      }
    }

    images_out->push_back(std::move(blk));

    if (progress && *progress &&
        (images_out->size() % 256u == 0u || images_out->size() == 1u))
      (*progress)(static_cast<int64_t>(images_out->size()), -1, "images");
  }

  if (images_out->empty()) {
    *err = "no images in " + path.string();
    return false;
  }
  std::sort(images_out->begin(), images_out->end(),
            [](const ImageBlock& a, const ImageBlock& b) { return a.image_id < b.image_id; });
  return true;
}

/// Read points3D.bin.  Layout:
///   uint64_t        num_points
///   for each point:
///     uint64_t      point3D_id
///     double[3]     xyz
///     uint8_t[3]    rgb
///     double        error
///     uint64_t      track_length
///     for each track element:
///       uint32_t    image_id
///       uint32_t    point2D_idx
static bool read_points3d_bin(
    const fs::path& path,
    const std::unordered_map<int, int>& image_id_to_cam_idx,
    const std::vector<ImageBlock>& images_by_sorted_idx,
    std::vector<BundlerPoint>* points_out, std::string* err,
    const ReconstructionLoadProgress* progress) {

  std::ifstream in(path, std::ios::binary);
  if (!in) {
    *err = "cannot open " + path.string();
    return false;
  }

  uint64_t num_pts = 0;
  in.read(reinterpret_cast<char*>(&num_pts), sizeof(uint64_t));
  if (!in) {
    // Empty file or read error — treat as zero points (valid empty reconstruction).
    if (in.eof() && in.gcount() == 0) {
      // File is 0 bytes — valid but empty.
      num_pts = 0;
      in.clear(); // clear EOF so further checks pass
    } else {
      *err = "points3D.bin: failed to read point count";
      return false;
    }
  }
  if (num_pts == 0) {
    // Valid empty reconstruction — return with empty points_out.
    if (progress && *progress)
      (*progress)(0, -1, "points3D");
    return true;
  }
  if (num_pts > 500'000'000ULL) {
    *err = "points3D.bin: unreasonably large point count " + std::to_string(num_pts);
    return false;
  }

  // uv_lookup: resolves (image_id, point2D_idx) → (u, v)
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

  if (progress && *progress)
    (*progress)(0, -1, "points3D");

  points_out->reserve(static_cast<size_t>(std::min(num_pts, uint64_t{10'000'000})));

  constexpr int64_t kPtsStride = 2048;
  int64_t n_loaded = 0;

  for (uint64_t i = 0; i < num_pts; ++i) {
    uint64_t point_id = 0;
    double xyz[3] = {0, 0, 0};
    uint8_t rgb[3] = {0, 0, 0};
    double error_val = 0.0;
    uint64_t track_len = 0;

    in.read(reinterpret_cast<char*>(&point_id), sizeof(uint64_t));
    in.read(reinterpret_cast<char*>(xyz), 3 * sizeof(double));
    in.read(reinterpret_cast<char*>(rgb), 3);
    in.read(reinterpret_cast<char*>(&error_val), sizeof(double));
    in.read(reinterpret_cast<char*>(&track_len), sizeof(uint64_t));

    if (!in) {
      *err = "points3D.bin: unexpected EOF at point " + std::to_string(point_id);
      return false;
    }

    BundlerPoint p;
    p.xyz = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
    p.rgb = Eigen::Vector3d(static_cast<double>(rgb[0]),
                            static_cast<double>(rgb[1]),
                            static_cast<double>(rgb[2]));
    (void)error_val; // not used in BundlerScene

    if (track_len > 0) {
      p.observations.reserve(static_cast<size_t>(track_len));
      // Batch-read track entries
      constexpr size_t kTrackBufSize = 4096;
      struct { uint32_t img_id; uint32_t p2d_idx; } track_buf[kTrackBufSize];
      uint64_t remaining = track_len;
      while (remaining > 0) {
        const size_t chunk = static_cast<size_t>(std::min(remaining, uint64_t{kTrackBufSize}));
        in.read(reinterpret_cast<char*>(track_buf),
                static_cast<std::streamsize>(chunk * 2 * sizeof(uint32_t)));
        if (!in) {
          *err = "points3D.bin: unexpected EOF reading track for point " + std::to_string(point_id);
          return false;
        }
        for (size_t j = 0; j < chunk; ++j) {
          const int img_id = static_cast<int>(track_buf[j].img_id);
          const int p2d_idx = static_cast<int>(track_buf[j].p2d_idx);
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
        remaining -= static_cast<uint64_t>(chunk);
      }
    }

    points_out->push_back(std::move(p));
    ++n_loaded;
    if (progress && *progress && (n_loaded == 1 || (n_loaded % kPtsStride == 0)))
      (*progress)(n_loaded, -1, "points3D");
  }

  if (progress && *progress && n_loaded > 0 && (n_loaded % kPtsStride != 0))
    (*progress)(n_loaded, -1, "points3D");

  return true;
}

} // namespace

bool load_colmap_binary_directory(const std::string& colmap_sparse_dir, BundlerScene* scene,
                                  std::string* error_message, ReconstructionLoadProgress progress) {
  if (!scene || !error_message)
    return false;

  GdalUtils::InitGDAL();

  std::error_code ec;
  const fs::path root = fs::absolute(colmap_sparse_dir, ec);
  if (ec || !fs::is_directory(root)) {
    *error_message = "not a directory: " + colmap_sparse_dir;
    return false;
  }

  const fs::path cam_path = root / "cameras.bin";
  const fs::path img_path = root / "images.bin";
  const fs::path pts_path = root / "points3D.bin";
  if (!fs::exists(cam_path) || !fs::exists(img_path) || !fs::exists(pts_path)) {
    *error_message =
        "COLMAP binary model requires cameras.bin, images.bin, points3D.bin in " + root.string();
    return false;
  }

  std::unordered_map<int, ColmapCameraRow> cam_rows;
  if (!read_cameras_bin(cam_path, &cam_rows, error_message))
    return false;

  const ReconstructionLoadProgress* prog_ptr = progress ? &progress : nullptr;

  std::vector<ImageBlock> blocks;
  if (!read_images_bin(img_path, cam_rows, &blocks, error_message, prog_ptr))
    return false;

  // Build image_id → sorted cam_idx map (same pattern as text loader)
  std::unordered_map<int, int> image_id_to_cam_idx;
  for (size_t i = 0; i < blocks.size(); ++i)
    image_id_to_cam_idx[blocks[i].image_id] = static_cast<int>(i);

  scene->image_paths.clear();
  scene->cameras.clear();
  scene->points.clear();

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
    auto crow = cam_rows.find(blk.camera_id);
    if (crow != cam_rows.end())
      fill_bundler_camera_principal_from_colmap_row(crow->second, &c);
    else {
      c.principal_cx = 0.5 * static_cast<double>(blk.width_hint);
      c.principal_cy = 0.5 * static_cast<double>(blk.height_hint);
    }
    scene->cameras.push_back(c);
    scene->image_paths.push_back(resolve_image_path(root, blk.name));
  }

  if (!read_points3d_bin(pts_path, image_id_to_cam_idx, blocks, &scene->points, error_message,
                         prog_ptr))
    return false;

  LOG(INFO) << "colmap binary: " << scene->cameras.size() << " cameras, "
            << scene->points.size() << " points from " << root.string();
  return true;
}

} // namespace render
} // namespace insight
