/**
 * @file ColmapExporter.cpp
 * @brief COLMAP导出器实现（骨架版本）
 *
 * 完整的COLMAP导出会很复杂，这里先提供基础框架，
 * 后续可以逐步完善实现细节。
 */

#include "ColmapExporter.h"
#include "database/database_types.h"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <glog/logging.h>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace fs = std::filesystem;

namespace insight {
namespace algorithm {

bool ColmapExporter::export_project(const insight::database::Project& project,
                                    const std::string& outputDir,
                                    const std::map<std::string, std::string>& options) {
  LOG(INFO) << "Starting COLMAP export to: " << outputDir;

  try {
    if (!create_directory_structure(outputDir)) {
      m_lastError = "Failed to create directory structure";
      return false;
    }

    std::string dbPath = outputDir + "/database.db";
    if (!create_colmap_database(project, dbPath)) {
      m_lastError = "Failed to create COLMAP database";
      return false;
    }

    if (!link_image_files(project, outputDir, options)) {
      m_lastError = "Failed to link image files";
      return false;
    }

    std::string sparseDir = outputDir + "/sparse/0";
    if (!create_sparse_files(project, sparseDir)) {
      m_lastError = "Failed to create sparse files";
      return false;
    }

    LOG(INFO) << "COLMAP export completed successfully";
    return true;

  } catch (const std::exception& e) {
    m_lastError = std::string("Export failed: ") + e.what();
    LOG(ERROR) << m_lastError;
    return false;
  }
}

bool ColmapExporter::create_directory_structure(const std::string& outputDir) {
  try {
    fs::path base(outputDir);
    fs::create_directories(base / "images");
    fs::create_directories(base / "sparse/0");
    LOG(INFO) << "Directory structure created";
    return true;
  } catch (const std::exception& e) {
    LOG(ERROR) << "Failed to create directory: " << e.what();
    return false;
  }
}

bool ColmapExporter::create_colmap_database(const insight::database::Project& project,
                                             const std::string& dbPath) {
  // TODO: 实现COLMAP数据库创建
  // 这需要：
  // 1. SQLite库支持
  // 2. 创建必要的表（images, cameras, keypoints, matches等）
  // 3. 填充相机和图像信息

  LOG(WARNING) << "COLMAP database creation not yet fully implemented";
  LOG(INFO) << "Database path would be: " << dbPath;

  // 暂时创建空文件占位
  std::ofstream file(dbPath, std::ios::binary);
  if (!file.is_open()) {
    LOG(ERROR) << "Failed to create database file";
    return false;
  }
  file.close();

  return true;
}

bool ColmapExporter::link_image_files(const insight::database::Project& project,
                                      const std::string& outputDir,
                                      const std::map<std::string, std::string>& options) {
  std::string imagesDir = outputDir + "/images";

  // 获取选项
  bool copyImages = false;
  auto it_copy = options.find("copy_images");
  if (it_copy != options.end()) {
    copyImages = (it_copy->second == "true");
  }

  bool linkImages = true;
  auto it_link = options.find("link_images");
  if (it_link != options.end()) {
    linkImages = (it_link->second == "true");
  }

  LOG(INFO) << "Processing image files (copy=" << copyImages << ", link=" << linkImages << ")";

  int imageCount = 0;
  for (const auto& group : project.image_groups) {
    for (const auto& image : group.images) {
      std::string srcPath = image.filename;
      std::string filename = fs::path(srcPath).filename().string();
      std::string dstPath = imagesDir + "/" + filename;

      try {
        if (copyImages) {
          fs::copy_file(srcPath, dstPath, fs::copy_options::overwrite_existing);
        } else if (linkImages) {
          // 创建符号链接
          if (!fs::exists(dstPath)) {
            fs::create_symlink(srcPath, dstPath);
          }
        }
        imageCount++;
      } catch (const std::exception& e) {
        LOG(WARNING) << "Failed to process image " << srcPath << ": " << e.what();
      }
    }
  }

  LOG(INFO) << "Processed " << imageCount << " images";
  return true;
}

bool ColmapExporter::create_sparse_files(const insight::database::Project& project,
                                          const std::string& sparseDir) {
  // 创建images.txt
  std::string imagesPath = sparseDir + "/images.txt";
  if (!write_images_text(project, imagesPath)) {
    return false;
  }

  // 创建cameras.txt
  std::string camerasPath = sparseDir + "/cameras.txt";
  if (!write_cameras_text(project, camerasPath)) {
    return false;
  }

  // 创建points3D.txt
  std::string points3DPath = sparseDir + "/points3D.txt";
  if (!write_points3d_text(project, points3DPath)) {
    return false;
  }

  return true;
}

bool ColmapExporter::write_images_text(const insight::database::Project& project,
                                     const std::string& filepath) {
  std::ofstream file(filepath);
  if (!file.is_open()) {
    LOG(ERROR) << "Failed to open images.txt for writing: " << filepath;
    return false;
  }

  // COLMAP images.txt 格式：
  // # image_list.txt
  // IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
  // ...

  uint32_t imageId = 1;
  for (const auto& group : project.image_groups) {
    uint32_t cameraId = (group.group_camera ? 1 : 0); // 简化

    for (const auto& image : group.images) {
      file << imageId << " "
           << "1 0 0 0 " // quaternion (qw qx qy qz)
           << "0 0 0 "   // translation (tx ty tz)
           << cameraId << " " << fs::path(image.filename).filename().string() // 仅保留文件名
           << "\n\n"; // COLMAP 每两行一张图

      imageId++;
    }
  }

  file.close();
  LOG(INFO) << "Created images.txt with " << (imageId - 1) << " images";
  return true;
}

bool ColmapExporter::write_cameras_text(const insight::database::Project& project,
                                      const std::string& filepath) {
  std::ofstream file(filepath);
  if (!file.is_open()) {
    LOG(ERROR) << "Failed to open cameras.txt for writing: " << filepath;
    return false;
  }

  uint32_t cameraId = 1;
  for (const auto& group : project.image_groups) {
    if (group.group_camera) {
      const auto& camera = *group.group_camera;

      // OPENCV: no k3. FULL_OPENCV: k1 k2 p1 p2 k3 k4 k5 k6 with k4=k5=k6=0 for polynomial radial.
      file << cameraId << " ";
      if (std::abs(camera.k3) <= 1e-12) {
        file << "OPENCV " << camera.width << " " << camera.height << " ";
      } else {
        file << "FULL_OPENCV " << camera.width << " " << camera.height << " ";
      }
      // OpenCV tangential order vs ContextCapture storage: write OpenCV p1=p2, p2=p1.
      file << std::fixed << std::setprecision(6) << camera.focal_length << " "
           << camera.focal_length * camera.aspect_ratio << " " << camera.principal_point_x << " "
           << camera.principal_point_y << " " << camera.k1 << " " << camera.k2 << " " << camera.p2
           << " " << camera.p1;
      if (std::abs(camera.k3) > 1e-12) {
        file << " " << camera.k3 << " 0 0 0";
      }
      file << "\n";

      cameraId++;
    }
  }

  file.close();
  LOG(INFO) << "Created cameras.txt with " << (cameraId - 1) << " cameras";
  return true;
}

bool ColmapExporter::write_points3d_text(const insight::database::Project& project,
                                       const std::string& filepath) {
  std::ofstream file(filepath);
  if (!file.is_open()) {
    LOG(ERROR) << "Failed to open points3D.txt for writing: " << filepath;
    return false;
  }

  uint32_t point3DId = 1;
  for (const auto& [gcp_id, gcp] : project.gcp_database) {
    file << point3DId << " " << std::fixed << std::setprecision(6) << gcp.x << " " << gcp.y << " "
         << gcp.z << " "
         << "255 255 255 0 "; // Colors (RGB) + Error

    for (const auto& obs : gcp.observations) {
      file << obs.image_id << " 0 ";
    }

    file << "\n";
    point3DId++;
  }

  file.close();
  LOG(INFO) << "Created points3D.txt with " << (point3DId - 1) << " points";
  return true;
}

} // namespace algorithm
} // namespace insight
