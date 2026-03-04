/**
 * @file ColmapExporter.h
 * @brief COLMAP格式导出器
 *
 * 将 database::Project 导出为 COLMAP 数据库格式
 * COLMAP数据库包含：
 * - database.db (SQLite)
 *   - images 表: image_id, name, camera_id, qvec, tvec, ...
 *   - cameras 表: camera_id, model, width, height, params
 *   - keypoints 表: image_id, keypoints (二进制)
 *   - descriptors 表: image_id, descriptors (二进制)
 *   - matches 表: image_id1, image_id2, matches (二进制)
 * - images/ 文件夹: 原始图像文件
 * - sparse/ 文件夹: 稀疏模型 (images.txt, cameras.txt, points3D.txt)
 */

#ifndef ALGORITHM_COLMAPEXPORTER_H
#define ALGORITHM_COLMAPEXPORTER_H

#include <map>
#include <memory>
#include <string>

namespace insight {
namespace database {
struct Project;
}

namespace algorithm {

/**
 * @class ColmapExporter
 * @brief COLMAP格式导出器
 */
class ColmapExporter {
public:
  ColmapExporter() = default;
  ~ColmapExporter() = default;

  /**
   * 导出项目到COLMAP格式
   *
   * @param[in] project 要导出的项目
   * @param[in] outputDir 输出目录（将创建以下文件：
   *                      - outputDir/database.db
   *                      - outputDir/images/
   *                      - outputDir/sparse/
   * @param[in] options 导出选项：
   *                    - "copy_images": true/false (是否复制图像文件)
   *                    - "link_images": true/false (是否符号链接图像)
   * @return 成功返回true
   */
  bool export_project(const insight::database::Project& project, const std::string& outputDir,
                      const std::map<std::string, std::string>& options = {});

  std::string get_last_error() const { return m_lastError; }

private:
  bool create_directory_structure(const std::string& outputDir);
  bool create_colmap_database(const insight::database::Project& project, const std::string& dbPath);
  bool link_image_files(const insight::database::Project& project, const std::string& outputDir,
                        const std::map<std::string, std::string>& options);
  bool create_sparse_files(const insight::database::Project& project, const std::string& sparseDir);
  bool write_images_text(const insight::database::Project& project, const std::string& filepath);
  bool write_cameras_text(const insight::database::Project& project, const std::string& filepath);
  bool write_points3d_text(const insight::database::Project& project, const std::string& filepath);

private:
  std::string m_lastError;
};

} // namespace algorithm
} // namespace insight

#endif // ALGORITHM_COLMAPEXPORTER_H
