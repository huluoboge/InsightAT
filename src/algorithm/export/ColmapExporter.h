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

#include <string>
#include <map>
#include <memory>

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
    bool exportProject(const insight::database::Project& project,
                      const std::string& outputDir,
                      const std::map<std::string, std::string>& options = {});

    /**
     * 获取最后一次操作的错误信息
     */
    std::string getLastError() const { return m_lastError; }

private:
    /**
     * 创建输出目录结构
     */
    bool createDirectoryStructure(const std::string& outputDir);

    /**
     * 创建COLMAP数据库（database.db）
     */
    bool createCOLMAPDatabase(const insight::database::Project& project,
                             const std::string& dbPath);

    /**
     * 将图像文件链接到输出目录
     */
    bool linkImageFiles(const insight::database::Project& project,
                       const std::string& outputDir,
                       const std::map<std::string, std::string>& options);

    /**
     * 创建稀疏模型文件（images.txt, cameras.txt, points3D.txt）
     */
    bool createSparseFiles(const insight::database::Project& project,
                          const std::string& sparseDir);

    /**
     * 写入 images.txt
     */
    bool writeImagesText(const insight::database::Project& project,
                        const std::string& filepath);

    /**
     * 写入 cameras.txt
     */
    bool writeCamerasText(const insight::database::Project& project,
                         const std::string& filepath);

    /**
     * 写入 points3D.txt
     */
    bool writePoints3DText(const insight::database::Project& project,
                          const std::string& filepath);

private:
    std::string m_lastError;
};

}  // namespace algorithm
}  // namespace insight

#endif  // ALGORITHM_COLMAPEXPORTER_H
