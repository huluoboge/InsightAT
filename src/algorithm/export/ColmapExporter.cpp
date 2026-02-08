/**
 * @file ColmapExporter.cpp
 * @brief COLMAP导出器实现（骨架版本）
 * 
 * 完整的COLMAP导出会很复杂，这里先提供基础框架，
 * 后续可以逐步完善实现细节。
 */

#include "ColmapExporter.h"
#include "database/database_types.h"
#include <QDir>
#include <QFile>
#include <QTextStream>
#include <glog/logging.h>
#include <sstream>
#include <iomanip>

namespace insight {
namespace algorithm {

bool ColmapExporter::exportProject(const insight::database::Project& project,
                                   const QString& outputDir,
                                   const QMap<QString, QString>& options) {
    LOG(INFO) << "Starting COLMAP export to: " << outputDir.toStdString();
    
    try {
        // 1. 创建目录结构
        if (!createDirectoryStructure(outputDir)) {
            m_lastError = "Failed to create directory structure";
            return false;
        }
        
        // 2. 创建COLMAP数据库
        QString dbPath = outputDir + "/database.db";
        if (!createCOLMAPDatabase(project, dbPath)) {
            m_lastError = "Failed to create COLMAP database";
            return false;
        }
        
        // 3. 链接或复制图像文件
        if (!linkImageFiles(project, outputDir, options)) {
            m_lastError = "Failed to link image files";
            return false;
        }
        
        // 4. 创建稀疏模型文件
        QString sparseDir = outputDir + "/sparse/0";
        if (!createSparseFiles(project, sparseDir)) {
            m_lastError = "Failed to create sparse files";
            return false;
        }
        
        LOG(INFO) << "COLMAP export completed successfully";
        return true;
        
    } catch (const std::exception& e) {
        m_lastError = QString("Export failed: %1").arg(e.what());
        LOG(ERROR) << m_lastError.toStdString();
        return false;
    }
}

bool ColmapExporter::createDirectoryStructure(const QString& outputDir) {
    QDir dir;
    
    // 创建根目录
    if (!dir.exists(outputDir)) {
        if (!dir.mkpath(outputDir)) {
            LOG(ERROR) << "Failed to create output directory: " << outputDir.toStdString();
            return false;
        }
    }
    
    // 创建子目录
    QStringList subdirs = {
        outputDir + "/images",
        outputDir + "/sparse",
        outputDir + "/sparse/0"
    };
    
    for (const auto& subdir : subdirs) {
        if (!dir.exists(subdir)) {
            if (!dir.mkpath(subdir)) {
                LOG(ERROR) << "Failed to create subdirectory: " << subdir.toStdString();
                return false;
            }
        }
    }
    
    LOG(INFO) << "Directory structure created";
    return true;
}

bool ColmapExporter::createCOLMAPDatabase(const insight::database::Project& project,
                                          const QString& dbPath) {
    // TODO: 实现COLMAP数据库创建
    // 这需要：
    // 1. SQLite库支持
    // 2. 创建必要的表（images, cameras, keypoints, matches等）
    // 3. 填充相机和图像信息
    
    LOG(WARNING) << "COLMAP database creation not yet fully implemented";
    LOG(INFO) << "Database path would be: " << dbPath.toStdString();
    
    // 暂时创建空文件占位
    QFile file(dbPath);
    if (!file.open(QIODevice::WriteOnly)) {
        LOG(ERROR) << "Failed to create database file";
        return false;
    }
    file.close();
    
    return true;
}

bool ColmapExporter::linkImageFiles(const insight::database::Project& project,
                                    const QString& outputDir,
                                    const QMap<QString, QString>& options) {
    QString imagesDir = outputDir + "/images";
    
    // 获取选项
    bool copyImages = options.value("copy_images", "false") == "true";
    bool linkImages = options.value("link_images", "true") == "true";
    
    LOG(INFO) << "Processing image files (copy=" << copyImages << ", link=" << linkImages << ")";
    
    int imageCount = 0;
    for (const auto& group : project.image_groups) {
        for (const auto& image : group.images) {
            QString srcPath = QString::fromStdString(image.filename);
            QString filename = QFileInfo(srcPath).fileName();
            QString dstPath = imagesDir + "/" + filename;
            
            if (copyImages) {
                // 复制文件
                if (!QFile::copy(srcPath, dstPath)) {
                    LOG(WARNING) << "Failed to copy image: " << srcPath.toStdString();
                    continue;
                }
            } else if (linkImages) {
                // 创建符号链接
                // TODO: 实现跨平台的符号链接创建
                LOG(WARNING) << "Symbolic link not yet implemented, would link: " 
                            << srcPath.toStdString();
            }
            
            imageCount++;
        }
    }
    
    LOG(INFO) << "Processed " << imageCount << " images";
    return true;
}

bool ColmapExporter::createSparseFiles(const insight::database::Project& project,
                                       const QString& sparseDir) {
    // 创建images.txt
    QString imagesPath = sparseDir + "/images.txt";
    if (!writeImagesText(project, imagesPath)) {
        return false;
    }
    
    // 创建cameras.txt
    QString camerasPath = sparseDir + "/cameras.txt";
    if (!writeCamerasText(project, camerasPath)) {
        return false;
    }
    
    // 创建points3D.txt
    QString points3DPath = sparseDir + "/points3D.txt";
    if (!writePoints3DText(project, points3DPath)) {
        return false;
    }
    
    return true;
}

bool ColmapExporter::writeImagesText(const insight::database::Project& project,
                                     const QString& filepath) {
    QFile file(filepath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        LOG(ERROR) << "Failed to open images.txt for writing: " << filepath.toStdString();
        return false;
    }
    
    QTextStream stream(&file);
    
    // COLMAP images.txt 格式：
    // # image_list.txt
    // IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
    // ...
    
    uint32_t imageId = 1;
    for (const auto& group : project.image_groups) {
        const auto* camera = group.group_camera ? &(*group.group_camera) : nullptr;
        uint32_t cameraId = (camera ? 1 : 0);
        
        for (const auto& image : group.images) {
            // 默认旋转（四元数）：(1, 0, 0, 0)
            // 默认位置：(0, 0, 0)
            stream << imageId << " "
                  << "1 0 0 0 "  // quaternion (qw qx qy qz)
                  << "0 0 0 "    // translation (tx ty tz)
                  << cameraId << " "
                  << QString::fromStdString(image.filename)
                  << "\n";
            
            imageId++;
        }
    }
    
    // 空行（分隔符）
    stream << "\n";
    
    // 写入keypoints (COLMAP格式要求，这里先留空)
    // 实际格式：每个图像一行，包含keypoint数量和坐标
    
    file.close();
    
    LOG(INFO) << "Created images.txt with " << (imageId - 1) << " images";
    return true;
}

bool ColmapExporter::writeCamerasText(const insight::database::Project& project,
                                      const QString& filepath) {
    QFile file(filepath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        LOG(ERROR) << "Failed to open cameras.txt for writing: " << filepath.toStdString();
        return false;
    }
    
    QTextStream stream(&file);
    
    // COLMAP cameras.txt 格式：
    // # Camera list with one line of data per camera:
    // #   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[0], PARAMS[1], ...
    // # Available camera models:
    // #   SIMPLE_PINHOLE: f, cx, cy
    // #   PINHOLE: fx, fy, cx, cy
    // #   SIMPLE_RADIAL: f, cx, cy, k
    // #   RADIAL: f, cx, cy, k1, k2
    // #   OPENCV: fx, fy, cx, cy, k1, k2, p1, p2
    // #   OPENCV_FISHEYE: fx, fy, cx, cy, k1, k2, k3, k4
    
    uint32_t cameraId = 1;
    for (const auto& group : project.image_groups) {
        if (group.group_camera) {
            const auto& camera = *group.group_camera;
            
            // 使用 OPENCV 模型（对应 Brown-Conrady）
            stream << cameraId << " "
                  << "OPENCV "
                  << camera.width << " "
                  << camera.height << " ";
            
            // 参数: fx, fy, cx, cy, k1, k2, p1, p2
            stream << QString::number(camera.focal_length, 'f', 6) << " "
                  << QString::number(camera.focal_length * camera.aspect_ratio, 'f', 6) << " "
                  << QString::number(camera.principal_point_x, 'f', 6) << " "
                  << QString::number(camera.principal_point_y, 'f', 6) << " "
                  << QString::number(camera.k1, 'f', 6) << " "
                  << QString::number(camera.k2, 'f', 6) << " "
                  << QString::number(camera.p1, 'f', 6) << " "
                  << QString::number(camera.p2, 'f', 6) << "\n";
            
            cameraId++;
        }
    }
    
    file.close();
    
    LOG(INFO) << "Created cameras.txt with " << (cameraId - 1) << " cameras";
    return true;
}

bool ColmapExporter::writePoints3DText(const insight::database::Project& project,
                                       const QString& filepath) {
    QFile file(filepath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        LOG(ERROR) << "Failed to open points3D.txt for writing: " << filepath.toStdString();
        return false;
    }
    
    QTextStream stream(&file);
    
    // COLMAP points3D.txt 格式：
    // #   POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] (IMAGE_ID, POINT2D_IDX)
    
    // 从GCP导入点云
    uint32_t point3DId = 1;
    for (const auto& [gcp_id, gcp] : project.gcp_database) {
        stream << point3DId << " "
              << QString::number(gcp.x, 'f', 6) << " "
              << QString::number(gcp.y, 'f', 6) << " "
              << QString::number(gcp.z, 'f', 6) << " ";
        
        // 颜色 (R, G, B) - 暂时默认白色
        stream << "255 255 255 "
              << "0 ";  // ERROR
        
        // TRACK - GCP的观测
        for (const auto& obs : gcp.observations) {
            stream << obs.image_id << " 0 ";
        }
        
        stream << "\n";
        point3DId++;
    }
    
    file.close();
    
    LOG(INFO) << "Created points3D.txt with " << (point3DId - 1) << " points";
    return true;
}

}  // namespace algorithm
}  // namespace insight
