/**
 * @file ProjectDocument.cpp
 * @brief ProjectDocument 实现
 */

#include "ProjectDocument.h"
#include <QUuid>
#include <QFileInfo>
#include <glog/logging.h>
#include <fstream>
#include <algorithm>
#include <set>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/string.hpp>

namespace insight {
namespace ui {

ProjectDocument::ProjectDocument(QObject* parent)
    : QObject(parent), m_projectLoaded(false) {
}

ProjectDocument::~ProjectDocument() {
}

// ─────────────────────────────────────────────────────────────
// 文件操作
// ─────────────────────────────────────────────────────────────

bool ProjectDocument::newProject(const QString& name, const QString& author,
                                 const QString& description) {
    closeProject();
    
    // 清除文件路径，新项目未保存到磁盘
    m_filepath.clear();
    
    // 初始化新项目
    m_project.name = name.toStdString();
    m_project.author = author.toStdString();
    m_project.description = description.toStdString();
    m_project.uuid = QUuid::createUuid().toString().toStdString();
    
    auto now = std::time(nullptr);
    m_project.creation_time = now;
    m_project.last_modified_time = now;
    
    // 初始化默认坐标系
    m_project.input_coordinate_system.type = insight::database::CoordinateSystem::Type::kLocal;
    m_project.input_coordinate_system.definition = "Local";
    
    m_projectLoaded = true;
    setModified(true);
    
    LOG(INFO) << "New project created: " << name.toStdString();
    emit projectCreated();
    return true;
}

bool ProjectDocument::openProject(const QString& filepath) {
    if (!QFileInfo(filepath).exists()) {
        LOG(ERROR) << "Project file not found: " << filepath.toStdString();
        return false;
    }
    
    if (!loadFromFile(filepath)) {
        return false;
    }
    
    m_filepath = filepath;
    m_projectLoaded = true;
    setModified(false);
    
    LOG(INFO) << "Project opened: " << filepath.toStdString();
    emit projectOpened();
    return true;
}

bool ProjectDocument::saveProject() {
    if (!m_projectLoaded) {
        LOG(ERROR) << "No project loaded";
        return false;
    }
    
    if (m_filepath.isEmpty()) {
        LOG(ERROR) << "Project path not set";
        return false;
    }
    
    return saveProjectAs(m_filepath);
}

bool ProjectDocument::saveProjectAs(const QString& filepath) {
    if (!m_projectLoaded) {
        LOG(ERROR) << "No project loaded";
        return false;
    }
    
    m_project.last_modified_time = std::time(nullptr);
    
    if (!saveToFile(filepath)) {
        return false;
    }
    
    m_filepath = filepath;
    setModified(false);
    
    LOG(INFO) << "Project saved: " << filepath.toStdString();
    emit projectSaved();
    return true;
}

void ProjectDocument::closeProject() {
    clearAllData();
    m_filepath.clear();
    m_projectLoaded = false;
    setModified(false);
    
    LOG(INFO) << "Project closed";
    emit projectCleared();
}

// ─────────────────────────────────────────────────────────────
// 项目信息编辑
// ─────────────────────────────────────────────────────────────

void ProjectDocument::updateProjectInfo(const QString& name,
                                       const QString& author,
                                       const QString& description) {
    m_project.name = name.toStdString();
    m_project.author = author.toStdString();
    m_project.description = description.toStdString();
    
    setModified(true);
    emit projectInfoChanged();
}

void ProjectDocument::updateCoordinateSystem(const insight::database::CoordinateSystem& cs) {
    m_project.input_coordinate_system = cs;
    setModified(true);
    emit projectInfoChanged();
}

// ─────────────────────────────────────────────────────────────
// ImageGroup 操作
// ─────────────────────────────────────────────────────────────

uint32_t ProjectDocument::createImageGroup(const QString& name,
                                          insight::database::ImageGroup::CameraMode mode) {
    if (!m_projectLoaded) {
        LOG(ERROR) << "No project loaded";
        return (uint32_t)-1;
    }
    
    uint32_t group_id = generateImageGroupId();
    
    insight::database::ImageGroup group;
    group.group_id = group_id;
    group.group_name = name.toStdString();
    group.camera_mode = mode;
    group.creation_time = std::time(nullptr);
    
    // 为 GroupLevel 模式创建默认相机参数
    if (mode == insight::database::ImageGroup::CameraMode::kGroupLevel) {
        insight::database::CameraModel defaultCamera;
        // 设置一些合理的默认值
        defaultCamera.width = 3840;           // 默认4K分辨率宽
        defaultCamera.height = 2160;          // 默认4K分辨率高
        defaultCamera.focal_length = 3600.0;  // 默认焦距约 36mm（35mm等效）
        defaultCamera.principal_point_x = 1920.0;  // 图像中心
        defaultCamera.principal_point_y = 1080.0;
        defaultCamera.sensor_width_mm = 36.0;   // 默认全画幅传感器
        defaultCamera.sensor_height_mm = 20.25;
        defaultCamera.focal_length_35mm = 36.0;
        group.group_camera = defaultCamera;
    }
    
    m_project.image_groups.push_back(group);
    
    setModified(true);
    emit imageGroupAdded(group_id);
    
    LOG(INFO) << "Image group created: " << name.toStdString() << " (ID: " << group_id << ")";
    return group_id;
}

bool ProjectDocument::deleteImageGroup(uint32_t group_id) {
    auto it = std::find_if(m_project.image_groups.begin(),
                          m_project.image_groups.end(),
                          [group_id](const auto& g) { return g.group_id == group_id; });
    
    if (it == m_project.image_groups.end()) {
        LOG(ERROR) << "Image group not found: " << group_id;
        return false;
    }
    
    m_project.image_groups.erase(it);
    
    setModified(true);
    emit imageGroupRemoved(group_id);
    
    LOG(INFO) << "Image group deleted: " << group_id;
    return true;
}

bool ProjectDocument::addImagesToGroup(uint32_t group_id, const QStringList& filenames) {
    auto it = std::find_if(m_project.image_groups.begin(),
                          m_project.image_groups.end(),
                          [group_id](const auto& g) { return g.group_id == group_id; });
    
    if (it == m_project.image_groups.end()) {
        LOG(ERROR) << "Image group not found: " << group_id;
        return false;
    }
    
    for (const auto& filename : filenames) {
        uint32_t image_id = static_cast<uint32_t>(it->images.size() + 1);
        
        insight::database::Image image;
        image.image_id = image_id;
        image.filename = filename.toStdString();
        
        it->images.push_back(image);
    }
    
    setModified(true);
    emit imagesAdded(group_id, filenames);
    emit imageGroupChanged(group_id);
    
    LOG(INFO) << "Added " << filenames.size() << " images to group " << group_id;
    return true;
}

bool ProjectDocument::removeImageFromGroup(uint32_t group_id, uint32_t image_id) {
    auto it = std::find_if(m_project.image_groups.begin(),
                          m_project.image_groups.end(),
                          [group_id](const auto& g) { return g.group_id == group_id; });
    
    if (it == m_project.image_groups.end()) {
        return false;
    }
    
    auto img_it = std::find_if(it->images.begin(),
                              it->images.end(),
                              [image_id](const auto& img) { return img.image_id == image_id; });
    
    if (img_it == it->images.end()) {
        return false;
    }
    
    it->images.erase(img_it);
    
    setModified(true);
    emit imageGroupChanged(group_id);
    
    return true;
}

void ProjectDocument::updateGroupCamera(uint32_t group_id,
                                       const insight::database::CameraModel& camera) {
    auto it = std::find_if(m_project.image_groups.begin(),
                          m_project.image_groups.end(),
                          [group_id](const auto& g) { return g.group_id == group_id; });
    
    if (it != m_project.image_groups.end()) {
        it->group_camera = camera;
        setModified(true);
        emit cameraModelChanged(group_id, 0);
        emit imageGroupChanged(group_id);
    }
}

// ─────────────────────────────────────────────────────────────
// CameraModel 操作
// ─────────────────────────────────────────────────────────────

void ProjectDocument::updateImageCamera(uint32_t group_id, uint32_t image_id,
                                       const insight::database::CameraModel& camera) {
    auto it = std::find_if(m_project.image_groups.begin(),
                          m_project.image_groups.end(),
                          [group_id](const auto& g) { return g.group_id == group_id; });
    
    if (it == m_project.image_groups.end()) {
        return;
    }
    
    auto img_it = std::find_if(it->images.begin(),
                              it->images.end(),
                              [image_id](const auto& img) { return img.image_id == image_id; });
    
    if (img_it != it->images.end()) {
        img_it->camera = camera;
        setModified(true);
        emit cameraModelChanged(group_id, image_id);
        emit imageGroupChanged(group_id);
    }
}

// ─────────────────────────────────────────────────────────────
// CameraRig 操作
// ─────────────────────────────────────────────────────────────

uint32_t ProjectDocument::createCameraRig(const QString& name,
                                         const QString& description) {
    if (!m_projectLoaded) {
        LOG(ERROR) << "No project loaded";
        return (uint32_t)-1;
    }
    
    uint32_t rig_id = generateRigId();
    
    insight::database::CameraRig rig;
    rig.rig_id = rig_id;
    rig.rig_name = name.toStdString();
    rig.description = description.toStdString();
    rig.calib_status = insight::database::CameraRig::CalibrationStatus::kUnknown;
    
    m_project.camera_rigs[rig_id] = rig;
    
    setModified(true);
    emit cameraRigAdded(rig_id);
    
    LOG(INFO) << "Camera rig created: " << name.toStdString() << " (ID: " << rig_id << ")";
    return rig_id;
}

bool ProjectDocument::deleteCameraRig(uint32_t rig_id) {
    auto it = m_project.camera_rigs.find(rig_id);
    
    if (it == m_project.camera_rigs.end()) {
        LOG(ERROR) << "Camera rig not found: " << rig_id;
        return false;
    }
    
    m_project.camera_rigs.erase(it);
    
    setModified(true);
    emit cameraRigRemoved(rig_id);
    
    LOG(INFO) << "Camera rig deleted: " << rig_id;
    return true;
}

bool ProjectDocument::addCameraToRig(uint32_t rig_id,
                                    const insight::database::CameraRig::CameraMount& mount,
                                    const insight::database::CameraModel& camera) {
    auto it = m_project.camera_rigs.find(rig_id);
    
    if (it == m_project.camera_rigs.end()) {
        LOG(ERROR) << "Camera rig not found: " << rig_id;
        return false;
    }
    
    it->second.mounts.push_back(mount);
    
    // 将相机参数与mount的camera_id关联（可在后续扩展）
    
    setModified(true);
    emit cameraRigChanged(rig_id);
    
    LOG(INFO) << "Camera added to rig " << rig_id << " (Camera ID: " << mount.camera_id << ")";
    return true;
}

bool ProjectDocument::removeCameraFromRig(uint32_t rig_id, uint32_t camera_id) {
    auto it = m_project.camera_rigs.find(rig_id);
    
    if (it == m_project.camera_rigs.end()) {
        return false;
    }
    
    auto mount_it = std::find_if(it->second.mounts.begin(),
                                it->second.mounts.end(),
                                [camera_id](const auto& m) { return m.camera_id == camera_id; });
    
    if (mount_it == it->second.mounts.end()) {
        return false;
    }
    
    it->second.mounts.erase(mount_it);
    
    setModified(true);
    emit cameraRigChanged(rig_id);
    
    return true;
}

void ProjectDocument::updateRigCameraModel(uint32_t rig_id, uint32_t camera_id,
                                          const insight::database::CameraModel& camera) {
    auto it = m_project.camera_rigs.find(rig_id);
    
    if (it != m_project.camera_rigs.end()) {
        setModified(true);
        emit cameraRigChanged(rig_id);
    }
}

void ProjectDocument::updateRigCalibrationStatus(
    uint32_t rig_id,
    insight::database::CameraRig::CalibrationStatus status) {
    auto it = m_project.camera_rigs.find(rig_id);
    
    if (it != m_project.camera_rigs.end()) {
        it->second.calib_status = status;
        setModified(true);
        emit cameraRigChanged(rig_id);
    }
}

// ─────────────────────────────────────────────────────────────
// GCP 操作
// ─────────────────────────────────────────────────────────────

size_t ProjectDocument::importGCPs(const QString& filepath, const QMap<QString, QString>& options) {
    // TODO: 实现从CSV/TXT文件解析GCP数据
    LOG(WARNING) << "GCP import not yet implemented";
    return 0;
}

uint32_t ProjectDocument::addGCP(const insight::database::GCPMeasurement& gcp) {
    uint32_t gcp_id = generateGCPId();
    
    insight::database::GCPMeasurement new_gcp = gcp;
    new_gcp.gcp_id = gcp_id;
    
    m_project.gcp_database[gcp_id] = new_gcp;
    m_project.InvalidateGCPCache();
    
    setModified(true);
    emit gcpAdded(gcp_id);
    
    return gcp_id;
}

bool ProjectDocument::deleteGCP(uint32_t gcp_id) {
    auto it = m_project.gcp_database.find(gcp_id);
    
    if (it == m_project.gcp_database.end()) {
        return false;
    }
    
    m_project.gcp_database.erase(it);
    m_project.InvalidateGCPCache();
    
    setModified(true);
    emit gcpRemoved(gcp_id);
    
    return true;
}

void ProjectDocument::updateGCP(uint32_t gcp_id, const insight::database::GCPMeasurement& gcp) {
    auto it = m_project.gcp_database.find(gcp_id);
    
    if (it != m_project.gcp_database.end()) {
        it->second = gcp;
        it->second.gcp_id = gcp_id;
        m_project.InvalidateGCPCache();
        
        setModified(true);
        emit gcpChanged(gcp_id);
    }
}

void ProjectDocument::clearAllGCPs() {
    m_project.gcp_database.clear();
    m_project.InvalidateGCPCache();
    
    setModified(true);
}

// ─────────────────────────────────────────────────────────────
// ATTask 操作
// ─────────────────────────────────────────────────────────────

std::string ProjectDocument::createATTask(const QString& name) {
    if (!m_projectLoaded) {
        LOG(ERROR) << "No project loaded";
        return "";
    }
    
    // 生成新的 UUID 作为任务唯一标识
    std::string task_id_str = QUuid::createUuid().toString().toStdString();
    uint32_t task_id = m_project.next_at_task_id++;
    
    // 确定任务名称
    std::string task_name = name.toStdString();
    if (task_name.empty()) {
        // 如果未指定名称，自动生成 AT_0, AT_1, ...
        task_name = "AT_" + std::to_string(task_id);
    }
    
    // 创建新的 AT Task
    insight::database::ATTask task;
    task.id = task_id_str;
    task.task_id = task_id;
    task.task_name = task_name;
    
    // 复制当前项目的快照到 InputSnapshot
    task.input_snapshot.image_groups = m_project.image_groups;
    task.input_snapshot.measurements = m_project.measurements;
    task.input_snapshot.input_coordinate_system = m_project.input_coordinate_system;
    
    // 设置默认的优化配置
    // 对所有camera_rigs中的相机设置优化标记
    for (const auto& rig_pair : m_project.camera_rigs) {
        const auto& rig = rig_pair.second;
        for (const auto& mount : rig.mounts) {
            insight::database::OptimizationFlags flags;
            flags.focal_length = true;
            flags.principal_point_x = true;
            flags.principal_point_y = true;
            flags.k1 = true;
            flags.k2 = true;
            flags.p1 = true;
            flags.p2 = true;
            
            task.optimization_config.camera_optimization[mount.camera_id] = flags;
        }
    }
    task.optimization_config.enable_gnss_constraint = true;
    task.optimization_config.gnss_weight = 1.0;
    task.optimization_config.max_reprojection_error = 10.0;
    
    m_project.at_tasks.push_back(task);
    
    setModified(true);
    emit atTaskCreated(QString::fromStdString(task_id_str));
    
    LOG(INFO) << "AT task created: " << task_name << " (ID: " << task_id_str << ", Number: " << task_id << ")";
    return task_id_str;
}

bool ProjectDocument::deleteATTask(const std::string& task_id) {
    auto it = std::find_if(m_project.at_tasks.begin(),
                          m_project.at_tasks.end(),
                          [&task_id](const auto& t) { return t.id == task_id; });
    
    if (it == m_project.at_tasks.end()) {
        LOG(ERROR) << "AT task not found: " << task_id;
        return false;
    }
    
    m_project.at_tasks.erase(it);
    
    setModified(true);
    emit atTaskRemoved(QString::fromStdString(task_id));
    
    LOG(INFO) << "AT task deleted: " << task_id;
    return true;
}

void ProjectDocument::updateATTask(const std::string& task_id,
                                  const insight::database::ATTask& task) {
    auto it = std::find_if(m_project.at_tasks.begin(),
                          m_project.at_tasks.end(),
                          [&task_id](const auto& t) { return t.id == task_id; });
    
    if (it != m_project.at_tasks.end()) {
        // 保留原有的 id，其他字段更新
        std::string originalId = it->id;
        *it = task;
        it->id = originalId;
        
        setModified(true);
        emit atTaskChanged(QString::fromStdString(task_id));
        
        LOG(INFO) << "AT task updated: " << task_id;
    } else {
        LOG(ERROR) << "AT task not found for update: " << task_id;
    }
}

insight::database::ATTask* ProjectDocument::getATTaskById(const std::string& task_id) {
    auto it = std::find_if(m_project.at_tasks.begin(),
                          m_project.at_tasks.end(),
                          [&task_id](const auto& t) { return t.id == task_id; });
    
    if (it != m_project.at_tasks.end()) {
        return &(*it);
    }
    return nullptr;
}

const insight::database::ATTask* ProjectDocument::getATTaskById(const std::string& task_id) const {
    auto it = std::find_if(m_project.at_tasks.begin(),
                          m_project.at_tasks.end(),
                          [&task_id](const auto& t) { return t.id == task_id; });
    
    if (it != m_project.at_tasks.end()) {
        return &(*it);
    }
    return nullptr;
}

// ─────────────────────────────────────────────────────────────
// 导出/导入
// ─────────────────────────────────────────────────────────────

bool ProjectDocument::exportToCOLMAP(const QString& outputDir, const QMap<QString, QString>& options) {
    // TODO: 实现COLMAP导出
    LOG(INFO) << "Exporting to COLMAP: " << outputDir.toStdString();
    emit exportStarted("COLMAP");
    emit exportFinished(false, "Not yet implemented");
    return false;
}

bool ProjectDocument::importFromCOLMAP(const QString& colmapDb, const QMap<QString, QString>& options) {
    // TODO: 实现从COLMAP导入
    LOG(INFO) << "Importing from COLMAP: " << colmapDb.toStdString();
    emit importStarted("COLMAP");
    emit importFinished(false, "Not yet implemented");
    return false;
}

// ─────────────────────────────────────────────────────────────
// 内部辅助方法
// ─────────────────────────────────────────────────────────────

void ProjectDocument::setModified(bool modified) {
    if (m_modified != modified) {
        m_modified = modified;
        emit modificationChanged(modified);
    }
}

void ProjectDocument::clearAllData() {
    m_project = insight::database::Project();
}

bool ProjectDocument::loadFromFile(const QString& filepath) {
    try {
        std::ifstream ifs(filepath.toStdString(), std::ios::binary);
        if (!ifs.is_open()) {
            LOG(ERROR) << "Failed to open file: " << filepath.toStdString();
            return false;
        }
        
        // cereal::BinaryInputArchive archive(ifs);
        cereal::JSONInputArchive archive(ifs);
        archive(cereal::make_nvp("project", m_project));
        ifs.close();
        
        m_projectLoaded = true;
        m_filepath = filepath;
        
        // 同步 ID 计数器，防止 ID 冲突
        syncCounters();
        
        LOG(INFO) << "Project loaded from file: " << filepath.toStdString();
        emit projectOpened();
        return true;
    } catch (const std::exception& e) {
        LOG(ERROR) << "Failed to load project: " << e.what();
        return false;
    }
}

bool ProjectDocument::saveToFile(const QString& filepath) {
    try {
        std::ofstream ofs(filepath.toStdString(), std::ios::binary);
        if (!ofs.is_open()) {
            LOG(ERROR) << "Failed to create file: " << filepath.toStdString();
            return false;
        }
        
        {
            // cereal::BinaryOutputArchive archive(ofs);
            cereal::JSONOutputArchive archive(ofs);
            archive(cereal::make_nvp("project", m_project));
            // 显式销毁 archive 以确保正确写入
        }
        
        ofs.close();
        m_filepath = filepath;
        
        LOG(INFO) << "Project saved to file: " << filepath.toStdString();
        emit projectSaved();
        return true;
    } catch (const std::exception& e) {
        LOG(ERROR) << "Failed to save project: " << e.what();
        return false;
    }
}

uint32_t ProjectDocument::generateImageId() {
    return m_project.next_image_id++;
}
void ProjectDocument::syncCounters() {
    uint32_t maxImageId = 0;
    uint32_t maxGroupId = 0;
    uint32_t maxRigId = 0;
    uint32_t maxGCPId = 0;
    uint32_t maxTaskIndex = 0;

    // 1. First Pass: Find Current Max IDs (ignoring invalid ones)
    for (const auto& group : m_project.image_groups) {
        if (group.group_id != static_cast<uint32_t>(-1)) {
            maxGroupId = std::max(maxGroupId, group.group_id);
        }
        for (const auto& image : group.images) {
            if (image.image_id != static_cast<uint32_t>(-1)) {
                maxImageId = std::max(maxImageId, image.image_id);
            }
        }
    }

    for (const auto& meas : m_project.measurements) {
        if (meas.image_id != static_cast<uint32_t>(-1)) {
            maxImageId = std::max(maxImageId, meas.image_id);
        }
    }

    for (const auto& pair : m_project.camera_rigs) {
        if (pair.first != static_cast<uint32_t>(-1)) {
            maxRigId = std::max(maxRigId, pair.first);
        }
    }

    for (const auto& pair : m_project.gcp_database) {
        if (pair.first != static_cast<uint32_t>(-1)) {
            maxGCPId = std::max(maxGCPId, pair.first);
        }
    }

    // 扫描现有任务中的最大 ID
    for (const auto& task : m_project.at_tasks) {
        if (task.task_id != static_cast<uint32_t>(-1)) {
            maxTaskIndex = std::max(maxTaskIndex, task.task_id);
        }
    }

    // Set counters to next available (only if scanned values are larger)
    m_project.next_image_id = std::max(m_project.next_image_id, maxImageId + 1);
    m_project.next_image_group_id = std::max(m_project.next_image_group_id, maxGroupId + 1);
    m_project.next_rig_id = std::max(m_project.next_rig_id, maxRigId + 1);
    m_project.next_gcp_id = std::max(m_project.next_gcp_id, maxGCPId + 1);
    m_project.next_at_task_id = std::max(m_project.next_at_task_id, maxTaskIndex + 1);

    // 2. Second Pass: Repair invalid or duplicate IDs to ensure uniqueness
    std::set<uint32_t> seenGroupIds;
    bool anyRepaired = false;
    for (auto& group : m_project.image_groups) {
        if (group.group_id == 0 || group.group_id == static_cast<uint32_t>(-1) || seenGroupIds.count(group.group_id)) {
            uint32_t oldId = group.group_id;
            group.group_id = m_project.next_image_group_id++;
            LOG(WARNING) << "Repaired ImageGroup ID conflict: " << (int)oldId << " -> " << group.group_id;
            anyRepaired = true;
        }
        seenGroupIds.insert(group.group_id);
    }

    // 修复 AT Task 的 ID (如果为 0 且列表非空，可能是旧版本导入)
    std::set<uint32_t> seenTaskIds;
    for (auto& task : m_project.at_tasks) {
        if (task.task_id == 0 || seenTaskIds.count(task.task_id)) {
            uint32_t oldId = task.task_id;
            task.task_id = m_project.next_at_task_id++;
            LOG(WARNING) << "Assigned/Repaired AT Task ID: " << oldId << " -> " << task.task_id;
            anyRepaired = true;
        }
        seenTaskIds.insert(task.task_id);
    }

    if (anyRepaired) {
        setModified(true);
    }

    LOG(INFO) << "Counters synchronized and repaired. Next IDs: Image=" << m_project.next_image_id 
              << ", Group=" << m_project.next_image_group_id 
              << ", Rig=" << m_project.next_rig_id 
              << ", GCP=" << m_project.next_gcp_id
              << ", AT_ID=" << m_project.next_at_task_id;
}
void ProjectDocument::applyGNSSToImages(const std::vector<database::Measurement::GNSSMeasurement>& gnssDataList,
                                        uint32_t groupId)
{
    // Find the image group by ID
    for (auto& group : m_project.image_groups) {
        if (group.group_id == groupId) {
            // Apply GNSS data to images in sequence
            size_t gnssIndex = 0;
            for (auto& image : group.images) {
                if (gnssIndex < gnssDataList.size()) {
                    image.gnss_data = gnssDataList[gnssIndex];
                    ++gnssIndex;
                } else {
                    break;  // No more GNSS data to apply
                }
            }
            
            // Mark as modified and save
            setModified(true);
            saveToFile(m_filepath);
            break;
        }
    }
}

uint32_t ProjectDocument::generateImageGroupId() {
    return m_project.next_image_group_id++;
}

uint32_t ProjectDocument::generateRigId() {
    return m_project.next_rig_id++;
}

uint32_t ProjectDocument::generateGCPId() {
    return m_project.next_gcp_id++;
}

void ProjectDocument::notifyImageGroupChanged(uint32_t group_id) {
    setModified(true);
    emit imageGroupChanged(group_id);
}

std::string ProjectDocument::generateNextATTaskName() {
    // 仅仅提供一个基于当前计数器的建议名称，不增加计数器
    // 计数器的增加发生在 createATTask 中
    return "AT_" + std::to_string(m_project.next_at_task_id);
}

}  // namespace ui
}  // namespace insight
