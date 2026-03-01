/**
 * @file ProjectDocument.h
 * @brief 项目文档管理器 - 单文档模式下的项目容器
 * 
 * ProjectDocument 是UI与database层之间的适配器，负责：
 * 1. 持有和管理 database::Project 实例
 * 2. 处理文件I/O（通过Cereal序列化）
 * 3. 发出数据变化信号，驱动UI更新
 * 4. 提供一致的API供UI层修改数据
 * 
 * 设计原则：
 * - 所有数据修改都通过 ProjectDocument 的 slots
 * - 修改后自动发出相应的信号
 * - UI 层监听信号来更新显示
 * - 数据始终与 database::Project 同步
 */

#ifndef UI_PROJECTDOCUMENT_H
#define UI_PROJECTDOCUMENT_H

#include <QObject>
#include <QString>
#include <QStringList>
#include <QMap>
#include <memory>

#include "database/database_types.h"

namespace insight {
namespace ui {

/**
 * @class ProjectDocument
 * @brief 项目文档 - 单文档模式的项目管理器
 */
class ProjectDocument : public QObject {
    Q_OBJECT

public:
    explicit ProjectDocument(QObject* parent = nullptr);
    ~ProjectDocument();

    // ─────────────────────────────────────────────────────────
    // 访问器 - 获取项目数据
    // ─────────────────────────────────────────────────────────
    
    /**
     * 获取项目的常量引用
     */
    const insight::database::Project& project() const { return m_project; }
    
    /**
     * 获取项目的非常量引用（用于特定的访问）
     */
    insight::database::Project& project() { return m_project; }
    
    /**
     * 检查项目是否有未保存的修改
     */
    bool isModified() const { return m_modified; }
    
    /**
     * 获取当前项目文件路径
     */
    QString filepath() const { return m_filepath; }
    
    /**
     * 检查项目是否已加载
     */
    bool isProjectLoaded() const { return m_projectLoaded; }

    // ─────────────────────────────────────────────────────────
    // 文件操作
    // ─────────────────────────────────────────────────────────

signals:
    void projectSaved();
    void projectOpened();
    void projectCreated();
    void modificationChanged(bool modified);
    void projectCleared();

public slots:
    /**
     * 创建新项目
     * 
     * @param[in] name 项目名称
     * @param[in] author 作者
     * @param[in] description 描述
     * @return 成功返回true
     */
    bool newProject(const QString& name, const QString& author = "", 
                   const QString& description = "");
    
    /**
     * 打开现有项目文件
     * 
     * @param[in] filepath 项目文件路径
     * @return 成功返回true
     */
    bool openProject(const QString& filepath);
    
    /**
     * 保存项目到当前路径
     * 
     * @return 成功返回true
     */
    bool saveProject();
    
    /**
     * 另存为指定路径
     * 
     * @param[in] filepath 新的文件路径
     * @return 成功返回true
     */
    bool saveProjectAs(const QString& filepath);
    
    /**
     * 关闭项目（清空数据）
     */
    void closeProject();

    // ─────────────────────────────────────────────────────────
    // 项目信息编辑
    // ─────────────────────────────────────────────────────────

signals:
    void projectInfoChanged();

public slots:
    /**
     * 更新项目基本信息
     */
    void updateProjectInfo(const QString& name, const QString& author,
                          const QString& description);
    
    /**
     * 更新坐标系
     */
    void updateCoordinateSystem(const insight::database::CoordinateSystem& cs);

    // ─────────────────────────────────────────────────────────
    // ImageGroup 操作
    // ─────────────────────────────────────────────────────────

signals:
    void imageGroupAdded(uint32_t group_id);
    void imageGroupRemoved(uint32_t group_id);
    void imageGroupChanged(uint32_t group_id);
    void imagesAdded(uint32_t group_id, const QStringList& filenames);

public slots:
    /**
     * 创建新的图像分组
     * 
     * @param[in] name 分组名称
     * @param[in] mode 相机模式（GroupLevel/ImageLevel/RigBased）
     * @return 新分组的ID，失败返回 (uint32_t)-1
     */
    uint32_t createImageGroup(const QString& name,
                             insight::database::ImageGroup::CameraMode mode);
    
    /**
     * 删除图像分组
     * 
     * @param[in] group_id 分组ID
     * @return 成功返回true
     */
    bool deleteImageGroup(uint32_t group_id);
    
    /**
     * 向分组添加图像
     * 
     * @param[in] group_id 分组ID
     * @param[in] filenames 图像文件路径列表
     * @return 成功返回true
     */
    bool addImagesToGroup(uint32_t group_id, const QStringList& filenames);
    
    /**
     * 从分组移除图像
     * 
     * @param[in] group_id 分组ID
     * @param[in] image_id 图像ID
     * @return 成功返回true
     */
    bool removeImageFromGroup(uint32_t group_id, uint32_t image_id);
    
    /**
     * 更新分组的相机参数（仅GroupLevel模式）
     */
    void updateGroupCamera(uint32_t group_id,
                          const insight::database::CameraModel& camera);

    // ─────────────────────────────────────────────────────────
    // CameraModel 操作
    // ─────────────────────────────────────────────────────────

signals:
    void cameraModelChanged(uint32_t group_id, uint32_t image_id);

public slots:
    /**
     * 更新图像级相机参数（ImageLevel模式）
     */
    void updateImageCamera(uint32_t group_id, uint32_t image_id,
                          const insight::database::CameraModel& camera);

    // ─────────────────────────────────────────────────────────
    // CameraRig 操作（多相机配置）
    // ─────────────────────────────────────────────────────────

signals:
    void cameraRigAdded(uint32_t rig_id);
    void cameraRigRemoved(uint32_t rig_id);
    void cameraRigChanged(uint32_t rig_id);

public slots:
    /**
     * 创建新的相机配置
     * 
     * @param[in] name Rig名称
     * @param[in] description 描述
     * @return 新Rig的ID，失败返回 (uint32_t)-1
     */
    uint32_t createCameraRig(const QString& name, const QString& description = "");
    
    /**
     * 删除相机配置
     */
    bool deleteCameraRig(uint32_t rig_id);
    
    /**
     * 向Rig添加相机挂载点
     */
    bool addCameraToRig(uint32_t rig_id,
                       const insight::database::CameraRig::CameraMount& mount,
                       const insight::database::CameraModel& camera);
    
    /**
     * 从Rig移除相机
     */
    bool removeCameraFromRig(uint32_t rig_id, uint32_t camera_id);
    
    /**
     * 更新Rig中特定相机的参数
     */
    void updateRigCameraModel(uint32_t rig_id, uint32_t camera_id,
                             const insight::database::CameraModel& camera);
    
    /**
     * 更新Rig的标定状态
     */
    void updateRigCalibrationStatus(uint32_t rig_id,
                                   insight::database::CameraRig::CalibrationStatus status);

    // ─────────────────────────────────────────────────────────
    // GCP（地面控制点）操作
    // ─────────────────────────────────────────────────────────

signals:
    void gcpAdded(uint32_t gcp_id);
    void gcpRemoved(uint32_t gcp_id);
    void gcpChanged(uint32_t gcp_id);
    void gcpsImported(size_t count);

public slots:
    /**
     * 导入GCP数据
     * 
     * @param[in] filepath GCP文件路径（CSV/TXT格式）
     * @param[in] options 导入选项（如坐标系转换等）
     * @return 导入的GCP数量，失败返回0
     */
    size_t importGCPs(const QString& filepath, const QMap<QString, QString>& options = {});
    
    /**
     * 添加单个GCP
     */
    uint32_t addGCP(const insight::database::GCPMeasurement& gcp);
    
    /**
     * 删除GCP
     */
    bool deleteGCP(uint32_t gcp_id);
    
    /**
     * 更新GCP数据
     */
    void updateGCP(uint32_t gcp_id, const insight::database::GCPMeasurement& gcp);
    
    /**
     * 清空所有GCP
     */
    void clearAllGCPs();

    // ─────────────────────────────────────────────────────────
    // ATTask（空三任务）操作
    // ─────────────────────────────────────────────────────────

signals:
    void atTaskCreated(const QString& task_id);
    void atTaskRemoved(const QString& task_id);
    void atTaskChanged(const QString& task_id);

public slots:
    /**
     * 创建新的空三任务
     * 
     * @param[in] name 任务名称
     * @return 任务ID，失败返回空字符串
     */
    std::string createATTask(const QString& name);
    
    /**
     * 删除空三任务
     */
    bool deleteATTask(const std::string& task_id);
    
    /**
     * 更新空三任务信息
     */
    void updateATTask(const std::string& task_id, const insight::database::ATTask& task);
    
    /**
     * 根据任务ID获取 AT Task
     * 
     * @param[in] task_id 任务的 UUID
     * @return 指向 ATTask 的指针，未找到返回 nullptr
     */
    insight::database::ATTask* getATTaskById(const std::string& task_id);
    
    /**
     * 根据任务ID获取 AT Task（const 版本）
     * 
     * @param[in] task_id 任务的 UUID
     * @return 指向 ATTask 的常量指针，未找到返回 nullptr
     */
    const insight::database::ATTask* getATTaskById(const std::string& task_id) const;

    // ─────────────────────────────────────────────────────────
    // 导出/导入（Algorithm Bridge）
    // ─────────────────────────────────────────────────────────

signals:
    void exportStarted(const QString& format);
    void exportFinished(bool success, const QString& message);
    void importStarted(const QString& format);
    void importFinished(bool success, const QString& message);

public slots:
    /**
     * 导出为COLMAP格式
     * 
     * @param[in] outputDir 输出目录
     * @param[in] options 导出选项
     * @return 成功返回true
     */
    bool exportToCOLMAP(const QString& outputDir, const QMap<QString, QString>& options = {});
    
    /**
     * 从COLMAP格式导入结果
     * 
     * @param[in] colmapDb COLMAP database.db 路径
     * @param[in] options 导入选项
     * @return 成功返回true
     */
    bool importFromCOLMAP(const QString& colmapDb, const QMap<QString, QString>& options = {});

    // ─────────────────────────────────────────────────────────
    // 公开的ID生成接口（用于图像编辑器等）
    // ─────────────────────────────────────────────────────────
    
    /**
     * 生成唯一的图像ID
     * @return 新的图像ID
     */
    uint32_t generateImageId();

    /**
     * 应用 GNSS 测量数据到图像
     * 
     * @param[in] gnssDataList GNSS 测量数据列表，按照向导匹配顺序排列
     * @param[in] groupId 目标图像组ID
     * 
     * 将 GNSS 数据按顺序应用到指定图像组中的对应图像。
     * 如果数据列表长度与组内图像数量不匹配，只应用对应部分。
     */
    void applyGNSSToImages(const std::vector<database::Measurement::GNSSMeasurement>& gnssDataList,
                          uint32_t groupId);

    /**
     * 通知外部组件 ImageGroup 数据已发生变化
     * @param[in] group_id 分组ID
     */
    void notifyImageGroupChanged(uint32_t group_id);

    /**
     * 生成下一个 AT Task 名称（AT_0, AT_1, ...）
     * @return 新的任务名称字符串
     */
    std::string generateNextATTaskName();

private:
    // ─────────────────────────────────────────────────────────
    // 内部辅助方法
    // ─────────────────────────────────────────────────────────
    
    void setModified(bool modified);
    void clearAllData();
    void syncCounters();
    bool loadFromFile(const QString& filepath);
    bool saveToFile(const QString& filepath);
    
    // 生成唯一ID的辅助函数
    uint32_t generateImageGroupId();
    uint32_t generateRigId();
    uint32_t generateGCPId();
    
private:
    // ─────────────────────────────────────────────────────────
    // 成员变量
    // ─────────────────────────────────────────────────────────
    
    insight::database::Project m_project;  ///< 真实项目数据
    QString m_filepath;                    ///< 项目文件路径
    bool m_modified = false;               ///< 是否有未保存的修改
    bool m_projectLoaded = false;          ///< 是否已加载项目
};

}  // namespace ui
}  // namespace insight

#endif  // UI_PROJECTDOCUMENT_H
