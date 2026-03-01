/**
 * @file WorkspaceTreeModel.h
 * @brief 工作区树模型 - 显示项目结构的树形视图
 * 
 * 树结构：
 * Project (root)
 *  ├─ Project Info
 *  ├─ Images
 *  │  ├─ ImageGroup_1
 *  │  │  ├─ Image_1
 *  │  │  └─ Image_2
 *  │  └─ ImageGroup_2
 *  ├─ Cameras
 *  │  ├─ Camera (if GroupLevel)
 *  │  └─ CameraRig_1
 *  │     ├─ Camera_Nadir
 *  │     └─ Camera_Forward
 *  ├─ GCPs
 *  │  ├─ GCP_1 [x, y, z]
 *  │  └─ GCP_2 [x, y, z]
 *  └─ AT Tasks
 *     └─ Task_1
 */

#ifndef UI_WORKSPACETREEMODEL_H
#define UI_WORKSPACETREEMODEL_H

#include <QAbstractItemModel>
#include <memory>

namespace insight {
namespace ui {

class ProjectDocument;

/**
 * @class WorkspaceTreeModel
 * @brief 树形模型，表示项目的层次结构
 */
class WorkspaceTreeModel : public QAbstractItemModel {
    Q_OBJECT

signals:
    /**
     * 树被刷新完成后发出此信号
     */
    void treeRefreshed();

public:
    /**
     * 树节点类型枚举
     */
    enum NodeType {
        ProjectRoot,           ///< 根节点（项目）
        ProjectInfoNode,       ///< 项目信息
        ImagesNode,            ///< 图像分组容器
        ImageGroupNode,        ///< 单个图像分组
        ImageNode,             ///< 单个图像
        CamerasNode,           ///< 相机配置容器
        SingleCameraNode,      ///< 单个相机（GroupLevel模式）
        CameraRigsNode,        ///< 多相机Rig容器
        CameraRigNode,         ///< 单个多相机Rig
        CameraRigMountNode,    ///< Rig中的单个相机
        GCPsNode,              ///< GCP容器
        GCPNode,               ///< 单个GCP
        ATTasksNode,           ///< 空三任务容器
        ATTaskNode             ///< 单个空三任务
    };

    /**
     * 树节点内部结构
     */
    struct TreeNode {
        NodeType type;
        QString displayName;
        
        // 关联的数据ID
        uint32_t groupId = (uint32_t)-1;    ///< 对于ImageGroup和Rig
        uint32_t imageId = (uint32_t)-1;    ///< 对于Image
        uint32_t gcpId = (uint32_t)-1;      ///< 对于GCP
        std::string taskId;                 ///< 对于ATTask
        
        // 树结构
        TreeNode* parent = nullptr;
        std::vector<std::unique_ptr<TreeNode>> children;
        
        TreeNode(NodeType t, const QString& name)
            : type(t), displayName(name) {}
    };

    /**
     * 构造函数（无参）
     * 
     * @param[in] parent 父部件
     */
    explicit WorkspaceTreeModel(QObject* parent = nullptr);
    
    /**
     * 构造函数
     * 
     * @param[in] doc 项目文档指针
     * @param[in] parent 父部件
     */
    explicit WorkspaceTreeModel(ProjectDocument* doc, QObject* parent = nullptr);
    
    ~WorkspaceTreeModel();
    
    /**
     * 设置项目文档
     * 
     * @param[in] doc 项目文档指针
     */
    void setProjectDocument(ProjectDocument* doc);

    // ─────────────────────────────────────────────────────────
    // QAbstractItemModel 接口实现
    // ─────────────────────────────────────────────────────────
    
    QModelIndex index(int row, int column,
                     const QModelIndex& parent = QModelIndex()) const override;
    QModelIndex parent(const QModelIndex& child) const override;
    int rowCount(const QModelIndex& parent = QModelIndex()) const override;
    int columnCount(const QModelIndex& parent = QModelIndex()) const override;
    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
    Qt::ItemFlags flags(const QModelIndex& index) const override;

    // ─────────────────────────────────────────────────────────
    // 公共方法
    // ─────────────────────────────────────────────────────────
    
    /**
     * 获取指定索引对应的树节点
     */
    TreeNode* getNode(const QModelIndex& index) const;
    
    /**
     * 刷新整个树
     */
    void refreshTree();

public slots:
    // 响应ProjectDocument的信号
    void onProjectChanged();
    void onImageGroupAdded(uint32_t group_id);
    void onImageGroupRemoved(uint32_t group_id);
    void onImageGroupChanged(uint32_t group_id);
    void onCameraRigAdded(uint32_t rig_id);
    void onCameraRigRemoved(uint32_t rig_id);
    void onCameraRigChanged(uint32_t rig_id);
    void onGCPAdded(uint32_t gcp_id);
    void onGCPRemoved(uint32_t gcp_id);
    void onGCPChanged(uint32_t gcp_id);
    void onATTaskCreated(const QString& task_id);
    void onATTaskRemoved(const QString& task_id);
    void onATTaskChanged(const QString& task_id);

private:
    // ─────────────────────────────────────────────────────────
    // 内部方法
    // ─────────────────────────────────────────────────────────
    
    /**
     * 构建整个树的内部结构
     */
    void buildTree();
    
    /**
     * 清空树
     */
    void clearTree();
    
    /**
     * 递归查找指定 taskId 的节点
     */
    TreeNode* findATTaskNode(TreeNode* node, const std::string& taskId);
    
    /**
     * 增量更新 AT Task 节点（只更新名称，不重建整个树）
     */
    void updateATTaskNode(const std::string& taskId);

private:
    ProjectDocument* m_document;
    std::unique_ptr<TreeNode> m_root;
};

}  // namespace ui
}  // namespace insight

#endif  // UI_WORKSPACETREEMODEL_H
