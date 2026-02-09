/**
 * @file WorkspaceTreeModel.cpp
 * @brief WorkspaceTreeModel 实现
 */

#include "WorkspaceTreeModel.h"
#include "ProjectDocument.h"
#include <glog/logging.h>

namespace insight {
namespace ui {

WorkspaceTreeModel::WorkspaceTreeModel(QObject* parent)
    : QAbstractItemModel(parent), m_document(nullptr) {
}

WorkspaceTreeModel::WorkspaceTreeModel(ProjectDocument* doc, QObject* parent)
    : QAbstractItemModel(parent), m_document(doc) {
    
    if (!m_document) {
        LOG(ERROR) << "ProjectDocument is null";
        return;
    }
    
    // 连接 ProjectDocument 的信号
    connect(m_document, &ProjectDocument::projectCreated,
           this, &WorkspaceTreeModel::onProjectChanged);
    connect(m_document, &ProjectDocument::projectOpened,
           this, &WorkspaceTreeModel::onProjectChanged);
    connect(m_document, &ProjectDocument::projectCleared,
           this, &WorkspaceTreeModel::onProjectChanged);
    
    connect(m_document, &ProjectDocument::imageGroupAdded,
           this, &WorkspaceTreeModel::onImageGroupAdded);
    connect(m_document, &ProjectDocument::imageGroupRemoved,
           this, &WorkspaceTreeModel::onImageGroupRemoved);
    connect(m_document, &ProjectDocument::imageGroupChanged,
           this, &WorkspaceTreeModel::onImageGroupChanged);
    
    connect(m_document, &ProjectDocument::cameraRigAdded,
           this, &WorkspaceTreeModel::onCameraRigAdded);
    connect(m_document, &ProjectDocument::cameraRigRemoved,
           this, &WorkspaceTreeModel::onCameraRigRemoved);
    connect(m_document, &ProjectDocument::cameraRigChanged,
           this, &WorkspaceTreeModel::onCameraRigChanged);
    
    connect(m_document, &ProjectDocument::gcpAdded,
           this, &WorkspaceTreeModel::onGCPAdded);
    connect(m_document, &ProjectDocument::gcpRemoved,
           this, &WorkspaceTreeModel::onGCPRemoved);
    connect(m_document, &ProjectDocument::gcpChanged,
           this, &WorkspaceTreeModel::onGCPChanged);
    
    connect(m_document, &ProjectDocument::atTaskCreated,
           this, &WorkspaceTreeModel::onATTaskCreated);
    connect(m_document, &ProjectDocument::atTaskRemoved,
           this, &WorkspaceTreeModel::onATTaskRemoved);
    connect(m_document, &ProjectDocument::atTaskChanged,
           this, &WorkspaceTreeModel::onATTaskChanged);
    
    // 初始化根节点
    m_root = std::make_unique<TreeNode>(ProjectRoot, "Project");
    
    // 如果已有项目，构建树
    if (m_document->isProjectLoaded()) {
        buildTree();
    }
}

WorkspaceTreeModel::~WorkspaceTreeModel() {
}

void WorkspaceTreeModel::setProjectDocument(ProjectDocument* doc) {
    if (m_document == doc) {
        return;
    }
    
    m_document = doc;
    
    if (!m_document) {
        clearTree();
        return;
    }
    
    // 连接新的 ProjectDocument 信号
    connect(m_document, &ProjectDocument::projectCreated,
           this, &WorkspaceTreeModel::onProjectChanged);
    connect(m_document, &ProjectDocument::projectOpened,
           this, &WorkspaceTreeModel::onProjectChanged);
    connect(m_document, &ProjectDocument::projectCleared,
           this, &WorkspaceTreeModel::onProjectChanged);
    
    connect(m_document, &ProjectDocument::imageGroupAdded,
           this, &WorkspaceTreeModel::onImageGroupAdded);
    connect(m_document, &ProjectDocument::imageGroupRemoved,
           this, &WorkspaceTreeModel::onImageGroupRemoved);
    connect(m_document, &ProjectDocument::imageGroupChanged,
           this, &WorkspaceTreeModel::onImageGroupChanged);
    
    connect(m_document, &ProjectDocument::cameraRigAdded,
           this, &WorkspaceTreeModel::onCameraRigAdded);
    connect(m_document, &ProjectDocument::cameraRigRemoved,
           this, &WorkspaceTreeModel::onCameraRigRemoved);
    connect(m_document, &ProjectDocument::cameraRigChanged,
           this, &WorkspaceTreeModel::onCameraRigChanged);
    
    connect(m_document, &ProjectDocument::gcpAdded,
           this, &WorkspaceTreeModel::onGCPAdded);
    connect(m_document, &ProjectDocument::gcpRemoved,
           this, &WorkspaceTreeModel::onGCPRemoved);
    connect(m_document, &ProjectDocument::gcpChanged,
           this, &WorkspaceTreeModel::onGCPChanged);
    
    connect(m_document, &ProjectDocument::atTaskCreated,
           this, &WorkspaceTreeModel::onATTaskCreated);
    connect(m_document, &ProjectDocument::atTaskRemoved,
           this, &WorkspaceTreeModel::onATTaskRemoved);
    connect(m_document, &ProjectDocument::atTaskChanged,
           this, &WorkspaceTreeModel::onATTaskChanged);
    
    // 初始化根节点
    m_root = std::make_unique<TreeNode>(ProjectRoot, "Project");
    
    // 如果已有项目，构建树
    if (m_document->isProjectLoaded()) {
        buildTree();
    }
    
    // 通知视图数据已改变
    beginResetModel();
    endResetModel();
}

// ─────────────────────────────────────────────────────────────
// QAbstractItemModel 接口
// ─────────────────────────────────────────────────────────────

QModelIndex WorkspaceTreeModel::index(int row, int column,
                                      const QModelIndex& parent) const {
    if (!hasIndex(row, column, parent)) {
        return QModelIndex();
    }
    
    TreeNode* parentNode = nullptr;
    if (!parent.isValid()) {
        parentNode = m_root.get();
    } else {
        parentNode = static_cast<TreeNode*>(parent.internalPointer());
    }
    
    if (!parentNode || row < 0 || row >= static_cast<int>(parentNode->children.size())) {
        return QModelIndex();
    }
    
    TreeNode* childNode = parentNode->children[row].get();
    return createIndex(row, column, childNode);
}

QModelIndex WorkspaceTreeModel::parent(const QModelIndex& child) const {
    if (!child.isValid()) {
        return QModelIndex();
    }
    
    TreeNode* childNode = static_cast<TreeNode*>(child.internalPointer());
    TreeNode* parentNode = childNode->parent;
    
    if (!parentNode || parentNode == m_root.get()) {
        return QModelIndex();
    }
    
    // 查找 parentNode 在其父节点中的位置
    TreeNode* grandparentNode = parentNode->parent;
    for (size_t i = 0; i < grandparentNode->children.size(); ++i) {
        if (grandparentNode->children[i].get() == parentNode) {
            return createIndex(i, 0, parentNode);
        }
    }
    
    return QModelIndex();
}

int WorkspaceTreeModel::rowCount(const QModelIndex& parent) const {
    if (!m_root) {
        return 0;
    }
    
    TreeNode* node = nullptr;
    if (!parent.isValid()) {
        node = m_root.get();
    } else {
        node = static_cast<TreeNode*>(parent.internalPointer());
    }
    
    return node ? static_cast<int>(node->children.size()) : 0;
}

int WorkspaceTreeModel::columnCount(const QModelIndex& /*parent*/) const {
    return 1;  // 只有一列
}

QVariant WorkspaceTreeModel::data(const QModelIndex& index, int role) const {
    if (!index.isValid()) {
        return QVariant();
    }
    
    TreeNode* node = static_cast<TreeNode*>(index.internalPointer());
    
    if (role == Qt::DisplayRole || role == Qt::EditRole) {
        return node->displayName;
    }
    
    return QVariant();
}

Qt::ItemFlags WorkspaceTreeModel::flags(const QModelIndex& index) const {
    if (!index.isValid()) {
        return Qt::NoItemFlags;
    }
    
    return Qt::ItemIsEnabled | Qt::ItemIsSelectable;
}

// ─────────────────────────────────────────────────────────────
// 公共方法
// ─────────────────────────────────────────────────────────────

WorkspaceTreeModel::TreeNode* WorkspaceTreeModel::getNode(const QModelIndex& index) const {
    if (!index.isValid()) {
        return m_root.get();
    }
    
    return static_cast<TreeNode*>(index.internalPointer());
}

void WorkspaceTreeModel::refreshTree() {
    beginResetModel();
    clearTree();
    if (m_document && m_document->isProjectLoaded()) {
        buildTree();
    }
    endResetModel();
    
    // 树刷新完成，发出信号
    emit treeRefreshed();
}

// ─────────────────────────────────────────────────────────────
// Slots
// ─────────────────────────────────────────────────────────────

void WorkspaceTreeModel::onProjectChanged() {
    refreshTree();
}

void WorkspaceTreeModel::onImageGroupAdded(uint32_t group_id) {
    // 找到 ImagesNode 或创建它
    TreeNode* imagesNode = nullptr;
    for (auto& child : m_root->children) {
        if (child->type == ImagesNode) {
            imagesNode = child.get();
            break;
        }
    }
    
    if (!imagesNode) {
        // 需要先创建 ImagesNode
        auto newNode = std::make_unique<TreeNode>(ImagesNode, "Images");
        newNode->parent = m_root.get();
        imagesNode = newNode.get();
        m_root->children.push_back(std::move(newNode));
    }
    
    // 添加新的 ImageGroupNode
    auto groupNode = std::make_unique<TreeNode>(ImageGroupNode, 
                                                QString("Group %1").arg(group_id));
    groupNode->groupId = group_id;
    groupNode->parent = imagesNode;
    
    int position = imagesNode->children.size();
    
    // 找到 ImagesNode 在 root 中的索引
    int imagesNodeIndex = 0;
    for (size_t i = 0; i < m_root->children.size(); ++i) {
        if (m_root->children[i].get() == imagesNode) {
            imagesNodeIndex = i;
            break;
        }
    }
    
    beginInsertRows(index(imagesNodeIndex, 0, QModelIndex()),
                   position, position);
    
    imagesNode->children.push_back(std::move(groupNode));
    
    endInsertRows();
}

void WorkspaceTreeModel::onImageGroupRemoved(uint32_t group_id) {
    // 找到并移除该分组
    for (size_t i = 0; i < m_root->children.size(); ++i) {
        auto& child = m_root->children[i];
        if (child->type == ImagesNode) {
            for (size_t j = 0; j < child->children.size(); ++j) {
                if (child->children[j]->type == ImageGroupNode 
                    && child->children[j]->groupId == group_id) {
                    
                    beginRemoveRows(index(i, 0), j, j);
                    child->children.erase(child->children.begin() + j);
                    endRemoveRows();
                    return;
                }
            }
        }
    }
}

void WorkspaceTreeModel::onImageGroupChanged(uint32_t group_id) {
    // 刷新该分组的显示
    // TODO: 实现增量更新而不是完全刷新
    refreshTree();
}

void WorkspaceTreeModel::onCameraRigAdded(uint32_t rig_id) {
    // 类似 onImageGroupAdded 的逻辑
    refreshTree();
}

void WorkspaceTreeModel::onCameraRigRemoved(uint32_t rig_id) {
    refreshTree();
}

void WorkspaceTreeModel::onCameraRigChanged(uint32_t rig_id) {
    refreshTree();
}

void WorkspaceTreeModel::onGCPAdded(uint32_t gcp_id) {
    refreshTree();
}

void WorkspaceTreeModel::onGCPRemoved(uint32_t gcp_id) {
    refreshTree();
}

void WorkspaceTreeModel::onGCPChanged(uint32_t gcp_id) {
    refreshTree();
}

void WorkspaceTreeModel::onATTaskCreated(const QString& task_id) {
    refreshTree();
}

void WorkspaceTreeModel::onATTaskRemoved(const QString& task_id) {
    refreshTree();
}

void WorkspaceTreeModel::onATTaskChanged(const QString& task_id) {
    // 任务被修改（如名称改变），增量更新节点而不是重建整个树
    std::string taskIdStr = task_id.toStdString();
    updateATTaskNode(taskIdStr);
}

// ─────────────────────────────────────────────────────────────
// 内部方法
// ─────────────────────────────────────────────────────────────

void WorkspaceTreeModel::buildTree() {
    if (!m_document || !m_document->isProjectLoaded()) {
        return;
    }
    
    const auto& project = m_document->project();
    
    // ─── 项目信息节点
    auto infoNode = std::make_unique<TreeNode>(ProjectInfoNode, "Project Info");
    infoNode->parent = m_root.get();
    m_root->children.push_back(std::move(infoNode));
    
    // ─── 图像分组节点（总是创建，即使为空）
    auto imagesContainerNode = std::make_unique<TreeNode>(ImagesNode, "Image Groups");
    imagesContainerNode->parent = m_root.get();
    
    for (const auto& group : project.image_groups) {
        auto groupNode = std::make_unique<TreeNode>(ImageGroupNode,
                                                   QString::fromStdString(group.group_name));
        groupNode->groupId = group.group_id;
        groupNode->parent = imagesContainerNode.get();
        
        // 不显示组内的图像 - 只显示分组节点
        // for (const auto& image : group.images) {
        //     auto imageNode = std::make_unique<TreeNode>(ImageNode,
        //                                                QString::fromStdString(image.filename));
        //     imageNode->groupId = group.group_id;
        //     imageNode->imageId = image.image_id;
        //     imageNode->parent = groupNode.get();
        //     groupNode->children.push_back(std::move(imageNode));
        // }
        
        imagesContainerNode->children.push_back(std::move(groupNode));
    }
    
    m_root->children.push_back(std::move(imagesContainerNode));
    
    // ─── 相机节点
    if (!project.camera_rigs.empty()) {
        auto camerasContainerNode = std::make_unique<TreeNode>(CamerasNode, "Cameras");
        camerasContainerNode->parent = m_root.get();
        
        for (const auto& [rig_id, rig] : project.camera_rigs) {
            auto rigNode = std::make_unique<TreeNode>(CameraRigNode,
                                                     QString::fromStdString(rig.rig_name));
            rigNode->groupId = rig_id;
            rigNode->parent = camerasContainerNode.get();
            
            // 添加 rig 内的相机
            for (const auto& mount : rig.mounts) {
                auto mountNode = std::make_unique<TreeNode>(CameraRigMountNode,
                                                           QString::fromStdString(mount.position_name));
                mountNode->parent = rigNode.get();
                rigNode->children.push_back(std::move(mountNode));
            }
            
            camerasContainerNode->children.push_back(std::move(rigNode));
        }
        
        m_root->children.push_back(std::move(camerasContainerNode));
    }
    
    // ─── GCP节点
    if (!project.gcp_database.empty()) {
        auto gcpsContainerNode = std::make_unique<TreeNode>(GCPsNode, "GCPs");
        gcpsContainerNode->parent = m_root.get();
        
        for (const auto& [gcp_id, gcp] : project.gcp_database) {
            QString displayName = QString("GCP_%1 [%2, %3, %4]")
                                 .arg(gcp_id)
                                 .arg(gcp.x, 0, 'f', 2)
                                 .arg(gcp.y, 0, 'f', 2)
                                 .arg(gcp.z, 0, 'f', 2);
            
            auto gcpNode = std::make_unique<TreeNode>(GCPNode, displayName);
            gcpNode->gcpId = gcp_id;
            gcpNode->parent = gcpsContainerNode.get();
            gcpsContainerNode->children.push_back(std::move(gcpNode));
        }
        
        m_root->children.push_back(std::move(gcpsContainerNode));
    }
    
    // ─── AT任务节点 - 构建真实的树结构（包括父子关系）
    if (!project.at_tasks.empty()) {
        auto tasksContainerNode = std::make_unique<TreeNode>(ATTasksNode, "AT Tasks");
        tasksContainerNode->parent = m_root.get();
        
        // 首先创建所有任务节点
        std::vector<TreeNode*> taskNodePtrs;
        for (size_t i = 0; i < project.at_tasks.size(); ++i) {
            const auto& task = project.at_tasks[i];
            auto taskNode = std::make_unique<TreeNode>(ATTaskNode,
                                                      QString::fromStdString(task.task_name));
            taskNode->taskId = task.id;
            TreeNode* taskNodePtr = taskNode.get();
            tasksContainerNode->children.push_back(std::move(taskNode));
            taskNodePtrs.push_back(taskNodePtr);
        }
        
        // 然后根据父子关系重新组织树结构
        // 重新整理：只有没有子任务的节点才保留在容器中
        std::vector<std::unique_ptr<TreeNode>> rootTaskNodes;
        std::vector<bool> isChild(project.at_tasks.size(), false);
        
        // 标记所有有父任务的节点
        for (size_t i = 0; i < project.at_tasks.size(); ++i) {
            const auto& task = project.at_tasks[i];
            if (task.initialization && task.initialization->prev_task_id != static_cast<uint32_t>(-1)) {
                uint32_t parentIdx = task.initialization->prev_task_id;
                if (parentIdx < project.at_tasks.size()) {
                    isChild[i] = true;
                }
            }
        }
        
        // 重新构建树：将子任务添加到父任务下，只保留根任务在容器中
        tasksContainerNode->children.clear();
        
        for (size_t i = 0; i < project.at_tasks.size(); ++i) {
            if (!isChild[i]) {
                // 这是一个根任务，应该直接添加到容器中
                auto rootTaskNode = std::make_unique<TreeNode>(ATTaskNode,
                                                               QString::fromStdString(project.at_tasks[i].task_name));
                rootTaskNode->taskId = project.at_tasks[i].id;
                rootTaskNode->parent = tasksContainerNode.get();
                
                // 找出所有继承自这个任务的后续任务，形成树链
                std::function<void(TreeNode*, size_t)> buildTaskHierarchy = 
                    [&](TreeNode* parentNode, size_t parentIdx) {
                        for (size_t j = i + 1; j < project.at_tasks.size(); ++j) {
                            const auto& task = project.at_tasks[j];
                            if (task.initialization && task.initialization->prev_task_id == static_cast<uint32_t>(parentIdx)) {
                                auto childTaskNode = std::make_unique<TreeNode>(ATTaskNode,
                                                                                QString::fromStdString(task.task_name));
                                childTaskNode->taskId = task.id;
                                childTaskNode->parent = parentNode;
                                TreeNode* childPtr = childTaskNode.get();
                                parentNode->children.push_back(std::move(childTaskNode));
                                
                                // 递归处理下一级
                                buildTaskHierarchy(childPtr, j);
                            }
                        }
                    };
                
                TreeNode* rootPtr = rootTaskNode.get();
                tasksContainerNode->children.push_back(std::move(rootTaskNode));
                buildTaskHierarchy(rootPtr, i);
            }
        }
        
        m_root->children.push_back(std::move(tasksContainerNode));
    }
}

void WorkspaceTreeModel::clearTree() {
    m_root->children.clear();
}

WorkspaceTreeModel::TreeNode* WorkspaceTreeModel::findATTaskNode(TreeNode* node, 
                                                                  const std::string& taskId) {
    if (!node) {
        return nullptr;
    }
    
    // 检查当前节点
    if (node->type == ATTaskNode && node->taskId == taskId) {
        return node;
    }
    
    // 递归搜索子节点
    for (auto& child : node->children) {
        TreeNode* result = findATTaskNode(child.get(), taskId);
        if (result) {
            return result;
        }
    }
    
    return nullptr;
}

void WorkspaceTreeModel::updateATTaskNode(const std::string& taskId) {
    if (!m_document || !m_document->isProjectLoaded()) {
        return;
    }
    
    // 查找树中对应的节点
    TreeNode* taskNode = findATTaskNode(m_root.get(), taskId);
    if (!taskNode) {
        LOG(WARNING) << "AT Task node not found in tree: " << taskId;
        return;
    }
    
    // 获取最新的 task 数据
    const auto* task = m_document->getATTaskById(taskId);
    if (!task) {
        LOG(ERROR) << "AT Task not found in document: " << taskId;
        return;
    }
    
    // 更新节点的显示名称
    QString newDisplayName = QString::fromStdString(task->task_name);
    if (taskNode->displayName != newDisplayName) {
        taskNode->displayName = newDisplayName;
        
        // 发出 dataChanged 信号以更新视图
        // 需要找到该节点在其父节点中的索引
        if (taskNode->parent) {
            for (size_t i = 0; i < taskNode->parent->children.size(); ++i) {
                if (taskNode->parent->children[i].get() == taskNode) {
                    TreeNode* parentNode = taskNode->parent;
                    TreeNode* grandParentNode = parentNode->parent;
                    
                    if (grandParentNode) {
                        int parentRow = 0;
                        for (size_t j = 0; j < grandParentNode->children.size(); ++j) {
                            if (grandParentNode->children[j].get() == parentNode) {
                                parentRow = j;
                                break;
                            }
                        }
                        
                        QModelIndex parentIdx = createIndex(parentRow, 0, parentNode);
                        QModelIndex nodeIdx = createIndex(i, 0, taskNode);
                        
                        emit dataChanged(nodeIdx, nodeIdx, {Qt::DisplayRole});
                    }
                    break;
                }
            }
        }
    }
}

}  // namespace ui
}  // namespace insight

