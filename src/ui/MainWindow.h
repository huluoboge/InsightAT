/**
 * @file MainWindow.h
 * @brief InsightAT 主窗口
 * 
 * MainWindow 是应用的主框架，负责：
 * 1. 菜单栏和工具栏管理
 * 2. 工作区布局 (左侧树 + 中心区域)
 * 3. 状态栏显示
 * 4. ProjectDocument 与 UI 的连接
 * 5. 文件操作和应用生命周期
 */

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QTreeView>
#include <QSplitter>
#include <QAction>
#include <QMenu>
#include <QMenuBar>
#include <memory>

namespace insight {
namespace ui {

// 前向声明
class ProjectDocument;
class WorkspaceTreeModel;
class NewProjectDialog;
class ImageGroupDialog;
class ProjectInfoDialog;
class CameraModelWidget;

/**
 * @class MainWindow
 * @brief 应用主窗口
 */
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

protected:
    void closeEvent(QCloseEvent* event) override;

private slots:
    // ─────────────────────────────────────────────────────
    // 文件菜单 - File Menu
    // ─────────────────────────────────────────────────────
    
    /**
     * 新建项目
     */
    void onNewProject();
    
    /**
     * 打开项目文件
     */
    void onOpenProject();
    
    /**
     * 保存项目
     */
    void onSaveProject();
    
    /**
     * 另存为
     */
    void onSaveProjectAs();
    
    /**
     * 退出应用
     */
    void onExit();

    // ─────────────────────────────────────────────────────
    // 编辑菜单 - Edit Menu
    // ─────────────────────────────────────────────────────
    
    /**
     * 查看和编辑项目信息
     */
    void onProjectInfo();
    
    /**
     * 设置坐标系
     */
    void onSetCoordinateSystem();
    
    /**
     * 添加图像分组
     */
    void onAddImageGroup();
    
    /**
     * 添加相机Rig
     */
    void onAddCameraRig();
    
    /**
     * 导入GCP数据
     */
    void onImportGCPs();

    // ─────────────────────────────────────────────────────
    // 视图菜单 - View Menu
    // ─────────────────────────────────────────────────────
    
    /**
     * 切换左侧工作区面板
     */
    void onToggleWorkspacePanel();
    
    /**
     * 切换属性面板
     */
    void onTogglePropertyPanel();

    // ─────────────────────────────────────────────────────
    // 帮助菜单 - Help Menu
    // ─────────────────────────────────────────────────────
    
    /**
     * 显示关于对话框
     */
    void onAbout();
    
    /**
     * 显示关于Qt对话框
     */
    void onAboutQt();

    // ─────────────────────────────────────────────────────
    // ProjectDocument 信号槽
    // ─────────────────────────────────────────────────────
    
    /**
     * 项目创建时更新窗口标题
     */
    void onProjectCreated();
    
    /**
     * 项目打开时更新窗口标题
     */
    void onProjectOpened();
    
    /**
     * 项目保存时更新窗口标题
     */
    void onProjectSaved();
    
    /**
     * 项目修改状态改变时更新窗口标题
     */
    void onModificationChanged(bool modified);

    /**
     * 工作区树视图双击事件处理
     */
    void onWorkspaceTreeDoubleClicked(const QModelIndex& index);

private:
    // ─────────────────────────────────────────────────────
    // 初始化方法
    // ─────────────────────────────────────────────────────
    
    /**
     * 创建菜单栏
     */
    void createMenuBar();
    
    /**
     * 创建工具栏
     */
    void createToolBar();
    
    /**
     * 创建工作区 (中央部件)
     */
    void createWorkspace();
    
    /**
     * 创建状态栏
     */
    void createStatusBar();
    
    /**
     * 连接所有信号槽
     */
    void connectSignalsSlots();
    
    /**
     * 加载应用设置 (窗口大小、位置等)
     */
    void loadSettings();
    
    /**
     * 保存应用设置
     */
    void saveSettings();
    
    /**
     * 更新窗口标题
     */
    void updateWindowTitle();

    // ─────────────────────────────────────────────────────
    // 成员变量
    // ─────────────────────────────────────────────────────
    
    // 文档和模型
    std::unique_ptr<ProjectDocument> m_projectDocument;
    std::unique_ptr<WorkspaceTreeModel> m_workspaceModel;
    
    // UI 组件 - 菜单和菜单项
    QMenu* m_fileMenu = nullptr;
    QMenu* m_editMenu = nullptr;
    QMenu* m_viewMenu = nullptr;
    QMenu* m_helpMenu = nullptr;
    
    // 文件菜单动作
    QAction* m_actionNewProject = nullptr;
    QAction* m_actionOpenProject = nullptr;
    QAction* m_actionSaveProject = nullptr;
    QAction* m_actionSaveProjectAs = nullptr;
    QAction* m_actionExit = nullptr;
    
    // 编辑菜单动作
    QAction* m_actionProjectInfo = nullptr;
    QAction* m_actionSetCoordinateSystem = nullptr;
    QAction* m_actionAddImageGroup = nullptr;
    QAction* m_actionAddCameraRig = nullptr;
    QAction* m_actionImportGCPs = nullptr;
    
    // 视图菜单动作
    QAction* m_actionToggleWorkspacePanel = nullptr;
    QAction* m_actionTogglePropertyPanel = nullptr;
    
    // 帮助菜单动作
    QAction* m_actionAbout = nullptr;
    QAction* m_actionAboutQt = nullptr;
    
    // UI 组件 - 工作区
    QSplitter* m_splitter = nullptr;              ///< 左右分割器
    QTreeView* m_workspaceTreeView = nullptr;     ///< 左侧工作区树
    QWidget* m_centerWidget = nullptr;            ///< 中央区域 (将来放3D视图/属性面板)
    
    // UI 组件 - 状态栏
    QLabel* m_statusLabel = nullptr;              ///< 状态信息标签
    QLabel* m_projectNameLabel = nullptr;         ///< 项目名称标签
    QLabel* m_modifiedIndicator = nullptr;        ///< 修改指示符
    
    // 应用状态
    QString m_currentFilePath;                    ///< 当前打开的文件路径
    bool m_isModified = false;                    ///< 是否有未保存的修改
    
    // 对话框和小部件（延迟创建）
    std::unique_ptr<NewProjectDialog> m_newProjectDialog;
    std::unique_ptr<ImageGroupDialog> m_imageGroupDialog;
};

}  // namespace ui
}  // namespace insight

#endif  // UI_MAINWINDOW_H
