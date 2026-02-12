/**
 * @file MainWindow.cpp
 * @brief MainWindow 实现
 */

#include "MainWindow.h"
#include "models/ProjectDocument.h"
#include "models/WorkspaceTreeModel.h"
#include "dialogs/NewProjectDialog.h"
#include "dialogs/ImageGroupDialog.h"
#include "dialogs/ProjectInfoDialog.h"
#include "dialogs/CoordinateSystemConfigDialog.h"
#include "dialogs/ImageGroupDetailPanel.h"
#include "dialogs/NewATTaskDialog.h"
#include "widgets/SpatialReferenceDialog.h"
#include "widgets/ImageGroupsManagementPanel.h"
#include "panels/ATTaskPanel.h"
#include "UISystemConfig.h"
#include "../Common/Coordinates.h"
#include "../database/database_types.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QToolBar>
#include <QStatusBar>
#include <QTreeView>
#include <QSplitter>
#include <QLabel>
#include <QFileDialog>
#include <QMessageBox>
#include <QSettings>
#include <QApplication>
#include <QCloseEvent>
#include <QShowEvent>
#include <QDialog>
#include <QPushButton>
#include <glog/logging.h>

namespace insight {
namespace ui {

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent) {
    
    // 初始化 UISystemConfig（加载坐标系数据库）
    UISystemConfig& config = UISystemConfig::instance();
    
    // 设置配置路径（优先尝试 build/config，备选 ../data/config）
    // config.setConfigPath("./config");
    // if (!config.loadCoordinateDatabases()) {
    //     LOG(WARNING) << "Failed to load from ./config, trying ../data/config";
    //     config.setConfigPath("../data/config");
    //     if (!config.loadCoordinateDatabases()) {
    //         LOG(ERROR) << "Failed to load coordinate databases from both paths";
    //     }
    // }
    
    // 设置窗口属性
    setWindowTitle("InsightAT - Photogrammetry Suite");
    setWindowIcon(QIcon(":/icons/app_icon.png"));  // TODO: 添加图标资源
    
    // 设置最小窗口大小
    setMinimumSize(1200, 800);
    
    // 创建项目文档
    m_projectDocument = std::make_unique<ProjectDocument>();
    
    // 创建工作区模型
    m_workspaceModel = std::make_unique<WorkspaceTreeModel>();
    
    // 创建 UI 组件
    createMenuBar();
    createToolBar();
    createWorkspace();
    createStatusBar();
    
    // 连接信号槽
    connectSignalsSlots();
    
    // 加载应用设置
    loadSettings();
    
    LOG(INFO) << "MainWindow initialized";
}

MainWindow::~MainWindow() {
    saveSettings();
}

void MainWindow::createMenuBar() {
    // ─────────────────────────────────────────────────────
    // 文件菜单 (File Menu)
    // ─────────────────────────────────────────────────────
    m_fileMenu = menuBar()->addMenu(tr("&File"));
    
    m_actionNewProject = m_fileMenu->addAction(tr("&New Project"));
    m_actionNewProject->setShortcut(QKeySequence::New);
    connect(m_actionNewProject, &QAction::triggered, this, &MainWindow::onNewProject);
    
    m_actionOpenProject = m_fileMenu->addAction(tr("&Open Project..."));
    m_actionOpenProject->setShortcut(QKeySequence::Open);
    connect(m_actionOpenProject, &QAction::triggered, this, &MainWindow::onOpenProject);
    
    m_fileMenu->addSeparator();
    
    m_actionSaveProject = m_fileMenu->addAction(tr("&Save Project"));
    m_actionSaveProject->setShortcut(QKeySequence::Save);
    m_actionSaveProject->setEnabled(false);
    connect(m_actionSaveProject, &QAction::triggered, this, &MainWindow::onSaveProject);
    
    m_actionSaveProjectAs = m_fileMenu->addAction(tr("Save Project &As..."));
    m_actionSaveProjectAs->setShortcut(QKeySequence::SaveAs);
    m_actionSaveProjectAs->setEnabled(false);
    connect(m_actionSaveProjectAs, &QAction::triggered, this, &MainWindow::onSaveProjectAs);
    
    m_fileMenu->addSeparator();
    
    m_actionExit = m_fileMenu->addAction(tr("E&xit"));
    m_actionExit->setShortcut(QKeySequence::Quit);
    connect(m_actionExit, &QAction::triggered, this, &MainWindow::onExit);
    
    // ─────────────────────────────────────────────────────
    // 编辑菜单 (Edit Menu)
    // ─────────────────────────────────────────────────────
    m_editMenu = menuBar()->addMenu(tr("&Edit"));
    
    m_actionProjectInfo = m_editMenu->addAction(tr("Project &Info..."));
    m_actionProjectInfo->setEnabled(false);
    connect(m_actionProjectInfo, &QAction::triggered, this, &MainWindow::onProjectInfo);
    
    m_editMenu->addSeparator();
    
    m_actionSetCoordinateSystem = m_editMenu->addAction(tr("Set &Coordinate System..."));
    m_actionSetCoordinateSystem->setEnabled(false);
    connect(m_actionSetCoordinateSystem, &QAction::triggered, this, &MainWindow::onSetCoordinateSystem);
    
    m_editMenu->addSeparator();
    
    m_actionAddImageGroup = m_editMenu->addAction(tr("Add Image &Group"));
    m_actionAddImageGroup->setShortcut(Qt::CTRL | Qt::Key_G);
    m_actionAddImageGroup->setEnabled(false);
    connect(m_actionAddImageGroup, &QAction::triggered, this, &MainWindow::onAddImageGroup);
    
    m_actionAddCameraRig = m_editMenu->addAction(tr("Add Camera &Rig"));
    m_actionAddCameraRig->setShortcut(Qt::CTRL | Qt::Key_R);
    m_actionAddCameraRig->setEnabled(false);
    connect(m_actionAddCameraRig, &QAction::triggered, this, &MainWindow::onAddCameraRig);
    
    m_actionImportGCPs = m_editMenu->addAction(tr("&Import GCPs..."));
    m_actionImportGCPs->setShortcut(Qt::CTRL | Qt::Key_I);
    m_actionImportGCPs->setEnabled(false);
    connect(m_actionImportGCPs, &QAction::triggered, this, &MainWindow::onImportGCPs);
    
    m_editMenu->addSeparator();
    
    m_actionCreateATTask = m_editMenu->addAction(tr("Create &AT Task"));
    m_actionCreateATTask->setShortcut(Qt::CTRL | Qt::Key_T);
    m_actionCreateATTask->setEnabled(false);
    connect(m_actionCreateATTask, &QAction::triggered, this, &MainWindow::onCreateATTask);
    
    // ─────────────────────────────────────────────────────
    // 视图菜单 (View Menu)
    // ─────────────────────────────────────────────────────
    m_viewMenu = menuBar()->addMenu(tr("&View"));
    
    m_actionToggleWorkspacePanel = m_viewMenu->addAction(tr("Toggle &Workspace Panel"));
    m_actionToggleWorkspacePanel->setCheckable(true);
    m_actionToggleWorkspacePanel->setChecked(true);
    connect(m_actionToggleWorkspacePanel, &QAction::triggered, this, &MainWindow::onToggleWorkspacePanel);
    
    m_actionTogglePropertyPanel = m_viewMenu->addAction(tr("Toggle &Property Panel"));
    m_actionTogglePropertyPanel->setCheckable(true);
    m_actionTogglePropertyPanel->setChecked(true);
    m_actionTogglePropertyPanel->setEnabled(false);  // TODO: 后续实现
    connect(m_actionTogglePropertyPanel, &QAction::triggered, this, &MainWindow::onTogglePropertyPanel);
    
    // ─────────────────────────────────────────────────────
    // 帮助菜单 (Help Menu)
    // ─────────────────────────────────────────────────────
    m_helpMenu = menuBar()->addMenu(tr("&Help"));
    
    m_actionAbout = m_helpMenu->addAction(tr("&About InsightAT"));
    connect(m_actionAbout, &QAction::triggered, this, &MainWindow::onAbout);
    
    m_actionAboutQt = m_helpMenu->addAction(tr("About &Qt"));
    connect(m_actionAboutQt, &QAction::triggered, this, &MainWindow::onAboutQt);
}

void MainWindow::createToolBar() {
    // 创建主工具栏
    QToolBar* toolbar = addToolBar(tr("Main Toolbar"));
    toolbar->setObjectName("MainToolbar");
    
    // 文件操作
    toolbar->addAction(m_actionNewProject);
    toolbar->addAction(m_actionOpenProject);
    toolbar->addAction(m_actionSaveProject);
    toolbar->addSeparator();
    
    // 编辑操作
    toolbar->addAction(m_actionAddImageGroup);
    toolbar->addAction(m_actionAddCameraRig);
    toolbar->addAction(m_actionImportGCPs);
    toolbar->addSeparator();
    toolbar->addAction(m_actionCreateATTask);
}

void MainWindow::createWorkspace() {
    // 创建中央部件
    QWidget* centralWidget = new QWidget(this);
    QHBoxLayout* layout = new QHBoxLayout(centralWidget);
    layout->setContentsMargins(0, 0, 0, 0);
    
    // 创建分割器
    m_splitter = new QSplitter(Qt::Horizontal);
    
    // 左侧：工作区树
    m_workspaceTreeView = new QTreeView();
    m_workspaceTreeView->setMinimumWidth(200);
    m_workspaceTreeView->setMaximumWidth(400);
    m_workspaceTreeView->setModel(m_workspaceModel.get());
    m_workspaceTreeView->setContextMenuPolicy(Qt::CustomContextMenu);
    m_workspaceTreeView->setHeaderHidden(true);  // 隐藏列标题
    m_splitter->addWidget(m_workspaceTreeView);
    
    // 中央/右侧：内容区域 (使用 QStackedWidget 管理多个面板)
    // 暂时创建占位符，ImageGroupsManagementPanel 将在连接阶段创建
    m_centerWidget = new QWidget();
    m_centerWidget->setMinimumWidth(400);
    QVBoxLayout* centerLayout = new QVBoxLayout(m_centerWidget);
    centerLayout->setContentsMargins(40, 40, 40, 40);
    
    // 欢迎页面
    QWidget* welcomeWidget = new QWidget();
    QVBoxLayout* welcomeLayout = new QVBoxLayout(welcomeWidget);
    welcomeLayout->setAlignment(Qt::AlignCenter);
    
    // Logo或标题
    QLabel* titleLabel = new QLabel(tr("InsightAT"));
    titleLabel->setAlignment(Qt::AlignCenter);
    titleLabel->setStyleSheet("font-size: 32px; font-weight: bold; color: #333; margin-bottom: 20px;");
    welcomeLayout->addWidget(titleLabel);
    
    // 副标题
    QLabel* subtitleLabel = new QLabel(tr("Photogrammetry Suite"));
    subtitleLabel->setAlignment(Qt::AlignCenter);
    subtitleLabel->setStyleSheet("font-size: 18px; color: #666; margin-bottom: 40px;");
    welcomeLayout->addWidget(subtitleLabel);
    
    // 欢迎信息
    QLabel* welcomeLabel = new QLabel(tr("Welcome! To get started:\n\n"
                                         "• Create a new project (File → New Project)\n"
                                         "• Open an existing project (File → Open Project)\n\n"
                                         "Your workspace will appear on the left."));
    welcomeLabel->setAlignment(Qt::AlignCenter);
    welcomeLabel->setStyleSheet("font-size: 14px; color: #888; line-height: 1.6;");
    welcomeLabel->setWordWrap(true);
    welcomeLayout->addWidget(welcomeLabel);
    
    centerLayout->addWidget(welcomeWidget);
    
    m_splitter->addWidget(m_centerWidget);
    
    // 设置分割器比例
    m_splitter->setStretchFactor(0, 1);  // 左侧
    m_splitter->setStretchFactor(1, 3);  // 中央
    m_splitter->setChildrenCollapsible(false);  // 防止子部件被折叠
    
    layout->addWidget(m_splitter);
    setCentralWidget(centralWidget);
}

void MainWindow::createStatusBar() {
    // 项目名称标签
    m_projectNameLabel = new QLabel(tr("No project loaded"));
    m_projectNameLabel->setMinimumWidth(150);
    statusBar()->addWidget(m_projectNameLabel, 0);
    
    // 修改指示符
    m_modifiedIndicator = new QLabel("");
    m_modifiedIndicator->setMinimumWidth(20);
    statusBar()->addPermanentWidget(m_modifiedIndicator, 0);
    
    // 状态信息标签
    m_statusLabel = new QLabel(tr("Ready"));
    statusBar()->addPermanentWidget(m_statusLabel, 1);
}

void MainWindow::connectSignalsSlots() {
    // ProjectDocument 信号
    connect(m_projectDocument.get(), &ProjectDocument::projectCreated,
            this, &MainWindow::onProjectCreated);
    connect(m_projectDocument.get(), &ProjectDocument::projectOpened,
            this, &MainWindow::onProjectOpened);
    connect(m_projectDocument.get(), &ProjectDocument::projectSaved,
            this, &MainWindow::onProjectSaved);
    connect(m_projectDocument.get(), &ProjectDocument::modificationChanged,
            this, &MainWindow::onModificationChanged);
    
    // 工作区树视图双击信号
    connect(m_workspaceTreeView, &QTreeView::doubleClicked,
            this, &MainWindow::onWorkspaceTreeDoubleClicked);
    
    // 工作区树视图单击选中信号
    connect(m_workspaceTreeView->selectionModel(), &QItemSelectionModel::currentChanged,
            this, &MainWindow::onWorkspaceTreeSelectionChanged);
    
    // 将 ProjectDocument 连接到 WorkspaceTreeModel
    m_workspaceModel->setProjectDocument(m_projectDocument.get());

    // ─── 新增：图像分组 UI 初始化 ───
    // 创建分组管理面板
    m_imageGroupsPanel = new widgets::ImageGroupsManagementPanel();
    m_imageGroupsPanel->SetProjectDocument(m_projectDocument.get());
    
    // 创建分组编辑对话框（单例）
    m_imageGroupDetailDialog = new dialogs::ImageGroupDetailPanel(this);
    m_imageGroupDetailDialog->SetProjectDocument(m_projectDocument.get());
    
    // 连接分组管理面板信号
    connect(m_imageGroupsPanel, &widgets::ImageGroupsManagementPanel::editGroupRequested,
            this, &MainWindow::onEditImageGroup);
    
    // 连接编辑对话框信号
    // ImageGroupDetailPanel 内部会调用 m_projectDocument->notifyImageGroupChanged()
    // 这将触发所有感兴趣组件（树、表格面板等）同步更新
    connect(m_imageGroupDetailDialog, &dialogs::ImageGroupDetailPanel::groupDataChanged,
            this, [this](uint32_t group_id) {
                // 如果将来有需要针对特定分组更新的内容，可以在这里处理
                Q_UNUSED(group_id);
            });
    
    // ─── 新增：AT Task UI 初始化 ───
    // 创建 AT Task 编辑面板（单例）
    m_atTaskPanel = new ATTaskPanel(m_projectDocument.get());
}

void MainWindow::updateWindowTitle() {
    QString title = "InsightAT";
    
    if (m_projectDocument->isProjectLoaded()) {
        title += " - " + QString::fromStdString(m_projectDocument->project().name);
        
        if (m_isModified) {
            title += " *";
        }
    }
    
    setWindowTitle(title);
}

// ─────────────────────────────────────────────────────
// 文件菜单槽函数
// ─────────────────────────────────────────────────────

void MainWindow::onNewProject() {
    // 先保存当前项目（如果有修改）
    if (!maybeSave()) {
        return;  // 用户取消了保存操作
    }
    
    // 创建新项目对话框
    if (!m_newProjectDialog) {
        m_newProjectDialog = std::make_unique<NewProjectDialog>(this);
        
        // 连接对话框信号
        connect(m_newProjectDialog.get(), &NewProjectDialog::projectCreated,
                m_projectDocument.get(), &ProjectDocument::newProject);
    }
    
    // 显示对话框
    if (m_newProjectDialog->exec() == QDialog::Accepted) {
        // 立即选择新项目的保存位置
        QString filePath = QFileDialog::getSaveFileName(this,
            tr("Save New Project"), "",
            tr("InsightAT Projects (*.iat);;All Files (*)"));
        
        if (filePath.isEmpty()) {
            // 用户取消了保存位置选择，回退新建操作
            m_projectDocument->closeProject();
            return;
        }
        
        // 确保扩展名正确
        if (!filePath.endsWith(".iat")) {
            filePath += ".iat";
        }
        
        // 保存新项目
        if (m_projectDocument->saveProjectAs(filePath)) {
            m_currentFilePath = filePath;
            m_statusLabel->setText(tr("New project created and saved: %1").arg(filePath));
            LOG(INFO) << "New project created and saved: " << filePath.toStdString();
        } else {
            QMessageBox::critical(this, tr("Error"), tr("Failed to save new project"));
            m_projectDocument->closeProject();
            return;
        }
        
        // 项目已创建并保存，现在显示坐标系设置对话框
        onSetCoordinateSystem();
        
        // 启用编辑菜单项
        m_actionProjectInfo->setEnabled(true);
        m_actionSetCoordinateSystem->setEnabled(true);
        m_actionAddImageGroup->setEnabled(true);
        m_actionAddCameraRig->setEnabled(true);
        m_actionImportGCPs->setEnabled(true);
        m_actionCreateATTask->setEnabled(true);
        m_actionSaveProject->setEnabled(true);
        m_actionSaveProjectAs->setEnabled(true);
        
        m_statusLabel->setText(tr("New project created"));
        LOG(INFO) << "New project created";
    }
}

void MainWindow::onOpenProject() {
    // 先保存当前项目（如果有修改）
    if (!maybeSave()) {
        return;  // 用户取消了保存操作
    }
    
    // 选择文件
    QString filePath = QFileDialog::getOpenFileName(this,
        tr("Open InsightAT Project"), "",
        tr("InsightAT Projects (*.iat);;All Files (*)"));
    
    if (filePath.isEmpty()) {
        return;
    }
    
    // 打开文件
    if (m_projectDocument->openProject(filePath)) {
        m_currentFilePath = filePath;
        
        // 启用编辑菜单项
        m_actionAddImageGroup->setEnabled(true);
        m_actionAddCameraRig->setEnabled(true);
        m_actionImportGCPs->setEnabled(true);
        m_actionCreateATTask->setEnabled(true);
        m_actionSaveProject->setEnabled(true);
        m_actionSaveProjectAs->setEnabled(true);
        
        m_statusLabel->setText(tr("Project opened: %1").arg(filePath));
        LOG(INFO) << "Project opened: " << filePath.toStdString();
    } else {
        QMessageBox::critical(this, tr("Error"), tr("Failed to open project file"));
        LOG(ERROR) << "Failed to open project: " << filePath.toStdString();
    }
}

void MainWindow::onSaveProject() {
    if (m_currentFilePath.isEmpty()) {
        onSaveProjectAs();
        return;
    }
    
    if (m_projectDocument->saveProjectAs(m_currentFilePath)) {
        m_statusLabel->setText(tr("Project saved"));
        LOG(INFO) << "Project saved";
    } else {
        QMessageBox::critical(this, tr("Error"), tr("Failed to save project"));
        LOG(ERROR) << "Failed to save project";
    }
}

void MainWindow::onSaveProjectAs() {
    QString filePath = QFileDialog::getSaveFileName(this,
        tr("Save InsightAT Project As"), "",
        tr("InsightAT Projects (*.iat);;All Files (*)"));
    
    if (filePath.isEmpty()) {
        return;
    }
    
    // 确保扩展名正确
    if (!filePath.endsWith(".iat")) {
        filePath += ".iat";
    }
    
    if (m_projectDocument->saveProjectAs(filePath)) {
        m_currentFilePath = filePath;
        m_statusLabel->setText(tr("Project saved as: %1").arg(filePath));
        LOG(INFO) << "Project saved as: " << filePath.toStdString();
    } else {
        QMessageBox::critical(this, tr("Error"), tr("Failed to save project"));
        LOG(ERROR) << "Failed to save project as: " << filePath.toStdString();
    }
}

void MainWindow::onExit() {
    close();
}

// ─────────────────────────────────────────────────────
// 编辑菜单槽函数
// ─────────────────────────────────────────────────────

void MainWindow::onProjectInfo() {
    if (!m_projectDocument || !m_projectDocument->isProjectLoaded()) {
        return;
    }

    ProjectInfoDialog dialog(&m_projectDocument->project(), this);
    dialog.exec();
}

void MainWindow::onSetCoordinateSystem() {
    if (!m_projectDocument->isProjectLoaded()) {
        QMessageBox::warning(this, tr("Warning"), 
                           tr("Please create or open a project first"));
        return;
    }

    // 使用新的 CoordinateSystemConfigDialog
    CoordinateSystemConfigDialog dialog(this);
    
    // 加载现有的坐标系配置（如果有）
    dialog.SetCoordinateSystem(m_projectDocument->project().input_coordinate_system);
    
    if (dialog.exec() == QDialog::Accepted) {
        auto coordSys = dialog.GetCoordinateSystem();
        // 更新项目中的坐标系
        m_projectDocument->updateCoordinateSystem(coordSys);
        
        // 更新状态栏
        m_statusLabel->setText(tr("Coordinate system configured successfully"));
        LOG(INFO) << "Coordinate system set: type=" << static_cast<int>(coordSys.type);
    }
}

void MainWindow::onAddImageGroup() {
    if (!m_projectDocument->isProjectLoaded()) {
        QMessageBox::warning(this, tr("Warning"), 
                           tr("Please create or open a project first"));
        return;
    }

    // 显示 Image Groups 管理面板
    onImageGroupsNodeSelected();
}

void MainWindow::onAddCameraRig() {
    // TODO: 显示添加相机Rig对话框
    QMessageBox::information(this, tr("TODO"), tr("Camera Rig configuration - not yet implemented"));
}

void MainWindow::onImportGCPs() {
    // TODO: 显示GCP导入对话框和文件选择
    QMessageBox::information(this, tr("TODO"), tr("GCP import - not yet implemented"));
}

void MainWindow::onCreateATTask() {
    if (!m_projectDocument->isProjectLoaded()) {
        QMessageBox::warning(this, tr("Warning"), 
                           tr("Please create or open a project first"));
        return;
    }
    
    // 创建"新建 AT Task"对话框（使用动态生成的下一个任务名称）
    std::string nextTaskName = m_projectDocument->generateNextATTaskName();
    NewATTaskDialog dialog(m_projectDocument.get(), QString::fromStdString(nextTaskName), this);
    
    if (dialog.exec() == QDialog::Accepted) {
        std::string taskName = dialog.getTaskName();
        uint32_t parentTaskIndex = dialog.getParentTaskIndex();
        
        // 创建新任务
        std::string taskId = m_projectDocument->createATTask(QString::fromStdString(taskName));
        
        if (!taskId.empty()) {
            // 如果指定了父任务，设置继承关系
            if (parentTaskIndex != static_cast<uint32_t>(-1)) {
                auto* task = m_projectDocument->getATTaskById(taskId);
                if (task) {
                    task->initialization = insight::database::ATTask::Initialization();
                    task->initialization->prev_task_id = parentTaskIndex;
                    m_projectDocument->updateATTask(taskId, *task);
                }
            }
            
            // 刷新树形视图
            if (m_workspaceModel) {
                m_workspaceModel->refreshTree();
            }
            
            m_statusLabel->setText(tr("AT Task created: %1").arg(QString::fromStdString(taskName)));
            LOG(INFO) << "AT Task created: " << taskName << " (ID: " << taskId << ")";
        } else {
            QMessageBox::critical(this, tr("Error"), tr("Failed to create AT Task"));
            LOG(ERROR) << "Failed to create AT Task";
        }
    }
}

void MainWindow::onToggleWorkspacePanel() {
    m_workspaceTreeView->setVisible(m_actionToggleWorkspacePanel->isChecked());
}

void MainWindow::onTogglePropertyPanel() {
    // TODO: 实现属性面板的切换
}

// ─────────────────────────────────────────────────────
// 帮助菜单槽函数
// ─────────────────────────────────────────────────────

void MainWindow::onAbout() {
    QMessageBox::about(this, tr("About InsightAT"),
        tr("InsightAT - Photogrammetry Suite\n"
           "Version 1.0.0\n\n"
           "A comprehensive photogrammetry processing application.\n\n"
           "© 2026 InsightAT Contributors"));
}

void MainWindow::onAboutQt() {
    QApplication::aboutQt();
}

// ─────────────────────────────────────────────────────
// ProjectDocument 信号槽
// ─────────────────────────────────────────────────────

void MainWindow::onProjectCreated() {
    updateWindowTitle();
    m_projectNameLabel->setText(QString::fromStdString(m_projectDocument->project().name));
    m_statusLabel->setText(tr("Project created"));
}

void MainWindow::onProjectOpened() {
    updateWindowTitle();
    m_projectNameLabel->setText(QString::fromStdString(m_projectDocument->project().name));
    m_statusLabel->setText(tr("Project opened"));
}

void MainWindow::onProjectSaved() {
    updateWindowTitle();
    m_statusLabel->setText(tr("Project saved"));
}

void MainWindow::onModificationChanged(bool modified) {
    m_isModified = modified;
    updateWindowTitle();
    
    if (modified) {
        m_modifiedIndicator->setText("*");
    } else {
        m_modifiedIndicator->setText("");
    }
}

// ─────────────────────────────────────────────────────
// 窗口事件处理
// ─────────────────────────────────────────────────────

void MainWindow::showEvent(QShowEvent* event) {
    QMainWindow::showEvent(event);
    
    // 在窗口首次显示时设置splitter的初始大小
    static bool firstShow = true;
    if (firstShow) {
        firstShow = false;
        
        // 计算合理的初始大小
        int totalWidth = width();
        int leftWidth = 250;  // workspace panel宽度
        int rightWidth = totalWidth - leftWidth;
        
        // 确保右侧有足够的宽度
        if (rightWidth < 400) {
            rightWidth = 400;
            leftWidth = totalWidth - rightWidth;
        }
        
        m_splitter->setSizes(QList<int>() << leftWidth << rightWidth);
    }
}

bool MainWindow::maybeSave() {
    if (!m_projectDocument->isModified()) {
        return true;  // 没有修改，继续
    }
    
    // 如果有保存路径，直接保存
    if (!m_currentFilePath.isEmpty()) {
        if (m_projectDocument->saveProjectAs(m_currentFilePath)) {
            m_statusLabel->setText(tr("Project saved"));
            LOG(INFO) << "Project auto-saved before creating new project";
            return true;
        } else {
            QMessageBox::critical(this, tr("Error"), tr("Failed to save project"));
            return false;
        }
    }
    
    // 没有保存路径，询问用户
    QMessageBox::StandardButton reply = QMessageBox::question(this,
        tr("Unsaved Changes"),
        tr("The project has not been saved. Do you want to save it?"),
        QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
    
    if (reply == QMessageBox::Save) {
        onSaveProjectAs();
        return !m_projectDocument->isModified();  // 保存成功才继续
    } else if (reply == QMessageBox::Discard) {
        return true;  // 放弃修改，继续
    } else {
        return false;  // 取消操作
    }
}

void MainWindow::closeEvent(QCloseEvent* event) {
    if (maybeSave()) {
        event->accept();
    } else {
        event->ignore();
    }
}

// ─────────────────────────────────────────────────────
// 设置管理
// ─────────────────────────────────────────────────────

void MainWindow::loadSettings() {
    QSettings settings("InsightAT", "InsightAT");
    
    // 恢复窗口大小和位置
    restoreGeometry(settings.value("mainWindow/geometry").toByteArray());
    restoreState(settings.value("mainWindow/windowState").toByteArray());
    
    // 恢复分割器大小
    if (settings.contains("mainWindow/splitterSizes")) {
        m_splitter->restoreState(settings.value("mainWindow/splitterSizes").toByteArray());
    }
}

void MainWindow::onWorkspaceTreeDoubleClicked(const QModelIndex& index) {
    if (!m_workspaceModel || !m_projectDocument || !m_projectDocument->isProjectLoaded()) {
        return;
    }

    // 获取树节点数据
    QVariant data = m_workspaceModel->data(index, Qt::DisplayRole);
    QString nodeName = data.toString();

    // 检查是否双击了 "Project Info" 节点
    if (nodeName == "Project Info") {
        onProjectInfo();
    }
    // 检查是否双击了 "Image Groups" 节点
    else if (nodeName == "Image Groups") {
        onImageGroupsNodeSelected();
    }
}

void MainWindow::onWorkspaceTreeSelectionChanged(const QModelIndex& index) {
    if (!m_workspaceModel || !m_projectDocument || !m_projectDocument->isProjectLoaded()) {
        return;
    }
    
    // 获取树节点信息
    WorkspaceTreeModel::TreeNode* node = m_workspaceModel->getNode(index);
    if (!node) {
        return;
    }
    
    // 根据节点类型显示对应的编辑面板
    if (node->type == WorkspaceTreeModel::ATTaskNode) {
        // 隐藏其他面板
        if (m_imageGroupsPanel) {
            m_imageGroupsPanel->hide();
        }
        m_centerWidget->hide();
        
        // 显示 ATTaskPanel
        if (m_atTaskPanel) {
            // 加载选中的任务
            m_atTaskPanel->loadTask(node->taskId);
            
            // 将右侧内容面板替换为 ATTaskPanel
            int panelIndex = m_splitter->indexOf(m_centerWidget);
            if (panelIndex >= 0) {
                m_splitter->replaceWidget(panelIndex, m_atTaskPanel);
            } else {
                m_splitter->addWidget(m_atTaskPanel);
            }
            m_atTaskPanel->show();
            
            m_statusLabel->setText(tr("AT Task: ") + QString::fromStdString(node->taskId).left(8));
        }
    }
    // 单击 ImageGroupNode 时显示 ImageGroupsManagementPanel
    else if (node->type == WorkspaceTreeModel::ImageGroupNode) {
        onImageGroupsNodeSelected();
    }
    // 其他节点类型可在此后续扩展
}

// ─────────────────────────────────────────────────────
// 新增：图像分组 UI 槽函数
// ─────────────────────────────────────────────────────

void MainWindow::onImageGroupsNodeSelected() {
    // 隐藏其他面板
    if (m_atTaskPanel) {
        m_atTaskPanel->hide();
    }
    m_centerWidget->hide();
    
    // 将右侧内容面板替换为 ImageGroupsManagementPanel
    if (m_imageGroupsPanel) {
        // 查找 m_centerWidget 在分割器中的位置
        int index = m_splitter->indexOf(m_centerWidget);
        if (index >= 0) {
            m_splitter->replaceWidget(index, m_imageGroupsPanel);
        } else {
            m_splitter->addWidget(m_imageGroupsPanel);
        }
        m_imageGroupsPanel->show();
        m_imageGroupsPanel->RefreshGroupList();
        
        m_statusLabel->setText(tr("Image Groups Management"));
    }
}

void MainWindow::onEditImageGroup(database::ImageGroup* group) {
    if (m_imageGroupDetailDialog && group) {
        m_imageGroupDetailDialog->LoadGroup(group);
    }
}

void MainWindow::saveSettings() {
    QSettings settings("InsightAT", "InsightAT");
    
    // 保存窗口大小和位置
    settings.setValue("mainWindow/geometry", saveGeometry());
    settings.setValue("mainWindow/windowState", saveState());
    
    // 保存分割器大小
    settings.setValue("mainWindow/splitterSizes", m_splitter->saveState());
}

}  // namespace ui
}  // namespace insight
