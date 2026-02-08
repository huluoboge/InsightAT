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
#include "widgets/SpatialReferenceDialog.h"
#include "UISystemConfig.h"
#include "../Common/Coordinates.h"

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
    config.setConfigPath("./config");
    if (!config.loadCoordinateDatabases()) {
        LOG(WARNING) << "Failed to load from ./config, trying ../data/config";
        config.setConfigPath("../data/config");
        if (!config.loadCoordinateDatabases()) {
            LOG(ERROR) << "Failed to load coordinate databases from both paths";
        }
    }
    
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
    m_splitter->addWidget(m_workspaceTreeView);
    
    // 中央：内容区域 (暂时放一个占位符，后续放3D视图)
    m_centerWidget = new QWidget();
    QVBoxLayout* centerLayout = new QVBoxLayout(m_centerWidget);
    
    // 欢迎标签
    QLabel* welcomeLabel = new QLabel(tr("Welcome to InsightAT!\n\n"
                                         "Create a new project or open an existing one to get started."));
    welcomeLabel->setAlignment(Qt::AlignCenter);
    welcomeLabel->setStyleSheet("font-size: 14px; color: #666;");
    centerLayout->addWidget(welcomeLabel);
    
    m_splitter->addWidget(m_centerWidget);
    
    // 设置分割器比例
    m_splitter->setStretchFactor(0, 1);  // 左侧
    m_splitter->setStretchFactor(1, 3);  // 中央
    
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
    
    // 将 ProjectDocument 连接到 WorkspaceTreeModel
    m_workspaceModel->setProjectDocument(m_projectDocument.get());
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
    // 创建新项目对话框
    if (!m_newProjectDialog) {
        m_newProjectDialog = std::make_unique<NewProjectDialog>(this);
        
        // 连接对话框信号
        connect(m_newProjectDialog.get(), &NewProjectDialog::projectCreated,
                m_projectDocument.get(), &ProjectDocument::newProject);
    }
    
    // 显示对话框
    if (m_newProjectDialog->exec() == QDialog::Accepted) {
        // 项目已创建，现在显示坐标系设置对话框
        onSetCoordinateSystem();
        
        // 启用编辑菜单项
        m_actionProjectInfo->setEnabled(true);
        m_actionSetCoordinateSystem->setEnabled(true);
        m_actionAddImageGroup->setEnabled(true);
        m_actionAddCameraRig->setEnabled(true);
        m_actionImportGCPs->setEnabled(true);
        m_actionSaveProject->setEnabled(true);
        m_actionSaveProjectAs->setEnabled(true);
        
        m_statusLabel->setText(tr("New project created"));
        LOG(INFO) << "New project created";
    }
}

void MainWindow::onOpenProject() {
    // 选择文件
    QString filePath = QFileDialog::getOpenFileName(this,
        tr("Open InsightAT Project"), "",
        tr("InsightAT Projects (*.ipt);;All Files (*)"));
    
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
        tr("InsightAT Projects (*.ipt);;All Files (*)"));
    
    if (filePath.isEmpty()) {
        return;
    }
    
    // 确保扩展名正确
    if (!filePath.endsWith(".ipt")) {
        filePath += ".ipt";
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
    // 使用新的 SpatialReferenceDialog，配合 UISystemConfig
    SpatialReferenceDialog dialog(this);
    
    if (dialog.exec() == QDialog::Accepted) {
        Coordinate coord = dialog.SelectCoordinate();
        
        if (!coord.CoordinateName.empty()) {
            // 设置坐标系到项目
            insight::database::CoordinateSystem cs;
            
            // 优先使用 EPSG，其次使用 WKT
            if (!coord.EPSGName.empty()) {
                // 从 EPSG 名称提取数字（例如 "EPSG:4326" -> 4326）
                std::string epsgStr = coord.EPSGName;
                size_t colonPos = epsgStr.find(':');
                if (colonPos != std::string::npos) {
                    try {
                        int epsg = std::stoi(epsgStr.substr(colonPos + 1));
                        cs.type = insight::database::CoordinateSystem::Type::kEPSG;
                        cs.definition = "EPSG:" + std::to_string(epsg);
                    } catch (...) {
                        cs.type = insight::database::CoordinateSystem::Type::kWKT;
                        cs.definition = coord.WKT;
                    }
                } else {
                    cs.type = insight::database::CoordinateSystem::Type::kWKT;
                    cs.definition = coord.WKT;
                }
            } else {
                cs.type = insight::database::CoordinateSystem::Type::kWKT;
                cs.definition = coord.WKT;
            }
            
            m_projectDocument->project().input_coordinate_system = cs;
            
            m_statusLabel->setText(tr("Coordinate system set: %1")
                                 .arg(QString::fromStdString(coord.CoordinateName)));
            LOG(INFO) << "Coordinate system set: " << coord.CoordinateName;
        }
    }
}

void MainWindow::onAddImageGroup() {
    if (!m_projectDocument->isProjectLoaded()) {
        QMessageBox::warning(this, tr("Warning"), 
                           tr("Please create or open a project first"));
        return;
    }
    
    // 创建图像分组对话框
    if (!m_imageGroupDialog) {
        m_imageGroupDialog = std::make_unique<ImageGroupDialog>(
            &m_projectDocument->project(), this);
    }
    
    if (m_imageGroupDialog->exec() == QDialog::Accepted) {
        const auto& group = m_imageGroupDialog->getImageGroup();
        
        // 添加分组到项目
        m_projectDocument->project().image_groups.push_back(group);
        
        m_statusLabel->setText(tr("Image group added: %1")
                             .arg(QString::fromStdString(group.group_name)));
        LOG(INFO) << "Image group added: " << group.group_name;
    }
}

void MainWindow::onAddCameraRig() {
    // TODO: 显示添加相机Rig对话框
    QMessageBox::information(this, tr("TODO"), tr("Camera Rig configuration - not yet implemented"));
}

void MainWindow::onImportGCPs() {
    // TODO: 显示GCP导入对话框和文件选择
    QMessageBox::information(this, tr("TODO"), tr("GCP import - not yet implemented"));
}

// ─────────────────────────────────────────────────────
// 视图菜单槽函数
// ─────────────────────────────────────────────────────

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

void MainWindow::closeEvent(QCloseEvent* event) {
    if (m_isModified) {
        QMessageBox::StandardButton reply = QMessageBox::question(this,
            tr("Unsaved Changes"),
            tr("The project has been modified. Do you want to save changes?"),
            QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
        
        if (reply == QMessageBox::Save) {
            onSaveProject();
            event->accept();
        } else if (reply == QMessageBox::Discard) {
            event->accept();
        } else {
            event->ignore();
        }
    } else {
        event->accept();
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
