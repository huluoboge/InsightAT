cmake_minimum_required(VERSION 3.16)

project(InsightATUI)

# ─────────────────────────────────────────────────────────────
# 设置Qt编译选项
# ─────────────────────────────────────────────────────────────

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# ─────────────────────────────────────────────────────────────
# 查找依赖
# ─────────────────────────────────────────────────────────────

find_package(Qt5 COMPONENTS
    Core
    Gui
    Widgets
    REQUIRED
)

find_package(glog REQUIRED)

# ─────────────────────────────────────────────────────────────
# 源文件
# ─────────────────────────────────────────────────────────────

set(UI_SOURCES
    # UI System Config
    UISystemConfig.h
    UISystemConfig.cpp
    
    # Models
    models/ProjectDocument.h
    models/ProjectDocument.cpp
    models/WorkspaceTreeModel.h
    models/WorkspaceTreeModel.cpp
    
    # Main Window
    MainWindow.h
    MainWindow.cpp
    
    # Dialogs
    dialogs/NewProjectDialog.h
    dialogs/NewProjectDialog.cpp
    dialogs/ImageGroupDialog.h
    dialogs/ImageGroupDialog.cpp
    dialogs/ProjectInfoDialog.h
    dialogs/ProjectInfoDialog.cpp
    dialogs/CoordinateSystemConfigDialog.h
    dialogs/CoordinateSystemConfigDialog.cpp
    dialogs/ImageGroupDetailPanel.h
    dialogs/ImageGroupDetailPanel.cpp
    dialogs/ImageEditorDialog.h
    dialogs/ImageEditorDialog.cpp
    dialogs/ImagePreviewDialog.h
    dialogs/ImagePreviewDialog.cpp
    dialogs/NewATTaskDialog.h
    dialogs/NewATTaskDialog.cpp
    
    # Widgets
    widgets/SpatialReferenceDialog.h
    widgets/SpatialReferenceDialog.cpp
    widgets/SpatialReferenceTool.ui
    widgets/CameraModelWidget.h
    widgets/CameraModelWidget.cpp
    widgets/CoordinateSystemConfigWidget.h
    widgets/CoordinateSystemConfigWidget.cpp
    widgets/CameraParameterEditorWidget.h
    widgets/CameraParameterEditorWidget.cpp
    widgets/ImageGroupsManagementPanel.h
    widgets/ImageGroupsManagementPanel.cpp
    
    # GPS/GNSS Import Wizards
    widgets/GPSPointsWizardDialog.h
    widgets/GPSPointsWizardDialog.cpp
    widgets/GPSPointsWizardDialog.ui
    widgets/GPSPointsWizardModel.h
    widgets/GPSPointsWizardModel.cpp
    widgets/GPSPointsWizardDelegate.h
    widgets/GPSPointsWizardDelegate.cpp
    widgets/GNSSMeasurementImportDialog.h
    widgets/GNSSMeasurementImportDialog.cpp
    
    # Panels
    panels/ATTaskPanel.h
    panels/ATTaskPanel.cpp
)

set(UI_HEADERS
    models/ProjectDocument.h
    models/WorkspaceTreeModel.h
    MainWindow.h
    dialogs/NewProjectDialog.h
    dialogs/ImageGroupDialog.h
    widgets/SpatialReferenceDialog.h
    widgets/CameraModelWidget.h
)

# ─────────────────────────────────────────────────────────────
# 创建库目标
# ─────────────────────────────────────────────────────────────

add_library(InsightATUI STATIC ${UI_SOURCES})

# ─────────────────────────────────────────────────────────────
# 链接依赖
# ─────────────────────────────────────────────────────────────

target_include_directories(InsightATUI
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        ${Qt5_INCLUDE_DIRS}
)

target_link_libraries(InsightATUI
    PUBLIC
        Qt5::Core
        Qt5::Gui
        Qt5::Widgets
        glog::glog
        InsightATDatabase
)

# ─────────────────────────────────────────────────────────────
# 编译选项
# ─────────────────────────────────────────────────────────────

if(MSVC)
    target_compile_options(InsightATUI PRIVATE /W4)
else()
    target_compile_options(InsightATUI PRIVATE -Wall -Wextra -Wpedantic)
endif()

set_property(TARGET InsightATUI PROPERTY FOLDER InsightAT/UI)
