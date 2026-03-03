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
    system_config.h
    system_config.cpp
    models/project_document.h
    models/project_document.cpp
    models/workspace_tree_model.h
    models/workspace_tree_model.cpp
    main_window.h
    main_window.cpp
    dialogs/new_project_dialog.h
    dialogs/new_project_dialog.cpp
    dialogs/project_info_dialog.h
    dialogs/project_info_dialog.cpp
    dialogs/coordinate_system_config_dialog.h
    dialogs/coordinate_system_config_dialog.cpp
    dialogs/image_group_detail_panel.h
    dialogs/image_group_detail_panel.cpp
    dialogs/image_editor_dialog.h
    dialogs/image_editor_dialog.cpp
    dialogs/image_preview_dialog.h
    dialogs/image_preview_dialog.cpp
    dialogs/new_at_task_dialog.h
    dialogs/new_at_task_dialog.cpp
    utils/coordinates.h
    utils/coordinates.cpp
    widgets/spatial_reference_dialog.h
    widgets/spatial_reference_dialog.cpp
    widgets/spatial_reference_tool.ui
    widgets/camera_model_widget.h
    widgets/camera_model_widget.cpp
    widgets/coordinate_system_config_widget.h
    widgets/coordinate_system_config_widget.cpp
    widgets/camera_parameter_editor_widget.h
    widgets/camera_parameter_editor_widget.cpp
    widgets/image_groups_management_panel.h
    widgets/image_groups_management_panel.cpp
    widgets/gps_points_wizard_dialog.h
    widgets/gps_points_wizard_dialog.cpp
    widgets/gps_points_wizard_dialog.ui
    widgets/gps_points_wizard_model.h
    widgets/gps_points_wizard_model.cpp
    widgets/gps_points_wizard_delegate.h
    widgets/gps_points_wizard_delegate.cpp
    widgets/gnss_measurement_import_dialog.h
    widgets/gnss_measurement_import_dialog.cpp
    panels/at_task_panel.h
    panels/at_task_panel.cpp
)

set(UI_HEADERS
    models/project_document.h
    models/workspace_tree_model.h
    main_window.h
    dialogs/new_project_dialog.h
    widgets/spatial_reference_dialog.h
    widgets/camera_model_widget.h
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
        ${CMAKE_SOURCE_DIR}/third_party
        ${Qt5_INCLUDE_DIRS}
        ${GDAL_INCLUDE_DIRS}
)

target_link_libraries(InsightATUI
    PUBLIC
        Qt5::Core
        Qt5::Gui
        Qt5::Widgets
        glog::glog
        InsightATDatabase
        util
        stlplus3
        ${GDAL_LIBRARIES}
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
