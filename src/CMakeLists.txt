

add_subdirectory(database)
add_subdirectory(Common)
add_subdirectory(Gui)
add_subdirectory(render)
add_subdirectory(ui)
add_subdirectory(algorithm)

# ─────────────────────────────────────────────────────────────
# 查找 Qt5 包
# ─────────────────────────────────────────────────────────────
find_package(Qt5Widgets QUIET)
find_package(Qt5Core QUIET)
find_package(Qt5Gui QUIET)
find_package(Qt5OpenGL QUIET)

# ─────────────────────────────────────────────────────────────
# 新的 InsightAT 可执行文件（使用新的 MainWindow）
# ─────────────────────────────────────────────────────────────
if(Qt5Widgets_FOUND AND Qt5Core_FOUND AND Qt5Gui_FOUND)
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_AUTORCC ON)

    add_executable(InsightAT_New
        main.cpp
    )

    target_include_directories(InsightAT_New
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}
    )

    target_link_libraries(InsightAT_New
        PRIVATE
            Qt5::Core
            Qt5::Gui
            Qt5::Widgets
            InsightATUI
            InsightATDatabase
            Common
            render
            glog::glog
    )

    set_property(TARGET InsightAT_New PROPERTY FOLDER InsightAT)
    message(STATUS "Building InsightAT_New with new MainWindow")
    
    # ==============================================================================
    # 复制数据文件到构建目录
    # ==============================================================================
    # 在构建时复制配置数据文件（坐标系数据库等）到 build 目录
    add_custom_command(TARGET InsightAT_New POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_SOURCE_DIR}/data/config"
            "${CMAKE_BINARY_DIR}/config"
        COMMENT "Copying coordinate system databases to build directory"
    )
else()
    message(STATUS "Qt5 not found, skipping InsightAT_New")
endif()