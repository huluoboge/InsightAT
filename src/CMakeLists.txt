
include_directories(${EIGEN3_INCLUDE_DIR})
add_subdirectory(database)
add_subdirectory(util)

# Legacy Qt UI (optional). Default builds are CLI-only; Node (simple-gui) is the
# intended product UI going forward.
if(INSIGHTAT_BUILD_QT_UI)
    add_subdirectory(ui)
    add_subdirectory(render)
else()
    message(STATUS "src: INSIGHTAT_BUILD_QT_UI=OFF, skipping ui/render")
endif()

if(INSIGHTAT_BUILD_GUI_ONLY)
    message(STATUS "src: GUI-only mode, skipping algorithm/cli targets")
else()
    add_subdirectory(algorithm)
    add_subdirectory(cli)
endif()

# ─────────────────────────────────────────────────────────────
# Qt5 targets (only when INSIGHTAT_BUILD_QT_UI=ON)
# ─────────────────────────────────────────────────────────────
if(INSIGHTAT_BUILD_QT_UI)
    find_package(Qt5Widgets REQUIRED)
    find_package(Qt5Core REQUIRED)
    find_package(Qt5Gui REQUIRED)
    find_package(Qt5OpenGL REQUIRED)

    if(NOT INSIGHTAT_BUILD_GUI_ONLY)
        add_subdirectory(tools/at_bundler_viewer)
    endif()

    set(CMAKE_AUTOMOC ON)
    set(CMAKE_AUTORCC ON)

    add_executable(InsightAT
        main.cpp
    )

    target_include_directories(InsightAT
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}
    )

    target_link_libraries(InsightAT
        PRIVATE
            Qt5::Core
            Qt5::Gui
            Qt5::Widgets
            InsightATUI
            InsightATDatabase
            glog::glog
    )

    set_property(TARGET InsightAT PROPERTY FOLDER InsightAT)
    message(STATUS "Building InsightAT (Qt MainWindow)")

    add_custom_command(TARGET InsightAT POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_SOURCE_DIR}/data/config"
            "${CMAKE_BINARY_DIR}/config"
        COMMENT "Copying coordinate system databases to build directory"
    )
else()
    message(STATUS "src: CLI-only build (isat_* tools); Qt UI disabled")
endif()
