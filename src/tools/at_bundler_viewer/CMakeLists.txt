# 独立工具：Bundler 结果可视化（不依赖 InsightATDatabase / 主工程 UI）

find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets OpenGL)

set(CMAKE_AUTOMOC ON)

add_executable(at_bundler_viewer
  main.cpp
  bundler_viewer_window.cpp
  bundler_viewer_window.h
  egl_render_widget.cpp
  egl_render_widget.h
  feature_overlay_layer.cpp
  feature_overlay_layer.h
  observation_image_view.cpp
  observation_image_view.h
  observation_list_widget.cpp
  observation_list_widget.h
)

target_include_directories(at_bundler_viewer
  PRIVATE
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(at_bundler_viewer
  PRIVATE
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    Qt5::OpenGL
    render
    glog::glog
)

set_property(TARGET at_bundler_viewer PROPERTY FOLDER InsightAT/tools)
