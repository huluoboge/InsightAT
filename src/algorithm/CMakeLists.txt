cmake_minimum_required(VERSION 3.16)

project(InsightATAlgorithm)

# ─────────────────────────────────────────────────────────────
# 设置Qt编译选项
# ─────────────────────────────────────────────────────────────

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

# ─────────────────────────────────────────────────────────────
# 查找依赖
# ─────────────────────────────────────────────────────────────

find_package(Qt5 COMPONENTS Core REQUIRED)
find_package(glog REQUIRED)

# ─────────────────────────────────────────────────────────────
# 源文件
# ─────────────────────────────────────────────────────────────

set(ALGORITHM_SOURCES
    # Export
    export/ColmapExporter.h
    export/ColmapExporter.cpp
)

# ─────────────────────────────────────────────────────────────
# 创建库目标
# ─────────────────────────────────────────────────────────────

add_library(InsightATAlgorithm STATIC ${ALGORITHM_SOURCES})

# ─────────────────────────────────────────────────────────────
# 链接依赖
# ─────────────────────────────────────────────────────────────

target_include_directories(InsightATAlgorithm
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        ${Qt5_INCLUDE_DIRS}
)

target_link_libraries(InsightATAlgorithm
    PUBLIC
        Qt5::Core
        glog::glog
        InsightATDatabase
)

# ─────────────────────────────────────────────────────────────
# CameraEstimator 独立进程工具
# ─────────────────────────────────────────────────────────────

add_executable(CameraEstimator CameraEstimator.cpp)

target_link_libraries(CameraEstimator
    PRIVATE
        InsightATAlgorithm
        Common
        InsightATDatabase
        ImageIO
        glog::glog
        ${GDAL_LIBRARIES}
)

target_include_directories(CameraEstimator
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party
)


# ─────────────────────────────────────────────────────────────
# 编译选项
# ─────────────────────────────────────────────────────────────

if(MSVC)
    target_compile_options(InsightATAlgorithm PRIVATE /W4)
else()
    target_compile_options(InsightATAlgorithm PRIVATE -Wall -Wextra -Wpedantic)
endif()

set_property(TARGET InsightATAlgorithm PROPERTY FOLDER InsightAT/Algorithm)
