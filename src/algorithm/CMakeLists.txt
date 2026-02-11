cmake_minimum_required(VERSION 3.16)

project(InsightATAlgorithm)

# ─────────────────────────────────────────────────────────────
# 设置编译选项
# ─────────────────────────────────────────────────────────────

# ─────────────────────────────────────────────────────────────
# 查找依赖
# ─────────────────────────────────────────────────────────────

find_package(glog REQUIRED)

# ─────────────────────────────────────────────────────────────
# 源文件
# ─────────────────────────────────────────────────────────────

set(ALGORITHM_SOURCES
    # Export
    export/ColmapExporter.h
    export/ColmapExporter.cpp
    
    # I/O
    io/idc_writer.h
    io/idc_writer.cpp
    
    # Modules - Feature Extraction
    modules/extraction/sift_gpu_extractor.h
    modules/extraction/sift_gpu_extractor.cpp
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
        ${CMAKE_SOURCE_DIR}/third_party
)

target_link_libraries(InsightATAlgorithm
    PUBLIC
        glog::glog
        InsightATDatabase
        ${OpenCV_LIBS}
        sift_gpu
        ${GLEW_LIBRARY}
        ${OPENGL_LIBRARIES}
)

# ─────────────────────────────────────────────────────────────
# CLI Tools
# ─────────────────────────────────────────────────────────────

# isat_extract - Feature extraction tool
add_executable(isat_extract tools/isat_extract.cpp)

target_link_libraries(isat_extract
    PRIVATE
        InsightATAlgorithm
        glog::glog
        ${OpenCV_LIBS}
        sift_gpu
        ${GLEW_LIBRARY}
        ${OPENGL_LIBRARIES}
)

target_include_directories(isat_extract
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party
)

set_property(TARGET isat_extract PROPERTY FOLDER InsightAT/Tools)

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
