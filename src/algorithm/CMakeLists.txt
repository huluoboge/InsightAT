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
    io/idc_reader.h
    io/idc_reader.cpp
    
    # Modules - Feature Extraction
    modules/extraction/sift_gpu_extractor.h
    modules/extraction/sift_gpu_extractor.cpp
    modules/extraction/feature_distribution.h
    modules/extraction/feature_distribution.cpp
    modules/extraction/KeyPointsNode.h
    modules/extraction/KeyPointsNode.cpp
    
    # Modules - Feature Matching
    modules/matching/sift_matcher.h
    modules/matching/sift_matcher.cpp
    
    # Modules - Image Retrieval
    modules/retrieval/retrieval_types.h
    modules/retrieval/retrieval_types.cpp
    modules/retrieval/spatial_retrieval.h
    modules/retrieval/spatial_retrieval.cpp
    modules/retrieval/vlad_encoding.h
    modules/retrieval/vlad_encoding.cpp
    modules/retrieval/vlad_retrieval.h
    modules/retrieval/vlad_retrieval.cpp
    modules/retrieval/pca_whitening.h
    modules/retrieval/pca_whitening.cpp
    modules/retrieval/vocab_tree_retrieval.h
    modules/retrieval/vocab_tree_retrieval.cpp
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
        ${CMAKE_SOURCE_DIR}/third_party/DBow3/src
)

target_link_libraries(InsightATAlgorithm
    PUBLIC
        glog::glog
        InsightATDatabase
        ${OpenCV_LIBS}
        DBoW3
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

# isat_match - Feature matching tool
add_executable(isat_match tools/isat_match.cpp)

target_link_libraries(isat_match
    PRIVATE
        InsightATAlgorithm
        glog::glog
        sift_gpu
        ${GLEW_LIBRARY}
        ${OPENGL_LIBRARIES}
)

target_include_directories(isat_match
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party
)

set_property(TARGET isat_match PROPERTY FOLDER InsightAT/Tools)

# isat_retrieve - Image pair retrieval tool
add_executable(isat_retrieve tools/isat_retrieve.cpp)

target_link_libraries(isat_retrieve
    PRIVATE
        InsightATAlgorithm
        glog::glog
        stlplus3
)

target_include_directories(isat_retrieve
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party
)

set_property(TARGET isat_retrieve PROPERTY FOLDER InsightAT/Tools)

# isat_train_vlad - VLAD codebook training tool
add_executable(isat_train_vlad tools/isat_train_vlad.cpp)

target_link_libraries(isat_train_vlad
    PRIVATE
        InsightATAlgorithm
        glog::glog
        ${OpenCV_LIBS}
)

target_include_directories(isat_train_vlad
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party
)

set_property(TARGET isat_train_vlad PROPERTY FOLDER InsightAT/Tools)

# isat_train_vocab - DBoW3 vocabulary tree training tool
add_executable(isat_train_vocab tools/isat_train_vocab.cpp)

target_link_libraries(isat_train_vocab
    PRIVATE
        InsightATAlgorithm
        glog::glog
        ${OpenCV_LIBS}
        DBoW3
)

target_include_directories(isat_train_vocab
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party
)

set_property(TARGET isat_train_vocab PROPERTY FOLDER InsightAT/Tools)

# export_project_json - Export Project to CLI-compatible JSON format
add_executable(export_project_json tools/export_project_json.cpp)

target_link_libraries(export_project_json
    PRIVATE
        InsightATDatabase
        glog::glog
)

target_include_directories(export_project_json
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party
)

set_property(TARGET export_project_json PROPERTY FOLDER InsightAT/Tools)


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
