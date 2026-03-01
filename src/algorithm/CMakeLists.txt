cmake_minimum_required(VERSION 3.16)

project(InsightATAlgorithm)

# ─────────────────────────────────────────────────────────────
# 设置编译选项
# ─────────────────────────────────────────────────────────────

# ─────────────────────────────────────────────────────────────
# 查找依赖
# ─────────────────────────────────────────────────────────────

find_package(glog REQUIRED)

# Optional CUDA for PCA training (cuBLAS + cuSOLVER)
set(INSIGHTAT_USE_CUDA_PCA FALSE)
set(CUDA_PCA_LIBS "")
find_package(CUDAToolkit QUIET)
if(CUDAToolkit_FOUND)
  enable_language(CUDA)
  # Default: multiple archs so one build runs on Pascal→Ada (60=Pascal, 70=Volta, 75=Turing, 80=Ampere, 86=RTX30, 89=RTX40).
  # Override with -DCMAKE_CUDA_ARCHITECTURES=native to build only for current GPU, or e.g. "86" for RTX 30xx.
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES OR CMAKE_CUDA_ARCHITECTURES STREQUAL "")
    set(CMAKE_CUDA_ARCHITECTURES "60;70;75;80;86;89" CACHE STRING "CUDA architectures (semicolon-separated)")
  endif()
  find_library(CUDA_PCA_CUBLAS cublas HINTS ${CUDAToolkit_LIBRARY_DIR} ${CUDAToolkit_ROOT})
  find_library(CUDA_PCA_CUSOLVER cusolver HINTS ${CUDAToolkit_LIBRARY_DIR} ${CUDAToolkit_ROOT})
  if(CUDA_PCA_CUBLAS AND CUDA_PCA_CUSOLVER)
    set(INSIGHTAT_USE_CUDA_PCA TRUE)
    set(CUDA_PCA_LIBS ${CUDA_PCA_CUBLAS} ${CUDA_PCA_CUSOLVER})
    message(STATUS "InsightATAlgorithm: CUDA PCA training enabled (cuBLAS + cuSOLVER)")
  else()
    message(STATUS "InsightATAlgorithm: CUDA found but cublas/cusolver not found, PCA on CPU")
  endif()
else()
    message(STATUS "InsightATAlgorithm: CUDA not found, PCA on CPU")
endif()

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
    io/exif/exif.h
    io/exif/exif.cpp
    io/exif/exif_IO.hpp
    io/exif/exif_IO_EasyExif.hpp
    
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

# Add SuperPoint sources if available
if(SUPERPOINT_AVAILABLE)
    list(APPEND ALGORITHM_SOURCES
        modules/extraction/superpoint_extractor.h
        modules/extraction/superpoint_extractor.cpp
    )
endif()

# Add CUDA PCA source when CUDA toolkit is available
if(INSIGHTAT_USE_CUDA_PCA)
    list(APPEND ALGORITHM_SOURCES
        modules/retrieval/pca_whitening_cuda.h
        modules/retrieval/pca_whitening_cuda.cu
    )
endif()

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
        util
        ${OpenCV_LIBS}
        DBoW3
        sift_gpu
        ${GLEW_LIBRARIES}
        ${OPENGL_LIBRARIES}
)

# CUDA PCA: compile definition + link cuBLAS/cuSOLVER so all consumers resolve the .cu symbols
if(INSIGHTAT_USE_CUDA_PCA)
    target_compile_definitions(InsightATAlgorithm PUBLIC INSIGHTAT_USE_CUDA_PCA)
    target_link_libraries(InsightATAlgorithm PUBLIC ${CUDA_PCA_LIBS})
endif()

# Link ONNXRuntime if SuperPoint is enabled
if(SUPERPOINT_AVAILABLE)
    target_link_libraries(InsightATAlgorithm
        PUBLIC
            ${onnxruntime_LIBRARIES}
            cudart
            cudnn
    )
    target_include_directories(InsightATAlgorithm
        PUBLIC
            ${ONNXRUNTIME_INCLUDE_DIR}
            ${CUDA_INCLUDE_DIRS}
    )
    target_compile_definitions(InsightATAlgorithm
        PUBLIC
            INSIGHTAT_HAS_SUPERPOINT
    )
    message(STATUS "InsightATAlgorithm: SuperPoint support enabled")
    message(STATUS "  Linking ONNX Runtime: ${onnxruntime_LIBRARIES}")
    message(STATUS "  ONNX Include: ${ONNXRUNTIME_INCLUDE_DIR}")
endif()

# ─────────────────────────────────────────────────────────────
# CLI Tools (shared logging helper)
# ─────────────────────────────────────────────────────────────
add_library(insightat_tools_logging STATIC tools/cli_logging.cpp)
target_include_directories(insightat_tools_logging PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(insightat_tools_logging PUBLIC glog::glog)
set_property(TARGET insightat_tools_logging PROPERTY FOLDER InsightAT/Tools)

# ─────────────────────────────────────────────────────────────
# CLI executables
# ─────────────────────────────────────────────────────────────

# isat_extract - Feature extraction tool
add_executable(isat_extract tools/isat_extract.cpp)

target_link_libraries(isat_extract
    PRIVATE
        insightat_tools_logging
        InsightATAlgorithm
        glog::glog
        ${OpenCV_LIBS}
        sift_gpu
        ${GLEW_LIBRARIES}
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
        insightat_tools_logging
        InsightATAlgorithm
        glog::glog
        sift_gpu
        ${GLEW_LIBRARIES}
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
        insightat_tools_logging
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
        insightat_tools_logging
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
        insightat_tools_logging
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

# isat_geo - Two-view geometry estimation (F / E / H) using GPU RANSAC
add_executable(isat_geo tools/isat_geo.cpp)

target_link_libraries(isat_geo
    PRIVATE
        insightat_tools_logging
        InsightATAlgorithm
        geo_ransac
        glog::glog
        ${OPENGL_LIBRARIES}
        ${GLEW_LIBRARIES}
        # ${EGL_LIBRARY}
        # EGL::EGL
)

target_include_directories(isat_geo
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/geometry
)

set_property(TARGET isat_geo PROPERTY FOLDER InsightAT/Tools)

# isat_project - Inspect/list/export project and ATTask data
add_executable(isat_project tools/isat_project.cpp)

target_link_libraries(isat_project
    PRIVATE
        insightat_tools_logging
        InsightATDatabase
        glog::glog
)

target_include_directories(isat_project
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party
)

set_property(TARGET isat_project PROPERTY FOLDER InsightAT/Tools)

# isat_twoview - Per-pair two-view reconstruction (focal estimation + triangulation + LM-BA)
add_executable(isat_twoview tools/isat_twoview.cpp)

target_link_libraries(isat_twoview
    PRIVATE
        insightat_tools_logging
        InsightATAlgorithm
        sfm_module
        glog::glog
        ${OpenCV_LIBS}
)

target_include_directories(isat_twoview
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/..  
        ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party
)

set_property(TARGET isat_twoview PROPERTY FOLDER InsightAT/Tools)

# isat_tracks - Build tracks from match + geo, write single IDC for SfM
add_executable(isat_tracks tools/isat_tracks.cpp)

target_link_libraries(isat_tracks
    PRIVATE
        insightat_tools_logging
        InsightATAlgorithm
        sfm_module
        glog::glog
)

target_include_directories(isat_tracks
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party
)

set_property(TARGET isat_tracks PROPERTY FOLDER InsightAT/Tools)

# isat_calibrate - Global focal-length refinement across image pairs
add_executable(isat_calibrate tools/isat_calibrate.cpp)

target_link_libraries(isat_calibrate
    PRIVATE
        insightat_tools_logging
        InsightATAlgorithm
        sfm_module
        glog::glog
)

target_include_directories(isat_calibrate
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/..  
        ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party
)

set_property(TARGET isat_calibrate PROPERTY FOLDER InsightAT/Tools)

# isat_camera_estimator - Per-group EXIF-based camera intrinsics estimator
add_executable(isat_camera_estimator tools/isat_camera_estimator.cpp)

target_link_libraries(isat_camera_estimator
    PRIVATE
        insightat_tools_logging
        InsightATAlgorithm
        InsightATDatabase
        ImageIO
        glog::glog
        ${GDAL_LIBRARIES}
)

target_include_directories(isat_camera_estimator
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party
)

set_property(TARGET isat_camera_estimator PROPERTY FOLDER InsightAT/Tools)


# ─────────────────────────────────────────────────────────────
# CameraEstimator 独立进程工具
# ─────────────────────────────────────────────────────────────

add_executable(CameraEstimator CameraEstimator.cpp)

target_link_libraries(CameraEstimator
    PRIVATE
        InsightATAlgorithm
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
# ─────────────────────────────────────────────────────────────
# Geometry module – GPU RANSAC (H / F / E)
# ─────────────────────────────────────────────────────────────
# geo_ransac is built unconditionally; it only needs OpenGL 4.3 + EGL.
# InsightATAlgorithm links against it so that the two-view geometry
# solver is available to all downstream tools and the main application.

add_subdirectory(modules/geometry)

target_link_libraries(InsightATAlgorithm
    PUBLIC
        geo_ransac
)

# ─────────────────────────────────────────────────────────────
# SfM module – two-view reconstruction (CPU/Eigen)
# ─────────────────────────────────────────────────────────────

add_subdirectory(modules/sfm)

target_link_libraries(InsightATAlgorithm
    PUBLIC
        sfm_module
)