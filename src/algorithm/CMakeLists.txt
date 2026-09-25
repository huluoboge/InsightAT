cmake_minimum_required(VERSION 3.16)

project(InsightATAlgorithm)

# ─────────────────────────────────────────────────────────────
# 设置编译选项
# ─────────────────────────────────────────────────────────────

# ─────────────────────────────────────────────────────────────
# 查找依赖
# ─────────────────────────────────────────────────────────────

find_package(Glog REQUIRED)

# Optional CUDA for PCA training (cuBLAS + cuSOLVER)
set(INSIGHTAT_USE_CUDA_PCA FALSE)
set(CUDA_PCA_LIBS "")
find_package(CUDAToolkit QUIET)
if(CUDAToolkit_FOUND)
  enable_language(CUDA)
  # Default: multiple archs so one build runs on Pascal→Ada (60=Pascal, 70=Volta, 75=Turing, 80=Ampere, 86=RTX30, 89=RTX40).
  # Override with -DCMAKE_CUDA_ARCHITECTURES=native to build only for current GPU, or e.g. "86" for RTX 30xx.
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES OR CMAKE_CUDA_ARCHITECTURES STREQUAL "")
    if(CUDAToolkit_VERSION VERSION_GREATER_EQUAL "11.8")
      set(CMAKE_CUDA_ARCHITECTURES "60;70;75;80;86;89;90" CACHE STRING "CUDA architectures (semicolon-separated)")
    elseif(CUDAToolkit_VERSION VERSION_GREATER_EQUAL "11.1")
      set(CMAKE_CUDA_ARCHITECTURES "60;70;75;80;86" CACHE STRING "CUDA architectures (semicolon-separated)")
    else()
      set(CMAKE_CUDA_ARCHITECTURES "60;70;75;80" CACHE STRING "CUDA architectures (semicolon-separated)")
    endif()
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
    io/geopack_index.h
    io/geopack_index.cpp
    io/track_store_idc.h
    io/track_store_idc.cpp
    io/exif/exif.h
    io/exif/exif.cpp
    io/exif/exif_IO.hpp
    io/exif/exif_IO_EasyExif.hpp
    
    # Modules - Feature Extraction
    modules/extraction/sift_gpu_extractor.h
    modules/extraction/sift_gpu_extractor.cpp
    modules/extraction/feature_distribution.h
    modules/extraction/feature_distribution.cpp
    modules/extraction/key_points_node.h
    modules/extraction/key_points_node.cpp
    
    # Modules - Feature Matching
    modules/matching/sift_matcher.h
    modules/matching/sift_matcher.cpp
    modules/cpu_cascade_hash/cpu_cascade_hash.h
    modules/cpu_cascade_hash/cpu_cascade_hash.cpp
    
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
    modules/sfm/view_graph_loader.h
    modules/sfm/view_graph_loader.cpp
    modules/sfm/resection.h
    modules/sfm/resection.cpp
    modules/sfm/incremental_sfm_pipeline.h
    modules/sfm/incremental_sfm_pipeline.cpp
    modules/sfm/resection_batch.h
    modules/sfm/resection_batch.cpp
    modules/sfm/visibility_pyramid.h
    modules/sfm/visibility_pyramid.cpp
    modules/sfm/incremental_triangulation.h
    modules/sfm/incremental_triangulation.cpp
    # Incremental SfM excluded from build (files kept); to be re-integrated after stepwise implementation
    # modules/sfm/incremental_sfm.h
    # modules/sfm/incremental_sfm.cpp
    # modules/sfm/incremental_sfm_engine.h
    # modules/sfm/incremental_sfm_engine.cpp
)

# ─────────────────────────────────────────────────────────────────────────────
# Camera module (intrinsics + distortion) – shared by sfm_module and InsightATAlgorithm
# ─────────────────────────────────────────────────────────────────────────────
if(NOT TARGET Eigen3::Eigen)
  find_package(Eigen3 REQUIRED)
endif()
add_library(algorithm_camera STATIC
    modules/camera/camera_types.h
    modules/camera/camera_utils.h
    modules/camera/camera_utils.cpp
)
target_include_directories(algorithm_camera
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        ${EIGEN3_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(algorithm_camera PUBLIC Eigen3::Eigen)
set_property(TARGET algorithm_camera PROPERTY FOLDER InsightAT/Modules)

# Add CUDA PCA source when CUDA toolkit is available
if(INSIGHTAT_USE_CUDA_PCA)
    list(APPEND ALGORITHM_SOURCES
        modules/retrieval/pca_whitening_cuda.h
        modules/retrieval/pca_whitening_cuda.cu
    )
endif()

# Add SfM CUDA kernels when CUDAToolkit is present
if(CUDAToolkit_FOUND)
    list(APPEND ALGORITHM_SOURCES
        modules/gpu_cascade_hash/gpu_cascade_hash.h
        modules/gpu_cascade_hash/gpu_cascade_hash.cpp
        modules/gpu_cascade_hash/cuda_cascade_hash_kernel.h
        modules/gpu_cascade_hash/cuda_cascade_hash_kernel.cu
        modules/sfm/cuda/cuda_sfm_common.cuh
        modules/sfm/cuda/cuda_triangulation.cuh
        modules/sfm/cuda/cuda_triangulation.cu
        modules/sfm/cuda/cuda_reproj.cuh
        modules/sfm/cuda/cuda_reproj.cu
        modules/sfm/cuda/cuda_resection.cuh
        modules/sfm/cuda/cuda_resection.cu
    )
    message(STATUS "InsightATAlgorithm: CUDA kernels enabled (cascade_hash / triangulation / reproj / resection)")
else()
    message(STATUS "InsightATAlgorithm: CUDA kernels disabled (CUDAToolkit not found)")
endif()

# ─────────────────────────────────────────────────────────────
# 创建库目标
# ─────────────────────────────────────────────────────────────

include_directories(
    ${CMAKE_SOURCE_DIR}/third_party/PoseLib
    ${CMAKE_BINARY_DIR}/third_party/PoseLib/generated_headers
)

add_library(InsightATAlgorithm STATIC ${ALGORITHM_SOURCES})

# ─────────────────────────────────────────────────────────────
# 链接依赖
# ─────────────────────────────────────────────────────────────

target_include_directories(InsightATAlgorithm
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        ${CMAKE_SOURCE_DIR}/third_party
        ${CMAKE_SOURCE_DIR}/third_party/PoseLib
)

target_link_libraries(InsightATAlgorithm
    PUBLIC
        glog::glog
        InsightATDatabase
        util
        algorithm_camera
        sfm_module
        PoseLib
        PoseLib::PoseLib
        ${OpenCV_LIBS}
)

if(TARGET sift_gpu)
    target_link_libraries(InsightATAlgorithm
        PUBLIC
            sift_gpu
            ${GLEW_LIBRARIES}
            ${OPENGL_LIBRARIES}
    )
    target_compile_definitions(InsightATAlgorithm PUBLIC INSIGHTAT_ENABLE_SIFTGPU=1)
    message(STATUS "InsightATAlgorithm: optional SiftGPU backend enabled")
else()
    target_compile_definitions(InsightATAlgorithm PUBLIC INSIGHTAT_ENABLE_SIFTGPU=0)
    message(STATUS "InsightATAlgorithm: SiftGPU backend disabled")
endif()

# Optional PopSift backend for SIFT extraction (CUDA required).
# If PopSift is not available, fall back to SiftGPU (EGL+GLSL).
if(TARGET PopSift::popsift)
    target_link_libraries(InsightATAlgorithm PUBLIC PopSift::popsift)
    target_compile_definitions(InsightATAlgorithm PUBLIC HAVE_CUDA)
    message(STATUS "InsightATAlgorithm: PopSift backend enabled (CUDA SIFT extraction)")
elseif(TARGET sift_gpu)
    message(STATUS "InsightATAlgorithm: PopSift not available, using SiftGPU backend (GLSL SIFT extraction)")
else()
    message(FATAL_ERROR
        "InsightATAlgorithm: Neither PopSift::popsift nor sift_gpu target is available. "
        "Either enable CUDA+PopSift or enable SiftGPU.")
endif()

# CUDA PCA: compile definition + link cuBLAS/cuSOLVER so all consumers resolve the .cu symbols
if(INSIGHTAT_USE_CUDA_PCA)
    target_compile_definitions(InsightATAlgorithm PUBLIC INSIGHTAT_USE_CUDA_PCA)
    target_link_libraries(InsightATAlgorithm PUBLIC ${CUDA_PCA_LIBS})
endif()

# Link CUDA runtime library when CUDA is available
if(CUDAToolkit_FOUND)
    target_link_libraries(InsightATAlgorithm PUBLIC cudart)
endif()

# ── Unit test: incremental triangulation (strategy B) ──
add_executable(test_incremental_triangulation modules/sfm/incremental_triangulation_test.cpp)
target_link_libraries(test_incremental_triangulation
    PRIVATE
        InsightATAlgorithm
        glog::glog
)
target_include_directories(test_incremental_triangulation
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        ${CMAKE_SOURCE_DIR}/third_party
)
set_property(TARGET test_incremental_triangulation PROPERTY FOLDER InsightAT/Tests)

# ── SfM diagnosis tool: analyze unregistered images after a SfM run ─────────
add_executable(test_sfm_diagnosis modules/sfm/test_sfm_diagnosis.cpp)
target_link_libraries(test_sfm_diagnosis
    PRIVATE
        InsightATAlgorithm
        PoseLib
        glog::glog
)
target_include_directories(test_sfm_diagnosis
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        ${CMAKE_SOURCE_DIR}/third_party
)
set_property(TARGET test_sfm_diagnosis PROPERTY FOLDER InsightAT/Tests)

# ── SfM diagnosis – clean from-scratch triangulation (no incremental_triangulation dependency) ──
add_executable(test_sfm_diag2 modules/sfm/test_sfm_diag2.cpp)
target_link_libraries(test_sfm_diag2
    PRIVATE
        InsightATAlgorithm
        PoseLib
        glog::glog
)
target_include_directories(test_sfm_diag2
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        ${CMAKE_SOURCE_DIR}/third_party
)
set_property(TARGET test_sfm_diag2 PROPERTY FOLDER InsightAT/Tests)

# ── Resection candidate cache unit test ────────────────────────────────────
add_executable(test_resection_candidate_cache modules/sfm/test_resection_candidate_cache.cpp)
target_link_libraries(test_resection_candidate_cache
    PRIVATE
        InsightATAlgorithm
        PoseLib
        glog::glog
)
target_include_directories(test_resection_candidate_cache
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        ${CMAKE_SOURCE_DIR}/third_party
)
set_property(TARGET test_resection_candidate_cache PROPERTY FOLDER InsightAT/Tests)

# ── CPU cascade hash test ──
add_executable(test_cpu_cascade_hash
    modules/cpu_cascade_hash/cpu_cascade_hash_test.cpp
)
target_link_libraries(test_cpu_cascade_hash
    PRIVATE
        InsightATAlgorithm
        glog::glog
)
target_include_directories(test_cpu_cascade_hash
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        ${CMAKE_SOURCE_DIR}/third_party
)
set_property(TARGET test_cpu_cascade_hash PROPERTY FOLDER InsightAT/Tests)

# CLI tools live in src/cli/ (added from src/CMakeLists.txt after this directory).

# ─────────────────────────────────────────────────────────────
# 编译选项
# ─────────────────────────────────────────────────────────────

if(MSVC)
    target_compile_options(InsightATAlgorithm PRIVATE
        "$<$<COMPILE_LANGUAGE:C>:/W4>"
        "$<$<COMPILE_LANGUAGE:CXX>:/W4>"
    )
else()
    target_compile_options(InsightATAlgorithm PRIVATE
        "$<$<COMPILE_LANGUAGE:C>:-Wall;-Wextra;-Wpedantic>"
        "$<$<COMPILE_LANGUAGE:CXX>:-Wall;-Wextra;-Wpedantic>"
    )
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

# CUDA geo is available project-wide when cuda_geo_ransac target exists.
# isat_geo / isat_geo_cuda linkage is handled in src/cli/CMakeLists.txt.
if(TARGET cuda_geo_ransac)
  add_compile_definitions(INSIGHTAT_HAS_CUDA_GEO=1)
  message(STATUS "InsightATAlgorithm: INSIGHTAT_HAS_CUDA_GEO=1 (global)")
endif()

# ─────────────────────────────────────────────────────────────
# SfM module – two-view reconstruction (CPU/Eigen)
# ─────────────────────────────────────────────────────────────

add_subdirectory(modules/sfm)

target_link_libraries(InsightATAlgorithm
    PUBLIC
        sfm_module
)
