# ─────────────────────────────────────────────────────────────────────────────
# InsightAT – SfM primitives module
# ─────────────────────────────────────────────────────────────────────────────
#
# Targets:
#   sfm_module   – static library:
#                    • two_view_reconstruction  – focal estimation, E decomposition, DLT (CPU/Eigen)
#                    • gpu_twoview_sfm          – triangulation + BA residuals (GPU/EGL)
#
# Dependencies:
#   Eigen3       – linear algebra (SVD, LM)
#   glog         – logging
#   OpenGL 4.3   – compute shaders
#   GLEW         – GL extension loader
#   EGL          – headless surfaceless context
# ─────────────────────────────────────────────────────────────────────────────

cmake_minimum_required(VERSION 3.16)

# Enable CUDA language
enable_language(CUDA)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(CUDA REQUIRED)
find_package(OpenMP REQUIRED)
find_package(CUDAToolkit QUIET)

if(TARGET CUDA::cudart)
    set(SFM_CUDA_TEST_RUNTIME CUDA::cudart)
else()
    set(SFM_CUDA_TEST_RUNTIME ${CUDA_LIBRARIES})
endif()

if(NOT OPENGL_FOUND)
    find_package(OpenGL REQUIRED)
endif()
if(NOT GLEW_FOUND)
    find_package(Glew REQUIRED)
endif()
find_package(EGL REQUIRED)
if(NOT EGL_FOUND)
    message(FATAL_ERROR "EGL not found. sfm_module requires EGL for GPU triangulation.")
endif()

# Set CUDA architecture (volta, turing, ampere)
if(NOT CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES 75 80 86)  # Turing, Ampere, RTX 3000
endif()

# ── Dedicated CUDA kernel library (triangulation / reproj / resection) ────────
# Separate target so sfm_module and InsightATAlgorithm both link to it
# without circular‐dependency linker issues.
add_library(sfm_cuda_kernels STATIC
    cuda/cuda_sfm_common.cuh
    cuda/cuda_triangulation.cuh
    cuda/cuda_triangulation.cu
    cuda/cuda_reproj.cuh
    cuda/cuda_reproj.cu
    cuda/cuda_resection.cuh
    cuda/cuda_resection.cu
    cuda/cuda_pnp_lambdatwist_device.cuh
    cuda/cuda_pnp_ransac.cuh
    cuda/cuda_pnp_ransac.cu
)
target_include_directories(sfm_cuda_kernels
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/../../..
)
target_link_libraries(sfm_cuda_kernels
    PUBLIC
        Eigen3::Eigen
        ${CUDA_LIBRARIES}
    PRIVATE
        glog::glog
)
set_property(TARGET sfm_cuda_kernels PROPERTY CUDA_ARCHITECTURES ${CMAKE_CUDA_ARCHITECTURES})
set_property(TARGET sfm_cuda_kernels PROPERTY FOLDER "InsightAT/Modules")

add_library(sfm_module STATIC
    two_view_reconstruction.cpp
    two_view_reconstruction.h
    gpu_twoview_sfm.cpp
    gpu_twoview_sfm.h
    gpu_twoview_sfm_cuda.cu
    track_store.cpp
    track_store.h
    scene_normalization.cpp
    scene_normalization.h
    view_graph.cpp
    view_graph.h
    bundle_adjustment_analytic.cpp
    bundle_adjustment_analytic.h
    # Incremental SfM helpers excluded (used only by incremental_sfm; files kept)
    # incremental_sfm_helpers.cpp
    # incremental_sfm_helpers.h
    # ── CUDA SfM state + outlier rejection ────────────────────────────────
    cuda/cuda_sfm_state.cu
    cuda/cuda_sfm_state.h
    cuda/cuda_sfm_math.cuh
    cuda/gpu_sfm_scene.cu
    cuda/gpu_sfm_scene.h
    cuda/cuda_sfm_outlier_reject.cu
    cuda/cuda_sfm_outlier_reject.cuh
    # ── GPU BA subset selection (M2) ──────────────────────────────────────
    cuda/cuda_ba_subset.cu
    cuda/cuda_ba_subset.cuh
    # ── GPU-accelerated incremental SfM pipeline ──────────────────────────
    incremental_sfm_cuda_pipeline.cu
    incremental_sfm_cuda_pipeline.h
    resection_batch_cuda.cu
    resection_batch_cuda.h
)

# ── Unit test: analytic BA ─────────────────────────────────────────────────
add_executable(test_ba_analytic bundle_adjustment_analytic_test.cpp)
target_link_libraries(test_ba_analytic
    PRIVATE
        sfm_module
        algorithm_camera
        Eigen3::Eigen
        Ceres::ceres
        glog::glog
)
set_property(TARGET test_ba_analytic PROPERTY FOLDER InsightAT/Tests)

# ── Unit test: GLOMAP-style ray + λ per track (Ceres) ─────────────────────
add_executable(test_track_ray_lambda_ceres test_track_ray_lambda_ceres.cpp)
target_link_libraries(test_track_ray_lambda_ceres
    PRIVATE
        sfm_module
        algorithm_camera
        Eigen3::Eigen
        Ceres::ceres
        glog::glog
)
set_property(TARGET test_track_ray_lambda_ceres PROPERTY FOLDER InsightAT/Tests)

# ── CUDA 自检可执行文件（不依赖 GTest，随默认 all 目标构建）──────────────────
add_executable(test_cuda_sfm_state test_cuda_sfm_state.cpp)
target_link_libraries(test_cuda_sfm_state
    PRIVATE
        sfm_module
        glog::glog
        ${SFM_CUDA_TEST_RUNTIME}
)
if(NOT TARGET CUDA::cudart AND CUDA_INCLUDE_DIRS)
    target_include_directories(test_cuda_sfm_state PRIVATE ${CUDA_INCLUDE_DIRS})
endif()
set_property(TARGET test_cuda_sfm_state PROPERTY FOLDER InsightAT/Tests)

add_executable(test_cuda_pnp_ransac test_cuda_pnp_ransac.cpp)
target_link_libraries(test_cuda_pnp_ransac
    PRIVATE
        sfm_cuda_kernels
        Eigen3::Eigen
        glog::glog
        ${SFM_CUDA_TEST_RUNTIME}
)
if(NOT TARGET CUDA::cudart AND CUDA_INCLUDE_DIRS)
    target_include_directories(test_cuda_pnp_ransac PRIVATE ${CUDA_INCLUDE_DIRS})
endif()
set_property(TARGET test_cuda_pnp_ransac PROPERTY FOLDER InsightAT/Tests)

add_executable(test_cuda_triangulation_smoke test_cuda_triangulation_smoke.cpp)
target_link_libraries(test_cuda_triangulation_smoke
    PRIVATE
        sfm_cuda_kernels
        Eigen3::Eigen
        glog::glog
        ${SFM_CUDA_TEST_RUNTIME}
)
if(NOT TARGET CUDA::cudart AND CUDA_INCLUDE_DIRS)
    target_include_directories(test_cuda_triangulation_smoke PRIVATE ${CUDA_INCLUDE_DIRS})
endif()
set_property(TARGET test_cuda_triangulation_smoke PROPERTY FOLDER InsightAT/Tests)

add_executable(test_cuda_outlier_smoke test_cuda_outlier_smoke.cpp)
target_link_libraries(test_cuda_outlier_smoke
    PRIVATE
        sfm_module
        glog::glog
        ${SFM_CUDA_TEST_RUNTIME}
)
if(NOT TARGET CUDA::cudart AND CUDA_INCLUDE_DIRS)
    target_include_directories(test_cuda_outlier_smoke PRIVATE ${CUDA_INCLUDE_DIRS})
endif()
set_property(TARGET test_cuda_outlier_smoke PROPERTY FOLDER InsightAT/Tests)

# ── Unit test: PnP resection ──────────────────────────────────────────────
# add_executable(test_pnp_resection test_pnp_resection.cpp resection.cpp resection.h)
# target_link_libraries(test_pnp_resection
#     PRIVATE
#         sfm_module
#         geo_ransac
#         algorithm_camera
#         Eigen3::Eigen
#         Ceres::ceres
#         glog::glog
# )
# set_property(TARGET test_pnp_resection PROPERTY FOLDER InsightAT/Tests)

target_include_directories(sfm_module
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}       # callers: #include "two_view_reconstruction.h"
                                          #          #include "gpu_twoview_sfm.h"
        ${EIGEN3_INCLUDE_DIR}
    PRIVATE
        ${EGL_INCLUDE_DIR}
)

target_link_libraries(sfm_module
    PUBLIC
        Eigen3::Eigen
        EGL::EGL
        Ceres::ceres
        algorithm_camera
        sfm_cuda_kernels
        ${OPENGL_LIBRARIES}
        ${GLEW_LIBRARIES}
        ${CUDA_LIBRARIES}
    PRIVATE
        glog::glog
        OpenMP::OpenMP_CXX
)

target_compile_features(sfm_module PUBLIC cxx_std_17)
set_property(TARGET sfm_module PROPERTY FOLDER "InsightAT/Modules")

message(STATUS "sfm_module: Eigen3 ${EIGEN3_VERSION_STRING}")
