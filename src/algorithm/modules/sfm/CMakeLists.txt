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

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

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

add_library(sfm_module STATIC
    two_view_reconstruction.cpp
    two_view_reconstruction.h
    gpu_twoview_sfm.cpp
    gpu_twoview_sfm.h
    track_store.cpp
    track_store.h
    view_graph.cpp
    view_graph.h
    bundle_adjustment_analytic.cpp
    bundle_adjustment_analytic.h
    # Incremental SfM helpers excluded (used only by incremental_sfm; files kept)
    # incremental_sfm_helpers.cpp
    # incremental_sfm_helpers.h
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
        ${OPENGL_LIBRARIES}
        ${GLEW_LIBRARIES}
    PRIVATE
        glog::glog
)

target_compile_features(sfm_module PUBLIC cxx_std_17)
set_property(TARGET sfm_module PROPERTY FOLDER "InsightAT/Modules")

message(STATUS "sfm_module: Eigen3 ${EIGEN3_VERSION_STRING}")
