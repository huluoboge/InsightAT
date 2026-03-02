# ─────────────────────────────────────────────────────────────────────────────
# geometry/CMakeLists.txt
#
# Builds:
#   geo_ransac          – static library: GPU RANSAC for H / F / E
#   test_geo_ransac     – standalone test executable (links geo_ransac)
#
# Requires:
#   OpenGL 4.3+ (Compute Shaders)
#   GLEW
#   EGL  (headless rendering – no display / Window-System required)
#
# All three dependencies are already found in the root CMakeLists.txt.
# EGL is resolved via cmakeFindModules/FindEGL.cmake.
# ─────────────────────────────────────────────────────────────────────────────

cmake_minimum_required(VERSION 3.16)

# ── Verify that the required upstream finds have been performed ───────────────
if(NOT OPENGL_FOUND)
    find_package(OpenGL REQUIRED)
endif()

if(NOT GLEW_FOUND)
    find_package(Glew REQUIRED)
endif()

# EGL is optional at the project level; make it required for this module.
find_package(EGL REQUIRED)
if(NOT EGL_FOUND)
    message(FATAL_ERROR "EGL not found. geo_ransac requires EGL for headless GPU compute.")
endif()

find_package(Eigen3 REQUIRED)

# ── Library target ────────────────────────────────────────────────────────────
add_library(geo_ransac STATIC
    gpu_geo_ransac.cpp
    gpu_geo_ransac.h
)

target_include_directories(geo_ransac
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}          # so callers can #include "gpu_geo_ransac.h"
    PRIVATE
        ${EGL_INCLUDE_DIR}
)

target_link_libraries(geo_ransac
    PUBLIC
        EGL::EGL
        Eigen3::Eigen
        ${OPENGL_LIBRARIES}
        ${GLEW_LIBRARIES}
)

# Require C++17 (for std::string manipulations in the implementation).
target_compile_features(geo_ransac PUBLIC cxx_std_17)

set_property(TARGET geo_ransac PROPERTY FOLDER "InsightAT/Modules")

# ── Test executable ───────────────────────────────────────────────────────────
add_executable(test_geo_ransac test.cpp)

target_link_libraries(test_geo_ransac
    PRIVATE
        geo_ransac
)

target_include_directories(test_geo_ransac
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

set_property(TARGET test_geo_ransac PROPERTY FOLDER "InsightAT/Tests")

message(STATUS "geo_ransac: OpenGL=${OPENGL_LIBRARIES}  GLEW=${GLEW_LIBRARIES}  EGL=${EGL_LIBRARY}")
