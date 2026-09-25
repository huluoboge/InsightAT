
CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

# Read project version from VERSION file at project root
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/VERSION" INSIGHTAT_VERSION_RAW)
string(STRIP "${INSIGHTAT_VERSION_RAW}" INSIGHTAT_VERSION)
project(InsightAT VERSION ${INSIGHTAT_VERSION} LANGUAGES C CXX)


# guard against in-source builds
if (${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(FATAL_ERROR "In-source builds not allowed.")
endif()

# Default build is in RelWithDebInfo mode
if(CMAKE_BUILD_TYPE)
    message(STATUS "Build type specified as ${CMAKE_BUILD_TYPE}")
else()
    message(STATUS "Build type not specified, using Release")
    set(CMAKE_BUILD_TYPE Release)
    set(IS_DEBUG FALSE)
endif()

if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    set(IS_DEBUG TRUE)
endif()

if (UNIX)
    set(CMAKE_C_FLAGS_RELEASE "-O3")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3")
endif ()

# ==============================================================================
# INSIGHTAT build options
# ==============================================================================
# Default: CLI-only (isat_* tools). Qt UI lives under src/ui + src/render and is
# optional; product UI is expected to move to Node (simple-gui) over time.
option(INSIGHTAT_BUILD_QT_UI "Build legacy Qt5 GUI (InsightAT + at_bundler_viewer + render)" OFF)
option(INSIGHTAT_BUILD_GUI_ONLY "Build only Qt GUI targets and skip CUDA/algorithm components" OFF)
option(INSIGHTAT_BUILD_RENDER_TESTS "Build render property tests and fetch RapidCheck" OFF)

if(INSIGHTAT_BUILD_GUI_ONLY AND NOT INSIGHTAT_BUILD_QT_UI)
    set(INSIGHTAT_BUILD_QT_UI ON CACHE BOOL "Build legacy Qt5 GUI" FORCE)
    message(STATUS "Top-level: INSIGHTAT_BUILD_GUI_ONLY implies INSIGHTAT_BUILD_QT_UI=ON")
endif()

find_package(Threads REQUIRED)

# Default: enable SiftGPU (+ CUDA backend when toolkit is present).
find_package(CUDAToolkit QUIET)
if(CUDAToolkit_FOUND)
    option(INSIGHTAT_ENABLE_SIFTGPU "Build and enable SiftGPU backend" ON)
    option(SIFTGPU_ENABLE_CUDA "Build SiftGPU with CUDA backend (third_party/SiftGPU)" ON)
else()
    # No CUDA toolkit: CPU+EGL SiftGPU fallback
    option(INSIGHTAT_ENABLE_SIFTGPU "Build and enable SiftGPU backend" ON)
    option(SIFTGPU_ENABLE_CUDA "Build SiftGPU with CUDA backend (third_party/SiftGPU)" OFF)
    message(STATUS "Top-level: CUDA not found, forcing CPU+EGL mode with SiftGPU")
endif()

if(INSIGHTAT_BUILD_GUI_ONLY)
    message(STATUS "Top-level: INSIGHTAT_BUILD_GUI_ONLY=ON, skipping algorithm/CUDA components")
endif()
message(STATUS "Top-level: INSIGHTAT_BUILD_QT_UI=${INSIGHTAT_BUILD_QT_UI}")
message(STATUS "Top-level: INSIGHTAT_ENABLE_SIFTGPU=${INSIGHTAT_ENABLE_SIFTGPU}")
message(STATUS "Top-level: SIFTGPU_ENABLE_CUDA=${SIFTGPU_ENABLE_CUDA}")

# Set build path
set(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}")
set(LIBRARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}")

message(STATUS "library save to : ${LIBRARY_OUTPUT_PATH}")

# Copy data/config to build for default sensor_db (isat_sfm / isat_camera_estimator use -d <build>/data/config/camera_sensor_database.txt)
if(EXISTS "${CMAKE_SOURCE_DIR}/data/config")
  file(COPY "${CMAKE_SOURCE_DIR}/data/config" DESTINATION "${CMAKE_BINARY_DIR}/data")
  message(STATUS "Copied data/config to ${CMAKE_BINARY_DIR}/data/config for default sensor DB")
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(IS_GNU)
    # Hide incorrect warnings for uninitialized Eigen variables under GCC.
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-maybe-uninitialized")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-maybe-uninitialized")
endif()



# ==============================================================================
# Additional cmake find modules
# ==============================================================================
set(CMAKE_MODULE_PATH
    ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmakeFindModules)
# include(OptimizeForArchitecture)
include(CMakeHelper)
set(INSIGHTMAP_DEFINITIONS "")
MACRO(register_definitions DEF)
add_definitions(${DEF})
string(REPLACE "-D" "" STRIPPED_DEF ${DEF})
list(APPEND INSIGHTMAP_DEFINITIONS ${STRIPPED_DEF})
ENDMACRO()
# add_definitions(-DVL_DISABLE_SSE2)
#ceres maybe crash if use ceres with eigen sse optimize
#AutodetectHostArchitecture()
#OptimizeForArchitecture()




# ==============================================================================
# Third-party libraries:
# ==============================================================================
# add_subdirectory(src/third_party)

# ==============================================================================
# GDAL 
# ==============================================================================
find_package(GDAL REQUIRED)
if(NOT GDAL_FOUND)
    message(STATUS "GDAL was not found . ")
else()
    message(STATUS "GDAL libraries: ${GDAL_LIBRARIES}")
endif()


# ==============================================================================
# Eigen
# ==============================================================================
# EIGEN_INCLUDE_DIR_HINTS set to eigen dir
# ==============================================================================

find_package(Eigen3 REQUIRED)
message(STATUS "Eigen3 version is ${EIGEN3_VERSION},dir is ${EIGEN3_INCLUDE_DIRS}")

include_directories(${EIGEN3_INCLUDE_DIRS}) 
if(NOT INSIGHTAT_BUILD_GUI_ONLY)
    # NVIDIA ships cuDSS under libcudss/<CUDA_major>/. The unversioned package
    # at .../cmake/cudss often resolves to the newest tree (e.g. 13), which then
    # requires cublas.so.13 and breaks a CUDA 12.x toolkit. Prefer a tree that
    # matches CUDAToolkit_VERSION_MAJOR unless the user already set a matching dir.
    if(CUDAToolkit_FOUND AND DEFINED CUDAToolkit_VERSION_MAJOR)
        set(_insightat_cudss_maj "${CUDAToolkit_VERSION_MAJOR}")
        set(_insightat_cudss_need_auto TRUE)
        if(DEFINED cudss_DIR AND NOT cudss_DIR STREQUAL "")
            string(FIND "${cudss_DIR}" "libcudss/${_insightat_cudss_maj}" _insightat_cudss_hit)
            if(_insightat_cudss_hit GREATER_EQUAL 0)
                set(_insightat_cudss_need_auto FALSE)
            endif()
        endif()
        if(_insightat_cudss_need_auto)
            set(_insightat_cudss_candidates
                "/usr/lib/x86_64-linux-gnu/libcudss/${_insightat_cudss_maj}/cmake/cudss"
                "/usr/lib64/libcudss/${_insightat_cudss_maj}/cmake/cudss"
                "/usr/local/libcudss/${_insightat_cudss_maj}/cmake/cudss"
            )
            if(DEFINED ENV{CUDSS_ROOT} AND NOT "$ENV{CUDSS_ROOT}" STREQUAL "")
                list(APPEND _insightat_cudss_candidates
                    "$ENV{CUDSS_ROOT}/lib/cmake/cudss"
                    "$ENV{CUDSS_ROOT}/lib64/cmake/cudss"
                )
            endif()
            foreach(_cand IN LISTS _insightat_cudss_candidates)
                if(EXISTS "${_cand}/cudss-config.cmake")
                    set(cudss_DIR "${_cand}" CACHE PATH "cuDSS CMake dir for CUDA ${_insightat_cudss_maj}" FORCE)
                    message(STATUS "Auto-selected cuDSS for CUDA ${_insightat_cudss_maj}: ${cudss_DIR}")
                    break()
                endif()
            endforeach()
        endif()
        unset(_insightat_cudss_maj)
        unset(_insightat_cudss_need_auto)
        unset(_insightat_cudss_hit)
        unset(_insightat_cudss_candidates)
        unset(_cand)
    endif()

    # Ceres (required: for bundle adjustment in incremental SfM)
    find_package(Ceres REQUIRED)
    message(STATUS "Ceres: ${Ceres_VERSION}, BA module enabled")
else()
    message(STATUS "GUI-only mode: skipping Ceres")
endif()


if(IS_DEBUG)
    register_definitions(-DEIGEN_INITIALIZE_MATRICES_BY_NAN)
endif()
#vs2017 error
if(IS_MSVC)
    register_definitions(-D_DISABLE_EXTENDED_ALIGNED_STORAGE)
    register_definitions(-D_USE_MATH_DEFINES)
    register_definitions(-DNOMINMAX)
    register_definitions(-DWIN32_LEAN_AND_MEAN)
    add_compile_options(
        "$<$<COMPILE_LANGUAGE:C>:/utf-8>"
        "$<$<COMPILE_LANGUAGE:CXX>:/utf-8>"
    )
endif()

# ==============================================================================
# OpenCV
# ==============================================================================
# - only external and enabled only if INSIGHTAT_USE_OPENCV is set to ON
# ==============================================================================
if(NOT INSIGHTAT_BUILD_GUI_ONLY)
    find_package(OpenCV REQUIRED)
    if (NOT OpenCV_FOUND OR OpenCV_VERSION VERSION_LESS "3.0.0")
        message(STATUS "OpenCV was not found (note that OpenCV version >= 3.0.0 is required). ")
    endif()
else()
    message(STATUS "GUI-only mode: skipping OpenCV")
endif()

#===============================================================================
# GLOG
#===============================================================================
find_package(Glog REQUIRED)
if(NOT GLOG_FOUND)
    message(FATAL_ERROR,"Can't find GLOG library, set GLOG_INCLUDE_DIR_HINTS path to find glog")
else()
    message(STATUS "Find GLOG ")
    set(GLOG_INCLUDE_DIRS ${GLOG_INCLUDE_DIR_HINTS})
    set(GLOG_LIBRARIES ${GLOG_LIBRARY})
    message(STATUS "GLOG lib is ${GLOG_LIBRARIES}")
endif()
if(IS_MSVC)
    register_definitions(-DGLOG_NO_ABBREVIATED_SEVERITIES)
    register_definitions(-DGL_GLEXT_PROTOTYPES)
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

include_directories(${GLOG_INCLUDE_DIRS})
if(NOT TARGET glog::glog)
    add_library(glog::glog INTERFACE IMPORTED)
    set_target_properties(glog::glog PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${GLOG_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${GLOG_LIBRARIES}"
    )
endif()
# GLEW / OpenGL — needed by algorithm GPU paths (SiftGPU, geo_ransac), not only Qt.
# ==============================================================================
if(NOT INSIGHTAT_BUILD_GUI_ONLY)
    find_package(OpenGL REQUIRED COMPONENTS OpenGL)
    if(INSIGHTAT_BUILD_QT_UI)
        find_package(OpenGL COMPONENTS GLU)
    endif()
    find_package(Glew REQUIRED)
else()
    message(STATUS "GUI-only mode: skipping OpenGL/GLEW")
endif()

include_directories(third_party)
include_directories(third_party/PoseLib)
include_directories(src)

# Allow top-level control over whether SiftGPU builds with CUDA.
# If you want to enable the CUDA backend for SiftGPU from the top-level
# configure step, set `-DSIFTGPU_ENABLE_CUDA=ON` when running CMake.


add_subdirectory(third_party)
add_subdirectory(src)








