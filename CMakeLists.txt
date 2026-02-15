
CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

project(InsightAT C CXX)


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
option(INSIGHTAT_BUILD_SOFTWARES "Build softwares" OFF)
option(INSIGHTAT_ENABLE_SUPERPOINT "Enable SuperPoint feature extraction (requires ONNXRuntime)" ON)

# Set build path
set(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}")
set(LIBRARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}")

message(STATUS "library save to : ${LIBRARY_OUTPUT_PATH}")

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
message(STATUS "Eigen3 version is ${EIGEN3_VERSION}")


if(IS_DEBUG)
    register_definitions(-DEIGEN_INITIALIZE_MATRICES_BY_NAN)
endif()
#vs2017 error
if(IS_MSVC)
    register_definitions(-D_DISABLE_EXTENDED_ALIGNED_STORAGE)
endif()

# ==============================================================================
# OpenCV
# ==============================================================================
# - only external and enabled only if INSIGHTAT_USE_OPENCV is set to ON
# ==============================================================================
find_package( OpenCV REQUIRED )
if (NOT OpenCV_FOUND OR OpenCV_VERSION VERSION_LESS "3.0.0")
    message(STATUS "OpenCV was not found (note that OpenCV version >= 3.0.0 is required). ")
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
# GLEW
# ==============================================================================
find_package(OpenGL REQUIRED)
find_package(Glew REQUIRED)

# ==============================================================================
# ONNXRuntime (Optional, for SuperPoint)
# ==============================================================================
set(SUPERPOINT_AVAILABLE FALSE)
if(INSIGHTAT_ENABLE_SUPERPOINT)
    # find_package(onnxruntime QUIET)
    # 设置 CUDA 路径（默认使用 CUDA 11.8）
    if(NOT DEFINED CUDA_TOOLKIT_ROOT_DIR)
        set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda-11.8")
    endif()

    set(CMAKE_CUDA_COMPILER "${CUDA_TOOLKIT_ROOT_DIR}/bin/nvcc")
    set(CUDA_INCLUDE_DIRS "${CUDA_TOOLKIT_ROOT_DIR}/include")
    set(CUDA_LIBRARIES "${CUDA_TOOLKIT_ROOT_DIR}/lib64")

    # 打印配置信息
    message(STATUS "Building SuperPoint ONNX C++")
    message(STATUS "CUDA Toolkit: ${CUDA_TOOLKIT_ROOT_DIR}")
    message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")

    # ONNX Runtime 路径
    set(ONNXRUNTIME_ROOT "/opt/onnxruntime-gpu" CACHE PATH "ONNX Runtime installation path")
    set(ONNXRUNTIME_INCLUDE_DIR "${ONNXRUNTIME_ROOT}/include")
    set(ONNXRUNTIME_LIB_DIR "${ONNXRUNTIME_ROOT}/lib")

    if(NOT EXISTS ${ONNXRUNTIME_INCLUDE_DIR})
        message(FATAL_ERROR "ONNX Runtime not found at ${ONNXRUNTIME_ROOT}")
    endif()

    message(STATUS "ONNX Runtime: ${ONNXRUNTIME_ROOT}")

    # 包含目录
    include_directories(
        ${ONNXRUNTIME_INCLUDE_DIR}
        ${CUDA_INCLUDE_DIRS}
    )

    # 链接目录
    link_directories(
        ${ONNXRUNTIME_LIB_DIR}
        ${CUDA_LIBRARIES}
    )
    # 链接库
    # target_link_libraries(superpoint_inference
    #     ${OpenCV_LIBS}
    #     onnxruntime
    #     cudart
    #     cudnn
    # )
    set(onnxruntime_FOUND TRUE)
    set(onnxruntime_LIBRARIES "onnxruntime")
    if(onnxruntime_FOUND)
        set(SUPERPOINT_AVAILABLE TRUE)
        message(STATUS "ONNXRuntime found - SuperPoint backend enabled")
        message(STATUS "  ONNXRuntime include: ${ONNXRUNTIME_INCLUDE_DIR}")
        message(STATUS "  ONNXRuntime library: ${onnxruntime_LIBRARIES}")
    else()
        
        message(STATUS "ONNXRuntime not found - SuperPoint backend disabled")
        message(STATUS "  To enable SuperPoint: install ONNXRuntime and set onnxruntime_DIR")
    endif()
else()
    message(STATUS "SuperPoint disabled via INSIGHTAT_ENABLE_SUPERPOINT=OFF")
endif()

include_directories(third_party)
include_directories(src)

add_subdirectory(third_party)
add_subdirectory(src)








