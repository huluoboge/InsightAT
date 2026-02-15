cmake_minimum_required(VERSION 3.18)
project(SuperPointONNX VERSION 1.0.0 LANGUAGES CXX CUDA)

# C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

# 编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -Wall")
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -O3")

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

# 查找依赖
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV Version: ${OpenCV_VERSION}")
message(STATUS "OpenCV Include: ${OpenCV_INCLUDE_DIRS}")

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
    ${CMAKE_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${ONNXRUNTIME_INCLUDE_DIR}
    ${CUDA_INCLUDE_DIRS}
)

# 链接目录
link_directories(
    ${ONNXRUNTIME_LIB_DIR}
    ${CUDA_LIBRARIES}
)

# 主程序
add_executable(superpoint_inference
    src/superpoint_inference.cpp
)

# 链接库
target_link_libraries(superpoint_inference
    ${OpenCV_LIBS}
    onnxruntime
    cudart
    cudnn
)

# 设置 RPATH
set_target_properties(superpoint_inference PROPERTIES
    BUILD_RPATH "${ONNXRUNTIME_LIB_DIR}:${CUDA_LIBRARIES}"
    INSTALL_RPATH "${ONNXRUNTIME_LIB_DIR}:${CUDA_LIBRARIES}"
)

# 安装规则
install(TARGETS superpoint_inference
    RUNTIME DESTINATION bin
)

# 可选：编译为库
option(BUILD_SHARED_LIB "Build as shared library" OFF)

if(BUILD_SHARED_LIB)
    # 提取核心代码为库（需要重构代码分离 main）
    message(STATUS "Shared library build not yet implemented")
endif()

# 打印构建信息
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")
