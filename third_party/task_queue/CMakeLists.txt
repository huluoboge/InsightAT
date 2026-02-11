cmake_minimum_required(VERSION 3.10)

# 项目名称和版本
project(TaskQueue VERSION 1.0.0 LANGUAGES CXX)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 查找线程库
find_package(Threads REQUIRED)

# 包含目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# 源文件
set(SOURCES
    task_queue_demo.cpp
)

# 头文件（可选，用于IDE支持）
set(HEADERS
    task_queue.hpp
)

# 创建可执行文件
add_executable(task_queue_demo ${SOURCES} ${HEADERS})

# 链接线程库
target_link_libraries(task_queue_demo PRIVATE Threads::Threads)

# 编译选项
target_compile_options(task_queue_demo PRIVATE
    -Wall
    -Wextra
    -pedantic
    $<$<CONFIG:Debug>:-g -O0>
    $<$<CONFIG:Release>:-O3 -DNDEBUG>
)

# 安装目标（可选）
install(TARGETS task_queue_demo
    RUNTIME DESTINATION bin
)

# 输出编译信息
message(STATUS "TaskQueue Demo Build Configuration:")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "  Threads Library: ${CMAKE_THREAD_LIBS_INIT}")

# 可选：添加测试
option(BUILD_TESTS "Build unit tests" OFF)
if(BUILD_TESTS)
    enable_testing()
    # 这里可以添加测试配置
    message(STATUS "Building with tests enabled")
endif()

# 可选：生成compile_commands.json用于IDE
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)