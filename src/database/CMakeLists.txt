# ─────────────────────────────────────────────────────────────
# CMakeLists.txt for InsightAT Database Module
# ─────────────────────────────────────────────────────────────
# 
# 数据库模块包含：
# - 坐标系定义和管理
# - 测量数据结构（GNSS/IMU/GCP/SLAM）
# - 输入位姿表示
# - 空三任务及快照设计

cmake_minimum_required(VERSION 3.20)

# ─────────────────────────────────────────────────────────────
# 源文件列表
# ─────────────────────────────────────────────────────────────
set(DATABASE_SOURCES
    database_types.cpp
)

set(DATABASE_HEADERS
    database_types.h
)

# ─────────────────────────────────────────────────────────────
# 创建对象库（可被其他target链接）
# ─────────────────────────────────────────────────────────────
add_library(InsightATDatabase OBJECT
    ${DATABASE_SOURCES}
    ${DATABASE_HEADERS}
)

# ─────────────────────────────────────────────────────────────
# 编译选项
# ─────────────────────────────────────────────────────────────
target_include_directories(InsightATDatabase
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/..
)

# C++17标准
target_compile_features(InsightATDatabase PUBLIC cxx_std_17)

# Eigen3 (线性代数)
if(NOT TARGET Eigen3::Eigen)
    find_package(Eigen3 REQUIRED)
endif()
target_link_libraries(InsightATDatabase PUBLIC Eigen3::Eigen)

# Glog (日志库)
if(NOT TARGET glog::glog)
    find_package(glog REQUIRED)
endif()
target_link_libraries(InsightATDatabase PUBLIC glog::glog)

# ─────────────────────────────────────────────────────────────
# 编译器警告级别
# ─────────────────────────────────────────────────────────────
if(MSVC)
    target_compile_options(InsightATDatabase PRIVATE /W4)
else()
    target_compile_options(InsightATDatabase PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ─────────────────────────────────────────────────────────────
# 单元测试
# ─────────────────────────────────────────────────────────────
# 创建序列化测试可执行文件
find_package(GTest QUIET)
if(GTest_FOUND)
    # 基础序列化测试
    add_executable(test_serialization_comprehensive
        test_serialization_comprehensive.cpp
    )
    
    target_include_directories(test_serialization_comprehensive
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}/..
    )
    
    target_compile_features(test_serialization_comprehensive PRIVATE cxx_std_17)
    
    target_link_libraries(test_serialization_comprehensive
        PRIVATE
            GTest::gtest_main
            GTest::gtest
    )
    
    add_test(NAME SerializationTests COMMAND test_serialization_comprehensive)
    
    # 项目序列化测试
    add_executable(test_project_serialization
        test_project_serialization.cpp
    )
    
    target_include_directories(test_project_serialization
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}/..
    )
    
    target_compile_features(test_project_serialization PRIVATE cxx_std_17)
    
    target_link_libraries(test_project_serialization
        PRIVATE
            GTest::gtest_main
            GTest::gtest
    )
    
    add_test(NAME ProjectSerializationTests COMMAND test_project_serialization)
    
    message(STATUS "Serialization unit tests enabled")
else()
    message(STATUS "GTest not found, skipping serialization tests")
endif()

# ─────────────────────────────────────────────────────────────
# 导出配置
# ─────────────────────────────────────────────────────────────
# 使用者可以链接此目标：
# target_link_libraries(MyTarget InsightATDatabase)
