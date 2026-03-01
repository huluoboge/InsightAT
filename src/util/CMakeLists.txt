# -----------------------------------------------------------------------------
# Util: 轻量公共工具（string_utils, numeric, insight_global）
# -----------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.10)

set(UTIL_SOURCES
    string_utils.cpp
    numeric.cpp
)

add_library(util STATIC ${UTIL_SOURCES})

target_include_directories(util
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/..
)

target_compile_features(util PUBLIC cxx_std_17)

if(NOT TARGET Eigen3::Eigen)
    find_package(Eigen3 REQUIRED)
endif()
target_link_libraries(util PUBLIC Eigen3::Eigen)

if(NOT TARGET glog::glog)
    find_package(glog REQUIRED)
endif()
target_link_libraries(util PUBLIC glog::glog)

if(MSVC)
    target_compile_options(util PRIVATE /W4)
else()
    target_compile_options(util PRIVATE -Wall -Wextra -Wpedantic)
endif()

set_property(TARGET util PROPERTY FOLDER InsightAT/lib)
