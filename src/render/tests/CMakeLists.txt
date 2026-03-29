# ─────────────────────────────────────────────────────────────────────────────
# CMakeLists.txt — render 库属性测试
# Feature: render-lib-refactor
# ─────────────────────────────────────────────────────────────────────────────

cmake_minimum_required(VERSION 3.14)

# ── GTest ────────────────────────────────────────────────────────────────────
find_package(GTest QUIET)
if(NOT GTest_FOUND)
    # 尝试从系统源码构建
    find_path(GTEST_SRC_DIR
        NAMES CMakeLists.txt
        PATHS /usr/src/googletest/googletest
              /usr/src/gtest
        NO_DEFAULT_PATH
    )
    if(GTEST_SRC_DIR)
        add_subdirectory(${GTEST_SRC_DIR} ${CMAKE_CURRENT_BINARY_DIR}/gtest EXCLUDE_FROM_ALL)
        set(GTEST_LIBRARIES gtest gtest_main)
        set(GTEST_INCLUDE_DIRS ${GTEST_SRC_DIR}/include)
        set(GTest_FOUND TRUE)
    endif()
endif()

# ── rapidcheck（通过 FetchContent 下载） ──────────────────────────────────────
include(FetchContent)

FetchContent_Declare(
    rapidcheck
    GIT_REPOSITORY https://github.com/emil-e/rapidcheck.git
    GIT_TAG        master
    GIT_SHALLOW    TRUE
)

set(RC_ENABLE_GTEST ON CACHE BOOL "Enable rapidcheck GTest integration" FORCE)
FetchContent_MakeAvailable(rapidcheck)

# ── Eigen3 ───────────────────────────────────────────────────────────────────
if(NOT TARGET Eigen3::Eigen)
    find_package(Eigen3 REQUIRED)
endif()
if(NOT TARGET glog::glog)
    find_package(glog REQUIRED)
endif()

# ── 属性测试可执行文件 ────────────────────────────────────────────────────────
if(GTest_FOUND OR TARGET gtest)
    add_executable(test_opengl_mat_property
        test_opengl_mat_property.cpp
    )

    target_include_directories(test_opengl_mat_property
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/..   # src/render/
            ${CMAKE_CURRENT_SOURCE_DIR}/../..  # src/
    )

    target_compile_features(test_opengl_mat_property PRIVATE cxx_std_17)

    # 链接 GTest
    if(TARGET GTest::gtest_main)
        target_link_libraries(test_opengl_mat_property
            PRIVATE
                GTest::gtest_main
                GTest::gtest
        )
    else()
        target_link_libraries(test_opengl_mat_property
            PRIVATE
                gtest_main
                gtest
        )
        target_include_directories(test_opengl_mat_property
            PRIVATE ${GTEST_INCLUDE_DIRS}
        )
    endif()

    # 链接 rapidcheck
    target_link_libraries(test_opengl_mat_property
        PRIVATE
            rapidcheck
            rapidcheck_gtest
    )

    # 链接 Eigen3
    target_link_libraries(test_opengl_mat_property
        PRIVATE
            Eigen3::Eigen
            glog::glog
    )

    # 注册 CTest
    include(GoogleTest)
    gtest_discover_tests(test_opengl_mat_property)

    message(STATUS "render property tests enabled (test_opengl_mat_property)")
else()
    message(STATUS "GTest not found, skipping render property tests")
endif()
