# Matching module
add_library(matching_module
    sift_matcher.cpp
)

target_link_libraries(matching_module
    PUBLIC
        Eigen3::Eigen
        glog::glog
        siftgpu
)

target_include_directories(matching_module
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/..
)
