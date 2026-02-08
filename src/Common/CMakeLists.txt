

include_directories(
    # ${INSIGHTMAP_INNER_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${GDAL_INCLUDE_DIRS}
    )

FILE(GLOB COMMON_SOURCE_FILES "*.h" "*.cpp" "*.hpp")

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${GLOG_INCLUDE_DIRS})


add_definitions(-DCOMMON_STATIC)

add_library(Common STATIC ${COMMON_SOURCE_FILES})

target_link_libraries(Common
    ${GLOG_LIBRARIES}
    ImageIO
    )

set_property(TARGET Common PROPERTY FOLDER INSIGHTMAP/lib)
