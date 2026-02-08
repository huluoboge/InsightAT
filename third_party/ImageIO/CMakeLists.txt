

#add_definitions(-DIMAGEIO_STATIC)

FILE(GLOB IMAGE_IO_SOURCE_FILES "*.h" "*.cpp" "*.hpp")

message("GDAL include dir i s" ${GDAL_INCLUDE_DIRS})
include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${GLOG_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${GDAL_INCLUDE_DIRS})

add_library(ImageIO STATIC ${IMAGE_IO_SOURCE_FILES})
target_link_libraries(ImageIO
    ${GLOG_LIBRARIES}
    ${OpenCV_LIBS}
    ${GDAL_LIBRARIES}
    )

set_property(TARGET ImageIO PROPERTY FOLDER INSIGHTMAP/lib)
