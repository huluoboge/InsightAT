
cmake_minimum_required(VERSION 3.10.0)
 
project(InsightAT)
 
include_directories(
    # ${INSIGHTMAP_INNER_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${GDAL_INCLUDE_DIRS}
    )
set(CMAKE_INCLUDE_CURRENT_DIR ON) 
 
#must be setted
set(CMAKE_AUTOMOC ON) 

find_package(Qt5Widgets)  
find_package(Qt5Core)
find_package(Qt5Gui)
find_package(Qt5OpenGL)
find_package(Qt5Network)

FILE(GLOB SRC_FILES 
"./*.cpp"
AdvanceAT.cpp
) 

FILE(GLOB HEAD_FILES 
"./*.h"
) 

FILE(GLOB UI_FILES 
"./*.ui"
) 
 
#must be set
qt5_wrap_ui(WRAP_FILES ${UI_FILES})
 

set(RCC_FILES InsightMapper.qrc)
 
#ui and generate ui files
source_group("Ui" FILES ${UI_FILES} ${WRAP_FILES} )
 
# include_directories(${INSIGHTMAP_INCLUDE_DIRS}
#     ${CMAKE_SOURCE_DIR}/software/InsightAT)
# link_directories(${INSIGHTMAP_LIB_DIRS} )

include_directories("../")


add_executable(${PROJECT_NAME} ${SRC_FILES} ${HEAD_FILES} ${RCC_FILES} ${WRAP_FILES})
 

target_link_libraries(${PROJECT_NAME}
Qt5::Core
Qt5::Gui
Qt5::OpenGL
Qt5::Network
${GLEW_LIBRARIES}
${OPENGL_LIBRARIES}
${GDAL_LIBRARIES}
${GLOG_LIBRARIES}
stlplus3
ImageIO
render
Common
)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER INSIGHTMAP/AT)
