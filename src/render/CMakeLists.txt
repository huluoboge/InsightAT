
project(render)
 
include_directories(
    #${INSIGHTMAP_INNER_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    #${GDAL_INCLUDE_DIRS}
    )

set(CMAKE_INCLUDE_CURRENT_DIR ON) 
 
# find_package(Qt5 COMPONENTS Core Gui Widgets OpenGL REQUIRED)

set(CMAKE_AUTOMOC ON) 
 
find_package(Qt5Widgets)  
find_package(Qt5Core)
find_package(Qt5Gui)
find_package(Qt5OpenGL)

FILE(GLOB RENDER_SRC_FILES 
"./*.cpp" 
) 

FILE(GLOB RENDER_HEAD_FILES 
"./*.h"
) 

FILE(GLOB RENDER_UI_FILES
"./*.ui"
) 


qt5_wrap_ui(WRAP_FILES ${RENDER_UI_FILES})


#set(RENDER_RCC_FILES InsightAT.qrc)
 
source_group("Ui" FILES ${RENDER_UI_FILES} ${WRAP_FILES} )
 
 include_directories(
     ${GLEW_INCLUDE_DIRS}
     "../")


 SET(RENDER_SOURCE_FILES
     ${RENDER_SRC_FILES}
     ${RENDER_HEAD_FILES}
     ${RENDER_RCC_FILES}
     ${WRAP_FILES}
     )

 #add_executable(${PROJECT_NAME} ${SRC_FILES} ${HEAD_FILES} ${RCC_FILES} ${WRAP_FILES})
add_library(render STATIC ${RENDER_SOURCE_FILES})

target_link_libraries(render
Qt5::Widgets 
Qt5::Core
Qt5::Gui
Qt5::OpenGL
${GLEW_LIBRARIES}
${OPENGL_LIBRARIES}
stlplus3
)

set_property(TARGET render PROPERTY FOLDER INSIGHTMAP/lib)

