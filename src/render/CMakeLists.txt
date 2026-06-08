
project(render)

if(NOT TARGET Eigen3::Eigen)
    find_package(Eigen3 REQUIRED)
endif()
if(NOT TARGET glog::glog)
    find_package(glog REQUIRED)
endif()

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    "../"
    )

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOMOC ON)

find_package(Qt5Widgets)
find_package(Qt5Core)
find_package(Qt5Gui)
find_package(Qt5OpenGL)

# In GUI-only mode, OpenGL/GLEW are optional
if(NOT INSIGHTAT_BUILD_GUI_ONLY)
    find_package(OpenGL REQUIRED)
    find_package(Glew REQUIRED)
else()
    find_package(OpenGL QUIET)
    find_package(Glew QUIET)
    if(NOT OPENGL_FOUND OR NOT GLEW_FOUND)
        message(STATUS "Render: GUI-only mode without OpenGL/GLEW - RenderWidget will have limited functionality")
    endif()
endif()

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
 
include_directories("../")


 SET(RENDER_SOURCE_FILES
     ${RENDER_SRC_FILES}
     ${RENDER_HEAD_FILES}
     ${RENDER_RCC_FILES}
     ${WRAP_FILES}
     )

 #add_executable(${PROJECT_NAME} ${SRC_FILES} ${HEAD_FILES} ${RCC_FILES} ${WRAP_FILES})
add_library(render STATIC ${RENDER_SOURCE_FILES})

if(GLEW_FOUND)
    target_include_directories(render PRIVATE ${GLEW_INCLUDE_DIRS})
endif()

target_link_libraries(render
Qt5::Widgets
Qt5::Core
Qt5::Gui
Qt5::OpenGL
stlplus3
Eigen3::Eigen
glog::glog
ImageIO
/usr/lib/x86_64-linux-gnu/libGLU.so
)

if(GLEW_FOUND AND OPENGL_FOUND)
    target_link_libraries(render
        ${GLEW_LIBRARIES}
        ${OPENGL_LIBRARIES}
    )
    message(STATUS "Render: Linking with GLEW/OpenGL libraries")
else()
    message(STATUS "Render: Linking without GLEW/OpenGL libraries - using Qt OpenGL backend with system GLU")
    target_compile_definitions(render PUBLIC RENDER_NO_GLEW)
endif()

set_property(TARGET render PROPERTY FOLDER INSIGHTMAP/lib)

# ── 属性测试 ──────────────────────────────────────────────────────────────────
add_subdirectory(tests)

