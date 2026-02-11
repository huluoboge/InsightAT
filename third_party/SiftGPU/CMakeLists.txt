project(SiftGPU C CXX)

if(NOT IS_MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
endif()

#add_definitions("-DSIFTGPU_NO_DEVIL")

SET(SIFTGPU_ENABLE_SERVER FALSE)
SET(SIFTGPU_ENABLE_OPENCL FALSE)
SET(SIFTGPU_ENABLE_CUDA FALSE)
SET(SIFTGPU_ENABLE_SSE TRUE)
SET(SIFTGPU_SSE_OPTIONS -march=core2 -mfpmath=sse)
SET(SIFTGPU_PREFER_GLUT FALSE)
SET(SIFTGPU_DISABLE_DEVIL TRUE)

# EGL headless rendering (default for Linux, Docker-friendly)
option(SIFTGPU_USE_EGL "Use EGL for headless rendering (Linux default)" ON)


if(NOT IS_MSVC)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -Wall -Wno-deprecated" )
endif()

IF(SIFTGPU_ENABLE_SSE)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=core2 -mfpmath=sse" )
ENDIF(SIFTGPU_ENABLE_SSE)

IF(SIFTGPU_PREFER_GLUT)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWINDOW_PREFER_GLUT" )
ENDIF(SIFTGPU_PREFER_GLUT)

IF(SIFTGPU_USE_EGL AND UNIX AND NOT APPLE)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_EGL" )
  message(STATUS "SiftGPU: Using EGL headless rendering")
ENDIF()

IF(SIFTGPU_ENABLE_OPENCL)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCL_SIFTGPU_ENABLED" )
ENDIF(SIFTGPU_ENABLE_OPENCL)

# Platform-specific library settings
IF(APPLE)
  SET(LIBS_SIFTGPU )
ELSE(APPLE)
  # Use EGL for headless rendering on Linux (no X11 dependency)
  IF(SIFTGPU_USE_EGL)
    SET(LIBS_SIFTGPU GL)
  ELSE()
    SET(LIBS_SIFTGPU glut GL X11)
  ENDIF()
ENDIF(APPLE)

IF(SIFTGPU_DISABLE_DEVIL)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DSIFTGPU_NO_DEVIL" )
ELSE(SIFTGPU_DISABLE_DEVIL)
  SET(LIBS_SIFTGPU ${LIBS_SIFTGPU} IL )
ENDIF(SIFTGPU_DISABLE_DEVIL)

find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)

# Find EGL for headless rendering (Linux only)
if(SIFTGPU_USE_EGL AND UNIX AND NOT APPLE)
    find_package(EGL)
    if(NOT EGL_FOUND)
        message(WARNING "EGL not found, falling back to X11/GLX mode")
        set(SIFTGPU_USE_EGL OFF)
        set(LIBS_SIFTGPU glut GL X11)
    else()
        message(STATUS "EGL found: ${EGL_LIBRARY}")
    endif()
endif()

message(STATUS "GLEW LIB IS ${GLEW_LIBRARIES}")
message(STATUS "OPENGL LIB IS ${OPENGL_LIBRARIES}")

set(SIFT_GPU_SOURCE_FILES
    FrameBufferObject.cpp
    FrameBufferObject.h
    GlobalUtil.cpp
    GlobalUtil.h
    GLTexImage.cpp
    GLTexImage.h
    ProgramGLSL.cpp
    ProgramGLSL.h
    ProgramGPU.h
    PyramidGL.cpp
    PyramidGL.h
    ShaderMan.cpp
    ShaderMan.h
    SiftGPU.cpp
    SiftGPU.h
    SiftMatch.cpp
    SiftMatch.h
    SiftPyramid.cpp
    SiftPyramid.h
)

set(CUDA_ENABLED OFF)
if(CUDA_ENABLED)
    add_definitions("-DCUDA_SIFTGPU_ENABLED")
    set(SIFT_GPU_SOURCE_FILES
        ${SIFT_GPU_SOURCE_FILES}
        CuTexImage.cpp
        CuTexImage.h
        ProgramCU.cu
        ProgramCU.h
        PyramidCU.cpp
        PyramidCU.h
        SiftMatchCU.cpp
        SiftMatchCU.h
    )
    add_library(sift_gpu ${SIFT_GPU_SOURCE_FILES})
else()
    add_library(sift_gpu ${SIFT_GPU_SOURCE_FILES})
endif()

if(IS_GNU)
    set(SIFT_GPU_LIBRARIES ${SIFT_GPU_LIBRARIES} "X11")
endif(IS_GNU)

# Link libraries
target_link_libraries(sift_gpu
    ${SIFT_GPU_LIBRARIES}
    ${GLEW_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${OPENGL_glu_LIBRARY}
    GLEW::GLEW
)

# Link EGL if enabled
if(SIFTGPU_USE_EGL AND EGL_FOUND)
    target_link_libraries(sift_gpu ${EGL_LIBRARY})
endif()
