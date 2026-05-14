# FindEGL.cmake
# Find EGL library for headless OpenGL rendering
#
# This module defines:
#  EGL_FOUND        - True if EGL is found
#  EGL_INCLUDE_DIR  - EGL include directory
#  EGL_LIBRARY      - EGL library path
#  EGL::EGL         - Imported target (if found)

# vcpkg provides EGL on Windows through ANGLE. Its wrapper sets
# EGL_LIBRARY to the ANGLE imported target, so keep that target form instead
# of forcing a raw library path.
if(WIN32 AND NOT TARGET unofficial::angle::libEGL)
    find_package(unofficial-angle CONFIG QUIET)
endif()

if(WIN32 AND TARGET unofficial::angle::libEGL)
    set(EGL_LIBRARY unofficial::angle::libEGL)
endif()

# Find EGL include directory
if(NOT EGL_INCLUDE_DIR)
    find_path(EGL_INCLUDE_DIR
        NAMES EGL/egl.h
        PATHS
            /usr/include
            /usr/local/include
            /opt/local/include
        DOC "EGL include directory"
    )
endif()

# Find EGL library
if(NOT EGL_LIBRARY)
    find_library(EGL_LIBRARY
        NAMES EGL
        PATHS
            /usr/lib
            /usr/local/lib
            /opt/local/lib
            /usr/lib/x86_64-linux-gnu
            /usr/lib/aarch64-linux-gnu
        DOC "EGL library"
    )
endif()

# Handle standard arguments
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(EGL
    REQUIRED_VARS EGL_LIBRARY EGL_INCLUDE_DIR
)

# Create imported target
if(EGL_FOUND AND NOT TARGET EGL::EGL)
    if(TARGET "${EGL_LIBRARY}")
        add_library(EGL::EGL INTERFACE IMPORTED)
        set_target_properties(EGL::EGL PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${EGL_INCLUDE_DIR}"
            INTERFACE_LINK_LIBRARIES "${EGL_LIBRARY}"
        )
    else()
        add_library(EGL::EGL UNKNOWN IMPORTED)
        set_target_properties(EGL::EGL PROPERTIES
            IMPORTED_LOCATION "${EGL_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${EGL_INCLUDE_DIR}"
        )
    endif()
endif()

mark_as_advanced(EGL_INCLUDE_DIR EGL_LIBRARY)
