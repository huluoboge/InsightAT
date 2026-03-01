# FindEGL.cmake
# Find EGL library for headless OpenGL rendering
#
# This module defines:
#  EGL_FOUND        - True if EGL is found
#  EGL_INCLUDE_DIR  - EGL include directory
#  EGL_LIBRARY      - EGL library path
#  EGL::EGL         - Imported target (if found)

# Find EGL include directory
find_path(EGL_INCLUDE_DIR
    NAMES EGL/egl.h
    PATHS
        /usr/include
        /usr/local/include
        /opt/local/include
    DOC "EGL include directory"
)

# Find EGL library
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

# Handle standard arguments
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(EGL
    REQUIRED_VARS EGL_LIBRARY EGL_INCLUDE_DIR
)

# Create imported target
if(EGL_FOUND AND NOT TARGET EGL::EGL)
    add_library(EGL::EGL UNKNOWN IMPORTED)
    set_target_properties(EGL::EGL PROPERTIES
        IMPORTED_LOCATION "${EGL_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${EGL_INCLUDE_DIR}"
    )
endif()

mark_as_advanced(EGL_INCLUDE_DIR EGL_LIBRARY)
