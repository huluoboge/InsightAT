#---------------------
#Third party libraries
#---------------------

if (MSVC)
  # Disable warning about size_t conversions for all third_party libraries.
  add_compile_options(
    "$<$<COMPILE_LANGUAGE:C>:/wd4267>"
    "$<$<COMPILE_LANGUAGE:CXX>:/wd4267>"
  )
endif()


# Basic filesystem utils
set(STLPLUS_INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/include/InsightAT/third_party/stlplus3)
add_subdirectory(stlplus3)
set(STLPLUS_LIBRARY stlplus3 PARENT_SCOPE)

add_subdirectory(ImageIO)

# SiftGPU for optional backend.
if(INSIGHTAT_BUILD_GUI_ONLY)
  message(STATUS "Third-party: GUI-only mode, skipping SiftGPU")
elseif(INSIGHTAT_ENABLE_SIFTGPU)
  message(STATUS "Third-party: enabling optional SiftGPU build")
  add_subdirectory(SiftGPU)
else()
  message(STATUS "Third-party: SiftGPU disabled")
endif()

# PopSift for default SIFT extraction/matching backend.
if(INSIGHTAT_BUILD_GUI_ONLY)
  message(STATUS "Third-party: GUI-only mode, skipping PopSift")
elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/popsift/CMakeLists.txt")
  message(STATUS "Third-party: enabling PopSift build (default backend)")
  add_subdirectory(popsift)
  # Ensure a stable namespaced target for downstream links even when PopSift
  # examples are disabled.
  if(TARGET popsift AND NOT TARGET PopSift::popsift)
    add_library(PopSift::popsift ALIAS popsift)
  endif()
else()
  message(FATAL_ERROR
    "third_party/popsift was not found. PopSift is required as the default backend.")
endif()

# PoseLib for alternate resection / absolute-pose backend
set(_INSIGHTAT_BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS})
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build shared libraries" FORCE)
add_subdirectory(PoseLib)
set(BUILD_SHARED_LIBS ${_INSIGHTAT_BUILD_SHARED_LIBS} CACHE BOOL "Build shared libraries" FORCE)
unset(_INSIGHTAT_BUILD_SHARED_LIBS)

list(APPEND directories cereal cmdLine progress stlplus3) 
foreach(inDirectory ${directories})
install(
  DIRECTORY ./${inDirectory}
  DESTINATION include/InsightAT/third_party/
  COMPONENT headers
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)
endforeach(inDirectory)

