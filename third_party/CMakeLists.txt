#---------------------
#Third party libraries
#---------------------

if (MSVC)
  # Disable warning about size_t conversions for all third_party libraries.
  add_compile_options(/wd4267)
endif()


# Basic filesystem utils
set(STLPLUS_INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/include/InsightAT/third_party/stlplus3)
add_subdirectory(stlplus3)
set(STLPLUS_LIBRARY stlplus3 PARENT_SCOPE)

add_subdirectory(ImageIO)

# SiftGPU for optional backend.
if(INSIGHTAT_ENABLE_SIFTGPU)
  message(STATUS "Third-party: enabling optional SiftGPU build")
  add_subdirectory(SiftGPU)
else()
  message(STATUS "Third-party: SiftGPU disabled")
endif()

# PopSift for default SIFT extraction/matching backend.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/popsift/CMakeLists.txt")
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
add_subdirectory(PoseLib)

list(APPEND directories cereal cmdLine progress stlplus3) 
foreach(inDirectory ${directories})
install(
  DIRECTORY ./${inDirectory}
  DESTINATION include/InsightAT/third_party/
  COMPONENT headers
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)
endforeach(inDirectory)

