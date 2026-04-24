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

# SiftGPU for feature extraction
add_subdirectory(SiftGPU)

# PopSift for optional SIFT extraction backend.
# Policy: when CUDA SiftGPU is enabled, PopSift must be built as well so
# backend availability issues are discovered at configure/build time.
if(SIFTGPU_ENABLE_CUDA)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/popsift/CMakeLists.txt")
    message(STATUS "Third-party: SIFTGPU_ENABLE_CUDA=ON, enabling PopSift build")
    add_subdirectory(popsift)
  else()
    message(FATAL_ERROR
      "SIFTGPU_ENABLE_CUDA=ON requires third_party/popsift, but it was not found. "
      "Please add the PopSift source tree or disable CUDA SiftGPU.")
  endif()
else()
  message(STATUS "Third-party: SIFTGPU_ENABLE_CUDA=OFF, PopSift build disabled")
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

