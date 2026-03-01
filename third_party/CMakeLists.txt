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

# DBow3 for vocabulary tree retrieval
set(BUILD_UTILS OFF CACHE BOOL "Disable DBow3 utils")
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build DBow3 as static library")
add_subdirectory(DBow3)

list(APPEND directories cereal cmdLine progress stlplus3) 
foreach(inDirectory ${directories})
install(
  DIRECTORY ./${inDirectory}
  DESTINATION include/InsightAT/third_party/
  COMPONENT headers
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)
endforeach(inDirectory)

