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

# PoseLib for alternate resection / absolute-pose backend
add_subdirectory(PoseLib)

# PBA (Parallel Bundle Adjustment, CPU-only static library)
# We build only the three CPU source files and suppress PBA's own flags.
find_package(Threads REQUIRED)
add_library(pba_cpu STATIC
    pba/src/pba/pba.cpp
    pba/src/pba/ConfigBA.cpp
    pba/src/pba/SparseBundleCPU.cpp
)
target_compile_definitions(pba_cpu PUBLIC PBA_NO_GPU)
target_include_directories(pba_cpu PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/pba/src)
target_link_libraries(pba_cpu PRIVATE Threads::Threads)
# PBA was written against C++03 and has many warnings under -Wall; suppress them
# so they don't pollute the parent-project build output.
target_compile_options(pba_cpu PRIVATE -w -fPIC)
set_property(TARGET pba_cpu PROPERTY FOLDER "InsightAT/ThirdParty")

list(APPEND directories cereal cmdLine progress stlplus3) 
foreach(inDirectory ${directories})
install(
  DIRECTORY ./${inDirectory}
  DESTINATION include/InsightAT/third_party/
  COMPONENT headers
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)
endforeach(inDirectory)

