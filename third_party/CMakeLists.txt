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

list(APPEND directories cereal cmdLine progress stlplus3) 
foreach(inDirectory ${directories})
install(
  DIRECTORY ./${inDirectory}
  DESTINATION include/InsightAT/third_party/
  COMPONENT headers
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)
endforeach(inDirectory)

