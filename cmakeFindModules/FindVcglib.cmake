###########################################################
#                  Find Lemon Library
#----------------------------------------------------------

FIND_PATH(VCGLIB_DIR wrap/ply/ply.h
    HINTS "${VCGLIB_ROOT}" "$ENV{VCGLIB_ROOT}" "${VCGLIB_INCLUDE_DIR_HINTS}"
    PATHS "$ENV{PROGRAMFILES}/vcglib" "$ENV{PROGRAMW6432}/vcglib"
    PATH_SUFFIXES vcglib
    DOC "Root directory of vcglib includes")

##====================================================
## Include LEMON library
##----------------------------------------------------
IF(EXISTS "${VCGLIB_DIR}" AND NOT "${VCGLIB_DIR}" STREQUAL "")
  SET(VCGLIB_FOUND TRUE)
  # Remove /lemon from path (math.h cannot be exposed all time)
  GET_FILENAME_COMPONENT(VCGLIB_INCLUDE_DIRS "${VCGLIB_DIR}" PATH)
  SET(VCGLIB_DIR "${VCGLIB_DIR}" CACHE PATH "" FORCE)
  MARK_AS_ADVANCED(VCGLIB_DIR)

  SET(VCGLIB_INCLUDE_DIR ${VCGLIB_DIR})
  SET(VCGLIB_INCLUDE_DIRS ${VCGLIB_DIR})

  MESSAGE(STATUS "Vcglib found (include: ${VCGLIB_DIR})")
ELSE()
  MESSAGE(FATAL_ERROR "Can't find Vcglib library. "
          "Please -DVCGLIB_DIR=\"PATH\" "
          "or VCGLIB_INCLUDE_DIRS env. variable to a valid Vcglib path. ")
  package_report_not_found(VCGLIB "Vcglib cannot be found")
ENDIF()
##====================================================
