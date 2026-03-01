# Try to find the MPIR libraries
# MPIR_FOUND - system has MPIR lib
# MPIR_INCLUDE_DIR - the MPIR include directory
# MPIR_LIBRARIES_DIR - Directory where the MPIR libraries are located
# MPIR_LIBRARIES - the MPIR libraries
# MPIR_IN_CGAL_AUXILIARY - TRUE if the MPIR found is the one distributed with CGAL in the auxiliary folder



include(FindPackageHandleStandardArgs)


if(MPIR_INCLUDE_DIR)
  set(MPIR_in_cache TRUE)
else()
  set(MPIR_in_cache FALSE)
endif()
if(NOT MPIR_LIBRARIES)
  set(MPIR_in_cache FALSE)
endif()

# Is it already configured?
if (MPIR_in_cache)

  set(MPIR_FOUND TRUE)

else()

  find_path(MPIR_INCLUDE_DIR
            NAMES mpir.h
            HINTS ENV MPIR_INC_DIR
                  ENV MPIR_DIR
                  ${MPIR_DIR}/../include
                  ${CGAL_INSTALLATION_PACKAGE_DIR}/auxiliary/gmp/include
            PATH_SUFFIXES include
                DOC "The directory containing the MPIR header files"
           )

   if(MPIR_INCLUDE_DIR)
       message(status " mpir include dir=${MPIR_INCLUDE_DIR}")
   endif()
  if ( MPIR_INCLUDE_DIR STREQUAL "${CGAL_INSTALLATION_PACKAGE_DIR}/auxiliary/gmp/include" )
    cache_set( MPIR_IN_CGAL_AUXILIARY TRUE )
  endif()

  find_library(MPIR_LIBRARIES
        NAMES mpir libmpir
        HINTS
            ENV MPIR_LIB_DIR
            ENV MPIR_DIR
            ${MPIR_DIR}
            ${MPIR_DIR}/lib
            ${MPIR_DIR}../lib
            ${CGAL_INSTALLATION_PACKAGE_DIR}/auxiliary/gmp/lib
         PATH_SUFFIXES lib
         PATHS
            ${MPIR_DIR}
            /usr/lib
            /usr/local/lib
    DOC "Path to the MPIR library"
    )

  if ( MPIR_LIBRARIES )
       message(status " mpir library dir=${MPIR_LIBRARIES}")
    get_filename_component(MPIR_LIBRARIES_DIR ${MPIR_LIBRARIES} PATH CACHE )
  endif()


  # Attempt to load a user-defined configuration for MPIR if couldn't be found
  if ( NOT MPIR_INCLUDE_DIR OR NOT MPIR_LIBRARIES_DIR )
    include( MPIRConfig OPTIONAL )
  else()
    set(MPIR_FOUND TRUE)
  endif()

  if(NOT MPIR_FOUND)
       message(FATAL_ERROR "MPIR was not found!")
  endif()
  #find_package_handle_standard_args(MPIR "DEFAULT_MSG" MPIR_LIBRARIES MPIR_INCLUDE_DIR)

endif()

