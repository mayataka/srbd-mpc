#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "srbd_mpc::srbd_mpc" for configuration "Release"
set_property(TARGET srbd_mpc::srbd_mpc APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(srbd_mpc::srbd_mpc PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libsrbd_mpc.so"
  IMPORTED_SONAME_RELEASE "libsrbd_mpc.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS srbd_mpc::srbd_mpc )
list(APPEND _IMPORT_CHECK_FILES_FOR_srbd_mpc::srbd_mpc "${_IMPORT_PREFIX}/lib/libsrbd_mpc.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
