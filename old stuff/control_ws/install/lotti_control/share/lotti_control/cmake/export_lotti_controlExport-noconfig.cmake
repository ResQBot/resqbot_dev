#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "lotti_control::lotti_control" for configuration ""
set_property(TARGET lotti_control::lotti_control APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(lotti_control::lotti_control PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/liblotti_control.so"
  IMPORTED_SONAME_NOCONFIG "liblotti_control.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS lotti_control::lotti_control )
list(APPEND _IMPORT_CHECK_FILES_FOR_lotti_control::lotti_control "${_IMPORT_PREFIX}/lib/liblotti_control.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
