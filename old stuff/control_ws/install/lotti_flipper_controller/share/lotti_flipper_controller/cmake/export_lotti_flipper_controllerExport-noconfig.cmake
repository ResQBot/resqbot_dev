#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "lotti_flipper_controller::lotti_flipper_controller" for configuration ""
set_property(TARGET lotti_flipper_controller::lotti_flipper_controller APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(lotti_flipper_controller::lotti_flipper_controller PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/liblotti_flipper_controller.so"
  IMPORTED_SONAME_NOCONFIG "liblotti_flipper_controller.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS lotti_flipper_controller::lotti_flipper_controller )
list(APPEND _IMPORT_CHECK_FILES_FOR_lotti_flipper_controller::lotti_flipper_controller "${_IMPORT_PREFIX}/lib/liblotti_flipper_controller.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
