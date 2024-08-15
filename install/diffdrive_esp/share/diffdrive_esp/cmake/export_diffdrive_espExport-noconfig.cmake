#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "diffdrive_esp::diffdrive_esp" for configuration ""
set_property(TARGET diffdrive_esp::diffdrive_esp APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(diffdrive_esp::diffdrive_esp PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libdiffdrive_esp.so"
  IMPORTED_SONAME_NOCONFIG "libdiffdrive_esp.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS diffdrive_esp::diffdrive_esp )
list(APPEND _IMPORT_CHECK_FILES_FOR_diffdrive_esp::diffdrive_esp "${_IMPORT_PREFIX}/lib/libdiffdrive_esp.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
