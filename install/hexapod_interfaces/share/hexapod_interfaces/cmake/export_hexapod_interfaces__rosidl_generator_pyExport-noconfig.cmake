#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "hexapod_interfaces::hexapod_interfaces__rosidl_generator_py" for configuration ""
set_property(TARGET hexapod_interfaces::hexapod_interfaces__rosidl_generator_py APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(hexapod_interfaces::hexapod_interfaces__rosidl_generator_py PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libhexapod_interfaces__rosidl_generator_py.so"
  IMPORTED_SONAME_NOCONFIG "libhexapod_interfaces__rosidl_generator_py.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS hexapod_interfaces::hexapod_interfaces__rosidl_generator_py )
list(APPEND _IMPORT_CHECK_FILES_FOR_hexapod_interfaces::hexapod_interfaces__rosidl_generator_py "${_IMPORT_PREFIX}/lib/libhexapod_interfaces__rosidl_generator_py.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
