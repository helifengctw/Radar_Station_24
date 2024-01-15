#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "radar_interfaces::yolov5_detect__rosidl_typesupport_cpp" for configuration "Debug"
set_property(TARGET radar_interfaces::yolov5_detect__rosidl_typesupport_cpp APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(radar_interfaces::yolov5_detect__rosidl_typesupport_cpp PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libyolov5_detect__rosidl_typesupport_cpp.so"
  IMPORTED_SONAME_DEBUG "libyolov5_detect__rosidl_typesupport_cpp.so"
  )

list(APPEND _cmake_import_check_targets radar_interfaces::yolov5_detect__rosidl_typesupport_cpp )
list(APPEND _cmake_import_check_files_for_radar_interfaces::yolov5_detect__rosidl_typesupport_cpp "${_IMPORT_PREFIX}/lib/libyolov5_detect__rosidl_typesupport_cpp.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
