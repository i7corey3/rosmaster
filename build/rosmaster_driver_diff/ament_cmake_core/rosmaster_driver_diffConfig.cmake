# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_rosmaster_driver_diff_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED rosmaster_driver_diff_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(rosmaster_driver_diff_FOUND FALSE)
  elseif(NOT rosmaster_driver_diff_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(rosmaster_driver_diff_FOUND FALSE)
  endif()
  return()
endif()
set(_rosmaster_driver_diff_CONFIG_INCLUDED TRUE)

# output package information
if(NOT rosmaster_driver_diff_FIND_QUIETLY)
  message(STATUS "Found rosmaster_driver_diff: 0.0.0 (${rosmaster_driver_diff_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'rosmaster_driver_diff' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${rosmaster_driver_diff_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(rosmaster_driver_diff_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_targets-extras.cmake;ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${rosmaster_driver_diff_DIR}/${_extra}")
endforeach()
