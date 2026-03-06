# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_rosmaster_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED rosmaster_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(rosmaster_FOUND FALSE)
  elseif(NOT rosmaster_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(rosmaster_FOUND FALSE)
  endif()
  return()
endif()
set(_rosmaster_CONFIG_INCLUDED TRUE)

# output package information
if(NOT rosmaster_FIND_QUIETLY)
  message(STATUS "Found rosmaster: 0.0.0 (${rosmaster_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'rosmaster' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${rosmaster_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(rosmaster_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${rosmaster_DIR}/${_extra}")
endforeach()
