# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_turn_on_agribot_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED turn_on_agribot_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(turn_on_agribot_FOUND FALSE)
  elseif(NOT turn_on_agribot_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(turn_on_agribot_FOUND FALSE)
  endif()
  return()
endif()
set(_turn_on_agribot_CONFIG_INCLUDED TRUE)

# output package information
if(NOT turn_on_agribot_FIND_QUIETLY)
  message(STATUS "Found turn_on_agribot: 0.0.0 (${turn_on_agribot_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'turn_on_agribot' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${turn_on_agribot_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(turn_on_agribot_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${turn_on_agribot_DIR}/${_extra}")
endforeach()
