# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_centerline_extraction_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED centerline_extraction_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(centerline_extraction_FOUND FALSE)
  elseif(NOT centerline_extraction_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(centerline_extraction_FOUND FALSE)
  endif()
  return()
endif()
set(_centerline_extraction_CONFIG_INCLUDED TRUE)

# output package information
if(NOT centerline_extraction_FIND_QUIETLY)
  message(STATUS "Found centerline_extraction: 0.0.0 (${centerline_extraction_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'centerline_extraction' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${centerline_extraction_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(centerline_extraction_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${centerline_extraction_DIR}/${_extra}")
endforeach()
