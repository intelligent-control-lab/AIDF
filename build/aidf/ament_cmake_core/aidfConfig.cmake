# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_aidf_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED aidf_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(aidf_FOUND FALSE)
  elseif(NOT aidf_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(aidf_FOUND FALSE)
  endif()
  return()
endif()
set(_aidf_CONFIG_INCLUDED TRUE)

# output package information
if(NOT aidf_FIND_QUIETLY)
  message(STATUS "Found aidf: 0.0.0 (${aidf_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'aidf' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${aidf_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(aidf_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_targets-extras.cmake;ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${aidf_DIR}/${_extra}")
endforeach()
