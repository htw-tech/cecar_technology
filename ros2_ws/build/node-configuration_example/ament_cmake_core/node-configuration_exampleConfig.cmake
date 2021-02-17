# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_node-configuration_example_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED node-configuration_example_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(node-configuration_example_FOUND FALSE)
  elseif(NOT node-configuration_example_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(node-configuration_example_FOUND FALSE)
  endif()
  return()
endif()
set(_node-configuration_example_CONFIG_INCLUDED TRUE)

# output package information
if(NOT node-configuration_example_FIND_QUIETLY)
  message(STATUS "Found node-configuration_example: 0.0.0 (${node-configuration_example_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'node-configuration_example' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message(WARNING "${_msg}")
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(node-configuration_example_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${node-configuration_example_DIR}/${_extra}")
endforeach()
