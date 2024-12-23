# This module defines the following variables:
# OpenNI_FOUND   : 1 if OpenNI was found, 0 otherwise
# OpenNI_LIBRARY : OpenNI location
# OpenNI_INCLUDE_DIR: directory where the headers can be found

include(FindPackageHandleStandardArgs)

find_library(OpenNI_LIBRARY Names OpenNI2 PATHS /usr/lib PATH_SUFFIXES lib )
find_path(OpenNI_INCLUDE_DIRS OpenNI.h PATHS /usr/include/openni2 /usr/include PATH_SUFFIXES openni2 )

message(STATUS ${OpenNI_LIBRARY})
message(STATUS ${OpenNI_INCLUDE_DIRS})

find_package_handle_standard_args(OpenNI DEFAULT_MSG OpenNI_LIBRARY OpenNI_INCLUDE_DIRS)

# Hack: since the macro makes the package name uppercase
set(OpenNI_FOUND ${OPENNI_FOUND})

mark_as_advanced(OpenNI_LIBRARY OpenNI_LIBRARY OpenNI_INCLUDE_DIR)
