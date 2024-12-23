# This module defines the following variables:
# swscale_FOUND   : 1 if swscale was found, 0 otherwise
# swscale_LIBRARIES : swscale location
# swscale_INCLUDE_DIRS: directory where the headers can be found

include(FindPackageHandleStandardArgs)

find_library(swscale_LIBRARIES
    Names swscale
    HINTS ${swscale_LIBRARY_DIRS}
    PATHS
        /usr/lib
        /usr/lib/x86_64-linux-gnu
    PATH_SUFFIXES lib lib64
    NO_DEFAULT_PATH)

find_path(swscale_INCLUDE_DIRS
    libswscale/swscale.h
    HINTS ${swscale_INCLUDE_DIRS}
    PATHS
        /usr/include/
        /usr/include/x86_64-linux-gnu/
    NO_DEFAULT_PATH )

#message("Looking for x264: ${swscale_INCLUDE_DIRS} ${swscale_LIBRARIES} ${SWSCALE_FOUND} - ${swscale_FOUND}")

find_package_handle_standard_args(swscale DEFAULT_MSG swscale_LIBRARIES swscale_INCLUDE_DIRS)

# Hack: since the macro makes the package name uppercase
set(swscale_FOUND ${SWSCALE_FOUND})

mark_as_advanced(swscale_LIBRARIES swscale_LIBRARIES swscale_INCLUDE_DIRS)
