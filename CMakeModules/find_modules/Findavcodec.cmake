# This module defines the following variables:
# avcodec_FOUND   : 1 if avcodec was found, 0 otherwise
# avcodec_LIBRARIES : avcodec location
# avcodec_INCLUDE_DIRS: directory where the headers can be found

include(FindPackageHandleStandardArgs)

find_library(avcodec_LIBRARY
    Names avcodec
    HINTS ${avcodec_LIBRARY_DIRS}
    PATHS
        /usr/lib
        /usr/lib/x86_64-linux-gnu
    PATH_SUFFIXES lib lib64
    NO_DEFAULT_PATH)

find_library(avutil_LIBRARY
    Names avutil
    HINTS ${avcodec_LIBRARY_DIRS}
    PATHS
        /usr/lib
        /usr/lib/x86_64-linux-gnu
    PATH_SUFFIXES lib lib64
    NO_DEFAULT_PATH)

set(avcodec_LIBRARIES
    ${avcodec_LIBRARY}
    ${avutil_LIBRARY}
)

find_path(avcodec_INCLUDE_DIRS
    libavcodec/avcodec.h
    HINTS ${avcodec_INCLUDE_DIRS}
    PATHS
        /usr/include/
        /usr/include/x86_64-linux-gnu
    NO_DEFAULT_PATH )

find_package_handle_standard_args(avcodec DEFAULT_MSG avcodec_LIBRARIES avcodec_INCLUDE_DIRS)

# Hack: since the macro makes the package name uppercase
set(avcodec_FOUND ${AVCODEC_FOUND})
message("Looking for x264: ${avcodec_INCLUDE_DIRS} ${avcodec_LIBRARIES} ${avcodec_FOUND}")

mark_as_advanced(avcodec_LIBRARIES avcodec_LIBRARIES avcodec_INCLUDE_DIRS)
