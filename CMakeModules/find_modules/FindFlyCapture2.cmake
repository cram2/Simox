# This module defines the following variables:
# FlyCapture2_FOUND   : 1 if FlyCapture2 was found, 0 otherwise
# FlyCapture2_LIBRARIES : FlyCapture2 location
# FlyCapture2_INCLUDE_DIRS: directory where the headers can be found

include(FindPackageHandleStandardArgs)

find_library(FlyCapture2_LIBRARIES
    Names flycapture
    HINTS
        ${FlyCapture2_LIBRARY_DIRS}
        /usr/lib
    PATHS
        /usr/lib
        $ENV{FlyCapture2_DIR}
    PATH_SUFFIXES
        lib
        lib64
    NO_DEFAULT_PATH
)
find_path(FlyCapture2_INCLUDE_DIRS
    flycapture/FlyCapture2.h
    HINTS
        ${FlyCapture2_INCLUDE_DIRS}
    PATHS
        /usr/include
        $ENV{FlyCapture2_DIR}
    PATH_SUFFIXES
        flycapture
        include
    NO_DEFAULT_PATH
)


find_package_handle_standard_args(FlyCapture2 DEFAULT_MSG FlyCapture2_LIBRARIES FlyCapture2_INCLUDE_DIRS)

# Hack: since the macro makes the package name uppercase
set(FlyCapture2_FOUND ${FLYCAPTURE2_FOUND})

mark_as_advanced(FlyCapture2_LIBRARIES FlyCapture2_LIBRARIES FlyCapture2_INCLUDE_DIRS)
