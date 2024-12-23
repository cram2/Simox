# - Try to find Freenect
# Once done this will define
#
#  Freenect_FOUND - system has Freenect
#  Freenect_INCLUDE_DIRS - the Freenect include directory
#  Freenect_LIBRARIES - Link these to use Freenect


include(FindPackageHandleStandardArgs)

find_path(Freenect_wrapper_INCLUDE_DIRS
        NAMES libfreenect.hpp
        PATHS
            $ENV{Freenect_DIR}/wrappers/cpp
        )

find_path(Freenect_header_INCLUDE_DIRS
        NAMES libfreenect.h
        PATHS
        $ENV{Freenect_DIR}/include
        )

set(Freenect_INCLUDE_DIRS "${Freenect_wrapper_INCLUDE_DIRS};${Freenect_header_INCLUDE_DIRS}")

find_library(Freenect_sync_LIBRARY
        NAMES freenect_sync
        PATHS
        $ENV{Freenect_DIR}/build/lib
            )

find_library(Freenect_LIBRARY
        NAMES freenect freenect_sync
        PATHS
        $ENV{Freenect_DIR}/build/lib
        )
set(Freenect_LIBRARIES "${Freenect_LIBRARY};${Freenect_sync_LIBRARY}")

find_package_handle_standard_args(Freenect DEFAULT_MSG Freenect_LIBRARIES Freenect_INCLUDE_DIRS)

# Hack: since the macro makes the package name uppercase
set(Freenect_FOUND ${FREENECT_FOUND})

mark_as_advanced(Freenect_LIBRARIES Freenect_LIBRARIES Freenect_INCLUDE_DIRS)