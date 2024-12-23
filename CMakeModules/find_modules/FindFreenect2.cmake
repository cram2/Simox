# - Try to find Freenect2
# Once done this will define
#
#  Freenect2_FOUND - system has Freenect2
#  Freenect2_INCLUDE_DIRS - the Freenect2 include directory
#  Freenect2_LIBRARIES - Link these to use Freenect2


include(FindPackageHandleStandardArgs)

find_path(Freenect2_lib_INCLUDE_DIRS
        NAMES libfreenect2/libfreenect2.hpp
        HINTS
            /usr/local/include/libfreenect2/
            /usr/include/libfreenect2
            /usr/local/include/
            /usr/include/
        PATHS
            $ENV{Freenect2_DIR}/include
            $ENV{Freenect2_DIR}/src
        )

find_path(Freenect2_build_INCLUDE_DIRS
        NAMES libfreenect2/config.h
        PATHS
        $ENV{Freenect2_DIR}/build
        )

set(Freenect2_INCLUDE_DIRS "${Freenect2_lib_INCLUDE_DIRS};${Freenect2_build_INCLUDE_DIRS}")

find_library(Freenect2_LIBRARIES
        NAMES freenect2
        PATHS
        $ENV{Freenect2_DIR}/build/lib
            )

find_package_handle_standard_args(Freenect2 DEFAULT_MSG Freenect2_LIBRARIES Freenect2_INCLUDE_DIRS)

# Hack: since the macro makes the package name uppercase
set(Freenect2_FOUND ${FREENECT2_FOUND})

mark_as_advanced(Freenect2_LIBRARIES Freenect2_LIBRARIES Freenect2_INCLUDE_DIRS)