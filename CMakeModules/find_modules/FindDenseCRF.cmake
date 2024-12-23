# - Try to find denseCRF
# Once done this will define
#
#  DenseCRF_FOUND - system has DenseCRF
#  DenseCRF_INCLUDE_DIRS - the DenseCRF include directory
#  DenseCRF_LIBRARIES - Link these to use DenseCRF


include(FindPackageHandleStandardArgs)

find_path(DenseCRF_INCLUDE_DIRS
        NAMES densecrf.h permutohedral.h unary.h pairwise.h
        HINTS
        /usr/local/include/densecrf/
        /usr/include/densecrf
        /usr/local/include/
        /usr/include/
        PATHS
        $ENV{DenseCRF_DIR}/include
        ${DenseCRF_DIR}/include
        )

find_library(DenseCRF_LIBRARIES
        NAMES densecrf
        PATHS
        $ENV{DenseCRF_DIR}/build/src
        ${DenseCRF_DIR}/build/src
        )

find_package_handle_standard_args(DenseCRF DEFAULT_MSG DenseCRF_LIBRARIES DenseCRF_INCLUDE_DIRS)

# Hack: since the macro makes the package name uppercase
set(DenseCRF_FOUND ${DENSECRF_FOUND})

mark_as_advanced(DenseCRF_LIBRARIES DenseCRF_LIBRARIES DenseCRF_INCLUDE_DIRS)