# This module defines the following variables:
# multisense_lib_FOUND   : 1 if LibMultiSense was found, 0 otherwise
# multisense_lib_LIBRARIES : LibMultiSense location
# multisense_lib_INCLUDE_DIRS: directory where the headers can be found

include(FindPackageHandleStandardArgs)


find_library(multisense_lib_LIBRARIES
    Names MultiSense
    PATHS
        /usr/lib
        /usr/local/lib/
        $ENV{multisense_DIR}/bin
        $ENV{multisense_DIR}/lib
)
find_path(multisense_lib_INCLUDE_DIRS
    MultiSense/MultiSenseTypes.hh
    PATHS
        /usr/include/
        $ENV{multisense_DIR}/bin
        $ENV{multisense_DIR}/source
        $ENV{multisense_DIR}/include
)

find_package_handle_standard_args(multisense_lib DEFAULT_MSG multisense_lib_LIBRARIES multisense_lib_INCLUDE_DIRS)

# Hack: since the macro makes the package name uppercase
set(multisense_lib_FOUND ${MULTISENSE_LIB_FOUND})

mark_as_advanced(multisense_lib_LIBRARIES multisense_lib_LIBRARIES multisense_lib_INCLUDE_DIRS)
