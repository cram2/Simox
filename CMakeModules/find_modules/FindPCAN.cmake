# This module defines the following variables:
# PCAN_FOUND : 1 if Ice is found, 0 otherwise
# PCAN_DIR  : path where to find include, lib, bin, etc.
# PCAN_INCLUDE_DIR
# PCAN_LIBRARY
# PCAN_LIBRARIES

set (PCAN_FOUND OFF CACHE BOOL "Do we have PCAN driver installed?")


find_path (PCAN_HOME include/pcan.h
    # installation selected by user
    /usr/
    /usr/local/
    ${PCAN_DIR}
    $ENV{PCAN_DIR}
    ${SFB_REFPATH}
)


if (PCAN_HOME)
    find_library (PCAN_LIBRARY NAMES pcan PATHS ${PCAN_HOME}/lib /usr/lib/ /usr/loca/lib/)

    if (PCAN_HOME AND PCAN_LIBRARY)
          set(PCAN_FOUND ON CACHE BOOL "Do we have PCAN driver installed?" FORCE)
          set(PCAN_INCLUDE_DIR ${PCAN_HOME}/include)
          set(PCAN_LIBRARIES ${PCAN_LIBRARY})
    endif()

    include (FindPackageHandleStandardArgs)
    find_package_handle_standard_args (PCAN DEFAULT_MSG PCAN_INCLUDE_DIR PCAN_LIBRARY)

    mark_as_advanced (PCAN_INCLUDE_DIR PCAN_LIBRARIES PCAN_LIBRARY PCAN_LIBRARIES PCAN_HOME)
endif()
