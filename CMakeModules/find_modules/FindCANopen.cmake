# This module defines the following variables:
# CANopen_FOUND : 1 if Ice is found, 0 otherwise
# CANopen_HOME  : path where to find include, lib, bin, etc.
# CANopen_INCLUDE_DIR
# CANopen_LIB

#
# CANopen for C++
#

set (CANopen_FOUND 0 CACHE BOOL "Do we have CANopen?")

if (ARMARX_OS_LINUX)
    set (SFB_REFPATH /org/share/archive/SFB588_RefDist/CANopen/linux)
else()
    set (SFB_REFPATH /org/share/archive/SFB588_RefDist/CANopen/win32)
endif()

find_path (CANopen_HOME include/can.h
    # installation selected by user
    ${CANOPENDIR}
    $ENV{CANOPENDIR}
    ${SFB_REFPATH}
)

if (CANopen_HOME)
    find_library (CANopen_LIB NAMES CANopen PATHS ${CANopen_HOME}/bin)

    if (CANopen_HOME AND CANopen_LIB)
          set (CANopen_FOUND TRUE)
          set (CANopen_INCLUDE_DIR ${CANopen_HOME}/include)
    endif()

    include (FindPackageHandleStandardArgs)
    find_package_handle_standard_args (CANopen DEFAULT_MSG CANopen_INCLUDE_DIR)

    mark_as_advanced (CANopen_INCLUDE_DIR)
endif()
