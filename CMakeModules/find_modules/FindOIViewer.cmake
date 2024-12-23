# This module defines the following variables:
# OIViewer_FOUND : 1 if Ice is found, 0 otherwise
# OIViewer_HOME  : path where to find include, lib, bin, etc.
# OIViewer_INCLUDE_DIR
# OIViewer_LIB

#
# OIViewer for C++
#

set (OIViewer_FOUND 0)

if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
    set (SFB_REFPATH /org/share/archive/SFB588_RefDist/x86_64)
else()
    set (SFB_REFPATH /org/share/archive/SFB588_RefDist)
endif()

find_path (OIViewer_HOME include/oiviewermain.h
    # installation selected by user
    $ENV{OIViewer_DIR}
    ${SFB_REFPATH}/OIViewer
)

if (OIViewer_HOME)
    find_library (OIViewer_LIB NAMES OIViewerPlugin PATHS ${OIViewer_HOME}/bin)

    if (OIViewer_HOME AND OIViewer_LIB)
          set (OIViewer_FOUND TRUE)
          set (OIViewer_INCLUDE_DIR ${OIViewer_HOME}/include)
    endif()

    include (FindPackageHandleStandardArgs)
    find_package_handle_standard_args (OIViewer DEFAULT_MSG OIViewer_INCLUDE_DIR)

    mark_as_advanced(OIViewer_INCLUDE_DIR)
endif()
