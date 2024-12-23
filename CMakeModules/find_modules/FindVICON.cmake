find_path(VICON_INCLUDE_DIR Client.h
    "/org/share/archive/HumanoidsLibs/install/12.04/64bit/ViconSDK/"
)

MESSAGE(STATUS "VICON_INCLUDE_DIR " ${VICON_INCLUDE_DIR})

find_library(VICON_LIBRARY libViconDataStreamSDK_CPP.so
    "/org/share/archive/HumanoidsLibs/install/12.04/64bit/ViconSDK/"
)

MESSAGE(STATUS "VICON_LIBRARY " ${VICON_LIBRARY})

include (FindPackageHandleStandardArgs)
find_package_handle_standard_args (
   VICON
   DEFAULT_MSG
   VICON_INCLUDE_DIR
   VICON_LIB
)

if( VICON_LIBRARY AND VICON_INCLUDE_DIR)
    set( VICON_FOUND TRUE )
endif()
