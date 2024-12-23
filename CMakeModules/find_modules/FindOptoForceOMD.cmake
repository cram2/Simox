# - Try to find OptoForceOMD
# Once done this will define
#
#  OptoForceOMD_FOUND - OptoForceOMD found
#  OptoForceOMD_INCLUDE_DIR - the OptoForceOMD include directory
#  OptoForceOMD_LIBRARIES - OptoForceOMD library
#

FIND_PATH(OptoForceOMD_INCLUDE_DIR NAMES opto.h
  PATHS
  $ENV{OptoForceOMD_DIR}/include/
  $ENV{OptoForceOMD_DIR}/include/OptoForceOMD/
  ${OptoForceOMD_DIR}/include/
  ${OptoForceOMD_DIR}/include/OptoForceOMD/
  ENV CPATH
  /usr/include/OptoForceOMD/
  /usr/local/include/OptoForceOMD/
  /opt/local/include/OptoForceOMD/
  NO_DEFAULT_PATH
)


FIND_LIBRARY(OptoForceOMD_LIBRARIES NAMES libOMD.so
  PATHS
  $ENV{OptoForceOMD_DIR}/lib
  ${OptoForceOMD_DIR}/lib
  ENV LD_LIBRARY_PATH
  ENV LIBRARY_PATH
  /usr/lib
  /usr/local/lib
  /opt/local/lib
  NO_DEFAULT_PATH
)

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set OODL_YOUBOT_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(OptoForceOMD DEFAULT_MSG
                                  OptoForceOMD_LIBRARIES OptoForceOMD_INCLUDE_DIR)

set(OptoForceOMD_FOUND ${OPTOFORCEOMD_FOUND}) # CMake UPPERCASE-FUNTIME!

#message( "OptoForceOMD_FOUND:" ${OptoForceOMD_FOUND})
#message( "OPTOFORCEOMD_FOUND:" ${OPTOFORCEOMD_FOUND})
#message( "OptoForceOMD_LIBRARIES:" ${OptoForceOMD_LIBRARIES})
#message( "OptoForceOMD_INCLUDE_DIR:" ${OptoForceOMD_INCLUDE_DIR})

# show the OptoForceOMD_INCLUDE_DIR and OptoForceOMD_LIBRARY_DIR variables only in the advanced view
MARK_AS_ADVANCED(OptoForceOMD_INCLUDE_DIR OptoForceOMD_LIBRARIES)
