# Copyright (c) 2015, Georgia Tech Graphics Lab and Humanoid Robotics Lab
# This file is provided under the "BSD-style" License

# Find NLOPT
#
# This sets the following variables:
# NLOPT_FOUND
# NLOPT_INCLUDE_DIRS
# NLOPT_LIBRARIES
# NLOPT_DEFINITIONS
# NLOPT_VERSION

find_package(PkgConfig QUIET)

# Check to see if pkgconfig is installed.
pkg_check_modules(PC_NLOPT nlopt QUIET)

# Definitions
set(NLOPT_DEFINITIONS ${PC_NLOPT_CFLAGS_OTHER})

# Include directories
find_path(NLOPT_INCLUDE_DIRS
    NAMES nlopt.h
    HINTS ${PC_NLOPT_INCLUDEDIR}
    PATHS "${CMAKE_INSTALL_PREFIX}/include")

# Libraries
find_library(NLOPT_LIBRARIES
    NAMES nlopt nlopt_cxx
    HINTS ${PC_NLOPT_LIBDIR})

# Version
set(NLOPT_VERSION ${PC_NLOPT_VERSION})

# Set (NAME)_FOUND if all the variables and the version are satisfied.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(NLOPT
    FAIL_MESSAGE  DEFAULT_MSG
    REQUIRED_VARS NLOPT_INCLUDE_DIRS NLOPT_LIBRARIES
    VERSION_VAR   NLOPT_VERSION)

# Create alias targets for nlopt and nlopt_cxx
# ===================================

if(NOT TARGET nlopt::nlopt)
  add_library(nlopt::nlopt UNKNOWN IMPORTED)
  set_target_properties(nlopt::nlopt PROPERTIES
    IMPORTED_LOCATION ${NLOPT_LIBRARIES}
    INTERFACE_INCLUDE_DIRECTORIES ${NLOPT_INCLUDE_DIRS}
  )
endif()

if(NOT TARGET nlopt::nlopt_cxx)
  add_library(nlopt::nlopt_cxx UNKNOWN IMPORTED)
  set_target_properties(nlopt::nlopt_cxx PROPERTIES
    IMPORTED_LOCATION ${NLOPT_LIBRARIES}
    INTERFACE_INCLUDE_DIRECTORIES ${NLOPT_INCLUDE_DIRS}
  )
endif()
