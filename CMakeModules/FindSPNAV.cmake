# This file was copied from the ROS project (https://github.com/ros-drivers) on 2024-12-19.
# Original author(s): Nils Schulte, Chris Lalancette.
#
# Licensed under the BSD License:
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions, and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions, and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software without
#    specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#*******************************************************************************
#
# Find the spnav library and header.
#
# Sets the usual variables expected for find_package scripts:
#
# spnav_INCLUDE_DIR - header location
# spnav_LIBRARIES - library to link against
# spnav_FOUND - true if pugixml was found.

if(UNIX)

  find_path(spnav_INCLUDE_DIR spnav.h)

  find_library(spnav_LIBRARY
    NAMES
    spnav libspnav
)

# Support the REQUIRED and QUIET arguments, and set spnav_FOUND if found.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SPNAV DEFAULT_MSG
  spnav_LIBRARY
  spnav_INCLUDE_DIR)

if(spnav_FOUND)
  set(spnav_LIBRARIES ${spnav_LIBRARY})
endif()

mark_as_advanced(
  spnav_LIBRARY
  spnav_INCLUDE_DIR)

endif()
