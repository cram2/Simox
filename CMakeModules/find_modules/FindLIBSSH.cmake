# - Try to find LibSSH
# Once done this will define
#
#  LIBSSH_FOUND - system has LibSSH
#  LIBSSH_INCLUDE_DIRS - the LibSSH include directory
#  LIBSSH_LIBRARY_DIR - the LibSSH library directory
#
#  Copyright (c) 2009 Andreas Schneider <asn@cryptomilk.org>
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#  1. Redistributions of source code must retain the copyright
#     notice, this list of conditions and the following disclaimer.
#  2. Redistributions in binary form must reproduce the copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#  3. The name of the author may not be used to endorse or promote products
#     derived from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
#  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
#  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
#  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
#  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
#  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

if (LIBSSH_LIBRARY_DIR AND LIBSSH_INCLUDE_DIRS)
  # in cache already
  set(LIBSSH_FOUND TRUE)
else ()

  find_path(LIBSSH_INCLUDE_DIR
    NAMES
      libssh/libssh.h
    PATHS
      /usr/include
      /usr/local/include
      /opt/local/include
      /sw/include
      ${CMAKE_INCLUDE_PATH}
      ${CMAKE_INSTALL_PREFIX}/include
  )
  set(LIBSSH_INCLUDE_DIRS
    ${LIBSSH_INCLUDE_DIR}
  )
  message(STATUS "LIBSSH include: ${LIBSSH_INCLUDE_DIRS}")

  find_library(LIBSSH_LIBRARY
    NAMES
      ssh.so
      libssh.so
    PATHS
      /usr/lib
      /usr/local/lib
      /opt/local/lib
      /sw/lib
      ${CMAKE_LIBRARY_PATH}
      ${CMAKE_INSTALL_PREFIX}/lib
  )
  message(STATUS "LIBSSH library: ${LIBSSH_LIBRARY}")

  if (LIBSSH_INCLUDE_DIR AND LIBSSH_LIBRARY)
    set(SSH_FOUND TRUE)
  endif ()


  if (SSH_FOUND)
    string(REPLACE "libssh.so" ""
      LIBSSH_LIBRARY_DIR
      ${LIBSSH_LIBRARY}
    )
    string(REPLACE "ssh.so" ""
      LIBSSH_LIBRARY_DIR
      ${LIBSSH_LIBRARY_DIR}
    )
  endif ()
  message(STATUS "LIBSSH library dir: ${LIBSSH_LIBRARY_DIR}")

  # find the library with previously defined arguments
  find_package_handle_standard_args(LibSSH DEFAULT_MSG LIBSSH_LIBRARY_DIR LIBSSH_INCLUDE_DIRS)

  # show the LIBSSH_INCLUDE_DIRS and LIBSSH_LIBRARY_DIR variables only in the advanced view
  mark_as_advanced(LIBSSH_INCLUDE_DIRS LIBSSH_LIBRARY_DIR LIBSSH_LIBRARY)

  # Make a target libssh which automatically adds the include directory and the library to link to
  if (LIBSSH_FOUND)
    add_library(libssh INTERFACE IMPORTED)
    set_property(TARGET libssh PROPERTY
        INTERFACE_INCLUDE_DIRECTORIES ${LIBSSH_INCLUDE_DIRS})
    set_property(TARGET libssh PROPERTY
        INTERFACE_LINK_LIBRARIES ${LIBSSH_LIBRARY})
  endif()

endif ()
