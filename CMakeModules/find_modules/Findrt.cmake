# This module searches for the Realtime Extensions library
# and defines the following variables:
# rt_FOUND   : 1 if libdl was found, 0 otherwise
# rt_LIBRARY : libdl location
# rt_LIBRARIES

include(FindPackageHandleStandardArgs)

find_library(rt_LIBRARY rt)

find_package_handle_standard_args(rt DEFAULT_MSG rt_LIBRARY)
# Hack: since the macro makes the package name uppercase
set(rt_FOUND ${RT_FOUND})
set(rt_LIBRARIES ${rt_LIBRARY})

mark_as_advanced(rt_LIBRARY rt_LIBRARIES)
