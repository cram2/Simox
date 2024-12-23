# This module defines the following variables:
# dl_FOUND   : 1 if libdl was found, 0 otherwise
# dl_LIBRARY : libdl location
# dl_LIBRARIES
# dl_INCLUDE_DIRS: directory where dlfcn.h can be found

include(FindPackageHandleStandardArgs)

find_library(dl_LIBRARY dl)
find_path(dl_INCLUDE_DIRS dlfcn.h)

find_package_handle_standard_args(dl DEFAULT_MSG dl_LIBRARY dl_INCLUDE_DIRS)
# Hack: since the macro makes the package name uppercase
set(dl_FOUND ${DL_FOUND})
set(dl_LIBRARIES ${dl_LIBRARY})

mark_as_advanced(dl_LIBRARY dl_LIBRARIES dl_INCLUDE_DIRS)
