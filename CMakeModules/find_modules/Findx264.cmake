# This module defines the following variables:
# x264_FOUND   : 1 if x264 was found, 0 otherwise
# x264_LIBRARIES : x264 location
# x264_INCLUDE_DIRS: directory where the headers can be found

include(FindPackageHandleStandardArgs)

find_library(x264_LIBRARIES Names libx264.so HINTS ${x264_LIBRARY_DIRS} /usr/lib /usr/lib/x86_64-linux-gnu PATHS /usr/lib  PATH_SUFFIXES lib lib64 NO_DEFAULT_PATH)
find_path(x264_INCLUDE_DIRS x264.h HINTS ${x264_INCLUDE_DIRS}  PATHS /usr/include PATH_SUFFIXES flycapture NO_DEFAULT_PATH )

find_package_handle_standard_args(x264 DEFAULT_MSG x264_LIBRARIES x264_INCLUDE_DIRS)
#message("Looking for x264: ${x264_INCLUDE_DIRS} ${x264_LIBRARIES} ${X264_FOUND} - ${x264_FOUND}")

# Hack: since the macro makes the package name uppercase
set(x264_FOUND ${X264_FOUND})

mark_as_advanced(X264_FOUND x264_FOUND x264_LIBRARIES x264_INCLUDE_DIRS)
