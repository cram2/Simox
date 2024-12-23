# This module defines the following variables:
# CppCheck_FOUND      : 1 if cppcheck was found, 0 otherwise
# CppCheck_EXECUTABLE : cppcheck executable location

include(FindPackageHandleStandardArgs)

find_program(CppCheck_EXECUTABLE cppcheck)

find_package_handle_standard_args(CppCheck DEFAULT_MSG CppCheck_EXECUTABLE)
# Hack: since the macro makes the package name uppercase
set(CppCheck_FOUND ${CPPCHECK_FOUND})

mark_as_advanced(CppCheck_EXECUTABLE)

