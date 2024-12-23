# This module defines the following variables:
# Lcov_FOUND     : 1 if lcov and genhtml are found, 0 otherwise
# Lcov_EXECUTABLE    : lcov location
# Lcov_GenHtml_EXECUTABLE : genhtml location

include(FindPackageHandleStandardArgs)

find_program(Lcov_EXECUTABLE lcov)
find_program(Lcov_GenHtml_EXECUTABLE genhtml)

find_package_handle_standard_args(Lcov DEFAULT_MSG Lcov_EXECUTABLE Lcov_GenHtml_EXECUTABLE)
# Hack: since the macro makes the package name uppercase
set(Lcov_FOUND ${LCOV_FOUND})

mark_as_advanced(Lcov_EXECUTABLE Lcov_GenHtml_EXECUTABLE)
