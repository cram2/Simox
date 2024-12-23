if (jsoncpp_INCLUDE_DIR AND jsoncpp_LIBRARIES)
    # in cache already
    set(jsoncpp_FOUND TRUE)
else()
    find_package(PkgConfig QUIET)
    if(PKG_CONFIG_FOUND)
        pkg_check_modules(jsoncpp jsoncpp QUIET)
        if(jsoncpp_FOUND)
            message(STATUS "jsoncpp-LIBRARIES: ${jsoncpp_LIBRARIES}")
            message(STATUS "jsoncpp-Include: ${jsoncpp_INCLUDE_DIRS}")
        endif()
    endif()

endif()
