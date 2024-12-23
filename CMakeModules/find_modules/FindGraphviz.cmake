if (Graphviz_INCLUDE_DIR AND Graphviz_LIBRARIES)
    # in cache already
    set(Graphviz_FOUND TRUE)
else()
    if (NOT Graphviz_DIR)
        set(Graphviz_DIR $ENV{GRAPHVIZ_DIR})
    endif (NOT Graphviz_DIR)

    find_path(Graphviz_INCLUDE_DIR graphviz/gvc.h
        # installation selected by user
        ${Graphviz_DIR}
        ${Graphviz_DIR}/include
        /usr/include
        /usr/local/include
    )

    message(STATUS "Graphviz_DIR: ${Graphviz_DIR}")
    message(STATUS "Graphviz_Include: ${Graphviz_INCLUDE_DIR}")

    find_library(Graphviz_gvc_LIBRARY
                 NAMES gvc
                 PATHS ${Graphviz_DIR}/lib
                       /usr/lib
                       /usr/local/lib)
    find_library(Graphviz_graph_LIBRARY
                 NAMES cgraph
                 PATHS ${Graphviz_DIR}/lib
                       /usr/lib
                       /usr/local/lib)
    message(STATUS "graphviz-lib: ${Graphviz_gvc_LIBRARY} ${Graphviz_graph_LIBRARY}")

    # Order of the libraries is important!
    set(Graphviz_LIBRARIES "${Graphviz_gvc_LIBRARY}" "${Graphviz_graph_LIBRARY}")
    set(Graphviz_INCLUDE_DIRS ${Graphviz_INCLUDE_DIR})

    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args(Graphviz DEFAULT_MSG Graphviz_INCLUDE_DIR Graphviz_gvc_LIBRARY Graphviz_graph_LIBRARY Graphviz_LIBRARIES)
    set(Graphviz_FOUND ${GRAPHVIZ_FOUND})

    mark_as_advanced(Graphviz_INCLUDE_DIR Graphviz_gvc_LIBRARY Graphviz_graph_LIBRARY Graphviz_LIBRARIES)
endif()
