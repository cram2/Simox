PROJECT ( GeometricPlanning )

# cmake_policy(SET CMP0076 NEW)

MESSAGE (STATUS "***** CONFIGURING Simox project GeometricPlanning *****")

find_package(Boost ${Simox_BOOST_VERSION} EXACT COMPONENTS unit_test_framework REQUIRED)

find_package(doctest QUIET)

########################### TESTING #####################################
MACRO(ADD_GEOMETRIC_PLANNING_TEST TEST_NAME)
    ADD_EXECUTABLE(${TEST_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/${TEST_NAME}.cpp)
    target_include_directories(${TEST_NAME} PRIVATE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/..)
    if (NOT Boost_USE_STATIC_LIBS)
        target_compile_definitions(${TEST_NAME} PRIVATE -DBOOST_TEST_DYN_LINK)
    endif ()
    TARGET_LINK_LIBRARIES(${TEST_NAME} PRIVATE VirtualRobot GeometricPlanning Boost::unit_test_framework)
    SET_TARGET_PROPERTIES(${TEST_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_TEST_DIR})
    SET_TARGET_PROPERTIES(${TEST_NAME} PROPERTIES FOLDER "GeometricPlanning Tests")
    ADD_TEST(NAME GeometricPlanning_${TEST_NAME}
    	        COMMAND ${Simox_TEST_DIR}/${TEST_NAME} --output_format=XML --log_level=all --report_level=no)
ENDMACRO(ADD_GEOMETRIC_PLANNING_TEST)

#################################### FILES ##################################
SET(SOURCES
    ParametricPath.cpp
    ArticulatedObjectGeometricPlanningHelper.cpp
    ArticulatedObjectDoorHelper.cpp
    path_primitives/PathPrimitive.cpp
    path_primitives/Circle.cpp
    path_primitives/CircleSegment.cpp
    path_primitives/Line.cpp
    visualization.cpp
    util.cpp
)

SET(HEADERS
    types.h
    ParametricPath.h
    ArticulatedObjectGeometricPlanningHelper.h
    ArticulatedObjectDoorHelper.h
    path_primitives/PathPrimitive.h
    path_primitives/Circle.h
    path_primitives/CircleSegment.h
    path_primitives/Line.h
    assert/assert.h
    visualization.h
    util.h
)

ADD_LIBRARY(GeometricPlanning SHARED) # sources will be added later

if(doctest_FOUND)
    message(STATUS "Using doctest")
    list(APPEND SOURCES
        assert/doctest/impl.cpp # required for doctest in general
        assert/doctest/handler.cpp # required for out-of-test assertions
    )

    list(APPEND HEADERS
        assert/doctest/assert.h
    )

    TARGET_LINK_LIBRARIES (GeometricPlanning PUBLIC doctest::doctest )
    target_compile_definitions (GeometricPlanning PUBLIC -DUSE_DOCTEST )

else()
    message(STATUS "doctest not found. Using VirtualRobot assertions.")

    list(APPEND SOURCES
        assert/virtual_robot/assert.cpp
    )

    list(APPEND HEADERS
        assert/virtual_robot/assert.h
    )

endif()


if (Simox_USE_COIN_VISUALIZATION)
	# SET(SOURCES
	# ${SOURCES}
	# Visualization/CoinVisualization/CoinRrtWorkspaceVisualization.cpp
	# )

	# SET(HEADERS
	# ${HEADERS}
	# Visualization/CoinVisualization/CoinRrtWorkspaceVisualization.h
	# )
endif ()

# target_sources(GeometricPlanning
#     PUBLIC ${HEADERS}
#     PRIVATE ${SOURCES}
# )

set_target_properties(GeometricPlanning PROPERTIES SOURCES "${SOURCES};${HEADERS}")


#MESSAGE(STATUS "VirtualRobot_ROBOT_LINK_LIBRARIES:" ${VirtualRobot_EXTERNAL_LIBRARIES})
TARGET_LINK_LIBRARIES (GeometricPlanning PUBLIC VirtualRobot)

# .DLL path
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_BIN_DIR})
# .so path
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${Simox_LIB_DIR})
# .lib path (this is needed for setting the DLL-import library path on windows)
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${Simox_LIB_DIR})

if(Simox_BUILD_EXAMPLES)
    # include examples
    # ADD_SUBDIRECTORY(examples/)
endif()

if(BUILD_TESTING)
    # include unit tests
    # ADD_SUBDIRECTORY(tests/)
endif()

#######################################################################################
############################ Setup for installation ###################################
#######################################################################################

install(TARGETS ${PROJECT_NAME}
  # IMPORTANT: Add the library to the "export-set"
  EXPORT SimoxTargets
  RUNTIME DESTINATION bin COMPONENT bin
  LIBRARY DESTINATION lib COMPONENT shlib
  ARCHIVE DESTINATION lib COMPONENT library
  COMPONENT dev)

INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR} DESTINATION ${Simox_INSTALL_HEADER_DIR}
	COMPONENT Headers
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        PATTERN ".git" EXCLUDE
        PATTERN "CMakeModules" EXCLUDE
        PATTERN "tests" EXCLUDE
        PATTERN "build*" EXCLUDE
        PATTERN "data" EXCLUDE
        PATTERN "examples" EXCLUDE
	)
MESSAGE (STATUS "***** Finished CONFIGURING Simox project GeometricPlanning *****\n")
