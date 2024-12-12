
# the variable "SRCS" contains all .cpp files of this project
FILE(GLOB SRCS ${PROJECT_SOURCE_DIR}/*.cpp)

# check for requested collision detection wrappers
IF (Simox_COLLISION_CHECKER_PQP)
    MESSAGE (STATUS " ** COLLISION DETECTION: PQP")
    SET (COL_CHECKER_WRAPPER_DIR ${PROJECT_SOURCE_DIR}/CollisionDetection/PQP)
    SET(VR_COLLISION_DETECTION_PQP 1)
    SET(COLLISIONDETECTION_LIB ColCheckerPQP)
    SET(COLLISIONDETECTION_DIR ${COL_CHECKER_WRAPPER_DIR}/PQP++)
    SET(COLLISIONDETECTION_INC_DIR ${COLLISIONDETECTION_DIR})


    ############################################################################
    #inline pqp
    FILE(GLOB PQP_SRCS ${COLLISIONDETECTION_DIR}/*.cpp)
    FILE(GLOB PQP_INCS ${COLLISIONDETECTION_DIR}/*.h)
    SET (SOURCES ${SOURCES} ${PQP_SRCS})
    SET (INCLUDES ${INCLUDES} ${PQP_INCS})
    target_compile_definitions(VirtualRobot PUBLIC -DUSE_PQP)
    ############################################################################

        # create PQP lib
ELSE()
    # DUMMY
    MESSAGE (STATUS "WARNING: NO LIBRARY DEFINED FOR COLLISION DETECTION: COLLISION DETECTION IS DISABLED!")
    SET (COL_CHECKER_WRAPPER_DIR ${PROJECT_SOURCE_DIR}/CollisionDetection/Dummy)
    SET(COLLISIONDETECTION_DIR ${Simox_SOURCE_DIR}/VirtualRobot/CollisionDetection/Dummy)
    SET(COLLISIONDETECTION_INC_DIR ${COLLISIONDETECTION_DIR})
ENDIF ()
MESSAGE (STATUS " ** Directory of collision detection wrapper: " ${COL_CHECKER_WRAPPER_DIR})

FILE (GLOB SRCS_COL ${COL_CHECKER_WRAPPER_DIR}/*.cpp)
FILE (GLOB INCS_COL ${COL_CHECKER_WRAPPER_DIR}/*.h)

SET (SOURCES ${SOURCES} ${SRCS_COL})
SET (INCLUDES ${INCLUDES} ${INCS_COL})

target_include_directories(VirtualRobot PRIVATE ${COL_CHECKER_WRAPPER_DIR} ${COLLISIONDETECTION_INC_DIR})
