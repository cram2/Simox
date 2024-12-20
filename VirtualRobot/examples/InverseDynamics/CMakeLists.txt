PROJECT ( InverseDynamics )

ADD_EXECUTABLE(${PROJECT_NAME} InverseDynamics.cpp)
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_BIN_DIR})
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES FOLDER "Examples")

target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${RBDL_INCLUDE_DIR})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} VirtualRobot ${RBDL_LIBRARY})


#######################################################################################
############################ Setup for installation ###################################
#######################################################################################

install(TARGETS ${PROJECT_NAME}
  # IMPORTANT: Add the library to the "export-set"
  EXPORT SimoxTargets
  RUNTIME DESTINATION bin COMPONENT bin
  COMPONENT dev)

MESSAGE( STATUS " ** Simox application ${PROJECT_NAME} will be placed into " ${Simox_BIN_DIR})
MESSAGE( STATUS " ** Simox application ${PROJECT_NAME} will be installed into " ${INSTALL_BIN_DIR})
