
ADD_VR_TEST( VirtualRobotExceptionTest )
ADD_VR_TEST( VirtualRobotChecksTest )

ADD_VR_TEST( VirtualRobotRobotTest )

ADD_VR_TEST( VirtualRobotTransformationTest )

if (Simox_VISUALIZATION)
    ADD_VR_TEST( VirtualRobotCollisionTest )
endif()

ADD_VR_TEST( VirtualRobotJacobianTest )

ADD_VR_TEST( VirtualRobotLinkedCoordinateTest )

ADD_VR_TEST( VirtualRobotSceneTest )

ADD_VR_TEST( VirtualRobotConfigTest )

ADD_VR_TEST( VirtualRobotMathToolsTest )

ADD_VR_TEST( VirtualRobotThreadsafetyTest )
ADD_VR_TEST( VirtualRobotTrajectoryTest )
ADD_VR_TEST( VirtualRobotWorkSpaceTest )
ADD_VR_TEST( VirtualRobotWorkSpaceGridTest )
ADD_VR_TEST( VirtualRobotSensorTest )
ADD_VR_TEST( VirtualRobotIOTest )
ADD_VR_TEST( VirtualRobotGazeIKTest )
ADD_VR_TEST( VirtualRobotMeshImportTest )

ADD_VR_TEST( ManipulationObjectGlobalPoseTest )
ADD_VR_TEST( ManipulationObjectAffordancesTest )

ADD_VR_TEST( VirtualRobotTimeOptimalTrajectoryTest )

ADD_VR_TEST( VirtualRobotRuntimeEnvironmentTest )

ADD_VR_TEST( VirtualRobotMjcfTest )

ADD_VR_TEST( VirtualRobotManipulabilityTest )


ADD_VR_TEST( VirtualRobotMujocoMeshTest )

configure_file(VirtualRobotMujocoMeshTestMesh.msh
    ${Simox_TEST_DIR}/VirtualRobotMujocoMeshTestMesh.msh COPYONLY)
target_compile_definitions(VirtualRobotMujocoMeshTest PRIVATE
    TEST_MESH_FILE="${Simox_TEST_DIR}/VirtualRobotMujocoMeshTestMesh.msh")


ADD_VR_TEST( MathFitPlaneTest )
ADD_VR_TEST( MathGaussianImplicitSurface3DNormalsTest )
ADD_VR_TEST( MathGaussianImplicitSurface3DTest )
ADD_VR_TEST( MathHelpersTest )
ADD_VR_TEST( PQP_optimization )

ADD_VR_TEST( MecanumPlatformKinematicsTest )
ADD_VR_TEST( OmniPlatformKinematicsTest )
