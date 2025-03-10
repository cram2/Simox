/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2013 Nikolaus Vahrenkamp
*/

#define BOOST_TEST_MODULE VirtualRobot_VirtualRobotIOTest

#include <string>

#include <VirtualRobot/ManipulationObject.h>
#include <VirtualRobot/Nodes/PositionSensor.h>
#include <VirtualRobot/Nodes/Sensor.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RuntimeEnvironment.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/XML/ObjectIO.h>
#include <VirtualRobot/XML/RobotIO.h>


using namespace VirtualRobot;

BOOST_AUTO_TEST_SUITE(VirtualRobotIO)

BOOST_AUTO_TEST_CASE(testRobotLoadXML)
{
    std::string filename = "robots/ArmarIII/ArmarIII.xml";
    bool fileOK = RuntimeEnvironment::getDataFileAbsolute(filename);
    BOOST_REQUIRE(fileOK);

    RobotPtr r;
    BOOST_REQUIRE_NO_THROW(r = RobotIO::loadRobot(filename));
    BOOST_REQUIRE(r);

    std::vector<RobotNodePtr> rn = r->getRobotNodes();
    BOOST_REQUIRE_GT(rn.size(), 0);

    std::vector<EndEffectorPtr> eefs = r->getEndEffectors();
    BOOST_REQUIRE_GT(eefs.size(), 0);

    std::vector<RobotNodeSetPtr> rns = r->getRobotNodeSets();
    BOOST_REQUIRE_GT(rns.size(), 0);
}

BOOST_AUTO_TEST_CASE(testRobotSaveXML)
{
    std::string filename = "robots/ArmarIII/ArmarIII.xml";
    bool fileOK = RuntimeEnvironment::getDataFileAbsolute(filename);
    BOOST_REQUIRE(fileOK);

    RobotPtr r;
    BOOST_REQUIRE_NO_THROW(r = RobotIO::loadRobot(filename));
    BOOST_REQUIRE(r);

    std::error_code ec;
    std::filesystem::path tempDir = std::filesystem::temp_directory_path(ec);
    BOOST_REQUIRE(ec == std::error_code{});

    std::filesystem::path robName("ArmarIII_tmp.xml");
    std::filesystem::path filenameTmp = tempDir / robName;

    bool saveOK{false};
    BOOST_REQUIRE_NO_THROW(saveOK = RobotIO::saveXML(r, robName.string(), tempDir.string()));
    BOOST_REQUIRE(saveOK);

    //reload robot
    RobotPtr r2;
    BOOST_REQUIRE_NO_THROW(r2 = RobotIO::loadRobot(filenameTmp.string()));
    BOOST_REQUIRE(r2);
}

BOOST_AUTO_TEST_CASE(testLoadStoreManipulationObjectPhysics)
{
    std::string filename("objects/physics-test.xml");
    bool fileOK = RuntimeEnvironment::getDataFileAbsolute(filename);
    BOOST_REQUIRE(fileOK);

    ManipulationObjectPtr manipulatioObject = ObjectIO::loadManipulationObject(filename);

    SceneObject::Physics physicsObject = manipulatioObject->getPhysics();

    BOOST_CHECK_EQUAL(physicsObject.simType, SceneObject::Physics::eStatic);
    BOOST_CHECK_CLOSE(physicsObject.massKg, 0.0, 0.0001);
    BOOST_CHECK_EQUAL(physicsObject.comLocation, SceneObject::Physics::eVisuBBoxCenter);


// This causes the test to get stuck. ToDo: Fix it and re-enable.
#if 0
    ManipulationObjectPtr savedObject = ObjectIO::createManipulationObjectFromString(manipulatioObject->toXML());
    physicsObject = savedObject->getPhysics();
#endif

    BOOST_CHECK_EQUAL(physicsObject.simType, SceneObject::Physics::eStatic);
    BOOST_CHECK_CLOSE(physicsObject.massKg, 0.0, 0.0001);
    BOOST_CHECK_EQUAL(physicsObject.comLocation, SceneObject::Physics::eVisuBBoxCenter);
}


BOOST_AUTO_TEST_SUITE_END()
