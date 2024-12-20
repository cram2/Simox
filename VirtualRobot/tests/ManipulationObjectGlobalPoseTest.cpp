/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2013 Nikolaus Vahrenkamp
*/

#include <boost/test/tools/old/interface.hpp>

#include "VirtualRobot.h"
#define BOOST_TEST_MODULE VirtualRobot_VirtualRobotIOTest

#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <VirtualRobot/ManipulationObject.h>
#include <VirtualRobot/Nodes/PositionSensor.h>
#include <VirtualRobot/Nodes/Sensor.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RuntimeEnvironment.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/XML/ObjectIO.h>
#include <VirtualRobot/XML/RobotIO.h>

BOOST_AUTO_TEST_SUITE(VirtualRobotIO)

BOOST_AUTO_TEST_CASE(testLoadManipulationObjectGlobalPose)
{
    std::string filename = "objects/tests/pipe.xml";
    const bool fileOK = VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(filename);
    BOOST_REQUIRE(fileOK);

    VirtualRobot::ManipulationObjectPtr object;

    VirtualRobot::RobotPtr r;
    BOOST_REQUIRE_NO_THROW(object = VirtualRobot::ObjectIO::loadManipulationObject(filename););
    BOOST_REQUIRE(object);

    Eigen::Vector3f gtPosition{2697.64, 5225.85, 750.395};

    BOOST_REQUIRE_MESSAGE((gtPosition - object->getGlobalPosition()).norm() < 0.1,
                          "Position is " << object->getGlobalPosition() << " but should be "
                                         << gtPosition);
}

BOOST_AUTO_TEST_SUITE_END()
