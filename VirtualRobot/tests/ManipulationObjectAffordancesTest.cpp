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

BOOST_AUTO_TEST_CASE(testLoadManipulationObjectWithAffordances)
{
    std::string filename = "objects/tests/CondensedMilk.xml";
    const bool fileOK = VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(filename);
    BOOST_REQUIRE(fileOK);

    VirtualRobot::ManipulationObjectPtr object;

    object = VirtualRobot::ObjectIO::loadManipulationObject(filename);

    BOOST_REQUIRE_NO_THROW(object = VirtualRobot::ObjectIO::loadManipulationObject(filename););
    BOOST_REQUIRE(object);

    BOOST_REQUIRE(object->getAffordances().size() == 1);
    BOOST_REQUIRE(object->getAffordancesOfType("pourable").size() == 1);
    BOOST_REQUIRE(object->getAffordancesOfType("nonexisting").empty());

    // for (const auto& location : object->getAffordances())
    // {
    //     VR_INFO << location.pose.pose.matrix() << std::endl;
    //     VR_INFO << location.pose.frame << std::endl;
    //     for (const auto& a : location.affordances)
    //     {
    //         VR_INFO << a.type << std::endl;
    //         VR_INFO << a.primitiveRepresentation.size() << std::endl;
    //     }
    // }
}

BOOST_AUTO_TEST_SUITE_END()
