#include <iostream>
#include <string>

#include <VirtualRobot/RuntimeEnvironment.h>
#include <VirtualRobot/VirtualRobot.h>

using std::cout;
using std::endl;
using namespace VirtualRobot;

#include <eigen3/Eigen/Core>
#include <Eigen/Geometry>

#include "GraspQualityWindow.h"

int
main(int argc, char* argv[])
{
    VirtualRobot::init(argc, argv, "Grasp Quality Demo");
    std::cout << " --- START --- " << std::endl;

    std::string robot("robots/ArmarIII/ArmarIII.xml");
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robot);

    std::string object("objects/riceBox.xml");
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(object);

    VirtualRobot::RuntimeEnvironment::considerKey("robot");
    VirtualRobot::RuntimeEnvironment::considerKey("object");
    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
    VirtualRobot::RuntimeEnvironment::print();

    std::string robFile = VirtualRobot::RuntimeEnvironment::getValue("robot");

    if (!robFile.empty() && VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robFile))
    {
        robot = robFile;
    }

    std::string objFile = VirtualRobot::RuntimeEnvironment::getValue("object");

    if (!objFile.empty() && VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(objFile))
    {
        object = objFile;
    }


    std::cout << "Using robot from " << robot << std::endl;
    std::cout << "Using object from " << object << std::endl;

    GraspQualityWindow rw(robot, object);

    rw.main();

    return 0;
}
