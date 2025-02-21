#include <iostream>
#include <string>

#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RuntimeEnvironment.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/XML/RobotIO.h>

#include <Inventor/Qt/SoQt.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/nodes/SoSeparator.h>

using std::cout;
using std::endl;
using namespace VirtualRobot;

#include <eigen3/Eigen/Core>
#include <Eigen/Geometry>

#include "showCamWindow.h"

bool useColModel = false;

int
main(int argc, char* argv[])
{
    VirtualRobot::init(argc, argv, "RobotViewer");


    VirtualRobot::RuntimeEnvironment::considerKey("robot");
    VirtualRobot::RuntimeEnvironment::considerKey("cam1");
    VirtualRobot::RuntimeEnvironment::considerKey("cam2");
    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
    VirtualRobot::RuntimeEnvironment::print();

    std::cout << " --- START --- " << std::endl;

    std::string filename("robots/ArmarIII/ArmarIII.xml");
    std::string cam1Name("EyeLeftCamera");
    std::string cam2Name("EyeRightCamera");

    if (VirtualRobot::RuntimeEnvironment::hasValue("robot"))
    {
        std::string robFile = VirtualRobot::RuntimeEnvironment::getValue("robot");

        if (VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robFile))
        {
            filename = robFile;
        }
    }

    if (VirtualRobot::RuntimeEnvironment::hasValue("cam1"))
    {
        cam1Name = VirtualRobot::RuntimeEnvironment::getValue("cam1");
    }

    if (VirtualRobot::RuntimeEnvironment::hasValue("cam2"))
    {
        cam2Name = VirtualRobot::RuntimeEnvironment::getValue("cam2");
    }


    std::cout << "Using robot:" << filename << ", cam1:" << cam1Name << ", cam2:" << cam2Name
              << std::endl;

    showCamWindow rw(filename, cam1Name, cam2Name);

    rw.main();

    return 0;
}
