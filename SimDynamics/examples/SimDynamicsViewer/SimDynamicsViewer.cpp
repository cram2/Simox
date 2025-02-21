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

#include "simDynamicsWindow.h"

int
main(int argc, char* argv[])
{
    VirtualRobot::init(argc, argv, "SimDynamicsViewer");

    VirtualRobot::RuntimeEnvironment::considerKey("robot");
    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
    VirtualRobot::RuntimeEnvironment::print();

    std::cout << " --- START --- " << std::endl;
    // --robot "robots/iCub/iCub.xml"
    //std::string filename("robots/iCub/iCub.xml");
    std::string filename("robots/ArmarIII/ArmarIII.xml");
    //std::string filename("robots/examples/SimpleRobot/Joint6.xml");
    //std::string filename("robots/ArmarIII/ArmarIII-RightArmTest6.xml");


    if (VirtualRobot::RuntimeEnvironment::hasValue("robot"))
    {
        std::string robFile = VirtualRobot::RuntimeEnvironment::getValue("robot");

        if (VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robFile))
        {
            filename = robFile;
        }
    }

    std::cout << "Using robot at " << filename << std::endl;

    SimDynamicsWindow rw(filename);

    rw.main();

    return 0;
}
