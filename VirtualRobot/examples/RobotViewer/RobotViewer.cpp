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

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "showRobotWindow.h"

bool useColModel = false;

int
main(int argc, char* argv[])
{
    VirtualRobot::init(argc, argv, "Simox Robot Viewer");

    VirtualRobot::RuntimeEnvironment::considerKey("robot");
    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
    VirtualRobot::RuntimeEnvironment::print();

    std::cout << " --- START --- " << std::endl;
    // --robot "robots/iCub/iCub.xml"
    std::string filename("robots/ArmarIII/ArmarIII.xml");
    //std::string filename("C:/Projects/MMM/mmmtools/data/Model/Winter/mmm.xml");
    //std::string filename("robots/ArmarIII/ArmarIII-RightArm.xml");
    //std::string filename("C:/Projects/IIT_Projects/iCubRobot/robot/iCub.xml");
    //std::string filename(DEMO_BASE_DIR "/robot/iCub_RightArm.xml");
    //std::string filename(DEMO_BASE_DIR "/robot/iCub_RightHand.xml");
    //std::string filename(DEMO_BASE_DIR "/robot/iCub_LeftHand.xml");

    if (VirtualRobot::RuntimeEnvironment::hasValue("robot"))
    {
        std::string robFile = VirtualRobot::RuntimeEnvironment::getValue("robot");

        if (VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robFile))
        {
            filename = robFile;
        }
    }

    std::cout << "Using robot at " << filename << std::endl;

    showRobotWindow rw(filename);

    rw.main();

    return 0;
}
