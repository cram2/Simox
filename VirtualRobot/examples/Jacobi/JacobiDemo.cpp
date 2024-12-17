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

#include "JacobiWindow.h"

bool useColModel = false;

int
main(int argc, char* argv[])
{
    VirtualRobot::init(argc, argv, "Jacobi Demo");

    std::cout << " --- START --- " << std::endl;
    std::string filename("robots/ArmarIII/ArmarIII.xml");
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(filename);
    JacobiWindow rw(filename);

    rw.main();

    return 0;
}
