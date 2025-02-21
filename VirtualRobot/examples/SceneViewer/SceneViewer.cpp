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

#include "showSceneWindow.h"

int
main(int argc, char* argv[])
{
    VirtualRobot::init(argc, argv, "Simox Scene Viewer");

    std::cout << " --- START --- " << std::endl;
    std::string filename("scenes/examples/SceneViewer/scene1.xml");
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(filename);
    VirtualRobot::RuntimeEnvironment::considerKey("scene");
    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
    VirtualRobot::RuntimeEnvironment::print();

    filename = VirtualRobot::RuntimeEnvironment::checkValidFileParameter("scene", filename);
    showSceneWindow rw(filename);

    rw.main();

    return 0;
}
