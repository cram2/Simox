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

#include <VirtualRobot/Grasping/GraspEditor/GraspEditorWindow.h>

int
main(int argc, char* argv[])
{
    VirtualRobot::init(argc, argv, "GraspEditor");
    std::cout << " --- START --- " << std::endl;

    std::string filename1("objects/plate.xml");
    std::string filename2("robots/ArmarIII/ArmarIII.xml");

    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(filename1);
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(filename2);

    VirtualRobot::RuntimeEnvironment::considerKey("object");
    VirtualRobot::RuntimeEnvironment::considerKey("robot");

    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
    VirtualRobot::RuntimeEnvironment::print();

    std::string objFile = VirtualRobot::RuntimeEnvironment::getValue("object");

    if (!objFile.empty() && VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(objFile))
    {
        filename1 = objFile;
    }

    if (VirtualRobot::RuntimeEnvironment::hasValue("robot"))
    {
        filename2 = VirtualRobot::RuntimeEnvironment::getValue("robot");
    }

    GraspEditorWindow rw(filename1, filename2);

    rw.main();

    return 0;
}
