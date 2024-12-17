#include <iostream>
#include <string>

#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/Nodes/RobotNodeRevoluteFactory.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RobotFactory.h>
#include <VirtualRobot/RuntimeEnvironment.h>
#include <VirtualRobot/Transformation/DHParameter.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/XML/RobotIO.h>

using std::cout;
using std::endl;
using namespace VirtualRobot;

int
main(int argc, char* argv[])
{
    std::shared_ptr<Robot> robot = RobotFactory::createRobot("Robbi");
    std::vector<std::shared_ptr<RobotNode>> robotNodes;
    VirtualRobot::RobotNodeRevoluteFactory revoluteNodeFactory;
    DHParameter dhParameter(0, 0, 0, 0, true);
    std::shared_ptr<RobotNode> node1 = revoluteNodeFactory.createRobotNodeDH(robot,
                                                                             "RootNode",
                                                                             VisualizationNodePtr(),
                                                                             CollisionModelPtr(),
                                                                             (float)-M_PI,
                                                                             (float)M_PI,
                                                                             0.0f,
                                                                             dhParameter);
    robotNodes.push_back(node1);
    std::map<RobotNodePtr, std::vector<std::string>> childrenMap;
    bool resInit = RobotFactory::initializeRobot(robot, robotNodes, childrenMap, node1);

    std::cout << "resInit:" << resInit << std::endl;
    std::cout << "First robot:" << std::endl;
    robot->print();

    VirtualRobot::RuntimeEnvironment::considerKey("robot");
    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
    VirtualRobot::RuntimeEnvironment::print();

    std::string filename("robots/examples/loadRobot/RobotExample.xml");
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(filename);

    if (VirtualRobot::RuntimeEnvironment::hasValue("robot"))
    {
        std::string robFile = VirtualRobot::RuntimeEnvironment::getValue("robot");

        if (VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robFile))
        {
            filename = robFile;
        }
    }

    std::cout << "Using robot at " << filename << std::endl;
    RobotPtr rob;

    try
    {
        rob = RobotIO::loadRobot(filename, RobotIO::eStructure);
    }
    catch (VirtualRobotException& e)
    {
        std::cout << "Error: " << e.what() << std::endl;
        return -1;
    }

    std::cout << "Second robot (XML):" << std::endl;

    if (rob)
    {
        rob->print();
    }
    else
    {
        std::cout << " ERROR while creating robot" << std::endl;
    }
}
