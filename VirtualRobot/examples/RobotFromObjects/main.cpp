#include <filesystem>
#include <fstream>
#include <random>

#include <VirtualRobot/CollisionDetection/CollisionModel.h>
#include <VirtualRobot/Nodes/RobotNodeFixedFactory.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RobotFactory.h>
#include <VirtualRobot/RuntimeEnvironment.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationNode.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <VirtualRobot/XML/RobotIO.h>

int
main(int argc, char* argv[])
{
    std::mt19937 gen{std::random_device{}()};

    VirtualRobot::init(argc, argv, "RGBOffscreenRendering");

    const std::string robotType = "randomboxbot";
    const std::string robotName = "randomboxbot-" + std::to_string(gen());
    std::cout << "generating " << robotName << "\n";
    std::filesystem::create_directories(robotName);
    std::filesystem::current_path(robotName);

    VirtualRobot::RobotPtr robot(new VirtualRobot::LocalRobot(robotName, robotType));

    std::vector<VirtualRobot::RobotNodePtr> allNodes;
    std::map<VirtualRobot::RobotNodePtr, std::vector<std::string>> childrenMap;
    const std::size_t numNodes = std::uniform_int_distribution<std::size_t>(2, 10)(gen);
    std::cout << "generating " << numNodes << " nodes\n";

    std::uniform_real_distribution<float> d(50, 500);

    const auto createBodyNode = [&](VirtualRobot::RobotNodePtr last, const std::string& name)
    {
        std::cout << "generating node " << name << "\n";
        Eigen::Matrix4f preJointTransform = Eigen::Matrix4f::Identity();
        if (last)
        {
            preJointTransform.topRightCorner<3, 1>() =
                last->getVisualization()->getTriMeshModel()->nonUniformSampleSurface(gen) * 0.5;
        }
        auto model = VirtualRobot::TriMeshModel::MakeBox(d(gen), d(gen), d(gen));
        model.setColor({0, 255, 0, 0});
        const auto cvisu = std::make_shared<VirtualRobot::CoinVisualizationNode>(model);
        auto rnCol = std::make_shared<VirtualRobot::CollisionModel>(cvisu);

        auto rnVisu = rnCol->getVisualization()->clone();

        auto fixedNodeFactory = VirtualRobot::RobotNodeFactory::fromName(
            VirtualRobot::RobotNodeFixedFactory::getName(), NULL);

        VirtualRobot::SceneObject::Physics physics;
        physics.massKg = 1;
        physics.inertiaMatrix.setIdentity();
        physics.comLocation = VirtualRobot::SceneObject::Physics::eVisuBBoxCenter;

        VirtualRobot::RobotNodePtr node =
            fixedNodeFactory->createRobotNode(robot,
                                              name,
                                              rnVisu,
                                              rnCol,
                                              0,
                                              0,
                                              0, //limits
                                              preJointTransform,
                                              Eigen::Vector3f::Zero(), //axis
                                              Eigen::Vector3f::Zero(), //translation direction
                                              physics);
        robot->registerRobotNode(node);
        allNodes.push_back(node);
        if (last)
        {
            childrenMap[last].emplace_back(name);
        }
        return node;
    };

    VirtualRobot::RobotNodePtr lastNode = createBodyNode(nullptr, "root");

    for (std::size_t i = 1; i < numNodes; ++i)
    {
        lastNode = createBodyNode(lastNode, std::to_string(i));
    }
    std::cout << "assemble robot\n";
    VirtualRobot::RobotFactory::initializeRobot(robot, allNodes, childrenMap, allNodes.front());

    std::cout << "save robot\n";
    VirtualRobot::RobotIO::saveXML(robot, robotName + ".xml", "", "models", true, true, true, true);
    return 0;
}
