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

#include "IKRRTWindow.h"

//#define ARMAR


int
main(int argc, char* argv[])
{
    VirtualRobot::init(argc, argv, "IK-RRT Demo");
    std::cout << " --- START --- " << std::endl;

#ifdef ARMAR
    std::string filenameScene("scenes/examples/IKRRT/planningHotSpot.xml");
    std::string filenameReach("reachability/ArmarIII_HipLeftArm.bin");
    std::string kinChain("TorsoLeftArm");
    std::string eef("Hand L");
    std::string colModel("LeftArmHandColModel");
    std::string colModelRob("PlatformTorsoHeadColModel");
#else
    // ICUB
    std::string filenameScene("scenes/IKRRT_scene_iCub.xml");
    std::string filenameReach("reachability/iCub_HipLeftArm.bin");
    std::string kinChain("Hip Left Arm");
    std::string eef("Left Hand");
    std::string colModel("Left HandArm ColModel");
    std::string colModelRob("BodyHeadLegsColModel");
#endif
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(filenameScene);
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(filenameReach);

    VirtualRobot::RuntimeEnvironment::considerKey("scene");
    VirtualRobot::RuntimeEnvironment::considerKey("reachability");
    VirtualRobot::RuntimeEnvironment::considerKey("kinematicChain");
    VirtualRobot::RuntimeEnvironment::considerKey("endEffector");
    VirtualRobot::RuntimeEnvironment::considerKey("collisionModelKinChain");
    VirtualRobot::RuntimeEnvironment::considerKey("collisionModelRobot");
    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
    VirtualRobot::RuntimeEnvironment::print();

    std::string scFile = VirtualRobot::RuntimeEnvironment::getValue("scene");

    if (!scFile.empty() && VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(scFile))
    {
        filenameScene = scFile;
    }

    std::string reachFile = VirtualRobot::RuntimeEnvironment::getValue("reachability");

    if (!reachFile.empty() && VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(reachFile))
    {
        filenameReach = reachFile;
    }

    std::string kc = VirtualRobot::RuntimeEnvironment::getValue("kinematicChain");

    if (!kc.empty())
    {
        kinChain = kc;
    }

    std::string EndEff = VirtualRobot::RuntimeEnvironment::getValue("endEffector");

    if (!EndEff.empty())
    {
        eef = EndEff;
    }

    std::string col1 = VirtualRobot::RuntimeEnvironment::getValue("collisionModelKinChain");

    if (!col1.empty())
    {
        colModel = col1;
    }

    std::string col2 = VirtualRobot::RuntimeEnvironment::getValue("collisionModelRobot");

    if (!col2.empty())
    {
        colModelRob = col2;
    }


    std::cout << "Using scene at " << filenameScene << std::endl;
    std::cout << "Using reachability at " << filenameReach << std::endl;
    std::cout << "Using end effector " << eef << std::endl;
    std::cout << "Using col model (kin chain) " << colModel << std::endl;
    std::cout << "Using col model (static robot)" << colModelRob << std::endl;

    IKRRTWindow rw(filenameScene, filenameReach, kinChain, eef, colModel, colModelRob);

    rw.main();

    return 0;
}
