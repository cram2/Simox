#include <filesystem>
#include <iostream>
#include <string>

#include <VirtualRobot/Import/URDF/SimoxURDFFactory.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/Nodes/RobotNodeRevoluteFactory.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RuntimeEnvironment.h>
#include <VirtualRobot/Transformation/DHParameter.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/XML/RobotIO.h>

using std::cout;
using std::endl;
using namespace VirtualRobot;

int
main(int /*argc*/, char* /*argv*/[])
{
    SimoxURDFFactory f;

    // atlas file
    std::string urdfFile = ("robots/urdf/atlas_description/urdf/atlas_v3.urdf");
    RuntimeEnvironment::getDataFileAbsolute(urdfFile);

    // to ensure that 3d model files can be loaded during converting we need to add the correct data path
    std::filesystem::path tmppath = urdfFile;
    tmppath = tmppath.parent_path();
    tmppath = tmppath / "/../..";
    std::string modelsBasePath = tmppath.generic_string();
    RuntimeEnvironment::addDataPath(modelsBasePath);

    // load model from file and convert it to the simox robot format
    RobotPtr r = f.loadFromFile(urdfFile);

    std::string outPath = std::filesystem::current_path().generic_string();
    std::cout << "Saving converted file to " << outPath << "/urdf_output.xml..." << std::endl;

    RobotIO::saveXML(r, "urdf_output.xml", outPath);
}
