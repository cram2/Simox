#include <VirtualRobot/RuntimeEnvironment.h>

#include "stabilityWindow.h"

using namespace VirtualRobot;

int
main(int argc, char* argv[])
{

    VirtualRobot::init(argc, argv, "Stability Demo");
    std::cout << " --- START --- " << std::endl;

    //std::string filenameRob("robots/ArmarIII/ArmarIII.xml");
    std::string filenameRob("robots/iCub/iCub.xml");
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(filenameRob);

    VirtualRobot::RuntimeEnvironment::considerKey("robot");
    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
    VirtualRobot::RuntimeEnvironment::print();

    std::cout << " --- START --- " << std::endl;

    if (VirtualRobot::RuntimeEnvironment::hasValue("robot"))
    {
        std::string robFile = VirtualRobot::RuntimeEnvironment::getValue("robot");

        if (VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robFile))
        {
            filenameRob = robFile;
        }
    }

    std::cout << "Using robot at " << filenameRob << std::endl;

    stabilityWindow rw(filenameRob);
    rw.main();

    return 0;
}
