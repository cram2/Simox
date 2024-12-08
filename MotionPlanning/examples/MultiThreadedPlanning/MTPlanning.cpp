
#include "MTPlanningWindow.h"
#include "VirtualRobot/CollisionDetection/CollisionChecker.h"


#include <cstring>
#include <iostream>
using namespace std;
using namespace VirtualRobot;


void startMTPlanning()
{
    MTPlanningWindow* agfw = new MTPlanningWindow();
    agfw->main();
    delete agfw;
}

int main(int argc, char** argv)
{
    SoDB::init();
    SoQt::init(argc, argv, "MT");
    std::cout << " --- START --- " << std::endl;

    if (!CollisionChecker::IsSupported_Multithreading_MultipleColCheckers())
    {
        std::cout << " The collision detection library that is linked to simox does not support multi threading. Aborting...." << std::endl;
        return -1;
    }

#ifdef WIN32
    std::cout << "Visual Studio users: Be sure to start this example with <ctrl>+F5 (RELEASE mode) for full performance (otherwise only 1 thread will be used)" << std::endl;
#endif

    try
    {
        startMTPlanning();
    }
    catch(const std::exception &e)
    {
        std::cout << "Exception: " << e.what() << std::endl ;
    }
    catch (...)
    {
        ;
    }

    std::cout << " --- END --- " << std::endl;

    return 0;
}
