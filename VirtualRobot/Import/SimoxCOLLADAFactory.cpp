

#include "SimoxCOLLADAFactory.h"

#include <iostream>

#include "COLLADA-light/ColladaSimox.h"
#include "Logging.h"
#include "VirtualRobotException.h"

namespace VirtualRobot
{

    SimoxCOLLADAFactory::SimoxCOLLADAFactory() = default;


    SimoxCOLLADAFactory::~SimoxCOLLADAFactory() = default;

    RobotPtr
    SimoxCOLLADAFactory::loadFromFile(const std::string& filename,
                                      RobotIO::RobotDescription /*loadMode*/)
    {
        RobotPtr robot;

        try
        {
            Collada::ColladaSimoxRobot colladaRobot(1000.0f);
            colladaRobot.parse(filename);
            colladaRobot.initialize();
            robot = colladaRobot.getSimoxRobot();
        }
        catch (VirtualRobotException& e)
        {
            std::cout << " ERROR while creating robot (exception)" << std::endl;
            std::cout << e.what();
            return robot;
        }

        if (!robot)
        {
            VR_ERROR << " ERROR while loading robot from file:" << filename << std::endl;
        }

        return robot;
    }

    std::string
    SimoxCOLLADAFactory::getFileExtension()
    {
        return std::string("dae");
    }

    std::string
    SimoxCOLLADAFactory::getFileFilter()
    {
        return std::string("COLLADA (*.dae)");
    }

    /**
     * register this class in the super class factory
     */
    RobotImporterFactory::SubClassRegistry
        SimoxCOLLADAFactory::registry(SimoxCOLLADAFactory::getName(),
                                      &SimoxCOLLADAFactory::createInstance);

    /**
     * \return "SimoxCOLLADA"
     */
    std::string
    SimoxCOLLADAFactory::getName()
    {
        return "SimoxCOLLADA";
    }

    /**
     * \return new instance of SimoxCOLLADAFactory.
     */
    std::shared_ptr<RobotImporterFactory>
    SimoxCOLLADAFactory::createInstance(void*)
    {
        std::shared_ptr<SimoxCOLLADAFactory> COLLADAFactory(new SimoxCOLLADAFactory());
        return COLLADAFactory;
    }

} // namespace VirtualRobot
