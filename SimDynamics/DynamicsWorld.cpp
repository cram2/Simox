
#include "DynamicsWorld.h"

#include "DynamicsEngine/DynamicsEngineFactory.h"

namespace SimDynamics
{


    namespace
    {
        std::mutex mutex;
    }

    DynamicsWorldPtr DynamicsWorld::world;

    bool DynamicsWorld::convertMM2M = true;

    DynamicsWorld::Cleanup::~Cleanup()
    {
        std::scoped_lock lock(mutex);
        DynamicsWorld::world.reset();
    }

    DynamicsWorldPtr
    DynamicsWorld::GetWorld()
    {
        if (!world)
        {
            Init();
        }

        return world;
    }

    DynamicsEnginePtr
    DynamicsWorld::getEngine()
    {
        return engine;
    }

    DynamicsWorldPtr
    DynamicsWorld::Init(DynamicsEngineConfigPtr config)
    {
        static Cleanup _Cleanup;

        if (true)
        {
            std::scoped_lock lock(mutex);

            if (!world)
            {
                world.reset(new DynamicsWorld(config));
            }
            else
            {
                VR_WARNING << "Dynamics world is already initialized..." << std::endl;
            }
        }

        return world;
    }

    void
    DynamicsWorld::Close()
    {
        world.reset();
    }

    DynamicsWorld::DynamicsWorld(DynamicsEngineConfigPtr config)
    {
        DynamicsEngineFactoryPtr factory = DynamicsEngineFactory::first(nullptr);
        THROW_VR_EXCEPTION_IF(!factory,
                              "No Physics Engine Found. Re-Compile with engine support...");
        engine = factory->createEngine(config);
        THROW_VR_EXCEPTION_IF(!engine, "Could not create Physics Engine.");
    }

    DynamicsWorld::~DynamicsWorld() = default;

    bool
    DynamicsWorld::addObject(DynamicsObjectPtr o)
    {
        return engine->addObject(o);
    }

    bool
    DynamicsWorld::removeObject(DynamicsObjectPtr o)
    {
        return engine->removeObject(o);
    }

    DynamicsObjectPtr
    DynamicsWorld::CreateDynamicsObject(VirtualRobot::SceneObjectPtr o)
    {
        SIMDYNAMICS_ASSERT(o);

        DynamicsEngineFactoryPtr factory = DynamicsEngineFactory::first(nullptr);
        SIMDYNAMICS_ASSERT(factory);

        return factory->createObject(o);
    }

    void
    DynamicsWorld::createFloorPlane(const Eigen::Vector3f& pos /*= Eigen::Vector3f(0,0,0)*/,
                                    const Eigen::Vector3f& up /*= Eigen::Vector3f(0,0,1.0f)*/,
                                    float friction)
    {
        engine->createFloorPlane(pos, up, friction);
    }

    bool
    DynamicsWorld::addRobot(DynamicsRobotPtr r)
    {
        return engine->addRobot(r);
    }

    bool
    DynamicsWorld::removeRobot(DynamicsRobotPtr r)
    {
        return engine->removeRobot(r);
    }

    SimDynamics::DynamicsRobotPtr
    DynamicsWorld::CreateDynamicsRobot(VirtualRobot::RobotPtr rob)
    {
        SIMDYNAMICS_ASSERT(rob);

        DynamicsEngineFactoryPtr factory = DynamicsEngineFactory::first(nullptr);
        SIMDYNAMICS_ASSERT(factory);

        return factory->createRobot(rob);
    }

    std::vector<DynamicsRobotPtr>
    DynamicsWorld::getRobots()
    {
        return engine->getRobots();
    }

    std::vector<DynamicsObjectPtr>
    DynamicsWorld::getObjects()
    {
        return engine->getObjects();
    }

    void
    DynamicsWorld::removeFloorPlane()
    {
        engine->removeFloorPlane();
    }

    void
    DynamicsWorld::clear()
    {
        for (const auto& robot : getRobots())
        {
            removeRobot(robot);
        }
        for (const auto& obj : getObjects())
        {
            removeObject(obj);
        }
        removeFloorPlane();
    }
} // namespace SimDynamics
