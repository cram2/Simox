#include "DynamicsEngine.h"

#include <algorithm>

namespace SimDynamics
{

    DynamicsEngine::DynamicsEngine(std::shared_ptr<std::recursive_mutex> engineMutex)
    {
        floorPos.setZero();
        floorUp.setZero();
        floorDepthMM = 500.0f;
        floorExtendMM = 50000.0f;
        //if (engineMutex)
        engineMutexPtr = engineMutex;
        //else
        //    engineMutexPtr.reset(new boost::recursive_mutex());
    }

    DynamicsEngine::~DynamicsEngine()
    {
        robots.clear();
        objects.clear();
    }

    Eigen::Vector3f
    DynamicsEngine::getGravity()
    {
        MutexLockPtr lock = getScopedLock();
        return dynamicsConfig->gravity;
    }

    bool
    DynamicsEngine::init(DynamicsEngineConfigPtr config)
    {
        if (config)
        {
            dynamicsConfig = config;
        }
        else
        {
            dynamicsConfig.reset(new DynamicsEngineConfig());
        }

        return true;
    }

    void
    DynamicsEngine::setMutex(std::shared_ptr<std::recursive_mutex> engineMutex)
    {
        engineMutexPtr = engineMutex;
    }

    bool
    DynamicsEngine::addObject(DynamicsObjectPtr o)
    {
        MutexLockPtr lock = getScopedLock();

        if (find(objects.begin(), objects.end(), o) == objects.end())
        {
            objects.push_back(o);
        }

        o->setMutex(engineMutexPtr);
        return true;
    }

    bool
    DynamicsEngine::removeObject(DynamicsObjectPtr o)
    {
        MutexLockPtr lock = getScopedLock();
        std::vector<DynamicsObjectPtr>::iterator it = find(objects.begin(), objects.end(), o);

        if (it == objects.end())
        {
            return false;
        }

        objects.erase(it);
        return true;
    }

    void
    DynamicsEngine::createFloorPlane(const Eigen::Vector3f& pos,
                                     const Eigen::Vector3f& up,
                                     float /*friction*/)
    {
        MutexLockPtr lock = getScopedLock();
        floorPos = pos;
        floorUp = up;
        floorDepthMM = 500.0f;
        floorExtendMM = 50000.0f;
    }

    void
    DynamicsEngine::removeFloorPlane()
    {
        MutexLockPtr lock = getScopedLock();
        floorPos.setZero();
        floorUp.setZero();
        floorDepthMM = 0.0f;
        floorExtendMM = 0.0f;
        removeObject(floor);
        floor.reset();
    }

    bool
    DynamicsEngine::addRobot(DynamicsRobotPtr r)
    {
        MutexLockPtr lock = getScopedLock();

        if (find(robots.begin(), robots.end(), r) == robots.end())
        {
            for (auto& robot : robots)
            {
                if (robot->getRobot() == r->getRobot())
                {
                    VR_ERROR << "Only one DynamicsWrapper per robot allowed. Robot " << r->getName()
                             << std::endl;
                    return false;
                }
            }

            robots.push_back(r);
        }

        r->setMutex(engineMutexPtr);
        return true;
    }

    bool
    DynamicsEngine::removeRobot(DynamicsRobotPtr r)
    {
        MutexLockPtr lock = getScopedLock();
        std::vector<DynamicsRobotPtr>::iterator it = find(robots.begin(), robots.end(), r);

        if (it == robots.end())
        {
            return false;
        }

        robots.erase(it);
        return true;
    }

    bool
    DynamicsEngine::attachObjectToRobot(const std::string& robotName,
                                        const std::string& nodeName,
                                        DynamicsObjectPtr object)
    {
        DynamicsRobotPtr r = getRobot(robotName);

        if (!r)
        {
            VR_ERROR << "No robot with name " << robotName << std::endl;
            return false;
        }

        return attachObjectToRobot(r, nodeName, object);
    }

    bool
    DynamicsEngine::attachObjectToRobot(DynamicsRobotPtr r,
                                        const std::string& nodeName,
                                        DynamicsObjectPtr object)
    {
        MutexLockPtr lock = getScopedLock();

        if (!r)
        {
            return false;
        }

        if (!r->attachObject(nodeName, object))
        {
            return false;
        }

        return true;
    }

    bool
    DynamicsEngine::detachObjectFromRobot(const std::string& robotName, DynamicsObjectPtr object)
    {
        DynamicsRobotPtr r = getRobot(robotName);

        if (!r)
        {
            VR_ERROR << "No robot with name " << robotName << std::endl;
            return false;
        }

        return detachObjectFromRobot(r, object);
    }

    bool
    DynamicsEngine::detachObjectFromRobot(DynamicsRobotPtr r, DynamicsObjectPtr object)
    {
        MutexLockPtr lock = getScopedLock();

        if (!r)
        {
            return false;
        }

        if (!r->detachObject(object))
        {
            return false;
        }

        return true;
    }

    void
    DynamicsEngine::disableCollision(DynamicsObject* o1, DynamicsObject* o2)
    {
        MutexLockPtr lock = getScopedLock();
        std::vector<DynamicsObject*>::iterator i1 =
            find(collisionDisabled[o1].begin(), collisionDisabled[o1].end(), o2);

        if (i1 == collisionDisabled[o1].end())
        {
            collisionDisabled[o1].push_back(o2);
        }

        std::vector<DynamicsObject*>::iterator i2 =
            find(collisionDisabled[o2].begin(), collisionDisabled[o2].end(), o1);

        if (i2 == collisionDisabled[o2].end())
        {
            collisionDisabled[o2].push_back(o1);
        }
    }

    void
    DynamicsEngine::disableCollision(DynamicsObject* o1)
    {
        MutexLockPtr lock = getScopedLock();
        std::vector<DynamicsObject*>::iterator i1 =
            find(collisionToAllDisabled.begin(), collisionToAllDisabled.end(), o1);

        if (i1 == collisionToAllDisabled.end())
        {
            collisionToAllDisabled.push_back(o1);
        }
    }

    void
    DynamicsEngine::enableCollision(DynamicsObject* o1, DynamicsObject* o2)
    {
        MutexLockPtr lock = getScopedLock();
        //if (o1 < o2)
        //{
        std::vector<DynamicsObject*>::iterator i1 =
            find(collisionDisabled[o1].begin(), collisionDisabled[o1].end(), o2);

        if (i1 != collisionDisabled[o1].end())
        {
            collisionDisabled[o1].erase(i1);
        }

        //} else
        //{
        std::vector<DynamicsObject*>::iterator i2 =
            find(collisionDisabled[o2].begin(), collisionDisabled[o2].end(), o1);

        if (i2 != collisionDisabled[o2].end())
        {
            collisionDisabled[o2].erase(i2);
        }

        //}
    }

    void
    DynamicsEngine::enableCollision(DynamicsObject* o1)
    {
        MutexLockPtr lock = getScopedLock();
        std::vector<DynamicsObject*>::iterator i1 =
            find(collisionToAllDisabled.begin(), collisionToAllDisabled.end(), o1);

        if (i1 != collisionToAllDisabled.end())
        {
            collisionToAllDisabled.erase(i1);
        }
    }

    bool
    DynamicsEngine::checkCollisionEnabled(DynamicsObject* o1)
    {
        MutexLockPtr lock = getScopedLock();
        std::vector<DynamicsObject*>::iterator i1 =
            find(collisionToAllDisabled.begin(), collisionToAllDisabled.end(), o1);
        return (i1 == collisionToAllDisabled.end());
    }

    void
    DynamicsEngine::getFloorInfo(Eigen::Vector3f& floorPos,
                                 Eigen::Vector3f& floorUp,
                                 double& floorExtendMM,
                                 double& floorDepthMM)
    {
        floorPos = this->floorPos;
        floorUp = this->floorUp;
        floorExtendMM = this->floorExtendMM;
        floorDepthMM = this->floorDepthMM;
    }

    bool
    DynamicsEngine::checkCollisionEnabled(DynamicsObject* o1, DynamicsObject* o2)
    {
        MutexLockPtr lock = getScopedLock();
        if (o1 == nullptr || o2 == nullptr)
        {
            return true;
        }
        if (!checkCollisionEnabled(o1))
        {
            return false;
        }

        if (!checkCollisionEnabled(o2))
        {
            return false;
        }

        //if (o1 < o2)
        //{
        std::vector<DynamicsObject*>::iterator i1 =
            find(collisionDisabled[o1].begin(), collisionDisabled[o1].end(), o2);

        if (i1 != collisionDisabled[o1].end())
        {
            return false;
        }

        //} else
        //{
        std::vector<DynamicsObject*>::iterator i2 =
            find(collisionDisabled[o2].begin(), collisionDisabled[o2].end(), o1);

        if (i2 != collisionDisabled[o2].end())
        {
            return false;
        }

        //}
        return true;
    }

    void
    DynamicsEngine::resetCollisions(DynamicsObject* o)
    {
        MutexLockPtr lock = getScopedLock();
        collisionDisabled.erase(o);
        std::map<DynamicsObject*, std::vector<DynamicsObject*>>::iterator i1 =
            collisionDisabled.begin();

        while (i1 != collisionDisabled.end())
        {
            std::vector<DynamicsObject*>::iterator i2 =
                find(i1->second.begin(), i1->second.end(), o);

            if (i2 != i1->second.end())
            {
                i1->second.erase(i2);
            }

            i1++;
        }

        enableCollision(o);
    }

    std::vector<DynamicsRobotPtr>
    DynamicsEngine::getRobots()
    {
        MutexLockPtr lock = getScopedLock();
        return robots;
    }

    std::vector<DynamicsObjectPtr>
    DynamicsEngine::getObjects()
    {
        MutexLockPtr lock = getScopedLock();
        return objects;
    }

    DynamicsObjectPtr
    DynamicsEngine::getObject(const std::string& objectName)
    {
        MutexLockPtr lock = getScopedLock();

        for (auto& object : objects)
        {
            if (object->getName() == objectName)
            {
                return object;
            }
        }

        return DynamicsObjectPtr();
    }

    void
    DynamicsEngine::activateAllObjects()
    {
        MutexLockPtr lock = getScopedLock();

        for (auto& object : objects)
        {
            object->activate();
        }
    }

    std::vector<DynamicsEngine::DynamicsContactInfo>
    DynamicsEngine::getContacts()
    {
        MutexLockPtr lock = getScopedLock();
        std::vector<DynamicsEngine::DynamicsContactInfo> result;
        return result;
    }

    SimDynamics::DynamicsRobotPtr
    DynamicsEngine::getRobot(VirtualRobot::RobotPtr r)
    {
        MutexLockPtr lock = getScopedLock();

        for (auto& robot : robots)
        {
            if (robot->getRobot() == r)
            {
                return robot;
            }
        }

        return DynamicsRobotPtr();
    }

    DynamicsRobotPtr
    DynamicsEngine::getRobot(const std::string& robName)
    {
        MutexLockPtr lock = getScopedLock();

        for (auto& robot : robots)
        {
            if (robot->getName() == robName)
            {
                return robot;
            }
        }

        return DynamicsRobotPtr();
    }

    DynamicsEngine::MutexLockPtr
    DynamicsEngine::getScopedLock()
    {
        std::shared_ptr<std::scoped_lock<std::recursive_mutex>> scoped_lock;

        if (engineMutexPtr)
        {
            scoped_lock.reset(new std::scoped_lock<std::recursive_mutex>(*engineMutexPtr));
        }

        return scoped_lock;
    }

} // namespace SimDynamics
