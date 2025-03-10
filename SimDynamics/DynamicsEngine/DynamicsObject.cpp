#include "DynamicsObject.h"

#include <VirtualRobot/SceneObject.h>

namespace SimDynamics
{

    DynamicsObject::DynamicsObject(VirtualRobot::SceneObjectPtr o)
    {
        THROW_VR_EXCEPTION_IF(!o, "NULL object");
        sceneObject = o;
        //engineMutexPtr.reset(new boost::recursive_mutex()); // may be overwritten by another mutex!
    }

    DynamicsObject::~DynamicsObject() = default;

    std::string
    DynamicsObject::getName() const
    {
        if (!sceneObject)
            return std::string();
        return sceneObject->getName();
    }

    VirtualRobot::SceneObject::Physics::SimulationType
    DynamicsObject::getSimType() const
    {
        return sceneObject->getSimulationType();
    }

    void
    DynamicsObject::setPose(const Eigen::Matrix4f& pose)
    {
        MutexLockPtr lock = getScopedLock();
        if (sceneObject->getSimulationType() == VirtualRobot::SceneObject::Physics::eDynamic)
        {
            // moving dynamic objects is not allowed
            return;
        }
        if (sceneObject->getSimulationType() == VirtualRobot::SceneObject::Physics::eStatic)
        {
            VR_ERROR << "Could not move static object, use kinematic instead, aborting..."
                     << std::endl;
            return;
        }
        try
        {
            sceneObject->setGlobalPose(pose);
        }
        catch (...) // robot node does not allow to set the pose
        {
        }
    }

    void
    DynamicsObject::setPosition(const Eigen::Vector3f& posMM)
    {
        MutexLockPtr lock = getScopedLock();
        Eigen::Matrix4f pose = sceneObject->getGlobalPose();
        pose.block(0, 3, 3, 1) = posMM;
        setPose(pose);
    }

    VirtualRobot::SceneObjectPtr
    DynamicsObject::getSceneObject()
    {
        return sceneObject;
    }

    Eigen::Vector3f
    DynamicsObject::getLinearVelocity()
    {
        return Eigen::Vector3f::Zero();
    }

    Eigen::Vector3f
    DynamicsObject::getAngularVelocity()
    {
        return Eigen::Vector3f::Zero();
    }

    void
    DynamicsObject::setLinearVelocity(const Eigen::Vector3f& /*vel*/)
    {
    }

    void
    DynamicsObject::setAngularVelocity(const Eigen::Vector3f& /*vel*/)
    {
    }

    void
    DynamicsObject::applyForce(const Eigen::Vector3f& /*force*/)
    {
    }

    void
    DynamicsObject::applyTorque(const Eigen::Vector3f& /*torque*/)
    {
    }

    void
    DynamicsObject::setMutex(std::shared_ptr<std::recursive_mutex> engineMutexPtr)
    {
        this->engineMutexPtr = engineMutexPtr;
    }

    void
    DynamicsObject::setSimType(VirtualRobot::SceneObject::Physics::SimulationType s)
    {
        sceneObject->setSimulationType(s);
    }

    void
    DynamicsObject::activate()
    {
    }

    DynamicsObject::MutexLockPtr
    DynamicsObject::getScopedLock()
    {
        std::shared_ptr<std::scoped_lock<std::recursive_mutex>> scoped_lock;

        if (engineMutexPtr)
        {
            scoped_lock.reset(new std::scoped_lock<std::recursive_mutex>(*engineMutexPtr));
        }

        return scoped_lock;
    }

} // namespace SimDynamics
