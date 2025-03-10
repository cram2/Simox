#include "DynamicsRobot.h"

#include <VirtualRobot/SceneObject.h>

#include "../DynamicsWorld.h"
#include "VirtualRobot/Assert.h"
#include "VirtualRobot/Nodes/RobotNode.h"

namespace SimDynamics
{

    DynamicsRobot::DynamicsRobot(VirtualRobot::RobotPtr rob)
    {
        THROW_VR_EXCEPTION_IF(!rob, "NULL object");
        robot = rob;
        sensors = rob->getSensors();
        //engineMutexPtr.reset(new boost::recursive_mutex()); // may be overwritten by another mutex!
    }

    DynamicsRobot::~DynamicsRobot() = default;

    std::string
    DynamicsRobot::getName() const
    {
        return robot->getName();
    }

    void
    DynamicsRobot::ensureKinematicConstraints()
    {
    }

    void
    DynamicsRobot::createDynamicsNode(VirtualRobot::RobotNodePtr node)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(node);

        // check if already created
        if (hasDynamicsRobotNode(node))
        {
            return;
        }

        DynamicsWorldPtr dw = DynamicsWorld::GetWorld();
        DynamicsObjectPtr drn = dw->CreateDynamicsObject(node);
        VR_ASSERT(drn);
        dynamicRobotNodes[node] = drn;
    }

    std::vector<DynamicsObjectPtr>
    DynamicsRobot::getDynamicsRobotNodes()
    {
        MutexLockPtr lock = getScopedLock();
        std::map<VirtualRobot::RobotNodePtr, DynamicsObjectPtr>::iterator it =
            dynamicRobotNodes.begin();
        std::vector<DynamicsObjectPtr> res;

        while (it != dynamicRobotNodes.end())
        {
            res.push_back(it->second);
            it++;
        }

        return res;
    }

    bool
    DynamicsRobot::hasDynamicsRobotNode(VirtualRobot::RobotNodePtr node)
    {
        MutexLockPtr lock = getScopedLock();

        if (dynamicRobotNodes.find(node) == dynamicRobotNodes.end())
        {
            return false;
        }

        return true;
    }

    DynamicsObjectPtr
    DynamicsRobot::getDynamicsRobotNode(const std::string& nodeName)
    {
        MutexLockPtr lock = getScopedLock();
        std::map<VirtualRobot::RobotNodePtr, DynamicsObjectPtr>::iterator it =
            dynamicRobotNodes.begin();
        std::vector<DynamicsObjectPtr> res;

        while (it != dynamicRobotNodes.end())
        {
            if (it->first->getName() == nodeName)
                return it->second;
            it++;
        }
        return DynamicsObjectPtr();
    }

    DynamicsObjectPtr
    DynamicsRobot::getDynamicsRobotNode(VirtualRobot::RobotNodePtr node)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(node);

        if (dynamicRobotNodes.find(node) == dynamicRobotNodes.end())
        {
            return DynamicsObjectPtr();
        }

        return dynamicRobotNodes[node];
    }

    void
    DynamicsRobot::actuateNode(const std::string& node, double jointValue)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(robot);
        VR_ASSERT(robot->hasRobotNode(node));
        actuateNode(robot->getRobotNode(node), jointValue);
    }

    void
    DynamicsRobot::addJointFriction(VirtualRobot::RobotNodePtr node)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(robot);
        VR_ASSERT(node);
        VR_ASSERT(robot->hasRobotNode(node));

        //if (!hasDynamicsRobotNode(node))
        //    createDynamicsNode(node);

        //DynamicsObjectPtr dnyRN = getDynamicsRobotNode(node);

        robotNodeActuationTarget target;
        target.actuation.modes.torque = 1;
        target.node = node;
        target.jointTorqueTarget = 1;
        //target.dynNode = dnyRN;

        actuationTargets[node] = target;

        if (actuationControllers.find(node) == actuationControllers.end())
        {
            actuationControllers[node] =
                VelocityMotorController(node->getMaxVelocity(), node->getMaxAcceleration());
            actuationControllers[node].reset(PID_p, PID_i, PID_d);
        }
        else
        {
            actuationControllers[node].reset();
        }
    }

    void
    DynamicsRobot::actuateNode(VirtualRobot::RobotNodePtr node, double jointValue)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(robot);
        VR_ASSERT(node);
        VR_ASSERT(robot->hasRobotNode(node));

        robotNodeActuationTarget target;
        target.actuation.modes.position = 1;
        target.node = node;
        target.jointValueTarget = jointValue;

        actuationTargets[node] = target;

        if (actuationControllers.find(node) == actuationControllers.end())
        {
            actuationControllers[node] =
                VelocityMotorController(node->getMaxVelocity(), node->getMaxAcceleration());
            actuationControllers[node].reset(PID_p, PID_i, PID_d);
        }
        else
        {
            actuationControllers[node].reset();
        }
    }

    void
    DynamicsRobot::actuateNode(VirtualRobot::RobotNodePtr node,
                               double jointValue,
                               double jointVelocity)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(robot);
        VR_ASSERT(node);
        VR_ASSERT(robot->hasRobotNode(node));

        robotNodeActuationTarget target;
        target.actuation.modes.position = 1;
        target.actuation.modes.velocity = 1;
        target.node = node;
        target.jointValueTarget = jointValue;
        target.jointVelocityTarget = jointVelocity;

        actuationTargets[node] = target;

        if (actuationControllers.find(node) == actuationControllers.end())
        {
            actuationControllers[node] =
                VelocityMotorController(node->getMaxVelocity(), node->getMaxAcceleration());
            actuationControllers[node].reset(PID_p, PID_i, PID_d);
        }
        else
        {
            actuationControllers[node].reset();
        }
    }

    void
    DynamicsRobot::actuateNodeVel(const std::string& node, double jointVelocity)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(robot);
        VR_ASSERT(robot->hasRobotNode(node));

        actuateNodeVel(robot->getRobotNode(node), jointVelocity);
    }

    void
    DynamicsRobot::actuateNodeVel(VirtualRobot::RobotNodePtr node, double jointVelocity)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(robot);
        VR_ASSERT(node);
        VR_ASSERT(robot->hasRobotNode(node));

        //if (!hasDynamicsRobotNode(node))
        //    createDynamicsNode(node);

        //DynamicsObjectPtr dnyRN = getDynamicsRobotNode(node);
        auto oldTargetIt = actuationTargets.find(node);
        robotNodeActuationTarget target;
        target.actuation.modes.velocity = 1;
        target.actuation.modes.position = 0;
        target.node = node;
        bool modeChanged = oldTargetIt == actuationTargets.end() ||
                           oldTargetIt->second.actuation.modes.velocity != 1 ||
                           oldTargetIt->second.actuation.modes.position != 0;
        //        if(!modeChanged)
        //        {
        //            target.jointValueTarget = oldTargetIt->second.jointValueTarget;
        //        }
        //        else
        if (std::abs(jointVelocity) < 1e-10)
        {
            //do not move
            target.jointValueTarget = oldTargetIt->second.jointValueTarget;
        }
        else
        {
            target.jointValueTarget = node->getJointValue();
        }

        target.jointVelocityTarget = jointVelocity;

        actuationTargets[node] = target;

        if (actuationControllers.find(node) == actuationControllers.end())
        {
            actuationControllers[node] =
                VelocityMotorController(node->getMaxVelocity(), node->getMaxAcceleration());
            actuationControllers[node].reset(PID_p, PID_i, PID_d);
        }
        else
        {
            if (modeChanged)
            {
                actuationControllers[node].reset();
            }
        }
    }

    void
    DynamicsRobot::actuateNodeTorque(const std::string& node, double jointTorque)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(robot);
        VR_ASSERT(robot->hasRobotNode(node));
        actuateNodeTorque(robot->getRobotNode(node), jointTorque);
    }

    void
    DynamicsRobot::actuateNodeTorque(VirtualRobot::RobotNodePtr node, double jointTorque)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(robot);
        VR_ASSERT(node);
        VR_ASSERT(robot->hasRobotNode(node));

        //if (!hasDynamicsRobotNode(node))
        //    createDynamicsNode(node);

        //DynamicsObjectPtr dnyRN = getDynamicsRobotNode(node);

        robotNodeActuationTarget target;
        target.actuation.modes.torque = 1;
        target.node = node;
        target.jointTorqueTarget = jointTorque;
        //target.dynNode = dnyRN;

        actuationTargets[node] = target;

        if (actuationControllers.find(node) == actuationControllers.end())
        {
            actuationControllers[node] =
                VelocityMotorController(node->getMaxVelocity(), node->getMaxAcceleration());
            actuationControllers[node].reset(PID_p, PID_i, PID_d);
        }
        else
        {
            actuationControllers[node].reset();
        }
    }

    void
    DynamicsRobot::disableNodeActuation(VirtualRobot::RobotNodePtr node)
    {
        MutexLockPtr lock = getScopedLock();

        if (actuationTargets.find(node) != actuationTargets.end())
        {
            actuationTargets.erase(node);
        }
    }

    void
    DynamicsRobot::enableActuation(ActuationMode mode)
    {
        MutexLockPtr lock = getScopedLock();
        std::map<VirtualRobot::RobotNodePtr, robotNodeActuationTarget>::iterator it =
            actuationTargets.begin();

        while (it != actuationTargets.end())
        {
            it->second.actuation = mode;
            it++;
        }
    }

    void
    DynamicsRobot::disableActuation()
    {
        MutexLockPtr lock = getScopedLock();
        std::map<VirtualRobot::RobotNodePtr, robotNodeActuationTarget>::iterator it =
            actuationTargets.begin();

        while (it != actuationTargets.end())
        {
            it->second.actuation.mode = 0;
            it++;
        }
    }

    void
    DynamicsRobot::actuateJoints(double /*dt*/)
    {
    }

    bool
    DynamicsRobot::isNodeActuated(VirtualRobot::RobotNodePtr node)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(node);

        if (actuationTargets.find(node) == actuationTargets.end())
        {
            return false;
        }

        return actuationTargets[node].actuation.mode != 0;
    }

    double
    DynamicsRobot::getNodeTarget(VirtualRobot::RobotNodePtr node)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(node);

        if (actuationTargets.find(node) == actuationTargets.end())
        {
            return 0.0f;
        }

        return actuationTargets[node].jointValueTarget;
    }

    double
    DynamicsRobot::getJointAngle(VirtualRobot::RobotNodePtr /*rn*/)
    {
        return 0.0f;
    }

    double
    DynamicsRobot::getJointSpeed(VirtualRobot::RobotNodePtr /*rn*/)
    {
        return 0.0f;
    }

    double
    DynamicsRobot::getJointTargetSpeed(VirtualRobot::RobotNodePtr /*rn*/)
    {
        return 0.0f;
    }

    Eigen::Matrix4f
    DynamicsRobot::getComGlobal(const VirtualRobot::RobotNodePtr& rn)
    {
        MutexLockPtr lock = getScopedLock();
        Eigen::Matrix4f com = Eigen::Matrix4f::Identity();
        com.block(0, 3, 3, 1) = rn->getCoMLocal();
        com = rn->getGlobalPose() * com;
        return com;
    }

    Eigen::Vector3f
    DynamicsRobot::getComGlobal(const VirtualRobot::RobotNodeSetPtr& /*bodies*/)
    {
        return Eigen::Vector3f::Zero();
    }

    Eigen::Vector3f
    DynamicsRobot::getComVelocityGlobal(const VirtualRobot::RobotNodeSetPtr& /*bodies*/)
    {
        return Eigen::Vector3f::Zero();
    }

    Eigen::Vector3f
    DynamicsRobot::getLinearMomentumGlobal(const VirtualRobot::RobotNodeSetPtr& /*set*/)
    {
        return Eigen::Vector3f::Zero();
    }

    Eigen::Vector3f
    DynamicsRobot::getAngularMomentumGlobal(const VirtualRobot::RobotNodeSetPtr& /*set*/)
    {
        return Eigen::Vector3f::Zero();
    }

    Eigen::Vector3f
    DynamicsRobot::getAngularMomentumLocal(const VirtualRobot::RobotNodeSetPtr& /*set*/)
    {
        return Eigen::Vector3f::Zero();
    }

    void
    DynamicsRobot::setGlobalPose(const Eigen::Matrix4f& gp)
    {
        MutexLockPtr lock = getScopedLock();

        std::map<VirtualRobot::RobotNodePtr, DynamicsObjectPtr>::iterator it =
            dynamicRobotNodes.begin();

        robot->setGlobalPose(gp);
        while (it != dynamicRobotNodes.end())
        {
            Eigen::Matrix4f newPose = it->second->getSceneObject()->getGlobalPose();
            it->second->setPose(newPose);
            it++;
        }
    }

    void
    DynamicsRobot::setMutex(std::shared_ptr<std::recursive_mutex> engineMutexPtr)
    {
        this->engineMutexPtr = engineMutexPtr;
    }

    std::map<VirtualRobot::RobotNodePtr, VelocityMotorController>&
    DynamicsRobot::getControllers()
    {
        return actuationControllers;
    }

    bool
    DynamicsRobot::attachObject(const std::string& /*nodeName*/, DynamicsObjectPtr /*object*/)
    {
        return false;
    }

    bool
    DynamicsRobot::detachObject(DynamicsObjectPtr /*object*/)
    {
        return false;
    }

    DynamicsRobot::MutexLockPtr
    DynamicsRobot::getScopedLock()
    {
        std::shared_ptr<std::scoped_lock<std::recursive_mutex>> scoped_lock;

        if (engineMutexPtr)
        {
            scoped_lock.reset(new std::scoped_lock<std::recursive_mutex>(*engineMutexPtr));
        }

        return scoped_lock;
    }

    void
    DynamicsRobot::setPIDParameters(float p, float i, float d)
    {
        PID_p = p;
        PID_i = i;
        PID_d = d;
        for (auto& pair : actuationControllers)
        {
            pair.second.reset(p, i, d);
        }
    }

    void
    DynamicsRobot::enableSelfCollisions(bool enable)
    {
        // deactivate all self collisions
        for (const auto& i : robotNodes)
        {
            auto drn1 = getDynamicsRobotNode(i);
            for (const auto& robotNode : robotNodes)
            {
                auto drn2 = getDynamicsRobotNode(robotNode);
                if (enable)
                    DynamicsWorld::GetWorld()->getEngine()->enableCollision(drn1.get(), drn2.get());
                else
                    DynamicsWorld::GetWorld()->getEngine()->disableCollision(drn1.get(),
                                                                             drn2.get());
            }
        }
    }

    /*
    void DynamicsRobot::setPose( const Eigen::Matrix4f &pose )
    {
        robot->setGlobalPose(pose);
    }

    void DynamicsRobot::setPosition( const Eigen::Vector3f &posMM )
    {
        Eigen::Matrix4f pose = robot->getGlobalPose();
        pose.block(0,3,3,1) = posMM;
        setPose(pose);
    }*/


} // namespace SimDynamics
