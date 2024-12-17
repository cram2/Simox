#pragma once

#include <VirtualRobot/VirtualRobotException.h>

namespace VirtualRobot::mujoco::error
{

    /// Indicates that no body exists or has been added for a robot node.
    class NoBodyOfRobotNode : public VirtualRobotException
    {
    public:
        NoBodyOfRobotNode(const std::string& nodeName);

    private:
        static std::string makeMsg(const std::string& nodeName);
    };

    /// Indicates that a robot node has to be a joint node, but is not.
    class NodeIsNoJoint : public VirtualRobotException
    {
    public:
        NodeIsNoJoint(const std::string& nodeName);
    };


} // namespace VirtualRobot::mujoco::error
