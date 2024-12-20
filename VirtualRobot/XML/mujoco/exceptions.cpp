#include "exceptions.h"

namespace VirtualRobot::mujoco::error
{

    NoBodyOfRobotNode::NoBodyOfRobotNode(const std::string& nodeName) :
        VirtualRobotException(makeMsg(nodeName))
    {
    }

    std::string
    NoBodyOfRobotNode::makeMsg(const std::string& nodeName)
    {
        return "No body of robot node " + nodeName + ". Add a node body beforehand.";
    }

    NodeIsNoJoint::NodeIsNoJoint(const std::string& nodeName) :
        VirtualRobotException("Node '" + nodeName + "' is no joint node.")
    {
    }


} // namespace VirtualRobot::mujoco::error
