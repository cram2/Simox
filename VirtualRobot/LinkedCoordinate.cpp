#include "LinkedCoordinate.h"

#include <Eigen/Dense>

#include "Nodes/RobotNode.h"
#include "Robot.h"
#include "VirtualRobotException.h"


using namespace VirtualRobot;
using namespace std;

LinkedCoordinate::LinkedCoordinate(const LinkedCoordinate& other)
{
    robot = other.robot;
    pose = other.pose;
    frame = other.frame;
}

LinkedCoordinate& LinkedCoordinate::operator=(const LinkedCoordinate& other) = default;

LinkedCoordinate::~LinkedCoordinate()
{
    frame.reset();
    robot.reset();
}

void
LinkedCoordinate::set(const RobotNodePtr& frame, const Eigen::Matrix4f& pose)
{
    if (!frame)
    {
        THROW_VR_EXCEPTION("RobotNodePtr not assigned");
    }

    if (!this->robot->hasRobotNode(frame))
    {
        THROW_VR_EXCEPTION("Robot node not a member of robot");
    }

    this->pose = pose;
    this->frame = frame;
}

void
LinkedCoordinate::set(const std::string& frame, const Eigen::Matrix4f& pose)
{
    if (!this->robot->hasRobotNode(frame))
    {
        THROW_VR_EXCEPTION("Robot node not a member of robot");
    }

    this->set(this->robot->getRobotNode(frame), pose);
}

void
LinkedCoordinate::set(const std::string& frame, const Eigen::Vector3f& coordinate)
{
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose.block<3, 1>(0, 3) = coordinate;
    this->set(frame, pose);
}

void
LinkedCoordinate::set(const RobotNodePtr& frame, const Eigen::Vector3f& coordinate)
{
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose.block<3, 1>(0, 3) = coordinate;
    this->set(frame, pose);
}

void
LinkedCoordinate::changeFrame(const RobotNodePtr& destination)
{
    if (!destination)
    {
        THROW_VR_EXCEPTION("RobotNodePtr not assigned");
    }

    if (!this->robot->hasRobotNode(destination))
    {
        THROW_VR_EXCEPTION("Robot node not a member of robot");
    }

    if (!this->frame)
    {
        this->pose = Eigen::Matrix4f::Identity();
    }
    else
    {
        this->pose =
            LinkedCoordinate::getCoordinateTransformation(this->frame, destination, this->robot) *
            this->pose;
    }

    this->frame = destination;
}

void
LinkedCoordinate::changeFrame(const std::string& destination)
{
    if (!this->robot->hasRobotNode(destination))
    {
        THROW_VR_EXCEPTION("Robot node not a member of robot");
    }

    this->changeFrame(this->robot->getRobotNode(destination));
}

Eigen::Matrix4f
LinkedCoordinate::getInFrame(const RobotNodePtr& destination) const
{
    if (!destination)
    {
        THROW_VR_EXCEPTION("RobotNodePtr not assigned");
    }

    if (!this->robot->hasRobotNode(destination))
    {
        THROW_VR_EXCEPTION("Robot node not a member of robot");
    }

    return LinkedCoordinate::getCoordinateTransformation(this->frame, destination, this->robot) *
           this->pose;
}

Eigen::Matrix4f
LinkedCoordinate::getInFrame(const std::string& destination) const
{
    if (!this->robot->hasRobotNode(destination))
    {
        THROW_VR_EXCEPTION("Robot node not a member of robot");
    }

    return LinkedCoordinate::getCoordinateTransformation(
               this->frame, this->robot->getRobotNode(destination), this->robot) *
           this->pose;
}

Eigen::Matrix4f
LinkedCoordinate::getCoordinateTransformation(const RobotNodePtr& origin,
                                              const RobotNodePtr& destination,
                                              const RobotPtr& robot)
{

    if (!destination)
    {
        THROW_VR_EXCEPTION("Destination RobotNodePtr not assigned ");
    }

    if (!origin)
    {
        THROW_VR_EXCEPTION("Origin RobotNodePtr not assigned");
    }

    if (!robot)
    {
        THROW_VR_EXCEPTION("RobotPtr not assigned");
    }

    if (!robot->hasRobotNode(origin))
    {
        THROW_VR_EXCEPTION("Origin robot node not a member of robot");
    }

    if (!robot->hasRobotNode(destination))
    {
        THROW_VR_EXCEPTION("Destination robot node not a member of robot");
    }

    //  std::cout << "Destination: " << destination->getName() <<std::endl << "Origin: " << origin->getName() << std::endl;
    //  std::cout << "Destination:\n" << destination->getGlobalPose() <<std::endl << "Origin:\n" << origin->getGlobalPose() << std::endl;
    return destination->getGlobalPose().inverse() * origin->getGlobalPose();
}
