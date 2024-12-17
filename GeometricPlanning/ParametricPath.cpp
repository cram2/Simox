#include "ParametricPath.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <VirtualRobot/RobotNodeSet.h>

#include "VirtualRobot/Nodes/RobotNode.h"

namespace simox::geometric_planning
{

    float
    ParametricPath::parameter(const Pose& global_T_pose) const
    {
        return path->parameter(toLocalPathFrame(global_T_pose));
    }

    float
    ParametricPath::progress(const Pose& global_T_pose) const
    {
        return path->progress(toLocalPathFrame(global_T_pose));
    }

    float
    ParametricPath::progress(const float param) const
    {
        return path->progress(param);
    }

    Pose
    ParametricPath::toLocalPathFrame(const Pose& global_T_pose) const
    {
        return (Pose(frame->getGlobalPose())).inverse() * global_T_pose; // * path_T_pose.inverse();
    }

    Eigen::Vector3f
    ParametricPath::getPosition(float t) const
    {
        // translation only

        const Pose frame_T_path(Eigen::Translation3f(path->getPosition(t)));

        return (Pose(frame->getGlobalPose()) * frame_T_path * path_T_pose).translation();
    }

    Eigen::Vector3f
    ParametricPath::getPositionDerivative(float t) const
    {
        // rotation only
        Pose frame_T_path_deriv = Pose::Identity();
        frame_T_path_deriv.translation() = path->getPositionDerivative(t);
        // frame_T_path_deriv.linear() = path->getOrientation(t).toRotationMatrix();

        Pose preRotate(Pose(frame->getGlobalPose()).linear() *
                       path->getOrientation(t).toRotationMatrix());
        Pose postRotate(path_T_pose.linear());

        // Pose T = Pose::Identity();

        return (preRotate * frame_T_path_deriv * postRotate).translation();

        // return R * path->getPositionDerivative(t) ;
    }

    Eigen::Quaternionf
    ParametricPath::getOrientation(float t) const
    {
        // rotation only
        const Pose frame_T_path(path->getOrientation(t));

        const Eigen::Quaternionf ori(
            (Pose(frame->getGlobalPose()) * frame_T_path * path_T_pose).linear());

        return ori;
    }

    Eigen::Vector3f
    ParametricPath::getOrientationDerivative(float t) const
    {
        const Eigen::Quaternionf frameOri((Pose(frame->getGlobalPose()) * path_T_pose).linear());

        return frameOri.toRotationMatrix() * path->getOrientationDerivative(t);
    }

    ParametricPath::ParametricPath(const VirtualRobot::RobotNodePtr& frame,
                                   const std::shared_ptr<PathPrimitive>& path,
                                   const Pose& postTransform) :
        frame(frame), path(path), path_T_pose(postTransform)
    {
    }

    ParameterRange
    ParametricPath::parameterRange() const
    {
        return path->parameterRange();
    }
} // namespace simox::geometric_planning
