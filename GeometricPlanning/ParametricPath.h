#pragma once

#include <memory>

#include <Eigen/Geometry>

#include <GeometricPlanning/path_primitives/PathPrimitive.h>
#include <GeometricPlanning/types.h>
#include <VirtualRobot/VirtualRobot.h>

namespace simox::geometric_planning
{

    using ParametricPathPtr = std::shared_ptr<class ParametricPath>;

    class ParametricPath : virtual public PathPrimitive
    {
    public:
        ParametricPath(const VirtualRobot::RobotNodePtr& frame,
                       const std::shared_ptr<PathPrimitive>& path,
                       const Pose& postTransform);

        //! reference frame describes the movement
        VirtualRobot::RobotNodePtr frame;

        //! the movement as a parameterized path
        std::shared_ptr<PathPrimitive> path;

        Pose path_T_pose;

        // helper function to obtain the PathPrimitive's pose directly in the global frame

        // helper functions to obtain the PathPrimitive's parameter and progress directly from a global pose
        float progress(const Pose& global_T_pose) const;
        float progress(float param) const;

        Pose toLocalPathFrame(const Pose& global_T_pose) const;

        float parameter(const Pose& global_T_pose) const override;
        ParameterRange parameterRange() const override;

        Eigen::Vector3f getPosition(float t) const override;
        Eigen::Vector3f getPositionDerivative(float t) const override;
        Eigen::Quaternionf getOrientation(float t) const override;
        Eigen::Vector3f getOrientationDerivative(float t) const override;
    };

} // namespace simox::geometric_planning
