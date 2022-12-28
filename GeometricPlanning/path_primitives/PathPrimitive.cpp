#include "PathPrimitive.h"

#include <Eigen/Geometry>

#include <VirtualRobot/math/Helpers.h>

namespace simox::geometric_planning
{
    bool
    ParameterRange::isInRange(const float t) const noexcept
    {
        return t >= min && t <= max;
    }

    float
    PathPrimitive::progress(const Pose& pose) const
    {
        const float param = parameter(pose);
        const auto range = parameterRange();

        return (param - range.min) / (range.max - range.min);
    }

    Pose
    PathPrimitive::getPose(const float t) const
    {
        return Pose(::math::Helpers::CreatePose(getPosition(t), getOrientation(t)));
    }

    Eigen::Vector3f
    PathPrimitive::GetPosition(float t)
    {
        return static_cast<const PathPrimitive*>(this)->getPosition(t);
    }

    Eigen::Vector3f
    PathPrimitive::GetPositionDerivative(float t)
    {
        return static_cast<const PathPrimitive*>(this)->getPositionDerivative(t);
    }

    Eigen::Quaternionf
    PathPrimitive::GetOrientation(float t)
    {
        return static_cast<const PathPrimitive*>(this)->getOrientation(t);
    }

    Eigen::Vector3f
    PathPrimitive::GetOrientationDerivative(float t)
    {
        return static_cast<const PathPrimitive*>(this)->getOrientationDerivative(t);
    }

} // namespace simox::geometric_planning
