#pragma once

#include <Eigen/Geometry>

#include <GeometricPlanning/types.h>
#include <GeometricPlanning/path_primitives/PathPrimitive.h>

namespace simox::geometric_planning
{

    class Line : virtual public PathPrimitive
    {
    public:
        Line(const ParameterRange& parameterRange);

        Eigen::Vector3f getPosition(float t) const override;
        Eigen::Vector3f getPositionDerivative(float t) const override;
        Eigen::Quaternionf getOrientation(float t) const override;
        Eigen::Vector3f getOrientationDerivative(float t) const override;

        ParameterRange parameterRange() const override;

        float parameter(const Pose& pose) const override;

    private:
        ParameterRange range;
    };

} // namespace simox::geometric_planning
