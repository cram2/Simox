

#pragma once

#include <GeometricPlanning/path_primitives/PathPrimitive.h>

namespace simox::geometric_planning
{

    /**
     * @brief The circle class.
     *
     * The circle is defined in the xy-plane, starting at r*(1,0,0)^T and rotating around the z-axis
     *
     */
    class Circle : virtual public PathPrimitive
    {
    public:
        Circle(float radius);

        Eigen::Vector3f getPosition(float t) const override;
        Eigen::Vector3f getPositionDerivative(float t) const override;
        Eigen::Quaternionf getOrientation(float t) const override;
        Eigen::Vector3f getOrientationDerivative(float t) const override;

        float
        getRadius() const
        {
            return radius;
        }

        ParameterRange parameterRange() const override;
        float parameter(const Pose& pose) const override;

    private:
        float radius;
    };

} // namespace simox::geometric_planning
