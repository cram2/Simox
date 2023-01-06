#include "Circle.h"

#include <cmath>

#include <Eigen/Geometry>

#include <GeometricPlanning/assert/assert.h>


namespace simox::geometric_planning
{

    Circle::Circle(const float radius) : radius(radius)
    {
    }

    ParameterRange
    Circle::parameterRange() const
    {
        return {.min = 0, .max = 2 * M_PI};
    }

    Eigen::Vector3f
    Circle::getPosition(float t) const
    {
        REQUIRE(parameterRange().isInRange(t));

        return radius * Eigen::Vector3f(std::cos(t), std::sin(t), 0.0F);
    }

    Eigen::Vector3f
    Circle::getPositionDerivative([[maybe_unused]] float t) const
    {
        REQUIRE(parameterRange().isInRange(t));

        // return radius * Eigen::Vector3f(-std::sin(t + M_PI_2f32), std::cos(t + M_PI_2f32), 0.0F);

        // the position derivative is always pointing into x direction as orientation changes as well
        // FIXME update this according to getPosition!
        return radius * -Eigen::Vector3f::UnitX();
    }

    Eigen::Quaternionf
    Circle::getOrientation(float t) const
    {
        // x pointing into tangential direction
        // return Eigen::Quaternionf::Identity(); //(Eigen::AngleAxisf(t, Eigen::Vector3f::UnitZ()));
        return Eigen::Quaternionf(Eigen::AngleAxisf(t, Eigen::Vector3f::UnitZ()));
    }

    Eigen::Vector3f
    Circle::getOrientationDerivative(float /*t*/) const
    {
        return Eigen::Vector3f::UnitZ();
    }

    float
    Circle::parameter(const Pose& pose) const
    {
        // ARMARX_DEBUG << "Local pose for parameter calculation " << pose.matrix();

        const float x = pose.translation().x();
        const float y = pose.translation().y();

        const float phi = std::atan2(y, x); // (-pi,pi]

        // -> the parameter is defined to be in range [0, 2pi)
        // if y is positive or zero, phi is in range [0,pi]
        // if y is negative, phi is in range (-pi, 0)
        // const float param = [&]() -> float
        // {
        //     if (y < 0)
        //     {
        //         return phi + M_PIf32;
        //     }

        //     return phi;
        // }();

        const float param = phi;
        // ARMARX_DEBUG << "Param is " << param;

        return std::clamp(param, parameterRange().min, parameterRange().max);
    }

} // namespace simox::geometric_planning
