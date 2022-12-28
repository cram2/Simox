#include "Line.h"

#include <algorithm>

#include <GeometricPlanning/assert/assert.h>

namespace simox::geometric_planning
{

    Line::Line(const ParameterRange& parameterRange) : range(parameterRange)
    {
    }

    ParameterRange
    Line::parameterRange() const
    {
        return range;
    }

    Eigen::Vector3f
    Line::getPosition(float t) const
    {
        REQUIRE(parameterRange().isInRange(t));

        return t * Eigen::Vector3f(1.0F, 0.0F, 0.0F);
    }

    Eigen::Vector3f
    Line::getPositionDerivative(float t) const
    {
        REQUIRE(parameterRange().isInRange(t));

        return Eigen::Vector3f::UnitX();
    }

    Eigen::Quaternionf
    Line::getOrientation(float /*t*/) const
    {
        return Eigen::Quaternionf::Identity();
    }

    Eigen::Vector3f
    Line::getOrientationDerivative(float /*t*/) const
    {
        return Eigen::Vector3f::Zero();
    }

    float
    Line::parameter(const Pose& pose) const
    {
        // we only need to consider the x-coordinate; see GetPosition()
        const float param = pose.translation().x();

        return std::clamp(param, parameterRange().min, parameterRange().max);
    }

} // namespace simox::geometric_planning
