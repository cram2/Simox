

#pragma once

#include "Circle.h"

namespace simox::geometric_planning
{

    class CircleSegment : virtual public Circle
    {
    public:
        CircleSegment(float radius, const ParameterRange& range);
        ParameterRange parameterRange() const override;

    private:
        ParameterRange range;
    };

} // namespace simox::geometric_planning
