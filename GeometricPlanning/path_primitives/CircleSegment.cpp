
#include "CircleSegment.h"

#include <GeometricPlanning/assert/assert.h>

namespace simox::geometric_planning
{

    CircleSegment::CircleSegment(const float radius, const ParameterRange& range) :
        Circle(radius), range(range)
    {
        // VR_INFO << "Circle parameter range " << Circle::parameterRange().min << " "
        //             << Circle::parameterRange().max;

        REQUIRE(Circle::parameterRange().isInRange(range.min));
        REQUIRE(Circle::parameterRange().isInRange(range.max));
    }

    ParameterRange
    CircleSegment::parameterRange() const
    {
        return range;
    }

} // namespace simox::geometric_planning
