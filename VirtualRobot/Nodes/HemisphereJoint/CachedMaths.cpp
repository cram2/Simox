#include "CachedMaths.h"


namespace VirtualRobot::hemisphere
{
    void CachedMaths::update(const Eigen::Vector2d& actuatorsAngle)
    {
        if (not actuatorsAngle.isApprox(_actuators, EQUALITY_PRECISION))
        {
            _actuators = actuatorsAngle;
            maths.computeFkOfAngle(_actuators);
        }
    }
}
