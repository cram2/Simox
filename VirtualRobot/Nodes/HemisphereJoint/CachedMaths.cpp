#include "CachedMaths.h"


namespace VirtualRobot::hemisphere
{

    void CachedMaths::update(const Eigen::Vector2f& actuators)
    {
        if (actuators != this->actuators)
        {
            maths.computeFkOfAngle(actuators.cast<double>());
        }
    }

}
