#include "periodic_diff.h"

namespace simox
{

    float
    math::periodic_diff(float a, float b, float periodLo, float periodHi)
    {
        return periodic_diff<float>(a, b, periodLo, periodHi);
    }

    double
    math::periodic_diff(double a, double b, double periodLo, double periodHi)
    {
        return periodic_diff<double>(a, b, periodLo, periodHi);
    }

} // namespace simox
