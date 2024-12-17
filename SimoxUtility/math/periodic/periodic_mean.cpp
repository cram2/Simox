#include "periodic_mean.h"

namespace simox
{

    float
    math::periodic_mean(const std::vector<float>& samples, float periodLo, float periodHi)
    {
        return periodic_mean<float>(samples, periodLo, periodHi);
    }

    double
    math::periodic_mean(const std::vector<double>& samples, double periodLo, double periodHi)
    {
        return periodic_mean<double>(samples, periodLo, periodHi);
    }

} // namespace simox
