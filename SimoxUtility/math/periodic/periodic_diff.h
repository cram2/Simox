#pragma once

#include <cmath>
#include <type_traits>

#include <Eigen/Core>

#include "periodic_clamp.h"

namespace simox::math
{

    /**
     * @brief Computes the periodic (cyclic, wrapped) difference `a - b`
     * over the cyclic interval `[periodLo, periodHi]`.
     */
    template <class Float, std::enable_if_t<std::is_floating_point_v<Float>, int> = 0>
    inline Float
    periodic_diff(Float a, Float b, Float periodLo, Float periodHi)
    {
        /* Python:
        a, b = wrap_angles([a, b])
        diffs = [a - b, (a + 2 * np.pi) - b, (a - 2 * np.pi) - b]
        i_min = np.argmin(np.abs(diffs))
        return diffs[int(i_min)]
        */
        a = periodic_clamp(a, periodLo, periodHi);
        b = periodic_clamp(b, periodLo, periodHi);

        const Float period = periodHi - periodLo;

        Eigen::Array<Float, 3, 1> diffs = {a - b, (a + period) - b, (a - period) - b};

        int i = 0;
        diffs.abs().minCoeff(&i);
        return diffs(i);
    }

    // Pre-compiled versions.

    float periodic_diff(float a, float b, float periodLo, float periodHi);
    double periodic_diff(double a, double b, double periodLo, double periodHi);

} // namespace simox::math
