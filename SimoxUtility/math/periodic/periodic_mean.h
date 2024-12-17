#pragma once

#include <cmath>
#include <type_traits>

#include <Eigen/Core>

#include <SimoxUtility/math/rescale.h>

namespace simox::math
{

    /**
     * @brief Computes the periodic (cyclic, wrapped) difference `a - b`
     * over the cyclic interval `[periodLo, periodHi]`.
     */
    template <class Float, std::enable_if_t<std::is_floating_point_v<Float>, int> = 0>
    inline Float
    periodic_mean(const std::vector<Float>& samples, Float periodLo, Float periodHi)
    {
        /* Python: (when samples in [0, 2*pi]
        nominator = np.sum(np.sin(samples))
        denominator = np.sum(np.cos(samples))
        return np.arctan2(nominator, denominator)
        */

        Float nominator = 0;
        Float denominator = 0;
        for (auto sample : samples)
        {
            auto scaled = rescale(sample, periodLo, periodHi, Float(0), Float(2 * M_PI));
            nominator += std::sin(scaled);
            denominator += std::cos(scaled);
        }
        Float result = std::atan2(nominator, denominator);
        return rescale(result, Float(0), Float(2 * M_PI), periodLo, periodHi);
    }

    // Pre-compiled versions.

    float periodic_mean(const std::vector<float>& samples, float periodLo, float periodHi);
    double periodic_mean(const std::vector<double>& samples, double periodLo, double periodHi);

} // namespace simox::math
