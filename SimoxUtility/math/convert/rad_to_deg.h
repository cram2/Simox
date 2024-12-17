#pragma once

#include <cmath>
#include <type_traits>

namespace simox::math
{
    template <class FloatT, typename = std::enable_if_t<std::is_floating_point_v<FloatT>>>
    FloatT
    rad_to_deg(FloatT rad)
    {
        return rad * static_cast<FloatT>(180. / M_PI);
    }

    template <class Derived,
              typename = std::enable_if_t<std::is_floating_point_v<typename Derived::Scalar>>>
    auto
    rad_to_deg(const Eigen::MatrixBase<Derived>& rad)
    {
        return rad * static_cast<typename Derived::Scalar>(180.0 / M_PI);
    }
} // namespace simox::math
