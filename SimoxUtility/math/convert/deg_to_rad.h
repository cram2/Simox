#pragma once

#include <cmath>
#include <type_traits>

#include <eigen3/Eigen/Core>

namespace simox::math
{
    template <class FloatT, typename = std::enable_if_t<std::is_floating_point_v<FloatT>>>
    FloatT
    deg_to_rad(FloatT deg)
    {
        return deg * static_cast<FloatT>(M_PI / 180.);
    }

    template <class Derived,
              typename = std::enable_if_t<std::is_floating_point_v<typename Derived::Scalar>>>
    auto
    deg_to_rad(const Eigen::MatrixBase<Derived>& deg)
    {
        return deg * static_cast<typename Derived::Scalar>(M_PI / 180.);
    }

} // namespace simox::math
