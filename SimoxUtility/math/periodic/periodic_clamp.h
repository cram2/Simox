#pragma once

#include <cmath>
#include <type_traits>

namespace simox::math
{
    template <class T>
    inline std::enable_if_t<std::is_floating_point_v<T>, T>
    periodic_clamp(T value, T periodLo, T periodHi)
    {
        const T dist = periodHi - periodLo;
        return std::fmod(std::fmod(value - periodLo, dist) + dist, dist) + periodLo;
    }
} // namespace simox::math
