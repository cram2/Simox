#pragma once

#include <type_traits>

namespace simox::math
{

    template <class Float, std::enable_if_t<std::is_floating_point_v<Float>, int> = 0>
    inline
        /// Rescale a scalar `value` from interval `[from_lo, from_hi]` to `[to_lo, to_hi]`.
        Float
        rescale(Float value, Float from_lo, Float from_hi, Float to_lo, Float to_hi)
    {
        // value: [from_lo, from_hi]
        Float norm = (value - from_lo) / (from_hi - from_lo); // [0, 1]
        return norm * (to_hi - to_lo) + to_lo; // [to_lo, to_hi]
    }

} // namespace simox::math
