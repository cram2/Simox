#pragma once

namespace simox::math
{
    template <typename Float>
    inline constexpr Float
    scale_value_from_to(Float val, Float from_lo, Float from_hi, Float to_lo, Float to_hi)
    {
        return to_lo + (val - from_lo) / (from_hi - from_lo) * (to_hi - to_lo);
    }

} // namespace simox::math
