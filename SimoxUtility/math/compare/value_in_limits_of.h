#pragma once

#include "is_greater_equal.h"
#include "is_less_equal.h"

namespace simox::math
{
    template <class TargetT, class T>
    constexpr std::enable_if_t<simox::meta::are_arithmetic_v<TargetT, T>, bool>
    value_in_limits_of_type(T v)
    {
        return simox::math::is_less_equal(std::numeric_limits<TargetT>::lowest(), v) &&
               simox::math::is_greater_equal(std::numeric_limits<TargetT>::max(), v);
    }

    template <class TargetT, class T>
    constexpr std::enable_if_t<simox::meta::are_arithmetic_v<TargetT, T>, bool>
    value_in_limits_of_type(std::array<T, 1> v)
    {
        return value_in_limits_of_type<TargetT>(v.at(0));
    }
} // namespace simox::math
