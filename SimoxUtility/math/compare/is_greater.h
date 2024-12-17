#pragma once

#include "is_less_equal.h"

namespace simox::math
{
    template <class L, class R>
    constexpr std::enable_if_t<!simox::meta::are_arithmetic_v<L, R>, bool>
    is_greater(const L& l, const R& r)
    {
        return l > r;
    }

    template <class L, class R>
    constexpr std::enable_if_t<simox::meta::are_arithmetic_v<L, R>, bool>
    is_greater(L l, R r)
    {
        return !is_less_equal(l, r);
    }

    template <class L, class R>
    constexpr std::enable_if_t<simox::meta::are_arithmetic_v<L, R>, bool>
    is_greater(std::array<L, 1> l, std::array<R, 1> r)
    {
        return is_greater(l.at(0), r.at(0));
    }

    template <class L, class R>
    constexpr std::enable_if_t<simox::meta::are_arithmetic_v<L, R>, bool>
    is_greater(std::array<L, 1> l, R r)
    {
        return is_greater(l.at(0), r);
    }

    template <class L, class R>
    constexpr std::enable_if_t<simox::meta::are_arithmetic_v<L, R>, bool>
    is_greater(L l, std::array<R, 1> r)
    {
        return is_greater(l, r.at(0));
    }
} // namespace simox::math
