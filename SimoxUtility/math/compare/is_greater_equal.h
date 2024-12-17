#pragma once

#include "is_less.h"

namespace simox::math
{
    template <class L, class R>
    constexpr std::enable_if_t<!simox::meta::are_arithmetic_v<L, R>, bool>
    is_greater_equal(const L& l, const R& r)
    {
        return l >= r;
    }

    template <class L, class R>
    constexpr std::enable_if_t<simox::meta::are_arithmetic_v<L, R>, bool>
    is_greater_equal(L l, R r)
    {
        return !is_less(l, r);
    }

    template <class L, class R>
    constexpr std::enable_if_t<simox::meta::are_arithmetic_v<L, R>, bool>
    is_greater_equal(std::array<L, 1> l, std::array<R, 1> r)
    {
        return is_greater_equal(l.at(0), r.at(0));
    }

    template <class L, class R>
    constexpr std::enable_if_t<simox::meta::are_arithmetic_v<L, R>, bool>
    is_greater_equal(std::array<L, 1> l, R r)
    {
        return is_greater_equal(l.at(0), r);
    }

    template <class L, class R>
    constexpr std::enable_if_t<simox::meta::are_arithmetic_v<L, R>, bool>
    is_greater_equal(L l, std::array<R, 1> r)
    {
        return is_greater_equal(l, r.at(0));
    }
} // namespace simox::math
