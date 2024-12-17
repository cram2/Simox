#pragma once

#include "is_equal.h"

namespace simox::math
{
    template <class L, class R>
    constexpr std::enable_if_t<!simox::meta::are_arithmetic_v<L, R>, bool>
    is_inequal(const L& l, const R& r)
    {
        return l != r;
    }

    template <class L, class R>
    constexpr std::enable_if_t<simox::meta::are_arithmetic_v<L, R>, bool>
    is_inequal(L l, R r)
    {
        return !is_equal(l, r);
    }

    template <class L, class R>
    constexpr std::enable_if_t<simox::meta::are_arithmetic_v<L, R>, bool>
    is_inequal(std::array<L, 1> l, std::array<R, 1> r)
    {
        return is_inequal(l.at(0), r.at(0));
    }

    template <class L, class R>
    constexpr std::enable_if_t<simox::meta::are_arithmetic_v<L, R>, bool>
    is_inequal(std::array<L, 1> l, R r)
    {
        return is_inequal(l.at(0), r);
    }

    template <class L, class R>
    constexpr std::enable_if_t<simox::meta::are_arithmetic_v<L, R>, bool>
    is_inequal(L l, std::array<R, 1> r)
    {
        return is_inequal(l, r.at(0));
    }
} // namespace simox::math
