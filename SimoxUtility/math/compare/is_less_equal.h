#pragma once

#include <array>
#include <type_traits>

#include "../../meta/type_traits/are_arithmetic.h"

namespace simox::math
{
    template <class L, class R>
    constexpr std::enable_if_t<!simox::meta::are_arithmetic_v<L, R>, bool>
    is_less_equal(const L& l, const R& r)
    {
        return l <= r;
    }

    template <class L, class R>
    constexpr std::enable_if_t<simox::meta::are_arithmetic_v<L, R>, bool>
    is_less_equal(L l, R r)
    {
        constexpr bool signed_l = std::is_signed_v<L>;
        constexpr bool signed_r = std::is_signed_v<R>;
        constexpr bool floating_l = std::is_floating_point_v<L>;
        constexpr bool floating_r = std::is_floating_point_v<R>;
        if constexpr (floating_l != floating_r)
        {
            //same sign -> cast to bigger type and compare
            using floating_t = std::conditional_t<floating_l, L, R>;
            return static_cast<floating_t>(l) <= static_cast<floating_t>(r);
        }
        else if constexpr (signed_l == signed_r)
        {
            //same sign -> cast to bigger type and compare
            using big_t = std::conditional_t<(sizeof(L) > sizeof(R)), L, R>;
            return static_cast<big_t>(l) <= static_cast<big_t>(r);
        }
        else if constexpr (signed_l)
        {
            // signed_l = 1, signed_r = 0
            if (l < 0)
            {
                return true;
            }
            // l>=0, r>=0
            //if same size use unsigned type, otherwise the bigger one
            using big_t = std::conditional_t<(sizeof(L) > sizeof(R)), L, R>;
            return static_cast<big_t>(l) <= static_cast<big_t>(r);
        }
        else
        {
            // signed_l = 0, signed_r = 1
            if (r < 0)
            {
                return false;
            }
            // l>=0, r>=0
            //if same size use unsigned type, otherwise the bigger one
            using big_t = std::conditional_t<(sizeof(L) < sizeof(R)), R, L>;
            return static_cast<big_t>(l) <= static_cast<big_t>(r);
        }
    }

    template <class L, class R>
    constexpr std::enable_if_t<simox::meta::are_arithmetic_v<L, R>, bool>
    is_less_equal(std::array<L, 1> l, std::array<R, 1> r)
    {
        return is_less_equal(l.at(0), r.at(0));
    }

    template <class L, class R>
    constexpr std::enable_if_t<simox::meta::are_arithmetic_v<L, R>, bool>
    is_less_equal(std::array<L, 1> l, R r)
    {
        return is_less_equal(l.at(0), r);
    }

    template <class L, class R>
    constexpr std::enable_if_t<simox::meta::are_arithmetic_v<L, R>, bool>
    is_less_equal(L l, std::array<R, 1> r)
    {
        return is_less_equal(l, r.at(0));
    }
} // namespace simox::math
