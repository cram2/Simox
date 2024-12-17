#pragma once

#include <array>

#include "undefined_t.h"

namespace simox::meta
{
    template <class T>
    constexpr bool
    is_undefined_t()
    {
        return std::is_same_v<T, undefined_t>;
    }

    template <class T>
    constexpr bool
    is_undefined_t(T)
    {
        return is_undefined_t<T>();
    }

    template <class T>
    constexpr bool
    is_not_undefined_t()
    {
        return !is_undefined_t<T>();
    }

    template <class T>
    constexpr bool
    is_not_undefined_t(T)
    {
        return !is_undefined_t<T>();
    }

    template <class Expected, class T>
    constexpr bool
    has_type_of()
    {
        return std::is_same_v<T, Expected>;
    }

    template <class Expected, class T>
    constexpr bool
    has_type_of(const T&)
    {
        return has_type_of<Expected, T>();
    }

    template <class Expected, class T>
    constexpr bool
    undefined_t_or_type()
    {
        return !is_not_undefined_t<T>() || has_type_of<Expected, T>();
    }

    template <class Expected, class T>
    constexpr bool
    undefined_t_or_type(T)
    {
        return undefined_t_or_type<Expected, T>();
    }

    template <class Expected, class T>
    constexpr bool
    undefined_t_or_type_or_array_sz_1()
    {
        return undefined_t_or_type<Expected, T>() || has_type_of<std::array<Expected, 1>, T>();
    }

    template <class Expected, class T>
    constexpr bool
    undefined_t_or_type_or_array_sz_1(T)
    {
        return undefined_t_or_type_or_array_sz_1<Expected, T>();
    }
} // namespace simox::meta
