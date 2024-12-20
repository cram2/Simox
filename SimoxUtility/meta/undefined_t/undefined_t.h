#pragma once

#include <type_traits>

namespace simox::meta
{
    struct undefined_t
    {
    };
} // namespace simox::meta

namespace simox::meta
{
    template <class... Ts>
    struct first_not_undefined
    {
        using type = undefined_t;
    };

    template <class T, class... Ts>
    struct first_not_undefined<T, Ts...>
    {
        using type = T;
    };

    template <class... Ts>
    struct first_not_undefined<undefined_t, Ts...> : first_not_undefined<Ts...>
    {
    };

    template <class... Ts>
    using first_not_undefined_t = typename first_not_undefined<Ts...>::type;
} // namespace simox::meta
