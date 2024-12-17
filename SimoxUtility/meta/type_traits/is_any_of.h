#pragma once

#include <type_traits>

namespace simox::meta
{
    template <class T, class... Ts>
    static constexpr bool is_any_of_v = (std::is_same_v<T, Ts> || ...);

    template <class T, class... Ts>
    struct is_any_of : std::bool_constant<is_any_of_v<T, Ts...>>
    {
    };
} // namespace simox::meta
