#pragma once

#include <array>

namespace simox::meta
{
    template <class T>
    struct is_std_array : std::false_type
    {
    };

    template <class T, std::size_t N>
    struct is_std_array<std::array<T, N>> : std::true_type
    {
    };

    template <class T>
    static constexpr bool is_std_array_v = is_std_array<T>::value;
} // namespace simox::meta
