#pragma once

#include <type_traits>

namespace simox::meta
{
    template <class... Ts>
    static constexpr bool are_arithmetic_v = (std::is_arithmetic_v<Ts> && ...);
}
