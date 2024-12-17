#pragma once

#include <iterator>

namespace simox::alg
{
    decltype(auto)
    advanced(auto&& it, auto n)
    {
        auto copy = it;
        std::advance(copy, n);
        return copy;
    }
} // namespace simox::alg
