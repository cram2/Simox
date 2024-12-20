#pragma once

#include <algorithm>
#include <functional>
#include <vector>

namespace simox::alg
{

    /// Return a function comparing the results of applying `unaryFunc` to values of type `T`.
    template <class T, class Scalar>
    std::function<bool(const T&, const T&)>
    get_compare_fn(const std::function<Scalar(const T&)> unaryFunc)
    {
        return [unaryFunc](const auto& lhs, const auto& rhs)
        { return unaryFunc(lhs) < unaryFunc(rhs); };
    }

    /// @brief Get the maximum of applying `unaryFunc` to `values`.
    template <class T, class Scalar>
    Scalar
    min(const std::vector<T>& values, std::function<Scalar(const T&)> unaryFunc)
    {
        return unaryFunc(
            *std::min_element(values.begin(), values.end(), get_compare_fn(unaryFunc)));
    }

    /// @brief Get the maximum of applying `unaryFunc` to `values`
    template <class T, class Scalar>
    Scalar
    max(const std::vector<T>& values, std::function<Scalar(const T&)> unaryFunc)
    {
        return unaryFunc(
            *std::max_element(values.begin(), values.end(), get_compare_fn(unaryFunc)));
    }

    /// @brief Get minimum and maximum of applying `unaryFunc` to `values`
    template <class T, class Scalar>
    std::pair<Scalar, Scalar>
    minmax(const std::vector<T>& values, std::function<Scalar(const T&)> unaryFunc)
    {
        const auto [min, max] =
            std::minmax_element(values.begin(), values.end(), get_compare_fn(unaryFunc));
        return std::make_pair(*min, *max);
    }

} // namespace simox::alg
