#pragma once

#include <algorithm>
#include <map>
#include <set>
#include <type_traits>
#include <vector>

namespace simox::alg
{

    template <typename ValueIn, typename UnaryOp>
    std::vector<std::invoke_result_t<UnaryOp, ValueIn>>
    apply(const std::vector<ValueIn>& vector, const UnaryOp& op)
    {
        using ValueOut = std::invoke_result_t<UnaryOp, ValueIn>;
        std::vector<ValueOut> result;
        result.reserve(vector.size());

        std::transform(vector.begin(), vector.end(), std::back_inserter(result), op);
        return result;
    }

    template <typename ValueIn, typename UnaryOp>
    std::vector<std::invoke_result_t<UnaryOp, ValueIn>>
    apply(const std::set<ValueIn>& set, const UnaryOp& op)
    {
        using ValueOut = std::invoke_result_t<UnaryOp, ValueIn>;
        std::vector<ValueOut> result;
        result.reserve(set.size());

        std::transform(set.begin(), set.end(), std::back_inserter(result), op);
        return result;
    }

    template <typename Key, typename ValueIn, typename UnaryOp>
    std::map<Key, std::invoke_result_t<UnaryOp, ValueIn>>
    apply(const std::map<Key, ValueIn>& map, const UnaryOp& op)
    {
        using ValueOut = std::invoke_result_t<UnaryOp, ValueIn>;
        std::map<Key, ValueOut> result;
        for (const auto& [name, value] : map)
        {
            result[name] = op(value);
        }
        return result;
    }


} // namespace simox::alg
