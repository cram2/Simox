#pragma once

#include <algorithm>
#include <vector>

namespace simox::alg
{

    template <typename TypeT>
    bool
    contains(const std::vector<TypeT>& haystack, const TypeT& needle)
    {
        return std::find(haystack.begin(), haystack.end(), needle) != haystack.end();
    }

    template <typename TypeT>
    std::vector<TypeT>
    appended(const std::vector<TypeT>& v1, const std::vector<TypeT>& v2)
    {
        std::vector<TypeT> ret;
        for (const auto& el : v1)
        {
            ret.emplace_back(el);
        }
        for (const auto& el : v2)
        {
            ret.emplace_back(el);
        }
        return ret;
    }

    template <typename TypeT>
    std::vector<TypeT>&
    append(std::vector<TypeT>& v1, const std::vector<TypeT>& v2)
    {
        for (const auto& el : v2)
        {
            v1.emplace_back(el);
        }
        return v1;
    }

    template <typename TypeT>
    std::vector<TypeT>
    subvector(const std::vector<TypeT>& v, int startPos, int endPos)
    {
        auto first = v.begin() + startPos;
        auto last = v.begin() + endPos + 1;
        std::vector<TypeT> vector(first, last);
        return vector;
    }

    template <typename TypeT>
    std::vector<TypeT>
    subvector(const std::vector<TypeT>& v, int startPos)
    {
        return subvector(v, startPos, v.size() - 1);
    }

    template <typename TypeT>
    bool
    startsWith(const std::vector<TypeT>& input, const std::vector<TypeT>& search)
    {
        if (search.size() > input.size())
        {
            return false;
        }

        for (unsigned int i = 0; i < search.size(); ++i)
        {
            if (input[i] != search[i])
            {
                return false;
            }
        }
    }

    template <typename TypeT>
    bool
    endsWith(const std::vector<TypeT>& input, const std::vector<TypeT>& search)
    {
        if (search.size() > input.size())
        {
            return false;
        }

        for (unsigned int i = input.size() - search.size(); i < input.size(); ++i)
        {
            if (input[i] != search[i])
            {
                return false;
            }
        }
    }

    // Todo: replace, split, ...
} // namespace simox::alg
