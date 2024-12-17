#pragma once

#include "../meta/key_type.h"
#include "../meta/template_traits/is_instantiation_of.h"
#include "advanced.h"

namespace simox::alg
{
    template <class ContT, class GetKey>
    inline decltype(auto)
    fuzzy_find(ContT& cont,
               meta::key_type_t<ContT> key,
               meta::key_type_t<ContT> accuracy,
               GetKey get_key)
    {
        if (cont.empty())
        {
            return cont.end();
        }
        auto fst_not_lt = cont.lower_bound(key);
        if (fst_not_lt == cont.end())
        {
            //no element is greater than, but one is smaller
            auto smaller = advanced(fst_not_lt, -1);
            const auto delta_to_smaller = key - get_key(smaller);
            return delta_to_smaller <= accuracy ? smaller : cont.end();
        }
        const auto delta_to_larger = get_key(fst_not_lt) - key;
        if (fst_not_lt == cont.begin())
        {
            //no element is smaller
            return delta_to_larger <= accuracy ? fst_not_lt : cont.end();
        }
        //some element is greater and some smaller
        auto smaller = advanced(fst_not_lt, -1);
        const auto delta_to_smaller = key - get_key(smaller);
        return (delta_to_larger < delta_to_smaller)
                   ? (delta_to_larger <= accuracy ? fst_not_lt : cont.end())
                   : (delta_to_smaller <= accuracy ? smaller : cont.end());
    }

    template <class ContT>
    inline decltype(auto)
    fuzzy_find(ContT& cont, meta::key_type_t<ContT> key, meta::key_type_t<ContT> accuracy)
    {
        using cont_t = std::remove_cv_t<ContT>;
        if constexpr (simox::meta::is_instantiation_of_v<std::map, cont_t>)
        {
            return fuzzy_find(cont, key, accuracy, [](auto&& i) { return i->first; });
        }
        else if constexpr (simox::meta::is_instantiation_of_v<std::set, cont_t>)
        {
            return fuzzy_find(cont, key, accuracy, [](auto&& i) { return *i; });
        }
        else
        {
            static_assert(!std::is_same_v<ContT, ContT>,
                          "You need to provide a key access functor");
        }
    }
} // namespace simox::alg
