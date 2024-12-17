#pragma once

#include <map>
#include <set>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>

namespace simox::meta
{
    template <class T, class = void>
    struct key_type
    {
    };

    template <class T>
    struct key_type<const T> : key_type<T>
    {
    };

    template <class T>
    struct key_type<const T&> : key_type<T>
    {
    };

    template <class T>
    struct key_type<T&> : key_type<T>
    {
    };

    template <class KeyT, class ValueT, class... OtherT>
    struct key_type<std::map<KeyT, ValueT, OtherT...>, void>
    {
        using type = KeyT;

        static decltype(auto)
        key_from_iterator(auto&& i)
        {
            return i->first;
        }
    };

    template <class KeyT, class ValueT, class... OtherT>
    struct key_type<std::unordered_map<KeyT, ValueT, OtherT...>, void>
    {
        using type = KeyT;

        static decltype(auto)
        key_from_iterator(auto&& i)
        {
            return i->first;
        }
    };

    template <class KeyT, class... OtherT>
    struct key_type<std::set<KeyT, OtherT...>, void>
    {
        using type = KeyT;

        static decltype(auto)
        key_from_iterator(auto&& i)
        {
            return *i;
        }
    };

    template <class KeyT, class... OtherT>
    struct key_type<std::unordered_set<KeyT, OtherT...>, void>
    {
        using type = KeyT;

        static decltype(auto)
        key_from_iterator(auto&& i)
        {
            return *i;
        }
    };

    template <class T>
    using key_type_t = typename key_type<T>::type;
} // namespace simox::meta
