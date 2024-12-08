#pragma once

#include <algorithm>
#include <string>
#include <map>


namespace simox::alg
{

    // For overloads of `simox::alg::contains` taking `std::string`:
    // #include <SimoxUtility/algorithm/string/string_tools.h>

    // GENERAL CONTAINERS

    /**
     * Return true if `value` is an element of `container`, false otherwise.
     */
    template <class ContainerT, class ValueT>
    bool contains(const ContainerT& container, const ValueT& value)
    {
        return std::find_if(container.begin(), container.end(), [&value](const auto& v)
        {
            return v == value;
        }) != container.end();
    }

    /**
     * Return true if `value` is an element of `container` (as indicated by `predicate`),
     * false otherwise.
     */
    template <class ContainerT, class ValueT, class PredicateT>
    bool contains(const ContainerT& container, const ValueT& value, const PredicateT& predicate)
    {
        return std::find_if(container.begin(), container.end(), [&value, &predicate](const auto& v)
        {
            return predicate(v, value);
        }) != container.end();
    }


    // MAPS

    /**
     * Return true if `key` is a key in `map`, false otherwise.
     */
    template <class K, class V, template<class...> class MapT = std::map, class...Ts>
    bool contains_key(const MapT<K, V, Ts...>& map, const K& key)
    {
        return map.count(key) > 0;
    }

    /**
     * Return true if `value` is a value in `map`, false otherwise.
     */
    template <class K, class V, template<class...> class MapT = std::map, class...Ts>
    bool contains_value(const MapT<K, V, Ts...>& map, const V& value)
    {
        return std::find_if(map.begin(), map.end(), [&value](const std::pair<K, V>& item)
        {
            return item.second == value;
        }) != map.end();
    }


    // Overloads for string literals (which otherwise don't match the previous definition).
    template <class V, template<class...> class MapT = std::map, class...Ts>
    bool contains_key(const MapT<std::string, V, Ts...>& map, const std::string& key)
    {
        return map.count(key) > 0;
    }
    template <class K, template<class...> class MapT = std::map, class...Ts>
    bool contains_value(const MapT<K, std::string, Ts...>& map, const std::string& value)
    {
        return contains_value<K, std::string>(map, value);
    }

}
