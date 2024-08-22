#pragma once

#include <map>
#include <string>
#include <stdexcept>
#include <set>

#include <boost/bimap.hpp>

#include <SimoxUtility/meta/type_name.h>

namespace simox::meta
{
    /**
     * @brief Provides a bidirectional mapping of enum values to names.
     *
     * You can construct it like this:
     *
     * @code
     * enum class Colors { RED, GREEN, BLUE };
     * const simox::meta::EnumNames<Color> color_names
     * {
     *      { Colors::RED, "Red" },
     *      { Colors::GREEN, "Green" },
     *      { Colors::Blue, "Blue" },
     * };
     * @endcode
     */
    template <typename _EnumT = int, typename _NameT = std::string>
    class EnumNames
    {
    public:
        using EnumT = _EnumT;
        using NameT = _NameT;
        using bimap = boost::bimap<EnumT, NameT>;
    public:
        EnumNames(std::initializer_list<std::pair<EnumT, NameT>> init)
        {
            for (const auto& it : init)
            {
                _names.insert(typename bimap::value_type(it.first, it.second));
            }
        }
        EnumNames(std::initializer_list<std::pair<NameT, EnumT>> init)
        {
            for (const auto& it : init)
            {
                _names.insert(typename bimap::value_type(it.second, it.first));
            }
        }

        EnumNames(const std::map<EnumT, NameT>& enum_to_name_map)
        {
            for (const auto& it : enum_to_name_map)
            {
                _names.insert(typename bimap::value_type(it.first, it.second));
            }
        }
        EnumNames(const std::map<NameT, EnumT>& name_to_enum_map)
        {
            for (const auto& it : name_to_enum_map)
            {
                _names.insert(typename bimap::value_type(it.second, it.first));
            }
        }

        /**
         * @brief Map an enum value to its name.
         * @throws `error::UnknownEnumValue` If there is no mapping of `e`.
         */
        NameT to_name(const EnumT& e) const;

        /**
         * @brief Map an enum name to its value.
         * @throws `error::UnknownEnumValue` If there is no mapping of `name`.
         */
        EnumT from_name(const NameT& name) const;

        /// Get the enum values.
        std::set<EnumT> values() const
        {
            std::set<EnumT> values;
            for (const auto& it : _names.left)
            {
                values.insert(it.first);
            }
            return values;
        }

        /// Get the enum names.
        std::set<NameT> names() const
        {
            std::set<NameT> values;
            for (const auto& it : _names.right)
            {
                values.insert(it.first);
            }
            return values;
        }
        template<template<class...> class Temp = std::vector>
        Temp<NameT> names() const
        {
            Temp<NameT> values;
            for (const auto& it : _names.right)
            {
                values.emplace_back(it.first);
            }
            return values;
        }

        template <typename E, typename N>
        friend std::ostream& operator<<(std::ostream& os, const EnumNames<E, N>& rhs);

        const bimap& map() const
        {
            return _names;
        }

//        const auto& map_names() const
//        {
//            return _names.right;
//        }
//        const auto& map_values() const
//        {
//            return _names.left;
//        }
    private:
        /// The bidirectional map of values to names and back.
        bimap _names;
    };

    /// `EnumNames` for integer-based enums.
    using IntEnumNames = EnumNames<int>;

    namespace error
    {
        /// Indicates that a given value or name was unknown.
        class UnknownEnumValue : public std::out_of_range
        {
        public:

            template <typename E, typename N>
            UnknownEnumValue(E value, const EnumNames<E, N>& names) :
                std::out_of_range(make_msg(int(value), names, "value"))
            {}

            template <typename E, typename N>
            UnknownEnumValue(const N& value, const EnumNames<E, N>& names) :
                std::out_of_range(make_msg(value, names, "name"))
            {}


            template <typename Key, typename E, typename N>
            static std::string make_msg(const Key& value, const EnumNames<E, N>& names, const std::string& what)
            {
                std::stringstream ss;
                ss << "Unknown enum " << what << " '" << value << "'"
                   << " of type '" << simox::meta::get_type_name<E>() << "'."
                   << "\n" << names;
                return ss.str();
            }
        };
    }


    template<typename E, typename N>
    N EnumNames<E, N>::to_name(const E& e) const
    {
        try
        {
            return _names.left.at(e);
        }
        catch (const std::out_of_range&)
        {
            throw error::UnknownEnumValue(e, *this);
        }
    }

    template<typename E, typename N>
    E EnumNames<E, N>::from_name(const N& name) const
    {
        try
        {
            return _names.right.at(name);
        }
        catch (const std::out_of_range&)
        {
            throw error::UnknownEnumValue(name, *this);
        }
    }


    template <typename E, typename N>
    std::ostream& operator<<(std::ostream& os, const EnumNames<E, N>& rhs)
    {
        os << "Names of enum '" << simox::meta::get_type_name<E>() << "':";
        for (const auto& it : rhs._names.left)
        {
            os << "\n\t" << int(it.first) << ": '" << it.second << "'";
        }
        return os;
    }

}

