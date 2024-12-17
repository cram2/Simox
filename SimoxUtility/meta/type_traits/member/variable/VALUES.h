// This file is generated using cmake configure_file!

#pragma once

#include <type_traits>

//implementation namespace
namespace simox::meta::member::variable::VALUES::impl
{
    template <class Class, class Type>
    Type variable_type_fn(Type Class::*);
    template <class T>
    using var_t = decltype(variable_type_fn(&T::VALUES));

    template <class T, class = void>
    struct member_variable_type
    {
    };

    template <class T>
    struct member_variable_type<T, std::void_t<var_t<T>>>
    {
        using type = var_t<T>;
    };

    template <class T, class = void>
    struct member_variable_exists : std::false_type
    {
    };

    template <class T>
    struct member_variable_exists<T, std::void_t<var_t<T>>> : std::true_type
    {
    };
} // namespace simox::meta::member::variable::VALUES::impl

//meta fncs namespaced by class
namespace simox::meta::member::variable::VALUES
{
    template <class T>
    using exists = impl::member_variable_exists<T>;
    template <class T>
    static constexpr bool exists_v = exists<T>::value;
    template <class T>
    using type = impl::member_variable_type<T>;
    template <class T>
    using type_t = typename type<T>::type;
} // namespace simox::meta::member::variable::VALUES

//exists_v
namespace simox::meta::member::variable::exists_v
{
    template <class T>
    static constexpr bool VALUES = ::simox::meta::member::variable::VALUES::exists_v<T>;
}

//concept
namespace simox::meta::member::variable::ccept
{
    template <class T>
    concept VALUES = ::simox::meta::member::variable::VALUES::exists_v<T>;
}

//type_t
namespace simox::meta::member::variable::type_t
{
    template <class T>
    using VALUES = simox::meta::member::variable::VALUES::type_t<T>;
}
