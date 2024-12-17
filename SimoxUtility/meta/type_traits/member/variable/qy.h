// This file is generated using cmake configure_file!

#pragma once

#include <type_traits>

//implementation namespace
namespace simox::meta::member::variable::qy::impl
{
    template <class Class, class Type>
    Type type_fn(Type Class::*);
    template <class T>
    using var_t = decltype(type_fn(&T::qy));

    template <class T, class = void>
    struct member_type
    {
    };

    template <class T>
    struct member_type<T, std::void_t<var_t<T>>>
    {
        using type = var_t<T>;
    };

    template <class T, class = void>
    struct member_exists : std::false_type
    {
    };

    template <class T>
    struct member_exists<T, std::void_t<var_t<T>>> : std::true_type
    {
    };
} // namespace simox::meta::member::variable::qy::impl

//meta fncs namespaced by class
namespace simox::meta::member::variable::qy
{
    template <class T>
    using exists = impl::member_exists<T>;
    template <class T>
    static constexpr bool exists_v = exists<T>::value;
    template <class T>
    using type = impl::member_type<T>;
    template <class T>
    using type_t = typename type<T>::type;
} // namespace simox::meta::member::variable::qy

//exists_v
namespace simox::meta::member::variable::exists_v
{
    template <class T>
    static constexpr bool qy = ::simox::meta::member::variable::qy::exists_v<T>;
}

//concept
namespace simox::meta::member::variable::ccept
{
    template <class T>
    concept qy = ::simox::meta::member::variable::qy::exists_v<T>;
}

//type_t
namespace simox::meta::member::variable::type_t
{
    template <class T>
    using qy = simox::meta::member::variable::qy::type_t<T>;
}
