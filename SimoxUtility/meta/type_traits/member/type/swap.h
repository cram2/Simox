// This file is generated using cmake configure_file!

#pragma once

#include <type_traits>

//implementation namespace
namespace simox::meta::member::type::swap::impl
{
    template<class T, class = void>
    struct member_type_exists : std::false_type
    {};
    template<class T>
    struct member_type_exists < T, std::void_t < typename T::swap > > : std::true_type
    {};

    template<class T, class = void>
    struct member_type_type
    {};

    template<class T>
    struct member_type_type < T, std::void_t < typename T::swap > >
    {
        using type = typename T::swap;
    };
}

//meta fncs namespaced by class
namespace simox::meta::member::type::swap
{
    template<class T> using exists                   = impl::member_type_exists<T>;
    template<class T> static constexpr bool exists_v = exists<T>::value;
    template<class T> using type                     = impl::member_type_type<T>;
    template<class T> using type_t                   = typename type<T>::type;
}

//exists_v
namespace simox::meta::member::type::exists_v
{
    template<class T>
    static constexpr bool swap = ::simox::meta::member::type::swap::exists_v<T>;
}
//concept
namespace simox::meta::member::type::ccept
{
    template<class T>
    concept swap = ::simox::meta::member::type::swap::exists_v<T>;
}
//type_t
namespace simox::meta::member::type::type_t
{
    template<class T>
    using swap = ::simox::meta::member::type::swap::type_t<T>;
}
