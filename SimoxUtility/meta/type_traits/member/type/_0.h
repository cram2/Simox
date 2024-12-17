// This file is generated using cmake configure_file!

#pragma once

#include <type_traits>

//implementation namespace
namespace simox::meta::member::type::_0::impl
{
    template<class T, class = void>
    struct member_type_exists : std::false_type
    {};
    template<class T>
    struct member_type_exists < T, std::void_t < typename T::_0 > > : std::true_type
    {};

    template<class T, class = void>
    struct member_type_type
    {};

    template<class T>
    struct member_type_type < T, std::void_t < typename T::_0 > >
    {
        using type = typename T::_0;
    };
}

//meta fncs namespaced by class
namespace simox::meta::member::type::_0
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
    static constexpr bool _0 = ::simox::meta::member::type::_0::exists_v<T>;
}
//concept
namespace simox::meta::member::type::ccept
{
    template<class T>
    concept _0 = ::simox::meta::member::type::_0::exists_v<T>;
}
//type_t
namespace simox::meta::member::type::type_t
{
    template<class T>
    using _0 = ::simox::meta::member::type::_0::type_t<T>;
}
