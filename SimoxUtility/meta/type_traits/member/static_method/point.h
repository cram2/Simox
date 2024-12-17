// This file is generated using cmake configure_file!

#pragma once

#include <type_traits>

//implementation namespace
namespace simox::meta::member::static_method::point::impl
{
    template <class RType, class... Ps>
    RType type_fn(RType (*)(Ps...));
    template <class T>
    using ret_t = decltype(type_fn(&T::point));

    template <class T, class = void>
    struct member_rtype
    {
    };

    template <class T>
    struct member_rtype<T, std::void_t<ret_t<T>>>
    {
        using type = ret_t<T>;
    };

    template <class T, class = void>
    struct member_exists : std::false_type
    {
    };

    template <class T>
    struct member_exists<T, std::void_t<ret_t<T>>> : std::true_type
    {
    };
} // namespace simox::meta::member::static_method::point::impl

//meta fncs namespaced by class
namespace simox::meta::member::static_method::point
{
    template <class T>
    using exists = impl::member_exists<T>;
    template <class T>
    static constexpr bool exists_v = exists<T>::value;
    template <class T>
    using return_type = impl::member_rtype<T>;
    template <class T>
    using return_type_t = typename return_type<T>::type;
} // namespace simox::meta::member::static_method::point

//exists_v
namespace simox::meta::member::static_method::exists_v
{
    template <class T>
    static constexpr bool point = ::simox::meta::member::static_method::point::exists_v<T>;
}

//concept
namespace simox::meta::member::static_method::ccept
{
    template <class T>
    concept point = ::simox::meta::member::static_method::point::exists_v<T>;
}

//return_type_t
namespace simox::meta::member::static_method::return_type_t
{
    template <class T>
    using point = simox::meta::member::static_method::point::return_type_t<T>;
}
