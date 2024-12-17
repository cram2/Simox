// This file is generated using cmake configure_file!

#pragma once

#include <type_traits>

//implementation namespace
namespace simox::meta::member::type::y::impl
{
    template <class T, class = void>
    struct member_type_exists : std::false_type
    {
    };

    template <class T>
    struct member_type_exists<T, std::void_t<typename T::y>> : std::true_type
    {
    };

    template <class T, class = void>
    struct member_type_type
    {
    };

    template <class T>
    struct member_type_type<T, std::void_t<typename T::y>>
    {
        using type = typename T::y;
    };
} // namespace simox::meta::member::type::y::impl

//meta fncs namespaced by class
namespace simox::meta::member::type::y
{
    template <class T>
    using exists = impl::member_type_exists<T>;
    template <class T>
    static constexpr bool exists_v = exists<T>::value;
    template <class T>
    using type = impl::member_type_type<T>;
    template <class T>
    using type_t = typename type<T>::type;
} // namespace simox::meta::member::type::y

//exists_v
namespace simox::meta::member::type::exists_v
{
    template <class T>
    static constexpr bool y = ::simox::meta::member::type::y::exists_v<T>;
}

//concept
namespace simox::meta::member::type::ccept
{
    template <class T>
    concept y = ::simox::meta::member::type::y::exists_v<T>;
}

//type_t
namespace simox::meta::member::type::type_t
{
    template <class T>
    using y = ::simox::meta::member::type::y::type_t<T>;
}
