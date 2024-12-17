// This file is generated using cmake configure_file!

#pragma once

#include <type_traits>

//implementation namespace
namespace simox::meta::member::value::npos::impl
{
    template <class Type>
    Type value_type_fn(const Type*);

    template <class T, class = void>
    struct member_value_type
    {
    };

    template <class T>
    struct member_value_type<T, std::void_t<decltype(value_type_fn(&T::npos))>>
    {
        using type = decltype(value_type_fn(&T::npos));
    };

    template <class T, class = void>
    struct member_value_exists : std::false_type
    {
    };

    template <class T>
    struct member_value_exists<T, std::void_t<decltype(value_type_fn(&T::npos))>> : std::true_type
    {
    };
} // namespace simox::meta::member::value::npos::impl

//meta fncs namespaced by class
namespace simox::meta::member::value::npos
{
    template <class T>
    using exists = impl::member_value_exists<T>;
    template <class T>
    static constexpr bool exists_v = exists<T>::value;
    template <class T>
    using type = impl::member_value_type<T>;
    template <class T>
    using type_t = typename type<T>::type;
} // namespace simox::meta::member::value::npos

//exists_v
namespace simox::meta::member::value::exists_v
{
    template <class T>
    static constexpr bool npos = ::simox::meta::member::value::npos::exists_v<T>;
}

//concept
namespace simox::meta::member::value::ccept
{
    template <class T>
    concept npos = ::simox::meta::member::value::npos::exists_v<T>;
}

//type_t
namespace simox::meta::member::value::type_t
{
    template <class T>
    using npos = ::simox::meta::member::value::npos::type_t<T>;
}
