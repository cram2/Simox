// This file is generated using cmake configure_file!

#pragma once

#include <type_traits>

//implementation namespace
namespace simox::meta::member::static_variable::front::impl
{
    template<class Type>
    Type type_fn(const Type*);
    template<class T>
    using var_t = decltype(type_fn(&T::front));
    
    template<class T, class = void>
    struct member_value_type {};
    template<class T>
    struct member_value_type<T, std::void_t<var_t<T>>>
    {
        using type = var_t<T>;
    };

    template<class T, class = void>
    struct member_value_exists : std::false_type {};
    template<class T>
    struct member_value_exists<T, std::void_t<var_t<T>>> : std::true_type {};
}

//meta fncs namespaced by class
namespace simox::meta::member::static_variable::front
{
    template<class T> using exists                   = impl::member_value_exists<T>;
    template<class T> static constexpr bool exists_v = exists<T>::value;
    template<class T> using type                     = impl::member_value_type<T>;
    template<class T> using type_t                   = typename type<T>::type;
}

//exists_v
namespace simox::meta::member::static_variable::exists_v
{
    template<class T>
    static constexpr bool front = ::simox::meta::member::static_variable::front::exists_v<T>;
}
//concept
namespace simox::meta::member::static_variable::ccept
{
    template<class T>
    concept front = ::simox::meta::member::static_variable::front::exists_v<T>;
}
//type_t
namespace simox::meta::member::static_variable::type_t
{
    template<class T>
    using front = ::simox::meta::member::static_variable::front::type_t<T>;
}
