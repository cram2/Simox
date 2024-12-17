// This file is generated using cmake configure_file!

#pragma once

#include <type_traits>

//implementation namespace
namespace simox::meta::member::static_variable::end::impl
{
    template<class Type>
    Type type_fn(const Type*);
    template<class T>
    using var_t = decltype(type_fn(&T::end));
    
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
namespace simox::meta::member::static_variable::end
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
    static constexpr bool end = ::simox::meta::member::static_variable::end::exists_v<T>;
}
//concept
namespace simox::meta::member::static_variable::ccept
{
    template<class T>
    concept end = ::simox::meta::member::static_variable::end::exists_v<T>;
}
//type_t
namespace simox::meta::member::static_variable::type_t
{
    template<class T>
    using end = ::simox::meta::member::static_variable::end::type_t<T>;
}
