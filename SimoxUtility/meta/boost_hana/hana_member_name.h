#pragma once

#include <boost/hana/core/to.hpp>

#include "has_hana_accessor.h"

namespace simox::meta::detail::_hana_member_name
{
    template <class CL, class MT, MT CL::*ptr>
    struct helper
    {
        static_assert(has_hana_accessor_v<CL>, "The given type was not adapted by hana");

        using accessor = boost::hana::accessors_impl<CL>;
        using tuple = decltype(accessor::apply());
        using member_ptr = boost::hana::struct_detail::member_ptr<MT CL::*, ptr>;

        template <class... Ts>
        struct search
        {
            static_assert(sizeof...(Ts), "Member not found");
        };

        template <class Name, class Ptr, class... Ts>
        struct search<boost::hana::pair<Name, Ptr>, Ts...>
        {
            static constexpr const char*
            get()
            {
                if constexpr (std::is_same_v<Ptr, member_ptr>)
                {
                    return boost::hana::to<char const*>(Name{});
                }
                else
                {
                    return search<Ts...>::get();
                }
            }
        };

        template <class T>
        struct search_init
        {
            static_assert(!std::is_same_v<T, T>, "The given type was not adapted by hana");
        };

        template <class... Ts>
        struct search_init<boost::hana::tuple<Ts...>> : search<Ts...>
        {
        };

        static constexpr const char* name = search_init<tuple>::get();
    };
} // namespace simox::meta::detail::_hana_member_name

namespace simox::meta
{
    template <class CL, class MT, MT CL::*ptr>
    static constexpr const char* hana_member_name =
        detail::_hana_member_name::helper<CL, MT, ptr>::name;
}
