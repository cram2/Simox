#pragma once

#include <boost/hana.hpp>

namespace simox::meta
{
    namespace hana = boost::hana;

    template <class T>
    static constexpr bool has_hana_accessor_v =
        !std::is_base_of_v<hana::default_, hana::accessors_impl<T>>;

    template <class T, class T2>
    using enable_if_has_hana_accessor_t = std::enable_if_t<has_hana_accessor_v<T>, T2>;

    template <class T, class T2>
    using enable_if_no_hana_accessor_t = std::enable_if_t<!has_hana_accessor_v<T>, T2>;
} // namespace simox::meta
