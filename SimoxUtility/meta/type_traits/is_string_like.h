#pragma once

#include <string>
#include <string_view>
#include <type_traits>

#include "is_any_of.h"

namespace simox::meta
{
    template <class T, class = void>
    struct is_string_like : std::false_type
    {
    };

    template <class T>
    struct is_string_like<
        T,
        std::enable_if_t<is_any_of_v<T, std::string, std::string_view, char*, const char*>>> :
        std::true_type
    {
        static constexpr std::string_view
        string_view(const T& t)
        {
            return {t};
        }
    };

    template <std::size_t N>
    struct is_string_like<char[N], void> : std::true_type
    {
        static constexpr std::string_view
        string_view(char (&t)[N])
        {
            return {t, N};
        }
    };

    template <std::size_t N>
    struct is_string_like<const char[N], void> : std::true_type
    {
        static constexpr std::string_view
        string_view(const char (&t)[N])
        {
            return {t, N};
        }
    };

    template <class T>
    static constexpr bool is_string_like_v = is_string_like<T>::value;

    template <class T, class R = void>
    using enable_if_is_string_like = std::enable_if<is_string_like_v<T>, R>;

    template <class T, class R = void>
    using enable_if_is_string_like_t = typename enable_if_is_string_like<T, R>::type;
} // namespace simox::meta

namespace simox::meta::detail::test::_is_string_like
{
    template <class T>
    constexpr bool
    test(const T&)
    {
        return is_string_like_v<T>;
    }

    static_assert(test("qwer"));
    static_assert(is_string_like_v<std::string>);
    static_assert(is_string_like_v<std::string_view>);
    static_assert(!is_string_like_v<int>);
} // namespace simox::meta::detail::test::_is_string_like
