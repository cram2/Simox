#pragma once

#include <array>

#include <boost/lexical_cast.hpp>

#include <SimoxUtility/meta/enum/EnumNames.hpp>
#include <SimoxUtility/meta/type_name.h>
#include <SimoxUtility/meta/type_traits/is_string_like.h>
#include <SimoxUtility/meta/undefined_t.h>

namespace simox::meta
{
    template <class T, class = void>
    struct is_enum_adapted : std::false_type
    {
    };

    template <class T>
    static constexpr bool is_enum_adapted_v = is_enum_adapted<T>::value;

    template <class T, class R = void>
    using enable_if_is_enum_adapted = std::enable_if<is_enum_adapted_v<T>, R>;

    template <class T, class R = void>
    using enable_if_enum_adapted_t = typename enable_if_is_enum_adapted<T, R>::type;

    template <class T>
    constexpr undefined_t enum_names;

    template <class T>
    struct is_enum_adapted<
        T,
        std::enable_if_t<has_type_of<simox::meta::EnumNames<T>>(enum_names<T>)>> : std::true_type
    {
        static_assert(std::is_enum_v<T>);

        static const auto&
        names()
        {
            return enum_names<T>;
        }
    };
} // namespace simox::meta

namespace std
{
    template <class T>
    simox::meta::enable_if_enum_adapted_t<T, std::string>
    to_string(T f)
    {
        std::string str;
        const auto& map = simox::meta::is_enum_adapted<T>::names().map();
        if (map.left.count(f))
        {
            str = map.left.at(f);
        }
        else
        {
            using base = std::underlying_type_t<T>;
            str = simox::meta::get_type_name<T>() + "::" + std::to_string(static_cast<base>(f));
        }
        return str;
    }
} // namespace std

template <class T>
simox::meta::enable_if_enum_adapted_t<T, std::ostream&>
operator<<(std::ostream& o, const T& f)
{
    return o << std::to_string(f);
}

namespace boost::detail
{
    template <class T>
    struct lexical_converter_impl<simox::meta::enable_if_enum_adapted_t<T, std::string>, T>
    {
        static_assert(std::is_enum_v<T>);

        static inline bool
        try_convert(T arg, std::string& result)
        {
            const auto& map = simox::meta::is_enum_adapted<T>::names().map();
            if (map.left.count(arg))
            {
                result = map.left.at(arg);
                return true;
            }
            using base = std::underlying_type_t<T>;
            result =
                simox::meta::get_type_name<T>() + "::" + std::to_string(static_cast<base>(arg));
            return false;
        }
    };
} // namespace boost::detail

namespace boost::detail::simoxdetail
{
    template <class T, class Src>
    struct lex_conv_help
    {
        static inline bool
        try_convert(const Src& arg, T& result)
        {
            const auto sv = std::string{simox::meta::is_string_like<Src>::string_view(arg)};
            const auto& map = simox::meta::is_enum_adapted<T>::names().map();
            if (map.right.count(sv))
            {
                result = map.right.at(sv);
                return true;
            }
            return false;
        }
    };
} // namespace boost::detail::simoxdetail

namespace boost::detail
{
#define make_specialization(StrT)                                                                  \
    template <class T>                                                                             \
    struct lexical_converter_impl<T, simox::meta::enable_if_enum_adapted_t<T, StrT>> :             \
        simoxdetail::lex_conv_help<T, StrT>                                                        \
    {                                                                                              \
    }

    make_specialization(std::string);
    make_specialization(std::string_view);
    make_specialization(char*);
    make_specialization(const char*);
    //can't do this via templates since the boost template does not support propper SFINAE
    make_specialization(char[1]);
    make_specialization(char[2]);
    make_specialization(char[3]);
    make_specialization(char[4]);
    make_specialization(char[5]);
    make_specialization(char[6]);
    make_specialization(char[7]);
    make_specialization(char[8]);
    make_specialization(char[9]);

    make_specialization(char[10]);
    make_specialization(char[11]);
    make_specialization(char[12]);
    make_specialization(char[13]);
    make_specialization(char[14]);
    make_specialization(char[15]);
    make_specialization(char[16]);
    make_specialization(char[17]);
    make_specialization(char[18]);
    make_specialization(char[19]);

    make_specialization(char[20]);
    make_specialization(char[21]);
    make_specialization(char[22]);
    make_specialization(char[23]);
    make_specialization(char[24]);
    make_specialization(char[25]);
    make_specialization(char[26]);
    make_specialization(char[27]);
    make_specialization(char[28]);
    make_specialization(char[29]);

    make_specialization(char[30]);
    make_specialization(char[31]);
    make_specialization(char[32]);
    make_specialization(char[33]);
    make_specialization(char[34]);
    make_specialization(char[35]);
    make_specialization(char[36]);
    make_specialization(char[37]);
    make_specialization(char[38]);
    make_specialization(char[39]);

    make_specialization(char[40]);
    make_specialization(char[41]);
    make_specialization(char[42]);
    make_specialization(char[43]);
    make_specialization(char[44]);
    make_specialization(char[45]);
    make_specialization(char[46]);
    make_specialization(char[47]);
    make_specialization(char[48]);
    make_specialization(char[49]);

    make_specialization(char[50]);
    make_specialization(char[51]);
    make_specialization(char[52]);
    make_specialization(char[53]);
    make_specialization(char[54]);
    make_specialization(char[55]);
    make_specialization(char[56]);
    make_specialization(char[57]);
    make_specialization(char[58]);
    make_specialization(char[59]);

    make_specialization(char[60]);
    make_specialization(char[61]);
    make_specialization(char[62]);
    make_specialization(char[63]);
    make_specialization(char[64]);
#undef make_specialization

    //    template<class T>
    //    struct lexical_converter_impl <
    //        T,
    //        simox::meta::enable_if_enum_adapted_t<T, std::string_view> >
    //        : simoxdetail::lex_conv_help<T, std::string_view>
    //    {};

    //    template<class T>
    //    struct lexical_converter_impl <
    //        T,
    //        simox::meta::enable_if_enum_adapted_t<T, const char*> >
    //        : simoxdetail::lex_conv_help<T, const char*>
    //    {};

    //    template<class T>
    //    struct lexical_converter_impl <
    //        T,
    //        simox::meta::enable_if_enum_adapted_t<T, char*> >
    //        : simoxdetail::lex_conv_help<T, char*>
    //    {};
} // namespace boost::detail
