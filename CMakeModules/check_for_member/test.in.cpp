// This file is generated using cmake configure_file!

#define BOOST_TEST_MODULE SimoxUtility / meta / check_for_members_ @CATEGORY @_ @NAME @

#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/meta/type_traits/member/method/@NAME@.h>
#include <SimoxUtility/meta/type_traits/member/static_method/@NAME@.h>
#include <SimoxUtility/meta/type_traits/member/static_variable/@NAME@.h>
#include <SimoxUtility/meta/type_traits/member/type/@NAME@.h>
#include <SimoxUtility/meta/type_traits/member/variable/@NAME@.h>

// ////////////////////////////////////////////////////////////////////////// //
//test stub
// *INDENT-OFF*
template <class T, class Texp, bool Exists>
struct test_method
{
    static void test();
};

template <class T, class Texp, bool Exists>
struct test_static_method
{
    static void test();
};

template <class T, class Texp, bool Exists>
struct test_variable
{
    static void test();
};

template <class T, class Texp, bool Exists>
struct test_static_variable
{
    static void test();
};

template <class T, class Texp, bool Exists>
struct test_type
{
    static void test();
};

// *INDENT-ON*
// ////////////////////////////////////////////////////////////////////////// //
//core test function
template <class T, class Texp, bool Exists>
void
test_method<T, Texp, Exists>::test()
{
    namespace ns = simox::meta::member::method;
    static_assert(Exists == ns::exists_v::@NAME @<T>);
    static_assert(Exists == ns::ccept ::@NAME @<T>);
    if constexpr (Exists)
    {
        static_assert(std::is_same_v<Texp, ns::return_type_t::@NAME @<T>>);
    }
}

template <class T, class Texp, bool Exists>
void
test_static_method<T, Texp, Exists>::test()
{
    namespace ns = simox::meta::member::static_method;
    static_assert(Exists == ns::exists_v::@NAME @<T>);
    static_assert(Exists == ns::ccept ::@NAME @<T>);
    if constexpr (Exists)
    {
        static_assert(std::is_same_v<Texp, ns::return_type_t::@NAME @<T>>);
    }
}

template <class T, class Texp, bool Exists>
void
test_variable<T, Texp, Exists>::test()
{
    namespace ns = simox::meta::member::variable;
    static_assert(Exists == ns::exists_v::@NAME @<T>);
    static_assert(Exists == ns::ccept ::@NAME @<T>);
    if constexpr (Exists)
    {
        static_assert(std::is_same_v<Texp, ns::type_t::@NAME @<T>>);
    }
}

template <class T, class Texp, bool Exists>
void
test_static_variable<T, Texp, Exists>::test()
{
    namespace ns = simox::meta::member::static_variable;
    static_assert(Exists == ns::exists_v::@NAME @<T>);
    static_assert(Exists == ns::ccept ::@NAME @<T>);
    if constexpr (Exists)
    {
        static_assert(std::is_same_v<Texp, ns::type_t::@NAME @<T>>);
    }
}

template <class T, class Texp, bool Exists>
void
test_type<T, Texp, Exists>::test()
{
    namespace ns = simox::meta::member::type;
    static_assert(Exists == ns::exists_v::@NAME @<T>);
    static_assert(Exists == ns::ccept ::@NAME @<T>);
    if constexpr (Exists)
    {
        static_assert(std::is_same_v<Texp, ns::type_t::@NAME @<T>>);
    }
}

// ////////////////////////////////////////////////////////////////////////// //
// default impl
class nonprimitive
{
};

// *INDENT-OFF*
template <class T>
struct with_variable
{
    T @NAME @ = {};
};

template <class T>
struct with_variable_2
{
    T @NAME @_2 = {};
};

template <class T>
struct with_static_variable
{
    static constexpr T @NAME @ = {};
};

template <class T>
struct with_static_variable_2
{
    static constexpr T @NAME @_2 = {};
};

template <class T>
struct with_type
{
    using @NAME @ = T;
};

template <class T>
struct with_type_2
{
    using @NAME @_2 = T;
};

template <class T>
struct with_method
{
    T @NAME @();
};

template <class T>
struct with_method_2
{
    T @NAME @_2();
};

template <class T>
struct with_static_method
{
    static T @NAME @();
};

template <class T>
struct with_static_method_2
{
    static T @NAME @_2();
};

// *INDENT-ON*

enum member_type
{
    variable,
    static_variable,
    method,
    static_method,
    type,
};

template <template <class, class, bool> class Tmp, class T, member_type MT>
void
test_all_for_type()
{
    Tmp<with_static_variable<T>, T, MT == member_type::static_variable>::test();
    Tmp<with_static_variable_2<T>, T, false>::test();

    Tmp<with_variable<T>, T, MT == member_type::variable>::test();
    Tmp<with_variable_2<T>, T, false>::test();

    Tmp<with_type<T>, T, MT == member_type::type>::test();
    Tmp<with_type_2<T>, T, false>::test();

    Tmp<with_method<T>, T, MT == member_type::method>::test();
    Tmp<with_method_2<T>, T, false>::test();

    Tmp<with_static_method<T>, T, MT == member_type::static_method>::test();
    Tmp<with_static_method_2<T>, T, false>::test();
}

template <class T>
void
test_all()
{
    test_all_for_type<test_method, T, member_type::method>();
    test_all_for_type<test_static_method, T, member_type::static_method>();
    test_all_for_type<test_variable, T, member_type::variable>();
    test_all_for_type<test_static_variable, T, member_type::static_variable>();
    test_all_for_type<test_type, T, member_type::type>();
}

BOOST_AUTO_TEST_CASE(test_check_for_members_ @NAME @)
{
    test_all<int>();
    test_all<nonprimitive>();
}
