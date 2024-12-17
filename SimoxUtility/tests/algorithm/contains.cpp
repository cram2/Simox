/**
* @package    SimoxUtility
* @author     Rainer Kartmann
* @copyright  2021 Rainer Kartmann
*/

#define BOOST_TEST_MODULE SimoxUtility / algorithm / contains

#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/algorithm/contains.h>
#include <SimoxUtility/algorithm/string/string_tools.h>

namespace contains_test
{
    struct Fixture
    {
        const std::vector<std::string> string_vec{"one", "two", "three"};
        const std::vector<int> int_vec{0, -1, 10};

        const std::set<std::string> string_set{string_vec.begin(), string_vec.end()};
        const std::set<int> int_set{int_vec.begin(), int_vec.end()};

        const std::string string = "my/string";


        std::map<std::string, int> m_si;
        std::map<std::string, std::string> m_ss;
        std::map<int, std::string> m_is;
        std::map<int, int> m_ii;

        std::unordered_map<std::string, int> um_si;
        std::unordered_map<std::string, std::string> um_ss;
        std::unordered_map<int, std::string> um_is;
        std::unordered_map<int, int> um_ii;

        Fixture()
        {
            fillMap(m_si);
            fillMap(m_ss);
            fillMap(m_is);
            fillMap(m_ii);
            fillMap(um_si);
            fillMap(um_ss);
            fillMap(um_is);
            fillMap(um_ii);
        }

        ~Fixture()
        {
        }

        template <class K, class V, template <class...> class MapT = std::map, class... Ts>
        static MapT<K, V, Ts...>
        makeMap()
        {
            MapT<K, V, Ts...> map;
            fillMap(map);
            return map;
        }

        template <class K, class V, template <class...> class MapT = std::map, class... Ts>
        static void
        fillMap(MapT<K, V, Ts...>& map)
        {
            for (int i = 1; i <= 3; ++i)
            {
                map[as<K>(i)] = as<V>(i);
            }
        }

        template <class T>
        static T
        as(int i)
        {
            if constexpr (std::is_same_v<T, int>)
            {
                return i;
            }
            else if constexpr (std::is_same_v<T, std::string>)
            {
                return std::to_string(i);
            }
        }
    };
} // namespace contains_test


BOOST_FIXTURE_TEST_SUITE(contains_test, contains_test::Fixture)

BOOST_AUTO_TEST_CASE(test_contains)
{
    BOOST_CHECK_EQUAL(simox::alg::contains(string_vec, "two"), true);
    BOOST_CHECK_EQUAL(simox::alg::contains(string_vec, "something"), false);

    BOOST_CHECK_EQUAL(simox::alg::contains(int_vec, -1), true);
    BOOST_CHECK_EQUAL(simox::alg::contains(int_vec, 1000), false);

    BOOST_CHECK_EQUAL(simox::alg::contains(int_set, -1), true);
    BOOST_CHECK_EQUAL(simox::alg::contains(int_set, 1000), false);

    BOOST_CHECK_EQUAL(simox::alg::contains(string_set, "two"), true);
    BOOST_CHECK_EQUAL(simox::alg::contains(string_set, "something"), false);
}

BOOST_AUTO_TEST_CASE(test_string_contains)
{
    // Uses string_tools.h
    // char
    BOOST_CHECK_EQUAL(simox::alg::contains(string, '/'), true);
    BOOST_CHECK_EQUAL(simox::alg::contains(string, ':'), false);

    // single-character string
    BOOST_CHECK_EQUAL(simox::alg::contains(string, "/"), true);
    BOOST_CHECK_EQUAL(simox::alg::contains(string, ":"), false);

    // multi-character string
    BOOST_CHECK_EQUAL(simox::alg::contains(string, "my"), true);
    BOOST_CHECK_EQUAL(simox::alg::contains(string, "your"), false);
}

BOOST_AUTO_TEST_CASE(test_map_contains)
{
    BOOST_CHECK_EQUAL(simox::alg::contains_key(m_si, "2"), true);
    BOOST_CHECK_EQUAL(simox::alg::contains_key(m_ss, "2"), true);
    BOOST_CHECK_EQUAL(simox::alg::contains_key(um_si, "2"), true);
    BOOST_CHECK_EQUAL(simox::alg::contains_key(um_ss, "2"), true);

    BOOST_CHECK_EQUAL(simox::alg::contains_key(m_si, "something"), false);
    BOOST_CHECK_EQUAL(simox::alg::contains_key(m_ss, "something"), false);
    BOOST_CHECK_EQUAL(simox::alg::contains_key(um_si, "something"), false);
    BOOST_CHECK_EQUAL(simox::alg::contains_key(um_ss, "something"), false);

    BOOST_CHECK_EQUAL(simox::alg::contains_key(m_is, 2), true);
    BOOST_CHECK_EQUAL(simox::alg::contains_key(m_ii, 2), true);
    BOOST_CHECK_EQUAL(simox::alg::contains_key(um_is, 2), true);
    BOOST_CHECK_EQUAL(simox::alg::contains_key(um_ii, 2), true);

    BOOST_CHECK_EQUAL(simox::alg::contains_key(m_is, -1), false);
    BOOST_CHECK_EQUAL(simox::alg::contains_key(m_ii, -1), false);
    BOOST_CHECK_EQUAL(simox::alg::contains_key(um_is, -1), false);
    BOOST_CHECK_EQUAL(simox::alg::contains_key(um_ii, -1), false);


    BOOST_CHECK_EQUAL(simox::alg::contains_value(m_is, "2"), true);
    BOOST_CHECK_EQUAL(simox::alg::contains_value(m_ss, "2"), true);
    BOOST_CHECK_EQUAL(simox::alg::contains_value(um_is, "2"), true);
    BOOST_CHECK_EQUAL(simox::alg::contains_value(um_ss, "2"), true);

    BOOST_CHECK_EQUAL(simox::alg::contains_value(m_is, "something"), false);
    BOOST_CHECK_EQUAL(simox::alg::contains_value(m_ss, "something"), false);
    BOOST_CHECK_EQUAL(simox::alg::contains_value(um_is, "something"), false);
    BOOST_CHECK_EQUAL(simox::alg::contains_value(um_ss, "something"), false);

    BOOST_CHECK_EQUAL(simox::alg::contains_value(m_si, 2), true);
    BOOST_CHECK_EQUAL(simox::alg::contains_value(m_ii, 2), true);
    BOOST_CHECK_EQUAL(simox::alg::contains_value(um_si, 2), true);
    BOOST_CHECK_EQUAL(simox::alg::contains_value(um_ii, 2), true);

    BOOST_CHECK_EQUAL(simox::alg::contains_value(m_si, -1), false);
    BOOST_CHECK_EQUAL(simox::alg::contains_value(m_ii, -1), false);
    BOOST_CHECK_EQUAL(simox::alg::contains_value(um_si, -1), false);
    BOOST_CHECK_EQUAL(simox::alg::contains_value(um_ii, -1), false);
}


BOOST_AUTO_TEST_SUITE_END()
