/**
* @package    SimoxUtility
* @author     Rainer Kartmann
* @copyright  2021 Rainer Kartmann
*/

#define BOOST_TEST_MODULE SimoxUtility / algorithm / get_map_keys_values

#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/algorithm/get_map_keys_values.h>

namespace get_map_keys_values_test
{

    struct Fixture
    {

        std::map<int, std::string> map{
            {1, "one"},
            {2, "two"},
            {3, "three"},
        };

        const std::map<int, std::string>&
        const_map()
        {
            return map;
        }
    };

    template <class R, class V, class K>
    void
    check_equal(const std::vector<R>& values, const std::map<K, V>& map)
    {
        BOOST_CHECK_EQUAL(map.size(), values.size());

        auto itv = values.begin();
        for (auto itm = map.begin(); itm != map.end(); ++itm, ++itv)
        {
            if constexpr (std::is_same_v<R, V*> || std::is_same_v<R, const V*>)
            {
                BOOST_CHECK_EQUAL(&itm->second, *itv);
            }
            else
            {
                static_assert(std::is_same_v<R, V>, "Expecting types to be equal.");
                BOOST_CHECK_EQUAL(itm->second, *itv);
            }
        }
    }

} // namespace get_map_keys_values_test


BOOST_FIXTURE_TEST_SUITE(get_map_keys_values_test, get_map_keys_values_test::Fixture)

BOOST_AUTO_TEST_CASE(test_get_values)
{
    std::vector<std::string> values = simox::alg::get_values(map);
    check_equal(values, map);
}

BOOST_AUTO_TEST_CASE(test_get_values_unary_fn)
{
    std::vector<std::stringstream> values;
    // const
    values = simox::alg::get_values(const_map(),
                                    [](const std::string& value)
                                    {
                                        std::stringstream ss;
                                        ss << value;
                                        return ss;
                                    });
    // non-const
    values = simox::alg::get_values(map,
                                    [](std::string& value)
                                    {
                                        std::stringstream ss;
                                        ss << value;
                                        return ss;
                                    });
}

BOOST_AUTO_TEST_CASE(test_get_value_ptrs_const)
{
    std::vector<const std::string*> values = simox::alg::get_value_ptrs(const_map());
    check_equal(values, const_map());
}

BOOST_AUTO_TEST_CASE(test_get_value_ptrs_non_const)
{
    std::vector<std::string*> values = simox::alg::get_value_ptrs(map);
    check_equal(values, map);
}

BOOST_AUTO_TEST_CASE(test_get_value_cptrs_non_const)
{
    std::vector<const std::string*> values = simox::alg::get_value_cptrs(map);
    check_equal(values, map);
}


BOOST_AUTO_TEST_SUITE_END()
