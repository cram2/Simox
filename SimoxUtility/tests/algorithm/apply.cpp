/**
* @package    SimoxUtility
* @author     Raphael Grimm
* @copyright  2019 Raphael Grimm
*/

#define BOOST_TEST_MODULE SimoxUtility / algorithm / apply

#include <iostream>
#include <random>

#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/algorithm/apply.hpp>

struct SimoxApplyTestFixture
{
    const std::function<std::string(int)> function = [](int i) { return std::to_string(i); };

    struct Functor
    {
        std::string
        operator()(int i) const
        {
            return std::to_string(i);
        }
    };

    Functor functor;
};


BOOST_FIXTURE_TEST_SUITE(SimoxApplyTest, SimoxApplyTestFixture)

BOOST_AUTO_TEST_CASE(test_apply_vector)
{
    const std::vector<int> vec{1, 2, 3};
    const std::vector<std::string> vec2 = simox::alg::apply(vec, function);
    const std::vector<std::string> vec3 = simox::alg::apply(vec, functor);

    BOOST_CHECK_EQUAL(vec.size(), vec2.size());
    BOOST_CHECK_EQUAL(vec.size(), vec3.size());
}

BOOST_AUTO_TEST_CASE(test_apply_map)
{
    const std::map<float, int> map{{1, 1}, {2, 2}, {3, 3}};
    const std::map<float, std::string> map2 = simox::alg::apply(map, function);
    const std::map<float, std::string> map3 = simox::alg::apply(map, functor);

    BOOST_CHECK_EQUAL(map.size(), map2.size());
    BOOST_CHECK_EQUAL(map.size(), map3.size());
}


BOOST_AUTO_TEST_SUITE_END()
