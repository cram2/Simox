/**
* @package    SimoxUtility
* @author     Fabian Peller
* @copyright  2021 Fabian Peller
*/

#define BOOST_TEST_MODULE SimoxUtility / algorithm / vector

#include <random>

#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/algorithm/vector.hpp>

struct SimoxVectorTestFixture
{
    const std::vector<int> vec{1, 2, 3};
    const std::vector<int> vec2{4, 5, 6};
};


BOOST_FIXTURE_TEST_SUITE(SimoxVectorTest, SimoxVectorTestFixture)

BOOST_AUTO_TEST_CASE(test_appended)
{
    const std::vector<int> ref{1, 2, 3, 4, 5, 6};
    auto res = simox::alg::appended(vec, vec2);

    BOOST_CHECK_EQUAL(res.size(), ref.size());
    for (unsigned int i = 0; i < res.size(); ++i)
    {
        BOOST_CHECK_EQUAL(res[i], ref[i]);
    }
}

BOOST_AUTO_TEST_CASE(test_subvec)
{
    const std::vector<int> ref{3};
    auto res = simox::alg::subvector(vec, 2);

    BOOST_CHECK_EQUAL(res.size(), ref.size());
    for (unsigned int i = 0; i < res.size(); ++i)
    {
        BOOST_CHECK_EQUAL(res[i], ref[i]);
    }
}


BOOST_AUTO_TEST_SUITE_END()
