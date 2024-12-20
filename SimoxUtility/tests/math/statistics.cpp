/**
* @package    SimoxUtility
* @author     Raphael Grimm
* @copyright  2019 Raphael Grimm
*/

#define BOOST_TEST_MODULE SimoxUtility_DeltaAngleTest

#include <iostream>
#include <random>

#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/math/statistics/measures.h>

template <typename FloatT>
void
test_measures_unsorted_temp(bool sorted)
{
    std::vector<FloatT> values{10, 8, 5, 2, -5, -8, -10};
    if (sorted)
    {
        std::sort(values.begin(), values.end());
    }

    BOOST_CHECK_CLOSE(simox::math::mean(values), 0.285714, 1e-3);
    BOOST_CHECK_CLOSE(simox::math::stddev(values), 7.973169, 1e-4);

    BOOST_CHECK_EQUAL(simox::math::min(values, sorted), -10);
    BOOST_CHECK_EQUAL(simox::math::lower_quartile(values, sorted), -6.5);
    BOOST_CHECK_EQUAL(simox::math::median(values, sorted), 2);
    BOOST_CHECK_EQUAL(simox::math::upper_quartile(values, sorted), 6.5);
    BOOST_CHECK_EQUAL(simox::math::max(values, sorted), 10);
}

BOOST_AUTO_TEST_CASE(test_measures_unsorted_float)
{
    test_measures_unsorted_temp<float>(false);
}

BOOST_AUTO_TEST_CASE(test_measures_unsorted_double)
{
    test_measures_unsorted_temp<double>(false);
}

BOOST_AUTO_TEST_CASE(test_measures_sorted_float)
{
    test_measures_unsorted_temp<float>(true);
}

BOOST_AUTO_TEST_CASE(test_measures_sorted_double)
{
    test_measures_unsorted_temp<double>(true);
}

BOOST_AUTO_TEST_CASE(test_measures_empty)
{
    std::vector<float> values;

    BOOST_CHECK_THROW(simox::math::mean(values), simox::error::SimoxError);
    BOOST_CHECK_THROW(simox::math::stddev(values), simox::error::SimoxError);
    BOOST_CHECK_THROW(simox::math::lower_quartile(values), simox::error::SimoxError);
    BOOST_CHECK_THROW(simox::math::median(values), simox::error::SimoxError);
    BOOST_CHECK_THROW(simox::math::upper_quartile(values), simox::error::SimoxError);
    BOOST_CHECK_THROW(simox::math::max(values), simox::error::SimoxError);
    BOOST_CHECK_THROW(simox::math::mean(values), simox::error::SimoxError);
}
