/**
* @package    SimoxUtility
* @author     Raphael Grimm
* @copyright  2019 Raphael Grimm
*/

#define BOOST_TEST_MODULE SimoxUtility / math / SoftMinMax

#include <iostream>
#include <random>

#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/math/SoftMinMax.h>

BOOST_AUTO_TEST_CASE(test_soft_min_max_of_random)
{
    std::default_random_engine gen;
    std::uniform_real_distribution<float> distrib(-100, 100);

    const size_t num = 100;
    const float percentile = 0.2f;
    // 20 values may be discarded

    std::vector<float> values;
    for (size_t i = 0; i < num; ++i)
    {
        //        values.emplace_back(100 - i);
        values.emplace_back(distrib(gen));
    }

    simox::math::SoftMinMax smm(percentile, values.size());
    for (float val : values)
    {
        smm.add(val);
    }

    std::sort(values.begin(), values.end());

    BOOST_TEST_MESSAGE(num * percentile);

    BOOST_CHECK_EQUAL(smm.getSoftMin(), values.at(20)); // Values 00-19 discarded.
    BOOST_CHECK_EQUAL(smm.getSoftMax(), values.at(79)); // Values 80-99 discarded.
}

BOOST_AUTO_TEST_CASE(test_exceptions)
{
    simox::math::SoftMinMax smm;
    BOOST_CHECK_THROW(smm.getSoftMin(), std::out_of_range);
    BOOST_CHECK_THROW(smm.getSoftMax(), std::out_of_range);

    BOOST_CHECK_THROW(simox::math::SoftMinMax(0, 0), std::invalid_argument);

    BOOST_CHECK_THROW(simox::math::SoftMinMax(-1.0f, 10), std::invalid_argument);
    BOOST_CHECK_THROW(simox::math::SoftMinMax(-0.1f, 10), std::invalid_argument);
    BOOST_CHECK_THROW(simox::math::SoftMinMax(0.6f, 10), std::invalid_argument);
    BOOST_CHECK_THROW(simox::math::SoftMinMax(1.0f, 10), std::invalid_argument);
}
