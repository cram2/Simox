/**
* @package    SimoxUtility
* @author     Raphael Grimm
* @copyright  2019 Raphael Grimm
*/

#define BOOST_TEST_MODULE SimoxUtility / algorithm / for_each_if

#include <iostream>
#include <random>

#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/algorithm/for_each_if.h>

BOOST_AUTO_TEST_CASE(test_for_each_if)
{
    std::mt19937 gen{std::random_device{}()};
    std::vector<std::size_t> num;
    std::size_t even = 0;
    for (int i = 0; i < 1000; ++i)
    {
        num.emplace_back(gen());
        if (0 == (num.back() % 2))
        {
            ++even;
        }
    }
    {
        std::size_t cntEven = 0;
        std::size_t cntOdd = 0;

        simox::for_each_if(
            num,
            [](auto n) { return n % 2; },
            [&cntOdd](auto) { ++cntOdd; },
            [&cntEven](auto) { ++cntEven; });
        BOOST_CHECK_EQUAL(cntEven, even);
        BOOST_CHECK_EQUAL(cntEven + cntOdd, num.size());
    }
    {
        std::size_t cntEven = 0;
        std::size_t cntOdd = 0;

        simox::for_each_if(
            num.begin(),
            num.end(),
            [](auto n) { return n % 2; },
            [&cntOdd](auto) { ++cntOdd; },
            [&cntEven](auto) { ++cntEven; });
        BOOST_CHECK_EQUAL(cntEven, even);
        BOOST_CHECK_EQUAL(cntEven + cntOdd, num.size());
    }
}
