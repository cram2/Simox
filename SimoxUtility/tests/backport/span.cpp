/**
* @package    SimoxUtility
* @author     Raphael Grimm
* @copyright  2019 Raphael Grimm
*/

#define BOOST_TEST_MODULE SimoxUtility / backport / span

#include <iostream>
#include <vector>

#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/backport/span.h>

BOOST_AUTO_TEST_CASE(test_apply_vector)
{
    const std::vector<int> vec{1, 2, 3};
    std::span span(vec);

    BOOST_CHECK_EQUAL(vec.size(), span.size());
    BOOST_CHECK_EQUAL(1, span[0]);
    BOOST_CHECK_EQUAL(2, span[1]);
    BOOST_CHECK_EQUAL(3, span[2]);
}
