/*
 * @package    SimoxUtility
 * @author     Rainer Kartmann
 * @copyright  2023 Rainer Kartmann
 */

#define BOOST_TEST_MODULE SimoxUtility / algorithm / fuzzy_match

#include <iostream>

#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/algorithm/string/fuzzy_match.h>
#include <SimoxUtility/algorithm/string/fuzzy_multi_match.hpp>

BOOST_AUTO_TEST_CASE(fuzzy_match_strings)
{
    BOOST_CHECK_EQUAL(simox::alg::fuzzy_match(std::string("hello"), std::string("hello")), 1.0);
    BOOST_CHECK_EQUAL(simox::alg::fuzzy_match(std::string(""), std::string("hello")), 0.0);
    BOOST_CHECK_EQUAL(simox::alg::fuzzy_match(std::string("abcdef"), std::string("defghi")), 0.5);
}

BOOST_AUTO_TEST_CASE(fuzzy_multi_match)
{
    std::map<int, std::vector<std::string>> option_to_word;
    option_to_word[0] = {"zero", "0", "naught"};
    option_to_word[1] = {"one", "1"};
    option_to_word[2] = {"two", "2"};

    BOOST_CHECK_EQUAL(*simox::alg::fuzzy_multi_match<int>(std::string("zeroes"), option_to_word),
                      0);
    BOOST_CHECK_EQUAL(*simox::alg::fuzzy_multi_match<int>(std::string("ones"), option_to_word), 1);
    BOOST_CHECK_EQUAL(*simox::alg::fuzzy_multi_match<int>(std::string("twos"), option_to_word), 2);
}
