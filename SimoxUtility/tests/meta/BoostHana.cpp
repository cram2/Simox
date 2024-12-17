/**
* @package    VirtualRobot
* @author     Rainer Kartmann
* @copyright  2018 Rainer Kartmann
*/

#define BOOST_TEST_MODULE SimoxUtility / meta / BoostHana

#include <iostream>
#include <string>

#include <boost/hana/adapt_struct.hpp>
#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/meta/boost_hana.h>

namespace hana = boost::hana;

namespace ns
{
    struct Person
    {
        std::string name;
        int age;
    };
} // namespace ns

BOOST_HANA_ADAPT_STRUCT(ns::Person, name, age);

BOOST_AUTO_TEST_CASE(test_hana_member_name)
{
    const auto name = simox::meta::hana_member_name<ns::Person, std::string, &ns::Person::name>;
    const auto age = simox::meta::hana_member_name<ns::Person, int, &ns::Person::age>;
    BOOST_CHECK_EQUAL("name", name);
    BOOST_CHECK_EQUAL("age", age);
    std::cout << "name -> " << name << '\n';
    std::cout << "age  -> " << age << '\n';
}
