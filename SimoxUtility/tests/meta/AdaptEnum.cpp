/**
* @package    VirtualRobot
* @author     Rainer Kartmann
* @copyright  2018 Rainer Kartmann
*/

#define BOOST_TEST_MODULE SimoxUtility / meta / AdaptEnum

#include <iostream>
#include <string>

#include <boost/bimap.hpp>
#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/meta/enum/adapt_enum.h>

namespace ns
{
    enum class enum_1
    {
        e0,
        e1,
        e2
    };
    enum class enum_2
    {
        e0,
        e1,
        e2
    };
} // namespace ns

namespace simox::meta
{
    template <>
    struct is_enum_adapted<ns::enum_1, void> : std::true_type
    {
        static const auto&
        names()
        {
            static const simox::meta::EnumNames<ns::enum_1> names = {
                {ns::enum_1::e0, "enum_1::e0"},
                {ns::enum_1::e1, "enum_1::e1"},
                {ns::enum_1::e2, "enum_1::e2"}};
            return names;
        }
    };

    template <>
    const simox::meta::EnumNames<ns::enum_2> enum_names<ns::enum_2>{{ns::enum_2::e0, "enum_2::e0"},
                                                                    {ns::enum_2::e1, "enum_2::e1"},
                                                                    {ns::enum_2::e2, "enum_2::e2"}};
} // namespace simox::meta

BOOST_AUTO_TEST_CASE(test_adapt_enum)
{
    {
        std::cout << ns::enum_1::e0 << '\n';
        std::cout << ns::enum_1::e1 << '\n';
        std::cout << ns::enum_1::e2 << '\n';

        BOOST_CHECK_EQUAL("enum_1::e0", boost::lexical_cast<std::string>(ns::enum_1::e0));
        BOOST_CHECK_EQUAL("enum_1::e1", boost::lexical_cast<std::string>(ns::enum_1::e1));
        BOOST_CHECK_EQUAL("enum_1::e2", boost::lexical_cast<std::string>(ns::enum_1::e2));

        BOOST_CHECK(ns::enum_1::e0 == boost::lexical_cast<ns::enum_1>("enum_1::e0"));
        BOOST_CHECK(ns::enum_1::e1 == boost::lexical_cast<ns::enum_1>("enum_1::e1"));
        BOOST_CHECK(ns::enum_1::e2 == boost::lexical_cast<ns::enum_1>("enum_1::e2"));
    }
    {
        std::cout << ns::enum_2::e0 << '\n';
        std::cout << ns::enum_2::e1 << '\n';
        std::cout << ns::enum_2::e2 << '\n';

        BOOST_CHECK_EQUAL("enum_2::e0", boost::lexical_cast<std::string>(ns::enum_2::e0));
        BOOST_CHECK_EQUAL("enum_2::e1", boost::lexical_cast<std::string>(ns::enum_2::e1));
        BOOST_CHECK_EQUAL("enum_2::e2", boost::lexical_cast<std::string>(ns::enum_2::e2));

        BOOST_CHECK(ns::enum_2::e0 == boost::lexical_cast<ns::enum_2>("enum_2::e0"));
        BOOST_CHECK(ns::enum_2::e1 == boost::lexical_cast<ns::enum_2>("enum_2::e1"));
        BOOST_CHECK(ns::enum_2::e2 == boost::lexical_cast<ns::enum_2>("enum_2::e2"));
    }
}
