/**
* @package    VirtualRobot
* @author     Rainer Kartmann
* @copyright  2018 Rainer Kartmann
*/

#define BOOST_TEST_MODULE SimoxUtility / meta / EnumNames

#include <filesystem>

#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/meta/EnumNames.hpp>

namespace EnumNamesTest
{
    enum class TestEnum
    {
        RED,
        GREEN,
        BLUE,
    };

    std::ostream&
    operator<<(std::ostream& os, TestEnum e)
    {
        return os << int(e);
    }

    void
    test(const simox::meta::EnumNames<TestEnum>& names)
    {
        BOOST_CHECK_EQUAL(names.to_name(TestEnum::RED), "Red");
        BOOST_CHECK_EQUAL(names.to_name(TestEnum::GREEN), "Green");
        BOOST_CHECK_EQUAL(names.to_name(TestEnum::BLUE), "Blue");

        BOOST_CHECK_EQUAL(names.from_name("Red"), TestEnum::RED);
        BOOST_CHECK_EQUAL(names.from_name("Green"), TestEnum::GREEN);
        BOOST_CHECK_EQUAL(names.from_name("Blue"), TestEnum::BLUE);
    }
} // namespace EnumNamesTest


BOOST_AUTO_TEST_SUITE(EnumNamesTest)

BOOST_AUTO_TEST_CASE(construct_from_value_name_initializer_list)
{
    const simox::meta::EnumNames<TestEnum> names{
        {TestEnum::RED, "Red"},
        {TestEnum::GREEN, "Green"},
        {TestEnum::BLUE, "Blue"},
    };
    test(names);
}

BOOST_AUTO_TEST_CASE(construct_from_name_value_initializer_list)
{
    const simox::meta::EnumNames<TestEnum> names{
        {"Red", TestEnum::RED},
        {"Green", TestEnum::GREEN},
        {"Blue", TestEnum::BLUE},
    };
    test(names);
}

BOOST_AUTO_TEST_CASE(construct_from_value_name_map)
{
    const std::map<TestEnum, std::string> valueNameMap{
        {TestEnum::RED, "Red"},
        {TestEnum::GREEN, "Green"},
        {TestEnum::BLUE, "Blue"},
    };
    const simox::meta::EnumNames<TestEnum> names(valueNameMap);
    test(names);
}

BOOST_AUTO_TEST_CASE(construct_from_name_value_map)
{
    const std::map<std::string, TestEnum> nameValueMap{
        {"Red", TestEnum::RED},
        {"Green", TestEnum::GREEN},
        {"Blue", TestEnum::BLUE},
    };
    const simox::meta::EnumNames<TestEnum> names(nameValueMap);
    test(names);
}

BOOST_AUTO_TEST_CASE(unknown_value)
{
    const simox::meta::EnumNames<EnumNamesTest::TestEnum> names{
        {TestEnum::RED, "Red"}, {TestEnum::GREEN, "Green"},
        // omit blue
    };

    BOOST_CHECK_THROW(names.to_name(TestEnum::BLUE), std::out_of_range);
    BOOST_CHECK_THROW(names.from_name("Blue"), std::out_of_range);

    BOOST_CHECK_THROW(names.to_name(static_cast<TestEnum>(42)),
                      simox::meta::error::UnknownEnumValue);
    BOOST_CHECK_THROW(names.from_name("Blue42"), simox::meta::error::UnknownEnumValue);

    try
    {
        names.to_name(TestEnum::BLUE);
    }
    catch (const simox::meta::error::UnknownEnumValue& e)
    {
        BOOST_TEST_MESSAGE("Message for unknown value: \n" << e.what());
    }

    try
    {
        names.from_name("Blue");
    }
    catch (const simox::meta::error::UnknownEnumValue& e)
    {
        BOOST_TEST_MESSAGE("Message for unknown name: \n" << e.what());
    }
}


BOOST_AUTO_TEST_SUITE_END()
