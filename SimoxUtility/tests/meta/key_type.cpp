/**
* @package    VirtualRobot
* @author     Rainer Kartmann
* @copyright  2018 Rainer Kartmann
*/

#define BOOST_TEST_MODULE SimoxUtility / meta / key_type

#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/meta/key_type.h>

BOOST_AUTO_TEST_CASE(test_key_type)
{
    using namespace simox::meta;
    static_assert(std::is_same_v<key_type_t<std::map<float, float>>, float>);
    static_assert(std::is_same_v<key_type_t<std::map<int, float>>, int>);
    static_assert(std::is_same_v<key_type_t<std::unordered_map<float, float>>, float>);
    static_assert(std::is_same_v<key_type_t<std::unordered_map<int, float>>, int>);
    static_assert(std::is_same_v<key_type_t<std::set<float, float>>, float>);
    static_assert(std::is_same_v<key_type_t<std::set<int, float>>, int>);
    static_assert(std::is_same_v<key_type_t<std::unordered_set<float, float>>, float>);
    static_assert(std::is_same_v<key_type_t<std::unordered_set<int, float>>, int>);
}
