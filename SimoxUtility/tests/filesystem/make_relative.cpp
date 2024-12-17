/**
* @package    VirtualRobot
* @author     Andre Meixner
* @copyright  2020 Andre Meixner
*/

#define BOOST_TEST_MODULE SimoxUtility / filesystem / make_relative

#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/filesystem/make_relative.h>

namespace fs = simox::fs;

BOOST_AUTO_TEST_CASE(make_relative)
{
    BOOST_CHECK_EQUAL(simox::fs::make_relative("a/b/c/e/", "a/b/d/f.xml"), "../../../d/f.xml");

    std::filesystem::path path = std::filesystem::path("a/b/c/e/g.xml").parent_path();

    BOOST_CHECK_EQUAL(simox::fs::make_relative(path, "a/b/d/f.xml"), "../../d/f.xml");

    BOOST_CHECK_EQUAL(simox::fs::make_relative("a/b", "a/d/f.xml"), "../d/f.xml");
}
