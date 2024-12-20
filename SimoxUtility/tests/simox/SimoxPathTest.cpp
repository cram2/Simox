/**
* @package    VirtualRobot
* @author     Rainer Kartmann
* @copyright  2019 Rainer Kartmann
*/

#define BOOST_TEST_MODULE SimoxUtility / simox / SimoxPathTest

#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/simox/SimoxPath.h>


namespace fs = std::filesystem;

// ToDo: Test whether paths have correct structure, but do not test existence
// (this breaks CI).
/*
BOOST_AUTO_TEST_CASE(test_path_existences)
{
    // BOOST_CHECK(fs::is_directory(simox::SimoxPath::getSimoxDir()));

    BOOST_CHECK(fs::is_directory(simox::SimoxPath::getSimoxUtilityDir()));

    BOOST_CHECK(fs::is_directory(simox::SimoxPath::getVirtualRobotDir()));
    // BOOST_CHECK(fs::is_directory(simox::SimoxPath::getVirtualRobotDataDir()));

    BOOST_CHECK(fs::is_directory(simox::SimoxPath::getSimDynamicsDir()));

    BOOST_CHECK(fs::is_directory(simox::SimoxPath::getGraspPlanningDir()));

    BOOST_CHECK(fs::is_directory(simox::SimoxPath::getMotionPlanningDir()));
}
*/


/*
BOOST_AUTO_TEST_CASE(test_test_file_path)
{
    BOOST_TEST_CONTEXT("This test file: '" << __FILE__ << "'")
    {
        BOOST_CHECK_EQUAL(simox::SimoxPath::getSimoxUtilityDir() / "tests/simox/SimoxPathTest.cpp",
                          __FILE__);
    }
}
*/
