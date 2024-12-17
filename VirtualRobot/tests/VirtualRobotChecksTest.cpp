/**
* @package    VirtualRobot
* @author     Manfred Kroehnert
* @copyright  2010 Manfred Kroehnert
*/

#define BOOST_TEST_MODULE VirtualRobot_VirtualRobotChecksTests

#include <VirtualRobot/VirtualRobotChecks.h>
#include <VirtualRobot/VirtualRobotTest.h>


using namespace VirtualRobot;

BOOST_AUTO_TEST_CASE(test_VR_CHECK_true)
{
    bool tru = true;

    BOOST_CHECK_NO_THROW(VR_CHECK(true));
    BOOST_CHECK_NO_THROW(VR_CHECK(tru));

    BOOST_CHECK_NO_THROW(VR_CHECK_HINT(true, "Must be true."));
    BOOST_CHECK_NO_THROW(VR_CHECK_HINT(tru, "Must be true."));
}

BOOST_AUTO_TEST_CASE(test_VR_CHECK_false)
{
    bool fals = false;

    BOOST_CHECK_THROW(VR_CHECK(false), VirtualRobotCheckException);
    BOOST_CHECK_THROW(VR_CHECK(fals), VirtualRobotCheckException);

    BOOST_CHECK_THROW(VR_CHECK_HINT(false, "Must throw."), VirtualRobotCheckException);
    BOOST_CHECK_THROW(VR_CHECK_HINT(fals, "Must throw."), VirtualRobotCheckException);
}

struct Fixture
{
    int a = -1, a2 = a, b = 5, c = 10, z = 0;
};


BOOST_FIXTURE_TEST_SUITE(Comparison, Fixture)

BOOST_AUTO_TEST_CASE(test_VR_CHECK_COMPARISON)
{
    BOOST_CHECK_NO_THROW(VR_CHECK_COMPARISON(4, 4, ==));
    BOOST_CHECK_THROW(VR_CHECK_COMPARISON(2, 4, ==), VirtualRobotCheckException);

    BOOST_CHECK_NO_THROW(VR_CHECK_COMPARISON(2, 4, !=));
    BOOST_CHECK_THROW(VR_CHECK_COMPARISON(4, 4, !=), VirtualRobotCheckException);
}

BOOST_AUTO_TEST_CASE(test_VR_CHECK_EQUAL)
{
    BOOST_CHECK_NO_THROW(VR_CHECK_EQUAL(4, 4));
    BOOST_CHECK_NO_THROW(VR_CHECK_EQUAL(z, 0));
    BOOST_CHECK_NO_THROW(VR_CHECK_EQUAL(-1, a));
    BOOST_CHECK_NO_THROW(VR_CHECK_EQUAL(a, a2));

    BOOST_CHECK_THROW(VR_CHECK_EQUAL(0, 1), VirtualRobotCheckException);
    BOOST_CHECK_THROW(VR_CHECK_EQUAL(z, 10), VirtualRobotCheckException);
    BOOST_CHECK_THROW(VR_CHECK_EQUAL(a, b), VirtualRobotCheckException);
}


BOOST_AUTO_TEST_SUITE_END()
