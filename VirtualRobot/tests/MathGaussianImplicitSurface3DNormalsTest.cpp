/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2012 Nikolaus Vahrenkamp
*/

#define BOOST_TEST_MODULE VirtualRobot_MathGaussianImplicitSurface3DNormalsTest

#include <stdio.h>

#include <string>

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/math/GaussianImplicitSurface3DNormals.h>

BOOST_AUTO_TEST_SUITE(MathGaussianImplicitSurface3DNormals)

using namespace math;

typedef Eigen::Vector3f Vec3;

BOOST_AUTO_TEST_CASE(testGPIS)
{
    ContactList contacts;
    contacts.push_back(Contact(1, 0, 0, 1, 0, 0));
    contacts.push_back(Contact(0, 1, 0, 0, 1, 0));
    GaussianImplicitSurface3DNormals gpis(
        std::unique_ptr<WilliamsPlusKernel>(new WilliamsPlusKernel));
    gpis.Calculate(contacts, 0, 0, 1);
    BOOST_CHECK_LE(gpis.Get(Vec3(0, 0, 0)), -0.2);
    BOOST_CHECK_LE(abs(gpis.Get(Vec3(1, 0, 0))), 0.001);
    BOOST_CHECK_LE(abs(gpis.Get(Vec3(0, 1, 0))), 0.001);
    BOOST_CHECK_GE(abs(gpis.Get(Vec3(2, 0, 0))), 1.5);
}


BOOST_AUTO_TEST_SUITE_END()
