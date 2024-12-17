/**
* @package    VirtualRobot
* @author     Rainer Kartmann
* @copyright  2019 Rainer Kartmann
*/

#define BOOST_TEST_MODULE SimoxUtility / math / pose / invert_pose_test

#include <stdio.h>

#include <random>
#include <string>

#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/math/pose/invert.h>
#include <SimoxUtility/math/pose/pose.h>


namespace math = ::simox::math;

BOOST_AUTO_TEST_CASE(test_invert_pose)
{
    Eigen::Vector3f translation(4, 5, 6);
    Eigen::AngleAxisf rotation(static_cast<float>(M_PI_2), Eigen::Vector3f::UnitY());

    Eigen::Matrix4f pose = math::pose(translation, rotation);
    Eigen::Matrix4f inv;

    // in-place
    inv = pose;
    math::invert_pose(inv);
    BOOST_CHECK((pose * inv).isIdentity(1e-6f));
    BOOST_CHECK((inv * pose).isIdentity(1e-6f));

    // returned
    inv.setIdentity();
    inv = math::inverted_pose(pose);
    BOOST_CHECK((pose * inv).isIdentity(1e-6f));
    BOOST_CHECK((inv * pose).isIdentity(1e-6f));
}
