/**
* @package    VirtualRobot
* @author     Rainer Kartmann
* @copyright  2019 Rainer Kartmann
*/

#define BOOST_TEST_MODULE SimoxUtility / math / pose / pose_test

#include <stdio.h>

#include <random>
#include <string>

#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/math/pose.h>


namespace math = ::simox::math;

struct BlockFixture
{
    BlockFixture()
    {
        quat = Eigen::Quaternionf{
            Eigen::AngleAxisf(static_cast<float>(M_PI), Eigen::Vector3f::UnitZ()) *
            Eigen::AngleAxisf(static_cast<float>(M_PI_2), Eigen::Vector3f::UnitY())};

        quat2 = Eigen::AngleAxisf(static_cast<float>(M_PI_4), Eigen::Vector3f::UnitX()) * quat;

        pos = Eigen::Vector3f{1, 2, 3};
        pos2 = Eigen::Vector3f{4, 5, 6};

        ori = quat.toRotationMatrix();
        ori2 = quat2.toRotationMatrix();

        pose.setIdentity();
        pose.block<3, 1>(0, 3) = pos;
        pose.block<3, 3>(0, 0) = ori;
    }

    Eigen::Matrix4f pose;
    Eigen::Vector3f pos, pos2;
    Eigen::Matrix3f ori, ori2;
    Eigen::Quaternionf quat, quat2;
};


BOOST_FIXTURE_TEST_SUITE(MathHelpers, BlockFixture)

BOOST_AUTO_TEST_CASE(test_position_const)
{
    BOOST_CHECK_EQUAL(math::position(const_cast<const Eigen::Matrix4f&>(pose)), pos);
}

BOOST_AUTO_TEST_CASE(test_position_nonconst)
{
    BOOST_CHECK_EQUAL(math::position(pose), pos);

    math::position(pose) = pos2;
    BOOST_CHECK_EQUAL(math::position(pose), pos2);
}

BOOST_AUTO_TEST_CASE(test_orientation_const)
{
    BOOST_CHECK_EQUAL(math::orientation(const_cast<const Eigen::Matrix4f&>(pose)), ori);
}

BOOST_AUTO_TEST_CASE(test_orientation_nonconst)
{
    BOOST_CHECK_EQUAL(math::orientation(pose), ori);

    math::orientation(pose) = ori2;
    BOOST_CHECK_EQUAL(math::orientation(pose), ori2);

    math::orientation(pose) = quat.toRotationMatrix();
    BOOST_CHECK_EQUAL(math::orientation(pose), quat.toRotationMatrix());
}

BOOST_AUTO_TEST_CASE(test_pose_matrix_and_quaternion)
{
    BOOST_CHECK_EQUAL(math::pose(pos, quat), pose);
}

BOOST_AUTO_TEST_CASE(test_pose_matrix_and_rotation_matrix)
{
    BOOST_CHECK_EQUAL(math::pose(pos, ori), pose);
}

BOOST_AUTO_TEST_CASE(test_pose_position)
{
    Eigen::Matrix4f posePos = posePos.Identity();
    posePos.block<3, 1>(0, 3) = pos;
    BOOST_CHECK_EQUAL(math::pose(pos), posePos);
}

BOOST_AUTO_TEST_CASE(test_pose_orientation_matrix)
{
    Eigen::Matrix4f poseOri = poseOri.Identity();
    poseOri.block<3, 3>(0, 0) = ori;
    BOOST_CHECK_EQUAL(math::pose(ori), poseOri);
}

BOOST_AUTO_TEST_CASE(test_pose_quaternion)
{
    Eigen::Matrix4f poseOri = poseOri.Identity();
    poseOri.block<3, 3>(0, 0) = ori;
    BOOST_CHECK_EQUAL(math::pose(quat), poseOri);
}


BOOST_AUTO_TEST_SUITE_END()
