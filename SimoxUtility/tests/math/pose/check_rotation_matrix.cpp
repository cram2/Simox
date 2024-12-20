/**
* @package    VirtualRobot
* @author     Rainer Kartmann
* @copyright  2019 Rainer Kartmann
*/

#define BOOST_TEST_MODULE SimoxUtility / math / pose / check_rotation_matrix

#include <boost/test/included/unit_test.hpp>

#include <Eigen/Geometry>

#include <SimoxUtility/math/pose/check_rotation_matrix.h>


namespace math = ::simox::math;
using Matrix3f = Eigen::Matrix3f;

BOOST_AUTO_TEST_CASE(test_check_rotation_matrix_basic)
{
    BOOST_CHECK_NO_THROW(math::check_rotation_matrix(Matrix3f::Identity()));
    BOOST_CHECK_THROW(math::check_rotation_matrix(Matrix3f::Zero()),
                      simox::error::InvalidRotationMatrix);
    BOOST_CHECK_THROW(math::check_rotation_matrix(Matrix3f::Ones()),
                      simox::error::InvalidRotationMatrix);
}

BOOST_AUTO_TEST_CASE(test_check_InvalidRotationMatrix_message)
{
    bool caught = false;
    try
    {
        math::check_rotation_matrix(Matrix3f::Ones());
    }
    catch (const simox::error::InvalidRotationMatrix& e)
    {
        caught = true;
        BOOST_TEST_MESSAGE(e.what());
    }
    BOOST_CHECK(caught);
}

BOOST_AUTO_TEST_CASE(test_check_rotation_matrix_random_rotations)
{
    const int N = 50;
    for (int i = 0; i < N; ++i)
    {
        const Eigen::Quaternion quat = Eigen::Quaternionf::UnitRandom();
        const Eigen::Matrix3f mat = quat.toRotationMatrix();
        BOOST_TEST_CONTEXT("Matrix: \n" << mat)
        {
            BOOST_CHECK_NO_THROW(simox::math::check_rotation_matrix(mat));
            BOOST_CHECK_NO_THROW(simox::math::check_rotation_matrix(mat.transpose()));

            BOOST_CHECK_THROW(simox::math::check_rotation_matrix((-mat).eval()),
                              simox::error::InvalidRotationMatrix);
            BOOST_CHECK_THROW(simox::math::check_rotation_matrix((-mat.transpose()).eval()),
                              simox::error::InvalidRotationMatrix);
        }
    }
}
