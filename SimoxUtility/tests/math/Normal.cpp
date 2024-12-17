/**
* @package    SimoxUtility
* @author     Raphael Grimm
* @copyright  2019 Raphael Grimm
*/

#define BOOST_TEST_MODULE SimoxUtility / math / Normal

#include <iostream>
#include <random>

#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/math/normal.h>
#include <SimoxUtility/math/pose/check_rotation_matrix.h>

static constexpr double eps = 1e-5;

BOOST_AUTO_TEST_CASE(test_normal_orth)
{
    for (std::size_t i = 0; i < 1000; ++i)
    {
        const Eigen::Vector3f v = Eigen::Vector3f::Random() * i; //check len 0
        if (v.norm() <= 1e-9) //has to be 1e-9 (internal epsilon
        {
            BOOST_CHECK_THROW(simox::math::orthogonal_vector(v), std::invalid_argument);
        }
        else
        {
            const Eigen::Vector3f vn = v.normalized();
            const Eigen::Vector3f vo = simox::math::orthogonal_vector(v);
            BOOST_CHECK_LE(vo.dot(vn), eps);
            BOOST_CHECK_LE(std::abs(vo.norm() - 1), eps);
        }
    }
}

BOOST_AUTO_TEST_CASE(test_valid_mx)
{

    for (std::size_t i = 0; i < 1000; ++i)
    {
        const Eigen::Vector3f v = Eigen::Vector3f::Random() * i; //check len 0
        if (v.norm() <= 1e-9) //has to be 1e-9 (internal epsilon
        {
            //has to fail
            BOOST_CHECK_THROW(simox::math::normal_to_mat3(v, simox::math::CoordinateSystemAxis::X),
                              std::invalid_argument);
            BOOST_CHECK_THROW(simox::math::normal_to_mat3(v, simox::math::CoordinateSystemAxis::Y),
                              std::invalid_argument);
            BOOST_CHECK_THROW(simox::math::normal_to_mat3(v, simox::math::CoordinateSystemAxis::Z),
                              std::invalid_argument);
        }
        else
        {
            const Eigen::Matrix3f mx =
                simox::math::normal_to_mat3(v, simox::math::CoordinateSystemAxis::X);
            const Eigen::Matrix3f my =
                simox::math::normal_to_mat3(v, simox::math::CoordinateSystemAxis::Y);
            const Eigen::Matrix3f mz =
                simox::math::normal_to_mat3(v, simox::math::CoordinateSystemAxis::Z);

            const Eigen::Vector3f vn = v.normalized();

            BOOST_CHECK_LE(std::abs(mx.col(0).dot(vn) - 1), eps);
            BOOST_CHECK_LE(std::abs(my.col(1).dot(vn) - 1), eps);
            BOOST_CHECK_LE(std::abs(mz.col(2).dot(vn) - 1), eps);

            BOOST_CHECK_LE(std::abs(mx.col(1).dot(vn)), eps);
            BOOST_CHECK_LE(std::abs(mx.col(2).dot(vn)), eps);

            BOOST_CHECK_LE(std::abs(my.col(0).dot(vn)), eps);
            BOOST_CHECK_LE(std::abs(my.col(2).dot(vn)), eps);

            BOOST_CHECK_LE(std::abs(mz.col(0).dot(vn)), eps);
            BOOST_CHECK_LE(std::abs(mz.col(1).dot(vn)), eps);

            simox::math::check_rotation_matrix(mx);
            simox::math::check_rotation_matrix(my);
            simox::math::check_rotation_matrix(mz);
        }
    }
}
