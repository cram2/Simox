/**
* @package    SimoxUtility
* @author     Raphael Grimm
* @copyright  2019 Raphael Grimm
*/

#define BOOST_TEST_MODULE SimoxUtility_AngleBetweenTest

#include <iostream>
#include <random>

#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/math/convert.h>
#include <SimoxUtility/math/distance/angle_between.h>
#include <SimoxUtility/math/periodic/periodic_clamp.h>
#include <SimoxUtility/math/project_to_plane.h>

static constexpr double eps = 5e-3;

BOOST_AUTO_TEST_CASE(test_delta_angle)
{
    std::mt19937 gen{std::random_device{}()};
    std::uniform_real_distribution<float> deltaDist{-M_PI, M_PI};

    for (int i = 0; i < 50; ++i)
    {
        const Eigen::Vector3f ax = Eigen::Vector3f::Random().normalized();
        const float ang = deltaDist(gen);
        const float ang_abs = std::abs(ang);

        const auto check = [](const auto& l,
                              const auto& r,
                              const float ang,
                              float ang_abs,
                              const Eigen::Vector3f& n)
        {
            const float ang_abs_l_r = simox::math::angle_between_vec3f_vec3f_abs(l, r);
            const float ang_abs_r_l = simox::math::angle_between_vec3f_vec3f_abs(r, l);
            const float ang_abs_l_l = simox::math::angle_between_vec3f_vec3f_abs(l, l);
            const float ang_abs_r_r = simox::math::angle_between_vec3f_vec3f_abs(r, r);
            const float ang_l_r = simox::math::angle_between_vec3f_vec3f(l, r, n);
            const float ang_r_l = simox::math::angle_between_vec3f_vec3f(r, l, n);
            const float ang_l_l = simox::math::angle_between_vec3f_vec3f(l, l, n);
            const float ang_r_r = simox::math::angle_between_vec3f_vec3f(r, r, n);
            const float ang_inv_ax_l_r = simox::math::angle_between_vec3f_vec3f(l, r, -n);
            const float ang_inv_ax_r_l = simox::math::angle_between_vec3f_vec3f(r, l, -n);
            const float ang_inv_ax_l_l = simox::math::angle_between_vec3f_vec3f(l, l, -n);
            const float ang_inv_ax_r_r = simox::math::angle_between_vec3f_vec3f(r, r, -n);

            std::cout << "###############################################"
                      << "\n ang             " << ang << "\n ang_abs         " << ang_abs
                      << "\n l               " << l.transpose() << " (normalized "
                      << l.normalized().transpose() << ')' << "\n r               " << r.transpose()
                      << " (normalized " << r.normalized().transpose() << ')'
                      << "\n dot             " << l.normalized().dot(r.normalized())
                      << "\n ang_abs_l_r     " << ang_abs_l_r << " (expected " << ang_abs
                      << "\t, detlta " << ang_abs_l_r - ang_abs << ')' << "\n ang_abs_r_l     "
                      << ang_abs_r_l << " (expected " << ang_abs << "\t, detlta "
                      << ang_abs_r_l - ang_abs << ')' << "\n ang_abs_l_l     " << ang_abs_l_l
                      << " (expected " << 0 << "\t, detlta " << ang_abs_l_l - 0 << ')'
                      << "\n ang_abs_r_r     " << ang_abs_r_r << " (expected " << 0 << "\t, detlta "
                      << ang_abs_r_r - 0 << ')' << "\n ang_l_r         " << ang_l_r << " (expected "
                      << ang << "\t, detlta " << ang_l_r - ang << ')' << "\n ang_r_l         "
                      << ang_r_l << " (expected " << -ang << "\t, detlta " << ang_r_l + ang << ')'
                      << "\n ang_l_l         " << ang_l_l << " (expected " << 0 << "\t, detlta "
                      << ang_l_l - 0 << ')' << "\n ang_r_r         " << ang_r_r << " (expected "
                      << 0 << "\t, detlta " << ang_r_r - 0 << ')' << "\n ang_inv_ax_l_r  "
                      << ang_inv_ax_l_r << " (expected " << -ang << "\t, detlta "
                      << ang_inv_ax_l_r + ang << ')' << "\n ang_inv_ax_r_l  " << ang_inv_ax_r_l
                      << " (expected " << ang << "\t, detlta " << ang_inv_ax_r_l - ang << ')'
                      << "\n ang_inv_ax_l_l  " << ang_inv_ax_l_l << " (expected " << 0
                      << "\t, detlta " << ang_inv_ax_l_l - 0 << ')' << "\n ang_inv_ax_r_r  "
                      << ang_inv_ax_r_r << " (expected " << 0 << "\t, detlta " << ang_inv_ax_r_r - 0
                      << ')' << "\n";

            BOOST_CHECK_LT(std::abs(ang_abs_l_r - ang_abs), eps);
            BOOST_CHECK_LT(std::abs(ang_abs_r_l - ang_abs), eps);
            BOOST_CHECK_LT(std::abs(ang_abs_l_l), eps);
            BOOST_CHECK_LT(std::abs(ang_abs_r_r), eps);

            BOOST_CHECK_LT(std::abs(ang_l_r - ang), eps);
            BOOST_CHECK_LT(std::abs(ang_r_l + ang), eps);
            BOOST_CHECK_LT(std::abs(ang_l_l), eps);
            BOOST_CHECK_LT(std::abs(ang_r_r), eps);

            BOOST_CHECK_LT(std::abs(ang_inv_ax_l_r + ang), eps);
            BOOST_CHECK_LT(std::abs(ang_inv_ax_r_l - ang), eps);
            BOOST_CHECK_LT(std::abs(ang_inv_ax_l_l), eps);
            BOOST_CHECK_LT(std::abs(ang_inv_ax_r_r), eps);
        };
        const Eigen::Vector3f vec =
            simox::math::project_to_plane(Eigen::Vector3f::Random().normalized(), ax);
        const Eigen::Vector3f rotated_r = Eigen::AngleAxisf{+ang, ax}.toRotationMatrix() * vec;
        const Eigen::Vector3f rotated_l = Eigen::AngleAxisf{-ang, ax}.toRotationMatrix() * vec;
        check(vec, rotated_r, ang, ang_abs, ax);
        check(vec, rotated_l, -ang, ang_abs, ax);
    }
}
