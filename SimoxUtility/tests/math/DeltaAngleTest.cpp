/**
* @package    SimoxUtility
* @author     Raphael Grimm
* @copyright  2019 Raphael Grimm
*/

#define BOOST_TEST_MODULE SimoxUtility_DeltaAngleTest

#include <iostream>
#include <random>

#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/math/convert.h>
#include <SimoxUtility/math/distance/delta_angle.h>

BOOST_AUTO_TEST_CASE(test_delta_angle)
{
    std::mt19937 gen{std::random_device{}()};
    std::uniform_real_distribution<float> deltaDist{-10 * M_PI, 10 * M_PI};
    static constexpr double eps = 1e-5;

    for (int i = 0; i < 50; ++i)
    {
        const Eigen::Vector3f ax = Eigen::Vector3f::Random().normalized();
        const float d = deltaDist(gen);
        const float dcmp = std::abs(simox::math::periodic_clamp<float>(d, -M_PI, M_PI));
        std::cout << "##########################\n##########################\nangle = " << d
                  << '\n';
        const Eigen::AngleAxisf l{0, ax};
        const Eigen::AngleAxisf r{d, ax};

        const auto check = [&](const auto& lhs, const auto& rhs)
        {
            BOOST_CHECK_LT(std::abs(simox::math::delta_angle(lhs, rhs) - dcmp), eps);
            BOOST_CHECK_LT(std::abs(simox::math::delta_angle(rhs, lhs) - dcmp), eps);

            BOOST_CHECK_LT(simox::math::delta_angle(lhs, lhs), eps);
            BOOST_CHECK_LT(simox::math::delta_angle(rhs, rhs), eps);
        };

        std::cout << "aa\n";
        check(l, r);
        std::cout << "3f\n";
        check(simox::math::aa_to_mat3f(l), simox::math::aa_to_mat3f(r));
        std::cout << "4f\n";
        check(simox::math::aa_to_mat4f(l), simox::math::aa_to_mat4f(r));
        std::cout << "q\n";
        check(simox::math::aa_to_quat(l), simox::math::aa_to_quat(r));

        std::cout << "aa 3f\n";
        check(l, simox::math::aa_to_mat3f(r));
        check(simox::math::aa_to_mat3f(l), r);

        std::cout << "aa 4f\n";
        check(l, simox::math::aa_to_mat4f(r));
        check(simox::math::aa_to_mat4f(l), r);

        std::cout << "aa q\n";
        check(l, simox::math::aa_to_quat(r));
        check(simox::math::aa_to_quat(l), r);

        std::cout << "3f 4f\n";
        check(simox::math::aa_to_mat3f(l), simox::math::aa_to_mat4f(r));
        check(simox::math::aa_to_mat4f(l), simox::math::aa_to_mat3f(r));

        std::cout << "3f q\n";
        check(simox::math::aa_to_quat(l), simox::math::aa_to_mat3f(r));
        check(simox::math::aa_to_mat3f(l), simox::math::aa_to_quat(r));

        std::cout << "4f q\n";
        check(simox::math::aa_to_quat(l), simox::math::aa_to_mat4f(r));
        check(simox::math::aa_to_mat4f(l), simox::math::aa_to_quat(r));
    }
}
