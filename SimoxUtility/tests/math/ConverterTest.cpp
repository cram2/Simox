/**
* @package    SimoxUtility
* @author     Raphael Grimm
* @copyright  2019 Raphael Grimm
*/

#define BOOST_TEST_MODULE SimoxUtility_ConverterTest

#include <iostream>
#include <random>

#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/math/convert.h>
#include <SimoxUtility/math/distance/delta_angle.h>

BOOST_AUTO_TEST_CASE(test_convert_oris)
{
    static constexpr double eps = 1e-5;

    for (int i = 0; i < 50; ++i)
    {
        const Eigen::Quaternionf in = Eigen::Quaternionf::UnitRandom();

        const auto check = [&](const auto& o)
        { BOOST_CHECK_LT(simox::math::delta_angle(in, o), eps); };

        //aa
        {
            check(simox::math::quat_to_aa(in));

            Eigen::AngleAxisf v;
            simox::math::quat_to_aa(in, v);
            check(v);

            check(simox::math::aa_to_quat(v));
            check(simox::math::aa_to_mat3f(v));
            check(simox::math::aa_to_mat4f(v));
            check(simox::math::rpy_to_aa(simox::math::aa_to_rpy(v)));
        }

        //mat3f
        {
            check(simox::math::quat_to_mat3f(in));

            Eigen::Matrix3f v;
            simox::math::quat_to_mat3f(in, v);
            check(v);

            check(simox::math::mat3f_to_aa(v));
            check(simox::math::mat3f_to_quat(v));
            check(simox::math::mat3f_to_mat4f(v));
            check(simox::math::rpy_to_mat3f(simox::math::mat3f_to_rpy(v)));
        }

        //mat4f
        {
            check(simox::math::quat_to_mat4f(in));

            Eigen::Matrix4f v;
            simox::math::quat_to_mat4f(in, v);
            check(v);

            check(simox::math::mat4f_to_aa(v));
            check(simox::math::mat4f_to_quat(v));
            check(simox::math::mat4f_to_mat3f(v));
            check(simox::math::rpy_to_mat4f(simox::math::mat4f_to_rpy(v)));
        }

        //rpy
        {
            check(simox::math::rpy_to_aa(simox::math::quat_to_rpy(in)));

            Eigen::Vector3f v;
            simox::math::quat_to_rpy(in, v);

            check(simox::math::rpy_to_aa(v));
            check(simox::math::rpy_to_quat(v));
            check(simox::math::rpy_to_mat3f(v));
            check(simox::math::rpy_to_mat4f(v));
        }
    }
}

#include <SimoxUtility/math/convert/deg_to_rad.h>
#include <SimoxUtility/math/convert/rad_to_deg.h>
#include <SimoxUtility/math/pose/pose.h>

template <class FloatT>
void
test_deg_to_rad_to_deg()
{
    const std::vector<std::pair<FloatT, FloatT>> degRad{
        // Rads calculated with Google.
        {0, 0},
        {30, 0.5235988},
        {45, 0.7853982},
        {90, 1.570796},
        {180, 3.1415926536},
        {360, 6.283185},
        {540, 9.424778},
        {-15, -0.2617994},
        {-270, -4.712389},
        {-720, -12.56637},
        {16.25, 0.283616},
        {-32.125, -0.560687},
        {-0, -0},
        {-30, -0.5235988},
        {-45, -0.7853982},
        {-180, -3.1415926536},
    };
    const FloatT prec = static_cast<FloatT>(1e-4);

    for (const auto& [deg, rad] : degRad)
    {
        const FloatT deg_to_rad = simox::math::deg_to_rad(deg);
        const FloatT rad_to_deg = simox::math::rad_to_deg(rad);
        BOOST_TEST_CONTEXT("\n> Got deg      = " << rad_to_deg << "\n> Expected deg = " << deg
                                                 << "\n> Got rad      = " << deg_to_rad
                                                 << "\n> Expected rad = " << rad)
        {
            BOOST_CHECK_LE(std::abs(deg_to_rad - rad), prec);
            BOOST_CHECK_LE(std::abs(rad_to_deg - deg), prec);
        }
    }


    const Eigen::IOFormat iof(Eigen::StreamPrecision, 0, " ", "\n", "[", "]", "[", "]");
    auto test_mat = [&iof, prec](const auto& deg, const auto& rad)
    {
        auto deg_to_rad = simox::math::deg_to_rad(deg);
        auto rad_to_deg = simox::math::rad_to_deg(rad);
        BOOST_TEST_CONTEXT("\n> Got Rad Mat: \n"
                           << deg_to_rad.format(iof) << "\n> Expected Rad Mat: \n"
                           << rad.format(iof) << "\n> Got Deg Mat: \n"
                           << rad_to_deg.format(iof) << "\n> Expected Deg Mat: \n"
                           << deg.format(iof))
        {
            BOOST_CHECK((deg_to_rad - rad).isZero(prec));
            BOOST_CHECK((rad_to_deg - deg).isZero(prec));
        }
    };


    BOOST_REQUIRE_EQUAL(degRad.size(), 16);
    Eigen::Matrix<FloatT, 4, 4> matDeg, matRad;
    for (size_t i = 0; i < degRad.size(); ++i)
    {
        matDeg(static_cast<Eigen::Index>(i)) = degRad[i].first;
        matRad(static_cast<Eigen::Index>(i)) = degRad[i].second;
    }
    // Concrete matrix.
    test_mat(matDeg, matRad);

    // Block expressions.
    test_mat(simox::math::position(matDeg), simox::math::position(matRad));
    test_mat(simox::math::orientation(matDeg), simox::math::orientation(matRad));
}

BOOST_AUTO_TEST_CASE(test_convert_deg_to_rad_to_deg_float)
{
    test_deg_to_rad_to_deg<float>();
}

BOOST_AUTO_TEST_CASE(test_convert_deg_to_rad_to_deg_double)
{
    test_deg_to_rad_to_deg<double>();
}
