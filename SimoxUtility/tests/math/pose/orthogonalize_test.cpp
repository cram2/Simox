/**
* @package    VirtualRobot
* @author     Rainer Kartmann
* @copyright  2019 Rainer Kartmann
*/

#define BOOST_TEST_MODULE SimoxUtility / math / pose / orthogonalize_test

#include <random>

#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/math/pose/orthogonalize.h>
#include <SimoxUtility/math/pose/pose.h>


namespace math = ::simox::math;

struct OrthogonolizeFixture
{
    void test(double angle,
              const Eigen::Vector3d& axis,
              float noiseAmpl,
              float precAngularDist,
              float precOrth = -1);
    Eigen::Matrix3f test(Eigen::Matrix3f matrix, float noiseAmpl, float precOrth = -1);

    template <typename Distribution>
    static Eigen::Matrix3f
    Random(Distribution& distrib)
    {
        static std::default_random_engine gen(42);
        return Eigen::Matrix3f::NullaryExpr([&](int) { return distrib(gen); });
    }
};

void
OrthogonolizeFixture::test(double angle,
                           const Eigen::Vector3d& axis,
                           float noiseAmpl,
                           float precAngularDist,
                           float precOrth)
{
    // construct matrix with double to avoid rounding errors
    Eigen::AngleAxisd rot(angle, axis);
    Eigen::Quaterniond quat(rot);

    Eigen::Matrix3f matrix = quat.toRotationMatrix().cast<float>();

    Eigen::Matrix3f orth = test(matrix, noiseAmpl, precOrth);

    Eigen::Quaternionf quatOrth(orth);
    BOOST_TEST_MESSAGE("Angular distance: " << quatOrth.angularDistance(quat.cast<float>()));
    BOOST_CHECK_LE(quatOrth.angularDistance(quat.cast<float>()), precAngularDist);
}

Eigen::Matrix3f
OrthogonolizeFixture::test(Eigen::Matrix3f matrix, float noiseAmpl, float _precOrth)
{
    const float precOrth = _precOrth > 0 ? _precOrth : 1e-6f;

    const Eigen::Vector3f pos(3, -1, 2);
    Eigen::Matrix4f pose = math::pose(pos, matrix);
    pose.row(3) << 1, 2, 3, 4; // destroy last row

    BOOST_TEST_MESSAGE("Rotation matrix: \n" << matrix);
    BOOST_CHECK(math::is_matrix_orthogonal(matrix, precOrth));

    BOOST_TEST_MESSAGE("Pose matrix: \n" << pose);


    // add noise (random coeffs are in [-1, 1])
    std::normal_distribution<float> distrib(0, noiseAmpl);
    const Eigen::Matrix3f noise = noiseAmpl * this->Random(distrib);

    matrix += noise;
    math::orientation(pose) += noise;

    BOOST_TEST_MESSAGE("Rotation matrix with noise: \n" << matrix);
    if (noiseAmpl > 0)
    {
        BOOST_CHECK(!math::is_matrix_orthogonal(matrix, precOrth));
        BOOST_CHECK(!math::is_matrix_orthogonal(math::orientation(pose), precOrth));
    }

    Eigen::Matrix3f orth = math::orthogonalize(matrix);
    Eigen::Matrix4f poseOrth = math::orthogonalize_pose(pose);

    BOOST_TEST_MESSAGE("Orthogonalized: \n" << orth);
    BOOST_TEST_MESSAGE("R * R.T: (should be Identitiy) \n" << (orth * orth.transpose()));
    BOOST_CHECK(math::is_matrix_orthogonal(orth, precOrth));

    BOOST_TEST_MESSAGE("Orthogonalized pose: \n" << poseOrth);
    const auto poseOrthOri = math::orientation(poseOrth);
    BOOST_TEST_MESSAGE("R * R.T: (should be Identitiy) \n"
                       << (poseOrthOri * poseOrthOri.transpose()));
    BOOST_CHECK(math::is_matrix_orthogonal(poseOrthOri, precOrth));
    BOOST_CHECK_EQUAL(math::position(poseOrth), pos);
    BOOST_CHECK_EQUAL(poseOrth.row(3).head<3>(), Eigen::Vector3f::Zero().transpose());
    BOOST_CHECK_EQUAL(poseOrth(3, 3), 1);

    return orth;
}


BOOST_FIXTURE_TEST_SUITE(Orthogonolization, OrthogonolizeFixture)

BOOST_AUTO_TEST_CASE(test_orthogonalize_zero_rotation)
{
    test(Eigen::Matrix3f::Identity(), 0);
    test(Eigen::Matrix3f::Identity(), 0.1f);

    test(0, Eigen::Vector3d::UnitX(), 0.0f, 0.0f);
    test(0, Eigen::Vector3d::UnitX(), 1e-3f, 1e-3f);
}

BOOST_AUTO_TEST_CASE(test_orthogonalize_aligned_axis)
{
    test(M_PI / 2, Eigen::Vector3d::UnitX(), 1e-3f, 1e-3f);
    test(M_PI / 2, Eigen::Vector3d::UnitX(), 0.1f, 0.2f);

    test(.75 * M_PI, Eigen::Vector3d::UnitZ(), 1e-3f, 1e-3f);
    test(.75 * M_PI, Eigen::Vector3d::UnitZ(), 0.1f, 0.2f);

    test(M_PI, Eigen::Vector3d::UnitY(), 1e-3f, 1e-3f);
    test(M_PI, Eigen::Vector3d::UnitY(), 0.1f, 0.2f);
}

BOOST_AUTO_TEST_CASE(test_orthogonalize_arbitrary_rotation)
{
    test(2.3, Eigen::Vector3d(0.3, 1., -.5).normalized(), 1e-3f, 1e-3f);
    test(2.3, Eigen::Vector3d(0.3, 1., -.5).normalized(), 0.1f, 0.2f);

    test(1.02, Eigen::Vector3d(-2, .3, -.25).normalized(), 1e-3f, 1e-3f);
    test(1.02, Eigen::Vector3d(-3, 2, -10).normalized(), 0.1f, 0.2f, 1e-5f);
}


BOOST_AUTO_TEST_SUITE_END()
