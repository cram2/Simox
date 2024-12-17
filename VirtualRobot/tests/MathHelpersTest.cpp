/**
* @package    VirtualRobot
* @author     Rainer Kartmann
* @copyright  2019 Rainer Kartmann
*/

#define BOOST_TEST_MODULE VirtualRobot_MathHelpersTest

#include <stdio.h>

#include <random>
#include <string>

#include <Eigen/Geometry>

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/math/Helpers.h>

using Helpers = math::Helpers;

BOOST_AUTO_TEST_SUITE(MathHelpers)

BOOST_AUTO_TEST_CASE(test_CwiseMin_CwiseMax)
{
    Eigen::Vector3f a(-1, 3, 5), b(0, 3, 1);
    Eigen::Vector3f min(-1, 3, 1);
    Eigen::Vector3f max(0, 3, 5);
    BOOST_CHECK_EQUAL(Helpers::CwiseMin(a, b), min);
    BOOST_CHECK_EQUAL(Helpers::CwiseMax(a, b), max);
}

BOOST_AUTO_TEST_CASE(test_CwiseDivide)
{
    Eigen::Vector3f a(0, 5, -9), b(10, 2, 3);
    Eigen::Vector3f quot(0, 2.5, -3);
    BOOST_CHECK_EQUAL(Helpers::CwiseDivide(a, b), quot);
}

BOOST_AUTO_TEST_CASE(test_Swap)
{
    float a = 5, b = -10;
    Helpers::Swap(a, b);
    BOOST_CHECK_EQUAL(a, -10);
    BOOST_CHECK_EQUAL(b, 5);
}

BOOST_AUTO_TEST_CASE(test_GetRotationMatrix)
{
    Eigen::Vector3f source(1, 2, 3), target(-3, 2, 5); // not normalized
    Eigen::Matrix3f matrix = Helpers::GetRotationMatrix(source, target);

    BOOST_CHECK((matrix * matrix.transpose()).isIdentity(1e-6f));
    BOOST_CHECK((matrix * source.normalized()).isApprox(target.normalized(), 1e-6f));
}

BOOST_AUTO_TEST_CASE(test_TransformPosition)
{
    Eigen::Vector3f vector(1, 2, 3);

    Eigen::Vector3f translation(4, 5, 6);
    Eigen::AngleAxisf rotation(static_cast<float>(M_PI_2), Eigen::Vector3f::UnitY());

    Eigen::Matrix4f transform = transform.Identity();

    // identity
    transform.setIdentity();
    BOOST_CHECK_EQUAL(Helpers::TransformPosition(transform, vector), vector);

    // translation only
    transform.setIdentity();
    Helpers::Position(transform) = translation;
    BOOST_CHECK_EQUAL(Helpers::TransformPosition(transform, vector), vector + translation);

    // rotation only
    transform.setIdentity();
    Helpers::Orientation(transform) = rotation.toRotationMatrix();
    BOOST_CHECK_EQUAL(Helpers::TransformPosition(transform, vector), rotation * vector);

    // full transform
    transform.setIdentity();
    Helpers::Position(transform) = translation;
    Helpers::Orientation(transform) = rotation.toRotationMatrix();
    BOOST_CHECK_EQUAL(Helpers::TransformPosition(transform, vector),
                      rotation * vector + translation);
}

BOOST_AUTO_TEST_CASE(test_InvertPose)
{
    Eigen::Vector3f translation(4, 5, 6);
    Eigen::AngleAxisf rotation(static_cast<float>(M_PI_2), Eigen::Vector3f::UnitY());

    Eigen::Matrix4f pose = Helpers::Pose(translation, rotation);
    Eigen::Matrix4f inv;

    // in-place
    inv = pose;
    Helpers::InvertPose(inv);
    BOOST_CHECK((pose * inv).isIdentity(1e-6f));
    BOOST_CHECK((inv * pose).isIdentity(1e-6f));

    // returned
    inv.setIdentity();
    inv = Helpers::InvertedPose(pose);
    BOOST_CHECK((pose * inv).isIdentity(1e-6f));
    BOOST_CHECK((inv * pose).isIdentity(1e-6f));
}


BOOST_AUTO_TEST_SUITE_END()

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

using namespace math;

BOOST_AUTO_TEST_CASE(test_Position_const)
{
    BOOST_CHECK_EQUAL(Helpers::Position(const_cast<const Eigen::Matrix4f&>(pose)), pos);
}

BOOST_AUTO_TEST_CASE(test_Position_nonconst)
{
    BOOST_CHECK_EQUAL(Helpers::Position(pose), pos);

    Helpers::Position(pose) = pos2;
    BOOST_CHECK_EQUAL(Helpers::Position(pose), pos2);
}

BOOST_AUTO_TEST_CASE(test_Orientation_const)
{
    BOOST_CHECK_EQUAL(Helpers::Orientation(const_cast<const Eigen::Matrix4f&>(pose)), ori);
}

BOOST_AUTO_TEST_CASE(test_Orientation_nonconst)
{
    BOOST_CHECK_EQUAL(Helpers::Orientation(pose), ori);

    Helpers::Orientation(pose) = ori2;
    BOOST_CHECK_EQUAL(Helpers::Orientation(pose), ori2);
}

BOOST_AUTO_TEST_CASE(test_Pose_matrix_and_quaternion)
{
    BOOST_CHECK_EQUAL(Helpers::Pose(pos, quat), pose);
}

BOOST_AUTO_TEST_CASE(test_Pose_matrix_and_rotation_matrix)
{
    BOOST_CHECK_EQUAL(Helpers::Pose(pos, ori), pose);
}

BOOST_AUTO_TEST_CASE(test_Pose_position)
{
    Eigen::Matrix4f posePos = posePos.Identity();
    posePos.block<3, 1>(0, 3) = pos;
    BOOST_CHECK_EQUAL(Helpers::Pose(pos), posePos);
}

BOOST_AUTO_TEST_CASE(test_Pose_orientation_matrix)
{
    Eigen::Matrix4f poseOri = poseOri.Identity();
    poseOri.block<3, 3>(0, 0) = ori;
    BOOST_CHECK_EQUAL(Helpers::Pose(ori), poseOri);
}

BOOST_AUTO_TEST_CASE(test_Pose_quaternion)
{
    Eigen::Matrix4f poseOri = poseOri.Identity();
    poseOri.block<3, 3>(0, 0) = ori;
    BOOST_CHECK_EQUAL(Helpers::Pose(quat), poseOri);
}


BOOST_AUTO_TEST_SUITE_END()

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
    Eigen::Matrix4f pose = Helpers::Pose(pos, matrix);
    pose.row(3) << 1, 2, 3, 4; // destroy last row

    BOOST_TEST_MESSAGE("Rotation matrix: \n" << matrix);
    BOOST_CHECK(math::Helpers::IsMatrixOrthogonal(matrix, precOrth));

    BOOST_TEST_MESSAGE("Pose matrix: \n" << pose);


    // add noise (random coeffs are in [-1, 1])
    std::normal_distribution<float> distrib(0, noiseAmpl);
    const Eigen::Matrix3f noise = noiseAmpl * this->Random(distrib);

    matrix += noise;
    Helpers::Orientation(pose) += noise;

    BOOST_TEST_MESSAGE("Rotation matrix with noise: \n" << matrix);
    if (noiseAmpl > 0)
    {
        BOOST_CHECK(!math::Helpers::IsMatrixOrthogonal(matrix, precOrth));
        BOOST_CHECK(!math::Helpers::IsMatrixOrthogonal(Helpers::Orientation(pose), precOrth));
    }

    Eigen::Matrix3f orth = math::Helpers::Orthogonalize(matrix);
    Eigen::Matrix4f poseOrth = math::Helpers::Orthogonalize(pose);

    BOOST_TEST_MESSAGE("Orthogonalized: \n" << orth);
    BOOST_TEST_MESSAGE("R * R.T: (should be Identitiy) \n" << (orth * orth.transpose()));
    BOOST_CHECK(math::Helpers::IsMatrixOrthogonal(orth, precOrth));

    BOOST_TEST_MESSAGE("Orthogonalized pose: \n" << poseOrth);
    const auto poseOrthOri = Helpers::Orientation(poseOrth);
    BOOST_TEST_MESSAGE("R * R.T: (should be Identitiy) \n"
                       << (poseOrthOri * poseOrthOri.transpose()));
    BOOST_CHECK(math::Helpers::IsMatrixOrthogonal(poseOrthOri, precOrth));
    BOOST_CHECK_EQUAL(math::Helpers::Position(poseOrth), pos);
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

struct DegreeRadianConversionFixture
{
    const double rad2degd = static_cast<double>(180. / M_PI);
    const float rad2degf = static_cast<float>(rad2degd);

    const double deg2radd = static_cast<double>(M_PI / 180.);
    const float deg2radf = static_cast<float>(deg2radd);

    const float prec = 1e-5f;

    const Eigen::Vector2f vector2f = {0, 1};
    const Eigen::Vector2d vector2d = {0, 1};

    const Eigen::Vector3f vector3f = {0, 1, -2};
    const Eigen::Vector3d vector3d = {0, 1, -2};
};

BOOST_FIXTURE_TEST_SUITE(DegreeRadianConversion, DegreeRadianConversionFixture)

BOOST_AUTO_TEST_CASE(test_rad2deg_scalar)
{
    // float
    BOOST_CHECK_CLOSE(Helpers::rad2deg(0.0f), 0.0f, prec);
    BOOST_CHECK_CLOSE(Helpers::rad2deg(1.0f), rad2degf, prec);
    BOOST_CHECK_CLOSE(Helpers::rad2deg(-2.0f), -2 * rad2degf, prec);

    // double
    BOOST_CHECK_CLOSE(Helpers::rad2deg(0.0), 0.0, prec);
    BOOST_CHECK_CLOSE(Helpers::rad2deg(1.0), rad2degd, prec);
    BOOST_CHECK_CLOSE(Helpers::rad2deg(-2.0), -2 * rad2degd, prec);

    // int
    BOOST_CHECK_CLOSE(Helpers::rad2deg(0), 0.0f, prec);
    BOOST_CHECK_CLOSE(Helpers::rad2deg(1), rad2degf, prec);
    BOOST_CHECK_CLOSE(Helpers::rad2deg(-2), -2 * rad2degf, prec);

    // long
    BOOST_CHECK_CLOSE(Helpers::rad2deg(0l), 0.0f, prec);
    BOOST_CHECK_CLOSE(Helpers::rad2deg(1l), rad2degf, prec);
    BOOST_CHECK_CLOSE(Helpers::rad2deg(-2l), -2 * rad2degf, prec);
}

BOOST_AUTO_TEST_CASE(test_rad2deg_vector)
{
    BOOST_CHECK(math::Helpers::rad2deg(vector2f).isApprox(vector2f * rad2degf));
    BOOST_CHECK(math::Helpers::rad2deg(vector2d).isApprox(vector2d * rad2degd));

    BOOST_CHECK(math::Helpers::rad2deg(vector3f).isApprox(vector3f * rad2degf));
    BOOST_CHECK(math::Helpers::rad2deg(vector3d).isApprox(vector3d * rad2degd));

    // Insert expression instead of value.
    BOOST_CHECK(math::Helpers::rad2deg(3 * vector3f).isApprox(3 * vector3f * rad2degf));
    BOOST_CHECK(math::Helpers::rad2deg(3 * vector3d).isApprox(3 * vector3d * rad2degd));
}

BOOST_AUTO_TEST_CASE(test_deg2rad_scalar)
{
    // float
    BOOST_CHECK_CLOSE(Helpers::deg2rad(0.0f), 0.0f, prec);
    BOOST_CHECK_CLOSE(Helpers::deg2rad(1.0f), deg2radf, prec);
    BOOST_CHECK_CLOSE(Helpers::deg2rad(-2.0f), -2 * deg2radf, prec);

    // double
    BOOST_CHECK_CLOSE(Helpers::deg2rad(0.0), 0.0, prec);
    BOOST_CHECK_CLOSE(Helpers::deg2rad(1.0), deg2radd, prec);
    BOOST_CHECK_CLOSE(Helpers::deg2rad(-2.0), -2 * deg2radf, prec);

    // int
    BOOST_CHECK_CLOSE(Helpers::deg2rad(0), 0, prec);
    BOOST_CHECK_CLOSE(Helpers::deg2rad(1), deg2radf, prec);
    BOOST_CHECK_CLOSE(Helpers::deg2rad(-2), -2 * deg2radf, prec);

    // long
    BOOST_CHECK_CLOSE(Helpers::deg2rad(0l), 0.0f, prec);
    BOOST_CHECK_CLOSE(Helpers::deg2rad(1l), deg2radf, prec);
    BOOST_CHECK_CLOSE(Helpers::deg2rad(-2l), -2 * deg2radf, prec);
}

BOOST_AUTO_TEST_CASE(test_deg2rad_vector)
{
    BOOST_CHECK(math::Helpers::deg2rad(vector2f).isApprox(vector2f * deg2radf));
    BOOST_CHECK(math::Helpers::deg2rad(vector2d).isApprox(vector2d * deg2radd));

    BOOST_CHECK(math::Helpers::deg2rad(vector3f).isApprox(vector3f * deg2radf));
    BOOST_CHECK(math::Helpers::deg2rad(vector3d).isApprox(vector3d * deg2radd));

    // Insert expression instead of value.
    BOOST_CHECK(math::Helpers::deg2rad(3 * vector3f).isApprox(3 * vector3f * deg2radf));
    BOOST_CHECK(math::Helpers::deg2rad(3 * vector3d).isApprox(3 * vector3d * deg2radd));
}


BOOST_AUTO_TEST_SUITE_END()


#define BOOST_CHECK_EQUAL_EIGEN(L, R)                                                              \
    {                                                                                              \
        BOOST_CHECK_MESSAGE(L.isApprox(R),                                                         \
                            "check " << #L << " == " << #R << " has failed\n[\n"                   \
                                     << L << "\n] != [\n"                                          \
                                     << R << "\n]");                                               \
    }

struct GetTransformFromToTestFixture
{
    // A -> Global
    Eigen::Matrix4f poseAG;

    // B -> Global
    Eigen::Matrix4f poseBG;

    // A -> B
    Eigen::Matrix4f poseAB;
    // B -> A
    Eigen::Matrix4f poseBA;

    Eigen::Vector3f origin = origin.Zero();

    GetTransformFromToTestFixture()
    {
    }

    void
    setFramePoses(const Eigen::Matrix4f& poseAG, const Eigen::Matrix4f& poseBG)
    {
        this->poseAG = poseAG;
        this->poseBG = poseBG;

        // A -> B
        poseAB = Helpers::GetTransformFromTo(poseAG, poseBG);
        // B -> A
        poseBA = Helpers::GetTransformFromTo(poseBG, poseAG);
    }

    void
    test_commuting()
    {
        // A -> B -> Global == A -> Global
        BOOST_CHECK_EQUAL_EIGEN((poseBG * poseAB).eval(), poseAG);
        // B -> A -> Global == B -> Global
        BOOST_CHECK_EQUAL_EIGEN((poseAG * poseBA).eval(), poseBG);
    }

    void
    test_inversion_consistency()
    {
        // (A -> B)^-1 == (B -> A) (and the other way round).
        BOOST_CHECK_EQUAL_EIGEN(Helpers::InvertedPose(poseAB), poseBA);
        BOOST_CHECK_EQUAL_EIGEN(Helpers::InvertedPose(poseBA), poseAB);
    }
};


BOOST_FIXTURE_TEST_SUITE(GetTransformFromToTest, GetTransformFromToTestFixture)

BOOST_AUTO_TEST_CASE(test_translation_only)
{
    /*   ^
     *   |B
     * 1 +-->
     *   |     ^
     *   |G    |A
     * 0 +-----+--->
     *   0     1
     *
     * 0_A:
     *  in A: ( 0,  0)
     *  in B: ( 1, -1)
     *  in G: ( 1,  0)
     * 0_B:
     *  in B: ( 0,  0)
     *  in A: (-1,  1)
     *  in G: ( 0,  1)
     */

    setFramePoses(Helpers::Pose(Eigen::Vector3f(1, 0, 0)), Helpers::Pose(Eigen::Vector3f(0, 1, 0)));

    BOOST_CHECK_EQUAL_EIGEN(poseAB, Helpers::Pose(Eigen::Vector3f(1, -1, 0)));
    BOOST_CHECK_EQUAL_EIGEN(poseBA, Helpers::Pose(Eigen::Vector3f(-1, 1, 0)));

    // 0_A -> Global
    BOOST_CHECK_EQUAL_EIGEN(Helpers::TransformPosition(poseAG, origin), Eigen::Vector3f(1, 0, 0));
    BOOST_CHECK_EQUAL_EIGEN(Helpers::TransformPosition(poseBG, origin), Eigen::Vector3f(0, 1, 0));

    test_commuting();
    test_inversion_consistency();
}

BOOST_AUTO_TEST_CASE(test_rotation_only)
{
    setFramePoses(Helpers::Pose(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()).cast<float>()),
                  Helpers::Pose(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()).cast<float>()));

    test_commuting();
    test_inversion_consistency();
}

BOOST_AUTO_TEST_CASE(test_arbitrary)
{
    setFramePoses(Helpers::Pose(Eigen::Vector3f(-1, 0, 2),
                                Eigen::AngleAxisf(1.25, Eigen::Vector3f(1, 0, 1).normalized())),
                  Helpers::Pose(Eigen::Vector3f(3, -5, 0),
                                Eigen::AngleAxisf(-0.5, Eigen::Vector3f(-1, 1, 0).normalized())));

    test_commuting();
    test_inversion_consistency();
}


BOOST_AUTO_TEST_SUITE_END()
