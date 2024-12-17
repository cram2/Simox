/**
* @package    VirtualRobot
* @author     Rainer Kartmann
* @copyright  2019 Rainer Kartmann
*/

#define BOOST_TEST_MODULE SimoxUtility / math / pose / align_box_orientation

#include <boost/test/included/unit_test.hpp>

#include <Eigen/Geometry>

#include <SimoxUtility/math/pose/align_box_orientation.h>
#include <SimoxUtility/math/pose/check_rotation_matrix.h>


namespace math = ::simox::math;
using Vector3f = Eigen::Vector3f;
using Matrix3f = Eigen::Matrix3f;

namespace
{
    void
    check_canonic(const Matrix3f orientation, const Matrix3f canonic)
    {
        for (int c = 0; c < 3; ++c)
        {
            BOOST_TEST_CONTEXT("Axis " << c << ":\n"
                                       << "Orientation:  " << orientation.col(c).transpose() << "\n"
                                       << "Canonic: " << canonic.col(c).transpose() << "\n")
            {
                float dot = orientation.col(c).dot(canonic.col(c));
                BOOST_CHECK_GE(dot, 0.5f - 1e-6f);
            }
        }
    }

    void
    test_align_box_orientation(const Matrix3f orientation,
                               const Matrix3f canonic = Matrix3f::Identity())
    {
        BOOST_TEST_CONTEXT("Orientation:\n"
                           << orientation << "\n"
                           << "Canonic (goal):\n"
                           << canonic << "\n")
        {
            try
            {
                const Matrix3f result = math::align_box_orientation(orientation, canonic);

                BOOST_TEST_CONTEXT("Result:\n" << result << "\n")
                {
                    check_canonic(result, canonic);
                }
            }
            catch (const simox::error::InvalidRotationMatrix& e)
            {
                BOOST_TEST_INFO("Caught simox::error::InvalidRotationMatrix:\n" << e.what());
                BOOST_CHECK(false);
            }
        }
    }
} // namespace

BOOST_AUTO_TEST_CASE(test_identity)
{
    test_align_box_orientation(Matrix3f::Identity());
}

BOOST_AUTO_TEST_CASE(test_edge_cases)
{
    const float pi = static_cast<float>(M_PI);

    for (int x = 0; x < 8; ++x)
    {
        const Matrix3f rotX =
            Eigen::AngleAxisf(2 * pi * x / 8.f, Vector3f::UnitX()).toRotationMatrix();

        for (int y = 0; y < 8; ++y)
        {
            const Matrix3f rotY =
                Eigen::AngleAxisf(2 * pi * y / 8.f, Vector3f::UnitY()).toRotationMatrix();

            for (int z = 0; z < 8; ++z)
            {
                const Matrix3f rotZ =
                    Eigen::AngleAxisf(2 * pi * z / 8.f, Vector3f::UnitZ()).toRotationMatrix();

                BOOST_TEST_CONTEXT("x: " << x << ", y: " << y << ", z: " << z << "\n"
                                         << "rotX:\n"
                                         << rotX << "\nrotY:\n"
                                         << rotY << "\nrotZ:\n"
                                         << rotZ)
                {
                    test_align_box_orientation(rotX * rotY * rotZ);
                }
            }
        }
    }
}

BOOST_AUTO_TEST_CASE(test_random_orientations)
{
    const int N = 10;
    for (int i = 0; i < N; ++i)
    {
        const Matrix3f rotation = Eigen::Quaternionf::UnitRandom().toRotationMatrix();
        const Matrix3f canonic = Eigen::Quaternionf::UnitRandom().toRotationMatrix();
        test_align_box_orientation(rotation, canonic);
    }
}
