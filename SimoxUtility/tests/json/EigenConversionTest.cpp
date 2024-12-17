/**
* @package    VirtualRobot
* @author     Rainer Kartmann
* @copyright  2018 Rainer Kartmann
*/

#define BOOST_TEST_MODULE SimoxUtility / json / EigenConversionTest

#include <boost/test/included/unit_test.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <SimoxUtility/json/eigen_conversion.h>
#include <SimoxUtility/math/pose/pose.h>

namespace Eigen
{
    bool
    operator==(const Eigen::Quaternionf& lhs, const Eigen::Quaternionf& rhs)
    {
        return lhs.isApprox(rhs, 0);
    }

    std::ostream&
    operator<<(std::ostream& os, const Eigen::Quaternionf& rhs)
    {
        os << "[ " << rhs.w() << " | " << rhs.x() << " " << rhs.y() << " " << rhs.z() << " ]";
        return os;
    }
} // namespace Eigen

BOOST_AUTO_TEST_SUITE(JsonEigenConversionTest)

BOOST_AUTO_TEST_CASE(test_Matrix4f_non_transform)
{
    Eigen::Matrix4f in = in.Identity(), out = out.Zero();

    for (int i = 0; i < in.size(); ++i)
    {
        in(i) = i;
    }

    nlohmann::json j;
    j = in;
    BOOST_TEST_MESSAGE("JSON: \n" << j.dump(2));

    out = j;
    BOOST_CHECK_EQUAL(in, out);

    out = j.get<Eigen::Matrix4f>();
    BOOST_CHECK_EQUAL(in, out);
}

BOOST_AUTO_TEST_CASE(test_Matrix4f_transform)
{
    Eigen::Matrix4f in = simox::math::pose(
        Eigen::Vector3f{3, 2, 3}, Eigen::AngleAxisf(1.2f, Eigen::Vector3f(1, 2, 3).normalized()));
    Eigen::Matrix4f out = out.Zero();

    nlohmann::json j;
    j = in;
    BOOST_TEST_MESSAGE("JSON: \n" << j.dump(2));

    out = j;
    BOOST_CHECK_EQUAL(in, out);

    out = j.get<Eigen::Matrix4f>();
    BOOST_CHECK_EQUAL(in, out);
}

BOOST_AUTO_TEST_CASE(test_Vector3f)
{
    Eigen::Vector3f in = in.Identity(), out = out.Zero();

    for (int i = 0; i < in.size(); ++i)
    {
        in(i) = i;
    }

    nlohmann::json j;
    j = in;
    BOOST_TEST_MESSAGE("JSON: \n" << j.dump(2));

    out = j;
    BOOST_CHECK_EQUAL(in, out);

    out = j.get<Eigen::Vector3f>();
    BOOST_CHECK_EQUAL(in, out);
}

BOOST_AUTO_TEST_CASE(test_Vector3f_from_xyz)
{
    Eigen::Vector3f in(0, -1, 2.5), out;
    nlohmann::json j;
    j["x"] = in.x();
    j["y"] = in.y();
    j["z"] = in.z();

    BOOST_TEST_MESSAGE("JSON: \n" << j.dump(2));

    out = j;
    BOOST_CHECK_EQUAL(in, out);

    out = j.get<Eigen::Vector3f>();
    BOOST_CHECK_EQUAL(in, out);
}

BOOST_AUTO_TEST_CASE(test_Quaternionf)
{
    Eigen::Quaternionf in{
        Eigen::AngleAxisf(static_cast<float>(M_PI), Eigen::Vector3f(1, 1, 1).normalized())};
    Eigen::Quaternionf out = out.Identity();

    nlohmann::json j;
    j = in;
    BOOST_TEST_MESSAGE("JSON: \n" << j.dump(2));

    // out = j; cannot be correctly resolved

    out = j.get<Eigen::Quaternionf>();
    BOOST_CHECK_EQUAL(in, out);
}

BOOST_AUTO_TEST_CASE(test_Matrix4f_transform_from_pos_ori)
{
    Eigen::Vector3f pos(0, -1, 2.5);
    Eigen::Quaternionf ori(Eigen::AngleAxisf(static_cast<float>(M_PI_2), Eigen::Vector3f::UnitY()));
    Eigen::Matrix4f in = simox::math::pose(pos, ori);
    Eigen::Matrix4f out = out.Zero();

    // ori = Eigen::Quaternion
    nlohmann::json j;
    j["pos"] = pos;
    j["ori"] = ori;
    BOOST_TEST_MESSAGE("JSON (ori = quat): \n" << j.dump(2));

    out = j;
    BOOST_CHECK_EQUAL(in, out);

    out = j.get<Eigen::Matrix4f>();
    BOOST_CHECK_EQUAL(in, out);

    // ori = Matrix
    j = nlohmann::json();
    j["pos"] = pos;
    j["ori"] = ori.toRotationMatrix();

    BOOST_TEST_MESSAGE("JSON (ori = mat): \n" << j.dump(2));

    out = j;
    BOOST_CHECK_EQUAL(in, out);

    out = j.get<Eigen::Matrix4f>();
    BOOST_CHECK_EQUAL(in, out);
}

BOOST_AUTO_TEST_CASE(test_from_json_Vector3d)
{
    nlohmann::json j = nlohmann::json::parse("[ 100.0, -200.0, 0.0 ]");
    Eigen::Vector3d v = j.get<Eigen::Vector3d>();
    BOOST_CHECK_EQUAL(v.x(), 100);
    BOOST_CHECK_EQUAL(v.y(), -200);
    BOOST_CHECK_EQUAL(v.z(), 0);
}


BOOST_AUTO_TEST_SUITE_END()
