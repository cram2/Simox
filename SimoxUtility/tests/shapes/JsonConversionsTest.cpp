/**
* @package    SimoxUtility
* @author     Raphael Grimm
* @copyright  2019 Raphael Grimm
*/

#define BOOST_TEST_MODULE SimoxUtility / shapes / JsonConversionsTest

#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>

#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/json/eigen_conversion.h>
#include <SimoxUtility/json/io.h>
#include <SimoxUtility/json/util.h>
#include <SimoxUtility/shapes/AxisAlignedBoundingBox.h>
#include <SimoxUtility/shapes/OrientedBox.h>
#include <SimoxUtility/shapes/json_conversions.h>

#include "JsonConversionsTest_files.h"


#define BOOST_CHECK_EQUAL_VECTOR(lhs, rhs, prec)                                                   \
    BOOST_TEST_INFO((lhs).transpose() << " == " << (rhs).transpose());                             \
    BOOST_CHECK_LE(((lhs) - (rhs)).norm(), prec);

namespace JsonConversionsTest
{
    struct Fixture
    {
        const float PRECF = 1e-3f;
        const double PRECD = 1e-3;
        std::string prefix = "JsonConversionsTest__";

        std::vector<std::string> jsonAabbFilenames;
        std::vector<std::string> jsonOrientedBoxFilenames;
        std::vector<std::string> jsonAllFilenames;

        Fixture()
        {
            auto writeFile = [this](std::string filename,
                                    const std::string& text,
                                    std::vector<std::string>& list)
            {
                filename = prefix + filename;

                BOOST_TEST_MESSAGE("Writing " << std::filesystem::absolute(filename));
                if (std::filesystem::exists(filename))
                {
                    std::filesystem::remove(filename);
                }
                {
                    std::ofstream ofs(filename);
                    ofs << text;
                }

                list.push_back(filename);
                jsonAllFilenames.push_back(filename);
            };

            writeFile("AABB_center_extents.json", AABB_center_extents, jsonAabbFilenames);
            writeFile("AABB_min_max.json", AABB_min_max, jsonAabbFilenames);
            writeFile("OrientedBox_position_orientation_extents.json",
                      OrientedBox_position_orientation_extents,
                      jsonOrientedBoxFilenames);
            writeFile("OrientedBox_pos_ori_dimensions.json",
                      OrientedBox_pos_ori_dimensions,
                      jsonOrientedBoxFilenames);

            for (const std::string& filename : jsonAllFilenames)
            {
                BOOST_TEST_CONTEXT("filename: " << filename)
                {
                    BOOST_REQUIRE(std::filesystem::is_regular_file(filename));
                }
            }
        }

        ~Fixture()
        {
            for (const std::string& filename : jsonAllFilenames)
            {
                if (std::filesystem::exists(filename))
                {
                    BOOST_TEST_MESSAGE("Removing " << std::filesystem::absolute(filename));
                    std::filesystem::remove(filename);
                }
            }
        }

        template <typename FloatT>
        void
        test_read_OrientedBox_type(const FloatT prec)
        {
            using Vector3 = Eigen::Matrix<FloatT, 3, 1>;
            using Quaternion = Eigen::Quaternion<FloatT>;

            for (const std::string& filename : jsonOrientedBoxFilenames)
            {
                BOOST_TEST_CONTEXT("File " << filename)
                {
                    BOOST_REQUIRE(std::filesystem::is_regular_file(filename));

                    const nlohmann::json j = nlohmann::read_json(filename);

                    const simox::OrientedBox<FloatT> ob = j.get<simox::OrientedBox<FloatT>>();
                    BOOST_CHECK_EQUAL_VECTOR(ob.center(), Vector3(10, -20, 0), prec);
                    BOOST_CHECK_EQUAL_VECTOR(
                        ob.rotation(), Quaternion(0.707, 0, 0.707, 0).toRotationMatrix(), prec);
                    BOOST_CHECK_EQUAL_VECTOR(
                        ob.dimensions(), Vector3(100, 200, 300), prec * FloatT(1e2));
                }
            }
        }
    };

} // namespace JsonConversionsTest


BOOST_FIXTURE_TEST_SUITE(JsonConversionsTest, Fixture)

BOOST_AUTO_TEST_CASE(test_read_AABB)
{
    namespace fs = std::filesystem;
    float prec = 1e-4f;


    for (const std::string& filename : jsonAabbFilenames)
    {
        BOOST_TEST_CONTEXT("File " << filename)
        {
            BOOST_REQUIRE(fs::is_regular_file(filename));

            const nlohmann::json j = nlohmann::read_json(filename);

            const simox::AxisAlignedBoundingBox aabb = j.get<simox::AxisAlignedBoundingBox>();
            BOOST_CHECK_EQUAL_VECTOR(aabb.center(), Eigen::Vector3f(45, 90, 135), prec);
            BOOST_CHECK_EQUAL_VECTOR(aabb.extents(), Eigen::Vector3f(110, 220, 330), prec);
            BOOST_CHECK_EQUAL_VECTOR(aabb.min(), Eigen::Vector3f(-10, -20, -30), prec);
            BOOST_CHECK_EQUAL_VECTOR(aabb.max(), Eigen::Vector3f(100, 200, 300), prec);
        }
    }
}

BOOST_AUTO_TEST_CASE(test_read_OrientedBox_float)
{
    test_read_OrientedBox_type<float>(PRECF);
}

BOOST_AUTO_TEST_CASE(test_read_OrientedBox_double)
{
    test_read_OrientedBox_type<double>(PRECD);
}


BOOST_AUTO_TEST_SUITE_END()
