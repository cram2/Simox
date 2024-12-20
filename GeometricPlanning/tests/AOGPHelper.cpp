
#include <filesystem>

#include "VirtualRobot/XML/RobotIO.h"
#define BOOST_TEST_MODULE GeometricPlanning_AOGPHelper

#include <boost/test/unit_test.hpp>

#include "../ArticulatedObjectGeometricPlanningHelper.h"

BOOST_AUTO_TEST_SUITE(ArticulatedObjectGeometricPlanningHelper)

static constexpr const char* scDataDir =
    "/common/homes/students/hoefer/workspace/base/simox-control/data/";

static constexpr float precision = 0.00001;
static constexpr size_t nSamples = 50;

static std::filesystem::path
getPath(const std::string& model)
{
    return std::filesystem::path(scDataDir) / "Model" / "mobile_kitchen" / model /
           (model + "_articulated.xml");
}

VirtualRobot::RobotPtr
load_robot(const std::string& model)
{
    // TODO(Hawo): somehow input a simple robot model
    const auto path = getPath(model);
    BOOST_REQUIRE(std::filesystem::exists(path));

    auto robot = VirtualRobot::RobotIO::loadRobot(path);
    BOOST_REQUIRE(robot != nullptr);

    return robot;
}

using AOGPHelper = simox::geometric_planning::ArticulatedObjectGeometricPlanningHelper;

static void
printMat(const Eigen::MatrixXf& mat)
{
    for (int row = 0; row < mat.rows(); ++row)
    {
        for (int col = 0; col < mat.cols(); ++col)
        {
            float val = mat(row, col);
            val = std::fabs(val) < precision ? 0 : val;
            std::cout << val << ",";
        }
        std::cout << "\n";
    }
}

BOOST_AUTO_TEST_CASE(AOGPHelper_circle_first_path_element)
{
    const std::string articulatedObjectName = "mobile-fridge";
    const std::string handleName = "Fridge_handle";
    auto robot = load_robot(articulatedObjectName);

    auto helper = AOGPHelper(robot);
    auto path = helper.getPathForNode(handleName);
    auto range = path.parameterRange();

    Eigen::Matrix4f pathPose = path.getPose(range.min).matrix();
    Eigen::Matrix4f objectPose = robot->getRobotNode(handleName)->getGlobalPose();

    Eigen::Matrix4f diff = objectPose.inverse() * pathPose - Eigen::Matrix4f::Identity();
    float sse = diff.array().square().sum();
    if (sse > precision)
    {
        std::cout << "Object Pose (target):\n";
        std::cout << objectPose << "\n";
        std::cout << "Path Pose:\n";
        std::cout << pathPose << "\n";
        printMat(diff);
    }
    BOOST_REQUIRE(sse < precision);
}

BOOST_AUTO_TEST_CASE(AOGPHelper_circle_path_whole_around_z)
{
    const std::string articulatedObjectName = "mobile-fridge";
    const std::string handleName = "Fridge_handle";
    const std::string jointName = "Fridge_joint";
    auto robot = load_robot(articulatedObjectName);

    auto helper = AOGPHelper(robot);
    auto path = helper.getPathForNode(handleName);
    auto range = path.parameterRange();


    for (size_t i = 0; i < nSamples; ++i)
    {
        float t = range.min + (range.max - range.min) / nSamples * i;
        Eigen::Matrix4f pathPose = path.getPose(t).matrix();

        robot->setJointValue(jointName, t);
        Eigen::Matrix4f objectPose = robot->getRobotNode(handleName)->getGlobalPose();

        Eigen::Matrix4f diff = objectPose.inverse() * pathPose - Eigen::Matrix4f::Identity();
        float sse = diff.array().square().sum();
        if (sse > precision)
        {
            std::cout << "SSE at " << t << ": " << sse << "\n";
            printMat(diff);
        }
        BOOST_REQUIRE(sse < precision);
    }
}

BOOST_AUTO_TEST_CASE(AOGPHelper_circle_path_whole_around_y)
{
    const std::string articulatedObjectName = "mobile-dishwasher";
    const std::string handleName = "Dishwasher->Dishwasher_handle";
    const std::string jointName = "Dishwasher_joint";
    auto robot = load_robot(articulatedObjectName);

    auto helper = AOGPHelper(robot);
    auto path = helper.getPathForNode(handleName);
    auto range = path.parameterRange();


    for (size_t i = 0; i < nSamples; ++i)
    {
        float t = range.min + (range.max - range.min) / nSamples * i;
        Eigen::Matrix4f pathPose = path.getPose(t).matrix();

        robot->setJointValue(jointName, t);
        Eigen::Matrix4f objectPose = robot->getRobotNode(handleName)->getGlobalPose();

        Eigen::Matrix4f diff = objectPose.inverse() * pathPose - Eigen::Matrix4f::Identity();
        float sse = diff.array().square().sum();
        if (sse > precision)
        {
            std::cout << "SSE at " << t << ": " << sse << "\n";
            printMat(diff);
        }
        BOOST_REQUIRE(sse < precision);
    }
}

BOOST_AUTO_TEST_CASE(AOGPHelper_linear_path_whole)
{
    const std::string articulatedObjectName = "mobile-kitchen-counter";
    const std::string handleName = "DrawerMiddle_handle";
    const std::string jointName = "DrawerMiddle_joint";
    auto robot = load_robot(articulatedObjectName);

    auto helper = AOGPHelper(robot);
    auto path = helper.getPathForNode(handleName);
    auto range = path.parameterRange();


    for (size_t i = 0; i < nSamples; ++i)
    {
        float t = range.min + (range.max - range.min) / nSamples * i;
        Eigen::Matrix4f pathPose = path.getPose(t).matrix();

        robot->setJointValue(jointName, t);
        Eigen::Matrix4f objectPose = robot->getRobotNode(handleName)->getGlobalPose();

        Eigen::Matrix4f diff = objectPose.inverse() * pathPose - Eigen::Matrix4f::Identity();
        float sse = diff.array().square().sum();
        if (sse > precision)
        {
            std::cout << "SSE at " << t << ": " << sse << "\n";
            printMat(diff);
        }
        BOOST_REQUIRE(sse < precision);
    }
}


BOOST_AUTO_TEST_SUITE_END()
