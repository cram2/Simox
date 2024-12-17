/*
 * This file is part of ArmarX.
 *
 * ArmarX is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * ArmarX is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * @package    Armar6RT::ArmarXObjects::Armar6MecanumPlatform
 * @author     Simon Ottenhaus ( simon dot ottenhaus at kit dot edu )
 * @date       2017
 * @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
 *             GNU General Public License
 */

#define BOOST_TEST_MODULE VirtualRobot_MecanumPlatformKinematics

#include <iostream>

#include <VirtualRobot/IK/platform/MecanumPlatformKinematics.h>
#include <VirtualRobot/VirtualRobotTest.h>


BOOST_AUTO_TEST_SUITE(MecanumPlatformKinematics)

BOOST_AUTO_TEST_CASE(testExample)
{
    Eigen::IOFormat vecfmt(5, 0, "", ", ", "", "", "(", ")");

    const VirtualRobot::MecanumPlatformKinematicsParams params{.l1 = 250, .l2 = 300, .R = 95.f};

    const VirtualRobot::MecanumPlatformKinematics p(params);

    // std::cout << "1 0 0: " << p.calcWheelVelocity(Eigen::Vector3f{1, 0, 0}).format(vecfmt) << std::endl;
    // std::cout << "0 1 0: " << p.calcWheelVelocity(Eigen::Vector3f{0, 1, 0}).format(vecfmt) << std::endl;
    // std::cout << "0 0 1: " << p.calcWheelVelocity(Eigen::Vector3f{0, 0, 1}).format(vecfmt) << std::endl;
    BOOST_CHECK_EQUAL(true, true);
}

BOOST_AUTO_TEST_CASE(testInv)
{
    const VirtualRobot::MecanumPlatformKinematicsParams params{.l1 = 250, .l2 = 300, .R = 95.f};

    const VirtualRobot::MecanumPlatformKinematics p(params);

    float vx = 100.0;
    float vy = 200.0;
    float vAngle = 0.1f;

    Eigen::Vector4f wheelVelocities = p.calcWheelVelocity(Eigen::Vector3f{vx, vy, vAngle});
    Eigen::Vector3f velocities = p.calcCartesianVelocity(wheelVelocities);

    BOOST_TEST(vx == velocities(0), boost::test_tools::tolerance(0.1));
    BOOST_TEST(vy == velocities(1), boost::test_tools::tolerance(0.1));
    BOOST_TEST(vAngle == velocities(2), boost::test_tools::tolerance(0.001));
}

BOOST_AUTO_TEST_CASE(testMoveForwardCartesianVel)
{
    const VirtualRobot::MecanumPlatformKinematicsParams params{.l1 = 250, .l2 = 300, .R = 95.f};

    const VirtualRobot::MecanumPlatformKinematics p(params);

    Eigen::Vector4f wheelVelocities = Eigen::Vector4f::Ones();
    Eigen::Vector3f velocities = p.calcCartesianVelocity(wheelVelocities);

    BOOST_TEST(0.0 == velocities(0), boost::test_tools::tolerance(0.1));
    BOOST_TEST(0.0 < velocities(1));
    BOOST_TEST(0.0 == velocities(2), boost::test_tools::tolerance(0.001));
}

BOOST_AUTO_TEST_CASE(testMoveForwardWheelVel)
{
    const VirtualRobot::MecanumPlatformKinematicsParams params{.l1 = 250, .l2 = 300, .R = 95.f};

    const VirtualRobot::MecanumPlatformKinematics p(params);

    Eigen::Vector4f wheelVelocities = p.calcWheelVelocity(Eigen::Vector3f{0.0, 100.0, 0.0});

    BOOST_TEST(0 < wheelVelocities(0));
    BOOST_TEST(wheelVelocities(0) == wheelVelocities(1), boost::test_tools::tolerance(0.1));
    BOOST_TEST(wheelVelocities(0) == wheelVelocities(2), boost::test_tools::tolerance(0.1));
    BOOST_TEST(wheelVelocities(0) == wheelVelocities(3), boost::test_tools::tolerance(0.1));
}

BOOST_AUTO_TEST_SUITE_END()
