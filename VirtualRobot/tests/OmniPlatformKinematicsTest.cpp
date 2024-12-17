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
 * @author     Fabian Reister ( fabian dot reister at kit dot edu )
 * @date       2021
 * @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
 *             GNU General Public License
 */

#include <cmath>

#include <boost/test/tools/old/interface.hpp>
#define BOOST_TEST_MODULE VirtualRobot_OmniPlatformKinematics

#include <boost/test/unit_test.hpp>

// #include <boost/test/tools/old/interface.hpp>
#include <iostream>

#include "MathTools.h"

//

#include <VirtualRobot/IK/platform/OmniWheelPlatformKinematics.h>
// #include <VirtualRobot/VirtualRobotTest.h>

BOOST_AUTO_TEST_SUITE(OmniPlatformKinematics)

const inline VirtualRobot::OmniWheelPlatformKinematics::Params ARMAR7_OMNI_PLATFORM_CONFIG{
    .L = 326, // [mm]
    .R = 125 / 2, // [mm]
    .delta = VirtualRobot::MathTools::deg2rad(30), // [rad]
    .n = 1,
    .relativeAngle = M_PI / 3,
    .wheelFactor = Eigen::Vector3f{-1, 1, 1}};

BOOST_AUTO_TEST_CASE(testExample)
{
    Eigen::IOFormat vecfmt(5, 0, "", ", ", "", "", "(", ")");

    const VirtualRobot::OmniWheelPlatformKinematicsParams params = ARMAR7_OMNI_PLATFORM_CONFIG;

    const VirtualRobot::OmniWheelPlatformKinematics p(params);

    // std::cout << "1 0 0: " << p.calcWheelVelocity(Eigen::Vector3f{1, 0, 0}).format(vecfmt) << std::endl;
    // std::cout << "0 1 0: " << p.calcWheelVelocity(Eigen::Vector3f{0, 1, 0}).format(vecfmt) << std::endl;
    // std::cout << "0 0 1: " << p.calcWheelVelocity(Eigen::Vector3f{0, 0, 1}).format(vecfmt) << std::endl;
    BOOST_CHECK_EQUAL(true, true);
}

BOOST_AUTO_TEST_CASE(testInv)
{
    const VirtualRobot::OmniWheelPlatformKinematicsParams params = ARMAR7_OMNI_PLATFORM_CONFIG;

    const VirtualRobot::OmniWheelPlatformKinematics p(params);

    const float vx = 100.0;
    const float vy = 200.0;
    const float vAngle = 0.1f;

    const Eigen::Vector3f wheelVelocities = p.calcWheelVelocity(Eigen::Vector3f{vx, vy, vAngle});
    const Eigen::Vector3f velocities = p.calcCartesianVelocity(wheelVelocities);

    BOOST_CHECK_CLOSE(vx, velocities(0), 0.1);
    BOOST_CHECK_CLOSE(vy, velocities(1), 0.1);
    BOOST_CHECK_CLOSE(vAngle, velocities(2), 0.001);
}

#define VAROUT(var) #var " = " << var

BOOST_AUTO_TEST_CASE(testMoveForward)
{
    const VirtualRobot::OmniWheelPlatformKinematicsParams params = ARMAR7_OMNI_PLATFORM_CONFIG;

    const VirtualRobot::OmniWheelPlatformKinematics p(params);

    const Eigen::Vector3f velForward = Eigen::Vector3f::UnitY();
    const auto wheelVelocities = p.calcWheelVelocity(velForward);

    // front wheel should not move
    BOOST_TEST_INFO(0.0 << " | " << VAROUT(wheelVelocities(0)));
    BOOST_CHECK_LE(std::abs(0.0 - wheelVelocities(0)), 0.02);

    // rear wheels should move at the same speed ...
    BOOST_TEST_INFO(VAROUT(wheelVelocities(1)) << " | " << VAROUT(wheelVelocities(2)));
    BOOST_CHECK_LE(std::abs(std::abs(wheelVelocities(1)) - std::abs(wheelVelocities(2))), 0.02);

    // .. but with different signs
    BOOST_CHECK_GE(wheelVelocities(1), 0.0);
    BOOST_CHECK_LE(wheelVelocities(2), 1e-8);
}

BOOST_AUTO_TEST_CASE(testRotate)
{
    const VirtualRobot::OmniWheelPlatformKinematicsParams params = ARMAR7_OMNI_PLATFORM_CONFIG;

    const VirtualRobot::OmniWheelPlatformKinematics p(params);

    const Eigen::Vector3f wheelVelocities = p.calcWheelVelocity(Eigen::Vector3f{0.0, 0.0, 1.0});

    // all wheels should move at the same speed ...
    BOOST_CHECK_CLOSE(wheelVelocities(0), wheelVelocities(1), 0.1);
    BOOST_CHECK_CLOSE(wheelVelocities(0), wheelVelocities(2), 0.1);

    // ... clockwise
    BOOST_CHECK_GE(wheelVelocities(0), 0.0);
}

BOOST_AUTO_TEST_CASE(testRotateInv)
{
    const VirtualRobot::OmniWheelPlatformKinematicsParams params = ARMAR7_OMNI_PLATFORM_CONFIG;

    const VirtualRobot::OmniWheelPlatformKinematics p(params);

    const Eigen::Vector3f wheelVelocities = p.calcWheelVelocity(Eigen::Vector3f{0.0, 0.0, -1.0});

    // all wheels should move at the same speed ...
    BOOST_CHECK_CLOSE(wheelVelocities(0), wheelVelocities(1), 0.1);
    BOOST_CHECK_CLOSE(wheelVelocities(0), wheelVelocities(2), 0.1);

    // ... counter-clockwise
    BOOST_CHECK_LE(wheelVelocities(0), 0);
}

BOOST_AUTO_TEST_SUITE_END()
