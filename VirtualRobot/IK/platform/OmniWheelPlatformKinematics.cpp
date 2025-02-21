/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* @author     Fabian Reister (fabian dot reister at kit dot edu)
* @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
*             GNU General Public License
*/

#include "OmniWheelPlatformKinematics.h"

#include <cmath>

#include <eigen3/Eigen/Core>
#include <Eigen/Dense>

#include "VirtualRobot.h"

namespace VirtualRobot
{
    // OmniWheelPlatformKinematicsParams

    Eigen::Matrix3f
    OmniWheelPlatformKinematicsParams::B() const
    {
        Eigen::Matrix3f b;

        // clang-format off
            b << 1.F, -std::sin(delta), -std::sin(delta), 
                 0.F,  std::cos(delta), -std::cos(delta), 
                 L, L, L;
        // clang-format on

        return b;
    }

    Eigen::Matrix3f
    OmniWheelPlatformKinematicsParams::C() const
    {
        // Rotation around platform center to define which direction is front
        Eigen::Matrix3f relativeRotation = Eigen::Matrix3f::Identity();
        relativeRotation.block<2, 2>(0, 0) = Eigen::Rotation2Df(relativeAngle).toRotationMatrix();

        return relativeRotation * wheelFactor.asDiagonal() * B().transpose().inverse() * 2 * M_PI *
               R / n;
    }

    // OmniWheelPlatformKinematics

    OmniWheelPlatformKinematics::OmniWheelPlatformKinematics(const Params& params) :
        params(params), C(params.C()), C_inv(C.inverse())
    {
    }

    OmniWheelPlatformKinematics::WheelVelocities
    OmniWheelPlatformKinematics::calcWheelVelocity(const CartesianVelocity& v) const
    {
        return C_inv * v;
    }

    OmniWheelPlatformKinematics::CartesianVelocity
    OmniWheelPlatformKinematics::calcCartesianVelocity(const WheelVelocities& w) const
    {
        return C * w;
    }

    const OmniWheelPlatformKinematics::Params&
    OmniWheelPlatformKinematics::getParams() const
    {
        return params;
    }

} // namespace VirtualRobot
