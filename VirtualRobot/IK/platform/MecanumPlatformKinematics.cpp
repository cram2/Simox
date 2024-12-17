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


#include "MecanumPlatformKinematics.h"

namespace VirtualRobot
{
    Eigen::Matrix<float, 4, 3>
    MecanumPlatformKinematicsParams::J_inv() const
    {
        const float k = l1 + l2;

        Eigen::Matrix<float, 4, 3> m;
        // clang-format off
        m <<  1,  1, -k,
             -1,  1,  k,
             -1,  1, -k,
              1,  1,  k;
        // clang-format on

        m *= 1 / R;

        return m;
    }

    Eigen::Matrix<float, 3, 4>
    MecanumPlatformKinematicsParams::J() const
    {
        const float k = 1 / (l1 + l2);

        Eigen::Matrix<float, 3, 4> m;

        // according to the paper
        // m << -1,  1,  1, -1,
        //       1,  1,  1,  1,
        //      -k,  k, -k,  k;

        // "ours"
        // clang-format off
        m <<  1, -1, -1,  1,
              1,  1,  1,  1,
             -k,  k, -k,  k;
        // clang-format on

        m *= R / 4;

        return m;
    }

    MecanumPlatformKinematics::MecanumPlatformKinematics(const Params& params) :
        params(params), J(params.J()), J_inv(params.J_inv())
    {
    }

    MecanumPlatformKinematics::WheelVelocities
    MecanumPlatformKinematics::calcWheelVelocity(const CartesianVelocity& v) const
    {
        return J_inv * v;
    }

    MecanumPlatformKinematics::CartesianVelocity
    MecanumPlatformKinematics::calcCartesianVelocity(const WheelVelocities& w) const
    {
        return J * w;
    }

    const MecanumPlatformKinematics::Params&
    MecanumPlatformKinematics::getParams() const
    {
        return params;
    }

} // namespace VirtualRobot
