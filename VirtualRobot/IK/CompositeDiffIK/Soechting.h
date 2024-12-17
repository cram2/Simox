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
* @author     André Meixner (andre dot meixner at kit dot edu)
* @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
*             GNU General Public License
*/

#pragma once

#include <Eigen/Geometry>

#include <VirtualRobot/MathTools.h>

namespace VirtualRobot
{
    struct Soechting
    {
        enum ArmType
        {
            Right = 0,
            Left = 1
        };

        struct Arm
        {
            Eigen::Vector3f shoulder;
            float upperArmLength;
            float forearmLength;

            ArmType armType;
        };

        struct SoechtingAngles
        {
            float SE;
            float SY;
            float EE;
            float EY;
        };

        struct ForwardPositions
        {
            Eigen::Vector3f shoulder;
            Eigen::Vector3f elbow;
            Eigen::Vector3f wrist;
        };

        static SoechtingAngles
        CalculateAngles(const Eigen::Vector3f& target,
                        const Arm& arm,
                        float scale = 1.0f,
                        bool accuratePointing = false)
        {
            Eigen::Vector3f soechtingTarget = ((target - arm.shoulder) / scale);
            soechtingTarget /= 10.0f; // Soechting is defined in cm

            if (arm.armType == ArmType::Left)
            {
                soechtingTarget(0) *= -1;
            }

            // spherical coordinates: distance, azimuth, elevation
            const float R = soechtingTarget.norm();
            const float Chi =
                MathTools::rad2deg(std::atan2(soechtingTarget.x(), soechtingTarget.y()));
            const float Psi = MathTools::rad2deg(
                std::atan2(soechtingTarget.z(), soechtingTarget.head<2>().norm()));

            //  Kondo, Koichi. “Inverse Kinematics of a Human Arm,” n.d., 16.
            SoechtingAngles sa;
            if (accuratePointing)
            {
                // Angles derived from accurate pointing
                sa.SE = -6.7 + 1.09 * R + 1.10 * Psi;
                sa.EE = 47.6 + 0.33 * R - 0.95 * Psi;
                sa.EY = -11.5 + 1.27 * Chi - 0.54 * Psi;
                sa.SY = 67.7 + 1.00 * Chi - 0.68 * R;
            }
            else
            {
                // Angles derived from pointing in the dark
                sa.SE = -4.0 + 1.10 * R + 0.90 * Psi;
                sa.EE = 39.4 + 0.54 * R - 1.06 * Psi;
                sa.SY = 13.2 + 0.86 * Chi + 0.11 * Psi;
                sa.EY = -10.0 + 1.08 * Chi - 0.35 * Psi;
            }

            // convert to radian
            sa.SE = MathTools::deg2rad(sa.SE);
            sa.SY = MathTools::deg2rad(sa.SY);
            sa.EE = MathTools::deg2rad(sa.EE);
            sa.EY = MathTools::deg2rad(sa.EY);

            if (arm.armType == ArmType::Left)
            {
                sa.SY = -sa.SY;
                sa.EY = -sa.EY;
            }

            return sa;
        }

        static ForwardPositions
        CalculateForwardPositions(const SoechtingAngles& sa, const Arm& arm)
        {
            const Eigen::AngleAxisf aaSE(sa.SE, Eigen::Vector3f::UnitX());
            const Eigen::AngleAxisf aaSY(-sa.SY, Eigen::Vector3f::UnitZ());
            const Eigen::AngleAxisf aaEE(-sa.EE, Eigen::Vector3f::UnitX());
            const Eigen::AngleAxisf aaEY(-sa.EY, Eigen::Vector3f::UnitZ());

            const Eigen::Vector3f elb =
                aaSY * aaSE * -Eigen::Vector3f::UnitZ() * arm.upperArmLength;
            const Eigen::Vector3f wri = aaEY * aaEE * Eigen::Vector3f::UnitZ() * arm.forearmLength;

            ForwardPositions res;
            res.shoulder = arm.shoulder;
            res.elbow = arm.shoulder + elb;
            res.wrist = arm.shoulder + elb + wri;

            return res;
        }
    };

} // namespace VirtualRobot
