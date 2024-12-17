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
* @author     Simon Ottenhaus ( simon dot ottenhaus at kit dot edu )
* @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
*             GNU General Public License
*/


#pragma once

#include <Eigen/Core>

namespace VirtualRobot
{

    /**
     * @brief The parameters to define the forward and inverse model of the mecanum model
     *   
     * See:
     *
     * Doroftei, Ioan, Victor Grosu, and Veaceslav Spinu. "Omnidirectional Mobile Robot - Design and Implementation."
     * Bioinspiration and Robotics Walking and Climbing Robots. IntechOpen, 2007. https://doi.org/10.5772/5518.
     * 
     * The variable names are consistent with the paper.
     *
     * Note:
     *    In the paper, the x-axis is pointing forwards. Here, y is pointing forwards.
     */
    struct MecanumPlatformKinematicsParams
    {
        //! gauge
        float l1;

        //! wheelbase
        float l2;

        //! wheel radius
        float R;

        // See Doroftei et al., formula 9
        Eigen::Matrix<float, 3, 4> J() const;

        // See Doroftei et al., formula 8
        Eigen::Matrix<float, 4, 3> J_inv() const;
    };

    /**
     * @brief The kinematic model of the mecanum platform model
     *     
     * See:
     *
     * Doroftei, Ioan, Victor Grosu, and Veaceslav Spinu. "Omnidirectional Mobile Robot - Design and Implementation."
     * Bioinspiration and Robotics Walking and Climbing Robots. IntechOpen, 2007. https://doi.org/10.5772/5518.
     *
     * The variable names are consistent with the paper.
     *
     *
     * Wheel definition:
     *
     * - wheel 0: left front
     * - wheel 1: right front
     * - wheel 2: rear left
     * - wheel 3: rear right
     */
    class MecanumPlatformKinematics
    {
    public:
        using Params = MecanumPlatformKinematicsParams;

        using WheelVelocities = Eigen::Vector4f;

        //! \f$ [\dot{x}, \dot{y}, \dot{yaw}] \f$
        using CartesianVelocity = Eigen::Vector3f;

        MecanumPlatformKinematics(const Params& params);

        //! inverse model
        WheelVelocities calcWheelVelocity(const CartesianVelocity& v) const;

        //! forward model
        CartesianVelocity calcCartesianVelocity(const WheelVelocities& w) const;

        const Params& getParams() const;

    private:
        const Params params;

        const Eigen::Matrix<float, 3, 4> J;
        const Eigen::Matrix<float, 4, 3> J_inv;
    };

} // namespace VirtualRobot
