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


#pragma once

#include <eigen3/Eigen/Core>

namespace VirtualRobot
{

    /**
     * @brief The parameters to define the forward and inverse model of the omni wheel model
     *   
     * See:  
     *   
     * Liu, Yong, Xiaofei Wu, J. Jim Zhu, and Jae Lew. “Omni-Directional Mobile Robot Controller Design by Trajectory Linearization.” 
     * In Proceedings of the 2003 American Control Conference, 2003., 4:3423–28 vol.4, 2003. https://doi.org/10.1109/ACC.2003.1244061.
     *
     * The variable names are consistent with the paper.
     *
     */
    struct OmniWheelPlatformKinematicsParams
    {
        //! radius of the body
        float L;

        //! wheel radius
        float R;

        //! angular position of the first wheel
        float delta;

        //! gear ratio
        float n;

        //! rotation to reference
        // Rotation around platform center to define which direction is front
        // For ARMAR-7, this is 60 degrees (between the two "front" wheels).
        float relativeAngle;

        Eigen::Vector3f wheelFactor;

        // see Liu et al., formula 2.2.2
        Eigen::Matrix3f B() const;

        // forward model, similar to Jacobian matrix, see Liu et al., formula 2.2.2
        Eigen::Matrix3f C() const;
    };

    /**
     * @brief The kinematic model of the omni wheel platform model
     *
     * See:  
     *   
     * Liu, Yong, Xiaofei Wu, J. Jim Zhu, and Jae Lew. “Omni-Directional Mobile Robot Controller Design by Trajectory Linearization.” 
     * In Proceedings of the 2003 American Control Conference, 2003., 4:3423–28 vol.4, 2003. https://doi.org/10.1109/ACC.2003.1244061.
     *
     */
    class OmniWheelPlatformKinematics
    {
    public:
        using Params = OmniWheelPlatformKinematicsParams;

        //! [w_m1, w_m2, w_m3]
        using WheelVelocities = Eigen::Vector3f;

        //! [u, v, r]
        using CartesianVelocity = Eigen::Vector3f;

        OmniWheelPlatformKinematics(const Params& info);

        //! inverse model
        WheelVelocities calcWheelVelocity(const CartesianVelocity& v) const;

        //! forward model
        CartesianVelocity calcCartesianVelocity(const WheelVelocities& w) const;

        const Params& getParams() const;

    private:
        const Params params;

        const Eigen::Matrix3f C;
        const Eigen::Matrix3f C_inv;
    };

} // namespace VirtualRobot
