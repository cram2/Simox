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
* @author     Simon Ottenhaus (simon dot ottenhaus at kit dot edu)
* @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
*             GNU General Public License
*/

#pragma once

#include "VirtualRobot/IK/IKSolver.h"
#include "VirtualRobot/VirtualRobot.h"

namespace VirtualRobot
{
    class CartesianPositionController;
    using CartesianPositionControllerPtr = std::shared_ptr<CartesianPositionController>;

    // This is a P-controller. In: Target pose, out Cartesian velocity.
    class CartesianPositionController
    {
    public:
        CartesianPositionController(const RobotNodePtr& tcp);

        CartesianPositionController(CartesianPositionController&&) = default;
        CartesianPositionController& operator=(CartesianPositionController&&) = default;

        Eigen::VectorXf calculate(const Eigen::Matrix4f& targetPose,
                                  IKSolver::CartesianSelection mode) const;
        Eigen::VectorXf calculatePos(const Eigen::Vector3f& targetPos) const;

        float getPositionError(const Eigen::Matrix4f& targetPose) const;
        float getOrientationError(const Eigen::Matrix4f& targetPose) const;
        static float GetPositionError(const Eigen::Matrix4f& targetPose, const RobotNodePtr& tcp);
        static float GetOrientationError(const Eigen::Matrix4f& targetPose,
                                         const RobotNodePtr& tcp);
        static bool Reached(const Eigen::Matrix4f& targetPose,
                            const RobotNodePtr& tcp,
                            bool checkOri,
                            float thresholdPosReached,
                            float thresholdOriReached);
        bool reached(const Eigen::Matrix4f& targetPose,
                     VirtualRobot::IKSolver::CartesianSelection mode,
                     float thresholdPosReached,
                     float thresholdOriReached);

        Eigen::Vector3f getPositionDiff(const Eigen::Matrix4f& targetPose) const;
        Eigen::Vector3f getPositionDiffVec3(const Eigen::Vector3f& targetPosition) const;
        Eigen::Vector3f getOrientationDiff(const Eigen::Matrix4f& targetPose) const;
        RobotNodePtr getTcp() const;

        float KpPos = 1;
        float KpOri = 1;
        float maxPosVel = -1;
        float maxOriVel = -1;


    private:
        RobotNodePtr tcp;
    };
} // namespace VirtualRobot
