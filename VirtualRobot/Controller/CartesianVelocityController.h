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
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @author      ()
* @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
*             GNU General Public License
*/

#pragma once

#include "VirtualRobot/IK/JacobiProvider.h"
#include "VirtualRobot/VirtualRobot.h"

namespace VirtualRobot
{
    class CartesianVelocityController;
    using CartesianVelocityControllerPtr = std::shared_ptr<CartesianVelocityController>;

    class CartesianVelocityController
    {
    public:
        CartesianVelocityController(
            const RobotNodeSetPtr& rns,
            const RobotNodePtr& tcp = nullptr,
            const JacobiProvider::InverseJacobiMethod invJacMethod = JacobiProvider::eSVDDamped,
            bool _considerJointLimits = true);

        CartesianVelocityController(CartesianVelocityController&&) = default;
        CartesianVelocityController& operator=(CartesianVelocityController&&) = default;

        Eigen::VectorXf calculate(const Eigen::VectorXf& cartesianVel,
                                  IKSolver::CartesianSelection mode);
        Eigen::VectorXf calculate(const Eigen::VectorXf& cartesianVel,
                                  float KpJointLimitAvoidanceScale,
                                  IKSolver::CartesianSelection mode);
        Eigen::VectorXf calculate(const Eigen::VectorXf& cartesianVel,
                                  const Eigen::VectorXf& nullspaceVel,
                                  IKSolver::CartesianSelection mode);
        Eigen::VectorXf calculateJointLimitAvoidance();
        Eigen::VectorXf calculateJointLimitAvoidanceWithMargins(const Eigen::VectorXf& margins);
        Eigen::VectorXf calculateNullspaceVelocity(const Eigen::VectorXf& cartesianVel,
                                                   float KpScale,
                                                   IKSolver::CartesianSelection mode);

        void setCartesianRegularization(float cartesianMMRegularization,
                                        float cartesianRadianRegularization);

        bool getConsiderJointLimits() const;
        void setConsiderJointLimits(bool value);

        Eigen::MatrixXf jacobi;
        RobotNodeSetPtr rns;
        DifferentialIKPtr ik;
        RobotNodePtr _tcp;
        Eigen::VectorXf maximumJointVelocities;

        void setJointCosts(const std::vector<float>& _jointCosts);
        Eigen::VectorXf calculateRegularization(VirtualRobot::IKSolver::CartesianSelection mode);

    private:
        void calculateJacobis(IKSolver::CartesianSelection mode);
        Eigen::MatrixXf _jacobiWithCosts;
        Eigen::MatrixXf _inv;
        bool clampJacobiAtJointLimits(IKSolver::CartesianSelection mode,
                                      const Eigen::VectorXf& cartesianVel,
                                      Eigen::MatrixXf& jacobi,
                                      Eigen::MatrixXf& _inv,
                                      float jointLimitCheckAccuracy = 0.001f);
        bool _considerJointLimits = true;
        float _cartesianMMRegularization;
        float _cartesianRadianRegularization;
        Eigen::VectorXf _jointCosts;
    };
} // namespace VirtualRobot
