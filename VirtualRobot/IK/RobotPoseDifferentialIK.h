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
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2013 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/

#pragma once

#include "DifferentialIK.h"

#include <string>
#include <vector>

namespace VirtualRobot
{
    class [[deprecated("MMMTools_LegacyConverter")]] VIRTUAL_ROBOT_IMPORT_EXPORT RobotPoseDifferentialIK : public DifferentialIK, public std::enable_shared_from_this<RobotPoseDifferentialIK>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
            @brief Initialize a Jacobian object.
            \param rns The robotNodes (i.e., joints) for which the Jacobians should be calculated.
            \param coordSystem The coordinate system in which the Jacobians are defined. By default the global coordinate system is used.
        */
        RobotPoseDifferentialIK(RobotPtr robot,RobotNodeSetPtr rns, RobotNodePtr coordSystem = RobotNodePtr(), JacobiProvider::InverseJacobiMethod invJacMethod = eSVDDamped);

        Eigen::MatrixXf getJacobianMatrix(SceneObjectPtr tcp, IKSolver::CartesianSelection mode) override;

        Eigen::MatrixXf getJacobianMatrix() override;

        /*!	@brief Compute a single IK step.
            @param stepSize Controls the amount of error to be reduced in each step: \f$ 0 < \beta \leq 1 \f$
            @return The changes \f$\Delta \theta\f$ in the joint angles.
            \note{Note} This does not affect the joints values of the robot.
        */
        Eigen::VectorXf computeStep(float stepSize=1.0) override;

        /*!	@brief Computes the complete inverse kinematics.
            @param stepSize Controls the amount of error to be reduced in each step: \f$ 0 < \beta \leq 1 \f$
            @param maxSteps Maximal numbers of steps.
            @param minChange The minimal change in joint angles (euclidean distance in radians)
            @param performMinOneStep If set, at least one step is performed (helps escaping local minima, but may also cause pose jittering)
            @note{Note}  Sets the node's joint angles automatically.
        */
        bool computeSteps(float stepSize, float minChange, int maxSteps, bool performMinOneStep);

        bool computeSteps(float stepSize, float minChange, int maxSteps) override;

        /*!
            If enabled (standard), joint limits are considered via box constraints.
        */
        void boxConstraints(bool enable);

    protected:

        bool checkTolerances() override;

        Eigen::MatrixXd computePseudoInverseJacobianMatrixDampedD(const Eigen::MatrixXd &m);
        std::vector<Eigen::MatrixXf> localJacobians;
        RobotPtr robot;

        bool considerBoxConstraints;

        Eigen::VectorXf _lLimits;
        Eigen::VectorXf _uLimits;
    };

    [[deprecated("MMMTools_LegacyConverter")]] typedef std::shared_ptr<RobotPoseDifferentialIK> RobotPoseDifferentialIKPtr;
}

