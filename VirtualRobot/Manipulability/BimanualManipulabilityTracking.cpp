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
* @author     Andre Meixner
* @author     Noémie Jaquier
* @copyright  2021 Andre Meixner
*             GNU Lesser General Public License
*
*/
#include "BimanualManipulabilityTracking.h"
#include <iostream>

#include <SimoxUtility/math/convert/pos_quat_to_mat4f.h>
#include <VirtualRobot/MathTools.h>

#include "BimanualManipulability.h"

namespace VirtualRobot
{

    BimanualManipulabilityTracking::BimanualManipulabilityTracking(
        BimanualManipulabilityPtr manipulability) :
        manipulability(manipulability)
    {
    }

    Eigen::VectorXf
    BimanualManipulabilityTracking::calculateVelocity(const Eigen::MatrixXd& manipulabilityDesired,
                                                      const Eigen::MatrixXd& gainMatrix,
                                                      bool jointLimitAvoidance)
    {
        // Compute left and right Jacobians
        Eigen::MatrixXd jacobianLeft = manipulability->computeFullJacobianLeft();
        Eigen::MatrixXd jacobianRight = manipulability->computeFullJacobianRight();

        // Compute bimanual Jacobian
        Eigen::MatrixXd jacobian =
            manipulability->computeBimanualJacobian(jacobianLeft, jacobianRight);

        // Compute grasp matrix
        Eigen::MatrixXd bimanualGraspMatrix = manipulability->computeBimanualGraspMatrix();

        // Compute manipulability
        Eigen::MatrixXd manipulabilityCurrent = manipulability->getManipulabilitySubMatrix(
            manipulability->computeBimanualManipulability(jacobian, bimanualGraspMatrix));

        Eigen::MatrixXd manipulability_velocity =
            logMap(manipulabilityDesired, manipulabilityCurrent);
        Eigen::MatrixXd manipulability_mandel = symMatrixToVector(manipulability_velocity);

        // Compute manipulability Jacobian
        Eigen::Tensor<double, 3> manipJ =
            computeBimanualManipulabilityJacobian(jacobianLeft, jacobianRight, bimanualGraspMatrix);
        Eigen::Tensor<double, 3> manipJ_sub = subCube(manipJ);
        Eigen::MatrixXd matManipJ = computeManipulabilityJacobianMandelNotation(manipJ_sub);

        if (jointLimitAvoidance)
        {
            // TODO
            std::cout << "Joint limit avoidance for bimanual manipulability is not yet implemented!"
                      << std::endl;
        }

        // Compute damped pseudo-inverse of manipulability Jacobian
        double dampingFactor = MathTools::getDamping(matManipJ);
        Eigen::MatrixXd dampedLSI =
            MathTools::getDampedLeastSquareInverse(matManipJ, dampingFactor);

        // Compute joint velocity
        Eigen::VectorXd dq = dampedLSI *
                             (gainMatrix.rows() == 0 ? getDefaultGainMatrix() : gainMatrix) *
                             manipulability_mandel;
        return dq.cast<float>();
    }

    Eigen::Tensor<double, 3>
    BimanualManipulabilityTracking::computeBimanualManipulabilityJacobian(
        const Eigen::MatrixXd& jacobianLeft,
        const Eigen::MatrixXd& jacobianRight,
        const Eigen::MatrixXd& bimanualGraspMatrix)
    {
        // Jacobians dimensions
        int nbVarsLeft = jacobianLeft.rows(); // task space dim
        int nbDofsLeft = jacobianLeft.cols(); // joint space dim
        int nbVarsRight = jacobianRight.rows(); // task space dim
        int nbDofsRight = jacobianRight.cols(); // joint space dim
        int nbVars = nbVarsLeft + nbVarsRight;
        int nbDofs = nbDofsLeft + nbDofsRight;

        // Compute bimanual jacobian
        Eigen::MatrixXd bimanualJacobian =
            manipulability->computeBimanualJacobian(jacobianLeft, jacobianRight);

        // Calculate derivative of Jacobian, see "Symbolic differentiation of the velocity mapping for a serial kinematic chain", Bruyninck et al., Mechanism and Machine Theory. 1996
        Eigen::Tensor<double, 3> dJdq(nbVars, nbDofs, nbDofs);
        dJdq = computeBimanualJacobianDerivative(jacobianLeft, jacobianRight);

        // Compute pseudo-inverse of grasp matrix
        Eigen::MatrixXd pinverseBimanualGraspMatrix =
            bimanualGraspMatrix.transpose() *
            (bimanualGraspMatrix * bimanualGraspMatrix.transpose()).inverse();

        // Permutation of first two dimensions
        Eigen::array<int, 3> shuffling({1, 0, 2}); // Permutation of first two dimensions
        Eigen::Tensor<double, 3> permdJdq = dJdq.shuffle(shuffling);

        // Compute manipulability jacobian
        Eigen::Tensor<double, 3> manipJ(nbVars / 2, nbVars / 2, nbDofs);
        manipJ = tensorMatrixProductPermutate(
                     tensorMatrixProduct(dJdq, pinverseBimanualGraspMatrix.transpose()),
                     pinverseBimanualGraspMatrix.transpose() * bimanualJacobian) +
                 tensorMatrixProductPermutate(
                     tensorMatrixProduct(
                         permdJdq, pinverseBimanualGraspMatrix.transpose() * bimanualJacobian),
                     pinverseBimanualGraspMatrix.transpose());

        switch (manipulability->getType())
        {
            case AbstractManipulability::Velocity:
                return manipJ;
            case AbstractManipulability::Force:
            {
                Eigen::MatrixXd kinematicManipulability =
                    manipulability->computeBimanualManipulability(
                        bimanualJacobian, bimanualGraspMatrix, AbstractManipulability::Velocity);
                return tensorMatrixProductPermutate(
                    tensorMatrixProduct(-manipJ, kinematicManipulability), kinematicManipulability);
            }
            default:
                throw std::runtime_error("Unknown manipulability type");
        }
    }

    Eigen::Tensor<double, 3>
    BimanualManipulabilityTracking::computeBimanualJacobianDerivative(
        const Eigen::MatrixXd& jacobianLeft,
        const Eigen::MatrixXd& jacobianRight)
    {
        // Jacobians dimensions
        int nbVarsLeft = jacobianLeft.rows(); // task space dim
        int nbDofsLeft = jacobianLeft.cols(); // joint space dim
        int nbVarsRight = jacobianRight.rows(); // task space dim
        int nbDofsRight = jacobianRight.cols(); // joint space dim

        // Compute the derivative of each Jacobian
        Eigen::Tensor<double, 3> dJdqLeft = computeJacobianDerivative(jacobianLeft);
        Eigen::Tensor<double, 3> dJdqRight = computeJacobianDerivative(jacobianRight);

        // Build the a block-diagonal tensor out of the two Jacobian derivatives
        Eigen::Tensor<double, 3> dJdqBimanual(
            nbVarsLeft + nbVarsRight, nbDofsLeft + nbDofsRight, nbDofsLeft + nbDofsRight);
        dJdqBimanual.setZero();
        Eigen::array<long, 3> offset = {0, 0, 0};
        Eigen::array<long, 3> extent = {nbVarsLeft, nbDofsLeft, nbDofsLeft};
        dJdqBimanual.slice(offset, extent) = dJdqLeft;
        offset = {nbVarsLeft, nbDofsLeft, nbDofsLeft};
        extent = {nbVarsRight, nbDofsRight, nbDofsRight};
        dJdqBimanual.slice(offset, extent) = dJdqRight;

        return dJdqBimanual;
    }

    int
    BimanualManipulabilityTracking::getTaskVars()
    {
        return manipulability->getTaskVars();
    }

    AbstractManipulability::Mode
    BimanualManipulabilityTracking::getMode()
    {
        return manipulability->getMode();
    }

    BimanualManipulabilityPtr
    BimanualManipulabilityTracking::getManipulability()
    {
        return manipulability;
    }

    Eigen::MatrixXd
    BimanualManipulabilityTracking::computeCurrentManipulability()
    {
        return manipulability->computeManipulability();
    }

    std::vector<std::string>
    BimanualManipulabilityTracking::getJointNames()
    {
        return manipulability->getJointNames();
    }

    VisualizationNodePtr
    BimanualManipulabilityTracking::getManipulabilityVis(const Eigen::MatrixXd& manipulability,
                                                         const std::string& visualizationType,
                                                         double scaling)
    {
        return this->manipulability->getManipulabilityVis(
            manipulability, visualizationType, scaling);
    }

    void
    BimanualManipulabilityTracking::setConvertMMtoM(bool value)
    {
        manipulability->setConvertMMtoM(value);
    }

} // namespace VirtualRobot
