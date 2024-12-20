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
#include "SingleChainManipulabilityTracking.h"

#include <VirtualRobot/MathTools.h>

#include "SingleChainManipulability.h"

namespace VirtualRobot
{

    SingleChainManipulabilityTracking::SingleChainManipulabilityTracking(
        AbstractSingleChainManipulabilityPtr manipulability) :
        manipulability(manipulability)
    {
        // Initialize the jointAngleLimitGradient vector
        Eigen::MatrixXd jacobian = manipulability->computeJacobian();
        int nbJoints = jacobian.cols();
        Eigen::VectorXd gradient(nbJoints);
        gradient.setZero();
        jointAngleLimitGradient = gradient;
    }

    Eigen::Tensor<double, 3>
    SingleChainManipulabilityTracking::computeManipulabilityJacobian(
        const Eigen::MatrixXd& jacobian)
    {
        int rows = jacobian.rows(); // task space dim
        int columns = jacobian.cols(); // joint space dim

        Eigen::Tensor<double, 3> derivJ = computeJacobianDerivative(jacobian);

        Eigen::array<int, 3> shuffling({1, 0, 2}); // Permutation of first two dimensions
        Eigen::Tensor<double, 3> permDerivJ = derivJ.shuffle(shuffling);

        // Define default weight matrix
        if (manipulability->weightMatrix.rows() == 0)
        {
            manipulability->weightMatrix.setIdentity(columns, columns);
        }

        // Compute dJ/dq x_2 J
        Eigen::Tensor<double, 3> dJtdq_J = tensorMatrixProduct(
            permDerivJ,
            jacobian * manipulability->weightMatrix * manipulability->weightMatrix.transpose());

        // Compute dJ'/dq x_1 J + dJ/dq x_2 J
        Eigen::array<int, 2> shufflingTranpose({1, 0}); // for transposing
        Eigen::Tensor<double, 3> manipJ(rows, rows, columns);
        for (int s = 0; s < columns; ++s)
        {
            Eigen::Tensor<double, 2> a = dJtdq_J.chip(s, 2);
            Eigen::Tensor<double, 2> b = dJtdq_J.chip(s, 2);
            Eigen::Tensor<double, 2> b_trans = b.shuffle(shufflingTranpose);
            manipJ.chip(s, 2) = a + b_trans;
        }

        switch (manipulability->getType())
        {
            case AbstractManipulability::Velocity:
                return manipJ;
            case AbstractManipulability::Force:
            {
                Eigen::MatrixXd kinematicManipulability = manipulability->computeManipulability(
                    jacobian, AbstractManipulability::Velocity);
                return tensorMatrixProductPermutate(
                    tensorMatrixProduct(-manipJ, kinematicManipulability), kinematicManipulability);
            }
            default:
                throw std::runtime_error("Unknown manipulability type");
        }
    }

    Eigen::VectorXf
    SingleChainManipulabilityTracking::calculateVelocity(
        const Eigen::MatrixXd& manipulabilityDesired,
        const Eigen::MatrixXd& gainMatrix,
        bool jointLimitAvoidance)
    {
        Eigen::MatrixXd jacobian =
            manipulability->computeFullJacobian(); // full jacobian is required for derivative
        Eigen::MatrixXd jacobian_sub = manipulability->getJacobianSubMatrix(jacobian);
        Eigen::MatrixXd manipulabilityCurrent = manipulability->computeManipulability(jacobian_sub);
        Eigen::MatrixXd manipulability_velocity =
            logMap(manipulabilityDesired, manipulabilityCurrent);
        Eigen::Tensor<double, 3> manipJ = computeManipulabilityJacobian(jacobian);
        Eigen::Tensor<double, 3> manipJ_sub = subCube(manipJ);

        Eigen::MatrixXd matManipJ = computeManipulabilityJacobianMandelNotation(manipJ_sub);
        Eigen::MatrixXd dampedLSI;
        if (jointLimitAvoidance)
        {
            // Compute weighted manipulability Jacobian
            Eigen::MatrixXd jointsWeightMatrix =
                getJointsLimitsWeightMatrix(manipulability->getJointAngles(),
                                            manipulability->getJointLimitsLow(),
                                            manipulability->getJointLimitsHigh());
            matManipJ = matManipJ * jointsWeightMatrix;
            // Compute pseudo-inverse of weighted manipulability Jacobian
            double dampingFactor = MathTools::getDamping(matManipJ);
            dampedLSI = jointsWeightMatrix *
                        MathTools::getDampedLeastSquareInverse(matManipJ, dampingFactor);
        }
        else
        {
            // Compute pseudo-inverse of manipulability Jacobian
            double dampingFactor = MathTools::getDamping(matManipJ);
            dampedLSI = MathTools::getDampedLeastSquareInverse(matManipJ, dampingFactor);
        }

        Eigen::MatrixXd manipulability_mandel = symMatrixToVector(manipulability_velocity);

        Eigen::VectorXd dq = dampedLSI *
                             (gainMatrix.rows() == 0 ? getDefaultGainMatrix() : gainMatrix) *
                             manipulability_mandel;
        return dq.cast<float>();
    }

    int
    SingleChainManipulabilityTracking::getTaskVars()
    {
        return manipulability->getTaskVars();
    }

    AbstractManipulability::Mode
    SingleChainManipulabilityTracking::getMode()
    {
        return manipulability->getMode();
    }

    AbstractSingleChainManipulabilityPtr
    SingleChainManipulabilityTracking::getManipulability()
    {
        return manipulability;
    }

    Eigen::MatrixXd
    SingleChainManipulabilityTracking::computeCurrentManipulability()
    {
        return manipulability->computeManipulability();
    }

    std::vector<std::string>
    SingleChainManipulabilityTracking::getJointNames()
    {
        return manipulability->getJointNames();
    }

    VisualizationNodePtr
    SingleChainManipulabilityTracking::getManipulabilityVis(const Eigen::MatrixXd& manipulability,
                                                            const std::string& visualizationType,
                                                            double scaling)
    {
        return this->manipulability->getManipulabilityVis(
            manipulability, visualizationType, scaling);
    }

    void
    SingleChainManipulabilityTracking::setConvertMMtoM(bool value)
    {
        manipulability->setConvertMMtoM(value);
    }

} // namespace VirtualRobot
