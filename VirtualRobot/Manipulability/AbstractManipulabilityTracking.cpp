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
#include "AbstractManipulabilityTracking.h"

#include <Eigen/Dense>

#include <VirtualRobot/Robot.h>

#include "AbstractManipulability.h"

namespace VirtualRobot
{

    std::map<std::string, float>
    AbstractManipulabilityTracking::calculateVelocityMap(
        const Eigen::MatrixXd& manipulabilityDesired,
        const std::vector<std::string>& jointNames,
        const Eigen::MatrixXd& gainMatrix,
        bool jointLimitAvoidance)
    {
        Eigen::VectorXf velocities =
            calculateVelocity(manipulabilityDesired, gainMatrix, jointLimitAvoidance);
        std::map<std::string, float> velocityMap;
        if ((unsigned int)velocities.rows() == jointNames.size())
        {
            for (unsigned int index = 0; index < jointNames.size(); index++)
            {
                velocityMap[jointNames.at(index)] = velocities(index);
            }
        }
        return velocityMap;
    }

    Eigen::Tensor<double, 3>
    AbstractManipulabilityTracking::computeJacobianDerivative(const Eigen::MatrixXd& jacobian)
    {
        int rows = jacobian.rows(); // task space dim
        int columns = jacobian.cols(); // joint space dim

        // Calculate derivative of Jacobian, see "Symbolic differentiation of the velocity mapping for a serial kinematic chain", Bruyninck et al., Mechanism and Machine Theory. 1996
        Eigen::Tensor<double, 3> dJdq(rows, columns, columns);
        dJdq.setZero();
        for (int i = 0; i < columns; i++)
        {
            for (int j = 0; j < columns; j++)
            {
                Eigen::VectorXd J_i = jacobian.col(i);
                Eigen::VectorXd J_j = jacobian.col(j);
                if (j < i)
                {
                    dJdq(0, i, j) = J_j(4) * J_i(2) - J_j(5) * J_i(1);
                    dJdq(1, i, j) = J_j(5) * J_i(0) - J_j(3) * J_i(2);
                    dJdq(2, i, j) = J_j(3) * J_i(1) - J_j(4) * J_i(0);
                    dJdq(3, i, j) = J_j(4) * J_i(5) - J_j(5) * J_i(4);
                    dJdq(4, i, j) = J_j(5) * J_i(3) - J_j(3) * J_i(5);
                    dJdq(5, i, j) = J_j(3) * J_i(4) - J_j(4) * J_i(3);
                }
                else if (j > i)
                {
                    dJdq(0, i, j) = -J_j(1) * J_i(5) + J_j(2) * J_i(4);
                    dJdq(1, i, j) = -J_j(2) * J_i(3) + J_j(0) * J_i(5);
                    dJdq(2, i, j) = -J_j(0) * J_i(4) + J_j(1) * J_i(3);
                }
                else
                {
                    dJdq(0, i, j) = J_i(4) * J_i(2) - J_i(5) * J_i(1);
                    dJdq(1, i, j) = J_i(5) * J_i(0) - J_i(3) * J_i(2);
                    dJdq(2, i, j) = J_i(3) * J_i(1) - J_i(4) * J_i(0);
                }
            }
        }
        return dJdq;
    }

    Eigen::Tensor<double, 3>
    AbstractManipulabilityTracking::tensorMatrixProduct(const Eigen::Tensor<double, 3>& tensor,
                                                        const Eigen::MatrixXd& matrix)
    {
        Eigen::array<long, 2> shape = {tensor.dimension(0),
                                       tensor.dimension(1) * tensor.dimension(2)};
        Eigen::Tensor<double, 2> reshapedTensor = tensor.reshape(shape);
        Eigen::MatrixXd mat_mult = matrix * Tensor_to_Matrix(reshapedTensor,
                                                             reshapedTensor.dimension(0),
                                                             reshapedTensor.dimension(1));
        Eigen::array<long, 3> shape2 = {matrix.rows(), tensor.dimension(1), tensor.dimension(2)};
        return Matrix_to_Tensor(mat_mult, mat_mult.rows(), mat_mult.cols()).reshape(shape2);
    }

    Eigen::Tensor<double, 3>
    AbstractManipulabilityTracking::tensorMatrixProductPermutate(
        const Eigen::Tensor<double, 3>& tensor,
        const Eigen::MatrixXd& matrix)
    {
        Eigen::array<int, 3> shuffling({1, 0, 2}); // Permutation of first two dimensions
        Eigen::Tensor<double, 3> perm_tensor = tensor.shuffle(shuffling);
        Eigen::array<long, 2> shape = {perm_tensor.dimension(0),
                                       perm_tensor.dimension(1) * perm_tensor.dimension(2)};
        Eigen::Tensor<double, 2> reshapedTensor = perm_tensor.reshape(shape);
        Eigen::MatrixXd mat_mult = matrix * Tensor_to_Matrix(reshapedTensor,
                                                             reshapedTensor.dimension(0),
                                                             reshapedTensor.dimension(1));
        Eigen::array<long, 3> shape2 = {
            matrix.rows(), perm_tensor.dimension(1), perm_tensor.dimension(2)};
        return Matrix_to_Tensor(mat_mult, mat_mult.rows(), mat_mult.cols())
            .reshape(shape2)
            .shuffle(shuffling);
    }

    Eigen::Tensor<double, 3>
    AbstractManipulabilityTracking::subCube(Eigen::Tensor<double, 3>& manipulationJacobian)
    {
        switch (getMode())
        {
            case AbstractManipulability::Whole:
                return manipulationJacobian;
            case AbstractManipulability::Position:
            {
                Eigen::array<long, 3> offset = {0, 0, 0};
                Eigen::array<long, 3> extent = {3, 3, manipulationJacobian.dimension(2)};
                return manipulationJacobian.slice(offset, extent);
            }
            case AbstractManipulability::Orientation:
            {
                Eigen::array<long, 3> offset = {3, 3, 0};
                Eigen::array<long, 3> extent = {3, 3, manipulationJacobian.dimension(2)};
                return manipulationJacobian.slice(offset, extent);
            }
            default:
                throw std::runtime_error("Mode not supported");
        }
    }

    Eigen::MatrixXd
    AbstractManipulabilityTracking::getDefaultGainMatrix()
    {
        Eigen::MatrixXd gainMatrix;
        switch (getMode())
        {
            case AbstractManipulability::Whole:
                gainMatrix.setIdentity(21, 21);
                return gainMatrix;
            case AbstractManipulability::Position:
            {
                gainMatrix.setIdentity(6, 6);
                return gainMatrix;
            }
            case AbstractManipulability::Orientation:
            {
                gainMatrix.setIdentity(6, 6);
                return gainMatrix;
            }
            default:
                throw std::runtime_error("Mode not supported");
        }
    }

    Eigen::MatrixXd
    AbstractManipulabilityTracking::logMap(const Eigen::MatrixXd& manipulabilityDesired,
                                           const Eigen::MatrixXd& manipulabilityCurrent)
    {
        Eigen::MatrixXd m = manipulabilityCurrent.inverse() * manipulabilityDesired;
        Eigen::EigenSolver<Eigen::MatrixXd> eigenSolver(m);
        Eigen::MatrixXd eigenVectors = eigenSolver.eigenvectors().real();
        Eigen::MatrixXd diagonalM =
            eigenSolver.eigenvalues().real().array().log().matrix().asDiagonal();
        return manipulabilityCurrent * eigenVectors * diagonalM * eigenVectors.inverse();
    }

    double
    AbstractManipulabilityTracking::computeDistance(const Eigen::MatrixXd& manipulabilityDesired)
    {
        return logMap(manipulabilityDesired, computeCurrentManipulability()).norm();
    }

    Eigen::MatrixXd
    AbstractManipulabilityTracking::getJointsLimitsWeightMatrix(
        const Eigen::VectorXd& jointAngles,
        const Eigen::VectorXd& jointLimitsLow,
        const Eigen::VectorXd& jointLimitsHigh)
    {
        int nbJoints = jointAngles.size();
        Eigen::VectorXd weights(nbJoints);
        Eigen::VectorXd gradient = Eigen::VectorXd::Zero(nbJoints);

        if (jointAngleLimitGradient.rows() == 0)
        {
            jointAngleLimitGradient = Eigen::VectorXd::Zero(nbJoints);
        }

        //std::cout << "Saved gradient:\n" << jointAngleLimitGradient << std::endl;

        for (int i = 0; i < nbJoints; i++)
        {
            if (jointLimitsLow(i) == 0 && jointLimitsHigh(i) == 0)
            {
                // Joint is limitless
                weights(i) = 1.;
            }
            else
            {
                gradient(i) = std::pow(jointLimitsHigh(i) - jointLimitsLow(i), 2) *
                              (2 * jointAngles(i) - jointLimitsHigh(i) - jointLimitsLow(i)) /
                              (4 * std::pow(jointLimitsHigh(i) - jointAngles(i), 2) *
                               std::pow(jointAngles(i) - jointLimitsLow(i), 2));

                if ((std::abs(gradient(i)) - std::abs(jointAngleLimitGradient(i))) >= 0.)
                {
                    // Joint going towards limits
                    weights(i) = 1. / std::sqrt((1. + std::abs(gradient(i))));
                }
                else
                {
                    // Joint going towards the center
                    weights(i) = 1.;
                }
            }
        }
        //std::cout << "Gradient:\n" << gradient << "\n\n" << std::endl;
        setjointAngleLimitGradient(gradient);

        Eigen::MatrixXd weightsMatrix = weights.asDiagonal();
        return weightsMatrix;
    }

    Eigen::MatrixXd
    AbstractManipulabilityTracking::computeManipulabilityJacobianMandelNotation(
        const Eigen::Tensor<double, 3>& manipulabilityJacobian)
    {
        int num_task_vars = getTaskVars();

        Eigen::MatrixXd jm_red((num_task_vars * num_task_vars + num_task_vars) / 2,
                               manipulabilityJacobian.dimension(2));
        jm_red.setZero();

        for (int i = 0; i < manipulabilityJacobian.dimension(2); i++)
        {
            Eigen::Tensor<double, 2> tensor = manipulabilityJacobian.chip<2>(i);
            Eigen::MatrixXd sym_matrix = Tensor_to_Matrix(tensor, num_task_vars, num_task_vars);
            jm_red.col(i) = symMatrixToVector(sym_matrix);
        }
        return jm_red;
    }

    Eigen::VectorXd
    AbstractManipulabilityTracking::symMatrixToVector(const Eigen::MatrixXd& sym_matrix)
    {
        int num_task_vars = sym_matrix.rows();
        Eigen::VectorXd v((num_task_vars * num_task_vars + num_task_vars) / 2);
        // sym matrix to Mandel notation vector
        Eigen::VectorXd diagonal = sym_matrix.diagonal();
        int diagSize = diagonal.size();
        int nextIndex = 0;
        v.block(nextIndex, 0, diagSize, 1) = diagonal;
        nextIndex += diagSize;
        double scalar = std::pow(2, 0.5);
        for (int i = 1; i < sym_matrix.rows(); i++)
        {
            Eigen::VectorXd diagonal_i = sym_matrix.diagonal(i);
            diagSize = diagonal_i.size();
            v.block(nextIndex, 0, diagonal_i.size(), 1) = scalar * diagonal_i;
            nextIndex += diagonal_i.size();
        }
        return v;
    }

    void
    AbstractManipulabilityTracking::setjointAngleLimitGradient(const Eigen::VectorXd& gradient)
    {
        this->jointAngleLimitGradient = gradient;
    }

} // namespace VirtualRobot
