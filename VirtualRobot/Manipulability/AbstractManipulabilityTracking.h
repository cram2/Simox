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
#pragma once

#include <unsupported/Eigen/CXX11/Tensor>
#include <map>
#include <memory>
#include "AbstractManipulability.h"

namespace VirtualRobot {

template<typename T>
using  MatrixType = Eigen::Matrix<T,Eigen::Dynamic, Eigen::Dynamic>;

template<typename Scalar,int rank, typename sizeType>
auto Tensor_to_Matrix(const Eigen::Tensor<Scalar,rank> &tensor,const sizeType rows,const sizeType cols)
{
    return Eigen::Map<const MatrixType<Scalar>> (tensor.data(), rows,cols);
}

template<typename Scalar, typename... Dims>
auto Matrix_to_Tensor(const MatrixType<Scalar> &matrix, Dims... dims)
{
    constexpr int rank = sizeof... (Dims);
    return Eigen::TensorMap<Eigen::Tensor<const Scalar, rank>>(matrix.data(), {dims...});
}

/**
 * Implementation of manipulability tracking, see
 * N. Jaquier, L. Rozo, D. G. Caldwell and S. Calinon. Geometry-aware Tracking of Manipulability Ellipsoids, in Robotics: Science and Systems (R:SS), 2018. (http://www.roboticsproceedings.org/rss14/p27.pdf)
 * N. Jaquier, L. Rozo, D. G. Caldwell and S. Calinon. Geometry-aware Manipulability Learning, Tracking and Transfer, International Journal of Robotics Research (IJRR), 2020.
 * @brief The ManipulabilityTracking class
 */
class AbstractManipulabilityTracking
{
public:
    virtual Eigen::VectorXf calculateVelocity(const Eigen::MatrixXd &manipulabilityDesired, const Eigen::MatrixXd &gainMatrix = Eigen::MatrixXd(), bool jointLimitAvoidance = false) = 0;

    std::map<std::string, float> calculateVelocityMap(const Eigen::MatrixXd &manipulabilityDesired, const std::vector<std::string> &jointNames, const Eigen::MatrixXd &gainMatrix = Eigen::MatrixXd(), bool jointLimitAvoidance = false);

    Eigen::Tensor<double, 3> computeJacobianDerivative(const Eigen::MatrixXd &jacobian);

    Eigen::Tensor<double, 3> tensorMatrixProduct(const Eigen::Tensor<double, 3> &tensor, const Eigen::MatrixXd &matrix);

    Eigen::Tensor<double, 3> tensorMatrixProductPermutate(const Eigen::Tensor<double, 3> &tensor, const Eigen::MatrixXd &matrix);

    Eigen::Tensor<double, 3> subCube(Eigen::Tensor<double, 3> &manipulationJacobian);

    Eigen::MatrixXd getDefaultGainMatrix();

    Eigen::MatrixXd logMap(const Eigen::MatrixXd &manipulabilityDesired, const Eigen::MatrixXd &manipulabilityCurrent);

    double computeDistance(const Eigen::MatrixXd &manipulabilityDesired);   

    /* Calculate weight matrix for joint limits avoidance */
    Eigen::MatrixXd getJointsLimitsWeightMatrix(const Eigen::VectorXd &jointAngles, const Eigen::VectorXd &jointLimitsLow, const Eigen::VectorXd &jointLimitsHigh);

    Eigen::MatrixXd computeManipulabilityJacobianMandelNotation(const Eigen::Tensor<double, 3> &manipulabilityJacobian);

    Eigen::VectorXd symMatrixToVector(const Eigen::MatrixXd &sym_matrix);

    virtual int getTaskVars() = 0;

    virtual AbstractManipulability::Mode getMode() = 0;

    virtual Eigen::MatrixXd computeCurrentManipulability() = 0;

    virtual std::vector<std::string> getJointNames() = 0;

    virtual VisualizationNodePtr getManipulabilityVis(const Eigen::MatrixXd &manipulability, const std::string &visualizationType = "", double scaling = 1000.0) = 0;

    virtual void setConvertMMtoM(bool value) = 0;

    void setjointAngleLimitGradient(const Eigen::VectorXd &gradient);

    Eigen::Matrix<double, Eigen::Dynamic, 1> jointAngleLimitGradient;  // TODO initialize
};

typedef std::shared_ptr<AbstractManipulabilityTracking> AbstractManipulabilityTrackingPtr;

}
