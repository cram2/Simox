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
#include "AbstractManipulability.h"

#include <Eigen/Dense>
#include <SimoxUtility/math/convert.h>
#include "VirtualRobot/Visualization/VisualizationFactory.h"
#include "VirtualRobot/Visualization/VisualizationNode.h"


namespace VirtualRobot
{

AbstractManipulability::AbstractManipulability(AbstractManipulability::Mode mode, AbstractManipulability::Type type,  Eigen::MatrixXd weightMatrixInit) :
    mode(mode),
    type(type)
{
    weightMatrix = weightMatrixInit;
}

Eigen::MatrixXd AbstractManipulability::computeJacobian() {
    return computeJacobian(getCartesianSelection());
}

Eigen::MatrixXd AbstractManipulability::computeFullJacobian() {
    return computeJacobian(VirtualRobot::IKSolver::All);
}

Eigen::MatrixXd AbstractManipulability::getJacobianSubMatrix(const Eigen::Matrix<double, 6, Eigen::Dynamic> &jacobian) {
    return GetJacobianSubMatrix(jacobian, mode);
}

Eigen::MatrixXd AbstractManipulability::getManipulabilitySubMatrix(const Eigen::Matrix<double, 6, 6> &manipulability) {
    switch (mode) {
    case AbstractManipulability::Whole:
        return manipulability;
    case AbstractManipulability::Position:
        return manipulability.block(0, 0, 3, 3);
    case AbstractManipulability::Orientation:
        return manipulability.block(3, 3, 3, 3);
    default:
        throw std::runtime_error("Mode not supported");
    }
}


Eigen::MatrixXd AbstractManipulability::computeManipulability() {
    return computeManipulability(computeJacobian());
}

Eigen::MatrixXd AbstractManipulability::computeManipulability(const Eigen::MatrixXd &jacobian) {
    return computeManipulability(jacobian, type);
}

VisualizationNodePtr AbstractManipulability::getManipulabilityVis(const std::string &visualizationType, double scaling) {
    return getManipulabilityVis(computeManipulability(), visualizationType, scaling);
}

VisualizationNodePtr AbstractManipulability::getManipulabilityVis(const Eigen::MatrixXd &manipulability, const std::string &visualizationType, double scaling) {
    return getManipulabilityVis(manipulability, getGlobalPosition(), visualizationType, scaling);
}

VisualizationNodePtr AbstractManipulability::getManipulabilityVis(const Eigen::MatrixXd &manipulability, const Eigen::Vector3f &position, const std::string &visualizationType, double scaling) {
    VisualizationFactoryPtr visualizationFactory;
    if (visualizationType.empty())
    {
        visualizationFactory = VisualizationFactory::first(NULL);
    }
    else
    {
        visualizationFactory = VisualizationFactory::fromName(visualizationType, NULL);
    }
    if (!visualizationFactory)
    {
        VR_WARNING << "No visualization factory for name " << visualizationType << std::endl;
        return nullptr;
    }

    Eigen::Quaternionf orientation;
    Eigen::Vector3d scale;
    getEllipsoidOrientationAndScale(manipulability, orientation, scale);
    scale *= scaling;
    auto vis = visualizationFactory->createEllipse(scale(0), scale(1), scale(2), false);
    Eigen::Matrix4f pose = simox::math::pos_mat3f_to_mat4f(position, simox::math::mat4f_to_mat3f(getCoordinateSystem()).inverse() * simox::math::quat_to_mat3f(orientation));
    vis->setUpdateVisualization(true);
    vis->setGlobalPose(pose);
    return vis;
}

void AbstractManipulability::getEllipsoidOrientationAndScale(const Eigen::MatrixXd& manipulability, Eigen::Quaternionf& orientation, Eigen::Vector3d& scale) {
    
    constexpr std::size_t POSITION_DIMS = 3;
    constexpr std::size_t ORIENTATION_DIMS = 3;
    constexpr std::size_t POSE_DIMS = POSITION_DIMS + ORIENTATION_DIMS;
    
    const Eigen::MatrixXd reduced_manipulability =
        (mode != Mode::Orientation || manipulability.rows() == 3)
            ? manipulability.block(0, 0, POSITION_DIMS, POSITION_DIMS)
            : manipulability.block(
                    POSITION_DIMS, POSITION_DIMS, ORIENTATION_DIMS, ORIENTATION_DIMS);
    const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(reduced_manipulability);

    // Eigenvectors are the columns of the matrix 'eigenVectors'
    // they are already normalized to have length 1
    const Eigen::MatrixXd& eigenVectors = eigenSolver.eigenvectors();

    // Eigen values are sorted in increasing order
    const Eigen::Vector3d& eigenValues = eigenSolver.eigenvalues();

    // We create a ortho-normal basis of the eigen vectors.
    // Here, we use the eigen vectors with the eigen values in decreasing order.
    // To ensure a right-handed coordinate system, the third basis vector is computed
    // by using the cross product.
    Eigen::Matrix3d rotationMatrix;
    rotationMatrix.col(0) = eigenVectors.col(2);
    rotationMatrix.col(1) = eigenVectors.col(1);
    rotationMatrix.col(2) = rotationMatrix.col(0).cross(rotationMatrix.col(1));

    orientation = rotationMatrix;

    scale = eigenValues.reverse();

    // normalize singular values for scaling
    scale /= scale.sum();
    for (int i = 0; i < eigenValues.rows(); i++)
    {
        constexpr double minEigenVal = 0.005; // [mm]

        if (scale(i) < minEigenVal)
        {
            scale(i) = minEigenVal;
        }
    }
}

void AbstractManipulability::getEllipsoidOrientationAndScale(Eigen::Quaternionf &orientation, Eigen::Vector3d &scale) {
    getEllipsoidOrientationAndScale(computeManipulability(), orientation, scale);
}

IKSolver::CartesianSelection AbstractManipulability::getCartesianSelection() {
    return GetCartesianSelection(mode);
}

AbstractManipulability::Mode AbstractManipulability::getMode() {
    return mode;
}

AbstractManipulability::Type AbstractManipulability::getType() {
    return type;
}

int AbstractManipulability::getTaskVars() {
    switch(mode) {
    case Whole:
        return 6;
    case Position:
    case Orientation:
        return 3;
    default:
        throw std::runtime_error("Unkown manipulability mode");
    }
}

Eigen::MatrixXd AbstractManipulability::GetJacobianSubMatrix(const Eigen::Matrix<double, 6, Eigen::Dynamic> &jacobian, IKSolver::CartesianSelection mode) {
    switch (mode) {
    case IKSolver::All:
        return jacobian;
    case IKSolver::Position:
        return jacobian.block(0, 0, 3, jacobian.cols());
    case IKSolver::Orientation:
        return jacobian.block(3, 0, 3, jacobian.cols());
    default:
        throw std::runtime_error("Mode not supported");
    }
}

Eigen::MatrixXd AbstractManipulability::GetJacobianSubMatrix(const Eigen::Matrix<double, 6, Eigen::Dynamic> &jacobian, Mode mode) {
    switch (mode) {
    case AbstractManipulability::Whole:
        return jacobian;
    case AbstractManipulability::Position:
        return jacobian.block(0, 0, 3, jacobian.cols());
    case AbstractManipulability::Orientation:
        return jacobian.block(3, 0, 3, jacobian.cols());
    default:
        throw std::runtime_error("Mode not supported");
    }
}

IKSolver::CartesianSelection AbstractManipulability::GetCartesianSelection(AbstractManipulability::Mode mode) {
    switch(mode) {
    case Whole:
        return IKSolver::All;
    case Position:
        return IKSolver::Position;
    case Orientation:
        return IKSolver::Orientation;
    default:
        throw std::runtime_error("Unkown manipulability mode");
    }
}

}
