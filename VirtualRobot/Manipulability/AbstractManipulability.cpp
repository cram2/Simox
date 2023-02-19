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
    Eigen::Matrix3d reduced_manipulability =
      (mode != Mode::Orientation || manipulability.rows() == 3)
        ? manipulability.block(0, 0, 3, 3)
        : manipulability.block(3, 3, 3, 3);
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(reduced_manipulability);
    const Eigen::Matrix3d& eigenVectors = eigenSolver.eigenvectors();
    const Eigen::Vector3d& eigenValues = eigenSolver.eigenvalues();

    // Eigenvectors are the columns of the matrix 'eigenVectors'
    // they are already normalized to have length 1
    // we sort them by the eigenvalues
    struct VecVal
    {
        Eigen::Vector3d vec;
        double val;
    };
    std::array<VecVal, 3> decomp = {
        VecVal{.vec = eigenVectors.col(0), .val = eigenValues(0)},
        VecVal{.vec = eigenVectors.col(1), .val = eigenValues(1)},
        VecVal{.vec = eigenVectors.col(2), .val = eigenValues(2)},
    };
    std::sort(decomp.begin(),
              decomp.end(),
              [](const auto& a, const auto& b) { return a.val > b.val; });

    Eigen::Matrix3d transform;
    transform.col(0) = decomp[0].vec;
    transform.col(1) = decomp[1].vec;
    transform.col(2) = decomp[2].vec;

    // Rectify the transformation matrix representing the orientation of the ellipse
    // We need to make sure that is *special* orthogonal, not just orthogonal
    // Flip the signs of eigenvectors that point opposite the first quadrant
    // Sum of the elements is the same as a dot product with (1, 1, 1)
    // We want this so that the directions of the eigenvectors is consistent,
    // and because this makes transform have determinant 1
    for (int col = 0; col < transform.cols(); ++col)
    {
        if (transform.col(col).sum() < 0.0)
        {
            transform.col(col) *= -1.0;
        }
    }

    // if the matrix still has determinant smaller than one, just flip one of the vectors back. This may be the case if all of the columns point opposite (1, 1, 1)
    if (transform.determinant() < 0)
    {
        transform.col(2) *= -1;
    }
    orientation = transform.cast<float>();

    // Normalize eigenvalues for scaling
    Eigen::Vector3d s =
        Eigen::Vector3d(decomp[0].val, decomp[1].val, decomp[2].val).array().sqrt();
    s /= s.sum();
    for (int i = 0; i < s.rows(); i++)
    {
        if (s(i) < 0.005) // 5mm
            s(i) = 0.005;
    }

    scale(0) = s(0);
    scale(1) = s(1);
    scale(2) = s(2);
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
