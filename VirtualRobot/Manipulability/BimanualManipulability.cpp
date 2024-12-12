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

#include "BimanualManipulability.h"

#include "VirtualRobotException.h"
#include <Visualization/VisualizationFactory.h>
#include <Visualization/VisualizationNode.h>
#include "../IK/DifferentialIK.h"
#include "VirtualRobot/RobotNodeSet.h"
#include "VirtualRobot/Robot.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/CXX11/Tensor>
#include <SimoxUtility/math/convert/pos_quat_to_mat4f.h>

#include "Nodes/RobotNode.h"
#include "RobotNodeSet.h"

namespace VirtualRobot
{

BimanualManipulability::BimanualManipulability(const RobotNodeSetPtr &rnsLeft, const RobotNodeSetPtr &rnsRight, Mode mode, Type type, bool convertMMtoM)
    : BimanualManipulability(rnsLeft, rnsRight, rnsLeft->getTCP(), rnsRight->getTCP(), mode, type, SceneObjectPtr(), RobotNodePtr(), convertMMtoM)
{
}

BimanualManipulability::BimanualManipulability(const RobotNodeSetPtr &rnsLeft, const RobotNodeSetPtr &rnsRight, const RobotNodePtr &nodeLeft, const RobotNodePtr &nodeRight, Mode mode, Type type,  const SceneObjectPtr& object, const RobotNodePtr &coordSystem, bool convertMMtoM)
    : AbstractManipulability(mode, type), rnsLeft(rnsLeft), rnsRight(rnsRight), nodeLeft(nodeLeft), nodeRight(nodeRight), object(object), coordSystem(coordSystem)
{
    if (rnsLeft->getRobot() != rnsRight->getRobot()) {
        throw VirtualRobotException("Left and right robot node set of bimanual manipulability do not map to the same robot!");
    }
    ikLeft.reset(new DifferentialIK(rnsLeft, coordSystem, JacobiProvider::eSVDDampedDynamic));
    ikLeft->convertModelScalingtoM(convertMMtoM);
    ikRight.reset(new DifferentialIK(rnsRight, coordSystem, JacobiProvider::eSVDDampedDynamic));
    ikRight->convertModelScalingtoM(convertMMtoM);
}

Eigen::MatrixXd BimanualManipulability::computeFullJacobianLeft() {
    return computeJacobianLeft(IKSolver::All);
}

Eigen::MatrixXd BimanualManipulability::computeFullJacobianRight() {
    return computeJacobianRight(IKSolver::All);
}

Eigen::MatrixXd BimanualManipulability::computeBimanualJacobian(const Eigen::MatrixXd &jacobianLeft, const Eigen::MatrixXd &jacobianRight){
    int nbVarsLeft = jacobianLeft.rows(); // task space dim
    int nbDofsLeft = jacobianLeft.cols(); // joint space dim
    int nbVarsRight = jacobianRight.rows(); // task space dim
    int nbDofsRight = jacobianRight.cols(); // joint space dim

    // Build a block-diagonal matrix with the two Jacobians
    // Note: we assume here that the kinematic chains of the two arms are independent! 
    Eigen::MatrixXd jacobianBimanual = Eigen::MatrixXd::Zero(nbVarsLeft + nbVarsRight, nbDofsLeft + nbDofsRight);
    jacobianBimanual.block(0, 0, nbVarsLeft, nbDofsLeft) = jacobianLeft;
    jacobianBimanual.block(nbVarsLeft, nbDofsLeft, nbVarsRight, nbDofsRight) = jacobianRight;

    return jacobianBimanual;
}

Eigen::MatrixXd BimanualManipulability::computeManipulability() {
    return computeManipulability(computeFullJacobian());
}

Eigen::MatrixXd BimanualManipulability::computeManipulability(const Eigen::MatrixXd &jacobian, Type type) {
    if (jacobian.rows() != 6)
        std::runtime_error("computeManipulability(...) of Bimanual Maniulability requires the full jacobian!");
    return getManipulabilitySubMatrix(computeBimanualManipulability(jacobian, computeBimanualGraspMatrix(), type));
}

Eigen::Vector3f BimanualManipulability::getLocalPosition() {
    return rnsLeft->getRobot()->toGlobalCoordinateSystemVec(getGlobalPosition());
}

Eigen::Vector3f BimanualManipulability::getGlobalPosition() {
    if (object) return object->getCoMGlobal();
    else return (nodeLeft->getGlobalPosition() + nodeRight->getGlobalPosition()) / 2.0f;
}

Eigen::Matrix4f BimanualManipulability::getCoordinateSystem() {
    return coordSystem ? coordSystem->getGlobalPose() : Eigen::Matrix4f::Identity();
}

Eigen::MatrixXd BimanualManipulability::computeBimanualGraspMatrix() {
    Eigen::Vector3f endeffectorLeft = nodeLeft->getPositionInRootFrame();
    Eigen::Vector3f endeffectorRight = nodeRight->getPositionInRootFrame();
    Eigen::MatrixXd rotation = Eigen::MatrixXd::Identity(3, 3);
    Eigen::MatrixXd bimanualGraspMatrix = computeBimanualGraspMatrix(endeffectorLeft, endeffectorRight, rotation);
    return bimanualGraspMatrix;
}

RobotNodeSetPtr BimanualManipulability::getLeftRobotNodeSet() {
    return rnsLeft;
}

RobotNodeSetPtr BimanualManipulability::getRightRobotNodeSet() {
    return rnsRight;
}

std::vector<std::string> BimanualManipulability::getJointNames() {
    std::vector<std::string> robotNodeNames = rnsLeft->getNodeNames();
    auto rightRobotNodeNames = rnsRight->getNodeNames();
    robotNodeNames.insert(robotNodeNames.end(), rightRobotNodeNames.begin(), rightRobotNodeNames.end());
    return robotNodeNames;
}

Eigen::VectorXd BimanualManipulability::getJointAngles() {
    Eigen::VectorXd jointAngles(rnsLeft->getSize() + rnsRight->getSize());
    jointAngles.head(rnsLeft->getSize()) = rnsLeft->getJointValuesEigen().cast<double>();
    jointAngles.tail(rnsRight->getSize()) = rnsRight->getJointValuesEigen().cast<double>();
    return jointAngles;
}

Eigen::VectorXd BimanualManipulability::getJointLimitsHigh() {
    Eigen::VectorXd jointLimitHi(rnsLeft->getSize() + rnsRight->getSize());
    for (size_t i = 0; i < rnsLeft->getSize(); i++)
    {
        RobotNodePtr rn = rnsLeft->getNode(i);
        if (rn->isLimitless())
        {
            jointLimitHi(i) = 0;
        }
        else
            jointLimitHi(i) = rn->getJointLimitHi();
    }
    for (size_t i = 0; i < rnsRight->getSize(); i++)
    {
        RobotNodePtr rn = rnsRight->getNode(i);
        if (rn->isLimitless())
        {
            jointLimitHi(i + rnsLeft->getSize()) = 0;
        }
        else
            jointLimitHi(i + rnsLeft->getSize()) = rn->getJointLimitHi();
    }
    return jointLimitHi;
}
    
Eigen::VectorXd BimanualManipulability::getJointLimitsLow() {
    Eigen::VectorXd jointLimitLo(rnsLeft->getSize() + rnsRight->getSize());
    for (size_t i = 0; i < rnsLeft->getSize(); i++)
    {
        RobotNodePtr rn = rnsLeft->getNode(i);
        if (rn->isLimitless())
        {
            jointLimitLo(i) = 0;
        }
        else
            jointLimitLo(i) = rn->getJointLimitLo();
    }
    for (size_t i = 0; i < rnsRight->getSize(); i++)
    {
        RobotNodePtr rn = rnsRight->getNode(i);
        if (rn->isLimitless())
        {
            jointLimitLo(i + rnsLeft->getSize()) = 0;
        }
        else
            jointLimitLo(i + rnsLeft->getSize()) = rn->getJointLimitLo();
    }
    return jointLimitLo;
}

RobotPtr BimanualManipulability::getRobot() {
    return rnsLeft->getRobot();
}

RobotNodeSetPtr BimanualManipulability::createRobotNodeSet(const std::string &name) {
    return RobotNodeSet::createRobotNodeSet(getRobot(), name, getJointNames(), getRobot()->getRootNode()->getName());
}

void BimanualManipulability::setConvertMMtoM(bool value) {
    ikLeft->convertModelScalingtoM(value);
    ikRight->convertModelScalingtoM(value);
}

Eigen::MatrixXd BimanualManipulability::computeBimanualManipulability(const Eigen::MatrixXd &bimanualJacobian, const Eigen::MatrixXd &bimanualGraspMatrix) {
    return computeBimanualManipulability(bimanualJacobian, bimanualGraspMatrix, type);
}

Eigen::MatrixXd BimanualManipulability::computeBimanualManipulability(const Eigen::MatrixXd &bimanualJacobian, const Eigen::MatrixXd &bimanualGraspMatrix, Type type) {
    // Compute pseudo-inverse of grasp matrix
    Eigen::MatrixXd pinverseBimanualGraspMatrix = bimanualGraspMatrix.transpose() * (bimanualGraspMatrix * bimanualGraspMatrix.transpose()).inverse();
    // Compute manipulability
    Eigen::MatrixXd velocityManipulability = pinverseBimanualGraspMatrix.transpose() * (bimanualJacobian * bimanualJacobian.transpose()) * pinverseBimanualGraspMatrix;

    switch (type) {
    case Velocity:
        return velocityManipulability;
    case Force:
        return velocityManipulability.inverse();
    default:
        throw std::runtime_error("Unkown manipulability type");
    }
}

Eigen::MatrixXd BimanualManipulability::computeJacobian(IKSolver::CartesianSelection mode) {
    return computeBimanualJacobian(computeJacobianLeft(mode), computeJacobianRight(mode));
}

Eigen::MatrixXd BimanualManipulability::computeJacobianLeft(IKSolver::CartesianSelection mode) {
    return ikLeft->getJacobianMatrix(nodeLeft, mode).cast<double>();
}

Eigen::MatrixXd BimanualManipulability::computeJacobianRight(IKSolver::CartesianSelection mode) {
    return ikRight->getJacobianMatrix(nodeRight, mode).cast<double>();
}

Eigen::MatrixXd BimanualManipulability::computeBimanualGraspMatrix(const Eigen::Vector3f &endeffectorLeftPosition, const Eigen::Vector3f &endeffectorRightPosition, const Eigen::MatrixXd &rotation) {
    Eigen::Vector3f objectPosition = getLocalPosition();
    // Compute left and right grasp matrices
    Eigen::MatrixXd graspMatrixLeft = computeGraspMatrix(endeffectorLeftPosition, objectPosition, rotation);
    Eigen::MatrixXd graspMatrixRight = computeGraspMatrix(endeffectorRightPosition, objectPosition, rotation);

    // Build the bimanual grasp matrix
    Eigen::MatrixXd graspMatrix = Eigen::MatrixXd::Zero(6, 12);
    graspMatrix.block(0, 0, 6, 6) = graspMatrixLeft;
    graspMatrix.block(0, 6, 6, 6) = graspMatrixRight;
    return graspMatrix;
}

Eigen::MatrixXd BimanualManipulability::computeGraspMatrix(const Eigen::VectorXf &endeffectorPosition, const Eigen::VectorXf &objectPosition, const Eigen::MatrixXd &rotation) {
    Eigen::MatrixXd crossProductPosition = Eigen::MatrixXd::Zero(3, 3);
    crossProductPosition(0, 1) = endeffectorPosition(2) - objectPosition(2);
    crossProductPosition(0, 2) = objectPosition(1) - endeffectorPosition(1);
    crossProductPosition(1, 0) = objectPosition(2) - endeffectorPosition(2);
    crossProductPosition(1, 2) = endeffectorPosition(0) - objectPosition(0);
    crossProductPosition(2, 0) = endeffectorPosition(1) - objectPosition(1);
    crossProductPosition(2, 1) = objectPosition(0) - endeffectorPosition(0);

    Eigen::MatrixXd graspMatrix = Eigen::MatrixXd::Zero(6, 6);
    graspMatrix.block(0, 0, 3, 3) = rotation;
    graspMatrix.block(3, 3, 3, 3) = rotation;
    graspMatrix.block(3, 0, 3, 3) = crossProductPosition;

    return graspMatrix;
}}
