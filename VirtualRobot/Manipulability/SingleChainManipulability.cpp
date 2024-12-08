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
#include "SingleChainManipulability.h"

#include <Visualization/VisualizationFactory.h>
#include <Visualization/VisualizationNode.h>
#include <Eigen/Dense>
#include "../IK/DifferentialIK.h"
#include "VirtualRobot/RobotNodeSet.h"
#include "VirtualRobot/Robot.h"
#include <SimoxUtility/math/convert/pos_quat_to_mat4f.h>
#include "Nodes/RobotNode.h"

namespace VirtualRobot
{

Eigen::MatrixXd AbstractSingleChainManipulability::computeManipulability(const Eigen::MatrixXd &jacobian, AbstractManipulability::Type type) {
    if (weightMatrix.rows() == 0){
        int nbJoints = jacobian.cols();
        weightMatrix.setIdentity(nbJoints, nbJoints); 
    }
    Eigen::MatrixXd velocityManipulability = jacobian * weightMatrix * weightMatrix.transpose() * jacobian.transpose();

    switch (type) {
    case Velocity:
        return velocityManipulability;
    case Force:
        return velocityManipulability.inverse();
    default:
        throw std::runtime_error("Unkown manipulability type");
    }
}





SingleRobotNodeSetManipulability::SingleRobotNodeSetManipulability(const RobotNodeSetPtr &rns, Mode mode, Type type, Eigen::MatrixXd weightMatrixInit, bool convertMMtoM)
    : SingleRobotNodeSetManipulability(rns, rns->getTCP(), mode, type, RobotNodePtr(), weightMatrixInit, convertMMtoM)
{
}

SingleRobotNodeSetManipulability::SingleRobotNodeSetManipulability(const RobotNodeSetPtr &rns, const RobotNodePtr &node, Mode mode, Type type, const RobotNodePtr &coordSystem, Eigen::MatrixXd weightMatrixInit, bool convertMMtoM)
    : AbstractSingleChainManipulability(mode, type, weightMatrixInit), rns(rns), node(node), coordSystem(coordSystem)
{
    ik.reset(new DifferentialIK(rns, coordSystem, JacobiProvider::eSVDDampedDynamic));
    ik->convertModelScalingtoM(convertMMtoM);
}

RobotNodeSetPtr SingleRobotNodeSetManipulability::getRobotNodeSet() {
    return rns;
}

RobotPtr SingleRobotNodeSetManipulability::getRobot() {
    return rns->getRobot();
}

void SingleRobotNodeSetManipulability::setConvertMMtoM(bool value) {
    ik->convertModelScalingtoM(value);
}

Eigen::Vector3f SingleRobotNodeSetManipulability::getLocalPosition() {
    return rns->getTCP()->getPositionInRootFrame();
}

Eigen::Vector3f SingleRobotNodeSetManipulability::getGlobalPosition() {
    return rns->getTCP()->getGlobalPosition();
}

Eigen::Matrix4f SingleRobotNodeSetManipulability::getCoordinateSystem() {
    return coordSystem ? coordSystem->getGlobalPose() : Eigen::Matrix4f::Identity();
}

std::vector<std::string> SingleRobotNodeSetManipulability::getJointNames() {
    return rns->getNodeNames();
}

Eigen::VectorXd SingleRobotNodeSetManipulability::getJointAngles() {
    return rns->getJointValuesEigen().cast<double>();
}

Eigen::VectorXd SingleRobotNodeSetManipulability::getJointLimitsHigh() {
    Eigen::VectorXd jointLimitHi(rns->getSize());
    for (size_t i = 0; i < rns->getSize(); i++)
    {
        RobotNodePtr rn = rns->getNode(i);
        if (rn->isLimitless())
        {
            jointLimitHi(i) = 0;
        }
        else
            jointLimitHi(i) = rn->getJointLimitHi();
    }
    return jointLimitHi;
}
    
Eigen::VectorXd SingleRobotNodeSetManipulability::getJointLimitsLow() {
    Eigen::VectorXd jointLimitLo(rns->getSize());
    for (size_t i = 0; i < rns->getSize(); i++)
    {
        RobotNodePtr rn = rns->getNode(i);
        if (rn->isLimitless())
        {
            jointLimitLo(i) = 0;
        }
        else
            jointLimitLo(i) = rn->getJointLimitLo();
    }
    return jointLimitLo;
}

Eigen::MatrixXd SingleRobotNodeSetManipulability::computeJacobian(IKSolver::CartesianSelection mode) {
    return ik->getJacobianMatrix(node, mode).cast<double>();
}




SingleChainManipulability::SingleChainManipulability(const Eigen::Matrix<double, 6, Eigen::Dynamic> &jacobian,
                                                     AbstractManipulability::Mode mode, AbstractManipulability::Type type,
                                                     const Eigen::Matrix<double, Eigen::Dynamic, 1> &jointAngles, const Eigen::VectorXd &jointLimitsHigh, const Eigen::VectorXd &jointLimitsLow,
                                                     const std::vector<std::string> &jointNames,
                                                     const Eigen::Vector3f &globalPosition, const Eigen::Vector3f &localPosition) :
    AbstractSingleChainManipulability(mode, type),
    jacobian(jacobian),
    jointNames(jointNames),
    globalPosition(globalPosition),
    localPosition(localPosition),
    jointAngles(jointAngles),
    jointLimitsHigh(jointLimitsHigh),
    jointLimitsLow(jointLimitsLow)
{
}

Eigen::Vector3f SingleChainManipulability::getLocalPosition() {
    return localPosition;
}

Eigen::Vector3f SingleChainManipulability::getGlobalPosition() {
    return globalPosition;
}

Eigen::Matrix4f SingleChainManipulability::getCoordinateSystem() {
    return Eigen::Matrix4f::Identity();
}

std::vector<std::string> SingleChainManipulability::getJointNames() {
    return jointNames;
}

Eigen::VectorXd SingleChainManipulability::getJointAngles() {
    return jointAngles;
}

Eigen::VectorXd SingleChainManipulability::getJointLimitsHigh() {
    return jointLimitsHigh;
}
    
Eigen::VectorXd SingleChainManipulability::getJointLimitsLow() {
    return jointLimitsLow;
}

void SingleChainManipulability::setJacobian(const Eigen::Matrix<double, 6, Eigen::Dynamic> &jacobian) {
    this->jacobian = jacobian;
}

void SingleChainManipulability::setLocalPosition(const Eigen::Vector3f &localPosition) {
    this->localPosition = localPosition;
}

void SingleChainManipulability::setGlobalPosition(const Eigen::Vector3f &globalPosition) {
    this->globalPosition = globalPosition;
}

Eigen::MatrixXd SingleChainManipulability::computeJacobian(IKSolver::CartesianSelection mode) {
    return GetJacobianSubMatrix(jacobian, mode);
}



}
