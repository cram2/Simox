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
* @author     Simon Ottenhaus (simon dot ottenhaus at kit dot edu)
* @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
*             GNU General Public License
*/

#include "CartesianPositionController.h"

#include "VirtualRobot/math/Helpers.h"
#include "../Nodes/RobotNode.h"
#include <Eigen/Dense>

namespace VirtualRobot
{
    CartesianPositionController::CartesianPositionController(const RobotNodePtr& tcp)
        : tcp(tcp)
    {
    }

    Eigen::VectorXf CartesianPositionController::calculate(const Eigen::Matrix4f& targetPose, IKSolver::CartesianSelection mode) const
    {
        int posLen = mode & IKSolver::Position ? 3 : 0;
        int oriLen = mode & IKSolver::Orientation ? 3 : 0;
        Eigen::VectorXf cartesianVel(posLen + oriLen);

        if (posLen)
        {
            Eigen::Vector3f targetPos = targetPose.block<3, 1>(0, 3);
            Eigen::Vector3f currentPos = tcp->getPositionInRootFrame();
            Eigen::Vector3f errPos = targetPos - currentPos;
            Eigen::Vector3f posVel =  errPos * KpPos;
            if (maxPosVel > 0)
            {
                posVel = math::Helpers::LimitTo(posVel, maxPosVel);
            }
            cartesianVel.block<3, 1>(0, 0) = posVel;
        }

        if (oriLen)
        {
            Eigen::Matrix3f targetOri = targetPose.block<3, 3>(0, 0);
            Eigen::Matrix3f tcpOri = tcp->getPoseInRootFrame().block<3, 3>(0, 0);
            Eigen::Matrix3f oriDir = targetOri * tcpOri.inverse();
            Eigen::AngleAxisf aa(oriDir);
            Eigen::Vector3f errOri = aa.axis() * aa.angle();
            Eigen::Vector3f oriVel = errOri * KpOri;

            if (maxOriVel > 0)
            {
                oriVel = math::Helpers::LimitTo(oriVel, maxOriVel);
            }
            cartesianVel.block<3, 1>(posLen, 0) = oriVel;
        }
        return cartesianVel;
    }

    Eigen::VectorXf CartesianPositionController::calculatePos(const Eigen::Vector3f& targetPos) const
    {
        Eigen::VectorXf cartesianVel(3);
        Eigen::Vector3f currentPos = tcp->getPositionInRootFrame();
        Eigen::Vector3f errPos = targetPos - currentPos;
        Eigen::Vector3f posVel =  errPos * KpPos;
        if (maxPosVel > 0)
        {
            posVel = math::Helpers::LimitTo(posVel, maxPosVel);
        }
        cartesianVel.block<3, 1>(0, 0) = posVel;
        return cartesianVel;
    }

    float CartesianPositionController::getPositionError(const Eigen::Matrix4f& targetPose) const
    {
        return GetPositionError(targetPose, tcp);
    }

    float CartesianPositionController::getOrientationError(const Eigen::Matrix4f& targetPose) const
    {
        return GetOrientationError(targetPose, tcp);
    }

    float CartesianPositionController::GetPositionError(const Eigen::Matrix4f& targetPose, const RobotNodePtr& tcp)
    {
        Eigen::Vector3f targetPos = targetPose.block<3, 1>(0, 3);
        Eigen::Vector3f errPos = targetPos - tcp->getPositionInRootFrame();
        return errPos.norm();
    }

    float CartesianPositionController::GetOrientationError(const Eigen::Matrix4f& targetPose, const RobotNodePtr& tcp)
    {
        Eigen::Matrix3f targetOri = targetPose.block<3, 3>(0, 0);
        Eigen::Matrix3f tcpOri = tcp->getPoseInRootFrame().block<3, 3>(0, 0);
        Eigen::Matrix3f oriDir = targetOri * tcpOri.inverse();
        Eigen::AngleAxisf aa(oriDir);
        return aa.angle();
    }

    bool CartesianPositionController::Reached(const Eigen::Matrix4f& targetPose, const RobotNodePtr& tcp, bool checkOri, float thresholdPosReached, float thresholdOriReached)
    {
        return GetPositionError(targetPose, tcp) < thresholdPosReached
               && (!checkOri || GetOrientationError(targetPose, tcp) < thresholdOriReached);
    }

    bool CartesianPositionController::reached(const Eigen::Matrix4f& targetPose, VirtualRobot::IKSolver::CartesianSelection mode, float thresholdPosReached, float thresholdOriReached)
    {
        if (mode & VirtualRobot::IKSolver::Position)
        {
            if (GetPositionError(targetPose, tcp) > thresholdPosReached)
            {
                return false;
            }
        }
        if (mode & VirtualRobot::IKSolver::Orientation)
        {
            if (GetOrientationError(targetPose, tcp) > thresholdOriReached)
            {
                return false;
            }
        }
        return true;
    }


    Eigen::Vector3f CartesianPositionController::getPositionDiff(const Eigen::Matrix4f& targetPose) const
    {
        Eigen::Vector3f targetPos = targetPose.block<3, 1>(0, 3);
        return targetPos - tcp->getPositionInRootFrame();
    }

    Eigen::Vector3f CartesianPositionController::getPositionDiffVec3(const Eigen::Vector3f& targetPosition) const
    {
        return targetPosition - tcp->getPositionInRootFrame();
    }

    Eigen::Vector3f CartesianPositionController::getOrientationDiff(const Eigen::Matrix4f& targetPose) const
    {
        Eigen::Matrix3f targetOri = targetPose.block<3, 3>(0, 0);
        Eigen::Matrix3f tcpOri = tcp->getPoseInRootFrame().block<3, 3>(0, 0);
        Eigen::Matrix3f oriDir = targetOri * tcpOri.inverse();
        Eigen::AngleAxisf aa(oriDir);
        return aa.axis() * aa.angle();
    }

    RobotNodePtr CartesianPositionController::getTcp() const
    {
        return tcp;
    }
}
