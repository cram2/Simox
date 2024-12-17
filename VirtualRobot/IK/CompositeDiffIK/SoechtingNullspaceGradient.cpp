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
* @author     André Meixner (andre dot meixner at kit dot edu)
* @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
*             GNU General Public License
*/

#include "SoechtingNullspaceGradient.h"

#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/Robot.h>

#include "RobotNodeSet.h"
#include "Soechting.h"

namespace VirtualRobot
{
    SoechtingNullspaceGradient::SoechtingNullspaceGradient(const CompositeDiffIK::TargetPtr& target,
                                                           const std::string& shoulderName,
                                                           const Soechting::ArmType& arm,
                                                           const ArmJoints& joints) :
        CompositeDiffIK::NullspaceGradient(joints.getRobotNodeNames()),
        rns(joints.createRobotNodeSet("Soechting" + std::to_string(arm))),
        target(target),
        shoulder(rns->getRobot()->getRobotNode(shoulderName)),
        arm(arm),
        joints(joints)
    {
    }

    void
    SoechtingNullspaceGradient::init(CompositeDiffIK::Parameters&)
    {
        // Do nothing
    }

    Eigen::VectorXf
    SoechtingNullspaceGradient::getGradient(CompositeDiffIK::Parameters& params, int /*stepNr*/)
    {
        const size_t dim = rns->getSize();

        auto sa = calcShoulderAngles(params);
        Eigen::VectorXf weights = Eigen::VectorXf::Zero(dim);
        Eigen::VectorXf target = Eigen::VectorXf::Zero(dim);

#define SET(joint, w, t)                                                                           \
    if (joint)                                                                                     \
    {                                                                                              \
        auto id = rns->getRobotNodeIndex(joint);                                                   \
        weights[id] = w;                                                                           \
        target[id] = t;                                                                            \
    }
        SET(joints.clavicula, 0.5f, sa.C)
        SET(joints.shoulder1, 1, sa.SE)
        SET(joints.shoulder2, 2, sa.SR)
        SET(joints.shoulder3, 0.5f, -M_PI / 4)
        SET(joints.elbow, 0.5f, sa.E)
#undef SET
        std::cout << target << std::endl;

        return (target - rns->getJointValuesEigen()).cwiseProduct(weights);
    }

    SoechtingNullspaceGradient::ShoulderAngles
    SoechtingNullspaceGradient::calcShoulderAngles(
        const CompositeDiffIK::Parameters& /*params*/) const
    {
        const Eigen::Matrix4f currentShoulder = shoulder->getPoseInRootFrame();

        Soechting::Arm arm;
        arm.armType = this->arm;
        arm.shoulder = currentShoulder.block<3, 1>(0, 3);
        arm.upperArmLength = 0.f;
        arm.forearmLength = 0.f;

        Eigen::Vector3f targetPosition = target->target.block<3, 1>(0, 3);
        const auto sa = Soechting::CalculateAngles(targetPosition, arm);

        const Eigen::AngleAxisf aaSE(sa.SE, Eigen::Vector3f::UnitX());
        const Eigen::AngleAxisf aaSY(-sa.SY, Eigen::Vector3f::UnitZ());
        const Eigen::AngleAxisf aaEE(-sa.EE, Eigen::Vector3f::UnitX());
        const Eigen::AngleAxisf aaEY(-sa.EY, Eigen::Vector3f::UnitZ());

        const Eigen::Vector3f elb = Eigen::AngleAxisf(-sa.SE, Eigen::Vector3f::UnitX()) * aaSY *
                                    aaSE * -Eigen::Vector3f::UnitZ();
        const float SR = std::atan2(elb(0), -elb(2));

        ShoulderAngles res;
        res.SR = std::max(-0.1f, SR);
        res.SE = sa.SE;
        res.E = sa.EE;
        res.C = 0;
        return res;
    }

    RobotNodeSetPtr
    SoechtingNullspaceGradient::ArmJoints::createRobotNodeSet(const std::string& name) const
    {
        std::vector<RobotNodePtr> robotNodes;
        for (auto node : {clavicula, shoulder1, shoulder2, shoulder3, elbow})
        {
            if (node)
                robotNodes.push_back(node);
        }
        if (robotNodes.empty())
            return nullptr;

        auto frontNode = robotNodes.front();
        return RobotNodeSet::createRobotNodeSet(frontNode->getRobot(), name, robotNodes, frontNode);
    }

    std::vector<std::string>
    SoechtingNullspaceGradient::ArmJoints::getRobotNodeNames() const
    {
        std::vector<std::string> nodeNames;
        for (auto node : {clavicula, shoulder1, shoulder2, shoulder3, elbow})
        {
            if (node)
                nodeNames.push_back(node->getName());
        }
        return nodeNames;
    }

} // namespace VirtualRobot
