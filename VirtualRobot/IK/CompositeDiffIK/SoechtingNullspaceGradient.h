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

#pragma once

#include "CompositeDiffIK.h"
#include "Soechting.h"

namespace VirtualRobot
{

    class SoechtingNullspaceGradient : public CompositeDiffIK::NullspaceGradient
    {
    public:
        struct ArmJoints
        {
            RobotNodePtr clavicula = nullptr;
            RobotNodePtr shoulder1 = nullptr;
            RobotNodePtr shoulder2 = nullptr;
            RobotNodePtr shoulder3 = nullptr;
            RobotNodePtr elbow = nullptr;

            RobotNodeSetPtr createRobotNodeSet(const std::string& name) const;
            std::vector<std::string> getRobotNodeNames() const;
        };

        struct ShoulderAngles
        {
            float SE, SR, E, C;
        };

        SoechtingNullspaceGradient(const CompositeDiffIK::TargetPtr& target,
                                   const std::string& shoulderName,
                                   const Soechting::ArmType& arm,
                                   const ArmJoints& joints);
        virtual ~SoechtingNullspaceGradient() = default;

        void init(CompositeDiffIK::Parameters&) override;
        Eigen::VectorXf getGradient(CompositeDiffIK::Parameters& params, int stepNr) override;

        RobotNodeSetPtr rns;
        CompositeDiffIK::TargetPtr target;
        VirtualRobot::RobotNodePtr shoulder;
        Soechting::ArmType arm;
        ArmJoints joints;

        ShoulderAngles calcShoulderAngles(const CompositeDiffIK::Parameters& params) const;
    };

    typedef std::shared_ptr<SoechtingNullspaceGradient> SoechtingNullspaceGradientPtr;

} // namespace VirtualRobot
