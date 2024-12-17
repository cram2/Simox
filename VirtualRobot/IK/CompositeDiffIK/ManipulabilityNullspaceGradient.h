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
#include <memory>
#include <set>

#include <VirtualRobot/Manipulability/AbstractManipulabilityTracking.h>

#include "CompositeDiffIK.h"

namespace VirtualRobot
{
    class NullspaceManipulability : public CompositeDiffIK::NullspaceGradient
    {
    public:
        NullspaceManipulability(AbstractManipulabilityTrackingPtr manipulabilityTracking,
                                const Eigen::MatrixXd& manipulabilityDesired,
                                const Eigen::MatrixXd& gainMatrix = Eigen::MatrixXd(),
                                bool jointLimitAvoidance = false);

        AbstractManipulabilityTrackingPtr manipulabilityTracking;
        Eigen::MatrixXd manipulabilityDesired;
        Eigen::MatrixXd gainMatrix;
        bool jointLimitAvoidance;

        double computeDistance();

        void init(CompositeDiffIK::Parameters&) override;
        Eigen::VectorXf getGradient(CompositeDiffIK::Parameters& params, int stepNr) override;

        std::vector<CompositeDiffIK::NullspaceTargetStep> ikSteps;
    };
    typedef std::shared_ptr<class NullspaceManipulability> NullspaceManipulabilityPtr;
} // namespace VirtualRobot
