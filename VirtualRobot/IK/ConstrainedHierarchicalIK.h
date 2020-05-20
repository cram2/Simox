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
* @author     Peter Kaiser
* @copyright  2015 Peter Kaiser
*             GNU Lesser General Public License
*
*/
#pragma once

#include "VirtualRobot/VirtualRobot.h"
#include "VirtualRobot/IK/ConstrainedIK.h"
#include "VirtualRobot/IK/HierarchicalIK.h"


namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT ConstrainedHierarchicalIK : public ConstrainedIK, public std::enable_shared_from_this<ConstrainedHierarchicalIK>
    {
    public:
        ConstrainedHierarchicalIK(RobotPtr& robot, const RobotNodeSetPtr& nodeSet, float stepSize = 0.2f, int maxIterations = 1000, float stall_epsilon = 0.0001, float raise_epsilon = 0.1);

        bool initialize() override;
        bool solveStep() override;

    protected:
        RobotNodeSetPtr nodeSet;
        HierarchicalIKPtr ik;

        std::vector<JacobiProviderPtr> jacobians;

        float stepSize;
        Eigen::VectorXf lastDelta;
    };

    typedef std::shared_ptr<ConstrainedHierarchicalIK> ConstrainedHierarchicalIKPtr;
}

