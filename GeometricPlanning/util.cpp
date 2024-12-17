/**
 * This file is part of ArmarX.
 *
 * ArmarX is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * ArmarX is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * @author     Fabian Reister ( fabian dot reister at kit dot edu )
 * @date       2023
 * @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
 *             GNU General Public License
 */

#include "util.h"

#include <algorithm>
#include <optional>
#include <tuple>
#include <vector>

#include <SimoxUtility/algorithm/string/string_tools.h>
#include <VirtualRobot/Grasping/GraspSet.h>
#include <VirtualRobot/Robot.h>

#include "VirtualRobot/Grasping/Grasp.h"
#include "VirtualRobot/Nodes/RobotNode.h"

namespace simox::geometric_planning
{


    std::vector<std::string>
    nodesMatching(const std::vector<std::string>& allNodeNames, const std::string& identifier)
    {

        std::vector<std::string> handleNodeNames;
        std::copy_if(allNodeNames.begin(),
                     allNodeNames.end(),
                     std::back_inserter(handleNodeNames),
                     [&identifier](const auto& nodeName)
                     {
                         // the second check is a bit hacky: we can define transformation nodes in URDF which might match, e.g. "parent->child"
                         return simox::alg::ends_with(nodeName, identifier) and
                                not simox::alg::contains(nodeName, '>');
                     });

        return handleNodeNames;
    }

    std::optional<std::tuple<VirtualRobot::RobotNodePtr, VirtualRobot::GraspPtr>>
    findGraspByName(const VirtualRobot::Robot& robot,
                    const std::string& graspSetName,
                    const std::string& graspName)
    {
        for (const auto& [node, graspSets] : allGraspSets(robot))
        {
            if (not graspSets.empty())
            {
                // ARMARX_VERBOSE << "Found grasp sets attached to node " << node->getName();
                for (const auto& graspSet : graspSets)
                {
                    // check if grasp set name matches if provided
                    if (not graspSetName.empty() and graspSet->getName() != graspSetName)
                    {
                        continue;
                    }

                    for (const auto& grasp : graspSet->getGrasps())
                    {
                        if (graspName == grasp->getName())
                        {
                            return std::make_tuple(node, grasp);
                        }
                    }
                }
            }
        }

        return std::nullopt;
    }

    std::map<VirtualRobot::RobotNodePtr, std::vector<VirtualRobot::GraspSetPtr>>
    allGraspSets(const VirtualRobot::Robot& robot)
    {
        std::map<VirtualRobot::RobotNodePtr, std::vector<VirtualRobot::GraspSetPtr>> nodeGraspSets;

        for (const auto& node : robot.getRobotNodes())
        {
            const auto graspSets = node->getAllGraspSets();
            for (const auto& graspSet : graspSets)
            {
                nodeGraspSets[node].push_back(graspSet);
            }
        }

        return nodeGraspSets;
    }

} // namespace simox::geometric_planning
