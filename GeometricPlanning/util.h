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

#pragma once


#include <map>
#include <optional>

#include <VirtualRobot/VirtualRobot.h>

namespace simox::geometric_planning
{

    std::vector<std::string> nodesMatching(const std::vector<std::string>& allNodeNames,
                                           const std::string& identifier);

    std::optional<std::tuple<VirtualRobot::RobotNodePtr, VirtualRobot::GraspPtr>>
    findGraspByName(const VirtualRobot::Robot& robot,
                    const std::string& graspSetName,
                    const std::string& graspName);

    std::map<VirtualRobot::RobotNodePtr, std::vector<VirtualRobot::GraspSetPtr>>
    allGraspSets(const VirtualRobot::Robot& robot);


} // namespace simox::geometric_planning
