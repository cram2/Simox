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
* @author     Fabian Reister ( fabian dot reister at kit dot edu )
* @date       2022
* @copyright  GNU Lesser General Public License
*
*/

#pragma once

#include <vector>

#include <Eigen/Geometry>


namespace simox::geometric_planning
{
    using Pose = Eigen::Isometry3f;
    using Poses = std::vector<Pose>;

    using Position = Eigen::Vector3f;

} // namespace armarx::manipulation::core
