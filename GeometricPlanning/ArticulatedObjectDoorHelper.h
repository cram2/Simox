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
 * @date       2022
 * @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
 *             GNU General Public License
 */

#pragma once

#include <optional>
#include <string>

#include <GeometricPlanning/types.h>
#include <VirtualRobot/VirtualRobot.h>

namespace simox::geometric_planning
{

    namespace constants
    {
        inline const std::string JointSuffix = "joint";
        inline const std::string HandleSuffix = "handle";
        inline const std::string SurfaceSuffix = "handle_surface_projection";
    } // namespace constants

    class ArticulatedObjectDoorHelper
    {
    public:
        struct Params
        {
            float preContactDistance = 300; // [mm] shift into z direction

            float doorContactHandleLateralShift = 0; // [mm] shift along handle, x direction
            float doorContactHandleDistance = 0; // [mm]; shift along y direction
        };

        struct NamedRobotNodeSet
        {
            VirtualRobot::RobotNodePtr joint;
            VirtualRobot::RobotNodePtr handle;
            VirtualRobot::RobotNodePtr handleSurfaceProjection;
        };

        struct DoorInteractionContext
        {
            NamedRobotNodeSet rns;

            Pose handleSurfaceProjection_T_door_initial_contact;

            Pose door_initial_contact_T_pre_contact;
        };

        struct DoorInteractionContextExtended : DoorInteractionContext
        {
            Pose handleSurfaceProjection_T_tcp_at_handle;
        };

        ArticulatedObjectDoorHelper(const VirtualRobot::RobotPtr& object, const Params& params);

        DoorInteractionContext planInteraction(
            const std::string& nodeSetName,
            const std::optional<std::string>& targetFrameSuffix = std::nullopt,
            const std::optional<std::string>& surfaceProjectionFrameSuffix = std::nullopt) const;

        DoorInteractionContextExtended planInteractionExtended(
            const std::string& nodeSetName,
            const Pose& global_T_tcp_in_contact,
            const std::optional<std::string>& targetFrameSuffix = std::nullopt,
            const std::optional<std::string>& surfaceProjectionFrameSuffix = std::nullopt) const;


    protected:
        Pose handleSurfaceProjection_T_tcp_at_handle(const DoorInteractionContext& interactionInfo,
                                                     const Pose& global_T_tcp_in_contact) const;

    private:
        VirtualRobot::RobotPtr object;

        const Params params;
    };
} // namespace simox::geometric_planning
