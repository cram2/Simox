#include "ArticulatedObjectDoorHelper.h"

#include <string>

#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RobotNodeSet.h>

#include <GeometricPlanning/assert/assert.h>


namespace simox::geometric_planning
{
    ArticulatedObjectDoorHelper::ArticulatedObjectDoorHelper(const VirtualRobot::RobotPtr& object,
                                                             const Params& params) :
        object(object), params(params)
    {
    }

    ArticulatedObjectDoorHelper::DoorInteractionContext
    ArticulatedObjectDoorHelper::planInteraction(const std::string& nodeSetName) const
    {
        const auto rns = object->getRobotNodeSet(nodeSetName);
        REQUIRE_MESSAGE(rns != nullptr, "Robot node set `" << nodeSetName << "` does not exist!");

        const std::string jointNodeName = nodeSetName + constants::JointSuffix;
        const std::string handleNodeName = nodeSetName + constants::HandleSuffix;
        const std::string surfaceProjectionNodeName = nodeSetName + constants::SurfaceSuffix;

        const auto checkNodeExists = [&rns, &nodeSetName](const std::string& nodeName)
        {
            REQUIRE_MESSAGE(rns->hasRobotNode(nodeName),
                            "Robot node `" << nodeName + "` does not exist within robot node set `"
                                           << nodeSetName << "`!");
        };

        for (const auto& nodeName : {jointNodeName, handleNodeName, surfaceProjectionNodeName})
        {
            checkNodeExists(nodeName);
        }

        REQUIRE_MESSAGE(params.doorContactHandleDistance > 0.,
                        "Grasping from the other side not implemented yet!");

        return DoorInteractionContext{
            .rns = {.joint = rns->getNode(jointNodeName),
                    .handle = rns->getNode(handleNodeName),
                    .handleSurfaceProjection = rns->getNode(surfaceProjectionNodeName)},
            .handleSurfaceProjection_T_door_initial_contact =
                Pose(Eigen::Translation3f{0.F, -params.doorContactHandleDistance, 0.F}),
            .door_initial_contact_T_pre_contact =
                Pose(Eigen::Translation3f{0.F, 0, params.preContactDistance})};
    }

    ArticulatedObjectDoorHelper::DoorInteractionContextExtended
    ArticulatedObjectDoorHelper::planInteractionExtended(const std::string& nodeSetName,
                                                         const Pose& global_T_tcp_in_contact) const
    {
        const auto interactionInfo = planInteraction(nodeSetName);

        DoorInteractionContextExtended extendedInfo;
        extendedInfo.door_initial_contact_T_pre_contact =
            interactionInfo.door_initial_contact_T_pre_contact;
        extendedInfo.rns = interactionInfo.rns;
        extendedInfo.handleSurfaceProjection_T_door_initial_contact =
            interactionInfo.handleSurfaceProjection_T_door_initial_contact;

        extendedInfo.handleSurfaceProjection_T_tcp_at_handle =
            handleSurfaceProjection_T_tcp_at_handle(interactionInfo, global_T_tcp_in_contact);

        return extendedInfo;
    }

    Pose
    ArticulatedObjectDoorHelper::handleSurfaceProjection_T_tcp_at_handle(
        const DoorInteractionContext& interactionInfo,
        const Pose& global_T_tcp_in_contact) const
    {
        const Pose global_T_handleSurfaceProjection(
            interactionInfo.rns.handleSurfaceProjection->getGlobalPose());

        // z points away from the door, y upwards, x to the side
        const Pose door_surface_T_tcp =
            global_T_handleSurfaceProjection.inverse() * global_T_tcp_in_contact;

        // VR_INFO << VAROUT(door_surface_T_tcp.translation());

        const float initialDoorDistance = door_surface_T_tcp.translation().z();
        // VR_INFO << "Initial door distance " << initialDoorDistance;

        return Pose(Eigen::Translation3f{0, 0, initialDoorDistance});
    }
} // namespace simox::geometric_planning
