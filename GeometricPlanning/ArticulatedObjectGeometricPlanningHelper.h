#pragma once

#include <Eigen/Geometry>

#include <GeometricPlanning/ParametricPath.h>
#include <GeometricPlanning/path_primitives/PathPrimitive.h>
#include <GeometricPlanning/types.h>
#include <VirtualRobot/VirtualRobot.h>

namespace simox::geometric_planning
{

    class ArticulatedObjectGeometricPlanningHelper
    {
    public:
        ArticulatedObjectGeometricPlanningHelper(const VirtualRobot::RobotPtr& articulatedObject);

        /**
         * @brief Compute the parametric path of a node that results when moving a joint from lower to upper limit.
         *        It is assumed that only one parent joint exists that is movable.
         *
         *
         * @param node
         * @param joint
         * @return ParametricPath node pose depending on joint state
         */
        ParametricPath getPathForNode(const std::string& node) const;


        /**
         * @brief Compute the parametric path of a node that results when moving a joint from lower to upper limit.
         *
         *
         * @param node
         * @param joint
         * @return ParametricPath node pose depending on joint state
         */
        ParametricPath getPathForNode(const std::string& node, const std::string& joint) const;

        /**
         * @brief Compute the parametric path of a node that results when moving a joint from lower to upper limit.
         *
         *
         * @param node
         * @param joint
         * @return ParametricPath node pose depending on joint state
         */
        ParametricPath getPathForNode(const VirtualRobot::RobotNodePtr& node,
                                      const VirtualRobot::RobotNodePtr& joint) const;

    private:
        ParametricPath createCircularPath(const VirtualRobot::RobotNodePtr& node,
                                          const VirtualRobot::RobotNodePtr& joint) const;

        ParametricPath createLinearPath(const VirtualRobot::RobotNodePtr& node,
                                        const VirtualRobot::RobotNodePtr& joint) const;

        const VirtualRobot::RobotPtr articulatedObject;
    };
} // namespace simox::geometric_planning
