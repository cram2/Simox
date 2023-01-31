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
* @author     Rainer Kartmann
* @copyright  2022 Rainer Kartmann
*             GNU Lesser General Public License
*/
#pragma once

#include "../VirtualRobot.h"

#include "RobotNode.h"
#include "HemisphereJoint/Joint.h"

#include <Eigen/Core>

#include <string>
#include <vector>
#include <optional>


namespace VirtualRobot
{

    using RobotNodeHemispherePtr = std::shared_ptr<class RobotNodeHemisphere>;

    class VIRTUAL_ROBOT_IMPORT_EXPORT RobotNodeHemisphere : public RobotNode
    {
    public:

        enum class Role
        {
            FIRST,
            SECOND,
        };
        static Role RoleFromString(const std::string& string);

        struct XmlInfo
        {
            Role role;

            // Only set for first:
            double theta0 = -1;
            double lever = -1;
        };

        friend class RobotFactory;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW


        RobotNodeHemisphere(
                RobotWeakPtr rob,                                   ///< The robot
                const std::string& name,                            ///< The name
                float jointLimitLo,                                 ///< lower joint limit
                float jointLimitHi,                                 ///< upper joint limit
                const Eigen::Matrix4f& preJointTransform,           ///< This transformation is applied before the translation of the joint is done
                const Eigen::Vector3f& axis,                        ///< The rotation axis (in local joint coord system)
                VisualizationNodePtr visualization = nullptr,       ///< A visualization model
                CollisionModelPtr collisionModel = nullptr,         ///< A collision model
                float jointValueOffset = 0.0f,                      ///< An offset that is internally added to the joint value
                const SceneObject::Physics& p = {},                 ///< physics information
                CollisionCheckerPtr colChecker = nullptr,           ///< A collision checker instance (if not set, the global col checker is used)
                RobotNodeType type = Generic
                );

        RobotNodeHemisphere(
                RobotWeakPtr rob,                                   ///< The robot
                const std::string& name,                            ///< The name
                float jointLimitLo,                                 ///< lower joint limit
                float jointLimitHi,                                 ///< upper joint limit
                float a,                                            ///< dh paramters
                float d,                                            ///< dh paramters
                float alpha,                                        ///< dh paramters
                float theta,                                        ///< dh paramters
                VisualizationNodePtr visualization = nullptr,       ///< A visualization model
                CollisionModelPtr collisionModel = nullptr,         ///< A collision model
                float jointValueOffset = 0.0f,                      ///< An offset that is internally added to the joint value
                const SceneObject::Physics& p = {},                 ///< physics information
                CollisionCheckerPtr colChecker = {},                ///< A collision checker instance (if not set, the global col checker is used)
                RobotNodeType type = Generic
                );

    public:

        ~RobotNodeHemisphere() override;


        void setXmlInfo(const XmlInfo& info);

        bool
        initialize(
                SceneObjectPtr parent = nullptr,
                const std::vector<SceneObjectPtr>& children = {}
                ) override;

        /// Print status information.
        void
        print(
                bool printChildren = false,
                bool printDecoration = true
                ) const override;

        bool
        isHemisphereJoint() const override;


    protected:

        RobotNodeHemisphere();

        /// Derived classes add custom XML tags here
        std::string
        _toXML(
                const std::string& modelPath
                ) override;

        /// Checks if nodeType constraints are fulfilled. Otherwise an exception is thrown.
        /// Called on initialization.
        void
        checkValidRobotNodeType() override;

        void
        updateTransformationMatrices(
                const Eigen::Matrix4f& parentPose
                ) override;

        RobotNodePtr
        _clone(const RobotPtr newRobot,
               const VisualizationNodePtr visualizationModel,
               const CollisionModelPtr collisionModel,
               CollisionCheckerPtr colChecker,
               float scaling
               ) override;


    public:

        struct JointMath
        {
            /// The actuator values that were used to compute the joint math.
            Eigen::Vector2f actuators = Eigen::Vector2f::Constant(std::numeric_limits<float>::min());
            /// The joint math.
            hemisphere::Joint joint;

            void update(const Eigen::Vector2f& actuators);
        };

        struct First
        {
            JointMath math;
        };
        std::optional<First> first;

        struct Second
        {
            /// The first actuator node.
            RobotNodeHemisphere* first = nullptr;

            JointMath& math()
            {
                return first->first->math;
            }
            const JointMath& math() const
            {
                return first->first->math;
            }
        };
        std::optional<Second> second;

    };

} // namespace VirtualRobot
