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

#include <VirtualRobot/VirtualRobot.h>

#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/Nodes/HemisphereJoint/CachedMaths.h>
#include <VirtualRobot/Nodes/HemisphereJoint/Maths.h>

#include <Eigen/Core>

#include <string>
#include <vector>
#include <optional>


namespace VirtualRobot
{

    using RobotNodeHemispherePtr = std::shared_ptr<class RobotNodeHemisphere>;

    /**
     * @brief A model of the 2 DoF wrist joint mechanism published in:
     *
     * C. Klas and T. Asfour, "A Compact, Lightweight and Singularity-Free
     * Wrist Joint Mechanism for Humanoid Robots," 2022 IEEE/RSJ International
     * Conference on Intelligent Robots and Systems (IROS), Kyoto, Japan, 2022,
     * pp. 457-464, doi: 10.1109/IROS47612.2022.9981787.
     *
     * This joint mechanism represents 2 Degrees of Freedom (DoF). Therefore,
     * it must be represented by two robot nodes in VirtualRobot.
     */
    class VIRTUAL_ROBOT_IMPORT_EXPORT RobotNodeHemisphere : public RobotNode
    {
    public:

        enum class Role
        {
            /// The first DoF in the kinematic chain.
            FIRST,
            /// The second DoF in the kinematic chain.
            SECOND,
        };
        static Role RoleFromString(const std::string& string);

        /// Information specified in the robot XML.
        struct XmlInfo
        {
            Role role;

            // Only set for first:
            double theta0 = -1;
            double lever = -1;
        };


        /// Data held by the first joint.
        struct FirstData
        {
            hemisphere::CachedMaths maths;
        };

        /// Data held by the second joint.
        struct SecondData
        {
            /// The first actuator node.
            RobotNodeHemisphere* firstNode = nullptr;
            RobotNodeHemisphere* secondNode = nullptr;

            hemisphere::CachedMaths& maths();
            const hemisphere::CachedMaths& maths() const;

            hemisphere::Maths::Jacobian getJacobian() const;
        };

        friend class RobotFactory;


    public:

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

        bool isFirstHemisphereJointNode() const;
        bool isSecondHemisphereJointNode() const;

        /**
         * @brief Get the data held by the second node.
         * May only be called if `isFirstHemisphereJointNode()`;
         */
        SecondData& getSecondData();
        const SecondData& getSecondData() const;

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


    private:

        std::optional<FirstData> firstData;
        std::optional<SecondData> secondData;

    };

} // namespace VirtualRobot

