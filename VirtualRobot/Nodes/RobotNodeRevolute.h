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
* @author     Manfred Kroehnert, Nikolaus Vahrenkamp
* @copyright  2010,2011 Manfred Kroehnert, Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#pragma once

#include "../VirtualRobot.h"

#include "RobotNode.h"

#include <Eigen/Core>

#include <string>
#include <vector>


namespace VirtualRobot
{
    class Robot;

    class VIRTUAL_ROBOT_IMPORT_EXPORT RobotNodeRevolute : public RobotNode
    {
    public:
        friend class RobotFactory;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW


        RobotNodeRevolute(RobotWeakPtr rob,                                      //!< The robot
                          const std::string& name,                            //!< The name
                          float jointLimitLo,                                 //!< lower joint limit
                          float jointLimitHi,                                 //!< upper joint limit
                          const Eigen::Matrix4f& preJointTransform,           //!< This transformation is applied before the translation of the joint is done
                          const Eigen::Vector3f& axis,                        //!< The rotation axis (in local joint coord system)
                          VisualizationNodePtr visualization = VisualizationNodePtr(),    //!< A visualization model
                          CollisionModelPtr collisionModel = CollisionModelPtr(),         //!< A collision model
                          float jointValueOffset = 0.0f,                                  //!< An offset that is internally added to the joint value
                          const SceneObject::Physics& p = SceneObject::Physics(),         //!< physics information
                          CollisionCheckerPtr colChecker = CollisionCheckerPtr(),         //!< A collision checker instance (if not set, the global col checker is used)
                          RobotNodeType type = Generic);
        RobotNodeRevolute(RobotWeakPtr rob,                                      //!< The robot
                          const std::string& name,                            //!< The name
                          float jointLimitLo,                                 //!< lower joint limit
                          float jointLimitHi,                                 //!< upper joint limit
                          float a,                                            //!< dh paramters
                          float d,                                            //!< dh paramters
                          float alpha,                                        //!< dh paramters
                          float theta,                                        //!< dh paramters
                          VisualizationNodePtr visualization = VisualizationNodePtr(),//!< A visualization model
                          CollisionModelPtr collisionModel = CollisionModelPtr(),     //!< A collision model
                          float jointValueOffset = 0.0f,                              //!< An offset that is internally added to the joint value
                          const SceneObject::Physics& p = SceneObject::Physics(),     //!< physics information
                          CollisionCheckerPtr colChecker = CollisionCheckerPtr(),     //!< A collision checker instance (if not set, the global col checker is used)
                          RobotNodeType type = Generic);

        ~RobotNodeRevolute() override;

        bool initialize(SceneObjectPtr parent = nullptr, const std::vector<SceneObjectPtr>& children = {}) override;

        //! Print status information.
        void print(bool printChildren = false, bool printDecoration = true) const override;

        bool isRotationalJoint() const override;
        /*!
            Standard: In global coordinate system.
            \param coordSystem When not set the axis is transformed to global coordinate system. Otherwise any scene object can be used as coord system.
        */
        Eigen::Vector3f getJointRotationAxis(const SceneObjectPtr coordSystem = nullptr) const;

        //! This is the original joint axis, without any transformations applied.
        Eigen::Vector3f getJointRotationAxisInJointCoordSystem() const;

        /*!
         * \brief getLMTC Calculates the spatial distance between the parent of a revolute joint and a given child with the joint set to a given angle (e.g. the length of a muscle-tendon complex attached to the parent and the given child)
         * \param child The child node
         * \param angle The angle of the revolute joint in radians
         * \return The spatial distance between parent and given child at given angle
         */
        virtual float getLMTC(float angle);

        /*!
         * \brief getLMomentArm Calculates the spatial length of a moment arm defined through the triangle given by the node's parent, the specified child and the specified angle at the revolulte joint.
         * \param child The child node
         * \param angle The angle of the revolute joint in radians
         * \return The spatial length of the moment arm
         */
        virtual float getLMomentArm(float angle);

        void setJointRotationAxis(Eigen::Vector3f newAxis);


    protected:

        RobotNodeRevolute() {};

        //! Derived classes add custom XML tags here
        std::string _toXML(const std::string& modelPath) override;

        /*!
            Can be called by a RobotNodeActuator in order to set the pose of this node.
            This is useful, if the node is actuated externally, i.e. via a physics engine.
            \param globalPose The new global pose. The joint value is *not* determined from this pose. The RobotNodeActuator is responsible for setting the corresponding joint value
            \param updateChildren Usually it is assumed that all RobotNodes are updated this way (updateChildren=false). If not, the children poses can be updated according to this node (updateCHildren=true).
        */
        void updateVisualizationPose(const Eigen::Matrix4f& globalPose, bool updateChildren = false) override;

        //! Checks if nodeType constraints are fulfilled. Otherwise an exception is thrown. Called on initialization.
        void checkValidRobotNodeType() override;

        void updateTransformationMatrices(const Eigen::Matrix4f& parentPose) override;

        RobotNodePtr _clone(const RobotPtr newRobot, const VisualizationNodePtr visualizationModel,
                            const CollisionModelPtr collisionModel, CollisionCheckerPtr colChecker,
                            float scaling) override;


    protected:

        Eigen::Vector3f jointRotationAxis;  // eRevoluteJoint  (given in local joint coord system)
        Eigen::Matrix4f tmpRotMat;

    };

    typedef std::shared_ptr<RobotNodeRevolute> RobotNodeRevolutePtr;

} // namespace VirtualRobot

