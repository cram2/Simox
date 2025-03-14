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
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#pragma once

#include <optional>
#include <string>

#include <eigen3/Eigen/Core>

#include <SimoxUtility/math/scale_value.h>

#include "VirtualRobot/GraspableSensorizedObject.h"
#include "VirtualRobot/Transformation/DHParameter.h"
#include "VirtualRobot/VirtualRobot.h"

namespace VirtualRobot
{
    class RobotNodeActuator;

    /*!
        Each RobotNode may consider two transformations:
        * localTransformation: This transformation is fixed.
        * jointTransformation: This is the flexible part of the joint. Here, the jointValue is used to compute the transformation according to the implementation of the joint type.

        The visualization (of limb and/or coordinateAxis) is linked to the local coordinate sysytem of this joint: localTransformation*jointTransformation
        The global pose of this joint is the same: localTransformation*jointTransformation
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT RobotNode : public GraspableSensorizedObject
    {
    public:
        friend class Robot;
        friend class LocalRobot;
        friend class RobotIO;
        friend class RobotNodeSet;
        friend class RobotConfig;
        friend class RobotFactory;
        friend class RobotNodeActuator;
        friend class ColladaIO;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        //! Experimental: Additional information about the node
        enum RobotNodeType
        {
            Generic, //! No constraints
            Joint, //! Only pre-joint transformations allowed (e.g. DH-theta for revolute joints). No visualization/collision models.
            Body, //! No transformations allowed. Only visualization and/or collision models together with physic information.
            Transform //! Only transformations allowed. No joint or model tags.
        };

        /*!
            Constructor with settings.
        */
        RobotNode(RobotWeakPtr rob,
                  const std::string& name,
                  float jointLimitLo,
                  float jointLimitHi,
                  VisualizationNodePtr visualization = nullptr,
                  CollisionModelPtr collisionModel = nullptr,
                  float jointValueOffset = 0.0f,
                  const SceneObject::Physics& physics = {},
                  CollisionCheckerPtr colChecker = nullptr,
                  RobotNodeType type = Generic);

        ~RobotNode() override;


        RobotPtr getRobot() const;

        /*!
            Set a joint value [rad].
            The internal matrices and visualizations are updated accordingly.
            If you intend to update multiple joints, use \ref setJointValueNoUpdate(float) for faster access.
        */
        virtual void setJointValue(float q);

        /*!
            All children and their children (and so on) are collected.
            The current instance is also added.
        */
        void collectAllRobotNodes(std::vector<RobotNodePtr>& storeNodes);

        virtual float getJointValue() const;

        /*!
            Checks if jointValue is within joint limits. If not jointValue is adjusted.
        */
        void respectJointLimits(float& jointValue) const;

        /*!
            Checks if jointValue is within joint limits. If verbose is set a warning is printed.
        */
        bool checkJointLimits(float jointValue, bool verbose = false) const;

        /*!
            The preJoint/preVisualization transformation. This transformation is applied before the joint and the visualization.
        */
        virtual Eigen::Matrix4f
        getLocalTransformation()
        {
            return localTransformation;
        }

        using SceneObject::getChildren;

        template <class T>
        auto
        getChildren()
        {
            std::vector<std::shared_ptr<T>> result;
            for (const auto& c : getChildren())
            {
                RobotNodePtr n = std::dynamic_pointer_cast<T>(c);
                if (n)
                {
                    result.emplace_back(n);
                }
            }
            return result;
        }

        /*!
            Initialize robot node. Here pointers to robot and children are created from names.
            Be sure all children are created and registered to robot before calling initialize.
            Usually RobotFactory manages the initialization.
        */
        bool initialize(
            SceneObjectPtr parent = SceneObjectPtr(),
            const std::vector<SceneObjectPtr>& children = std::vector<SceneObjectPtr>()) override;

        /*!
            Calling this method will cause an exception, since RobotNodes are controlled via joint values.
        */
        void setGlobalPose(const Eigen::Matrix4f& pose) override;

        // Make all all overloads of getGlobalPose from SceneObject avaliable in RobotNode.
        // If the using statement is not present RobotNode::getGlobalPose(const Eigen::Matrix4f& localPose) cannot be called.
        using SceneObject::getGlobalPose;
        /*
            This call locks the robot's mutex.
        */
        Eigen::Matrix4f getGlobalPose() const override;

        /*!
            The pose of this node in the root coordinate system of the robot.
            \return The pose in root frame

        */
        virtual Eigen::Matrix4f getPoseInRootFrame() const;
        virtual Eigen::Matrix4f getPoseInFrame(const RobotNodePtr& frame) const;

        /*!
            The position of this node in the root coordinate system of the robot.
            \return The position in root frame

        */
        virtual Eigen::Vector3f getPositionInRootFrame() const;
        virtual Eigen::Vector3f getPositionInFrame(const RobotNodePtr& frame) const;

        virtual Eigen::Matrix3f getOrientationInRootFrame() const;


        virtual Eigen::Matrix4f getPoseInRootFrame(const Eigen::Matrix4f& localPose) const;
        virtual Eigen::Vector3f getPositionInRootFrame(const Eigen::Vector3f& localPosition) const;
        virtual Eigen::Vector3f getDirectionInRootFrame(const Eigen::Vector3f& localPosition) const;
        virtual Eigen::Matrix3f
        getOrientationInRootFrame(const Eigen::Matrix3f& localOrientation) const;

        /*!
            Display the coordinate system of this RobotNode. This is the global pose of it's visualization.
            \p enable Show or hide coordinate system
            \p scaling Size of coordinate system
            \p text Text to display at coordinate system. If not given, the name of this robot node will be displayed.
            \p izationType    This option is only needed when the current robot node does not yet own a visualization.
                                    Then a visualziationNode has to be built and the \p visualizationType specifies which type of visualization should be used.
                                    If not given, the first registered visaulizationfactory is used.
        */
        void showCoordinateSystem(bool enable,
                                  float scaling = 1.0f,
                                  std::string* text = NULL,
                                  const std::string& visualizationType = "") override;


        /*!
            Print status information.
        */
        void print(bool printChildren = false, bool printDecoration = true) const override;

        float getJointLimitLo();
        float getJointLimitHi();

        struct JointLimits
        {
            float low;
            float high;
        };

        JointLimits
        getJointLimits()
        {
            return JointLimits{.low = getJointLimitLo(), .high = getJointLimitHi()};
        }

        /*!
            Set joint limits [rad].
        */
        virtual void setJointLimits(float lo, float hi);

        bool isJoint() const noexcept;
        virtual bool isTranslationalJoint() const noexcept;
        virtual bool isRotationalJoint() const noexcept;
        virtual bool isHemisphereJoint() const noexcept;
        virtual bool isFourBarJoint() const noexcept;

        /**
         * @param limitless wheter this node has joint limits or not.
         */
        virtual void setLimitless(bool limitless);
        bool isLimitless() const noexcept;

        /**
         * @param target the target joint value in [rad]
         * @return the signed distance between current and target joint values in [rad].
         *         If the given target value violates joint limits or this robotnode is not a joint, 0.0f is returned instead.
         */
        virtual float getDelta(float target) const;

        /*!
            Visualize the structure of this RobotNode.
            \p enable Show or hide the structure visualization
            \p visualizationType    This option is only needed when the current robot node does not yet own a visualization.
                                    Then a visualziationNode has to be build and the \p visualizationType specifies which type of visualization should be used.
                                    If not given, the first registered visaulizationfactory is used.
        */
        virtual void showStructure(bool enable, const std::string& visualizationType = "");


        /*!
            Find all robot nodes whose movements affect this RobotNode
            \param rns If set, the search is limited to the rns.
        */
        virtual std::vector<RobotNodePtr> getAllParents(RobotNodeSetPtr rns = RobotNodeSetPtr());

        /*!
            Clone this RobotNode.
            \param newRobot The newly created RobotNode belongs to newRobot.
            \param cloneChildren If true, all children are cloned (and their children, etc).
            \param initializeWithParent If given, the RobotNode is initialized with this parent.
            \param colChecker Must only be set if the cloned RobotNode should be registered to a different collision checker instance.
            \param scaling Scale Can be set to create a scaled version of this robot. Scaling is applied on kinematic, visual, and collision data.
        */
        virtual RobotNodePtr clone(RobotPtr newRobot,
                                   bool cloneChildren = true,
                                   RobotNodePtr initializeWithParent = RobotNodePtr(),
                                   CollisionCheckerPtr colChecker = CollisionCheckerPtr(),
                                   std::optional<float> scaling = std::nullopt,
                                   bool preventCloningMeshesIfScalingIs1 = false);

        inline float
        getJointValueOffset() const noexcept
        {
            return jointValueOffset;
        }

        inline float
        getJointLimitHigh() const noexcept
        {
            return jointLimitHi;
        }

        inline float
        getJointLimitLow() const noexcept
        {
            return jointLimitLo;
        }

        inline float
        scaleJointValue(float val, float min = -1, float max = 1)
        {
            return simox::math::scale_value_from_to(val, jointLimitLo, jointLimitHi, min, max);
        }

        inline float
        unscaleJointValue(float val, float min = -1, float max = 1)
        {
            return simox::math::scale_value_from_to(val, min, max, jointLimitLo, jointLimitHi);
        }

        inline float
        getScaledJointValue(float min = -1, float max = 1)
        {
            return scaleJointValue(getJointValue(), min, max);
        }

        inline void
        setScaledJointValue(float val, float min = -1, float max = 1)
        {
            setJointValue(unscaleJointValue(val, min, max));
        }

        inline void
        setScaledJointValueNoUpdate(float val, float min = -1, float max = 1)
        {
            setJointValueNoUpdate(unscaleJointValue(val, min, max));
        }

        /*!
            Set maximum velocity of this joint in rad/s or m/s.
            To disable max. velocity set to -1.0f.
        */
        virtual void setMaxVelocity(float maxVel);

        /*!
            Set maximum acceleration pf this joint in rad/s^2 or m/s^2.
            To disable max. acceleration set to -1.0f.
        */
        virtual void setMaxAcceleration(float maxAcc);

        /*!
            Set maximum deceleration pf this joint in rad/s^2 or m/s^2.
            To disable max. deceleration set to -1.0f.
        */
        virtual void setMaxDeceleration(float maxAcc);

        /*!
            Set maximum jerk for this joint in rad/s^3 or m/s^3.
            To disable max. jerk, set to -1.0f.
        */
        virtual void setMaxJerk(float maxTo);

        /*!
            Set maximum torque pf this joint in Nm.
            To disable max. torque set to -1.0f.
        */
        virtual void setMaxTorque(float maxTo);

        /*!
            Maximum velocity in rad/s or m/s.
            Returns -1.0f if no maxium value is set.
        */
        float getMaxVelocity() const noexcept;

        /*!
            Maximum acceleration in rad/s^2 or m/s^2.
            Returns -1.0f if no maxium value is set.
        */
        float getMaxAcceleration() const noexcept;

        /*!
            Maximum acceleration in Nm.
            Returns -1.0f if no maxium value is set.
        */
        float getMaxTorque() const noexcept;

        /*!
            Maximum jerk in rad/s^3 or m/s^3.
            Returns -1.0f if no maxium value is set.
        */
        float getMaxJerk() const noexcept;


        RobotNodeType getType() const noexcept;

        //! Forbid cloning method from SceneObject. We need to know the new robot for cloning
        SceneObjectPtr clone(const std::string& /*name*/,
                             CollisionCheckerPtr /*colChecker*/ = CollisionCheckerPtr(),
                             float /*scaling*/ = 1.0f) const;

        /*!
            When joint was created via DH parameters, they can be accessed here.
        */
        const DHParameter&
        getOptionalDHParameters() const noexcept
        {
            return optionalDHParameter;
        }

        /*!
            Compute/Update the transformations of this joint and all child joints. Therefore the parent is queried for its pose.
            This method is called by the robot in order to update the pose matrices.
        */
        void updatePose(bool updateChildren = true) override;

        void copyPoseFrom(const SceneObjectPtr& sceneobj) override;

        void copyPoseFrom(const RobotNodePtr& other);

        /*!
            Automatically propagate the joint value to another joint.
            \param jointName The name of the other joint. must be part of the same robot.
            \param factor The propagated joint value is scaled according to this value.

        */
        virtual void propagateJointValue(const std::string& jointName, float factor = 1.0f);

        /*!
            Creates an XML string that defines the robotnode. Filenames of all visualization models are set to modelPath/RobotNodeName_visu and/or modelPath/RobotNodeName_colmodel.
            @see RobotIO::saveXML.
        */
        virtual std::string toXML(const std::string& basePath,
                                  const std::string& modelPathRelative = "models",
                                  bool storeSensors = true,
                                  bool storeModelFiles = true);

        /*!
            Set the local transformation matrix that is used in this node.
        */
        void setLocalTransformation(Eigen::Matrix4f& newLocalTransformation);

        /*!
            Set the joint value without updating the internal matrices.
            After setting all jointvalues the transformations are calculated by calling \ref applyJointValues()
            This method is used when multiple joints should be updated at once.
            Access by RobotNodeSets, Robots or RobotConfigs must be protected by a \ref WriteLock.
            \param q The joint value [rad] eventually clamped to limits.
        */
        virtual void setJointValueNoUpdate(float q);

        bool getEnforceJointLimits() const noexcept;
        void setEnforceJointLimits(bool value);

        bool getAllowJointLimitAvoidance() const noexcept;
        void setAllowJointLimitAvoidance(bool value);

        /*! Removes all sensors (for faster forward kinematics) */
        void removeAllSensors(bool recursive = true);

    protected:
        /*!
            Queries parent for global pose and updates visualization accordingly
        */
        virtual void updateTransformationMatrices();

        /*!
            Update the pose according to parent
        */
        void updatePose(const Eigen::Matrix4f& parentPose, bool updateChildren = true) override;


        /*!
            Can be called by a RobotNodeActuator in order to set the pose of the visualization, which means that the preJointTransform is ignored and
            the pose of the visualization can be set directly.
            This is useful, if the node is actuated externally, i.e. via a physics engine.
            todo: Protect such updates by a mutex.
            \param globalPose The new global pose. The joint value is *not* determined from this pose. The RobotNodeActuator is responsible for setting the corresponding joint value
            \param updateChildren Usually it is assumed that all RobotNodes are updated this way (updateChildren=false). If not, the children poses can be updated according to this node.
        */
        virtual void updateVisualizationPose(const Eigen::Matrix4f& globalPose,
                                             bool updateChildren = false);
        virtual void updateVisualizationPose(const Eigen::Matrix4f& globalPose,
                                             float jointValue,
                                             bool updateChildren = false);

        //! Checks if nodeType constraints are fulfilled. Otherwise an exception is thrown. Called on initialization.
        virtual void checkValidRobotNodeType();

        ///////////////////////// SETUP ////////////////////////////////////
        RobotNode(){};

        float jointValueOffset;
        float jointLimitLo, jointLimitHi;
        bool enforceJointLimits = true;
        bool allowJointLimitAvoidance = true;

        // whether this joint has limits or not (ignored if nodeType != Joint).
        bool limitless;
        // When the joint is defined via DH parameters they are stored here
        DHParameter optionalDHParameter;
        float maxVelocity; //! given in [m/s] or [rad/s]
        float maxAcceleration; //! given in [m/s^2] or [rad/s^2]
        float maxDeceleration; //! given in [m/s^2] or [rad/s^2]
        float maxJerk; //! given in [m/s^3] or [rad/s^3]
        float maxTorque; //! given in [Nm]
        Eigen::Matrix4f localTransformation;

        std::map<std::string, float> propagatedJointValues;
        ///////////////////////// SETUP ////////////////////////////////////

        virtual void updateTransformationMatrices(const Eigen::Matrix4f& parentPose);
        RobotWeakPtr robot;

        RobotNodeType nodeType;

        float jointValue; //< The joint value

        /*!
            Derived classes must implement their clone method here.
            Passed models are already scaled. Scaling factor should be only used for kinematic computations.
        */
        virtual RobotNodePtr _clone(const RobotPtr newRobot,
                                    const VisualizationNodePtr visualizationModel,
                                    const CollisionModelPtr collisionModel,
                                    CollisionCheckerPtr colChecker,
                                    float scaling) = 0;

        SceneObject* _clone(const std::string& /*name*/,
                            CollisionCheckerPtr /*colChecker*/ = CollisionCheckerPtr(),
                            float /*scaling*/ = 1.0f) const override;

        /*!
            Derived classes add custom XML tags here
        */
        virtual std::string _toXML(const std::string& modelPath) = 0;

        void setJointValueNotInitialized(float q);
    };

} // namespace VirtualRobot
