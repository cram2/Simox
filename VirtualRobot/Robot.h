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

#include "VirtualRobot.h"
#include "SceneObject.h"
#include "Nodes/RobotNode.h"
#include "Nodes/ConditionedLock.h"
#include "BoundingBox.h"

#include <type_traits>
#include <string>
#include <unordered_map>
#include <vector>
#include <map>

#include <Eigen/Core>

namespace VirtualRobot
{
    class Visualization;
    class RobotNode;


    struct NodeMappingElement
    {
        std::string node;

        float sign;
    };

    using NodeMapping = std::unordered_map<std::string, NodeMappingElement>;

    struct HumanMapping
    {
        struct ArmDescription
        {
            struct Segment
            {
                std::string nodeName;
                Eigen::Matrix4f offset;
            };

            struct Segments
            {
                Segment shoulder;
                Segment elbow;
                Segment wrist;
                Segment palm;
                Segment tcp;
            } segments;

            struct HumanJointDescription
            {
                std::string type;
                std::string motion;

                /// an offset that needs to be applied to the robot's node to match the human
                float offset;

                /// indicates whether the joint rotates into the opposite direction
                bool inverted; 
            };

            // key: "NodeName" 
            using JointMapping = std::unordered_map<std::string, HumanJointDescription>;

            JointMapping jointMapping;

            std::string nodeSet;
        };

        ArmDescription leftArm;
        ArmDescription rightArm;
    };

    /*!
        This is the main object defining the kinematic structure of a robot.

        \see RobotIO, RobotNode, RobotNodeSet, EndEffector
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT Robot : public SceneObject
    {
        friend class RobotIO;
    public:
        static const RobotPtr NullPtr;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
        Constructor
        \param name Specifies the name of this instance.
        \param type Specifies the type of the robot (e.g. multiple robots of the same type could exists with different names)
        */
        Robot(const std::string& name, const std::string& type = "");

        /*!
        */
        ~Robot() override;

        /*!
            The root node is the first RobotNode of this robot.
        */
        virtual void setRootNode(RobotNodePtr node) = 0;
        virtual RobotNodePtr getRootNode() const = 0;

        virtual void applyJointValues();

        /** Configures the robot to threadsafe or not.
         * Per default the robot is threadsafe, i.e., updating the
         * robot state and reading the Poses from the nodes is mutual
         * exclusive. This feature can be turned off, however, in
         * order to be make data access faster in single threaded
         * applications.
         */
        virtual void setThreadsafe(bool);


        /*!
            Retrieve a visualization in the given format.
            \param visuType The visualization type (Full or Collision model)
            \param sensors Add sensors models to the visualization.
            Example usage:
             std::shared_ptr<VirtualRobot::CoinVisualization> visualization = robot->getVisualization<CoinVisualization>();
             SoNode* visualisationNode = NULL;
             if (visualization)
                 visualisationNode = visualization->getCoinVisualization();
        */
        template <typename T> std::shared_ptr<T> getVisualization(SceneObject::VisualizationType visuType = SceneObject::Full, bool sensors = true);
        /*!
            Shows the structure of the robot
        */
        void showStructure(bool enable, const std::string& type = "");
        /*!
            Shows the coordinate systems of the robot nodes
        */
        void showCoordinateSystems(bool enable, const std::string& type = "");

        /*!
            Display some physics debugging information.
            \p enableCoM If true, the center of mass is shown (if given). If a comModel is given it is used for visualization, otherwise a standrad marker is shown.
            \p enableInertial If true, a visualization of the inertial matrix is shown (if given).
            \p comModel If set, this visualization is used to display the CoM location. If not set, a standard marker is used.
        */
        void showPhysicsInformation(bool enableCoM, bool enableInertial, VisualizationNodePtr comModel = VisualizationNodePtr()) override;

        /*!
            Setup the full model visualization.
            \param showVisualization If false, the visualization is disabled.
            \param showAttachedVisualizations If false, the visualization of any attached optional visualizations is disabled.
        */
        void setupVisualization(bool showVisualization, bool showAttachedVisualizations) override;


        virtual std::string getName() const;
        virtual std::string getType() const;

        void setType(const std::string& type);
        void setName(const std::string& name);

        /*!
        Print status information.
        */
        void print(bool printChildren = false, bool printDecoration = true) const override;

        /*!
            Enables/Disables the visualization updates of collision model and visualization model.
        */
        void setUpdateVisualization(bool enable) override;
        void setUpdateCollisionModel(bool enable) override;

        std::shared_ptr<Robot> shared_from_this() const;

        /*!
            get the complete setup of all robot nodes
        */
        virtual RobotConfigPtr getConfig() const;
        /*!
            Sets the configuration according to the RobotNodes, defined in c. All other nodes are not affected.
        */
        virtual bool setConfig(RobotConfigPtr c);

        /*!
            This method is automatically called in RobotNode's initialization routine.
        */
        virtual void registerRobotNode(RobotNodePtr node) = 0;
        virtual void deregisterRobotNode(RobotNodePtr node) = 0;
        virtual bool hasRobotNode(RobotNodePtr node) const = 0;
        virtual bool hasRobotNode(const std::string& robotNodeName) const = 0;
        virtual RobotNodePtr getRobotNode(const std::string& robotNodeName) const = 0;
        virtual std::vector< RobotNodePtr > getRobotNodes() const;
        virtual std::vector<std::string> getRobotNodeNames() const;
        virtual void getRobotNodes(std::vector< RobotNodePtr >& storeNodes, bool clearVector = true) const  = 0;

        /*!
            This method is automatically called in RobotNodeSet's initialization routine.
        */
        virtual void registerRobotNodeSet(RobotNodeSetPtr nodeSet) = 0;
        virtual void deregisterRobotNodeSet(RobotNodeSetPtr nodeSet) = 0;
        virtual bool hasRobotNodeSet(RobotNodeSetPtr nodeSet) const;
        virtual bool hasRobotNodeSet(const std::string& name) const = 0;
        virtual RobotNodeSetPtr getRobotNodeSet(const std::string& nodeSetName) const = 0;
        virtual std::vector<RobotNodeSetPtr> getRobotNodeSets() const;
        virtual std::vector<std::string> getRobotNodeSetNames() const;
        virtual void getRobotNodeSets(std::vector<RobotNodeSetPtr>& storeNodeSet) const = 0;

        /*!
        * Node mapping
        */
        const NodeMapping& getNodeMapping() const;
        const std::optional<HumanMapping>& getHumanMapping() const;

        /**
         *
         */
        virtual void registerEndEffector(EndEffectorPtr endEffector) = 0;
        virtual bool hasEndEffector(EndEffectorPtr endEffector) const;
        virtual bool hasEndEffector(const std::string& endEffectorName) const = 0;
        virtual EndEffectorPtr getEndEffector(const std::string& endEffectorName) const = 0;
        virtual std::vector<EndEffectorPtr> getEndEffectors() const;
        virtual void getEndEffectors(std::vector<EndEffectorPtr>& storeEEF) const = 0;

        virtual std::vector< CollisionModelPtr > getCollisionModels() const;

        CollisionCheckerPtr getCollisionChecker() override;

        /*!
          Convenient method for highlighting the visualization of this robot.
          It is automatically checked whether the collision model or the full model is part of the visualization.
          \param visualization The visualization for which the highlighting should be performed.
          \param enable On or off
        */
        virtual void highlight(VisualizationPtr visualization, bool enable);


        /*!
            get number of faces (i.e. triangles) of this object
            \p collisionModel Indicates weather the faces of the collision model or the full model should be returned.
        */
        int getNumFaces(bool collisionModel = false) override;

        //! Set the global pose of this robot
        virtual void setGlobalPose(const Eigen::Matrix4f& globalPose, bool applyValues) = 0;
        void setGlobalPose(const Eigen::Matrix4f& globalPose) override;

        //! Get the global pose of this robot so that the RobotNode node is at pose globalPoseNode.
        virtual Eigen::Matrix4f getGlobalPoseForRobotNode(const RobotNodePtr& node, const Eigen::Matrix4f& globalPoseNode) const;
        //! Set the global pose of this robot so that the RobotNode node is at pose globalPoseNode.
        virtual void setGlobalPoseForRobotNode(const RobotNodePtr& node, const Eigen::Matrix4f& globalPoseNode);

        //! Get the global position of this robot so that the RobotNode node is at position globalPoseNode
        virtual Eigen::Matrix4f getGlobalPositionForRobotNode(const RobotNodePtr& node, const Eigen::Vector3f& globalPositionNode) const;
        //! Set the global position of this robot so that the RobotNode node is at position globalPoseNode
        virtual void setGlobalPositionForRobotNode(const RobotNodePtr& node, const Eigen::Vector3f& globalPositionNode);

        //virtual Eigen::Matrix4f getGlobalPose() = 0;


        //! Return center of mass of this robot in local coordinate frame. All RobotNodes of this robot are considered according to their mass.
        Eigen::Vector3f getCoMLocal() override;

        //! Return Center of Mass of this robot in global coordinates. All RobotNodes of this robot are considered according to their mass.
        Eigen::Vector3f getCoMGlobal() override;

        //! Return accumulated mass of this robot.
        virtual float getMass() const;


        /*!
            Extract a sub kinematic from this robot and create a new robot instance.
            \param startJoint The kinematic starts with this RobotNode
            \param newRobotType The name of the newly created robot type
            \param newRobotName The name of the newly created robot
            \param cloneRNS Clone all robot node sets that belong to the original robot and for which the remaining robot nodes of the subPart are sufficient.
            \param cloneEEFs Clone all end effectors that belong to the original robot and for which the remaining robot nodes of the subPart are sufficient.
            \param collisionChecker The new robot can be registered to a different collision checker. If not set, the collision checker of the original robot is used.
            \param scaling Can be set to create a scaled version of this robot. Scaling is applied on kinematic, visual, and collision data.
        */
        virtual RobotPtr extractSubPart(RobotNodePtr startJoint,
                                        const std::string& newRobotType,
                                        const std::string& newRobotName,
                                        bool cloneRNS = true,
                                        bool cloneEEFs = true,
                                        CollisionCheckerPtr collisionChecker = CollisionCheckerPtr(),
                                        float scaling = 1.0f,
                                        bool preventCloningMeshesIfScalingIs1 = false);

        /*!
            Clones this robot.
            \param name The new name.
            \param collisionChecker If set, the returned robot is registered with this col checker, otherwise the CollisionChecker of the original robot is used.
            \param scaling Scale Can be set to create a scaled version of this robot. Scaling is applied on kinematic, visual, and collision data.

        */
        virtual RobotPtr clone(const std::string& name,
                               CollisionCheckerPtr collisionChecker = CollisionCheckerPtr(),
                               float scaling = 1.0f,
                               bool preventCloningMeshesIfScalingIs1 = false);
        virtual RobotPtr clone(CollisionCheckerPtr collisionChecker = CollisionCheckerPtr(),
                               float scaling = 1.0f,
                               bool preventCloningMeshesIfScalingIs1 = false);

        /*! @brief Clones this robot and keeps the current scaling for the robot and each robot node */
        virtual RobotPtr cloneScaling();

        //! Just storing the filename.
        virtual void setFilename(const std::string& filename);

        //! Retrieve the stored filename.
        virtual std::string getFilename() const;

        /*!
            This readlock can be used to protect data access. It locks the mutex until deletion.
            For internal use. API users will usually not need this functionality since all data access is protected automatically.

            Exemplary usage:
            {
                ReadLockPtr lock = robot->getReadLock();
                // now the mutex is locked

                // access data
                // ...

            } // end of scope -> lock gets deleted and mutex is released automatically
        */
        virtual ReadLockPtr getReadLock();
        /*!
            This writelock can be used to protect data access. It locks the mutex until deletion.
            For internal use. API users will usually not need this functionality since all data access is protected automatically.

            Exemplary usage:
            {
                WriteLockPtr lock = robot->getWriteLock();
                // now the mutex is locked

                // access data
                // ...

            } // end of scope -> lock gets deleted and mutex is released automatically
        */
        virtual WriteLockPtr getWriteLock();

        /*!
            Set a joint value [rad].
            The internal matrices and visualizations are updated accordingly.
            If you intend to update multiple joints, use \ref setJointValues for faster access.
        */
        virtual void setJointValue(RobotNodePtr rn, float jointValue);
        /*!
            Set a joint value [rad].
            The internal matrices and visualizations are updated accordingly.
            If you intend to update multiple joints, use \ref setJointValues for faster access.
        */
        virtual void setJointValue(const std::string& nodeName, float jointValue);

        /*!
            Set joint values [rad].
            The complete robot is updated to apply the new joint values.
            \param jointValues A map containing RobotNode names with according values.
        */
        virtual void setJointValues(const std::map< std::string, float >& jointValues);
        virtual std::map<std::string, float> getJointValues() const;
        /*!
            Set joint values [rad].
            The complete robot is updated to apply the new joint values.
            \param jointValues A map containing RobotNodes with according values.
        */
        virtual void setJointValues(const std::map< RobotNodePtr, float >& jointValues);
        /*!
            Set joint values [rad].
            The subpart of the robot, defined by the start joint (kinematicRoot) of rns, is updated to apply the new joint values.
            \param rns The RobotNodeSet defines the joints
            \param jointValues A vector with joint values, size must be equal to rns.
        */
        virtual void setJointValues(RobotNodeSetPtr rns, const std::vector<float>& jointValues);
        /*!
            Set joint values [rad].
            The complete robot is updated to apply the new joint values.
            \param rn The RobotNodes
            \param jointValues A vector with joint values, size must be equal to rn.
        */
        virtual void setJointValues(const std::vector<RobotNodePtr> rn, const std::vector<float>& jointValues);
        /*!
            Set joint values [rad].
            The subpart of the robot, defined by the start joint (kinematicRoot) of rns, is updated to apply the new joint values.
            \param rns The RobotNodeSet defines the joints
            \param jointValues A vector with joint values, size must be equal to rns.
        */
        virtual void setJointValues(RobotNodeSetPtr rns, const Eigen::VectorXf& jointValues);
        /*!
            Set joint values [rad].
            The complete robot is updated to apply the new joint values.
            \param config The RobotConfig defines the RobotNodes and joint values.
        */
        virtual void setJointValues(RobotConfigPtr config);
        /*!
            Set joint values [rad].
            Only those joints in config are affected which are present in rns.
            The subpart of the robot, defined by the start joint (kinematicRoot) of rns, is updated to apply the new joint values.
            \param rns Only joints in this rns are updated.
            \param config The RobotConfig defines the RobotNodes and joint values.
        */
        virtual void setJointValues(RobotNodeSetPtr rns, RobotConfigPtr config);
        /*!
            Apply configuration of trajectory at time t
            \param trajectory The trajectory
            \param t The time (0<=t<=1)
        */
        virtual void setJointValues(TrajectoryPtr trajectory, float t);

        /*!
            The (current) bounding box in global coordinate system.
            \param collisionModel Either the collision or the visualization model is considered.
            \return The bounding box.
        */
        virtual BoundingBox getBoundingBox(bool collisionModel = true) const;

        /*!
         * Returns the sensor with the given name.
         */
        virtual SensorPtr getSensor(const std::string& name) const;

        template<class SensorType>
        std::shared_ptr<SensorType> getSensor(const std::string& name) const
        {
            return std::dynamic_pointer_cast<SensorType>(getSensor(name));
        }

        /*!
            Returns all sensors that are defined within this robot.
        */
        virtual std::vector<SensorPtr> getSensors() const;

        template<class SensorType>
        std::vector<std::shared_ptr<SensorType> > getSensors() const
        {
            std::vector<std::shared_ptr<SensorType> > result;
            std::vector<SensorPtr> sensors = getSensors();
            result.reserve(sensors.size());
            for (std::size_t i = 0; i < sensors.size(); ++i)
            {
                if (dynamic_cast<SensorType*>(sensors.at(i).get()))
                {
                    result.push_back(std::static_pointer_cast<SensorType>(sensors.at(i)));
                }
            }
            return result;
        }
        /*!
            Creates an XML string that defines the complete robot. Filenames of all visualization models are set to modelPath/RobotNodeName_visu and/or modelPath/RobotNodeName_colmodel.
            @see RobotIO::saveXML.
        */

        virtual std::string toXML(const std::string& basePath = ".", const std::string& modelPath = "models", bool storeEEF = true, bool storeRNS = true, bool storeSensors = true, bool storeModelFiles = true) const;

        float getScaling() const;
        void setScaling(float scaling);

        /**
         * @brief Inflates the collision models of all robot nodes of this robot. Useful for motion planning with a safety margin.
         * @param inflationInMM The collision model is inflated by this value. This value is absolute to the original collision model.
         * Repetitive inflation with the same value has no effect.
         */
        void inflateCollisionModel(float inflationInMM);


        bool getPropagatingJointValuesEnabled() const;
        void setPropagatingJointValuesEnabled(bool enabled);


        void setPassive(){
            passive = true;
        }

        bool isPassive() const
        {
            return passive;
        }

        void registerNodeMapping(const NodeMapping& nodeMapping);

        void registerHumanMapping(const HumanMapping& humanMapping);


    protected:
        Robot();

        void validateNodeMapping(const NodeMapping& nodeMapping) const;
        void validateHumanMapping(const HumanMapping& humanMapping) const;


        /*!
            Goes through all RobotNodes and if no visualization is present:
            * the collision model is checked and in case it owns a visualization
            * it is cloned and used as visualization.
        */
        void createVisualizationFromCollisionModels();

        //! It is assumed that the mutex is already set
        virtual void applyJointValuesNoLock();

        std::string filename; // RobotIO stores the filename here
        std::string type;

        bool updateVisualization;

        mutable std::recursive_mutex mutex;
        bool use_mutex;
        bool propagatingJointValuesEnabled = true;

        bool passive{false};

        //float radianToMMfactor = 10;

    private:
        NodeMapping nodeMapping;
        std::optional<HumanMapping> humanMapping;

    };

    /**
     * This method collects all visualization nodes and creates a new Visualization
     * subclass which is given by the template parameter T.
     * T must be a subclass of VirtualRobot::Visualization.
     * A compile time error is thrown if a different class type is used as template argument.
     */
    template <typename T>
    std::shared_ptr<T> Robot::getVisualization(SceneObject::VisualizationType visuType, bool sensors)
    {
        static_assert(::std::is_base_of_v<Visualization, T>,
                      "TEMPLATE_PARAMETER_FOR_VirtualRobot_getVisualization_MUST_BT_A_SUBCLASS_OF_VirtualRobot__Visualization");
        std::vector<RobotNodePtr> collectedRobotNodes;
        getRobotNodes(collectedRobotNodes);
        std::vector<VisualizationNodePtr> collectedVisualizationNodes;

        for (size_t i = 0; i < collectedRobotNodes.size(); i++)
        {
            collectedVisualizationNodes.push_back(collectedRobotNodes[i]->getVisualization(visuType));
        }

        if (sensors)
        {
            std::vector<SensorPtr> sn = getSensors();

            for (size_t i = 0; i < sn.size(); i++)
            {
                collectedVisualizationNodes.push_back(sn[i]->getVisualization(visuType));
            }
        }

        std::shared_ptr<T> visualization(new T(collectedVisualizationNodes));
        return visualization;
    }

    class VIRTUAL_ROBOT_IMPORT_EXPORT LocalRobot : public Robot
    {
    public:
        LocalRobot(const std::string& name, const std::string& type = "");
        ~LocalRobot() override;

        void setRootNode(RobotNodePtr node) override;
        RobotNodePtr getRootNode() const override;

        void registerRobotNode(RobotNodePtr node) override;
        void deregisterRobotNode(RobotNodePtr node) override;
        bool hasRobotNode(const std::string& robotNodeName) const override;
        bool hasRobotNode(RobotNodePtr node) const override;
        RobotNodePtr getRobotNode(const std::string& robotNodeName) const override;
        void getRobotNodes(std::vector< RobotNodePtr >& storeNodes, bool clearVector = true) const override;

        void registerRobotNodeSet(RobotNodeSetPtr nodeSet) override;
        void deregisterRobotNodeSet(RobotNodeSetPtr nodeSet) override;
        bool hasRobotNodeSet(const std::string& name) const override;
        RobotNodeSetPtr getRobotNodeSet(const std::string& nodeSetName) const override;
        void getRobotNodeSets(std::vector<RobotNodeSetPtr>& storeNodeSet) const override;

        void registerEndEffector(EndEffectorPtr endEffector) override;
        bool hasEndEffector(const std::string& endEffectorName) const override;
        EndEffectorPtr getEndEffector(const std::string& endEffectorName) const override;
        void getEndEffectors(std::vector<EndEffectorPtr>& storeEEF) const override;

        void setGlobalPose(const Eigen::Matrix4f& globalPose, bool applyJointValues = true) override;
        void setGlobalPose(const Eigen::Matrix4f& globalPose) override;
        Eigen::Matrix4f getGlobalPose() const override;

    protected:
        //Eigen::Matrix4f globalPose; //!< The pose of this robot in the world
        RobotNodePtr rootNode;
        void applyJointValuesNoLock() override;

        std::map< std::string, RobotNodePtr > robotNodeMap;
        std::map< std::string, RobotNodeSetPtr > robotNodeSetMap;
        std::map< std::string, EndEffectorPtr > endEffectorMap;

    };



} // namespace VirtualRobot
