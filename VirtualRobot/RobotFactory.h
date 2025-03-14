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

#include <map>
#include <string>
#include <vector>

#include "Nodes/Sensor.h"
#include "VirtualRobot.h"

namespace VirtualRobot
{

    class Robot;
    class RobotNode;

    class VIRTUAL_ROBOT_IMPORT_EXPORT RobotFactory
    {
    public:
        /*!
        Creates an empty robot.
        */
        static RobotPtr createRobot(const std::string& name, const std::string& type = "");

        /*!
            Initializes Robot and all RobotNodes.
            \param robotNodes All nodes of the robot. Must contain rootNode.
            \param childrenMap Parent-child relations are built according to this data.
            \param rootNode The root.
        */
        static bool initializeRobot(RobotPtr robot,
                                    std::vector<RobotNodePtr>& robotNodes,
                                    std::map<RobotNodePtr, std::vector<std::string>> childrenMap,
                                    RobotNodePtr rootNode);

        struct robotNodeDef
        {
            std::string name;
            std::vector<std::string> children;
            // used to mark children whose transformation should be inverted
            std::vector<bool> invertTransformation;
        };

        struct robotStructureDef
        {
            std::string rootName;
            std::vector<robotNodeDef> parentChildMapping;
        };

        /*!
            Clones the robot.
            \param robot The robot to clone
            \param name The new name
            \param collisionChecker Optional: A collision checker to which the robot should be registered. If not set, the collision checker of the input robot is used.
            \param scaling Scale the resulting robot.
        */
        static RobotPtr clone(RobotPtr robot,
                              const std::string& name,
                              CollisionCheckerPtr collisionChecker = CollisionCheckerPtr(),
                              float scaling = 1.0f);

        /*!
         * \brief Clones all nodesets from one robot to another that fullfill the condition that all nodes are contained in the other robot
         * \param from The nodesets copied from
         * \param to The nodesets copied to
         */
        static void cloneRNS(const Robot& from, RobotPtr to);

        /*!
        Clones the robot, but only leave the defined joints active. ALl other joints are accumulated and set to one model which is fixed (may result in faster updates)
        The human mapping is cloned but it is not guranteed that all joints are still contained in the robot.
        \param robot The robot to clone
        \param rns The robot node set of active joints. The joints must be given as an ordered set, i.e. node i must be located before node i+1 in the kinematic structure of the robot.
        \param name The new name
        \param addTCP Wheter the tcp of the robot node set should also be added
        */
        static RobotPtr cloneSubSet(RobotPtr robot,
                                    RobotNodeSetPtr rns,
                                    const std::string& name,
                                    bool addTCP = false);

        /*!
            Creates a robot clone with reduced structure.
            The human mapping is cloned but it is not guranteed that all joints are still contained in the robot.
            \param robot The robot to clone.
            \param uniteWithAllChildren List of RobotNodeNames. Each listed robot ndoe is united with all of its children to one fixed RobotNode.
                                        This means that all related coordinate systems and joints will not be present in the clone. The visualizations are united.
        */
        static RobotPtr cloneUniteSubsets(RobotPtr robot,
                                          const std::string& name,
                                          std::vector<std::string> uniteWithAllChildren);

        /*!
         * Creates a cloned robot model that only include the given nodes by joining nodes together.
         * The human mapping is cloned but it is not guranteed that all joints are still contained in the robot.
         * Physics parameters such as segment inertia or information about whether to ignore collision between nodes are not (yet) updated correctly.
         * \param robot The original robot
         * \param actuatedJointNames The actuated joint nodes
         * \param otherNodeNames Other robot nodes that should be contained with a fixed transformation (given current pose if actuated joint as these are converted to RobotNodeFixed)
         * \return The reduced robot model or nullptr (if actuatedJointNames contains not only joint nodes that are in robot)
         */
        static VirtualRobot::RobotPtr createReducedModel(
            Robot& robot,
            const std::vector<std::string>& actuatedJointNames,
            const std::vector<std::string>& otherNodeNames = std::vector<std::string>());

        /**
         * Creates a cloned robot model that flattens the robot structure.
         * This means that the pattern <joint> -> [<body>, <link>] is transformed to <joint> -> <body> -> <link> which is required for bullet simulation.
         * @param robot The robot to clone.
         * @return The created robot.
         */
        static VirtualRobot::RobotPtr createFlattenedModel(Robot& robot);

        /*!
            Creates a clone with changed structure, so that the given robot node is the new root of the resulting kinematic tree.
        */
        static RobotPtr cloneInversed(RobotPtr robot,
                                      const std::string& newRootName,
                                      bool cloneRNS = true,
                                      bool cloneEEF = true);

        /*!
            Chenge the structure of the clone according to the given defintion.
        */
        static RobotPtr cloneChangeStructure(RobotPtr robot, robotStructureDef& newStructure);

        /*! Clone kinematic chain and reverse direction.
         *
         * \param startNode Name of the start node of the original kinematic chain.
         * \param endNode Name of the end node of the original kinematic chain. Will be the new root.
         */
        static RobotPtr cloneChangeStructure(RobotPtr robot,
                                             const std::string& startNode,
                                             const std::string& endNode);

        /*!
         * \brief attach Attach an object to a robot. The object is cloned.
         * \param robot
         * \param o The object and its visualization model is cloned
         * \param rn The robot node to which the object should be attached
         * \param transformation The RN to object transformation
         * \return true on succes
         */
        static bool attach(RobotPtr robot,
                           SceneObjectPtr o,
                           RobotNodePtr rn,
                           const Eigen::Matrix4f& transformation);

        static bool detach(RobotPtr robot, RobotNodePtr rn);

        static void scaleLinear(RobotNode& node, float sizeScaling, float weightScaling = 1.);

        /**
         * Method to linearly scale the size (in all axes) and the weight of the robot, e.g. for the MMM reference model.
         * This includes scaling all kinematic and dynamic properties as well as the visualization, collision and primitive approximation models.
         * @param robot
         * @param sizeScaling linear scaling factor in x,y,z-direction
         * @param weightScaling linear weight factor
         * @param customSegmentLengths (Optional) RobotNode name -> custom translation length. Takes priority over sizeScaling
         * @param customSizeScaling (Optional) RobotNode name -> custom size scaling. Takes priority over customSegmentLengths.
         */
        static void scaleLinear(Robot& robot,
                                float sizeScaling,
                                float weightScaling = 1.,
                                const std::map<std::string, float>& customSegmentLengths =
                                    std::map<std::string, float>(),
                                const std::map<std::string, float>& customSegmentSizeScaling =
                                    std::map<std::string, float>());

    protected:
        // some internal stuff
        static RobotNodePtr createUnitedRobotNode(RobotPtr robot,
                                                  const std::vector<RobotNodePtr>& nodes,
                                                  RobotNodePtr parent,
                                                  RobotNodePtr parentClone,
                                                  const Eigen::Matrix4f& trafo,
                                                  const std::vector<SensorPtr>& sensors);
        static RobotNodePtr accumulateTransformations(RobotPtr robot,
                                                      RobotNodePtr nodeA,
                                                      RobotNodePtr nodeAClone,
                                                      RobotNodePtr nodeB,
                                                      Eigen::Matrix4f& storeTrafo);
        static void getChildNodes(RobotNodePtr nodeA,
                                  RobotNodePtr nodeExclude,
                                  std::vector<RobotNodePtr>& appendNodes);
        static void getChildSensorNodes(RobotNodePtr nodeA,
                                        RobotNodePtr nodeExclude,
                                        std::vector<SensorPtr>& appendNodes);

        static void cloneRecursiveUnite(RobotPtr robot,
                                        RobotNodePtr currentNode,
                                        RobotNodePtr currentNodeClone,
                                        std::vector<std::string> uniteWithAllChildren);

        // instantiation not allowed
        RobotFactory();
        virtual ~RobotFactory();
    };
} // namespace VirtualRobot
