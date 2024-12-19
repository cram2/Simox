
#include "RobotFactory.h"

#include <algorithm>
#include <cassert>
#include <deque>
#include <iostream>
#include <queue>

#include <SimoxUtility/math/pose/invert.h>

#include "Assert.h"
#include "CollisionDetection/CollisionModel.h"
#include "EndEffector/EndEffector.h"
#include "Logging.h"
#include "Nodes/RobotNode.h"
#include "Nodes/RobotNodeFixed.h"
#include "Nodes/RobotNodeFixedFactory.h"
#include "Nodes/RobotNodePrismatic.h"
#include "Nodes/RobotNodeRevolute.h"
#include "Robot.h"
#include "RobotNodeSet.h"
#include "VirtualRobotException.h"
#include "Visualization//VisualizationFactory.h"
#include "Visualization/VisualizationNode.h"

namespace VirtualRobot
{
    using std::cout;
    using std::endl;

    RobotFactory::RobotFactory() = default;

    RobotFactory::~RobotFactory() = default;

    RobotPtr
    RobotFactory::createRobot(const std::string& name, const std::string& type)
    {
        RobotPtr result(new LocalRobot(name, type));
        return result;
    }

    bool
    RobotFactory::initializeRobot(RobotPtr robot,
                                  std::vector<RobotNodePtr>& robotNodes,
                                  std::map<RobotNodePtr, std::vector<std::string>> childrenMap,
                                  RobotNodePtr rootNode)
    {
        THROW_VR_EXCEPTION_IF(!robot, "Robot is null");
        bool result = true;

        // check for root
        bool foundRoot = false;

        for (const auto& node : robotNodes)
        {
            THROW_VR_EXCEPTION_IF(!node, "Robot node is null! check robotNodes");
            if (node == rootNode)
            {
                foundRoot = true;
            }

            if (node->getRobot() != robot)
            {
                THROW_VR_EXCEPTION("Invalid robot node (robot is not set correctly)");
            }
        }

        THROW_VR_EXCEPTION_IF(!foundRoot, "Invalid robot node (root is not available)");

        // process children
        for (const auto& [node, childNames] : childrenMap)
        {
            THROW_VR_EXCEPTION_IF(!node, "Key in childrenMap is null! check childrenMap");
            for (const auto& childName : childNames)
            {
                if (!robot->hasRobotNode(childName))
                {
                    std::stringstream str;
                    str << "Robot " << robot->getName() << ": corrupted RobotNode <"
                        << node->getName() << " child :" << childName << " does not exist...\n"
                        << "These nodes were given:\n";
                    for (const auto& n : robotNodes)
                    {
                        str << "--- " << n->getName() << "\n";
                    }
                    str << "This child map was given\n";
                    for (const auto& [node, childNames] : childrenMap)
                    {
                        str << "--- " << node->getName() << "\n";
                        for (const auto& child : childNames)
                        {
                            str << "------- " << child << "\n";
                        }
                    }
                    THROW_VR_EXCEPTION(str.str());
                }

                RobotNodePtr c = robot->getRobotNode(childName);
                node->attachChild(c);
            }
        }

        // register root (performs an initialization of all robot nodes)
        robot->setRootNode(rootNode);

        return result;
    }

    struct robotNodeDef
    {
        std::string name;
        std::vector<std::string> children;
    };

    struct robotStructureDef
    {
        std::string rootName;
        std::vector<robotNodeDef> parentChildMapping;
    };

    RobotPtr
    RobotFactory::cloneInversed(RobotPtr robot,
                                const std::string& newRootName,
                                bool cloneRNS,
                                bool cloneEEF)
    {
        VR_ASSERT(robot);

        RobotNodePtr newRoot = robot->getRobotNode(newRootName);

        if (!newRoot)
        {
            VR_ERROR << "No node " << newRootName << std::endl;
        }

        RobotFactory::robotStructureDef newStructure;
        newStructure.rootName = newRootName;

        typedef std::pair<RobotNodePtr, RobotNodePtr> RobotTreeEdge;

        std::deque<RobotTreeEdge> edges;
        RobotTreeEdge rootEdge;
        rootEdge.second = newRoot;
        edges.push_back(rootEdge);

        while (!edges.empty())
        {
            RobotTreeEdge currentEdge = edges.front();
            edges.pop_front();

            RobotNodePtr parent =
                std::dynamic_pointer_cast<RobotNode>(currentEdge.second->getParent());

            std::vector<SceneObjectPtr> children = currentEdge.second->getChildren();
            RobotFactory::robotNodeDef rnDef;
            rnDef.name = currentEdge.second->getName();

            // invert transformation of old parent
            if (parent && parent != currentEdge.first)
            {
                rnDef.invertTransformation.push_back(true);
                rnDef.children.push_back(parent->getName());

                RobotTreeEdge edge;
                edge.first = currentEdge.second;
                edge.second = parent;

                assert(edge.second);
                edges.push_back(edge);
            }

            for (auto& i : children)
            {
                if (i != currentEdge.first)
                {
                    RobotNodePtr childNode = std::dynamic_pointer_cast<RobotNode>(i);

                    // not a robot node
                    if (!childNode)
                    {
                        continue;
                    }

                    rnDef.children.push_back(i->getName());
                    rnDef.invertTransformation.push_back(false);
                    RobotTreeEdge edge;
                    edge.second = childNode;
                    edge.first = currentEdge.second;

                    assert(edge.second);
                    edges.push_back(edge);
                }
            }

            newStructure.parentChildMapping.push_back(rnDef);
        }


        RobotPtr r = RobotFactory::cloneChangeStructure(robot, newStructure);

        if (cloneRNS)
        {
            std::vector<VirtualRobot::RobotNodeSetPtr> robotNodeSets;
            for (RobotNodeSetPtr rns : robot->getRobotNodeSets())
            {
                robotNodeSets.push_back(robot->getRobotNodeSet(rns->getName())->clone(r));
            }
        }

        if (cloneEEF)
        {
            // Copy end effectors
            for (auto& eef : robot->getEndEffectors())
            {
                eef->clone(r);
            }
        }

        return r;
    }

    RobotPtr
    RobotFactory::cloneChangeStructure(RobotPtr robot,
                                       const std::string& startNode,
                                       const std::string& endNode)
    {
        VR_ASSERT(robot);

        if (!robot->hasRobotNode(startNode))
        {
            VR_ERROR << "No node with name " << startNode << std::endl;
            return RobotPtr();
        }

        if (!robot->hasRobotNode(endNode))
        {
            VR_ERROR << "No node with name " << endNode << std::endl;
            return RobotPtr();
        }

        if (!robot->getRobotNode(startNode)->hasChild(endNode, true))
        {
            VR_ERROR << "No node " << endNode << " is not a child of " << startNode << std::endl;
            return RobotPtr();
        }

        std::vector<std::string> nodes;
        std::string currentNodeName = endNode;
        RobotNodePtr rn = robot->getRobotNode(currentNodeName);

        while (rn && !(rn->getName() == startNode))
        {
            currentNodeName = rn->getName();
            nodes.push_back(currentNodeName);
            rn = std::dynamic_pointer_cast<RobotNode>(rn->getParent());
        }

        if (!rn)
        {
            VR_ERROR << "No node " << endNode << " is not a child of " << startNode << std::endl;
            return RobotPtr();
        }

        nodes.push_back(startNode);
        //std::reverse(nodes.begin(), nodes.end());

        RobotFactory::robotStructureDef newStructure;
        newStructure.rootName = endNode;

        for (size_t i = 0; i < nodes.size() - 1; i++)
        {
            RobotFactory::robotNodeDef rnDef;
            rnDef.name = nodes[i];
            rnDef.children.push_back(nodes[i + 1]);
            rnDef.invertTransformation.push_back(true);
            newStructure.parentChildMapping.push_back(rnDef);
        }

        RobotFactory::robotNodeDef rnDef;
        rnDef.name = nodes[nodes.size() - 1];
        rnDef.invertTransformation.push_back(true);
        newStructure.parentChildMapping.push_back(rnDef);


        return RobotFactory::cloneChangeStructure(robot, newStructure);
    }

    bool
    RobotFactory::attach(RobotPtr robot,
                         SceneObjectPtr o,
                         RobotNodePtr rn,
                         const Eigen::Matrix4f& transformation)
    {
        if (!robot || !o || !rn)
        {
            return false;
        }

        std::string name = o->getName();

        if (robot->hasRobotNode(name))
        {
            VR_WARNING << "RN with name " << name << " already present" << std::endl;
            return false;
        }

        SceneObject::Physics p = o->physics;
        VisualizationNodePtr v;

        if (o->getVisualization())
        {
            v = o->getVisualization()->clone();
        }

        CollisionModelPtr c;

        if (o->getCollisionModel())
        {
            c = o->getCollisionModel();
        }

        auto rnf = RobotNodeFixedFactory::createInstance(nullptr);
        RobotNodePtr newRN = rnf->createRobotNode(robot,
                                                  name,
                                                  v,
                                                  c,
                                                  0,
                                                  0,
                                                  0,
                                                  transformation,
                                                  Eigen::Vector3f::Zero(),
                                                  Eigen::Vector3f::Zero(),
                                                  p);
        rn->attachChild(newRN);
        newRN->initialize(rn);
        newRN->primitiveApproximation = o->getPrimitiveApproximation().clone();
        return true;
    }

    bool
    RobotFactory::detach(RobotPtr robot, RobotNodePtr rn)
    {
        if (!robot || !rn || !rn->getParent())
        {
            return false;
        }

        if (!robot->hasRobotNode(rn))
        {
            return false;
        }

        rn->getParent()->detachChild(rn);
        robot->deregisterRobotNode(rn);
        return true;
    }

    void
    RobotFactory::scaleLinear(RobotNode& node, float sizeScaling, float weightScaling)
    {
        Eigen::Matrix4f localTransformation = node.getLocalTransformation();
        localTransformation.block(0, 3, 3, 1) *= sizeScaling;
        node.setLocalTransformation(localTransformation);

        if (node.optionalDHParameter.isSet)
        {
            node.optionalDHParameter.setAInMM(node.optionalDHParameter.aMM() * sizeScaling);
            node.optionalDHParameter.setDInMM(node.optionalDHParameter.dMM() * sizeScaling);
        }

        node.setMass(node.getMass() * weightScaling);
        node.setCoMLocal(node.getCoMLocal() * sizeScaling);
        node.setInertiaMatrix(node.getInertiaMatrix() * pow(sizeScaling, 2) * weightScaling);
        node.setScaling(node.getScaling() * sizeScaling);
    }

    void
    RobotFactory::scaleLinear(Robot& robot,
                              float sizeScaling,
                              float weightScaling,
                              const std::map<std::string, float>& customSegmentLengths,
                              const std::map<std::string, float>& customSizeScaling)
    {
        for (const auto& node : robot.getRobotNodes())
        {
            float model_height_scaling = sizeScaling;

            const auto segLengthIt = customSegmentLengths.find(node->getName());
            const auto sizeScaleIt = customSizeScaling.find(node->getName());

            if (sizeScaleIt != customSizeScaling.end())
            {
                model_height_scaling = sizeScaleIt->second;
            }

            if (segLengthIt != customSegmentLengths.end())
            {
                if (sizeScaleIt != customSizeScaling.end())
                {
                    VR_WARNING << "Custom segment length ignored for node " << node->getName()
                               << " because custom height scaling was already specified ";
                }
                else
                {
                    const float l =
                        node->getLocalTransformation().block(0, 3, 3, 1).cwiseAbs().maxCoeff();
                    if (l > 0.0f)
                    {
                        model_height_scaling = 1.0f / l * segLengthIt->second;
                    }
                }
            }

            for (const auto& sensor : node->getSensors())
            {
                Eigen::Matrix4f lt = sensor->getParentNodeToSensorTransformation();
                lt.block(0, 3, 3, 1) *= model_height_scaling;
                sensor->setRobotNodeToSensorTransformation(lt);
            }

            scaleLinear(*node.get(), model_height_scaling, weightScaling);

            if (auto vis = node->visualizationModel)
            {
                node->setVisualization(node->visualizationModel->clone(true, model_height_scaling));
            }

            if (auto col = node->getCollisionModel())
            {
                node->setCollisionModel(
                    col->clone(node->getCollisionChecker(), model_height_scaling, true));
            }

            node->getPrimitiveApproximation().scaleLinear(model_height_scaling);
        }
        robot.setScaling(robot.getScaling() * sizeScaling);
        robot.setMass(robot.getMass() * weightScaling);
    }

    RobotPtr
    RobotFactory::cloneChangeStructure(RobotPtr robot, robotStructureDef& newStructure)
    {
        VR_ASSERT(robot);

        if (!robot->hasRobotNode(newStructure.rootName))
        {
            VR_ERROR << "No root with name " << newStructure.rootName << std::endl;
            return RobotPtr();
        }

        std::map<std::string, RobotNodePtr> newNodes;
        RobotPtr newRobot = createRobot(
            robot->getName(), robot->getType() + "_restructured_" + newStructure.rootName);
        RobotNodePtr rn = robot->getRobotNode(newStructure.rootName);
        rn = rn->clone(newRobot, false);
        newNodes[newStructure.rootName] = rn;
        newRobot->setRootNode(newNodes[newStructure.rootName]);

        std::string nodeName;
        typedef std::map<RobotNodePtr,
                         Eigen::Matrix4f,
                         std::less<RobotNodePtr>,
                         Eigen::aligned_allocator<std::pair<const RobotNodePtr, Eigen::Matrix4f>>>
            NodeTransformationMapT;

        NodeTransformationMapT localTransformations;
        std::map<RobotNodePtr, VisualizationNodePtr> visuMap;
        std::map<RobotNodePtr, CollisionModelPtr> colMap;
        std::map<RobotNodePtr, SceneObject::Physics> physicsMap;
        std::map<RobotNodePtr, std::vector<SensorPtr>> sensorMap;
        std::map<RobotNodePtr, bool> directionInversion;

        for (auto& i : newStructure.parentChildMapping)
        {
            if (!robot->hasRobotNode(i.name))
            {
                VR_ERROR << "Error in parentChildMapping, no node with name " << i.name
                         << std::endl;
                return RobotPtr();
            }

            nodeName = i.name;

            if (newNodes.find(nodeName) == newNodes.end())
            {
                rn = robot->getRobotNode(nodeName);
                rn = rn->clone(newRobot, false);
                newNodes[nodeName] = rn;
            }

            RobotNodePtr parent = newNodes[i.name];

            for (size_t j = 0; j < i.children.size(); j++)
            {
                nodeName = i.children[j];

                if (!robot->hasRobotNode(nodeName))
                {
                    VR_ERROR << "Error in parentChildMapping, no child node with name " << nodeName
                             << std::endl;
                    return RobotPtr();
                }

                if (newNodes.find(nodeName) == newNodes.end())
                {
                    rn = robot->getRobotNode(nodeName);
                    rn = rn->clone(newRobot, false);
                    newNodes[nodeName] = rn;
                }

                //children.push_back(newNodes[nodeName]);
                RobotNodePtr child = newNodes[nodeName];
                parent->attachChild(child);

                if (i.invertTransformation[j])
                {
                    Eigen::Matrix4f tr = parent->getLocalTransformation().inverse();
                    localTransformations[child] = tr;
                    // we also need to invert the direction
                    directionInversion[child] = true;

                    // check for models
                    if (child->getVisualization())
                    {
                        VisualizationNodePtr v = child->getVisualization();
                        VisualizationFactoryPtr vf = VisualizationFactory::first(NULL);
                        Eigen::Matrix4f tr2 = tr;
                        //tr2.block(0, 3, 3, 1) *= 0.001f; // m is needed here?
                        vf->applyDisplacement(v, tr2);
                        visuMap[parent] = v;

                        for (auto& primitive : v->primitives)
                        {
                            primitive->transform = tr * primitive->transform;
                        }
                    }

                    if (child->getCollisionModel())
                    {
                        CollisionModelPtr c = child->getCollisionModel();
                        VisualizationNodePtr v = child->getCollisionModel()->getVisualization();
                        VisualizationFactoryPtr vf = VisualizationFactory::first(NULL);
                        Eigen::Matrix4f tr2 = tr;
                        //tr2.block(0, 3, 3, 1) *= 0.001f; // m is needed here?
                        vf->applyDisplacement(v, tr2);
                        v->createTriMeshModel(); // update trimesh model
                        c->setVisualization(v);
                        colMap[parent] = c;

                        for (auto& primitive : v->primitives)
                        {
                            primitive->transform = tr * primitive->transform;
                        }
                    }

                    // exchange physics
                    physicsMap[parent] = child->physics;
                    // change local com to new coord system
                    Eigen::Vector4f l;
                    SceneObject::Physics p = physicsMap[parent];
                    Eigen::Vector3f loc = p.localCoM;
                    l << loc(0), loc(1), loc(2), 1.0f;
                    physicsMap[parent].localCoM = (tr * l).head(3);

                    if (physicsMap.find(child) == physicsMap.end())
                    {
                        // no entry for child, set it to empty, may be overwritten later on
                        physicsMap[child] = SceneObject::Physics();
                    }

                    // exchange sensors
                    std::vector<SceneObjectPtr> childChildren =
                        robot->getRobotNode(nodeName)->getChildren();

                    for (const auto& cc : childChildren)
                    {
                        SensorPtr cs = std::dynamic_pointer_cast<Sensor>(cc);

                        if (cs)
                        {
                            // cloning sensor
                            SensorPtr newSensor = cs->clone(parent);
                            sensorMap[parent].push_back(newSensor);
                        }
                    }
                }
                else
                {
                    localTransformations[child] = child->getLocalTransformation();
                    directionInversion[child] = false;

                    if (child->getVisualization())
                    {
                        visuMap[child] = child->getVisualization();
                    }

                    if (child->getCollisionModel())
                    {
                        colMap[child] = child->getCollisionModel();
                    }

                    // clone sensors
                    std::vector<SceneObjectPtr> childChildren =
                        robot->getRobotNode(nodeName)->getChildren();

                    for (const auto& cc : childChildren)
                    {
                        SensorPtr cs = std::dynamic_pointer_cast<Sensor>(cc);

                        if (cs)
                        {
                            // cloning sensor
                            SensorPtr newSensor = cs->clone(child);
                            sensorMap[child].push_back(newSensor);
                        }
                    }
                }
            }

            // if parent has no parent: reset local transformation
            if (localTransformations.find(parent) == localTransformations.end())
            {
                localTransformations[parent] = Eigen::Matrix4f::Identity();
                directionInversion[parent] = false;
            }
        }

        // apply all transformations
        NodeTransformationMapT::iterator it = localTransformations.begin();

        while (it != localTransformations.end())
        {
            it->first->localTransformation = it->second;
            std::map<RobotNodePtr, bool>::iterator inv_it = directionInversion.find(it->first);
            VR_ASSERT(inv_it != directionInversion.end());
            if (inv_it->second)
            {
                RobotNodeRevolutePtr rotJoint =
                    std::dynamic_pointer_cast<RobotNodeRevolute>(it->first);
                if (rotJoint)
                {
                    rotJoint->jointRotationAxis *= -1.0f;
                }

                RobotNodePrismaticPtr prismaticJoint =
                    std::dynamic_pointer_cast<RobotNodePrismatic>(it->first);
                if (prismaticJoint)
                {
                    prismaticJoint->jointTranslationDirection *= -1.0f;
                }
            }

            it++;
        }

        std::vector<RobotNodePtr> nodes = newRobot->getRobotNodes();

        for (auto& node : nodes)
        {
            if (visuMap.find(node) != visuMap.end())
            {
                node->setVisualization(visuMap[node]);
            }
            else
            {
                node->setVisualization(VisualizationNodePtr());
            }

            if (colMap.find(node) != colMap.end())
            {
                node->setCollisionModel(colMap[node]);
            }
            else
            {
                node->setCollisionModel(CollisionModelPtr());
            }

            if (physicsMap.find(node) != physicsMap.end())
            {
                node->physics = physicsMap[node];
            }

            if (sensorMap.find(node) != sensorMap.end())
            {
                auto sensors = sensorMap[node];

                for (const auto& s : sensors)
                {
                    node->registerSensor(s);
                }
            }
        }

        newRobot->getRootNode()->initialize();

        return newRobot;
    }

    RobotPtr
    RobotFactory::clone(RobotPtr robot,
                        const std::string& name,
                        CollisionCheckerPtr collisionChecker,
                        float scaling)
    {
        THROW_VR_EXCEPTION_IF(!robot, "NULL data");

        if (!collisionChecker)
        {
            collisionChecker = robot->getCollisionChecker();
        }

        VirtualRobot::RobotPtr result = robot->extractSubPart(
            robot->getRootNode(), robot->getType(), name, true, true, collisionChecker, scaling);
        result->setGlobalPose(robot->getGlobalPose());
        return result;
    }

    void
    RobotFactory::getChildNodes(RobotNodePtr nodeA,
                                RobotNodePtr nodeExclude,
                                std::vector<RobotNodePtr>& appendNodes)
    {
        THROW_VR_EXCEPTION_IF(!nodeA, "NULL data");
        std::vector<SceneObjectPtr> children = nodeA->getChildren();
        std::vector<RobotNodePtr> childNodes;

        for (const auto& c : children)
        {
            RobotNodePtr cRN = std::dynamic_pointer_cast<RobotNode>(c);

            if (cRN && cRN != nodeExclude)
            {
                appendNodes.push_back(cRN);
                getChildNodes(cRN, nodeExclude, appendNodes);
            }
        }
    }

    void
    RobotFactory::getChildSensorNodes(RobotNodePtr nodeA,
                                      RobotNodePtr nodeExclude,
                                      std::vector<SensorPtr>& appendNodes)
    {
        THROW_VR_EXCEPTION_IF(!nodeA, "NULL data");
        std::vector<SceneObjectPtr> children = nodeA->getChildren();
        std::vector<RobotNodePtr> childNodes;

        for (const auto& c : children)
        {
            SensorPtr cS = std::dynamic_pointer_cast<Sensor>(c);
            RobotNodePtr cRN = std::dynamic_pointer_cast<RobotNode>(c);

            if (cS)
            {
                appendNodes.push_back(cS);
            }

            if (cRN && cRN != nodeExclude)
            {
                getChildSensorNodes(cRN, nodeExclude, appendNodes);
            }
        }
    }

    RobotNodePtr
    RobotFactory::accumulateTransformations(RobotPtr robot,
                                            RobotNodePtr nodeA,
                                            RobotNodePtr nodeAClone,
                                            RobotNodePtr nodeB,
                                            Eigen::Matrix4f& storeTrafo)
    {
        THROW_VR_EXCEPTION_IF(!robot, "NULL data");
        THROW_VR_EXCEPTION_IF(!nodeA, "NULL data");
        if (nodeA == nodeB)
        {
            return RobotNodePtr();
        }

        std::vector<RobotNodePtr> childNodes;
        std::vector<SensorPtr> childSensorNodes;

        getChildNodes(nodeA, nodeB, childNodes);
        getChildSensorNodes(nodeA, nodeB, childSensorNodes);

        if (childNodes.size() == 0)
        {
            return RobotNodePtr();
        }

        storeTrafo = Eigen::Matrix4f::Identity();

        if (nodeA && nodeB)
        {
            Eigen::Matrix4f startPose = nodeA->getGlobalPose();
            Eigen::Matrix4f goalPose = nodeB->getParent()->getGlobalPose();
            storeTrafo = simox::math::inverted_pose(startPose) * goalPose;
        }

        RobotNodePtr res = createUnitedRobotNode(
            robot, childNodes, nodeA, nodeAClone, Eigen::Matrix4f::Identity(), childSensorNodes);

        return res;
    }

    RobotNodePtr
    RobotFactory::createUnitedRobotNode(RobotPtr robot,
                                        const std::vector<RobotNodePtr>& nodes,
                                        RobotNodePtr parent,
                                        RobotNodePtr parentClone,
                                        const Eigen::Matrix4f& trafo,
                                        const std::vector<SensorPtr>& sensors)
    {
        THROW_VR_EXCEPTION_IF(!robot, "NULL data");

        auto rnf = RobotNodeFixedFactory::createInstance(nullptr);
        SceneObject::Physics p;
        VisualizationNodePtr v;
        CollisionModelPtr c;
        std::string name = "root";

        if (parentClone)
        {
            name = parentClone->getName() + "_FixedTrafo";
        }

        if (nodes.size() == 0)
        {
            RobotNodePtr newRN = rnf->createRobotNode(robot,
                                                      name,
                                                      v,
                                                      c,
                                                      0,
                                                      0,
                                                      0,
                                                      trafo,
                                                      Eigen::Vector3f::Zero(),
                                                      Eigen::Vector3f::Zero(),
                                                      p);

            if (parentClone)
            {
                newRN->initialize(parentClone);
            }

            // attach sensors
            for (const auto& sensor : sensors)
            {
                SensorPtr s = sensor->clone(newRN);
            }

            return newRN;
        }

        VisualizationFactoryPtr vf = VisualizationFactory::first(NULL);
        std::vector<VisualizationNodePtr> visus;
        std::vector<VisualizationNodePtr> colVisus;
        float kg = 0;


        for (const auto& node : nodes)
        {
            if (node->getVisualization())
            {
                visus.push_back(node->getVisualization());
            }

            if (node->getCollisionModel() && node->getCollisionModel()->getVisualization())
            {
                colVisus.push_back(node->getCollisionModel()->getVisualization());
            }

            kg += node->getMass();
        }

        if (visus.size() > 0)
        {
            v = vf->createUnitedVisualization(visus)->clone();
            if (parent)
            {
                Eigen::Matrix4f invTr = simox::math::inverted_pose(parent->getGlobalPose());
                vf->applyDisplacement(v, invTr);
            }
        }

        if (colVisus.size() > 0)
        {
            VisualizationNodePtr colVisu = vf->createUnitedVisualization(colVisus)->clone();
            if (parent)
            {
                Eigen::Matrix4f invTr = simox::math::inverted_pose(parent->getGlobalPose());
                vf->applyDisplacement(colVisu, invTr);
            }
            c.reset(new CollisionModel(colVisu, nodes[0]->getName()));
        }

        p.massKg = kg;

        RobotNodePtr newRN = rnf->createRobotNode(
            robot, name, v, c, 0, 0, 0, trafo, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), p);

        newRN->initialize(parentClone);

        const Eigen::Matrix4f trafoToNewRN = parent ? parent->getGlobalPose() * trafo : trafo;

        // add primitive models
        for (const auto& node : nodes)
        {
            const auto& primitiveApproximation = node->getPrimitiveApproximation();
            const Eigen::Matrix4f localTransformation =
                simox::math::inverted_pose(trafoToNewRN) * node->getGlobalPose();
            newRN->getPrimitiveApproximation().join(
                primitiveApproximation.clone().localTransformation(localTransformation));
        }

        // attach sensors
        for (const auto& sensor : sensors)
        {
            SensorPtr s = sensor->clone(newRN);
            Eigen::Matrix4f t = simox::math::inverted_pose(trafoToNewRN) * sensor->getGlobalPose();
            s->setRobotNodeToSensorTransformation(t);
        }

        return newRN;
    }

    void
    RobotFactory::cloneRNS(const Robot& from, RobotPtr to)
    {
        for (const auto& rns : from.getRobotNodeSets())
        {
            bool hasNodes = true;
            for (const auto& nodeName : rns->getNodeNames())
            {
                if (!to->hasRobotNode(nodeName))
                {
                    hasNodes = false;
                    break;
                }
            }
            const auto& tcp = rns->getTCP();
            if (hasNodes && (!tcp || to->hasRobotNode(tcp->getName())))
            {
                if (const auto& kinRoot = rns->getKinematicRoot())
                {
                    if (!to->hasRobotNode(kinRoot->getName()))
                    {
                        rns->clone(to, to->getRootNode());
                        continue;
                    }
                }
                rns->clone(to);
            }
        }
    }

    RobotPtr
    RobotFactory::cloneSubSet(RobotPtr robot,
                              RobotNodeSetPtr rns,
                              const std::string& name,
                              bool addTCP)
    {
        THROW_VR_EXCEPTION_IF(!robot, "NULL data");
        THROW_VR_EXCEPTION_IF(!rns, "NULL data");
        THROW_VR_EXCEPTION_IF(rns->getRobot() != robot, "Inconsitent data");

        std::vector<RobotNodePtr> nodes = rns->getAllRobotNodes();
        THROW_VR_EXCEPTION_IF(nodes.size() == 0, "0 data");

        // ensure tcp is part of nodes
        if (addTCP && rns->getTCP() && !rns->hasRobotNode(rns->getTCP()))
            nodes.push_back(rns->getTCP());

        // ensure kinemtic root is part of nodes
        if (rns->getKinematicRoot() && !rns->hasRobotNode(rns->getKinematicRoot()))
        {
            nodes.insert(nodes.begin(), rns->getKinematicRoot());
        }

        RobotNodePtr startNode = rns->getKinematicRoot();
        if (!startNode)
        {
            startNode = nodes[0];
        }

        for (size_t i = 1; i < nodes.size(); i++)
        {
            RobotNodePtr a = nodes[i - 1];
            RobotNodePtr b = nodes[i];

            if (!a->hasChild(b, true))
            {
                std::stringstream ss;
                ss << "Node " << a->getName() << " is not parent of " << b->getName();
                THROW_VR_EXCEPTION(ss.str());
            }
        }

        // first create initial node
        std::vector<RobotNodePtr> initialNodes = startNode->getAllParents();
        // check for static nodes which are not parent of startNode
        std::vector<RobotNodePtr> allNodes = robot->getRobotNodes();
        for (const auto& rn : allNodes)
        {
            bool isFixed = true;
            for (const auto& node : nodes)
            {
                if (rn->hasChild(node, true))
                {
                    isFixed = false;
                    break;
                }
            }
            if (isFixed &&
                std::find(initialNodes.begin(), initialNodes.end(), rn) == initialNodes.end())
            {
                // check if rn is child of the nodes in the rns
                if (!startNode->hasChild(rn, true))
                {
                    initialNodes.push_back(rn);
                }
            }
        }

        RobotPtr result(new LocalRobot(name, robot->getType()));

        Eigen::Matrix4f currentTrafo = Eigen::Matrix4f::Identity();

        if (startNode->getParent())
        {
            currentTrafo = startNode->getParent()->getGlobalPose();
        }

        // collect sensor nodes
        std::vector<SensorPtr> childSensorNodes;
        for (const auto& rn : initialNodes)
        {
            std::vector<SceneObjectPtr> c = rn->getChildren();
            for (const auto& j : c)
            {
                SensorPtr s = std::dynamic_pointer_cast<Sensor>(j);
                if (s)
                {
                    childSensorNodes.push_back(s);
                }
            }
        }

        RobotNodePtr rootNode = createUnitedRobotNode(result,
                                                      initialNodes,
                                                      RobotNodePtr(),
                                                      RobotNodePtr(),
                                                      Eigen::Matrix4f::Identity(),
                                                      childSensorNodes);
        result->setRootNode(rootNode);
        RobotNodePtr currentParent = rootNode;

        for (size_t i = 0; i < nodes.size(); i++)
        {
            RobotNodePtr newNode = nodes[i]->clone(result, false, currentParent);
            Eigen::Matrix4f newTrafo = currentTrafo * newNode->getLocalTransformation();
            newNode->setLocalTransformation(newTrafo);
            currentParent = newNode;
            currentTrafo.setIdentity();

            RobotNodePtr secondNode;

            if (i < nodes.size() - 1)
            {
                secondNode = nodes[i + 1];
            }
            else
            {
                std::cout << "end";
            }

            RobotNodePtr newNodeFixed =
                accumulateTransformations(result, nodes[i], newNode, secondNode, currentTrafo);

            if (newNodeFixed)
            {
                currentParent = newNodeFixed;
            }
        }

        cloneRNS(*robot.get(), result);

        result->setGlobalPose(robot->getGlobalPose());
        if (robot->getHumanMapping().has_value())
        {
            result->registerHumanMapping(robot->getHumanMapping().value());
        }
        return result;
    }

    RobotPtr
    RobotFactory::cloneUniteSubsets(RobotPtr robot,
                                    const std::string& name,
                                    std::vector<std::string> uniteWithAllChildren)
    {
        THROW_VR_EXCEPTION_IF(!robot, "NULL data");
        if (uniteWithAllChildren.size() == 0)
        {
            return RobotFactory::clone(robot, robot->getName());
        }

        for (const auto& i : uniteWithAllChildren)
        {
            THROW_VR_EXCEPTION_IF(!robot->hasRobotNode(i), "Could not find RobotNode in robot");
        }


        RobotPtr result(new LocalRobot(name, robot->getType()));


        RobotNodePtr currentNode = robot->getRootNode();
        RobotNodePtr currentNodeClone = currentNode->clone(result, false);
        result->setRootNode(currentNodeClone);

        cloneRecursiveUnite(result, currentNode, currentNodeClone, uniteWithAllChildren);

        cloneRNS(*robot.get(), result);

        result->setGlobalPose(robot->getGlobalPose());
        if (robot->getHumanMapping().has_value())
        {
            result->registerHumanMapping(robot->getHumanMapping().value());
        }
        return result;
    }

    RobotPtr
    RobotFactory::createReducedModel(Robot& robot,
                                     const std::vector<std::string>& actuatedJointNames,
                                     const std::vector<std::string>& otherNodeNames)
    {
        const Eigen::Matrix4f globalPose = robot.getGlobalPose();
        robot.setGlobalPose(Eigen::Matrix4f::Identity());

        std::set<std::string> joint_set(actuatedJointNames.begin(), actuatedJointNames.end());
        std::set<std::string> other_set(otherNodeNames.begin(), otherNodeNames.end());

        // Check joint nodes and set robot joints to default value
        std::map<std::string, float> joint_values;
        std::map<std::string, bool> enforce_joint_limits;
        for (const auto& joint_name : actuatedJointNames)
        {
            if (robot.hasRobotNode(joint_name))
            {
                auto node = robot.getRobotNode(joint_name);
                if (!node->isJoint())
                {
                    VR_WARNING << "Robot node " << joint_name << " is not a joint node";
                    return nullptr;
                }
                joint_values[joint_name] = node->getJointValue();
                enforce_joint_limits[joint_name] = node->getEnforceJointLimits();
                // Set joints do default starting value
                node->setEnforceJointLimits(false); // required if 0. is not within in the limits
                node->setJointValueNoUpdate(0.);
            }
            else
            {
                VR_WARNING << "Robot does not contain node " << joint_name;
                return nullptr;
            }
        }
        robot.updatePose();

        RobotPtr reducedModel =
            std::make_shared<LocalRobot>("Reduced_" + robot.getName(), robot.getType());

        struct Node
        {
            RobotNodePtr node;
            RobotNodePtr node_cloned = nullptr;
            bool is_actuated_joint = false;
            RobotNodePtr parentNode_cloned = nullptr;
            std::vector<RobotNodePtr> childNodes = std::vector<RobotNodePtr>();
        };

        std::queue<Node> nodes;
        nodes.push(Node{.node = robot.getRootNode()});

        std::function<void(RobotNodePtr, Node&)> collect;
        collect = [joint_set, other_set, &collect, &nodes](RobotNodePtr currentNode,
                                                           Node& node) -> void
        {
            for (const auto& child : currentNode->getChildren())
            {
                if (const auto& childNode = std::dynamic_pointer_cast<RobotNode>(child))
                {
                    const bool is_actuated_joint = joint_set.count(childNode->getName()) > 0;
                    if (!is_actuated_joint && other_set.count(childNode->getName()) == 0)
                    {
                        node.childNodes.push_back(childNode);
                        collect(childNode, node);
                    }
                    else
                    {
                        nodes.push(Node{.node = childNode,
                                        .is_actuated_joint = is_actuated_joint,
                                        .parentNode_cloned = node.node_cloned});
                    }
                }
            }
        };

        const auto fixed_node_factory = RobotNodeFixedFactory::createInstance(nullptr);
        while (!nodes.empty())
        {
            auto n = nodes.front();
            nodes.pop();

            // If node is a joint but is not part of the joints it will be converted to a fixed transformation node
            if (!n.is_actuated_joint && n.node->isJoint())
            {
                n.node_cloned = fixed_node_factory->createRobotNode(
                    reducedModel,
                    n.node->getName(),
                    n.node->getVisualization() ? n.node->getVisualization()->clone() : nullptr,
                    n.node->getCollisionModel() ? n.node->getCollisionModel()->clone() : nullptr,
                    0.f,
                    0.f,
                    0.f,
                    n.node->getLocalTransformation(),
                    Eigen::Vector3f::Zero(),
                    Eigen::Vector3f::Zero(),
                    n.node->getPhysics());
                n.node_cloned->primitiveApproximation = n.node->getPrimitiveApproximation().clone();
                if (n.parentNode_cloned)
                {
                    n.node_cloned->initialize(n.parentNode_cloned);
                }
                n.node_cloned->basePath = n.node->basePath;
            }
            else
            {
                n.node_cloned = n.node->clone(reducedModel, false, n.parentNode_cloned);
            }
            // Probably will not be valid anymore after reducing robot as models will be missing
            n.node_cloned->visualizationModelXML = std::string();
            n.node_cloned->collisionModelXML = std::string();

            collect(n.node, n);

            if (!n.parentNode_cloned)
            {
                reducedModel->setRootNode(n.node_cloned);
            }
            else
            {
                Eigen::Matrix4f localTransformation =
                    simox::math::inverted_pose(n.parentNode_cloned->getGlobalPose()) *
                    n.node->getGlobalPose();
                n.node_cloned->setLocalTransformation(localTransformation);
                n.node_cloned->updateTransformationMatrices();
            }

            const VisualizationFactoryPtr vf = VisualizationFactory::first(NULL);
            std::vector<VisualizationNodePtr> visus;
            std::vector<VisualizationNodePtr> colVisus;
            for (const auto& childNode : n.childNodes)
            {
                if (childNode->getVisualization())
                {
                    visus.push_back(childNode->getVisualization());
                }
                if (childNode->getCollisionModel() &&
                    childNode->getCollisionModel()->getVisualization())
                {
                    colVisus.push_back(childNode->getCollisionModel()->getVisualization());
                }
                n.node_cloned->physics.massKg += childNode->getMass(); // TODO fix physics somehow
            }
            const Eigen::Matrix4f globalPoseInv =
                simox::math::inverted_pose(n.node_cloned->getGlobalPose());
            if (!visus.empty())
            {
                if (visus.size() == 1)
                {
                    n.node_cloned->setVisualization(visus.front()->clone());
                }
                else
                {
                    if (const auto visu = n.node->getVisualization())
                    {
                        visus.insert(visus.begin(), visu->clone());
                    }
                    auto v = vf->createUnitedVisualization(visus);
                    if (n.parentNode_cloned)
                    {
                        v->setLocalPose(globalPoseInv);
                    }
                    n.node_cloned->setVisualization(v);
                }
            }
            if (!colVisus.empty())
            {
                if (colVisus.size() == 1)
                {
                    n.node_cloned->setCollisionModel(
                        std::make_shared<CollisionModel>(colVisus.front(), n.node->getName()));
                }
                else
                {
                    if (const auto colModel = n.node->getCollisionModel())
                    {
                        if (const auto vis = colModel->getVisualization())
                        {
                            colVisus.insert(colVisus.begin(), vis->clone());
                        }
                    }
                    auto colVisu = vf->createUnitedVisualization(colVisus);
                    if (n.parentNode_cloned)
                    {
                        colVisu->setLocalPose(globalPoseInv);
                    }
                    n.node_cloned->setCollisionModel(
                        std::make_shared<CollisionModel>(colVisu, n.node->getName()));
                }
            }

            for (const auto& childNode : n.childNodes)
            {
                const auto& primitiveApproximation = childNode->getPrimitiveApproximation();
                n.node_cloned->getPrimitiveApproximation().join(
                    primitiveApproximation.clone().localTransformation(globalPoseInv *
                                                                       childNode->getGlobalPose()));

                // attach sensors
                for (const auto& sensor : childNode->getSensors())
                {
                    SensorPtr s = sensor->clone(n.node_cloned);
                    s->setRobotNodeToSensorTransformation(globalPoseInv * sensor->getGlobalPose());
                }
            }
        }

        reducedModel->setGlobalPose(globalPose);
        reducedModel->setJointValues(joint_values);

        for (const auto& [joint_name, enforce_limits] : enforce_joint_limits)
        {
            robot.getRobotNode(joint_name)->setEnforceJointLimits(enforce_limits);
        }

        cloneRNS(robot, reducedModel);

        if (robot.getHumanMapping().has_value())
        {
            reducedModel->registerHumanMapping(robot.getHumanMapping().value());
        }
        return reducedModel;
    }

    VirtualRobot::RobotPtr
    RobotFactory::createFlattenedModel(Robot& robot)
    {
        const Eigen::Matrix4f globalPose = robot.getGlobalPose();
        robot.setGlobalPose(Eigen::Matrix4f::Identity());

        RobotPtr flattenedRobot =
            std::make_shared<LocalRobot>("Flattened_" + robot.getName(), robot.getType());

        struct NodeToClone
        {
            RobotNodePtr nodeInOriginalRobot;
            RobotNodePtr parentInFlattenedRobot;
        };

        auto isBody = [](RobotNodePtr const& node) -> bool
        { return node->getVisualization() != nullptr and node->getChildren<RobotNode>().empty(); };

        auto isLink = [](RobotNodePtr const& node) -> bool {
            return node->getVisualization() == nullptr and
                   not node->getChildren<RobotNode>().empty();
        };

        auto pushAllChildren = [](RobotNodePtr const& original,
                                  RobotNodePtr const& cloned,
                                  std::queue<NodeToClone>& nodes)
        {
            for (const auto& child : original->getChildren())
            {
                auto const& childNode = std::dynamic_pointer_cast<RobotNode>(child);
                if (childNode == nullptr)
                {
                    continue;
                }

                nodes.push({childNode, cloned});
            }
        };

        struct BodyAndLink
        {
            RobotNodePtr body;
            RobotNodePtr linkOrJoint;
            std::vector<RobotNodePtr> others;
        };

        auto findBodyAndLink = [isBody,
                                isLink](RobotNodePtr const& jointNode) -> std::optional<BodyAndLink>
        {
            std::optional<RobotNodePtr> body;
            std::optional<RobotNodePtr> linkOrJoint;
            std::vector<RobotNodePtr> others;

            for (const auto& child : jointNode->getChildren())
            {
                auto const& childNode = std::dynamic_pointer_cast<RobotNode>(child);
                if (childNode == nullptr)
                {
                    continue;
                }

                if (isBody(childNode))
                {
                    if (body.has_value())
                    {
                        return std::nullopt;
                    }

                    body = childNode;
                }
                else if (isLink(childNode) or childNode->isJoint())
                {
                    if (linkOrJoint.has_value())
                    {
                        return std::nullopt;
                    }

                    linkOrJoint = childNode;
                }
                else
                {
                    others.emplace_back(childNode);
                }
            }

            if (body.has_value() and linkOrJoint.has_value())
            {
                return BodyAndLink{
                    .body = body.value(), .linkOrJoint = linkOrJoint.value(), .others = others};
            }

            return std::nullopt;
        };

        auto performFlattenedAttach =
            [&flattenedRobot](RobotNodePtr const& clonedParentJoint,
                              RobotNodePtr const& originalBody,
                              RobotNodePtr const& originalLinkOrJoint) -> RobotNodePtr
        {
            const bool cloneChildren = false;
            RobotNodePtr clonedBody =
                originalBody->clone(flattenedRobot, cloneChildren, clonedParentJoint);
            RobotNodePtr clonedLinkOrJoint =
                originalLinkOrJoint->clone(flattenedRobot, cloneChildren, clonedBody);

            Eigen::Matrix4f global_T_body = originalBody->getGlobalPose();
            Eigen::Matrix4f global_T_link = originalLinkOrJoint->getGlobalPose();

            Eigen::Matrix4f body_T_link = simox::math::inverted_pose(global_T_body) * global_T_link;

            clonedLinkOrJoint->setLocalTransformation(body_T_link);
            clonedLinkOrJoint->updateTransformationMatrices();

            return clonedLinkOrJoint;
        };

        std::queue<NodeToClone> nodes({{robot.getRootNode(), nullptr}});
        while (not nodes.empty())
        {
            auto [currentOriginal, currentClonedParent] = nodes.front();
            nodes.pop();

            const bool cloneChildren = false;
            RobotNodePtr currentClone =
                currentOriginal->clone(flattenedRobot, cloneChildren, currentClonedParent);

            if (currentClonedParent == nullptr)
            {
                flattenedRobot->setRootNode(currentClone);
            }

            while (currentOriginal->isJoint())
            {
                std::cout << "Checking joint: " << currentOriginal->getName();

                std::optional<BodyAndLink> bodyAndLink = findBodyAndLink(currentOriginal);

                if (not bodyAndLink.has_value())
                {
                    std::cout << " but no specials found!" << std::endl;
                    break;
                }

                auto [body, linkOrJoint, others] = bodyAndLink.value();
                std::cout << " and found body " << body->getName() << " and link/joint "
                          << linkOrJoint->getName() << std::endl;

                RobotNodePtr nextClone = performFlattenedAttach(currentClone, body, linkOrJoint);

                for (const auto& other : others)
                {
                    nodes.push({other, currentClone});
                }

                currentOriginal = linkOrJoint;
                currentClone = nextClone;
            }

            pushAllChildren(currentOriginal, currentClone, nodes);
        }

        robot.setGlobalPose(globalPose);
        flattenedRobot->setGlobalPose(globalPose);

        std::cout << flattenedRobot->toXML() << std::endl;

        return flattenedRobot;
    }

    void
    RobotFactory::cloneRecursiveUnite(RobotPtr robot,
                                      RobotNodePtr currentNode,
                                      RobotNodePtr currentNodeClone,
                                      std::vector<std::string> uniteWithAllChildren)
    {
        std::vector<SceneObjectPtr> c = currentNode->getChildren();


        for (auto& i : c)
        {
            if (std::find(uniteWithAllChildren.begin(), uniteWithAllChildren.end(), i->getName()) !=
                uniteWithAllChildren.end())
            {
                RobotNodePtr currentRN = std::dynamic_pointer_cast<RobotNode>(i);
                THROW_VR_EXCEPTION_IF(!currentRN, "Only RN allowed in list");
                RobotNodePtr currentRNClone = currentRN->clone(robot, false, currentNodeClone);

                std::vector<RobotNodePtr> childNodes;
                std::vector<SensorPtr> childSensorNodes;

                getChildNodes(currentRN, RobotNodePtr(), childNodes);
                getChildSensorNodes(currentRN, RobotNodePtr(), childSensorNodes);

                if (childNodes.size() > 0 || childSensorNodes.size() > 0)
                {
                    RobotNodePtr res = createUnitedRobotNode(robot,
                                                             childNodes,
                                                             currentRN,
                                                             currentRNClone,
                                                             Eigen::Matrix4f::Identity(),
                                                             childSensorNodes);
                    // res is automatically added
                }
            }
            else
            {
                RobotNodePtr currentRN = std::dynamic_pointer_cast<RobotNode>(i);
                if (currentRN)
                {
                    RobotNodePtr currentRNClone = currentRN->clone(robot, false, currentNodeClone);
                    cloneRecursiveUnite(robot, currentRN, currentRNClone, uniteWithAllChildren);
                }
                else
                {
                    SensorPtr s = std::dynamic_pointer_cast<Sensor>(i);
                    if (s)
                    {
                        s->clone(currentNodeClone);
                    }
                    else
                    {
                        VR_INFO << "Skipping node " << i->getName() << std::endl;
                    }
                }
            }
        }
    }

} // namespace VirtualRobot
