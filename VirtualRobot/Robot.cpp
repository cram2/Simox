
#include "Robot.h"
#include "RobotConfig.h"
#include "Trajectory.h"
#include "VirtualRobot.h"
#include "VirtualRobotException.h"
#include "Nodes/Sensor.h"
#include "Visualization/VisualizationNode.h"
#include "CollisionDetection/CollisionChecker.h"
#include "CollisionDetection/CollisionModel.h"
#include "EndEffector/EndEffector.h"
#include "Grasping/GraspSet.h"
#include "math/Helpers.h"


namespace VirtualRobot
{
    using std::cout;
    using std::endl;

    const RobotPtr Robot::NullPtr{nullptr};
    Robot::Robot(const std::string& name, const std::string& type)
        : SceneObject(name)
    {
        this->type = type;
        updateVisualization = true;
        use_mutex = true;
    }

    Robot::Robot()
        = default;

    Robot::~Robot()
        = default;

    LocalRobot::~LocalRobot()
    {
        // clean up connections
        std::map< std::string, RobotNodePtr > ::iterator it = robotNodeMap.begin();

        while (it != robotNodeMap.end())
        {
            std::vector<SceneObjectPtr> c = it->second->getChildren();

            for (const auto& j : c)
            {
                it->second->detachChild(j);
            }

            it++;
        }

        //int i = (int)rootNode.use_count();
        robotNodeSetMap.clear();
        robotNodeMap.clear();
        //i = (int)rootNode.use_count();
        rootNode.reset();
        //i = (int)rootNode.use_count();
    }

    LocalRobot::LocalRobot(const std::string& name, const std::string& type) : Robot(name, type)
    {
    }

    void LocalRobot::setRootNode(RobotNodePtr node)
    {
        attachChild(node);
        rootNode = node;
        node->initialize();

        //robotNodeMap.clear();
        if (!node)
        {
            VR_WARNING << "NULL root node..." << std::endl;
        }
        else
        {
            // create all globalposes
            applyJointValues();

            std::vector< RobotNodePtr > allNodes;
            node->collectAllRobotNodes(allNodes);

            for (auto& allNode : allNodes)
            {
                std::string name = allNode->getName();

                if (!this->hasRobotNode(name))
                {
                    VR_WARNING << "Robot node with name <" << name << "> was not registered, adding it to RobotNodeMap" << std::endl;
                    registerRobotNode(allNode);
                }
            }
        }
    }

    void Robot::setThreadsafe(bool flag)
    {
        use_mutex = flag;
    }

    RobotNodePtr LocalRobot::getRobotNode(const std::string& robotNodeName) const
    {
        if (robotNodeMap.find(robotNodeName) == robotNodeMap.end())
        {
            VR_WARNING << "No robot node with name <" << robotNodeName << "> defined." << std::endl;
            return RobotNodePtr();
        }

        return robotNodeMap.at(robotNodeName);
    }

    void LocalRobot::registerRobotNode(RobotNodePtr node)
    {
        if (!node)
        {
            return;
        }

        if (robotNodeMap.size() > 0)
        {
            // check for collision checker
            if (node->getCollisionChecker() != robotNodeMap.begin()->second->getCollisionChecker())
            {
                THROW_VR_EXCEPTION("Different Collision Checkers in " << node->getName() << " and " << robotNodeMap.begin()->second->getName());
            }
        }

        std::string robotNodeName = node->getName();

        if (robotNodeMap.find(robotNodeName) != robotNodeMap.end())
        {
            THROW_VR_EXCEPTION("There are (at least) two robot nodes with name <" << robotNodeName << "> defined, the second one is skipped!");
        }
        else
        {
            robotNodeMap[robotNodeName] = node;
        }
    }

    bool LocalRobot::hasRobotNode(RobotNodePtr node) const
    {
        if (!node)
        {
            return false;
        }

        std::string robotNodeName = node->getName();

        if (this->hasRobotNode(robotNodeName))
        {
            return (this->getRobotNode(robotNodeName) == node);
        }

        return false;
    }

    bool LocalRobot::hasRobotNode(const std::string& robotNodeName) const
    {
        return (robotNodeMap.find(robotNodeName) != robotNodeMap.end());
    }


    void LocalRobot::deregisterRobotNode(RobotNodePtr node)
    {
        if (!node)
        {
            return;
        }

        std::string robotNodeName = node->getName();
        std::map< std::string, RobotNodePtr >::iterator i = robotNodeMap.find(robotNodeName);

        if (i != robotNodeMap.end())
        {
            robotNodeMap.erase(i);
        }
    }


    void LocalRobot::registerRobotNodeSet(RobotNodeSetPtr nodeSet)
    {
        if (!nodeSet)
        {
            return;
        }

        std::string nodeSetName = nodeSet->getName();

        if (robotNodeSetMap.find(nodeSetName) != robotNodeSetMap.end())
        {
            VR_WARNING << "There are (at least) two robot node sets with name <" << nodeSetName << "> defined, the second one overwrites first definition!" << std::endl;
            // overwrite
        }

        robotNodeSetMap[nodeSetName] = nodeSet;
    }

    bool Robot::hasRobotNodeSet(RobotNodeSetPtr nodeSet) const
    {
        if (!nodeSet)
        {
            return false;
        }

        std::string nodeSetName = nodeSet->getName();

        return hasRobotNodeSet(nodeSetName);
    }

    bool LocalRobot::hasRobotNodeSet(const std::string& name) const
    {
        return robotNodeSetMap.find(name) != robotNodeSetMap.end();
    }

    void LocalRobot::deregisterRobotNodeSet(RobotNodeSetPtr nodeSet)
    {
        if (!nodeSet)
        {
            return;
        }

        std::string nodeSetName = nodeSet->getName();
        std::map< std::string, RobotNodeSetPtr >::iterator i = robotNodeSetMap.find(nodeSetName);

        if (i != robotNodeSetMap.end())
        {
            robotNodeSetMap.erase(i);
        }
    }


    /**
     * This method registers \p endEffector with the current VirtualRobot::Robot
     * instance.
     * It throws an exception if a VirtualRobot::EndEffector with the same name
     * has already been registered.
     */
    void LocalRobot::registerEndEffector(EndEffectorPtr endEffector)
    {
        if (!endEffector)
        {
            return;
        }

        std::string endEffectorName = endEffector->getName();

        if (endEffectorMap.find(endEffectorName) != endEffectorMap.end())
        {
            THROW_VR_EXCEPTION("Trying to register a second endeffector with name <" << endEffectorName << ">");
        }
        else
        {
            endEffectorMap[endEffectorName] = endEffector;
        }
    }


    /**
     * \return true if instance of VirtualRobot::Robot contains a reference to \p endEffector and false otherwise
     */
    bool Robot::hasEndEffector(EndEffectorPtr endEffector) const
    {
        if (!endEffector)
        {
            return false;
        }

        std::string endEffectorName = endEffector->getName();

        if (this->hasEndEffector(endEffectorName))
        {
            return (this->getEndEffector(endEffectorName) == endEffector);
        }

        return false;
    }


    /**
     * \return true if instance of VirtualRobot::Robot contains an endeffector with name \p endEffectorName and false otherwise
     */
    bool LocalRobot::hasEndEffector(const std::string& endEffectorName) const
    {
        if (endEffectorName.empty())
        {
            return false;
        }

        if (endEffectorMap.find(endEffectorName) == endEffectorMap.end())
        {
            return false;
        }

        return true;
    }


    /**
     * \return reference to endeffector with name \p endEffectorName or Null-Pointer otherwise
     */
    EndEffectorPtr LocalRobot::getEndEffector(const std::string& endEffectorName) const
    {
        if (endEffectorMap.find(endEffectorName) == endEffectorMap.end())
        {
            VR_WARNING << "No endeffector node with name <" << endEffectorName << "> defined." << std::endl;
            return EndEffectorPtr();
        }

        return endEffectorMap.at(endEffectorName);
    }

    /**
     * This method stores all endeffectors belonging to the robot in \p storeEEF.
     * If there are no registered endeffectors \p storeEEF will be empty.
     */
    void LocalRobot::getEndEffectors(std::vector<EndEffectorPtr>& storeEEF) const
    {
        storeEEF.clear();
        storeEEF.reserve(endEffectorMap.size());
        std::map<std::string, EndEffectorPtr>::const_iterator iterator = endEffectorMap.begin();

        while (endEffectorMap.end() != iterator)
        {
            storeEEF.push_back(iterator->second);
            ++iterator;
        }
    }


    /**
    * Can be called internally, when lock is already set.
    */
    void LocalRobot::applyJointValuesNoLock()
    {
        rootNode->updatePose(globalPose);
    }


    std::vector<EndEffectorPtr> Robot::getEndEffectors() const
    {
        std::vector<EndEffectorPtr> res;
        getEndEffectors(res);
        return res;
    }


    /**
     * \return VirtualRobot::Robot::name
     */
    std::string Robot::getName() const
    {
        return name;
    }

    /**
     * \return VirtualRobot::Robot::type
     */
    std::string Robot::getType() const
    {
        return type;
    }
    
    void Robot::setType(const std::string& type) 
    {
        this->type = type;
    }
    
    void Robot::setName(const std::string& name) 
    {
        this->name = name;
    }

    void Robot::print(bool /*printChildren*/, bool /*printDecoration*/) const
    {
        std::cout << "******** Robot ********" << std::endl;
        std::cout << "* Name: " << name << std::endl;
        std::cout << "* Type: " << type << std::endl;

        if (this->getRootNode())
        {
            std::cout << "* Root Node: " << this->getRootNode()->getName() << std::endl;
        }
        else
        {
            std::cout << "* Root Node: not set" << std::endl;
        }

        std::cout << std::endl;

        if (this->getRootNode())
        {
            this->getRootNode()->print(true, true);
        }

        std::cout << std::endl;

        std::vector<RobotNodeSetPtr> robotNodeSets = this->getRobotNodeSets();

        if (robotNodeSets.size() > 0)
        {
            std::cout << "* RobotNodeSets:" << std::endl;

            std::vector<RobotNodeSetPtr>::iterator iter = robotNodeSets.begin();

            while (iter != robotNodeSets.end())
            {
                std::cout << "----------------------------------" << std::endl;
                (*iter)->print();
                iter++;
            }

            std::cout << std::endl;
        }

        std::cout << "******** Robot ********" << std::endl;
    }


    /**
     * This method returns a reference to Robot::rootNode;
     */
    RobotNodePtr LocalRobot::getRootNode() const
    {
        return this->rootNode;
    }

    /**
     * Update the transformations of all joints
     */
    void Robot::applyJointValues()
    {
        WriteLock lock(mutex, use_mutex);
        this->getRootNode()->updatePose(this->getGlobalPose());
    }

    /**
     * Can be called internally, when lock is already set.
     */
    void Robot::applyJointValuesNoLock()
    {
        this->getRootNode()->updatePose(this->getGlobalPose());
        //rootNode->updatePose(globalPose);
    }

    /*float Robot::getRadianToMMfactor() const
    {
        return radianToMMfactor;
    }

    void Robot::setRadianToMMfactor(float value)
    {
        radianToMMfactor = value;
    }*/

    /**
     * This method stores all nodes belonging to the robot in \p storeNodes.
     * If there are no registered nodes \p storeNodes will be empty.
     */
    void LocalRobot::getRobotNodes(std::vector< RobotNodePtr >& storeNodes, bool clearVector /*=true*/) const
    {
        if (clearVector)
        {
            storeNodes.clear();
        }

        storeNodes.reserve(robotNodeMap.size());
        std::map< std::string, RobotNodePtr>::const_iterator iterator = robotNodeMap.begin();

        while (robotNodeMap.end() != iterator)
        {
            storeNodes.push_back(iterator->second);
            ++iterator;
        }
    }

    std::vector< RobotNodePtr > Robot::getRobotNodes() const
    {
        std::vector< RobotNodePtr > res;
        getRobotNodes(res);
        return res;
    }

    std::vector<std::string> Robot::getRobotNodeNames() const
    {
        std::vector<std::string> res;
        const auto nodes = getRobotNodes();
        res.reserve(nodes.size());
        for (const auto& node : nodes)
        {
            res.emplace_back(node->getName());
        }
        return res;
    }

    void Robot::setUpdateVisualization(bool enable)
    {
        SceneObject::setUpdateVisualization(enable);

        std::vector<RobotNodePtr> robotNodes = this->getRobotNodes();
        std::vector<RobotNodePtr> ::const_iterator iterator = robotNodes.begin();

        while (robotNodes.end() != iterator)
        {
            (*iterator)->setUpdateVisualization(enable);
            ++iterator;
        }
    }

    void Robot::setUpdateCollisionModel(bool enable)
    {
        SceneObject::setUpdateCollisionModel(enable);

        std::vector<RobotNodePtr> robotNodes = this->getRobotNodes();
        std::vector<RobotNodePtr> ::const_iterator iterator = robotNodes.begin();

        while (robotNodes.end() != iterator)
        {
            (*iterator)->setUpdateCollisionModel(enable);
            ++iterator;
        }
    }

    std::shared_ptr<Robot> Robot::shared_from_this() const
    {
        auto sceneObject = SceneObject::shared_from_this();
        std::shared_ptr<const Robot> robotPtr = std::static_pointer_cast<const Robot>(sceneObject);
        std::shared_ptr<Robot> result = std::const_pointer_cast<Robot>(robotPtr);
        return result;
    }

    RobotNodeSetPtr LocalRobot::getRobotNodeSet(const std::string& nodeSetName) const
    {
        auto it = robotNodeSetMap.find(nodeSetName);
        if (it == robotNodeSetMap.end())
        {
            VR_WARNING << "No robot node set with name <" << nodeSetName << "> defined." << std::endl;
            return RobotNodeSetPtr();
        }

        return it->second;
    }

    /**
     * This method stores all endeffectors belonging to the robot in \p storeEEF.
     * If there are no registered endeffectors \p storeEEF will be empty.
     */
    void LocalRobot::getRobotNodeSets(std::vector<RobotNodeSetPtr>& storeNodeSets) const
    {
        storeNodeSets.clear();
        storeNodeSets.reserve(robotNodeSetMap.size());
        std::map<std::string, RobotNodeSetPtr>::const_iterator iterator = robotNodeSetMap.begin();

        while (robotNodeSetMap.end() != iterator)
        {
            storeNodeSets.push_back(iterator->second);
            ++iterator;
        }
    }

    std::vector<RobotNodeSetPtr> Robot::getRobotNodeSets() const
    {
        std::vector<RobotNodeSetPtr> res;
        getRobotNodeSets(res);
        return res;
    }

    std::vector<std::string> Robot::getRobotNodeSetNames() const
    {
        std::vector<std::string> res;
        const auto sets = getRobotNodeSets();
        res.reserve(sets.size());
        for (const auto& set : sets)
        {
            res.emplace_back(set->getName());
        }
        return res;
    }

    void Robot::highlight(VisualizationPtr visualization, bool enable)
    {
        std::vector<RobotNodePtr> robotNodes = this->getRobotNodes();
        std::vector<RobotNodePtr>::const_iterator iterator = robotNodes.begin();

        while (robotNodes.end() != iterator)
        {
            (*iterator)->highlight(visualization, enable);
            ++iterator;
        }
    }

    void Robot::showStructure(bool enable, const std::string& type)
    {
        std::vector<RobotNodePtr> robotNodes = this->getRobotNodes();
        std::vector<RobotNodePtr>::const_iterator iterator = robotNodes.begin();

        while (robotNodes.end() != iterator)
        {
            (*iterator)->showStructure(enable, type);
            ++iterator;
        }

    }

    void Robot::showPhysicsInformation(bool enableCoM, bool enableInertial, VisualizationNodePtr comModel)
    {
        std::vector<RobotNodePtr> robotNodes = this->getRobotNodes();
        std::vector<RobotNodePtr>::const_iterator iterator = robotNodes.begin();

        while (robotNodes.end() != iterator)
        {
            (*iterator)->showPhysicsInformation(enableCoM, enableInertial, comModel);
            ++iterator;
        }

    }

    void Robot::showCoordinateSystems(bool enable, const std::string& type)
    {
        std::vector<RobotNodePtr> robotNodes = this->getRobotNodes();
        std::vector<RobotNodePtr>::const_iterator iterator = robotNodes.begin();

        while (robotNodes.end() != iterator)
        {
            (*iterator)->showCoordinateSystem(enable, 1.0f, nullptr, type);
            ++iterator;
        }
    }

    void Robot::setupVisualization(bool showVisualization, bool showAttachedVisualizations)
    {
        std::vector<RobotNodePtr> robotNodes = this->getRobotNodes();
        std::vector<RobotNodePtr>::const_iterator iterator = robotNodes.begin();

        while (robotNodes.end() != iterator)
        {
            (*iterator)->setupVisualization(showVisualization, showAttachedVisualizations);
            ++iterator;
        }

    }

    int Robot::getNumFaces(bool collisionModel)
    {
        int res = 0;
        std::vector<RobotNodePtr> robotNodes = this->getRobotNodes();
        std::vector<RobotNodePtr>::const_iterator iterator = robotNodes.begin();

        while (robotNodes.end() != iterator)
        {
            res += (*iterator)->getNumFaces(collisionModel);
            ++iterator;
        }

        return res;
    }

    void Robot::setGlobalPose(const Eigen::Matrix4f& globalPose)
    {
        setGlobalPose(globalPose, true);
    }

    VirtualRobot::CollisionCheckerPtr Robot::getCollisionChecker()
    {
        std::vector<RobotNodePtr> robotNodes = this->getRobotNodes();

        if (robotNodes.empty())
        {
            return CollisionChecker::getGlobalCollisionChecker();
        }

        return (*robotNodes.begin())->getCollisionChecker();
    }

    void LocalRobot::setGlobalPose(const Eigen::Matrix4f& globalPose)
    {
        setGlobalPose(globalPose, true);
    }

    void LocalRobot::setGlobalPose(const Eigen::Matrix4f& globalPose, bool applyJointValues /*= true*/)
    {
        WriteLock lock(mutex, use_mutex);
        this->globalPose = globalPose;

        if (applyJointValues)
        {
            applyJointValuesNoLock();
        }
    }

    Eigen::Matrix4f LocalRobot::getGlobalPose() const
    {
        ReadLock lock(mutex, use_mutex);
        return globalPose;
    }


    Eigen::Vector3f Robot::getCoMLocal()
    {
        Eigen::Vector3f com = getCoMGlobal();
        return toLocalCoordinateSystemVec(com);
    }

    Eigen::Vector3f Robot::getCoMGlobal()
    {
        Eigen::Vector3f res;
        res.setZero();

        float m = getMass();

        if (m <= 0)
        {
            return res;
        }

        std::vector<RobotNodePtr> robotNodes = this->getRobotNodes();
        std::vector<RobotNodePtr>::const_iterator iterator = robotNodes.begin();

        while (robotNodes.end() != iterator)
        {
            res += (*iterator)->getCoMGlobal() * (*iterator)->getMass() / m;

            iterator++;
        }

        return res;
    }

    float Robot::getMass() const
    {
        float res = 0;
        std::vector<RobotNodePtr> robotNodes = this->getRobotNodes();
        std::vector<RobotNodePtr>::const_iterator iterator = robotNodes.begin();

        while (robotNodes.end() != iterator)
        {
            res += (*iterator)->getMass();
            iterator++;
        }

        return res;
    }

    std::vector< CollisionModelPtr > Robot::getCollisionModels() const
    {
        std::vector< CollisionModelPtr > result;
        std::vector<RobotNodePtr> robotNodes = this->getRobotNodes();
        std::vector<RobotNodePtr>::const_iterator iterator = robotNodes.begin();

        while (robotNodes.end() != iterator)
        {
            if ((*iterator)->getCollisionModel())
            {
                result.push_back((*iterator)->getCollisionModel());
            }

            iterator++;
        }

        return result;
    }

    VirtualRobot::RobotPtr Robot::extractSubPart(RobotNodePtr startJoint,
            const std::string& newRobotType,
            const std::string& newRobotName,
            bool cloneRNS,
            bool cloneEEFs,
            CollisionCheckerPtr collisionChecker,
            float scaling,
            bool preventCloningMeshesIfScalingIs1)
    {
        THROW_VR_EXCEPTION_IF(!startJoint, " StartJoint is nullptr");
        THROW_VR_EXCEPTION_IF(!hasRobotNode(startJoint), " StartJoint '" + startJoint->getName() + "' is not part of this robot '" + getName() + "'");
        //THROW_VR_EXCEPTION_IF(scaling <= 0, " Scaling must be >0.");
        // If scaling is <= 0 this->scaling is used instead. This enables different scalings while still able to clone the robot

        CollisionCheckerPtr colChecker = collisionChecker;

        if (!colChecker)
        {
            colChecker = this->getCollisionChecker();
        }

        //stefan Warning!!!!! which robot-type to create
        RobotPtr result(new LocalRobot(newRobotName, newRobotType));

        RobotNodePtr rootNew = startJoint->clone(result, true, RobotNodePtr(), colChecker, scaling, preventCloningMeshesIfScalingIs1);
        THROW_VR_EXCEPTION_IF(!rootNew, "Clone failed...");
        result->setRootNode(rootNew);
        result->setScaling(scaling > 0 ? scaling : this->scaling);

        std::vector<RobotNodePtr> rn = result->getRobotNodes();

        // check for RNS that are covered by subpart
        if (cloneRNS)
        {
            std::vector<RobotNodeSetPtr> robotNodeSets = this->getRobotNodeSets();
            std::vector<RobotNodeSetPtr>::const_iterator iterator = robotNodeSets.begin();

            while (robotNodeSets.end() != iterator)
            {
                if ((*iterator)->nodesSufficient(rn))
                {
                    RobotNodeSetPtr rns = (*iterator)->clone(result);

                    // already done in rns->clone()
                    //if (rns)
                    //  result->registerRobotNodeSet(rns);
                }

                ++iterator;
            }
        }

        if (cloneEEFs)
        {
            std::vector<EndEffectorPtr> endEffectors = this->getEndEffectors();
            std::vector<EndEffectorPtr>::const_iterator iterator = endEffectors.begin();

            while (endEffectors.end() != iterator)
            {
                if ((*iterator)->nodesSufficient(rn))
                {
                    // registers eef to result:
                    EndEffectorPtr eef = (*iterator)->clone(result);
                }

                ++iterator;
            }
        }

        std::vector<RobotNodePtr> allNodes;
        startJoint->collectAllRobotNodes(allNodes);

        for (auto& allNode : allNodes)
        {
            RobotNodePtr roN = result->getRobotNode(allNode->getName());

            if (roN)
            {
                roN->setJointValueNoUpdate(allNode->getJointValue());
            }
        }

        result->propagatingJointValuesEnabled = propagatingJointValuesEnabled;
        result->applyJointValues();
        result->basePath = basePath;
        return result;
    }

    RobotPtr Robot::cloneScaling() {
        return clone(getName(), CollisionCheckerPtr(), -1.0f);
    }

    Eigen::Matrix4f Robot::getGlobalPoseForRobotNode(
        const RobotNodePtr& node, const Eigen::Matrix4f& globalPoseNode) const
    {
        THROW_VR_EXCEPTION_IF(!node, "NULL node");

        if (math::Helpers::Orientation(globalPoseNode).isIdentity())
        {
            // set position to avoid accumulating rounding errors when using rotation
            return getGlobalPositionForRobotNode(node, math::Helpers::Position(globalPoseNode));
        }

        // get transformation from current to wanted tcp pose
        Eigen::Matrix4f tf = globalPoseNode * math::Helpers::InvertedPose(node->getGlobalPose());

        // apply transformation to current global pose of robot
        tf = tf * getGlobalPose();

        // re-orthogonolize to keep it a valid transformation matrix
        tf = math::Helpers::Orthogonalize(tf);

        // return tf
        return tf;
    }

    void Robot::setGlobalPoseForRobotNode(
        const RobotNodePtr& node, const Eigen::Matrix4f& globalPoseNode)
    {
        THROW_VR_EXCEPTION_IF(!node, "NULL node");

        setGlobalPose(getGlobalPoseForRobotNode(node, globalPoseNode));
    }

    Eigen::Matrix4f Robot::getGlobalPositionForRobotNode(
        const RobotNodePtr& node, const Eigen::Vector3f& globalPositionNode) const
    {
        // get translation from current to wanted tcp pose
        const Eigen::Vector3f translation = globalPositionNode - node->getGlobalPosition();

        // apply translation to current global position of robot
        const Eigen::Vector3f robotPosition = getGlobalPosition() + translation;

        return math::Helpers::Pose(robotPosition);
    }

    void Robot::setGlobalPositionForRobotNode(
        const RobotNodePtr& node, const Eigen::Vector3f& globalPositionNode)
    {
        setGlobalPose(getGlobalPositionForRobotNode(node, globalPositionNode));
    }


    VirtualRobot::RobotPtr Robot::clone(const std::string& name,
                                        CollisionCheckerPtr collisionChecker,
                                        float scaling,
                                        bool preventCloningMeshesIfScalingIs1)
    {
        VirtualRobot::RobotPtr result = extractSubPart(this->getRootNode(),
                                        this->getType(),
                                        name, true, true,
                                        collisionChecker,
                                        scaling,
                                        preventCloningMeshesIfScalingIs1);
        result->setGlobalPose(getGlobalPose());
        result->filename = filename;
        result->type = type;
        //result->radianToMMfactor = radianToMMfactor;

        if(getHumanMapping().has_value())
        {
            result->registerHumanMapping(getHumanMapping().value());
        }

        return result;
    }

    RobotPtr Robot::clone(CollisionCheckerPtr collisionChecker,
                          float scaling,
                          bool preventCloningMeshesIfScalingIs1)
    {
        return clone(getName(), collisionChecker, scaling, preventCloningMeshesIfScalingIs1);
    }

    void Robot::createVisualizationFromCollisionModels()
    {
        std::vector<RobotNodePtr> robotNodes = this->getRobotNodes();
        std::vector<RobotNodePtr>::const_iterator iterator = robotNodes.begin();

        while (robotNodes.end() != iterator)
        {
            RobotNodePtr rn = *iterator;

            if (rn->getVisualization(SceneObject::Full) && rn->getVisualization(SceneObject::Collision))
            {
                VisualizationNodePtr v = rn->getVisualization(SceneObject::Collision)->clone();
                rn->setVisualization(v);
            }

            iterator++;
        }
    }

    VirtualRobot::RobotConfigPtr Robot::getConfig() const
    {
        RobotConfigPtr r(new RobotConfig(shared_from_this(), getName()));

        for (const auto& rn : this->getRobotNodes())
        {
            if (rn->isTranslationalJoint() || rn->isRotationalJoint())
            {
                r->setConfig(rn, rn->getJointValue());
            }
        }

        return r;
    }

    bool Robot::setConfig(RobotConfigPtr c)
    {
        if (!c)
        {
            return false;
        }

        setJointValues(c);
        return true;
    }

    void Robot::setFilename(const std::string& filename)
    {
        this->filename = filename;
    }

    std::string Robot::getFilename() const
    {
        return filename;
    }

    ReadLockPtr Robot::getReadLock()
    {
        if (!use_mutex)
        {
            return ReadLockPtr();
        }
        else
        {
            return ReadLockPtr(new ReadLock(mutex, use_mutex));
        }
    }

    WriteLockPtr Robot::getWriteLock()
    {
        if (!use_mutex)
        {
            return WriteLockPtr();
        }
        else
        {
            return WriteLockPtr(new WriteLock(mutex, use_mutex));
        }
    }

    void Robot::setJointValue(const std::string& nodeName, float jointValue)
    {
        RobotNodePtr rn = getRobotNode(nodeName);
        setJointValue(rn, jointValue);
    }

    void Robot::setJointValue(RobotNodePtr rn, float jointValue)
    {
        VR_ASSERT(rn);
        rn->setJointValue(jointValue);
    }

    void Robot::setJointValues(const std::map< std::string, float >& jointValues)
    {
        WriteLock lock(mutex, use_mutex);

        std::map< std::string, float >::const_iterator it = jointValues.begin();

        while (it != jointValues.end())
        {
            RobotNodePtr rn = getRobotNode(it->first);
            if (!rn)
            {
                VR_WARNING << "No joint with name " << it->first << std::endl;
            }
            else
            {
                rn->setJointValueNoUpdate(it->second);
            }
            it++;
        }

        applyJointValuesNoLock();
    }

    std::map<std::string, float> Robot::getJointValues() const
    {
        ReadLock lock(mutex, use_mutex);
        std::map<std::string, float> values;
        for (const auto& node : getRobotNodes())
        {
            if (node->getJointLimitLo() < node->getJointLimitHi())
            {
                values[node->getName()] = node->getJointValue();
            }
        }
        return values;
    }

    void Robot::setJointValues(const std::map<RobotNodePtr, float>& jointValues)
    {
        WriteLock lock(mutex, use_mutex);

        std::map< RobotNodePtr, float >::const_iterator it = jointValues.begin();

        while (it != jointValues.end())
        {
            RobotNodePtr rn = it->first;
            rn->setJointValueNoUpdate(it->second);
            it++;
        }

        applyJointValuesNoLock();
    }

    void Robot::setJointValues(RobotNodeSetPtr rns, const std::vector<float>& jointValues)
    {
        VR_ASSERT(rns);
        rns->setJointValues(jointValues);
    }

    void Robot::setJointValues(RobotNodeSetPtr rns, const Eigen::VectorXf& jointValues)
    {
        VR_ASSERT(rns);
        rns->setJointValues(jointValues);
    }

    void Robot::setJointValues(RobotConfigPtr config)
    {
        VR_ASSERT(config);
        config->setJointValues(shared_from_this());
    }

    void Robot::setJointValues(TrajectoryPtr trajectory, float t)
    {
        VR_ASSERT(trajectory);
        Eigen::VectorXf c;
        trajectory->interpolate(t, c);
        setJointValues(trajectory->getRobotNodeSet(), c);
    }

    void Robot::setJointValues(RobotNodeSetPtr rns, RobotConfigPtr config)
    {
        VR_ASSERT(rns);
        VR_ASSERT(config);
        rns->setJointValues(config);
    }

    void Robot::setJointValues(const std::vector<RobotNodePtr> rn, const std::vector<float>& jointValues)
    {
        VR_ASSERT(rn.size() == jointValues.size());
        WriteLock lock(mutex, use_mutex);

        for (size_t i = 0; i < rn.size(); i++)
        {
            rn[i]->setJointValueNoUpdate(jointValues[i]);
        }

        applyJointValuesNoLock();
    }

    VirtualRobot::BoundingBox Robot::getBoundingBox(bool collisionModel) const
    {
        VirtualRobot::BoundingBox bbox;
        std::vector<RobotNodePtr> rn = getRobotNodes();

        for (auto& i : rn)
        {
            if (collisionModel && i->getCollisionModel())
            {
                bbox.addPoints(i->getCollisionModel()->getBoundingBox());
            }
            else if (!collisionModel && i->getVisualization())
            {
                bbox.addPoints(i->getVisualization()->getBoundingBox());
            }
        }

        return bbox;
    }

    SensorPtr Robot::getSensor(const std::string& name) const
    {
        std::vector<RobotNodePtr> rn = getRobotNodes();

        for (auto& i : rn)
        {
            std::vector<SensorPtr> sensors = i->getSensors();

            for (auto& sensor : sensors)
            {
                if (sensor->getName() == name)
                {
                    return sensor;
                }
            }
        }

        return SensorPtr();
    }

    std::vector<SensorPtr> Robot::getSensors() const
    {
        std::vector<SensorPtr> result;
        std::vector<RobotNodePtr> rn = getRobotNodes();

        for (auto& i : rn)
        {
            std::vector<SensorPtr> s = i->getSensors();

            if (!s.empty())
            {
                result.insert(result.end(), s.begin(), s.end());
            }
        }

        return result;
    }

    std::string Robot::toXML(const std::string& basePath,  const std::string& modelPath, bool storeEEF, bool storeRNS, bool storeSensors, bool storeModelFiles) const
    {
        std::stringstream ss;
        ss << "<?xml version='1.0' encoding='UTF-8'?>" << endl << std::endl;
        //ss << "<Robot Type='" << this->type << "' RootNode='" << this->getRootNode()->getName() << "' RadianToMMfactor='" << this->radianToMMfactor << "'>" << endl << std::endl;
        ss << "<Robot Type='" << this->type << "' RootNode='" << this->getRootNode()->getName() << "'>" << endl << std::endl;
        std::vector<RobotNodePtr> nodes = getRobotNodes();

        for (auto& node : nodes)
        {
            ss << node->toXML(basePath, modelPath, storeSensors, storeModelFiles) << std::endl;
        }

        ss << std::endl;

        if (storeRNS)
        {
            std::vector<RobotNodeSetPtr> rns;
            this->getRobotNodeSets(rns);

            for (auto& rn : rns)
            {
                ss << rn->toXML(1) << std::endl;
            }

            if (rns.size() > 0)
            {
                ss << std::endl;
            }
        }

        if (storeEEF)
        {
            std::vector<EndEffectorPtr> eefs = this->getEndEffectors();

            for (auto& eef : eefs)
            {
                ss << eef->toXML(1) << std::endl;
            }

            if (eefs.size() > 0)
            {
                ss << std::endl;
            }
        }

        ss << "</Robot>" << std::endl;

        return ss.str();
    }

    float Robot::getScaling() const
    {
        return scaling;
    }

    void Robot::setScaling(float scaling)
    {
        this->scaling = scaling;
    }

    void Robot::inflateCollisionModel(float inflationInMM)
    {
        for (auto& node : getRobotNodes())
        {
            if (node->getCollisionModel())
            {
                node->getCollisionModel()->inflateModel(inflationInMM);
            }
        }
    }

    bool Robot::getPropagatingJointValuesEnabled() const
    {
        return propagatingJointValuesEnabled;
    }

    void Robot::setPropagatingJointValuesEnabled(bool enabled)
    {
        propagatingJointValuesEnabled = enabled;
    }

    void Robot::validateNodeMapping(const NodeMapping& nodeMapping) const
    {
        for(const auto& nodeMap : nodeMapping)
        {
            THROW_VR_EXCEPTION_IF(not hasRobotNode(nodeMap.first), "Node '" + nodeMap.first + " not found in the node set!");
            THROW_VR_EXCEPTION_IF(not hasRobotNode(nodeMap.second.node), "Node '" + nodeMap.second.node + " not found in the node set!");
        }
    }

    void Robot::registerNodeMapping(const NodeMapping& nodeMapping){

        validateNodeMapping(nodeMapping);

        this->nodeMapping = nodeMapping;
    }

    void Robot::registerHumanMapping(const HumanMapping& humanMapping)
    {
        this->humanMapping = humanMapping;
    }


    const NodeMapping& Robot::getNodeMapping() const
    {
        return nodeMapping;
    }

    const std::optional<HumanMapping>& Robot::getHumanMapping() const
    {
        return humanMapping;
    }

    

} // namespace VirtualRobot
