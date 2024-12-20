
#include "Scene.h"

#include "Logging.h"
#include "ManipulationObject.h"
#include "MathTools.h"
#include "Nodes/RobotNode.h"
#include "SceneObjectSet.h"
#include "Trajectory.h"
#include "VirtualRobotException.h"
#include "Visualization/CoinVisualization/CoinVisualization.h"
#include "XML/BaseIO.h"

namespace VirtualRobot
{
    using std::cout;
    using std::endl;

    Scene::Scene(const std::string& name) : name(name)
    {
    }

    Scene::~Scene()
    {
        robots.clear();
        robotConfigs.clear();
        obstacles.clear();
        manipulationObjects.clear();
        sceneObjectSets.clear();
        trajectories.clear();
    }

    void
    Scene::registerRobot(RobotPtr robot)
    {
        THROW_VR_EXCEPTION_IF(!robot, "NULL data");

        if (hasRobot(robot))
        {
            return;
        }

        robots.push_back(robot);
    }

    void
    Scene::deRegisterRobot(RobotPtr robot)
    {
        THROW_VR_EXCEPTION_IF(!robot, "NULL data");

        if (!hasRobot(robot))
        {
            return;
        }

        for (std::vector<RobotPtr>::iterator i = robots.begin(); i != robots.end(); i++)
        {
            if (*i == robot)
            {
                robots.erase(i);
                break;
            }
        }
    }

    void
    Scene::deRegisterRobot(const std::string& name)
    {
        if (!hasRobot(name))
        {
            return;
        }

        for (std::vector<RobotPtr>::iterator i = robots.begin(); i != robots.end(); i++)
        {
            if ((*i)->getName() == name)
            {
                robots.erase(i);
                break;
            }
        }
    }

    void
    Scene::registerObstacle(ObstaclePtr obstacle)
    {
        THROW_VR_EXCEPTION_IF(!obstacle, "NULL data");

        if (hasObstacle(obstacle))
        {
            return;
        }

        obstacles.push_back(obstacle);
    }

    void
    Scene::deRegisterObstacle(ObstaclePtr obstacle)
    {
        THROW_VR_EXCEPTION_IF(!obstacle, "NULL data");

        if (!hasObstacle(obstacle))
        {
            return;
        }

        for (std::vector<ObstaclePtr>::iterator i = obstacles.begin(); i != obstacles.end(); i++)
        {
            if ((*i) == obstacle)
            {
                obstacles.erase(i);
                break;
            }
        }
    }

    void
    Scene::deRegisterObstacle(const std::string& name)
    {
        if (!hasObstacle(name))
        {
            return;
        }

        for (std::vector<ObstaclePtr>::iterator i = obstacles.begin(); i != obstacles.end(); i++)
        {
            if ((*i)->getName() == name)
            {
                obstacles.erase(i);
                break;
            }
        }
    }

    void
    Scene::registerTrajectory(TrajectoryPtr t)
    {
        THROW_VR_EXCEPTION_IF(!t, "NULL data");

        if (hasTrajectory(t))
        {
            return;
        }

        trajectories.push_back(t);
    }

    void
    Scene::deRegisterTrajectory(TrajectoryPtr t)
    {
        THROW_VR_EXCEPTION_IF(!t, "NULL data");

        if (!hasTrajectory(t))
        {
            return;
        }

        for (std::vector<TrajectoryPtr>::iterator i = trajectories.begin(); i != trajectories.end();
             i++)
        {
            if ((*i) == t)
            {
                trajectories.erase(i);
                break;
            }
        }
    }

    void
    Scene::deRegisterTrajectory(const std::string& name)
    {
        if (!hasTrajectory(name))
        {
            return;
        }

        for (std::vector<TrajectoryPtr>::iterator i = trajectories.begin(); i != trajectories.end();
             i++)
        {
            if ((*i)->getName() == name)
            {
                trajectories.erase(i);
                break;
            }
        }
    }

    void
    Scene::registerManipulationObject(ManipulationObjectPtr manipulationObject)
    {
        THROW_VR_EXCEPTION_IF(!manipulationObject, "NULL data");

        if (hasManipulationObject(manipulationObject))
        {
            return;
        }

        manipulationObjects.push_back(manipulationObject);
    }

    void
    Scene::deRegisterManipulationObject(ManipulationObjectPtr manipulationObject)
    {
        THROW_VR_EXCEPTION_IF(!manipulationObject, "NULL data");

        if (!hasManipulationObject(manipulationObject))
        {
            return;
        }

        for (std::vector<ManipulationObjectPtr>::iterator i = manipulationObjects.begin();
             i != manipulationObjects.end();
             i++)
        {
            if ((*i) == manipulationObject)
            {
                manipulationObjects.erase(i);
                break;
            }
        }
    }

    void
    Scene::deRegisterManipulationObject(const std::string& name)
    {
        if (!hasManipulationObject(name))
        {
            return;
        }

        for (std::vector<ManipulationObjectPtr>::iterator i = manipulationObjects.begin();
             i != manipulationObjects.end();
             i++)
        {
            if ((*i)->getName() == name)
            {
                manipulationObjects.erase(i);
                break;
            }
        }
    }

    bool
    Scene::hasRobot(RobotPtr robot) const
    {
        THROW_VR_EXCEPTION_IF(!robot, "NULL data");

        for (const auto& i : robots)
        {
            if (i == robot)
            {
                return true;
            }
        }

        return false;
    }

    bool
    Scene::hasRobot(const std::string& name) const
    {
        for (const auto& robot : robots)
        {
            if (robot->getName() == name)
            {
                return true;
            }
        }

        return false;
    }

    bool
    Scene::hasObstacle(ObstaclePtr obstacle) const
    {
        THROW_VR_EXCEPTION_IF(!obstacle, "NULL data");

        for (const auto& i : obstacles)
        {
            if (i == obstacle)
            {
                return true;
            }
        }

        return false;
    }

    bool
    Scene::hasObstacle(const std::string& name) const
    {
        for (const auto& obstacle : obstacles)
        {
            if (obstacle->getName() == name)
            {
                return true;
            }
        }

        return false;
    }

    bool
    Scene::hasTrajectory(TrajectoryPtr t) const
    {
        THROW_VR_EXCEPTION_IF(!t, "NULL data");

        for (const auto& trajectorie : trajectories)
        {
            if (trajectorie == t)
            {
                return true;
            }
        }

        return false;
    }

    bool
    Scene::hasTrajectory(const std::string& name) const
    {
        for (const auto& trajectorie : trajectories)
        {
            if (trajectorie->getName() == name)
            {
                return true;
            }
        }

        return false;
    }

    bool
    Scene::hasManipulationObject(ManipulationObjectPtr manipulationObject) const
    {
        THROW_VR_EXCEPTION_IF(!manipulationObject, "NULL data");

        for (const auto& i : manipulationObjects)
        {
            if (i == manipulationObject)
            {
                return true;
            }
        }

        return false;
    }

    bool
    Scene::hasManipulationObject(const std::string& name) const
    {
        for (const auto& manipulationObject : manipulationObjects)
        {
            if (manipulationObject->getName() == name)
            {
                return true;
            }
        }

        return false;
    }

    VirtualRobot::RobotPtr
    Scene::getRobot(const std::string& name)
    {
        for (std::vector<RobotPtr>::const_iterator i = robots.begin(); i != robots.end(); i++)
        {
            if ((*i)->getName() == name)
            {
                return *i;
            }
        }

        VR_WARNING << "No robot with name " << name << " registered in scene " << this->name
                   << std::endl;
        return RobotPtr();
    }

    VirtualRobot::ObstaclePtr
    Scene::getObstacle(const std::string& name)
    {
        for (std::vector<ObstaclePtr>::const_iterator i = obstacles.begin(); i != obstacles.end();
             i++)
        {
            if ((*i)->getName() == name)
            {
                return *i;
            }
        }

        VR_WARNING << "No obstacle with name " << name << " registered in scene " << this->name
                   << std::endl;
        return ObstaclePtr();
    }

    VirtualRobot::TrajectoryPtr
    Scene::getTrajectory(const std::string& name)
    {
        for (std::vector<TrajectoryPtr>::const_iterator i = trajectories.begin();
             i != trajectories.end();
             i++)
        {
            if ((*i)->getName() == name)
            {
                return *i;
            }
        }

        VR_WARNING << "No Trajectory with name " << name << " registered in scene " << this->name
                   << std::endl;
        return TrajectoryPtr();
    }

    VirtualRobot::ManipulationObjectPtr
    Scene::getManipulationObject(const std::string& name)
    {
        for (std::vector<ManipulationObjectPtr>::const_iterator i = manipulationObjects.begin();
             i != manipulationObjects.end();
             i++)
        {
            if ((*i)->getName() == name)
            {
                return *i;
            }
        }

        VR_WARNING << "No ManipulationObject with name " << name << " registered in scene "
                   << this->name << std::endl;
        return ManipulationObjectPtr();
    }

    void
    Scene::registerRobotConfig(RobotPtr robot, RobotConfigPtr config)
    {
        THROW_VR_EXCEPTION_IF(!robot, "NULL robot data");
        THROW_VR_EXCEPTION_IF(!config, "NULL config data");

        if (hasRobotConfig(robot, config))
        {
            return;
        }

        robotConfigs[robot].push_back(config);
    }

    void
    Scene::registerRobotConfig(RobotPtr robot, std::vector<RobotConfigPtr> configs)
    {
        for (const auto& config : configs)
        {
            registerRobotConfig(robot, config);
        }
    }

    void
    Scene::deRegisterRobotConfig(RobotPtr robot, RobotConfigPtr config)
    {
        THROW_VR_EXCEPTION_IF(!robot, "NULL data");
        THROW_VR_EXCEPTION_IF(!config, "NULL data");

        if (!hasRobotConfig(robot, config))
        {
            return;
        }

        std::vector<RobotConfigPtr> configs = robotConfigs[robot];

        for (std::vector<RobotConfigPtr>::iterator i = configs.begin(); i != configs.end(); i++)
        {
            if ((*i) == config)
            {
                configs.erase(i);
                robotConfigs[robot] = configs;
                break;
            }
        }
    }

    void
    Scene::deRegisterRobotConfig(RobotPtr robot, const std::string& name)
    {
        THROW_VR_EXCEPTION_IF(!robot, "NULL data");

        if (!hasRobotConfig(robot, name))
        {
            return;
        }

        std::vector<RobotConfigPtr> configs = robotConfigs[robot];

        for (std::vector<RobotConfigPtr>::iterator i = configs.begin(); i != configs.end(); i++)
        {
            if ((*i)->getName() == name)
            {
                configs.erase(i);
                robotConfigs[robot] = configs;
                break;
            }
        }
    }

    bool
    Scene::hasRobotConfig(RobotPtr robot, RobotConfigPtr config)
    {
        THROW_VR_EXCEPTION_IF(!robot, "NULL data");
        THROW_VR_EXCEPTION_IF(!config, "NULL data");
        std::vector<RobotConfigPtr> configs = robotConfigs[robot];

        for (auto& i : configs)
        {
            if (i == config)
            {
                return true;
            }
        }

        return false;
    }

    bool
    Scene::hasRobotConfig(RobotPtr robot, const std::string& name)
    {
        THROW_VR_EXCEPTION_IF(!robot, "NULL data");
        std::vector<RobotConfigPtr> configs = robotConfigs[robot];

        for (auto& config : configs)
        {
            if (config->getName() == name)
            {
                return true;
            }
        }

        return false;
    }

    VirtualRobot::RobotConfigPtr
    Scene::getRobotConfig(const std::string& robotName, const std::string& name)
    {
        for (auto& robotConfig : robotConfigs)
        {
            if (robotConfig.first->getName() == robotName)
            {
                return getRobotConfig(robotConfig.first, name);
            }
        }

        VR_WARNING << "No robot with name " << robotName << " registered in scene " << this->name
                   << std::endl;
        return RobotConfigPtr();
    }

    VirtualRobot::RobotConfigPtr
    Scene::getRobotConfig(RobotPtr robot, const std::string& name)
    {
        THROW_VR_EXCEPTION_IF(!robot, "NULL data");
        std::vector<RobotConfigPtr> configs = robotConfigs[robot];

        for (auto& config : configs)
        {
            if (config->getName() == name)
            {
                return config;
            }
        }

        VR_WARNING << "No robotConfig with name " << name << " registered for robot "
                   << robot->getName() << " in scene " << this->name << std::endl;
        return RobotConfigPtr();
    }

    std::string
    Scene::getName() const
    {
        return name;
    }

    std::vector<RobotPtr>
    Scene::getRobots()
    {
        return robots;
    }

    std::vector<ObstaclePtr>
    Scene::getObstacles()
    {
        return obstacles;
    }

    std::vector<TrajectoryPtr>
    Scene::getTrajectories()
    {
        return trajectories;
    }

    std::vector<TrajectoryPtr>
    Scene::getTrajectories(const std::string& robotName)
    {
        std::vector<TrajectoryPtr> res;

        for (auto& trajectorie : trajectories)
        {
            if (trajectorie->getRobotName() == robotName)
            {
                res.push_back(trajectorie);
            }
        }

        return res;
    }

    std::vector<ManipulationObjectPtr>
    Scene::getManipulationObjects()
    {
        return manipulationObjects;
    }

    std::vector<RobotConfigPtr>
    Scene::getRobotConfigs(RobotPtr robot)
    {
        if (!robot || !hasRobot(robot->getName()))
        {
            return std::vector<RobotConfigPtr>();
        }

        if (robotConfigs.find(robot) == robotConfigs.end())
        {
            return std::vector<RobotConfigPtr>();
        }

        return robotConfigs[robot];
    }

    VirtualRobot::RobotNodeSetPtr
    Scene::getRobotNodeSet(const std::string& robot, const std::string rns)
    {
        RobotPtr r = getRobot(robot);

        if (!r)
        {
            VR_ERROR << " no robot with name " << robot << std::endl;
            return RobotNodeSetPtr();
        }

        return r->getRobotNodeSet(rns);
    }

    void
    Scene::registerSceneObjectSet(SceneObjectSetPtr sos)
    {
        THROW_VR_EXCEPTION_IF(!sos, "NULL config data");

        if (hasSceneObjectSet(sos))
        {
            return;
        }

        sceneObjectSets.push_back(sos);
    }

    void
    Scene::deRegisterSceneObjectSet(SceneObjectSetPtr sos)
    {
        THROW_VR_EXCEPTION_IF(!sos, "NULL data");

        if (!hasSceneObjectSet(sos))
        {
            return;
        }

        for (std::vector<SceneObjectSetPtr>::iterator i = sceneObjectSets.begin();
             i != sceneObjectSets.end();
             i++)
        {
            if ((*i) == sos)
            {
                sceneObjectSets.erase(i);
                break;
            }
        }
    }

    void
    Scene::deRegisterSceneObjectSet(const std::string& name)
    {
        if (!hasSceneObjectSet(name))
        {
            return;
        }

        for (std::vector<SceneObjectSetPtr>::iterator i = sceneObjectSets.begin();
             i != sceneObjectSets.end();
             i++)
        {
            if ((*i)->getName() == name)
            {
                sceneObjectSets.erase(i);
                break;
            }
        }
    }

    bool
    Scene::hasSceneObjectSet(SceneObjectSetPtr sos) const
    {
        THROW_VR_EXCEPTION_IF(!sos, "NULL data");

        for (const auto& sceneObjectSet : sceneObjectSets)
        {
            if (sceneObjectSet == sos)
            {
                return true;
            }
        }

        return false;
    }

    bool
    Scene::hasSceneObjectSet(const std::string& name) const
    {
        for (const auto& sceneObjectSet : sceneObjectSets)
        {
            if (sceneObjectSet->getName() == name)
            {
                return true;
            }
        }

        return false;
    }

    VirtualRobot::SceneObjectSetPtr
    Scene::getSceneObjectSet(const std::string& name)
    {
        for (std::vector<SceneObjectSetPtr>::const_iterator i = sceneObjectSets.begin();
             i != sceneObjectSets.end();
             i++)
        {
            if ((*i)->getName() == name)
            {
                return (*i);
            }
        }

        return SceneObjectSetPtr();
    }

    std::vector<SceneObjectSetPtr>
    Scene::getSceneObjectSets()
    {
        return sceneObjectSets;
    }

    std::string
    Scene::getXMLString(const std::string& basePath)
    {
        std::stringstream ss;
        ss << "<Scene";

        if (!name.empty())
        {
            ss << " name='" << name << "'";
        }

        ss << ">\n";
        ss << "\n";

        // process robots
        for (auto& robot : robots)
        {
            std::string rob = robot->getName();
            RobotConfigPtr currentConfig = robot->getConfig();
            ss << "\t<Robot name='" << rob << "' initConfig='" << currentConfig->getName()
               << "'>\n";
            std::string robFile = robot->getFilename();

            if (!basePath.empty())
            {
                BaseIO::makeRelativePath(basePath, robFile);
            }

            ss << "\t\t<File>" << robFile << "</File>\n";

            // store global pose (if not identity)
            Eigen::Matrix4f gp = robot->getGlobalPose();

            if (!gp.isIdentity())
            {
                ss << "\t\t<GlobalPose>\n";
                ss << "\t\t\t<Transform>\n";
                ss << MathTools::getTransformXMLString(gp, 4);
                ss << "\t\t\t</Transform>\n";
                ss << "\t\t</GlobalPose>\n";
            }

            // store current config
            ss << currentConfig->toXML(2);

            // store all other configs for robot
            std::vector<RobotConfigPtr> rc = getRobotConfigs(robot);

            for (auto& j : rc)
            {
                ss << j->toXML(2);
            }

            ss << "\t</Robot>\n";
            ss << "\n";
        }

        // process manipulation objects
        for (auto& manipulationObject : manipulationObjects)
        {
            ss << manipulationObject->toXML(basePath, 1, true);
            ss << "\n";
        }

        // process obstacles
        for (auto& obstacle : obstacles)
        {
            ss << obstacle->toXML(basePath, 1);
            ss << "\n";
        }

        // process sceneObjectSets
        for (auto& sceneObjectSet : sceneObjectSets)
        {
            ss << sceneObjectSet->toXML(1);
            ss << "\n";
        }

        // process trajectories
        for (auto& trajectorie : trajectories)
        {
            ss << trajectorie->toXML(1);
            ss << "\n";
        }

        ss << "</Scene>\n";

        return ss.str();
    }

    /**
     * This method collects all visualization nodes and creates a new Visualization
     * subclass which is given by the template parameter T.
     * T must be a subclass of VirtualRobot::Visualization.
     * A compile time error is thrown if a different class type is used as template argument.
     */
    // template <typename T>
    std::shared_ptr<CoinVisualization>
    Scene::getVisualization(SceneObject::VisualizationType visuType,
                            bool addRobots,
                            bool addObstacles,
                            bool addManipulationObjects,
                            bool addTrajectories,
                            bool addSceneObjectSets)
    {
        // static_assert(::std::is_base_of_v<Visualization, T>,
        // "TEMPLATE_PARAMETER_FOR_VirtualRobot_getVisualization_MUST_BT_A_SUBCLASS_OF_VirtualRobot__Visualization");
        std::vector<VisualizationNodePtr> collectedVisualizationNodes;

        if (addRobots)
        {
            std::vector<VirtualRobot::RobotPtr> collectedRobots = getRobots();
            // collect all robotnodes
            std::vector<VirtualRobot::RobotNodePtr> collectedRobotNodes;

            for (size_t i = 0; i < collectedRobots.size(); i++)
            {
                collectedRobots[i]->getRobotNodes(collectedRobotNodes, false);
            }

            for (size_t i = 0; i < collectedRobotNodes.size(); i++)
            {
                collectedVisualizationNodes.push_back(
                    collectedRobotNodes[i]->getVisualization(visuType));
            }
        }

        if (addObstacles)
        {
            std::vector<VirtualRobot::ObstaclePtr> collectedObstacles = getObstacles();

            for (size_t i = 0; i < collectedObstacles.size(); i++)
            {
                collectedVisualizationNodes.push_back(
                    collectedObstacles[i]->getVisualization(visuType));
            }
        }

        if (addManipulationObjects)
        {
            std::vector<VirtualRobot::ManipulationObjectPtr> collectedManipulationObjects =
                getManipulationObjects();

            for (size_t i = 0; i < collectedManipulationObjects.size(); i++)
            {
                collectedVisualizationNodes.push_back(
                    collectedManipulationObjects[i]->getVisualization(visuType));
            }
        }

        if (addTrajectories)
        {
            std::vector<VirtualRobot::TrajectoryPtr> collectedTrajectories = getTrajectories();

            for (size_t i = 0; i < collectedTrajectories.size(); i++)
            {
                collectedVisualizationNodes.push_back(collectedTrajectories[i]->getVisualization(
                    CoinVisualization::getFactoryName()));
            }
        }

        if (addSceneObjectSets)
        {
            std::vector<VirtualRobot::SceneObjectSetPtr> collectedSceneObjectSets =
                getSceneObjectSets();

            for (size_t i = 0; i < collectedSceneObjectSets.size(); i++)
            {
                std::vector<SceneObjectPtr> sos = collectedSceneObjectSets[i]->getSceneObjects();

                for (size_t j = 0; j < sos.size(); j++)
                {
                    collectedVisualizationNodes.push_back(sos[j]->getVisualization(visuType));
                }
            }
        }

        std::shared_ptr<CoinVisualization> visualization(
            new CoinVisualization(collectedVisualizationNodes));
        return visualization;
    }


} // namespace VirtualRobot
