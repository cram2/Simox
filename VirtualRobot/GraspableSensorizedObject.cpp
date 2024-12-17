#include "GraspableSensorizedObject.h"

#include "EndEffector/EndEffector.h"
#include "Grasping/GraspSet.h"
#include "Nodes/Sensor.h"
#include "VirtualRobotException.h"

namespace VirtualRobot
{

    GraspableSensorizedObject::GraspableSensorizedObject(const std::string& name,
                                                         VisualizationNodePtr visualization,
                                                         CollisionModelPtr collisionModel,
                                                         const Physics& p,
                                                         CollisionCheckerPtr colChecker) :
        SceneObject(name, visualization, collisionModel, p, colChecker)
    {
    }

    bool
    GraspableSensorizedObject::initialize(SceneObjectPtr parent,
                                          const std::vector<SceneObjectPtr>& children)
    {
        for (const auto& sensor : sensors)
        {
            sensor->initialize(shared_from_this());
        }

        return SceneObject::initialize(parent, children);
    }

    std::string
    GraspableSensorizedObject::getGraspableSensorizedObjectXML(const std::string& modelPathRelative,
                                                               bool storeSensors,
                                                               int tabs)
    {
        std::stringstream ss;
        for (const auto& graspSet : graspSets)
        {
            ss << graspSet->getXMLString(tabs) << "\n";
        }

        if (storeSensors)
        {
            for (const auto& sensor : sensors)
            {
                ss << sensor->toXML(modelPathRelative, tabs);
            }
        }
        return ss.str();
    }

    void
    GraspableSensorizedObject::addGraspSet(GraspSetPtr graspSet)
    {
        THROW_VR_EXCEPTION_IF(!graspSet, "NULL data");
        THROW_VR_EXCEPTION_IF(hasGraspSet(graspSet), "Grasp set already added");
        // don't be too strict
        //THROW_VR_EXCEPTION_IF(hasGraspSet(graspSet->getRobotType(), graspSet->getEndEffector()), "Only one GraspSet per EEF allowed.");
        this->graspSets.push_back(graspSet);
    }

    void
    GraspableSensorizedObject::includeGraspSet(GraspSetPtr toBeIncludedGraspSet) //maybe delete
    {
        THROW_VR_EXCEPTION_IF(!toBeIncludedGraspSet, "NULL data");
        std::string robotType = toBeIncludedGraspSet->getRobotType();
        std::string eef = toBeIncludedGraspSet->getEndEffector();

        //include new Grasps
        //check if grasp is existing
        int index = -1;
        for (size_t i = 0; i < graspSets.size(); i++)
        {
            if (graspSets.at(i)->getRobotType() == robotType &&
                graspSets.at(i)->getEndEffector() == eef)
            {
                index = i;
            }
        }
        THROW_VR_EXCEPTION_IF(index == -1, "Index wrong defined");
        graspSets.at(index)->includeGraspSet(toBeIncludedGraspSet);
    }

    bool
    GraspableSensorizedObject::hasGraspSet(GraspSetPtr graspSet)
    {
        VR_ASSERT_MESSAGE(graspSet, "NULL data");

        for (const auto& i : graspSets)
            if (*i == *graspSet)
            {
                return true;
            }

        return false;
    }

    bool
    GraspableSensorizedObject::hasGraspSet(const std::string& robotType, const std::string& eef)
    {
        for (auto& graspSet : graspSets)
            if (graspSet->getRobotType() == robotType && graspSet->getEndEffector() == eef)
            {
                return true;
            }

        return false;
    }

    VirtualRobot::GraspSetPtr
    GraspableSensorizedObject::getGraspSet(EndEffectorPtr eef)
    {
        THROW_VR_EXCEPTION_IF(!eef, "NULL data");

        return getGraspSet(eef->getRobotType(), eef->getName());
    }

    VirtualRobot::GraspSetPtr
    GraspableSensorizedObject::getGraspSet(const std::string& robotType, const std::string& eefName)
    {
        for (auto& graspSet : graspSets)
            if (graspSet->getRobotType() == robotType && graspSet->getEndEffector() == eefName)
            {
                return graspSet;
            }

        return GraspSetPtr();
    }

    VirtualRobot::GraspSetPtr
    GraspableSensorizedObject::getGraspSet(const std::string& name)
    {
        for (auto& graspSet : graspSets)
            if (graspSet->getName() == name)
            {
                return graspSet;
            }

        return GraspSetPtr();
    }

    const std::vector<GraspSetPtr>&
    GraspableSensorizedObject::getAllGraspSets()
    {
        return graspSets;
    }

    SensorPtr
    GraspableSensorizedObject::getSensor(const std::string& name) const
    {
        for (const auto& sensor : sensors)
        {
            if (sensor->getName() == name)
            {
                return sensor;
            }
        }

        THROW_VR_EXCEPTION("No sensor with name" << name << " registerd at robot node "
                                                 << getName());
        return SensorPtr();
    }

    bool
    GraspableSensorizedObject::hasSensor(const std::string& name) const
    {
        for (const auto& sensor : sensors)
        {
            if (sensor->getName() == name)
            {
                return true;
            }
        }

        return false;
    }

    bool
    GraspableSensorizedObject::registerSensor(SensorPtr sensor)
    {
        if (!this->hasChild(sensor))
        {
            sensors.push_back(sensor);
            this->attachChild(sensor);
        }

        // if we are already initialized, be sure the sensor is also intialized
        if (initialized)
        {
            sensor->initialize(shared_from_this());
        }

        return true;
    }

    std::vector<SensorPtr>
    GraspableSensorizedObject::getSensors() const
    {
        return sensors;
    }

    void
    GraspableSensorizedObject::appendGraspSetsTo(GraspableSensorizedObjectPtr other) const
    {
        for (auto graspSet : graspSets)
        {
            other->addGraspSet(graspSet->clone());
        }
    }

    void
    GraspableSensorizedObject::appendSensorsTo(GraspableSensorizedObjectPtr other) const
    {
        for (auto sensor : sensors)
        {
            other->registerSensor(sensor->clone(other));
        }
    }

    void
    GraspableSensorizedObject::printGrasps() const
    {
        for (size_t i = 0; i < graspSets.size(); i++)
        {
            std::cout << " * Grasp set " << i << ":" << std::endl;
            graspSets[i]->print();
        }
    }

} // namespace VirtualRobot
