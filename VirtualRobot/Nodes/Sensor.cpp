
#include "Sensor.h"

#include <iomanip>

#include <eigen3/Eigen/Core>

#include <VirtualRobot/Visualization/VisualizationNode.h>

#include "../VirtualRobot.h"
#include "../VirtualRobotException.h"
#include "../Visualization/TriMeshModel.h"
#include "../XML/BaseIO.h"
#include "GraspableSensorizedObject.h"
#include "Grasping/GraspSet.h"
#include "Logging.h"
#include "Nodes/RobotNode.h"

namespace VirtualRobot
{
    using std::cout;
    using std::endl;

    Sensor::Sensor(GraspableSensorizedObjectWeakPtr parentNode,
                   const std::string& name,
                   VisualizationNodePtr visualization,
                   const Eigen::Matrix4f& rnTrafo) :
        SceneObject(name, visualization)
    {
        this->parentNode = parentNode;
        rnTransformation = rnTrafo;
    }

    Sensor::~Sensor() = default;

    bool
    Sensor::initialize(SceneObjectPtr parent, const std::vector<SceneObjectPtr>& children)
    {
        GraspableSensorizedObjectPtr node = parentNode.lock();
        THROW_VR_EXCEPTION_IF(!node, "Could not init Sensor without parent node");

        if (!node->hasSensor(name))
        {
            node->registerSensor(std::static_pointer_cast<Sensor>(shared_from_this()));
        }

        return SceneObject::initialize(parent, children);
    }

    RobotNodePtr
    Sensor::getRobotNode() const
    {
        return std::dynamic_pointer_cast<RobotNode>(getParentNode());
    }

    GraspableSensorizedObjectPtr
    Sensor::getParentNode() const
    {
        GraspableSensorizedObjectPtr result(parentNode);
        return result;
    }

    void
    Sensor::setGlobalPose(const Eigen::Matrix4f& /*pose*/)
    {
        THROW_VR_EXCEPTION("Use parent node to control the position of a Sensor");
    }

    void
    Sensor::print(bool printChildren, bool printDecoration) const
    {
        if (printDecoration)
        {
            std::cout << "******** Sensor ********" << std::endl;
        }

        std::cout << "* Name: " << name << std::endl;
        std::cout << "* Parent: ";
        SceneObjectPtr p = this->getParent();

        if (p)
        {
            std::cout << p->getName() << std::endl;
        }
        else
        {
            std::cout << " -- " << std::endl;
        }

        std::cout << "* visualization model: " << std::endl;

        if (visualizationModel)
        {
            visualizationModel->print();
        }
        else
        {
            std::cout << "  No visualization model" << std::endl;
        }

        if (initialized)
        {
            std::cout << "* initialized: true" << std::endl;
        }
        else
        {
            std::cout << "* initialized: false" << std::endl;
        }

        {
            // scope1
            std::ostringstream sos;
            sos << std::setiosflags(std::ios::fixed);
            sos << "* Parent node to sensor transformation:" << endl
                << rnTransformation << std::endl;
            sos << "* globalPose:" << endl << getGlobalPose() << std::endl;
            std::cout << sos.str();
        } // scope1

        if (printDecoration)
        {
            std::cout << "******** End Sensor ********" << std::endl;
        }

        if (printChildren)
        {
            std::vector<SceneObjectPtr> children = this->getChildren();

            for (auto& i : children)
            {
                i->print(true, true);
            }
        }
    }

    SensorPtr
    Sensor::clone(GraspableSensorizedObjectPtr newNode, float scaling)
    {
        if (!newNode)
        {
            VR_ERROR << "Attempting to clone Sensor for invalid parent node";
            return SensorPtr();
        }


        VisualizationNodePtr clonedVisualizationNode;

        if (visualizationModel)
        {
            clonedVisualizationNode = visualizationModel->clone(true, scaling);
        }


        SensorPtr result = _clone(newNode, clonedVisualizationNode, scaling);

        if (!result)
        {
            VR_ERROR << "Cloning failed.." << std::endl;
            return result;
        }

        newNode->registerSensor(result);

        result->initialize(newNode);

        return result;
    }

    void
    Sensor::setRobotNodeToSensorTransformation(const Eigen::Matrix4f& t)
    {
        this->rnTransformation = t;
        updatePose();
    }

    void
    Sensor::updatePose(bool updateChildren)
    {
        THROW_VR_EXCEPTION_IF(!initialized, "Not initialized");

        SceneObjectPtr p = getParent();

        if (p)
        {
            this->globalPose = p->getGlobalPose() * rnTransformation;
        }
        else
            this->globalPose = rnTransformation;

        // update collision and visualization model and children
        SceneObject::updatePose(updateChildren);
    }

    void
    Sensor::updatePose(const Eigen::Matrix4f& globalPose, bool updateChildren)
    {
        THROW_VR_EXCEPTION_IF(!initialized, "Not initialized");

        this->globalPose = globalPose * rnTransformation;

        // update collision and visualization model and children
        SceneObject::updatePose(updateChildren);
    }

    std::string
    Sensor::toXML(const std::string& modelPath, int tabs)
    {
        // this will not work, since no type is available, just a reference implementation of a sensor XML tag
        std::stringstream ss;
        std::string t = "\t";
        std::string pre = "";

        for (int i = 0; i < tabs; i++)
        {
            pre += t;
        }

        ss << pre << "<Sensor name='" << name << "'>" << std::endl;
        std::string pre2 = pre + t;
        ss << pre << "<Transform>" << std::endl;
        ss << BaseIO::toXML(rnTransformation, pre2);
        ss << pre << "</Transform>" << std::endl;

        if (visualizationModel && visualizationModel->getTriMeshModel() &&
            visualizationModel->getTriMeshModel()->faces.size() > 0)
        {
            ss << visualizationModel->toXML(modelPath, tabs + 1);
        }

        ss << pre << "</Sensor>" << std::endl;
        return ss.str();
    }

    SceneObjectPtr
    Sensor::clone(const std::string& /*name*/,
                  CollisionCheckerPtr /*colChecker*/,
                  float /*scaling*/) const
    {
        THROW_VR_EXCEPTION("Cloning not allowed this way...");
    }

    SceneObject*
    Sensor::_clone(const std::string& /*name*/,
                   CollisionCheckerPtr /*colChecker*/,
                   float /*scaling*/) const
    {
        THROW_VR_EXCEPTION("Cloning not allowed this way...");
    }
} // namespace VirtualRobot
