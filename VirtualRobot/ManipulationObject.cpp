
#include "ManipulationObject.h"

#include "CollisionDetection/CollisionModel.h"
#include "GraspableSensorizedObject.h"
#include "Logging.h"
#include "MathTools.h"
#include "VirtualRobotException.h"
#include "Visualization/VisualizationNode.h"
#include "XML/BaseIO.h"

namespace VirtualRobot
{
    using std::cout;
    using std::endl;

    ManipulationObject::ManipulationObject(const std::string& name,
                                           VisualizationNodePtr visualization,
                                           CollisionModelPtr collisionModel,
                                           const SceneObject::Physics& p,
                                           CollisionCheckerPtr colChecker) :
        Obstacle(name, visualization, collisionModel, p, colChecker)
    {
    }

    ManipulationObject::ManipulationObject(const std::string& name,
                                           const TriMeshModelPtr& trimesh,
                                           const std::string& filename) :
        Obstacle(name, trimesh, filename)
    {
    }

    ManipulationObject::ManipulationObject(const TriMeshModelPtr& trimesh) :
        ManipulationObject("", trimesh, "")
    {
    }

    ManipulationObject::~ManipulationObject() = default;

    void
    ManipulationObject::print(bool printChildren, bool printDecoration) const
    {
        if (printDecoration)
        {
            std::cout << "**** Manipulation Object ****" << std::endl;
        }

        Obstacle::print(printChildren, false);

        if (printDecoration)
        {
            std::cout << std::endl;
        }
    }

    std::string
    ManipulationObject::toXML(const std::string& basePath,
                              int tabs,
                              bool storeLinkToFile,
                              const std::string& modelPathRelative,
                              bool storeSensors)
    {
        std::stringstream ss;
        std::string t = "\t";
        std::string pre = "";

        for (int i = 0; i < tabs; i++)
        {
            pre += "\t";
        }

        ss << pre << "<ManipulationObject name='" << name << "'>\n";

        if (storeLinkToFile && !filename.empty())
        {
            std::string relFile = filename;

            if (!basePath.empty())
            {
                BaseIO::makeRelativePath(basePath, relFile);
            }

            ss << pre << t << "<File>" << relFile << "</File>\n";
            Eigen::Matrix4f gp = getGlobalPose();

            if (!gp.isIdentity())
            {
                ss << pre << t << "<GlobalPose>\n";
                ss << pre << t << t << "<Transform>\n";
                ss << MathTools::getTransformXMLString(gp, tabs + 3);
                ss << pre << t << t << "</Transform>\n";
                ss << pre << t << "</GlobalPose>\n";
            }
        }
        else
        {
            ss << getSceneObjectXMLString(basePath, tabs + 1, modelPathRelative);
            ss << getGraspableSensorizedObjectXML(modelPathRelative, storeSensors, tabs + 1);
        }

        ss << pre << "</ManipulationObject>\n";

        return ss.str();
    }

    ManipulationObjectPtr
    ManipulationObject::clone(CollisionCheckerPtr colChecker, bool deepVisuCopy) const
    {
        return clone(getName(), colChecker, deepVisuCopy);
    }

    ManipulationObjectPtr
    ManipulationObject::clone(const std::string& name,
                              CollisionCheckerPtr colChecker,
                              bool deepVisuCopy) const
    {
        VisualizationNodePtr clonedVisualizationNode;

        if (visualizationModel)
        {
            clonedVisualizationNode = visualizationModel->clone(deepVisuCopy);
        }

        CollisionModelPtr clonedCollisionModel;

        if (collisionModel)
        {
            clonedCollisionModel = collisionModel->clone(colChecker, 1.0, deepVisuCopy);
        }

        ManipulationObjectPtr result(new ManipulationObject(
            name, clonedVisualizationNode, clonedCollisionModel, physics, colChecker));

        result->setGlobalPose(getGlobalPose());
        result->primitiveApproximation = primitiveApproximation;

        appendSensorsTo(result);
        appendGraspSetsTo(result);

        return result;
    }

    VirtualRobot::ManipulationObjectPtr
    ManipulationObject::createFromMesh(TriMeshModelPtr mesh,
                                       std::string visualizationType,
                                       CollisionCheckerPtr colChecker)
    {
        THROW_VR_EXCEPTION_IF(!mesh, "Null data");

        ManipulationObjectPtr result;
        VisualizationFactoryPtr visualizationFactory;

        if (visualizationType.empty())
        {
            visualizationFactory = VisualizationFactory::first(NULL);
        }
        else
        {
            visualizationFactory = VisualizationFactory::fromName(visualizationType, NULL);
        }

        if (!visualizationFactory)
        {
            VR_ERROR << "Could not create factory for visu type " << visualizationType << std::endl;
            return result;
        }


        Eigen::Matrix4f gp = Eigen::Matrix4f::Identity();
        VisualizationNodePtr visu = visualizationFactory->createTriMeshModelVisualization(mesh, gp);

        if (!visu)
        {
            VR_ERROR << "Could not create sphere visualization with visu type " << visualizationType
                     << std::endl;
            return result;
        }

        int id = idCounter;
        idCounter++;

        std::stringstream ss;
        ss << "Mesh_" << id;

        std::string name = ss.str();

        CollisionModelPtr colModel(new CollisionModel(visu->clone(), name, colChecker, id));
        result.reset(
            new ManipulationObject(name, visu, colModel, SceneObject::Physics(), colChecker));

        result->initialize();

        return result;
    }


} // namespace VirtualRobot
