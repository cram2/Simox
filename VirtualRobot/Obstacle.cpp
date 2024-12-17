
#include "Obstacle.h"

#include <iostream>
#include <vector>

#include "CollisionDetection/CollisionChecker.h"
#include "CollisionDetection/CollisionModel.h"
#include "Nodes/RobotNode.h"
#include "VirtualRobotException.h"
#include "Visualization/VisualizationFactory.h"
#include "Visualization/VisualizationNode.h"

namespace VirtualRobot
{
    using std::cout;
    using std::endl;

    // obstacle models start with 20000
    int Obstacle::idCounter = 20000;

    Obstacle::Obstacle(const std::string& name,
                       VisualizationNodePtr visualization,
                       CollisionModelPtr collisionModel,
                       const SceneObject::Physics& p,
                       CollisionCheckerPtr colChecker) :
        GraspableSensorizedObject(name, visualization, collisionModel, p, colChecker)
    {
        if (name == "")
        {
            // my id
            id = idCounter++;

            std::stringstream ss;
            ss << "VirtualRobot Obstacle <" << id << ">";
            this->name = ss.str();
        }
        else
        {
            if (collisionModel)
            {
                id = collisionModel->getId();
            }
            else
            {
                // my id
                id = idCounter++;
            }
        }
    }

    Obstacle::Obstacle(const std::string& name,
                       const TriMeshModelPtr& trimesh,
                       const std::string& filename) :
        Obstacle(TagTrimeshCtor{}, name, std::make_shared<VisualizationNode>(trimesh))
    {
        getVisualization()->setFilename(filename, false);
        getCollisionModel()->getVisualization()->setFilename(filename, false);
    }

    Obstacle::Obstacle(TagTrimeshCtor, const std::string& name, const VisualizationNodePtr& vis) :
        Obstacle(name, vis, std::make_shared<CollisionModel>(vis))
    {
    }

    Obstacle::~Obstacle() = default;

    int
    Obstacle::getID()
    {
        return id;
    }

    VirtualRobot::ObstaclePtr
    Obstacle::createBox(float width,
                        float height,
                        float depth,
                        VisualizationFactory::Color color,
                        std::string visualizationType,
                        CollisionCheckerPtr colChecker)
    {
        ObstaclePtr result;
        VisualizationFactoryPtr visualizationFactory;

        if (visualizationType.empty())
        {
            visualizationFactory = VisualizationFactory::first(nullptr);
        }
        else
        {
            visualizationFactory = VisualizationFactory::fromName(visualizationType, nullptr);
        }

        if (!visualizationFactory)
        {
            VR_ERROR << "Could not create factory for visu type " << visualizationType << std::endl;
            return result;
        }

        /*
        std::vector<Primitive::PrimitivePtr> primitives;
        Primitive::PrimitivePtr p(new Primitive::Box(width,height,depth));
        primitives.push_back(p);

        VisualizationNodePtr visu = visualizationFactory->getVisualizationFromPrimitives(primitives,false,color);
        */
        VisualizationNodePtr visu = visualizationFactory->createBox(width, height, depth, color);

        if (!visu)
        {
            VR_ERROR << "Could not create box visualization with visu type " << visualizationType
                     << std::endl;
            return result;
        }

        //TriMeshModelPtr trimesh = visu->getTriMeshModel();

        int id = idCounter;
        idCounter++;

        std::stringstream ss;
        ss << "Box_" << id;

        std::string name = ss.str();

        CollisionModelPtr colModel(new CollisionModel(visu, name, colChecker, id));
        result.reset(new Obstacle(name, visu, colModel, SceneObject::Physics(), colChecker));
        result->initialize();

        return result;
    }

    VirtualRobot::ObstaclePtr
    Obstacle::createSphere(float radius,
                           VisualizationFactory::Color color,
                           std::string visualizationType,
                           CollisionCheckerPtr colChecker)
    {
        ObstaclePtr result;
        VisualizationFactoryPtr visualizationFactory;

        if (visualizationType.empty())
        {
            visualizationFactory = VisualizationFactory::first(nullptr);
        }
        else
        {
            visualizationFactory = VisualizationFactory::fromName(visualizationType, nullptr);
        }

        if (!visualizationFactory)
        {
            VR_ERROR << "Could not create factory for visu type " << visualizationType << std::endl;
            return result;
        }

        VisualizationNodePtr visu = visualizationFactory->createSphere(radius, color);

        /*std::vector<Primitive::PrimitivePtr> primitives;
        Primitive::PrimitivePtr p(new Primitive::Sphere(radius));
        primitives.push_back(p);
        VisualizationNodePtr visu = visualizationFactory->getVisualizationFromPrimitives(primitives,false,color);
        */
        if (!visu)
        {
            VR_ERROR << "Could not create sphere visualization with visu type " << visualizationType
                     << std::endl;
            return result;
        }

        //TriMeshModelPtr trimesh = visu->getTriMeshModel();

        int id = idCounter;
        idCounter++;

        std::stringstream ss;
        ss << "Sphere_" << id;

        std::string name = ss.str();

        CollisionModelPtr colModel(new CollisionModel(visu, name, colChecker, id));
        result.reset(new Obstacle(name, visu, colModel, SceneObject::Physics(), colChecker));

        result->initialize();

        return result;
    }

    VirtualRobot::ObstaclePtr
    Obstacle::createCylinder(float radius,
                             float height,
                             VisualizationFactory::Color color,
                             std::string visualizationType,
                             CollisionCheckerPtr colChecker)
    {
        ObstaclePtr result;
        VisualizationFactoryPtr visualizationFactory;

        if (visualizationType.empty())
        {
            visualizationFactory = VisualizationFactory::first(nullptr);
        }
        else
        {
            visualizationFactory = VisualizationFactory::fromName(visualizationType, nullptr);
        }

        if (!visualizationFactory)
        {
            VR_ERROR << "Could not create factory for visu type " << visualizationType << std::endl;
            return result;
        }

        VisualizationNodePtr visu = visualizationFactory->createCylinder(radius, height, color);

        /*std::vector<Primitive::PrimitivePtr> primitives;
        Primitive::PrimitivePtr p(new Primitive::Cylinder(radius, height));
        primitives.push_back(p);
        VisualizationNodePtr visu = visualizationFactory->getVisualizationFromPrimitives(primitives,false,color);
        */
        if (!visu)
        {
            VR_ERROR << "Could not create cylinder visualization with visu type "
                     << visualizationType << std::endl;
            return result;
        }

        //TriMeshModelPtr trimesh = visu->getTriMeshModel();

        int id = idCounter;
        idCounter++;

        std::stringstream ss;
        ss << "Cylinder_" << id;

        std::string name = ss.str();

        CollisionModelPtr colModel(new CollisionModel(visu, name, colChecker, id));
        result.reset(new Obstacle(name, visu, colModel, SceneObject::Physics(), colChecker));

        result->initialize();

        return result;
    }

    VirtualRobot::ObstaclePtr
    Obstacle::createFromMesh(TriMeshModelPtr mesh,
                             std::string visualizationType,
                             CollisionCheckerPtr colChecker)
    {
        THROW_VR_EXCEPTION_IF(!mesh, "Null data");

        ObstaclePtr result;
        VisualizationFactoryPtr visualizationFactory;

        if (visualizationType.empty())
        {
            visualizationFactory = VisualizationFactory::first(nullptr);
        }
        else
        {
            visualizationFactory = VisualizationFactory::fromName(visualizationType, nullptr);
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

        CollisionModelPtr colModel(new CollisionModel(visu, name, colChecker, id));
        result.reset(new Obstacle(name, visu, colModel, SceneObject::Physics(), colChecker));

        result->initialize();

        return result;
    }

    void
    Obstacle::print(bool printChildren, bool printDecoration) const
    {
        if (printDecoration)
        {
            std::cout << "**** Obstacle ****" << std::endl;
        }

        SceneObject::print(printChildren, printDecoration);
        std::cout << " * id: " << id << std::endl;

        printGrasps();

        if (printDecoration)
        {
            std::cout << std::endl;
        }
    }

    ObstaclePtr
    Obstacle::clone(const std::string& name, CollisionCheckerPtr colChecker, float scaling) const
    {
        VisualizationNodePtr clonedVisualizationNode;

        if (visualizationModel)
        {
            clonedVisualizationNode = visualizationModel->clone(true, scaling);
        }

        CollisionModelPtr clonedCollisionModel;

        if (collisionModel)
        {
            clonedCollisionModel = collisionModel->clone(colChecker, scaling);
        }

        ObstaclePtr result(
            new Obstacle(name, clonedVisualizationNode, clonedCollisionModel, physics, colChecker));

        result->setGlobalPose(getGlobalPose());
        result->primitiveApproximation = primitiveApproximation;

        appendSensorsTo(result);
        appendGraspSetsTo(result);

        return result;
    }

    std::string
    Obstacle::toXML(const std::string& basePath,
                    int tabs,
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

        ss << pre << "<Obstacle name='" << name << "'>\n";


        ss << getSceneObjectXMLString(basePath, tabs, modelPathRelative);
        ss << getGraspableSensorizedObjectXML(modelPathRelative, storeSensors, tabs + 1);

        ss << pre << "</Obstacle>\n";

        return ss.str();
    }

    std::string
    Obstacle::getFilename() const
    {
        return filename;
    }

    void
    Obstacle::setFilename(const std::string& fname)
    {
        this->filename = fname;
    }

} // namespace VirtualRobot
