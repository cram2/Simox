
#include "ObjectIO.h"
#include "../VirtualRobotException.h"
#include "rapidxml.hpp"


#include "RobotIO.h"
#include "../Grasping/Grasp.h"
#include "../Grasping/GraspSet.h"
#include "../ManipulationObject.h"
#include "../Visualization/TriMeshModel.h"

#include <iostream>

using namespace std;

namespace VirtualRobot
{



    ObjectIO::ObjectIO()
        = default;

    ObjectIO::~ObjectIO()
        = default;

    VirtualRobot::ObstaclePtr ObjectIO::loadObstacle(const std::string& xmlFile)
    {
        // load file
        std::ifstream in(xmlFile.c_str());

        THROW_VR_EXCEPTION_IF(!in.is_open(), "Could not open XML file:" << xmlFile);

        std::filesystem::path filenameBaseComplete(xmlFile);
        std::filesystem::path filenameBasePath = filenameBaseComplete.parent_path();
        std::string basePath = filenameBasePath.string();
        VirtualRobot::ObstaclePtr res = loadObstacle(in, basePath);
        THROW_VR_EXCEPTION_IF(!res, "Error while parsing file " << xmlFile);
        res->setFilename(xmlFile);
        return res;
    }

    VirtualRobot::ObstaclePtr ObjectIO::loadObstacle(const std::ifstream& xmlFile, const std::string& basePath /*= ""*/)
    {
        // load file
        THROW_VR_EXCEPTION_IF(!xmlFile.is_open(), "Could not open XML file");

        std::stringstream buffer;
        buffer << xmlFile.rdbuf();
        std::string objectXML(buffer.str());

        VirtualRobot::ObstaclePtr res = createObstacleFromString(objectXML, basePath);
        THROW_VR_EXCEPTION_IF(!res, "Error while parsing file.");
        return res;
    }


    VirtualRobot::ManipulationObjectPtr ObjectIO::loadManipulationObject(const std::string& xmlFile)
    {
        // load file
        std::ifstream in(xmlFile.c_str());

        THROW_VR_EXCEPTION_IF(!in.is_open(), "Could not open XML file:" << xmlFile);

        std::filesystem::path filenameBaseComplete(xmlFile);
        std::filesystem::path filenameBasePath = filenameBaseComplete.parent_path();
        std::string basePath = filenameBasePath.string();
        VirtualRobot::ManipulationObjectPtr res = loadManipulationObject(in, basePath);
        THROW_VR_EXCEPTION_IF(!res, "Error while parsing file " << xmlFile);
        res->setFilename(xmlFile);
        return res;
    }

    VirtualRobot::ManipulationObjectPtr ObjectIO::loadManipulationObject(const std::ifstream& xmlFile, const std::string& basePath /*= ""*/)
    {
        // load file
        THROW_VR_EXCEPTION_IF(!xmlFile.is_open(), "Could not open XML file");

        std::stringstream buffer;
        buffer << xmlFile.rdbuf();
        std::string objectXML(buffer.str());

        VirtualRobot::ManipulationObjectPtr res = createManipulationObjectFromString(objectXML, basePath);
        res->initialize();
        THROW_VR_EXCEPTION_IF(!res, "Error while parsing file.");
        return res;
    }

    GraspPtr ObjectIO::processGrasp(rapidxml::xml_node<char>* graspXMLNode, const std::string& robotType, const std::string& eef, const std::string& /*objName*/)
    {
        THROW_VR_EXCEPTION_IF(!graspXMLNode, "No <Grasp> tag ?!");
        // get name
        std::string name = processNameAttribute(graspXMLNode, true);
        std::string method = processStringAttribute("creation", graspXMLNode, true);
        float quality = processFloatAttribute(std::string("quality"), graspXMLNode, true);
        std::string preshapeName = processStringAttribute("preshape", graspXMLNode, true);
        Eigen::Matrix4f pose;
        pose.setIdentity();
        std::vector< RobotConfig::Configuration > configDefinitions;
        std::string configName;

        rapidxml::xml_node<>* node = graspXMLNode->first_node();

        while (node)
        {
            std::string nodeName = getLowerCase(node->name());

            if (nodeName == "transform")
            {
                processTransformNode(graspXMLNode, name, pose);

            }
            else if (nodeName == "configuration")
            {
                THROW_VR_EXCEPTION_IF(configDefinitions.size() > 0, "Only one configuration per grasp allowed");
                bool cOK = processConfigurationNode(node, configDefinitions, configName);
                THROW_VR_EXCEPTION_IF(!cOK, "Invalid configuration defined in grasp tag '" << name << "'." << endl);

            }
            else
            {
                THROW_VR_EXCEPTION("XML definition <" << nodeName << "> not supported in Grasp <" << name << ">." << endl);
            }

            node = node->next_sibling();
        }

        GraspPtr grasp(new Grasp(name, robotType, eef, pose, method, quality, preshapeName));

        if (configDefinitions.size() > 0)
        {
            // create & register configs
            std::map< std::string, float > rc;

            for (auto& configDefinition : configDefinitions)
            {
                rc[ configDefinition.name ] = configDefinition.value;
            }

            grasp->setConfiguration(rc);
        }

        return grasp;
    }

    bool ObjectIO::writeSTL(TriMeshModelPtr t, const std::string& filename, const std::string& objectName, float scaling)
    {
        if (!t || t->faces.size() == 0)
        {
            VR_ERROR << "Wrong input" << endl;
            return false;
        }

        ofstream of;
        of.open(filename.c_str());
        if (!of)
        {
            VR_ERROR << "Could not open " << filename << " for writing" << endl;
            return false;
        }

        of << "solid " << objectName << endl;

        // write triangles
        for (size_t i = 0; i < t->faces.size(); i++)
        {
            MathTools::TriangleFace& face = t->faces.at(i);
            auto& p1 = t->vertices.at(face.id1);
            auto& p2 = t->vertices.at(face.id2);
            auto& p3 = t->vertices.at(face.id3);
            auto normal1 = face.idNormal1 < t->normals.size() ? t->normals.at(face.idNormal1) : face.normal;
            auto normal2 = face.idNormal2 < t->normals.size() ? t->normals.at(face.idNormal2) : face.normal;
            auto normal3 = face.idNormal3 < t->normals.size() ? t->normals.at(face.idNormal3) : face.normal;
            Eigen::Vector3f n = normal1 + normal2 + normal3;
            n.normalize();

            //Point p = h->vertex()->point();
            //Point q = h->next()->vertex()->point();
            //Point r = h->next()->next()->vertex()->point();
            // compute normal
            //Vector n = CGAL::cross_product( q-p, r-p);
            //Vector norm = n / std::sqrt( n * n);
            of << "    facet normal " << n(0) << " " << n(1) << " " << n(2) << endl;
            of << "      outer loop " << endl;
            of << "        vertex " << p1(0)*scaling << " " << p1(1)*scaling << " " << p1(2)*scaling <<  endl;
            of << "        vertex " << p2(0)*scaling << " " << p2(1)*scaling << " " << p2(2)*scaling << endl;
            of << "        vertex " << p3(0)*scaling << " " << p3(1)*scaling << " " << p3(2)*scaling << endl;
            of << "      endloop " << endl;
            of << "    endfacet " << endl;
        }

        of << "endsolid " << objectName << endl;
        of.close();
        return true;
    }


    GraspSetPtr ObjectIO::processGraspSet(rapidxml::xml_node<char>* graspSetXMLNode, const std::string& objName)
    {
        THROW_VR_EXCEPTION_IF(!graspSetXMLNode, "No <GraspSet> tag ?!");

        // get name
        std::string gsName = processNameAttribute(graspSetXMLNode, true);
        std::string gsRobotType = processStringAttribute(std::string("robottype"), graspSetXMLNode, true);
        std::string gsEEF = processStringAttribute(std::string("endeffector"), graspSetXMLNode, true);

        if (gsName.empty() || gsRobotType.empty() || gsEEF.empty())
        {
            THROW_VR_EXCEPTION("GraspSet tags must have valid attributes 'Name', 'RobotType' and 'EndEffector'");
        }

        GraspSetPtr result(new GraspSet(gsName, gsRobotType, gsEEF));

        rapidxml::xml_node<>* node = graspSetXMLNode->first_node();

        while (node)
        {
            std::string nodeName = getLowerCase(node->name());

            if (nodeName == "grasp")
            {
                GraspPtr grasp = processGrasp(node, gsRobotType, gsEEF, objName);
                THROW_VR_EXCEPTION_IF(!grasp, "Invalid 'Grasp' tag in '" << objName << "'." << endl);
                result->addGrasp(grasp);
            }
            else
            {
                THROW_VR_EXCEPTION("XML definition <" << nodeName << "> not supported in GraspSet <" << gsName << ">." << endl);
            }

            node = node->next_sibling();
        }

        return result;
    }

    ManipulationObjectPtr ObjectIO::processManipulationObject(rapidxml::xml_node<char>* objectXMLNode, const std::string& basePath)
    {
        THROW_VR_EXCEPTION_IF(!objectXMLNode, "No <ManipulationObject> tag in XML definition");

        bool visuProcessed = false;
        bool colProcessed = false;
        VisualizationNodePtr visualizationNode;
        CollisionModelPtr collisionModel;
        bool useAsColModel = false;
        SceneObject::Physics physics;
        bool physicsDefined = false;
        std::vector<GraspSetPtr> graspSets;
        Eigen::Matrix4f globalPose = Eigen::Matrix4f::Identity();

        // get name
        std::string objName = processNameAttribute(objectXMLNode);

        // first check if there is an xml file to load
        rapidxml::xml_node<>* xmlFileNode = objectXMLNode->first_node("file", 0, false);

        if (xmlFileNode)
        {
            std::string xmlFile = processFileNode(xmlFileNode, basePath);
            ManipulationObjectPtr result = loadManipulationObject(xmlFile);

            if (!result)
            {
                return result;
            }

            if (!objName.empty())
            {
                result->setName(objName);
            }

            // update global pose
            rapidxml::xml_node<>* poseNode = objectXMLNode->first_node("globalpose", 0, false);

            if (poseNode)
            {
                processTransformNode(poseNode, objName, globalPose);
                result->setGlobalPose(globalPose);
            }

            return result;
        }



        THROW_VR_EXCEPTION_IF(objName.empty(), "ManipulationObject definition expects attribute 'name'");

        rapidxml::xml_node<>* node = objectXMLNode->first_node();

        while (node)
        {
            std::string nodeName = getLowerCase(node->name());

            if (nodeName == "visualization")
            {
                THROW_VR_EXCEPTION_IF(visuProcessed, "Two visualization tags defined in ManipulationObject '" << objName << "'." << endl);
                visualizationNode = processVisualizationTag(node, objName, basePath, useAsColModel);
                visuProcessed = true;

                if (useAsColModel)
                {
                    THROW_VR_EXCEPTION_IF(colProcessed, "Two collision tags defined in ManipulationObject '" << objName << "'." << endl);
                    std::string colModelName = objName;
                    colModelName += "_VISU_ColModel";
                    // todo: ID?
                    collisionModel.reset(new CollisionModel(visualizationNode, colModelName, CollisionCheckerPtr()));
                    colProcessed = true;
                }
            }
            else if (nodeName == "collisionmodel")
            {
                THROW_VR_EXCEPTION_IF(colProcessed, "Two collision tags defined in ManipulationObject '" << objName << "'." << endl);
                collisionModel = processCollisionTag(node, objName, basePath);
                colProcessed = true;
            }
            else if (nodeName == "physics")
            {
                THROW_VR_EXCEPTION_IF(physicsDefined, "Two physics tags defined in ManipulationObject '" << objName << "'." << endl);
                processPhysicsTag(node, objName, physics);
                physicsDefined = true;
            }
            else if (nodeName == "graspset")
            {
                GraspSetPtr gs = processGraspSet(node, objName);
                THROW_VR_EXCEPTION_IF(!gs, "Invalid grasp set in '" << objName << "'." << endl);
                graspSets.push_back(gs);

            }
            else if (nodeName == "globalpose")
            {
                processTransformNode(node, objName, globalPose);
            }
            else
            {
                THROW_VR_EXCEPTION("XML definition <" << nodeName << "> not supported in ManipulationObject <" << objName << ">." << endl);
            }

            node = node->next_sibling();
        }



        // build object
        ManipulationObjectPtr object(new ManipulationObject(objName, visualizationNode, collisionModel, physics));

        for (const auto& graspSet : graspSets)
        {
            object->addGraspSet(graspSet);
        }

        object->setGlobalPose(globalPose);

        return object;
    }

    ObstaclePtr ObjectIO::processObstacle(rapidxml::xml_node<char>* objectXMLNode, const std::string& basePath)
    {
        THROW_VR_EXCEPTION_IF(!objectXMLNode, "No <Obstacle> tag in XML definition");

        bool visuProcessed = false;
        bool colProcessed = false;
        VisualizationNodePtr visualizationNode;
        CollisionModelPtr collisionModel;
        bool useAsColModel = false;
        SceneObject::Physics physics;
        bool physicsDefined = false;
        Eigen::Matrix4f globalPose = Eigen::Matrix4f::Identity();

        // get name
        std::string objName = processNameAttribute(objectXMLNode);

        // first check if there is an xml file to load
        rapidxml::xml_node<>* xmlFileNode = objectXMLNode->first_node("file", 0, false);

        if (xmlFileNode)
        {
            std::string xmlFile = processFileNode(xmlFileNode, basePath);
            ObstaclePtr result = loadObstacle(xmlFile);

            if (!result)
            {
                return result;
            }

            if (!objName.empty())
            {
                result->setName(objName);
            }

            // update global pose
            rapidxml::xml_node<>* poseNode = objectXMLNode->first_node("globalpose", 0, false);

            if (poseNode)
            {
                processTransformNode(poseNode, objName, globalPose);
                result->setGlobalPose(globalPose);
            }

            return result;
        }

        THROW_VR_EXCEPTION_IF(objName.empty(), "Obstacle definition expects attribute 'name'");


        rapidxml::xml_node<>* node = objectXMLNode->first_node();

        while (node)
        {
            std::string nodeName = getLowerCase(node->name());

            if (nodeName == "visualization")
            {
                THROW_VR_EXCEPTION_IF(visuProcessed, "Two visualization tags defined in Obstacle '" << objName << "'." << endl);
                visualizationNode = processVisualizationTag(node, objName, basePath, useAsColModel);
                visuProcessed = true;

                if (useAsColModel)
                {
                    THROW_VR_EXCEPTION_IF(colProcessed, "Two collision tags defined in Obstacle '" << objName << "'." << endl);
                    std::string colModelName = objName;
                    colModelName += "_VISU_ColModel";
                    // todo: ID?
                    collisionModel.reset(new CollisionModel(visualizationNode, colModelName, CollisionCheckerPtr()));
                    colProcessed = true;
                }
            }
            else if (nodeName == "collisionmodel")
            {
                THROW_VR_EXCEPTION_IF(colProcessed, "Two collision tags defined in Obstacle '" << objName << "'." << endl);
                collisionModel = processCollisionTag(node, objName, basePath);
                colProcessed = true;
            }
            else if (nodeName == "physics")
            {
                THROW_VR_EXCEPTION_IF(physicsDefined, "Two physics tags defined in Obstacle '" << objName << "'." << endl);
                processPhysicsTag(node, objName, physics);
                physicsDefined = true;
            }
            else if (nodeName == "globalpose")
            {
                processTransformNode(node, objName, globalPose);
            }
            else
            {
                THROW_VR_EXCEPTION("XML definition <" << nodeName << "> not supported in Obstacle <" << objName << ">." << endl);
            }

            node = node->next_sibling();
        }

        // build object
        ObstaclePtr object(new Obstacle(objName, visualizationNode, collisionModel, physics));
        object->setGlobalPose(globalPose);
        return object;
    }



    VirtualRobot::ManipulationObjectPtr ObjectIO::createManipulationObjectFromString(const std::string& xmlString, const std::string& basePath /*= ""*/)
    {
        // copy string content to char array
        char* y = new char[xmlString.size() + 1];
        strncpy(y, xmlString.c_str(), xmlString.size() + 1);

        VirtualRobot::ManipulationObjectPtr obj;

        try
        {
            rapidxml::xml_document<char> doc;    // character type defaults to char
            doc.parse<0>(y);    // 0 means default parse flags
            rapidxml::xml_node<char>* objectXMLNode = doc.first_node("ManipulationObject");

            obj = processManipulationObject(objectXMLNode, basePath);



        }
        catch (rapidxml::parse_error& e)
        {
            delete[] y;
            THROW_VR_EXCEPTION("Could not parse data in xml definition" << endl
                               << "Error message:" << e.what() << endl
                               << "Position: " << endl << e.where<char>() << endl);
            return ManipulationObjectPtr();
        }
        catch (VirtualRobot::VirtualRobotException&)
        {
            // rethrow the current exception
            delete[] y;
            throw;
        }
        catch (std::exception& e)
        {
            delete[] y;
            THROW_VR_EXCEPTION("Error while parsing xml definition" << endl
                               << "Error code:" << e.what() << endl);
            return ManipulationObjectPtr();
        }
        catch (...)
        {
            delete[] y;
            THROW_VR_EXCEPTION("Error while parsing xml definition" << endl);
            return ManipulationObjectPtr();
        }

        delete[] y;
        return obj;
    }


    VirtualRobot::ObstaclePtr ObjectIO::createObstacleFromString(const std::string& xmlString, const std::string& basePath /*= ""*/)
    {
        // copy string content to char array
        char* y = new char[xmlString.size() + 1];
        strncpy(y, xmlString.c_str(), xmlString.size() + 1);

        VirtualRobot::ObstaclePtr obj;

        try
        {
            rapidxml::xml_document<char> doc;    // character type defaults to char
            doc.parse<0>(y);    // 0 means default parse flags
            rapidxml::xml_node<char>* objectXMLNode = doc.first_node("Obstacle");

            obj = processObstacle(objectXMLNode, basePath);



        }
        catch (rapidxml::parse_error& e)
        {
            delete[] y;
            THROW_VR_EXCEPTION("Could not parse data in xml definition" << endl
                               << "Error message:" << e.what() << endl
                               << "Position: " << endl << e.where<char>() << endl);
            return ObstaclePtr();
        }
        catch (VirtualRobot::VirtualRobotException&)
        {
            // rethrow the current exception
            delete[] y;
            throw;
        }
        catch (std::exception& e)
        {
            delete[] y;
            THROW_VR_EXCEPTION("Error while parsing xml definition" << endl
                               << "Error code:" << e.what() << endl);
            return ObstaclePtr();
        }
        catch (...)
        {
            delete[] y;
            THROW_VR_EXCEPTION("Error while parsing xml definition" << endl);
            return ObstaclePtr();
        }

        delete[] y;
        return obj;
    }

    bool ObjectIO::saveManipulationObject(ManipulationObjectPtr object, const std::string& xmlFile)
    {
        THROW_VR_EXCEPTION_IF(!object, "NULL object");

        std::filesystem::path filenameBaseComplete(xmlFile);
        std::filesystem::path filenameBasePath = filenameBaseComplete.parent_path();
        std::string basePath = filenameBasePath.string();

        std::string xmlString = object->toXML(basePath);

        // save file
        return BaseIO::writeXMLFile(xmlFile, xmlString, true);
    }


} // namespace VirtualRobot
