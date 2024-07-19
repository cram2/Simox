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

#include "../VirtualRobot.h"
#include "../Units.h"
#include "../RobotConfig.h"
#include "../Nodes/RobotNode.h"
#include "../Primitive.h"

#include <string>
#include <vector>
#include <map>



// using forward declarations here, so that the rapidXML header does not have to be parsed when this file is included
namespace rapidxml
{
    template<class Ch>
    class xml_node;
}

namespace VirtualRobot
{

    /*!
        Several basic XML IO methods.
        \see RobotIO, SceneIO, ObjectIO
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT BaseIO
    {
    public:
        enum RobotDescription
        {
            eFull,              // load complete robot definition
            eCollisionModel,    // skip visualization tags and load only collision model
            eStructure,         // load only the structure of the robot, ignore visualization and collision tags -> faster access when robot is only used for coordinate transformations
            eStructureStore,    // load only the structure of the robot, ignore visualization and collision tags -> faster access when robot is only used for coordinate transformations. Both tags are stored in string and can be retrieved by reloadVisualizationFromXML()
            eFullVisAsCol       // load complete robot definition - use visualization as collision model if col model not available
        };


        static void makeAbsolutePath(const std::string& basePath, std::string& filename);
        static void makeRelativePath(const std::string& basePath, std::string& filename);

        /*!
            Create a file and store XML content.
            \param filename The filename
            \param content The XML content as string. No checks are performed.
            \param overwrite If true, a potentially existing file is silently overwritten.
            \return True on success
        */
        static bool writeXMLFile(const std::string& filename, const std::string& content, bool overwrite = true);


        static bool isTrue(const char* s);
        static float convertToFloat(const char* s);
        static int convertToInt(const char* s);
        static void processNodeList(const rapidxml::xml_node<char>* parentNode, RobotPtr robot, std::vector<RobotNodePtr>& nodeList, bool clearList = true);
        static void processLimitsNode(const rapidxml::xml_node<char>* limitsXMLNode, float& jointLimitLo, float& jointLimitHi);
        static std::string processFileNode(const rapidxml::xml_node<char>* fileNode, const std::string& basePath);
        static void processTransformNode(const rapidxml::xml_node<char>* transformXMLNode, const std::string& nodeName, Eigen::Matrix4f& transform);
        static Units getUnitsAttribute(const rapidxml::xml_node<char>* node, Units::UnitsType u);
        static std::string processNameAttribute(const rapidxml::xml_node<char>* node, bool allowOtherAttributes = false);
        static float processFloatAttribute(const std::string& attributeName, const rapidxml::xml_node<char>* node, bool allowOtherAttributes = false);
        static int processIntAttribute(const std::string& attributeName, const rapidxml::xml_node<char>* node, bool allowOtherAttributes = false);
        static bool processOptionalBoolAttribute(const std::string& attributeName, const rapidxml::xml_node<char>* node, bool defaultValue);
        static float getFloatByAttributeName(const rapidxml::xml_node<char>* xmlNode, const std::string& attributeName);
        static float getOptionalFloatByAttributeName(const rapidxml::xml_node<char>* xmlNode, const std::string& attributeName, float standardValue);

        static bool processGraspConfigurationNode(const rapidxml::xml_node<char>* configXMLNode, std::vector< RobotConfig::Configuration >& storeConfigDefinitions, std::string&  storeConfigName);
        static bool processConfigurationNodeList(const rapidxml::xml_node<char>* configXMLNode, std::vector< std::vector< RobotConfig::Configuration > >& configDefinitions, std::vector< std::string >& configNames, std::vector< std::string >& tcpNames);

        static std::string getLowerCase(const char* c);
        static void getLowerCase(std::string& aString);
        static std::string processStringAttribute(const std::string& attributeName, const rapidxml::xml_node<char>* node, bool allowOtherAttributes = false);

        static VisualizationNodePtr processVisualizationTag(const rapidxml::xml_node<char>* visuXMLNode, const std::string& tagName, const std::string& basePath, bool& useAsColModel);
        static CollisionModelPtr processCollisionTag(const rapidxml::xml_node<char>* colXMLNode, const std::string& tagName, const std::string& basePath);
        static void processPrimitiveModelTag(SceneObject::PrimitiveApproximation& primitiveApproximation, const rapidxml::xml_node<char>* colXMLNode);
        static std::vector<Primitive::PrimitivePtr> processPrimitives(const rapidxml::xml_node<char>* primitivesXMLNode);
        static void processPhysicsTag(const rapidxml::xml_node<char>* physicsXMLNode, const std::string& nodeName, SceneObject::Physics& physics);
        static RobotNodeSetPtr processRobotNodeSet(const rapidxml::xml_node<char>* setXMLNode, RobotPtr robo, const std::string& robotRootNode, int& robotNodeSetCounter);
        static TrajectoryPtr processTrajectory(const rapidxml::xml_node<char>* trajectoryXMLNode, std::vector<RobotPtr>& robots);
        static Eigen::Matrix3f process3x3Matrix(const rapidxml::xml_node<char>* matrixXMLNode);
        static bool processFloatValueTags(const rapidxml::xml_node<char>* XMLNode, int dim, Eigen::VectorXf& stroreResult);
        static bool hasUnitsAttribute(const rapidxml::xml_node<char>* node);
        static std::vector< Units > getUnitsAttributes(const rapidxml::xml_node<char>* node);
        static void getAllAttributes(const rapidxml::xml_node<char>* node, const std::string& attrString, std::vector<std::string>& storeValues);
        static void processDHNode(const rapidxml::xml_node<char>* dhXMLNode, DHParameter& dh);

        static NodeMapping processNodeMapping(const rapidxml::xml_node<char>* XMLNode, RobotPtr robot);
        static HumanMapping processHumanMapping(const rapidxml::xml_node<char>* XMLNode, const RobotPtr& robot);

        static std::tuple<std::string, std::map<std::string, float>> processConfigurationNode(const rapidxml::xml_node<char>* XMLNode);

        static void processAffordances(const rapidxml::xml_node<char>* XMLNode, SceneObject::Affordances& affordances);
        static void processManipulationCapabilities(const rapidxml::xml_node<char>* XMLNode, ManipulationCapabilities& manipulationCapabilities);

        static std::string toXML(const Eigen::Matrix4f& m, std::string ident = "\t");

        static std::vector<VisualizationNodePtr> processVisuFiles(const rapidxml::xml_node<char>* visualizationXMLNode, const std::string& basePath, std::string& fileType);

        static GraspSetPtr processGraspSet(const rapidxml::xml_node<char>* graspSetXMLNode, const std::string& objName);
        static GraspPtr processGrasp(const rapidxml::xml_node<char>* graspXMLNode, const std::string& robotType, const std::string& eef, const std::string& objName);

        static bool processSensor(GraspableSensorizedObjectPtr node, const rapidxml::xml_node<char>* sensorXMLNode, RobotDescription loadMode, const std::string& basePath);
    protected:
        // instantiation not allowed
        BaseIO();
        virtual ~BaseIO();


        static std::mutex mutex;
    };

}
