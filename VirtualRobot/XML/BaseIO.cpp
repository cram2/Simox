
#include "BaseIO.h"
#include "../Robot.h"
#include "../RobotFactory.h"
#include "../RobotNodeSet.h"
#include "../Trajectory.h"
#include "../RuntimeEnvironment.h"
#include "../VirtualRobotException.h"
#include "../CollisionDetection/CollisionModel.h"
#include "../EndEffector/EndEffector.h"
#include "../EndEffector/EndEffectorActor.h"
#include "../Nodes/RobotNodeFactory.h"
#include "../Nodes/RobotNodeFixedFactory.h"
#include "../Transformation/DHParameter.h"
#include "../Visualization/VisualizationFactory.h"
#include "../Visualization/VisualizationNode.h"
#include "../Grasping/Grasp.h"
#include "../Grasping/ChainedGrasp.h"
#include "../Grasping/GraspSet.h"
#include "../Nodes/SensorFactory.h"
#include <SimoxUtility/filesystem/make_relative.h>
#include <SimoxUtility/algorithm/string/string_conversion.h>
#include "SimoxUtility/algorithm/string/string_tools.h"
#include "VirtualRobot.h"
#include "rapidxml.hpp"

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

namespace VirtualRobot
{
    using std::cout;
    using std::endl;

    std::mutex BaseIO::mutex;

    BaseIO::BaseIO() = default;

    BaseIO::~BaseIO() = default;

    bool BaseIO::isTrue(const char* s)
    {
        std::string e = getLowerCase(s);

        if (e == "true" || e == "1" || e == "yes")
        {
            return true;
        }

        return false;
    }


    /**
     * This method converts \p s into a float value and returns it.
     * If \p s is NULL or not a string representation of a float
     * a VirtualRobot::VirtualRobotException is thrown.
     *
     * \param s the string to convert to float
     *
     * \return the passed in float value
     */
    float BaseIO::convertToFloat(const char* s)
    {
        THROW_VR_EXCEPTION_IF(nullptr == s, "Passing Null string to convertToFloat()");
        std::stringstream floatStream;
        floatStream << std::string(s);
        float result;

        if (!(floatStream >> result))
        {
            THROW_VR_EXCEPTION("The string can not be parsed into a float value");
        }

        return result;
    }

    int BaseIO::convertToInt(const char* s)
    {
        THROW_VR_EXCEPTION_IF(nullptr == s, "Passing Null string to convertToInt()");
        std::stringstream intStream;
        intStream << std::string(s);
        int result;

        if (!(intStream >> result))
        {
            THROW_VR_EXCEPTION("The string can not be parsed into an int value");
        }

        return result;
    }


    /**
     * This method gets the attribute \p attributeName from xml_node \p xmlNode and
     * returns its value as float.
     * If an error occurs (NULL data, missing attribute, conversion failed)
     * a VirtualRobot::VirtualRobotException is thrown.
     */
    float BaseIO::getFloatByAttributeName(const rapidxml::xml_node<char>* xmlNode, const std::string& attributeName)
    {
        THROW_VR_EXCEPTION_IF(!xmlNode, "getFloatByAttributeName got NULL data");
        rapidxml::xml_attribute<>* attr = xmlNode->first_attribute(attributeName.c_str(), 0, false);
        THROW_VR_EXCEPTION_IF(!attr, "The node <" << xmlNode->name() << "> does not contain an attribute named " << attributeName);
        return convertToFloat(attr->value());
    }

    /**
     * This method gets an optional attribute \p attributeName from xml_node \p xmlNode and
     * returns its value as float.
     * When no attribute \p attributeName is present the \p standardValue is returned.
     *
     */
    float BaseIO::getOptionalFloatByAttributeName(const rapidxml::xml_node<char>* xmlNode, const std::string& attributeName, float standardValue)
    {
        THROW_VR_EXCEPTION_IF(!xmlNode, "getFloatByAttributeName got NULL data");
        rapidxml::xml_attribute<>* attr = xmlNode->first_attribute(attributeName.c_str(), 0, false);

        if (!attr)
        {
            return standardValue;
        }

        return convertToFloat(attr->value());
    }

    Eigen::Matrix3f BaseIO::process3x3Matrix(const rapidxml::xml_node<char>* matrixXMLNode)
    {
        Eigen::Matrix3f m;
        m.setIdentity();

        if (!matrixXMLNode)
        {
            VR_ERROR << "NULL matrix transform node?!" << std::endl;
            return m;
        }

        rapidxml::xml_node<>* row1XMLNode = matrixXMLNode->first_node("row1", 0, false);
        rapidxml::xml_node<>* row2XMLNode = matrixXMLNode->first_node("row2", 0, false);
        rapidxml::xml_node<>* row3XMLNode = matrixXMLNode->first_node("row3", 0, false);

        if (row1XMLNode)
        {
            m(0, 0) = getFloatByAttributeName(row1XMLNode, "c1");
            m(0, 1) = getFloatByAttributeName(row1XMLNode, "c2");
            m(0, 2) = getFloatByAttributeName(row1XMLNode, "c3");
        }

        if (row2XMLNode)
        {
            m(1, 0) = getFloatByAttributeName(row2XMLNode, "c1");
            m(1, 1) = getFloatByAttributeName(row2XMLNode, "c2");
            m(1, 2) = getFloatByAttributeName(row2XMLNode, "c3");
        }

        if (row3XMLNode)
        {
            m(2, 0) = getFloatByAttributeName(row3XMLNode, "c1");
            m(2, 1) = getFloatByAttributeName(row3XMLNode, "c2");
            m(2, 2) = getFloatByAttributeName(row3XMLNode, "c3");
        }

        return m;
    }


    /**
     * This method processes \<Transform\> tags.
     * If \p transformXMLNode is NULL (e.g. the tag does not exist) \p transform
     * is set to contain the identity matrix.
     */
    void BaseIO::processTransformNode(const rapidxml::xml_node<char>* transformXMLNode, const std::string& tagName, Eigen::Matrix4f& transform)
    {
        if (!transformXMLNode)
        {
            transform = Eigen::Matrix4f::Identity();
            return;
        }

        const rapidxml::xml_node<>* trXMLNode = nullptr;
        std::string nodeName = getLowerCase(transformXMLNode->name());

        if (nodeName == "transform")
        {
            trXMLNode = transformXMLNode;
        }
        else
        {
            trXMLNode = transformXMLNode->first_node("transform", 0, false);
            THROW_VR_EXCEPTION_IF(!trXMLNode, "transformation node does not specify a <transform> tag")
        }

        //bool rotation = false;
        //bool translation = false;
        transform = Eigen::Matrix4f::Identity();

        rapidxml::xml_node<>* node = trXMLNode->first_node();

        while (node)
        {
            std::string nodeName = getLowerCase(node->name());


            // Homogeneous Matrix 4x4
            //rapidxml::xml_node<> *matrixXMLNode = trXMLNode->first_node("matrix4x4",0,false);
            if (nodeName == "matrix4x4")
            {
                Eigen::Matrix4f localT = Eigen::Matrix4f::Identity();
                rapidxml::xml_node<>* row1XMLNode = node->first_node("row1", 0, false);
                rapidxml::xml_node<>* row2XMLNode = node->first_node("row2", 0, false);
                rapidxml::xml_node<>* row3XMLNode = node->first_node("row3", 0, false);
                rapidxml::xml_node<>* row4XMLNode = node->first_node("row4", 0, false);

                if (row1XMLNode)
                {
                    localT(0, 0) = getFloatByAttributeName(row1XMLNode, "c1");
                    localT(0, 1) = getFloatByAttributeName(row1XMLNode, "c2");
                    localT(0, 2) = getFloatByAttributeName(row1XMLNode, "c3");
                    localT(0, 3) = getFloatByAttributeName(row1XMLNode, "c4");
                }

                if (row2XMLNode)
                {
                    localT(1, 0) = getFloatByAttributeName(row2XMLNode, "c1");
                    localT(1, 1) = getFloatByAttributeName(row2XMLNode, "c2");
                    localT(1, 2) = getFloatByAttributeName(row2XMLNode, "c3");
                    localT(1, 3) = getFloatByAttributeName(row2XMLNode, "c4");
                }

                if (row3XMLNode)
                {
                    localT(2, 0) = getFloatByAttributeName(row3XMLNode, "c1");
                    localT(2, 1) = getFloatByAttributeName(row3XMLNode, "c2");
                    localT(2, 2) = getFloatByAttributeName(row3XMLNode, "c3");
                    localT(2, 3) = getFloatByAttributeName(row3XMLNode, "c4");
                }

                if (row4XMLNode)
                {
                    localT(3, 0) = getFloatByAttributeName(row4XMLNode, "c1");
                    localT(3, 1) = getFloatByAttributeName(row4XMLNode, "c2");
                    localT(3, 2) = getFloatByAttributeName(row4XMLNode, "c3");
                    localT(3, 3) = getFloatByAttributeName(row4XMLNode, "c4");
                }

                if (hasUnitsAttribute(node))
                {
                    Units u = getUnitsAttribute(node, Units::eLength);

                    if (u.isMeter())
                    {
                        localT(0, 3) *= 1000.0f;
                        localT(1, 3) *= 1000.0f;
                        localT(2, 3) *= 1000.0f;
                    }
                }

                transform *= localT;
            }
            else if (nodeName == "matrix3x3")
            {
                // Rotation Matrix 3x3
                //matrixXMLNode = trXMLNode->first_node("matrix3x3",0,false);
                //THROW_VR_EXCEPTION_IF((rotation && matrixXMLNode), "Multiple rotations defined in <Transformation> tag: " << tagName << ". Ignoring matrix3x3 node." << endl);

                Eigen::Matrix4f localT = Eigen::Matrix4f::Identity();
                Eigen::Matrix3f m = process3x3Matrix(node);

                localT.block(0, 0, 3, 3) = m;
                transform *= localT;
            }
            else if (nodeName == "rollpitchyaw" || nodeName == "rpy")
            {
                // ROLL PITCH YAW
                //rapidxml::xml_node<> *rpyXMLNode = trXMLNode->first_node("rollpitchyaw",0,false);
                //THROW_VR_EXCEPTION_IF((rpyXMLNode && rotation), "Multiple rotations defined in <Transformation> tag: " << tagName << "! Ignoring rpy node." << endl);

                float r, p, y;
                r = p = y = 0.0f;
                r = getFloatByAttributeName(node, "roll");
                p = getFloatByAttributeName(node, "pitch");
                y = getFloatByAttributeName(node, "yaw");

                if (!hasUnitsAttribute(node))
                {
                    VR_ERROR << "No units attribute at <" << tagName << ">" << std::endl;
                }

                Units u = getUnitsAttribute(node, Units::eAngle);

                if (u.isDegree())
                {
                    r = r / 180.0f * (float)M_PI;
                    p = p / 180.0f * (float)M_PI;
                    y = y / 180.0f * (float)M_PI;
                }

                Eigen::Matrix4f localT = Eigen::Matrix4f::Identity();
                MathTools::rpy2eigen4f(r, p, y, localT);
                transform *= localT;
            }
            else if (nodeName == "quaternion" || nodeName == "quat")
            {
                // Quaternions
                //rapidxml::xml_node<> *quatXMLNode = trXMLNode->first_node("quaternion",0,false);
                //THROW_VR_EXCEPTION_IF((quatXMLNode && rotation), "Multiple rotations defined in <Transformation> tag: " << tagName << "! Ignoring quaternion node." << endl);

                float x, y, z, w;
                x = y = z = w = 0.0f;
                x = getFloatByAttributeName(node, "x");
                y = getFloatByAttributeName(node, "y");
                z = getFloatByAttributeName(node, "z");
                w = getFloatByAttributeName(node, "w");
                Eigen::Matrix4f r = MathTools::quat2eigen4f(x, y, z, w);
                transform *= r;
            }
            else if (nodeName == "translation" || nodeName == "trans")
            {
                // Translation
                //rapidxml::xml_node<> *translationXMLNode = trXMLNode->first_node("translation",0,false);
                //THROW_VR_EXCEPTION_IF((translationXMLNode && translation), "Multiple translations defined in <Transformation> tag: " << tagName << "! Ignoring translation node." << endl);
                Eigen::Matrix4f localT = Eigen::Matrix4f::Identity();
                localT(0, 3) = getFloatByAttributeName(node, "x");
                localT(1, 3) = getFloatByAttributeName(node, "y");
                localT(2, 3) = getFloatByAttributeName(node, "z");

                if (hasUnitsAttribute(node))
                {
                    Units u = getUnitsAttribute(node, Units::eLength);

                    if (u.isMeter())
                    {
                        localT(0, 3) *= 1000.0f;
                        localT(1, 3) *= 1000.0f;
                        localT(2, 3) *= 1000.0f;
                    }
                }

                transform *= localT;
            }
            else if (nodeName == "dh")
            {
                // DH
                //rapidxml::xml_node<> *dhXMLNode = trXMLNode->first_node("dh",0,false);
                //THROW_VR_EXCEPTION_IF((dhXMLNode && (rotation||translation)), "Multiple rotations/translations defined in <Transformation> tag: " << tagName << "! Ignoring DH node." << endl);
                DHParameter dh;
                processDHNode(node, dh);
                Eigen::Matrix4f localT = dh.transformation();
                transform *= localT;
            }
            else
            {
                VR_ERROR << "Ignoring unknown tag " << nodeName << std::endl;
            }

            node = node->next_sibling();
        }

    }


    void BaseIO::processDHNode(const rapidxml::xml_node<char>* dhXMLNode, DHParameter& dh)
    {
        rapidxml::xml_attribute<>* attr;
        std::vector< Units > unitsAttr = getUnitsAttributes(dhXMLNode);
        Units uAngle("rad");
        Units uLength("mm");

        for (auto& i : unitsAttr)
        {
            if (i.isAngle())
            {
                uAngle = i;
            }

            if (i.isLength())
            {
                uLength = i;
            }
        }

        dh.isSet = true;
        bool isRadian = uAngle.isRadian();

        attr = dhXMLNode->first_attribute("a", 0, false);

        if (attr)
        {
            dh.setAInMM(uLength.toMillimeter(convertToFloat(attr->value())));
        }

        attr = dhXMLNode->first_attribute("d", 0, false);

        if (attr)
        {
            dh.setDInMM(uLength.toMillimeter(convertToFloat(attr->value())));
        }

        attr = dhXMLNode->first_attribute("alpha", 0, false);

        if (attr)
        {
            dh.setAlphaRadian(convertToFloat(attr->value()), isRadian);
        }

        attr = dhXMLNode->first_attribute("theta", 0, false);

        if (attr)
        {
            dh.setThetaRadian(convertToFloat(attr->value()), isRadian);
        }
    }
    
    

    bool BaseIO::hasUnitsAttribute(const rapidxml::xml_node<char>* node)
    {
        rapidxml::xml_attribute<>* attr = node->first_attribute("unit", 0, false);

        if (!attr)
        {
            attr = node->first_attribute("units", 0, false);
        }

        if (!attr)
        {
            attr = node->first_attribute("unitsWeight", 0, false);
        }

        if (!attr)
        {
            attr = node->first_attribute("unitsLength", 0, false);
        }

        if (!attr)
        {
            attr = node->first_attribute("unitsAngle", 0, false);
        }

        if (!attr)
        {
            attr = node->first_attribute("unitsTime", 0, false);
        }

        return (attr != nullptr);
    }

    void BaseIO::getAllAttributes(const rapidxml::xml_node<char>* node, const std::string& attrString, std::vector<std::string>& storeValues)
    {
        rapidxml::xml_attribute<>* attr = node->first_attribute(attrString.c_str(), 0, false);

        while (attr)
        {
            std::string s(attr->value());
            storeValues.push_back(s);

            attr = attr->next_attribute(attrString.c_str(), 0, false);
        }
    }

    std::vector< Units > BaseIO::getUnitsAttributes(const rapidxml::xml_node<char>* node)
    {
        std::vector< Units > result;
        std::vector<std::string> attrStr;
        getAllAttributes(node, "unit", attrStr);
        getAllAttributes(node, "units", attrStr);
        getAllAttributes(node, "unitsWeight", attrStr);
        getAllAttributes(node, "unitsLength", attrStr);
        getAllAttributes(node, "unitsAngle", attrStr);
        getAllAttributes(node, "unitsTime", attrStr);

        for (auto& i : attrStr)
        {
            Units unitsAttribute(getLowerCase(i.c_str()));
            result.push_back(unitsAttribute);
        }

        return result;
    }

    /**
     * This method processes the unit, units, unitsAngle, unitsLength, unitsTime or unitsWeight attribute of xml_node \p node.
     * The first matching unit is returned.
     *
     * \return instance of VirtualRobot::Units
     */
    Units BaseIO::getUnitsAttribute(const rapidxml::xml_node<char>* node, Units::UnitsType u)
    {
        THROW_VR_EXCEPTION_IF(!node, "NULL data for getUnitsAttribute().")
        rapidxml::xml_attribute<>* attr = node->first_attribute("unit", 0, false);

        if (!attr)
        {
            attr = node->first_attribute("units", 0, false);
        }

        if (!attr)
        {
            switch (u)
            {
                case Units::eAngle:
                    attr = node->first_attribute("unitsAngle", 0, false);
                    break;

                case Units::eLength:
                    attr = node->first_attribute("unitsLength", 0, false);
                    break;

                case Units::eWeight:
                    attr = node->first_attribute("unitsWeight", 0, false);
                    break;

                case Units::eTime:
                    attr = node->first_attribute("unitsTime", 0, false);
                    break;

                case Units::eIgnore:
                    attr = node->first_attribute("unitsAngle", 0, false);

                    if (!attr)
                    {
                        attr = node->first_attribute("unitsLength", 0, false);
                    }

                    if (!attr)
                    {
                        attr = node->first_attribute("unitsWeight", 0, false);
                    }

                    if (!attr)
                    {
                        attr = node->first_attribute("unitsTime", 0, false);
                    }

                    break;

                default:
                    break;
            }
        }

        THROW_VR_EXCEPTION_IF(!attr, "Tag <" << node->name() << "> is missing a unit/units attribute.")

        Units unitsAttribute(getLowerCase(attr->value()));

        switch (u)
        {
            case Units::eAngle:
            {
                THROW_VR_EXCEPTION_IF(!unitsAttribute.isAngle(), "Wrong <Units> tag! Expecting angle type." << endl);
            }
            break;

            case Units::eLength:
            {
                THROW_VR_EXCEPTION_IF(!unitsAttribute.isLength(), "Wrong <Units> tag! Expecting length type." << endl);
            }
            break;

            case Units::eWeight:
            {
                THROW_VR_EXCEPTION_IF(!unitsAttribute.isWeight(), "Wrong <Units> tag! Expecting weight type." << endl);
            }
            break;

            case Units::eTime:
            {
                THROW_VR_EXCEPTION_IF(!unitsAttribute.isTime(), "Wrong <Units> tag! Expecting time type." << endl);
            }
            break;

            default:
                break;
        }

        return unitsAttribute;
    }


    /**
     * This method creates a std::string from the parameter \p c and calls
     * VirtualRobot::BaseIO::getLowerCase(std::string) on the created string.
     * Afterwards the transformed string is returned.
     */
    std::string BaseIO::getLowerCase(const char* c)
    {
        THROW_VR_EXCEPTION_IF(nullptr == c, "Passing Null string to getLowerCase()");
        std::string res = c;
        getLowerCase(res);
        return res;
    }

    /**
     * This method applies tolower() to all characters in the string \p aString.
     */
    void BaseIO::getLowerCase(std::string& aString)
    {
        std::transform(aString.begin(), aString.end(), aString.begin(), [](unsigned char c){ return std::tolower(c); });
    }


    /**
     * This method processes the \p parentNode Tag and extracts a list of \<Node name="xyz"/\> tags.
     * All other child tags raise a VirtualRobot::VirtualRobotException.
     * The resulting nodes are stored in \p nodeList.
     *
     * If the parameter \p clearList is true all elements from \p nodeList are removed.
     */
    void BaseIO::processNodeList(const rapidxml::xml_node<char>* parentNode, RobotPtr robot, std::vector<RobotNodePtr>& nodeList, bool clearList /*= true*/)
    {
        if (clearList)
        {
            nodeList.clear();
        }

        std::string parentName = processNameAttribute(parentNode, true);
        rapidxml::xml_node<>* node = parentNode->first_node();

        while (node)
        {
            std::string nodeName = getLowerCase(node->name());

            if (nodeName == "node")
            {
                std::string nodeNameAttr = processNameAttribute(node);
                THROW_VR_EXCEPTION_IF(nodeNameAttr.empty(), "Missing name attribute for <Node> belonging to Robot node set " << parentName);
                RobotNodePtr robotNode = robot->getRobotNode(nodeNameAttr);
                THROW_VR_EXCEPTION_IF(!robotNode, "<node> tag with name '" << nodeNameAttr << "' not present in the current robot");
                nodeList.push_back(robotNode);
            }
            else
            {
                THROW_VR_EXCEPTION("XML definition <" << nodeName << "> not supported in <RobotNodeSet> with name " << parentName);
            }

            node = node->next_sibling();
        }
    }


    NodeMapping BaseIO::processNodeMapping(const rapidxml::xml_node<char>* XMLNode, RobotPtr robot)
    {
        (void) robot;

        std::string parentName = processNameAttribute(XMLNode, true);
        rapidxml::xml_node<>* node = XMLNode->first_node();

        NodeMapping nodeMapping;
        while (node != nullptr)
        {
            std::string nodeName = getLowerCase(node->name());

            if (nodeName == "mapping")
            {
                const std::string from = processStringAttribute("from", node, true);
                const std::string to = processStringAttribute("to", node, true);
                const float sign = processFloatAttribute("sign", node, true);

                THROW_VR_EXCEPTION_IF(not(sign == 1 or sign == -1), "'sign' attribute has to be either '1' or '-1'");
                THROW_VR_EXCEPTION_IF(from.empty(),  "'from' attribute is empty!");
                THROW_VR_EXCEPTION_IF(to.empty(), "'to' attribute is empty!");

                // allow bidirectional lookup
                nodeMapping.emplace(from, NodeMappingElement{.node = to, .sign = sign});
                nodeMapping.emplace(to, NodeMappingElement{.node = from, .sign = sign});
            }
            else
            {
                THROW_VR_EXCEPTION("XML definition <" << nodeName << "> not supported in <NodeMapping> with name " << parentName);
            }

            node = node->next_sibling();
        }

        return nodeMapping;
    }

    HumanMapping BaseIO::processHumanMapping(const rapidxml::xml_node<char>* XMLNode, const RobotPtr& robot)
    {
        (void) robot;

        HumanMapping humanMapping;

        enum class Side
        {
            LEFT,
            RIGHT
        };

        const auto getSide = [&](const auto node){
            VR_ASSERT(node != nullptr);
            const std::string side = processStringAttribute("side", node, true);

            if(side == "left")
            {
                return Side::LEFT;
            }

            if(side == "right")
            {
                return Side::RIGHT;
            }

            VR_ERROR << "Unknown side `" << side <<  "`";
            throw std::invalid_argument("Unknown side `" + side + "`");
        };

        {
            if(XMLNode == nullptr)
            {
                VR_ERROR << "XMLNode must not be null!";
            }

            rapidxml::xml_node<>* armNode = XMLNode->first_node("humanmapping", 0, false);
            if(armNode == nullptr)
            {
                VR_ERROR << "No humanmapping found!";
            }

            while (armNode != nullptr)
            {
                VR_INFO << "arm node";

                const auto side = getSide(armNode);

                const rapidxml::xml_node<char> *segmentsNode =
                    armNode->first_node("segmentnodes", 0, false);

                if(segmentsNode == nullptr)
                {
                    VR_ERROR << "SegmentNodes missing for side " << static_cast<int>(side) << "!";
                }

                const auto findSegmentNode = [&segmentsNode](const std::string& type) -> rapidxml::xml_node<char>*
                {

                    auto* jointNode = segmentsNode->first_node("SegmentNode");

                    while(jointNode != nullptr)
                    {
                        const std::string nodeType = processStringAttribute("type", jointNode, true);

                        if(simox::alg::to_lower(type) == simox::alg::to_lower(nodeType))
                        {
                            return jointNode;
                        }

                        // advance to next sibling
                        jointNode = jointNode->next_sibling("SegmentNode", 0, false);
                    }

                    VR_ERROR << "Failed to find SegmentNode with type `" << type << "`!";
                    return nullptr;
                };

                const auto processSegmentNode = [&segmentsNode, &findSegmentNode](const std::string& type)
                {
                    // find the segment node with the correct type
                    const auto* node = findSegmentNode(type);
                    if(node == nullptr)
                    {
                        VR_ERROR << "SegmentNode missing for type " << type << "!";
                    }

                    const std::string name = processStringAttribute("name", node, true);

                    Eigen::Matrix4f transform;
                    processTransformNode(segmentsNode->first_node("transform", 0, false), "transform", transform);

                    return HumanMapping::ArmDescription::Segment
                    {
                        .nodeName = name,
                        .offset = transform
                    };
                };

                const HumanMapping::ArmDescription::Segment shoulder = processSegmentNode("shoulder");
                const HumanMapping::ArmDescription::Segment elbow = processSegmentNode("elbow");
                const HumanMapping::ArmDescription::Segment wrist = processSegmentNode("wrist");
                const HumanMapping::ArmDescription::Segment palm = processSegmentNode("palm");
                const HumanMapping::ArmDescription::Segment tcp = processSegmentNode("tcp");

                const std::string nodeSet = processStringAttribute("name", armNode, true);

                const HumanMapping::ArmDescription::Segments segments
                {
                    .shoulder = shoulder,
                    .elbow = elbow,
                    .wrist = wrist,
                    .palm = palm,
                    .tcp = tcp
                };

                const rapidxml::xml_node<char>* armJointsNode = armNode->first_node("JointNodes", 0, false);
                VR_ASSERT(armJointsNode != nullptr);

                HumanMapping::ArmDescription::JointMapping jointMapping;
                const rapidxml::xml_node<char>* jointNode = armJointsNode->first_node("JointNode", 0, false);
                VR_ASSERT(jointNode != nullptr);

                if(jointNode == nullptr)
                {
                    VR_ERROR << "No JointNode available!";
                }

                while(jointNode != nullptr)
                {
                    const std::string type = simox::alg::to_lower(processStringAttribute("type", jointNode, true));
                    const std::string motion = simox::alg::to_lower(processStringAttribute("motion", jointNode, true));
                    const std::string node = processStringAttribute("node", jointNode, true);
                    const bool inverted = processOptionalBoolAttribute("inverted", jointNode, false);
                    const float offset = processFloatAttribute("offset", jointNode, true);

                    // VR_INFO << type << "/" << motion << std::endl;

                    jointMapping[node] = HumanMapping::ArmDescription::HumanJointDescription{.type = type, .motion = motion, .offset = offset, .inverted = inverted};

                    // advance to next sibling
                    jointNode = jointNode->next_sibling("JointNode", 0, false);
                }

                // VR_INFO << "Joint mapping size: " << jointMapping.size() << std::endl;

                const HumanMapping::ArmDescription armDescription
                {
                    .segments = segments,
                    .jointMapping = jointMapping,
                    .nodeSet = nodeSet  
                };

                switch(side)
                {
                    case Side::LEFT:
                        humanMapping.leftArm = armDescription;
                        break;
                    case Side::RIGHT:
                        humanMapping.rightArm = armDescription;
                        break;
                }

                armNode = armNode->next_sibling("humanmapping", 0, false);

            }
        }

        return humanMapping;
    }


    /**
    * This method takes a rapidxml::xml_node and returns the value of the
    * first tag it finds with name \p attributeName.
    * If an error occurs an exception is thrown
    * If more than one name attribute is found an exception is thrown.
    * If no attribute can be found 0.0 is returned.
    *
    * \warning Do NOT use this method if there are several attributes which need to be processed (or set \p allowOtherAttributes to silently ignore other attributes)
    *
    * \return the value of the attribute or 0.0 if no attribute was found
    */
    float BaseIO::processFloatAttribute(const std::string& attributeName, const rapidxml::xml_node<char>* node, bool allowOtherAttributes /* = false */)
    {
        THROW_VR_EXCEPTION_IF(!node, "Can not process name attribute `" << attributeName << "` of NULL node" << endl);

        bool result = false;
        float value = 0.0f;
        //std::string nodeNameAttr("");
        rapidxml::xml_attribute<char>* attr = node->first_attribute();

        while (attr)
        {
            std::string name = getLowerCase(attr->name());

            if (attributeName == name)
            {
                THROW_VR_EXCEPTION_IF(result, "<" << node->name() << "> tag contains multiple attributes with name " << attributeName);
                value = convertToFloat(attr->value());
                result = true;
            }
            else
            {
                if (!allowOtherAttributes)
                {
                    THROW_VR_EXCEPTION("<" << node->name() << "> tag contains unknown attribute: " << attr->name());
                }
            }

            attr = attr->next_attribute();
        }

        return value;
    }


    /**
    * This method takes a rapidxml::xml_node and returns the value of the
    * first tag it finds with name \p attributeName.
    * If an error occurs an exception is thrown
    * If more than one name attribute is found an exception is thrown.
    * If no attribute can be found 0 is returned.
    *
    * \warning Do NOT use this method if there are several attributes which need to be processed (or set \p allowOtherAttributes to silently ignore other attributes)
    *
    * \return the value of the attribute or 0.0 if no attribute was found
    */
    int BaseIO::processIntAttribute(const std::string& attributeName, const rapidxml::xml_node<char>* node, bool allowOtherAttributes /* = false */)
    {
        THROW_VR_EXCEPTION_IF(!node, "Can not process name attribute `" << attributeName << "` of NULL node" << endl);

        bool result = false;
        int value = 0;
        //std::string nodeNameAttr("");
        rapidxml::xml_attribute<char>* attr = node->first_attribute();

        while (attr)
        {
            std::string name = getLowerCase(attr->name());

            if (attributeName == name)
            {
                THROW_VR_EXCEPTION_IF(result, "<" << node->name() << "> tag contains multiple attributes with name " << attributeName);
                value = convertToInt(attr->value());
                result = true;
            }
            else
            {
                if (!allowOtherAttributes)
                {
                    THROW_VR_EXCEPTION("<" << node->name() << "> tag contains unknown attribute: " << attr->name());
                }
            }

            attr = attr->next_attribute();
        }

        return value;
    }


    bool BaseIO::processOptionalBoolAttribute(const std::string& attributeName, const rapidxml::xml_node<char>* node, bool defaultValue)
    {
        THROW_VR_EXCEPTION_IF(!node, "Can not process name attribute `" << attributeName << "` of NULL node" << endl);

        rapidxml::xml_attribute<char>* attr = node->first_attribute(attributeName.c_str(), 0, false);

        if(attr == nullptr)
        {
            return defaultValue;
        }

        return isTrue(attr->value());
    }
    

    /**
        * This method takes a rapidxml::xml_node and returns the value of the
        * first tag it finds with name \p attributeName.
        * If an error occurs a message is logged to the console and "" is
        * returned.
        * If more than one name attribute is found an exception is thrown.
        * If no attribute can be found 0.0 is returned.
        *
        * \warning Do NOT use this method if there are several attributes which need to be processed (or set \p allowOtherAttributes to silently ignore other attributes)
        *
        * \return the value of the name attribute or an empty string on error
        */
    std::string BaseIO::processStringAttribute(const std::string& attributeName, const rapidxml::xml_node<char>* node, bool allowOtherAttributes /* = false */)
    {
        THROW_VR_EXCEPTION_IF(!node, "Can not process name attribute `" << attributeName << "` of NULL node" << endl);

        bool result = false;
        std::string value;
        std::string nodeNameAttr("");
        rapidxml::xml_attribute<char>* attr = node->first_attribute();

        while (attr)
        {
            std::string name = getLowerCase(attr->name());

            if (attributeName == name)
            {
                THROW_VR_EXCEPTION_IF(result, "<" << node->name() << "> tag contains multiple attributes with name " << attributeName);
                value = attr->value();
                result = true;
            }
            else
            {
                if (!allowOtherAttributes)
                {
                    THROW_VR_EXCEPTION("<" << node->name() << "> tag contains unknown attribute: " << attr->name());
                }
            }

            attr = attr->next_attribute();
        }

        return value;
    }

    void BaseIO::makeAbsolutePath(const std::string& basePath, std::string& filename)
    {
        if (filename.empty())
        {
            return;
        }

        std::filesystem::path filenameNew(filename);
        std::filesystem::path filenameBasePath(basePath);

        std::filesystem::path filenameNewComplete = filenameBasePath / filenameNew;
        filename = filenameNewComplete.string();
    }

    void BaseIO::makeRelativePath(const std::string& basePath, std::string& filename)
    {
        if (filename.empty())
        {
            return;
        }

        filename = simox::fs::make_relative(std::filesystem::path(basePath), std::filesystem::path(filename));
    }



    /**
     * This method takes a rapidxml::xml_node and returns the value of the
     * first name tag it finds.
     * If an error occurs a message is logged to the console and an empty string is
     * returned.
     * If more than one name attribute is found an exception is thrown.
     *
     * \warning Do NOT use this method if there are several attributes which need to be processed (or set \p allowOtherAttributes to silently ignore other attributes)
     *
     * \return the value of the name attribute or an empty string on error
     */
    std::string BaseIO::processNameAttribute(const rapidxml::xml_node<char>* node, bool allowOtherAttributes /* = false */)
    {
        std::string nameStr("name");
        return processStringAttribute(nameStr, node, allowOtherAttributes);
    }



    VisualizationNodePtr BaseIO::processVisualizationTag(const rapidxml::xml_node<char>* visuXMLNode, const std::string& tagName, const std::string& basePath, bool& useAsColModel)
    {
        bool enableVisu = true;
        bool coordAxis = false;
        float coordAxisFactor = 1.0f;
        std::string coordAxisText = "";
        std::string visuCoordType = "";
        useAsColModel = false;
        std::string visuFileType = "";
        rapidxml::xml_attribute<>* attr;
        std::vector<Primitive::PrimitivePtr> primitives;
        VisualizationNodePtr visualizationNode;
        std::vector<VisualizationNodePtr> visualizationNodes;

        if (!visuXMLNode)
        {
            return visualizationNode;
        }

        attr = visuXMLNode->first_attribute("enable", 0, false);

        if (attr)
        {
            enableVisu = isTrue(attr->value());
        }

        attr = visuXMLNode->first_attribute("useascollisionmodel", 0, false);

        if (attr)
        {
            useAsColModel = isTrue(attr->value());
        }

        if (enableVisu)
        {
            visualizationNodes = processVisuFiles(visuXMLNode, basePath, visuFileType);
            primitives = processPrimitives(visuXMLNode);
            THROW_VR_EXCEPTION_IF(primitives.size() != 0 && visualizationNodes.size() != 0, "Multiple visualization sources defined (file and primitives)" << endl);

            if (visualizationNodes.size() == 1)
            {
                visualizationNode = visualizationNodes.at(0);
            }
            else if (visualizationNodes.size() > 1)
            {
                VisualizationFactoryPtr visualizationFactory = VisualizationFactory::first(NULL);
                visualizationNode = visualizationFactory->createUnitedVisualization(visualizationNodes);
            }

            else if (primitives.size() != 0)
            {
                VisualizationFactoryPtr visualizationFactory = VisualizationFactory::first(NULL);
                visualizationNode = visualizationFactory->getVisualizationFromPrimitives(primitives);
            }


            rapidxml::xml_node<>* coordXMLNode = visuXMLNode->first_node("coordinateaxis", 0, false);

            if (coordXMLNode)
            {
                attr = coordXMLNode->first_attribute("enable", 0, false);

                if (attr)
                {
                    coordAxis = isTrue(attr->value());
                }

                if (coordAxis)
                {

                    coordAxisFactor = getOptionalFloatByAttributeName(coordXMLNode, "scaling", 1.0f);

                    attr = coordXMLNode->first_attribute("text", 0, false);

                    if (attr)
                    {
                        coordAxisText = attr->value();
                    }
                    else
                    {
                        coordAxisText = tagName;
                    }

                    attr = coordXMLNode->first_attribute("type", 0, false);

                    //THROW_VR_EXCEPTION_IF(!attr, "Missing 'type' attribute in <CoordinateAxis> tag of node " << tagName << "." << endl)
                    if (!attr)
                    {
                        VisualizationFactoryPtr f = VisualizationFactory::first(NULL);

                        if (f)
                        {
                            visuCoordType = f->getDescription();
                        }
                        else
                        {
                            VR_WARNING << "No visualization present..." << std::endl;
                        }
                    }
                    else
                    {
                        visuCoordType = attr->value();
                    }

                    getLowerCase(visuCoordType);
                    VisualizationFactoryPtr visualizationFactory = VisualizationFactory::fromName(visuCoordType, NULL);

                    if (!visualizationNode)
                    {
                        // create dummy visu
                        if (visualizationFactory)
                        {
                            visualizationNode = visualizationFactory->createVisualization();
                        }
                        else
                        {
                            VR_WARNING << "VisualizationFactory of type '" << visuFileType << "' not present. Ignoring Visualization data in Robot Node <" << tagName << ">" << std::endl;
                        }
                    }
                    else
                    {
                        THROW_VR_EXCEPTION_IF(visuCoordType.compare(visuFileType) != 0, "Different 'type' attributes not supported for <CoordinateAxis> tag and <File> tag of node " << tagName << "." << endl);
                    }

                    if (visualizationNode && visualizationFactory)
                    {
                        VisualizationNodePtr coordVisu = visualizationFactory->createCoordSystem(coordAxisFactor, &coordAxisText);
                        visualizationNode->attachVisualization("CoordinateSystem", coordVisu);
                        //visualizationNode->showCoordinateSystem(true,coordAxisFactor,&coordAxisText);
                    }
                    else
                    {
                        VR_WARNING << "VisualizationFactory of type '" << visuFileType << "' not present. Ignoring Visualization data in Robot Node <" << tagName << ">" << std::endl;
                    }

                }
            }

            rapidxml::xml_node<>* useColModel = visuXMLNode->first_node("useascollisionmodel", 0, false);

            if (useColModel)
            {
                useAsColModel = true;
            }
        }

        return visualizationNode;
    }

    CollisionModelPtr BaseIO::processCollisionTag(const rapidxml::xml_node<char>* colXMLNode, const std::string& tagName, const std::string& basePath)
    {
        rapidxml::xml_attribute<>* attr;
        std::string collisionFileType = "";
        VisualizationNodePtr visualizationNode;
        CollisionModelPtr collisionModel;
        std::vector<Primitive::PrimitivePtr> primitives;
        std::vector<VisualizationNodePtr> visuNodes;
        bool enableCol = true;

        attr = colXMLNode->first_attribute("enable", 0, false);

        if (attr)
        {
            enableCol = isTrue(attr->value());
        }

        if (enableCol)
        {

            visuNodes = processVisuFiles(colXMLNode, basePath, collisionFileType);
            primitives = processPrimitives(colXMLNode);
            THROW_VR_EXCEPTION_IF(primitives.size() != 0 && visuNodes.size() != 0, "Multiple collision model sources defined (file and primitives)" << endl);

            if (visuNodes.size() != 0)
            {
                if (visuNodes.size() == 1)
                {
                    visualizationNode = visuNodes.at(0);
                }
                else
                {
                    if (auto factory = VisualizationFactory::fromName(collisionFileType, NULL))
                    {
                        visualizationNode = factory->createUnitedVisualization(visuNodes);
                    }
                    else if (auto factory = VisualizationFactory::fromName("inventor", NULL))
                    {
                        VR_WARNING << "VisualizationFactory of type '" << collisionFileType << "' not present. Trying factory for 'inventor' " << std::endl;
                        visualizationNode = factory->createUnitedVisualization(visuNodes);
                    }
                    else
                    {
                        VR_WARNING << "VisualizationFactory of type '" << collisionFileType << "' not present. Ignoring Visualization data in Robot Node <" << tagName << ">" << std::endl;
                    }
                }
            }
            else if (primitives.size() != 0)
            {
                VisualizationFactoryPtr visualizationFactory = VisualizationFactory::first(NULL);
                visualizationNode = visualizationFactory->getVisualizationFromPrimitives(primitives);
            }

            if (visualizationNode)
            {
                std::string colModelName = tagName;
                colModelName += "_ColModel";
                // todo: ID?
                collisionModel.reset(new CollisionModel(visualizationNode, colModelName, CollisionCheckerPtr()));
            }
        }

        return collisionModel;
    }

    std::vector<VisualizationNodePtr> BaseIO::processVisuFiles(const rapidxml::xml_node<char>* visualizationXMLNode, const std::string& basePath, std::string& fileType)
    {
        const rapidxml::xml_node<>* node = visualizationXMLNode;
        std::vector<VisualizationNodePtr> result;
        bool bbox = false;

        if (!node)
        {
            return result;
        }

        rapidxml::xml_node<>* visuFileXMLNode = node->first_node("file", 0, false);

        while (visuFileXMLNode)
        {
            std::string visuFile = "";
            std::string tmpFileType = "";

            rapidxml::xml_attribute<>* attr = visuFileXMLNode->first_attribute("type", 0, false);

            if (!attr)
            {
                if (VisualizationFactory::first(NULL))
                {
                    tmpFileType = VisualizationFactory::first(NULL)->getDescription();
                    getLowerCase(tmpFileType);
                }
                else
                {
                    VR_WARNING << "No visualization present..." << std::endl;
                }
            }
            else
            {
                tmpFileType = attr->value();
                getLowerCase(tmpFileType);
            }

            if (fileType == "")
            {
                fileType = tmpFileType;
            }

            attr = visuFileXMLNode->first_attribute("boundingbox", 0, false);

            if (attr)
            {
                bbox = isTrue(attr->value());
            }

            getLowerCase(fileType);
            visuFile = processFileNode(visuFileXMLNode, basePath);

            if (visuFile != "")
            {
                VisualizationFactoryPtr factory = VisualizationFactory::fromName(fileType, NULL);

                if (factory = VisualizationFactory::fromName(fileType, NULL))
                {
                    if (tmpFileType == fileType)
                    {
                        result.push_back(factory->getVisualizationFromFile(visuFile, bbox));
                    }
                    else
                    {
                        VR_WARNING << "Ignoring data from " << visuFileXMLNode->value() << ": visualization type does not match to data from before." << std::endl;
                    }
                }
                else if (auto factory = VisualizationFactory::fromName("inventor", NULL))
                {
                    VR_WARNING << "VisualizationFactory of type '" << fileType << "' not present. Trying factory for 'inventor' " << std::endl;
                    if (tmpFileType == fileType)
                    {
                        result.push_back(factory->getVisualizationFromFile(visuFile, bbox));
                    }
                    else
                    {
                        VR_WARNING << "Ignoring data from " << visuFileXMLNode->value() << ": visualization type does not match to data from before." << std::endl;
                    }
                }
                else
                {
                    VR_WARNING << "VisualizationFactory of type '" << fileType << "' not present. Ignoring Visualization data from " << visuFileXMLNode->value() << std::endl;
                }
            }

            visuFileXMLNode = visuFileXMLNode->next_sibling("file", 0, false);
        }

        return result;
    }

    std::vector<Primitive::PrimitivePtr> BaseIO::processPrimitives(const rapidxml::xml_node<char>* primitivesXMLNode)
    {
        std::vector<Primitive::PrimitivePtr> result;
        const rapidxml::xml_node<>* node;

        if (!primitivesXMLNode)
        {
            return result;
        }

        if (primitivesXMLNode->name() == std::string{"primitives"})
        {
            node = primitivesXMLNode;
        }
        else
        {
            node = primitivesXMLNode->first_node("primitives", 0, false);
        }

        if (!node)
        {
            return result;
        }

        rapidxml::xml_node<>* primitiveXMLNode = node->first_node();

        while (primitiveXMLNode)
        {
            std::string pName = primitiveXMLNode->name();
            getLowerCase(pName);
            Primitive::PrimitivePtr primitive;

            float lenFactor = 1.f;

            if (hasUnitsAttribute(primitiveXMLNode))
            {
                Units u = getUnitsAttribute(primitiveXMLNode, Units::eLength);

                if (u.isMeter())
                {
                    lenFactor = 1000.f;
                }
            }

            if (pName == "box")
            {
                Primitive::Box* box = new Primitive::Box;
                box->width = convertToFloat(primitiveXMLNode->first_attribute("width", 0, false)->value()) * lenFactor;
                box->height = convertToFloat(primitiveXMLNode->first_attribute("height", 0, false)->value()) * lenFactor;
                box->depth = convertToFloat(primitiveXMLNode->first_attribute("depth", 0, false)->value()) * lenFactor;
                primitive.reset(box);
            }
            else if (pName == "sphere")
            {
                Primitive::Sphere* sphere = new Primitive::Sphere;
                sphere->radius = convertToFloat(primitiveXMLNode->first_attribute("radius", 0, false)->value()) * lenFactor;
                primitive.reset(sphere);
            }
            else if (pName == "cylinder")
            {
                Primitive::Cylinder* cylinder = new Primitive::Cylinder;
                cylinder->radius = convertToFloat(primitiveXMLNode->first_attribute("radius", 0, false)->value()) * lenFactor;
                cylinder->height = convertToFloat(primitiveXMLNode->first_attribute("height", 0, false)->value()) * lenFactor;
                primitive.reset(cylinder);
            }
            else
            {
                VR_ERROR << "Unknown primitive type \"" << pName << "\"; skipping";
            }

            if (primitive)
            {
                Eigen::Matrix4f transform;
                processTransformNode(primitiveXMLNode->first_node("transform", 0, false), "transform", transform);
                primitive->transform = transform;
                result.push_back(primitive);
            }

            primitiveXMLNode = primitiveXMLNode->next_sibling();
        }

        return result;
    }

    void BaseIO::processPhysicsTag(const rapidxml::xml_node<char>* physicsXMLNode, const std::string& nodeName, SceneObject::Physics& physics)
    {
        THROW_VR_EXCEPTION_IF(!physicsXMLNode, "NULL data for physicsXMLNode in processPhysicsNode()");
        rapidxml::xml_attribute<>* attr;
        rapidxml::xml_node<>* massXMLNode = physicsXMLNode->first_node("mass", 0, false);

        if (massXMLNode)
        {
            physics.massKg = getFloatByAttributeName(massXMLNode, "value");

            if (!hasUnitsAttribute(massXMLNode))
            {
                VR_ERROR << "No units attribute at <" << nodeName << ">" << std::endl;
            }

            Units unit = getUnitsAttribute(massXMLNode, Units::eWeight);

            if (unit.isGram())
            {
                physics.massKg *= 0.001f;
            }

            if (unit.isTon())
            {
                physics.massKg *= 1000.0f;
            }

        }
        else
        {
            VR_WARNING << "Expecting mass tag for physics node in <" << nodeName << ">." << std::endl;
            physics.massKg = 0.0f;
        }

        rapidxml::xml_node<>* comXMLNode = physicsXMLNode->first_node("com", 0, false);

        if (comXMLNode)
        {
            attr = comXMLNode->first_attribute("location", 0, false);

            if (attr)
            {
                std::string loc = attr->value();
                getLowerCase(loc);

                if (loc == "visualizationbboxcenter")
                {
                    physics.comLocation = SceneObject::Physics::eVisuBBoxCenter;
                }
                else if (loc == "joint" || loc == "custom")
                {
                    physics.comLocation = SceneObject::Physics::eCustom;
                }
                else
                {
                    THROW_VR_EXCEPTION("Unsupported Physics <CoM> tag attribute:" << loc);
                }
            }
            else
            {
                physics.comLocation = SceneObject::Physics::eCustom;
            }

            if (physics.comLocation == SceneObject::Physics::eCustom)
            {
                physics.localCoM(0) = getOptionalFloatByAttributeName(comXMLNode, "x", 0.0f);
                physics.localCoM(1) = getOptionalFloatByAttributeName(comXMLNode, "y", 0.0f);
                physics.localCoM(2) = getOptionalFloatByAttributeName(comXMLNode, "z", 0.0f);

                if (hasUnitsAttribute(comXMLNode))
                {
                    Units unitCom = getUnitsAttribute(comXMLNode, Units::eLength);

                    if (unitCom.isMeter())
                    {
                        physics.localCoM *= 1000.0f;
                    }

                }
            }

        }

        rapidxml::xml_node<>* inMatXMLNode = physicsXMLNode->first_node("inertiamatrix", 0, false);

        if (inMatXMLNode)
        {
            physics.inertiaMatrix = process3x3Matrix(inMatXMLNode);
            std::vector< Units > unitsAttr = getUnitsAttributes(inMatXMLNode);
            Units uWeight("kg");
            Units uLength("m");

            for (auto& i : unitsAttr)
            {
                if (i.isWeight())
                {
                    uWeight = i;
                }

                if (i.isLength())
                {
                    uLength = i;
                }
            }

            float factor = 1.0f;

            if (uWeight.isGram())
            {
                factor *= 0.001f;
            }

            if (uWeight.isTon())
            {
                factor *= 1000.0f;
            }

            if (uLength.isMillimeter())
            {
                factor *= 0.000001f;
            }

            physics.inertiaMatrix *= factor;

        }
        else
        {
            physics.inertiaMatrix.setZero(); // this will trigger an automatically determination of the inertia matrix during initialization
        }

        rapidxml::xml_node<>* ignoreColXMLNode = physicsXMLNode->first_node("ignorecollision", 0, false);

        while (ignoreColXMLNode)
        {
            rapidxml::xml_attribute<>* attr = ignoreColXMLNode->first_attribute("name", 0, false);
            THROW_VR_EXCEPTION_IF(!attr, "Expecting 'name' attribute in <IgnoreCollision> tag..." << endl)
            std::string s(attr->value());
            physics.ignoreCollisions.push_back(s);
            ignoreColXMLNode = ignoreColXMLNode->next_sibling("ignorecollision", 0, false);
        }

        rapidxml::xml_node<>* simulationtype = physicsXMLNode->first_node("simulationtype", 0, false);

        if (simulationtype)
        {
            rapidxml::xml_attribute<>* attr = simulationtype->first_attribute("value", 0, false);
            THROW_VR_EXCEPTION_IF(!attr, "Expecting 'value' attribute in <SimulationType> tag..." << endl)
            std::string s(attr->value());
            getLowerCase(s);

            if (s == "dynamic" || s == "dynamics")
            {
                physics.simType = VirtualRobot::SceneObject::Physics::eDynamic;
            }
            else if (s == "static")
            {
                physics.simType = VirtualRobot::SceneObject::Physics::eStatic;
            }
            else if (s == "kinematic")
            {
                physics.simType = VirtualRobot::SceneObject::Physics::eKinematic;
            }

            // otherwise eUnknown remains
        }
        rapidxml::xml_node<>* frictionXMLNode = physicsXMLNode->first_node("friction", 0, false);

        if (frictionXMLNode)
        {
            physics.friction = getFloatByAttributeName(frictionXMLNode, "value");
        }
        else
        {
            physics.friction = -1.0f;
        }


    }

    std::string BaseIO::processFileNode(const rapidxml::xml_node<char>* fileNode, const std::string& basePath)
    {
        THROW_VR_EXCEPTION_IF(!fileNode, "NULL data");
        std::string fileName = fileNode->value();
        THROW_VR_EXCEPTION_IF(fileName.empty(), "Invalid file defined in FILE tag");
        //bool relative = true;
        std::string pathStr("path");
        std::string pathAttribute = processStringAttribute(pathStr, fileNode, true);

        if (!pathAttribute.empty())
        {
            pathAttribute = getLowerCase(pathAttribute.c_str());

            if (pathAttribute == "relative")
            {
                makeAbsolutePath(basePath, fileName);
            }
            else if (pathAttribute != "absolute")
            {
                THROW_VR_EXCEPTION("Unknown path attribute in <File> tag:" << pathAttribute)
            }
        }
        else
        {

            // check file absolute
            std::filesystem::path fn(fileName);

            try
            {
                if (std::filesystem::exists(fn))
                {
                    return fileName;
                }
            }
            catch (...) {}

            // check file relative
            std::string absFileName = fileName;
            makeAbsolutePath(basePath, absFileName);
            fn = absFileName;

            try
            {
                if (std::filesystem::exists(fn))
                {
                    return absFileName;
                }
            }
            catch (...) {}

            // check file in data paths
            absFileName = fileName;

            if (RuntimeEnvironment::getDataFileAbsolute(absFileName))
            {
                return absFileName;
            }

            absFileName = fileName;
            makeAbsolutePath(basePath, absFileName);

            if (RuntimeEnvironment::getDataFileAbsolute(absFileName))
            {
                return absFileName;
            }

            VR_ERROR << "Could not determine valid filename from " << fileName << std::endl;
        }

        return fileName;
    }

    bool BaseIO::processConfigurationNode(const rapidxml::xml_node<char>* configXMLNode, std::vector< RobotConfig::Configuration >& storeConfigDefinitions, std::string&  storeConfigName)
    {
        THROW_VR_EXCEPTION_IF(!configXMLNode, "NULL data in processConfigurationNode");
        storeConfigName = processNameAttribute(configXMLNode, true);
        THROW_VR_EXCEPTION_IF(storeConfigName.empty(), "Expecting a name in configuration tag");

        rapidxml::xml_node<>* node = configXMLNode->first_node();

        while (node)
        {
            std::string nodeName = getLowerCase(node->name());

            if (nodeName == "node")
            {
                RobotConfig::Configuration c;
                c.name = processNameAttribute(node, true);
                THROW_VR_EXCEPTION_IF(c.name.empty(), "Expecting a name in configuration tag '" << storeConfigName << "'.");
                c.value = getFloatByAttributeName(node, "value");

                if (!hasUnitsAttribute(node))
                {
                    VR_ERROR << "No units attribute at <" << storeConfigName << ">" << std::endl;
                }

                /*if (getUnitsAttribute(node, Units::eAngle).isDegree())
                {
                    c.value = c.value / 180.0f * (float)M_PI;
                }*/

                Units u = getUnitsAttribute(node, Units::eIgnore);

                if (u.isDegree())
                {
                    c.value = c.value / 180.0f * (float)M_PI;
                }
                else if (u.isMeter())
                {
                    c.value = c.value / 1000.0f;
                }

                storeConfigDefinitions.push_back(c);
            }
            else
            {
                THROW_VR_EXCEPTION("XML definition <" << nodeName << "> not supported in configuration definition with name '" << storeConfigName << "'." << endl);
            }

            node = node->next_sibling();
        }

        return true;
    }

    bool BaseIO::processConfigurationNodeList(const rapidxml::xml_node<char>* configXMLNode, std::vector< std::vector< RobotConfig::Configuration > >& configDefinitions, std::vector< std::string >& configNames, std::vector< std::string >& tcpNames)
    {
        THROW_VR_EXCEPTION_IF(!configXMLNode, "NULL data in processConfigurationNode");
        std::string name = processNameAttribute(configXMLNode, true);
        THROW_VR_EXCEPTION_IF(name.empty(), "Expecting a name in configuration tag");

        std::string tcpStr("tcp");
        std::string tcp = processStringAttribute(tcpStr, configXMLNode, true);

        //std::cout << "tcp: " << tcp << std::endl;


        std::vector< RobotConfig::Configuration > configs;


        if (!processConfigurationNode(configXMLNode, configs, name))
        {
            return false;
        }

        configDefinitions.push_back(configs);
        configNames.push_back(name);
        tcpNames.push_back(tcp);
        return true;
    }


    RobotNodeSetPtr BaseIO::processRobotNodeSet(const rapidxml::xml_node<char>* setXMLNode, RobotPtr robo, const std::string& robotRootNode, int& robotNodeSetCounter)
    {
        THROW_VR_EXCEPTION_IF(!setXMLNode, "NULL data for setXMLNode");

        std::string nodeSetName;
        std::string rootNodeName;
        std::string tcpName;

        // get name and root
        rapidxml::xml_attribute<>* attr = setXMLNode->first_attribute();

        while (attr)
        {
            std::string name = getLowerCase(attr->name());

            if (name == "name")
            {
                THROW_VR_EXCEPTION_IF(!nodeSetName.empty(), "Robot node set contains multiple definitions of attribute name. First value of name is: " << nodeSetName);
                nodeSetName = attr->value();
            }
            else if (name == "kinematicroot")
            {
                THROW_VR_EXCEPTION_IF(!rootNodeName.empty(), "Robot node set contains multiple definitions of attribute kinematicroot. First value of kinematicroot is: " << rootNodeName);
                rootNodeName = attr->value();
            }
            else if (name == "tcp")
            {
                THROW_VR_EXCEPTION_IF(!tcpName.empty(), "Robot node set contains multiple definitions of attribute tcp. First value of tcpis: " << tcpName);
                tcpName = attr->value();
            }

            attr = attr->next_attribute();
        }

        if (nodeSetName.empty())
        {
            std::stringstream ss;
            ss << robo->getType() << "_RobotNodeSet_" << robotNodeSetCounter;
            nodeSetName = ss.str();
            robotNodeSetCounter++;
            VR_WARNING << "RobotNodeSet definition expects attribute 'name'. Setting name to " << nodeSetName << std::endl;
        }

        if (rootNodeName.empty())
        {
            rootNodeName = robotRootNode;
        }

        std::vector<RobotNodePtr> nodeList;
        processNodeList(setXMLNode, robo, nodeList);

        RobotNodePtr kinRoot;

        if (!rootNodeName.empty())
        {
            if (!robo->hasRobotNode(rootNodeName))
            {
                VR_WARNING << "In robot node set '" << nodeSetName
                           << "': No root node '" << rootNodeName << "' found";
            }
            kinRoot = robo->getRobotNode(rootNodeName);
        }

        RobotNodePtr tcp;

        if (!tcpName.empty())
        {
            if (!robo->hasRobotNode(tcpName))
            {
                VR_WARNING << "In robot node set '" << nodeSetName
                           << "': No root node '" << tcpName << "' found";
            }
            tcp = robo->getRobotNode(tcpName);
        }

        RobotNodeSetPtr rns = RobotNodeSet::createRobotNodeSet(robo, nodeSetName, nodeList, kinRoot, tcp, true);

        return rns;
    }


    bool BaseIO::processFloatValueTags(const rapidxml::xml_node<char>* XMLNode, int dim, Eigen::VectorXf& stroreResult)
    {
        if (!XMLNode || dim <= 0)
        {
            return false;
        }

        stroreResult.resize(dim);
        int entry = 0;
        rapidxml::xml_node<>* node = XMLNode->first_node();

        while (node)
        {
            std::string nodeName = getLowerCase(node->name());

            if (nodeName == "c")
            {
                THROW_VR_EXCEPTION_IF(entry >= dim, "Too many entries in trajectory's point definition..." << endl);
                stroreResult(entry) = getFloatByAttributeName(node, "value");
            }
            else
            {
                THROW_VR_EXCEPTION("XML definition <" << nodeName << "> not supported in trajectory point definition" << endl);
            }

            entry++;
            node = node->next_sibling();
        }

        THROW_VR_EXCEPTION_IF(entry < dim, "Not enough entries in trajectory's point definition..." << endl);
        return true;
    }

    TrajectoryPtr BaseIO::processTrajectory(const rapidxml::xml_node<char>* trajectoryXMLNode, std::vector<RobotPtr>& robots)
    {
        THROW_VR_EXCEPTION_IF(!trajectoryXMLNode, "NULL data for trajectoryXMLNode");

        std::string robotName;
        std::string nodeSetName;
        std::string trajName;
        int dim = -1;

        // get name and root
        rapidxml::xml_attribute<>* attr = trajectoryXMLNode->first_attribute();

        while (attr)
        {
            std::string name = getLowerCase(attr->name());

            if (name == "name")
            {
                THROW_VR_EXCEPTION_IF(!trajName.empty(), "Trajectory contains multiple definitions of attribute name. First value  is: " << trajName);
                trajName = attr->value();
            }
            else if (name == "robot" || name == "model")
            {
                THROW_VR_EXCEPTION_IF(!robotName.empty(), "Trajectory contains multiple definitions of attribute Robot. First value is: " << robotName);
                robotName = attr->value();
            }
            else if (name == "robotnodeset" || name == "modelnodeset")
            {
                THROW_VR_EXCEPTION_IF(!nodeSetName.empty(), "Trajectory contains multiple definitions of attribute RobotNodeSet. First value is: " << nodeSetName);
                nodeSetName = attr->value();
            }
            else if (name == "dim" || name == "dimension")
            {
                THROW_VR_EXCEPTION_IF(dim != -1, "Trajectory contains multiple definitions of attribute dim. First value of dim: " << dim);
                dim = convertToInt(attr->value());
            }

            attr = attr->next_attribute();
        }

        THROW_VR_EXCEPTION_IF(robotName.empty(), "Invalid or missing Robot attribute");
        THROW_VR_EXCEPTION_IF(nodeSetName.empty(), "Invalid or missing RobotNodeSet attribute");

        if (trajName.empty())
        {
            trajName = "Trajectory";
            VR_WARNING << "Trajectory definition expects attribute 'RobotNodeSet'. Setting to " << trajName << std::endl;
        }

        RobotPtr r;

        for (auto& robot : robots)
        {
            if (robot->getType() == robotName)
            {
                r = robot;
                break;
            }
        }

        THROW_VR_EXCEPTION_IF(!r, "Could not find robot with name " << robotName);
        RobotNodeSetPtr rns = r->getRobotNodeSet(nodeSetName);
        THROW_VR_EXCEPTION_IF(!rns, "Could not find RNS with name " << nodeSetName << " in robot " << robotName);

        assert(dim >= 0);
        if (static_cast<unsigned int>(dim) != rns->getSize())
        {
            VR_WARNING << " Invalid dim attribute (" << dim << "). Setting dimension to " << rns->getSize() << std::endl;
            dim = rns->getSize();
        }

        TrajectoryPtr res(new Trajectory(rns, trajName));
        rapidxml::xml_node<>* node = trajectoryXMLNode->first_node();

        while (node)
        {
            std::string nodeName = getLowerCase(node->name());

            if (nodeName == "point")
            {
                Eigen::VectorXf p;

                if (!processFloatValueTags(node, dim, p))
                {
                    VR_ERROR << "Error in processing configuration. Skipping entry" << std::endl;
                }
                else
                {
                    res->addPoint(p);
                }

            }
            else
            {
                THROW_VR_EXCEPTION("XML definition <" << nodeName << "> not supported in trajectory definition with name '" << trajName << "'." << endl);
            }

            node = node->next_sibling();
        }

        return res;
    }

    bool BaseIO::writeXMLFile(const std::string& filename, const std::string& content, bool overwrite)
    {
        try
        {
            if (!overwrite && std::filesystem::exists(filename))
            {
                return false;
            }
        }
        catch (...) {}

        // save file
        std::ofstream out(filename.c_str(), std::ios::out | std::ios::trunc);

        if (!out.is_open())
        {
            return false;
        }

        out << content;
        out.close();
        return true;
    }

    std::string BaseIO::toXML(const Eigen::Matrix4f& m, std::string ident /*= "\t"*/)
    {
        std::stringstream ss;
        ss << ident << "<Matrix4x4 units='mm'>" << std::endl;

        for (int r = 1; r <= 4; r++)
        {
            ss << ident << "\t<row" << r << " ";

            for (int i = 1; i <= 4; i++)
            {
                ss << "c" << i << "='" << m(r - 1, i - 1) << "' ";
            }

            ss << "/>" << std::endl;
        }

        ss << ident << "</Matrix4x4>" << std::endl;
        return ss.str();
    }

    GraspPtr BaseIO::processGrasp(const rapidxml::xml_node<char>* graspXMLNode, const std::string& robotType, const std::string& eef, const std::string& /*objName*/)
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

        rapidxml::xml_node<char>* chainedGraspNode = NULL;

        while (node)
        {
            std::string nodeName = getLowerCase(node->name());

            if (nodeName == "transform")
            {
                processTransformNode(node, name, pose);

            }
            else if (nodeName == "chaingrasp")
            {
                chainedGraspNode = node;
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

        GraspPtr grasp = nullptr;

        if (!chainedGraspNode)
            grasp.reset(new Grasp(name, robotType, eef, pose, method, quality, preshapeName));
        else {
            ChainedGraspPtr chainedGrasp(new ChainedGrasp(name, robotType, eef, Eigen::Matrix4f::Zero(), method, quality, preshapeName));
            node = chainedGraspNode->first_node();
            while (node)
            {
                std::string nodeName = getLowerCase(node->name());
                if (nodeName == "transform")
                {
                    processTransformNode(node, name, pose);
                    chainedGrasp->setObjectTransformation(pose);
                }
                else if (nodeName == "virtualjoint")
                {
                    rapidxml::xml_attribute<>* attrName = node->first_attribute("name", 0, false);
                    THROW_VR_EXCEPTION_IF(!attrName, "XML tag <VirtualJoint> does not contain an attribute 'name'!" << std::endl);
                    auto joint = chainedGrasp->getVirtualJoint(attrName->value());
                    rapidxml::xml_attribute<>* attrValue = node->first_attribute("value", 0, false);
                    THROW_VR_EXCEPTION_IF(!attrName, "XML tag <VirtualJoint> does not contain an attribute 'value'!" << std::endl);
                    joint->setValue(simox::alg::to_<float>(attrValue->value()));
                    rapidxml::xml_attribute<>* attrMin = node->first_attribute("min", 0, false);
                    rapidxml::xml_attribute<>* attrMax = node->first_attribute("max", 0, false);
                    if (attrMin && attrMax)
                        joint->setLimits(simox::alg::to_<float>(attrMin->value()), simox::alg::to_<float>(attrMax->value()));
                }
                else
                {
                    THROW_VR_EXCEPTION("XML definition <" << nodeName << "> not supported in Grasp <" << name << ">." << endl);
                }

                node = node->next_sibling();
            }
            grasp = chainedGrasp;
        }

        if (!configDefinitions.empty())
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

    GraspSetPtr BaseIO::processGraspSet(const rapidxml::xml_node<char>* graspSetXMLNode, const std::string& objName)
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

    bool BaseIO::processSensor(GraspableSensorizedObjectPtr rn, const rapidxml::xml_node<char>* sensorXMLNode, RobotDescription loadMode, const std::string& basePath)
    {
        if (!rn || !sensorXMLNode)
        {
            VR_ERROR << "NULL DATA ?!" << std::endl;
            return false;
        }

        rapidxml::xml_attribute<>* attr;
        std::string sensorType;

        attr = sensorXMLNode->first_attribute("type", 0, false);

        if (attr)
        {
            sensorType = getLowerCase(attr->value());
        }
        else
        {
            VR_WARNING << "No 'type' attribute for <Sensor> tag. Skipping Sensor definition of RobotNode " << rn->getName() << "!" << std::endl;
            return false;
        }

        SensorPtr s;


        SensorFactoryPtr sensorFactory = SensorFactory::fromName(sensorType, nullptr);

        if (sensorFactory)
        {
            s = sensorFactory->createSensor(rn, sensorXMLNode, loadMode, basePath);
        }
        else
        {
            VR_WARNING << "No Factory found for sensor of type " << sensorType << ". Skipping Sensor definition of RobotNode " << rn->getName() << "!" << std::endl;
            return false;
        }

        return rn->registerSensor(s);
    }

} // namespace VirtualRobot
