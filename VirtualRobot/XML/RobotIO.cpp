
#include "RobotIO.h"

#include <fstream>
#include <iostream>
#include <vector>

#include <SimoxUtility/algorithm/string/string_tools.h>
#include <SimoxUtility/filesystem/remove_trailing_separator.h>
#include <SimoxUtility/math/convert/deg_to_rad.h>
#include <SimoxUtility/xml/rapidxml/rapidxml_print.hpp>
#include <VirtualRobot/Import/RobotImporterFactory.h>
#include <VirtualRobot/Nodes/FourBar/Joint.h>

#include "../CollisionDetection/CollisionModel.h"
#include "../EndEffector/EndEffector.h"
#include "../EndEffector/EndEffectorActor.h"
#include "../Nodes/RobotNodeFactory.h"
#include "../Nodes/RobotNodeFixedFactory.h"
#include "../Nodes/RobotNodeHemisphere.h"
#include "../Nodes/RobotNodePrismatic.h"
#include "../RobotConfig.h"
#include "../RobotFactory.h"
#include "../RobotNodeSet.h"
#include "../RuntimeEnvironment.h"
#include "../Transformation/DHParameter.h"
#include "../VirtualRobotException.h"
#include "../Visualization/TriMeshModel.h"
#include "../Visualization/VisualizationFactory.h"
#include "../Visualization/VisualizationNode.h"
#include "Nodes/RobotNodeFourBar.h"
#include "VirtualRobot.h"
#include "mujoco/RobotMjcf.h"
#include "rapidxml.hpp"

namespace VirtualRobot
{
    using std::endl;

    std::map<std::string, int> RobotIO::robot_name_counter;

    RobotIO::RobotIO() = default;

    RobotIO::~RobotIO() = default;

    void
    RobotIO::processChildNode(rapidxml::xml_node<char>* childXMLNode,
                              std::vector<std::string>& childrenNames)
    {
        THROW_VR_EXCEPTION_IF(!childXMLNode, "NULL data for childXMLNode in processChildNode()")

        rapidxml::xml_attribute<>* attr;
        attr = childXMLNode->first_attribute("name", 0, false);
        THROW_VR_EXCEPTION_IF(!attr, "Expecting 'name' attribute in <Child> tag..." << endl)

        std::string s(attr->value());
        childrenNames.push_back(s);
    }

    void
    RobotIO::processChildFromRobotNode(rapidxml::xml_node<char>* childXMLNode,
                                       const std::string& nodeName,
                                       std::vector<ChildFromRobotDef>& childrenFromRobot)
    {
        rapidxml::xml_attribute<>* attr;

        THROW_VR_EXCEPTION_IF(!childXMLNode,
                              "NULL data for childXMLNode in processChildFromRobotNode()")

        ChildFromRobotDef d;

        rapidxml::xml_node<>* fileXMLNode = childXMLNode->first_node("file", 0, false);
        int counter = 0;

        while (fileXMLNode)
        {
            d.filename = fileXMLNode->value();

            d.importEEF = true;
            attr = fileXMLNode->first_attribute("importEEF", 0, false);

            if (attr)
            {
                if (!isTrue(attr->value()))
                {
                    d.importEEF = false;
                }
            }

            childrenFromRobot.push_back(d);
            fileXMLNode = fileXMLNode->next_sibling("file", 0, false);
        }

        THROW_VR_EXCEPTION_IF(((!counter) == 0),
                              "Missing file for <ChildFromRobot> tag (in node '" << nodeName
                                                                                 << "')." << endl);
    }

    /**
     * This method processes Limits tags.
     * The values for the attributes "lo" and "hi" are extracted based on the
     * "unit" or "units" attribute.
     *
     * The values are stored in \p jointLimitLo and \p jointLimitHi
     */
    void
    RobotIO::processLimitsNode(rapidxml::xml_node<char>* limitsXMLNode,
                               float& jointLimitLo,
                               float& jointLimitHi,
                               bool& limitless,
                               bool& allowJointLimitAvoidance)
    {
        THROW_VR_EXCEPTION_IF(!limitsXMLNode, "NULL data for limitsXMLNode in processLimitsNode()");

        Units unit = getUnitsAttribute(limitsXMLNode, Units::eIgnore);

        try
        {
            jointLimitLo = getFloatByAttributeName(limitsXMLNode, "lo");
        }
        catch (...)
        {
            if (unit.isLength())
            {
                VR_WARNING << "No 'lo' attribute in <Limits> tag. Assuming -1000 [mm]."
                           << std::endl;
                jointLimitLo = -1000.0f;
                unit = Units("mm");
            }
            else
            {
                VR_WARNING << "No 'lo' attribute in <Limits> tag. Assuming -180 [deg]."
                           << std::endl;
                jointLimitLo = -M_PIf32;
                unit = Units("rad");
            }
        }

        try
        {
            jointLimitHi = getFloatByAttributeName(limitsXMLNode, "hi");
        }
        catch (...)
        {
            if (unit.isLength())
            {
                VR_WARNING << "No 'hi' attribute in <Limits> tag. Assuming 1000 [mm]." << std::endl;
                jointLimitLo = 1000.0f;
                unit = Units("mm");
            }
            else
            {
                VR_WARNING << "No 'hi' attribute in <Limits> tag. Assuming 180 [deg]." << std::endl;
                jointLimitHi = M_PIf32;
                unit = Units("rad");
            }
        }

        // if values are stored as degrees convert them to radian
        if (unit.isAngle())
        {
            jointLimitLo = unit.toRadian(jointLimitLo);
            jointLimitHi = unit.toRadian(jointLimitHi);
            unit = Units("rad");
        }

        if (unit.isLength())
        {
            jointLimitLo = unit.toMillimeter(jointLimitLo);
            jointLimitHi = unit.toMillimeter(jointLimitHi);
            unit = Units("mm");
        }

        // limitless attribute
        rapidxml::xml_attribute<>* llAttr = limitsXMLNode->first_attribute("limitless");
        if (llAttr != nullptr)
        {
            limitless = isTrue(llAttr->value());
            if (limitless && unit.isAngle() &&
                (unit.toDegree(jointLimitHi) - unit.toDegree(jointLimitLo) != 360))
            {
                VR_WARNING << "Limitless Joint: Angular distance between 'lo' and 'hi' attributes "
                              "should equal 2*pi [rad] or 360 [deg]."
                           << endl
                           << "Setting 'lo' to -pi and 'hi' to pi [rad]..." << std::endl;
                jointLimitLo = -M_PIf32;
                jointLimitHi = M_PIf32;
            }
        }

        rapidxml::xml_attribute<>* llAllowAvoidance =
            limitsXMLNode->first_attribute("allowAvoidance");
        if (llAllowAvoidance != nullptr)
        {
            allowJointLimitAvoidance = isTrue(llAllowAvoidance->value());
        }
    }

    RobotNodePtr
    RobotIO::processJointNode(rapidxml::xml_node<char>* jointXMLNode,
                              const std::string& robotNodeName,
                              RobotPtr robot,
                              VisualizationNodePtr visualizationNode,
                              CollisionModelPtr collisionModel,
                              SceneObject::Physics& physics,
                              RobotNode::RobotNodeType rntype,
                              Eigen::Matrix4f& transformationMatrix)
    {
        static const float jointLimitLowDefault = -M_PIf32;
        static const float jointLimitHighDefault = M_PIf32;

        std::optional<float> jointLimitLow;
        std::optional<float> jointLimitHigh;
        bool limitless = false;
        bool allowJointLimitAvoidance = true;

        Eigen::Matrix4f preJointTransform = transformationMatrix; //Eigen::Matrix4f::Identity();
        Eigen::Vector3f axis = Eigen::Vector3f::Zero();
        Eigen::Vector3f translationDir = Eigen::Vector3f::Zero();

        std::vector<std::string> propagateJVName;
        std::vector<float> propagateJVFactor;

        rapidxml::xml_attribute<>* attr;
        float jointOffset = 0.0f;
        float initialvalue = 0.0f;

        if (!jointXMLNode)
        {
            // no <Joint> tag -> fixed joint
            RobotNodePtr robotNode;
            RobotNodeFactoryPtr fixedNodeFactory =
                RobotNodeFactory::fromName(RobotNodeFixedFactory::getName(), nullptr);
            if (fixedNodeFactory)
            {
                robotNode = fixedNodeFactory->createRobotNode(
                    robot,
                    robotNodeName,
                    visualizationNode,
                    collisionModel,
                    jointLimitLow.value_or(jointLimitLowDefault),
                    jointLimitHigh.value_or(jointLimitHighDefault),
                    jointOffset,
                    preJointTransform,
                    axis,
                    translationDir,
                    physics,
                    rntype);
            }
            return robotNode;
        }

        std::string jointType;
        attr = jointXMLNode->first_attribute("type", 0, false);
        if (attr)
        {
            jointType = getLowerCase(attr->value());
        }
        else
        {
            VR_WARNING << "No 'type' attribute for <Joint> tag. "
                       << "Assuming fixed joint for RobotNode " << robotNodeName << "!"
                       << std::endl;
            jointType = RobotNodeFixedFactory::getName();
        }

        attr = jointXMLNode->first_attribute("offset", 0, false);
        if (attr)
        {
            jointOffset = convertToFloat(attr->value());
        }

        attr = jointXMLNode->first_attribute("initialvalue", 0, false);
        if (attr)
        {
            initialvalue = convertToFloat(attr->value());
        }

        rapidxml::xml_node<>* node = jointXMLNode->first_node();
        rapidxml::xml_node<>* tmpXMLNodeAxis = nullptr;
        rapidxml::xml_node<>* tmpXMLNodeTranslation = nullptr;
        rapidxml::xml_node<>* limitsNode = nullptr;

        float maxVelocity = -1.0f; // m/s
        float maxAcceleration = -1.0f; // m/s^2
        float maxDeceleration = -1.0f; // m/s^2
        float maxTorque = -1.0f; // Nm
        float maxJerk = -1.0f;
        bool scaleVisu = false;
        Eigen::Vector3f scaleVisuFactor = Eigen::Vector3f::Zero();

        std::optional<RobotNodeHemisphere::XmlInfo> hemisphere;
        std::optional<RobotNodeFourBar::XmlInfo> fourBarXmlInfo;

        while (node)
        {
            const std::string nodeName = getLowerCase(node->name());

            if (nodeName == "dh")
            {
                THROW_VR_EXCEPTION("DH specification in Joint tag is DEPRECATED! "
                                   "Use "
                                   "<RobotNode><Transform><DH>...</DH></Transform><Joint>...</"
                                   "Joint></RobotNode> structure.")
                //THROW_VR_EXCEPTION_IF(dhXMLNode, "Multiple DH definitions in <Joint> tag of robot node <" << robotNodeName << ">." << endl);
                //dhXMLNode = node;
            }
            else if (nodeName == "limits")
            {
                THROW_VR_EXCEPTION_IF(limitsNode,
                                      "Multiple limits definitions in <Joint> tag of robot node <"
                                          << robotNodeName << ">." << endl);
                limitsNode = node;
                processLimitsNode(limitsNode,
                                  jointLimitLow.emplace(),
                                  jointLimitHigh.emplace(),
                                  limitless,
                                  allowJointLimitAvoidance);
            }
            else if (nodeName == "prejointtransform")
            {
                THROW_VR_EXCEPTION(
                    "PreJointTransform is DEPRECATED! "
                    "Use <RobotNode><Transform>...</Transform><Joint>...</Joint></RobotNode> "
                    "structure")
                //THROW_VR_EXCEPTION_IF(prejointTransformNode, "Multiple preJoint definitions in <Joint> tag of robot node <" << robotNodeName << ">." << endl);
                //prejointTransformNode = node;
            }
            else if (nodeName == "axis")
            {
                THROW_VR_EXCEPTION_IF(tmpXMLNodeAxis,
                                      "Multiple axis definitions in <Joint> tag of robot node <"
                                          << robotNodeName << ">." << endl);
                tmpXMLNodeAxis = node;
            }
            else if (nodeName == "translationdirection")
            {
                THROW_VR_EXCEPTION_IF(
                    tmpXMLNodeTranslation,
                    "Multiple translation definitions in <Joint> tag of robot node <"
                        << robotNodeName << ">." << endl);
                tmpXMLNodeTranslation = node;
            }
            else if (nodeName == "postjointtransform")
            {
                THROW_VR_EXCEPTION(
                    "postjointtransform is DEPRECATED and not longer allowed! "
                    "Use <RobotNode><Transform>...</Transform><Joint>...</Joint></RobotNode> "
                    "structure")
                //THROW_VR_EXCEPTION_IF(postjointTransformNode, "Multiple postjointtransform definitions in <Joint> tag of robot node <" << robotNodeName << ">." << endl);
                //postjointTransformNode = node;
            }
            else if (nodeName == "maxvelocity")
            {
                maxVelocity = getFloatByAttributeName(node, "value");

                // convert to m/s
                std::vector<Units> unitsAttr = getUnitsAttributes(node);
                Units uTime("sec");
                Units uLength("m");
                Units uAngle("rad");

                for (auto& i : unitsAttr)
                {
                    if (i.isTime())
                    {
                        uTime = i;
                    }

                    if (i.isLength())
                    {
                        uLength = i;
                    }

                    if (i.isAngle())
                    {
                        uAngle = i;
                    }
                }

                float factor = 1.0f;

                if (uTime.isMinute())
                {
                    factor /= 60.0f;
                }

                if (uTime.isHour())
                {
                    factor /= 3600.0f;
                }

                if (uLength.isMillimeter())
                {
                    factor *= 0.001f;
                }

                if (uAngle.isDegree())
                {
                    factor *= float(M_PI / 180.0);
                }

                maxVelocity *= factor;
            }
            else if (nodeName == "maxacceleration")
            {
                maxAcceleration = getFloatByAttributeName(node, "value");

                // convert to m/s^2
                std::vector<Units> unitsAttr = getUnitsAttributes(node);
                Units uTime("sec");
                Units uLength("m");
                Units uAngle("rad");

                for (auto& i : unitsAttr)
                {
                    if (i.isTime())
                    {
                        uTime = i;
                    }

                    if (i.isLength())
                    {
                        uLength = i;
                    }

                    if (i.isAngle())
                    {
                        uAngle = i;
                    }
                }

                float factor = 1.0f;

                if (uTime.isMinute())
                {
                    factor /= 3600.0f;
                }

                if (uTime.isHour())
                {
                    factor /= 12960000.0f;
                }

                if (uLength.isMillimeter())
                {
                    factor *= 0.001f;
                }

                if (uAngle.isDegree())
                {
                    factor *= float(M_PI / 180.0);
                }

                maxAcceleration *= factor;
            }
            else if (nodeName == "maxdeceleration")
            {
                maxDeceleration = getFloatByAttributeName(node, "value");

                // convert to m/s^2
                std::vector<Units> unitsAttr = getUnitsAttributes(node);
                Units uTime("sec");
                Units uLength("m");
                Units uAngle("rad");

                for (auto& i : unitsAttr)
                {
                    if (i.isTime())
                    {
                        uTime = i;
                    }

                    if (i.isLength())
                    {
                        uLength = i;
                    }

                    if (i.isAngle())
                    {
                        uAngle = i;
                    }
                }

                float factor = 1.0f;

                if (uTime.isMinute())
                {
                    factor /= 3600.0f;
                }

                if (uTime.isHour())
                {
                    factor /= 12960000.0f;
                }

                if (uLength.isMillimeter())
                {
                    factor *= 0.001f;
                }

                if (uAngle.isDegree())
                {
                    factor *= float(M_PI / 180.0);
                }

                maxDeceleration *= factor;
            }
            else if (nodeName == "maxjerk")
            {
                maxJerk = getFloatByAttributeName(node, "value");

                // convert to m/s^3
                std::vector<Units> unitsAttr = getUnitsAttributes(node);
                Units uTime("sec");
                Units uLength("m");
                Units uAngle("rad");

                for (auto& i : unitsAttr)
                {
                    if (i.isTime())
                    {
                        uTime = i;
                    }

                    if (i.isLength())
                    {
                        uLength = i;
                    }

                    if (i.isAngle())
                    {
                        uAngle = i;
                    }
                }

                float factor = 1.0f;

                if (uTime.isMinute())
                {
                    factor /= std::pow<float>(60.0f, 3.f);
                }

                if (uTime.isHour())
                {
                    factor /= std::pow<float>(3600.f, 3.f);
                }

                if (uLength.isMillimeter())
                {
                    factor *= 0.001f;
                }

                if (uAngle.isDegree())
                {
                    factor *= float(M_PI / 180.0);
                }

                maxJerk *= factor;
            }
            else if (nodeName == "maxtorque")
            {
                maxTorque = getFloatByAttributeName(node, "value");
                // convert to Nm
                std::vector<Units> unitsAttr = getUnitsAttributes(node);
                Units uLength("m");

                for (auto& i : unitsAttr)
                {
                    if (i.isLength())
                    {
                        uLength = i;
                    }
                }

                float factor = 1.0f;

                if (uLength.isMillimeter())
                {
                    factor *= 1000.0f;
                }

                maxTorque *= factor;
            }
            else if (nodeName == "propagatejointvalue")
            {
                rapidxml::xml_attribute<>* attrPropa;
                attrPropa = node->first_attribute("name", 0, false);
                THROW_VR_EXCEPTION_IF(!attrPropa,
                                      "Expecting 'name' attribute in <PropagateJointValue> tag..."
                                          << endl);

                std::string s(attrPropa->value());
                propagateJVName.push_back(s);
                float f = 1.0f;
                attrPropa = node->first_attribute("factor", 0, false);

                if (attrPropa)
                {
                    f = convertToFloat(attrPropa->value());
                }

                propagateJVFactor.push_back(f);
            }
            else if (nodeName == "scalevisualization")
            {
                scaleVisu = true;
                scaleVisuFactor[0] = getFloatByAttributeName(node, "x");
                scaleVisuFactor[1] = getFloatByAttributeName(node, "y");
                scaleVisuFactor[2] = getFloatByAttributeName(node, "z");
            }
            else if (nodeName == "hemisphere")
            {
                hemisphere.emplace();

                std::string roleString = processStringAttribute("role", node, true);
                roleString = simox::alg::to_lower(roleString);
                try
                {
                    hemisphere->role = RobotNodeHemisphere::RoleFromString(roleString);
                }
                catch (const std::out_of_range& e)
                {
                    THROW_VR_EXCEPTION("Invalid role in hemisphere joint: " << e.what())
                }

                switch (hemisphere->role)
                {
                    case RobotNodeHemisphere::Role::FIRST:
                        hemisphere->lever = getFloatByAttributeName(node, "lever");
                        hemisphere->theta0Rad =
                            simox::math::deg_to_rad(getFloatByAttributeName(node, "theta0"));
                        break;
                    case RobotNodeHemisphere::Role::SECOND:
                        break;
                }
            }
            else if (nodeName == "four_bar")
            {
                fourBarXmlInfo.emplace();

                std::string roleString = processStringAttribute("role", node, true);
                roleString = simox::alg::to_lower(roleString);
                try
                {
                    fourBarXmlInfo->role = RobotNodeFourBar::RoleFromString(roleString);
                }
                catch (const std::out_of_range& e)
                {
                    THROW_VR_EXCEPTION("Invalid role in four_bar joint: " << e.what())
                }

                const rapidxml::xml_node<>* dimensionsNode =
                    node->first_node("dimensions", 0, false);

                if (fourBarXmlInfo->role == RobotNodeFourBar::Role::ACTIVE)
                {
                    if (dimensionsNode == nullptr)
                    {
                        THROW_VR_EXCEPTION("Missing <dimensions> node for four_bar joint.");
                    }

                    fourBarXmlInfo->dimensions = four_bar::Joint::Dimensions{
                        .shank = getFloatByAttributeName(dimensionsNode, "shank"),
                        .p1 = getFloatByAttributeName(dimensionsNode, "p1"),
                        .p2 = getFloatByAttributeName(dimensionsNode, "p2"),
                        .p3 = getFloatByAttributeName(dimensionsNode, "p3")};
                }
            }
            else
            {
                THROW_VR_EXCEPTION("XML definition <"
                                   << nodeName << "> not supported in <Joint> tag of RobotNode <"
                                   << robotNodeName << ">." << endl);
            }

            node = node->next_sibling();
        }

        /*
        if (dhXMLNode)
        {
            // check for wrongly defined nodes
            THROW_VR_EXCEPTION_IF((prejointTransformNode || tmpXMLNodeAxis || tmpXMLNodeTranslation || postjointTransformNode), "DH specification can not be used together with Axis, TranslationDirection, PreJointTransform or PostJointTransform definitions in <Joint> tag of node " << robotNodeName << endl);
            processDHNode(dhXMLNode,dh);

        } else
        {
            dh.isSet = false;
            processTransformNode(prejointTransformNode, robotNodeName, preJointTransform);
            processTransformNode(postjointTransformNode, robotNodeName, postJointTransform);
            */
        if (jointType == "revolute")
        {
            if (scaleVisu)
            {
                VR_WARNING << "Ignoring ScaleVisualization in Revolute joint." << std::endl;
                scaleVisu = false;
            }

            if (tmpXMLNodeAxis)
            {
                axis[0] = getFloatByAttributeName(tmpXMLNodeAxis, "x");
                axis[1] = getFloatByAttributeName(tmpXMLNodeAxis, "y");
                axis[2] = getFloatByAttributeName(tmpXMLNodeAxis, "z");
            }
            else
            {
                // silently setting axis to (0,0,1)
                //VR_WARNING << "Joint '" << robotNodeName << "' without 'axis' tag. Setting rotation axis to (0,0,1)." << std::endl;
                axis << 0, 0, 1.0f;
                //THROW_VR_EXCEPTION("joint '" << robotNodeName << "' wrongly defined, expecting 'axis' tag." << endl);
            }
        }
        else if (jointType == "prismatic")
        {
            if (tmpXMLNodeTranslation)
            {
                translationDir[0] = getFloatByAttributeName(tmpXMLNodeTranslation, "x");
                translationDir[1] = getFloatByAttributeName(tmpXMLNodeTranslation, "y");
                translationDir[2] = getFloatByAttributeName(tmpXMLNodeTranslation, "z");
            }
            else
            {
                THROW_VR_EXCEPTION("Prismatic joint '"
                                   << robotNodeName
                                   << "' wrongly defined, expecting 'TranslationDirection' tag."
                                   << endl);
            }

            if (scaleVisu)
            {
                THROW_VR_EXCEPTION_IF(scaleVisuFactor.norm() == 0.0f, "Zero scale factor");
            }
        }
        else if (jointType == "hemisphere")
        {
        }

        //}

        RobotNodePtr robotNode;
        RobotNodeFactoryPtr robotNodeFactory = RobotNodeFactory::fromName(jointType, nullptr);
        if (robotNodeFactory)
        {
            /*if (dh.isSet)
            {
                robotNode = robotNodeFactory->createRobotNodeDH(robot, robotNodeName, visualizationNode, collisionModel, jointLimitLow, jointLimitHigh, jointOffset, dh, physics, rntype);
            } else
            {*/
            // create nodes that are not defined via DH parameters
            robotNode =
                robotNodeFactory->createRobotNode(robot,
                                                  robotNodeName,
                                                  visualizationNode,
                                                  collisionModel,
                                                  jointLimitLow.value_or(jointLimitLowDefault),
                                                  jointLimitHigh.value_or(jointLimitHighDefault),
                                                  jointOffset,
                                                  preJointTransform,
                                                  axis,
                                                  translationDir,
                                                  physics,
                                                  rntype);
            //}
        }
        else
        {
            THROW_VR_EXCEPTION("RobotNode of type " << jointType << " nyi..." << endl);
        }

        robotNode->setMaxVelocity(maxVelocity);
        robotNode->setMaxAcceleration(maxAcceleration);
        robotNode->setMaxDeceleration(maxDeceleration);
        robotNode->setMaxTorque(maxTorque);
        robotNode->setMaxJerk(maxJerk);
        robotNode->setLimitless(limitless);
        robotNode->setAllowJointLimitAvoidance(allowJointLimitAvoidance);

        robotNode->jointValue = initialvalue;

        if (robotNode->isJoint())
        {
            if (robotNode->jointValue < robotNode->jointLimitLo)
            {
                robotNode->jointValue = robotNode->jointLimitLo;
            }
            else if (robotNode->jointValue > robotNode->jointLimitHi)
            {
                robotNode->jointValue = robotNode->jointLimitHi;
            }
        }
        if (robotNode->isHemisphereJoint() and hemisphere.has_value())
        {
            RobotNodeHemispherePtr node = std::dynamic_pointer_cast<RobotNodeHemisphere>(robotNode);
            hemisphere->limitLoRadians = jointLimitLow;
            hemisphere->limitHiRadians = jointLimitHigh;
            node->setXmlInfo(hemisphere.value());
        }

        if (robotNode->isFourBarJoint() and fourBarXmlInfo.has_value())
        {
            auto node = std::dynamic_pointer_cast<RobotNodeFourBar>(robotNode);
            node->setXmlInfo(fourBarXmlInfo.value());
        }

        if (scaleVisu)
        {
            RobotNodePrismaticPtr rnPM = std::dynamic_pointer_cast<RobotNodePrismatic>(robotNode);
            if (rnPM)
            {
                rnPM->setVisuScaleFactor(scaleVisuFactor);
            }
        }

        VR_ASSERT(propagateJVName.size() == propagateJVFactor.size());

        for (size_t i = 0; i < propagateJVName.size(); i++)
        {
            robotNode->propagateJointValue(propagateJVName[i], propagateJVFactor[i]);
        }

        return robotNode;
    }

    RobotNodePtr
    RobotIO::processRobotNode(rapidxml::xml_node<char>* robotNodeXMLNode,
                              RobotPtr robo,
                              const std::string& basePath,
                              int& robotNodeCounter,
                              std::vector<std::string>& childrenNames,
                              std::vector<ChildFromRobotDef>& childrenFromRobot,
                              RobotDescription loadMode,
                              RobotNode::RobotNodeType rntype)
    {
        childrenFromRobot.clear();
        THROW_VR_EXCEPTION_IF(!robotNodeXMLNode, "NULL data in processRobotNode");

        // get name
        std::string robotNodeName = processNameAttribute(robotNodeXMLNode);

        if (robotNodeName.empty())
        {
            std::stringstream ss;
            ss << robo->getType() << "_Node_" << robotNodeCounter;
            robotNodeName = ss.str();
            robotNodeCounter++;
            VR_WARNING << "RobotNode definition expects attribute 'name'. Setting name to "
                       << robotNodeName << std::endl;
        }


        // visu data
        bool visuProcessed = false;
        //bool enableVisu = true;
        bool useAsColModel;

        // collision information
        bool colProcessed = false;

        VisualizationNodePtr visualizationNode = nullptr;
        CollisionModelPtr collisionModel = nullptr;
        RobotNodePtr robotNode;
        std::string visualizationModelXML;
        std::string collisionModelXML;
        SceneObject::Physics physics;
        bool physicsDefined = false;
        Eigen::Matrix4f transformMatrix = Eigen::Matrix4f::Identity();
        SceneObject::PrimitiveApproximation primitiveApproximation;

        rapidxml::xml_node<>* node = robotNodeXMLNode->first_node();
        rapidxml::xml_node<>* jointNodeXML = nullptr;

        std::vector<rapidxml::xml_node<>*> sensorTags;
        std::vector<GraspSetPtr> graspSets;

        while (node)
        {
            std::string nodeName = getLowerCase(node->name());

            if (nodeName == "visualization")
            {
                if (loadMode == eFull || loadMode == eFullVisAsCol)
                {
                    THROW_VR_EXCEPTION_IF(visuProcessed,
                                          "Two visualization tags defined in RobotNode '"
                                              << robotNodeName << "'." << endl);
                    visualizationNode =
                        processVisualizationTag(node, robotNodeName, basePath, useAsColModel);
                    visuProcessed = true;

                    if (useAsColModel && visualizationNode)
                    {
                        THROW_VR_EXCEPTION_IF(colProcessed,
                                              "Two collision tags defined in RobotNode '"
                                                  << robotNodeName << "'." << endl);
                        std::string colModelName = robotNodeName;
                        colModelName += "_VISU_ColModel";
                        // clone model
                        VisualizationNodePtr visualizationNodeClone = visualizationNode->clone();
                        // todo: ID?
                        collisionModel.reset(new CollisionModel(
                            visualizationNodeClone, colModelName, CollisionCheckerPtr()));
                        colProcessed = true;
                    }
                }
                else if (loadMode == eCollisionModel)
                {
                    VisualizationNodePtr visualizationNodeCM =
                        checkUseAsColModel(node, robotNodeName, basePath);

                    if (visualizationNodeCM)
                    {
                        useAsColModel = true;
                        THROW_VR_EXCEPTION_IF(colProcessed,
                                              "Two collision tags defined in RobotNode '"
                                                  << robotNodeName << "'." << endl);
                        std::string colModelName = robotNodeName;
                        colModelName += "_VISU_ColModel";
                        // todo: ID?
                        collisionModel.reset(new CollisionModel(
                            visualizationNodeCM, colModelName, CollisionCheckerPtr()));
                        colProcessed = true;
                    }
                }
                else if (loadMode == eStructureStore)
                {
                    std::stringstream ss;
                    ss << *node;
                    visualizationModelXML = ss.str();
                    //rapidxml::print(std::back_inserter(robotNode->visualizationModelXML), node, rapidxml::print_no_indenting);
                }
            }
            else if (nodeName == "collisionmodel")
            {
                if (loadMode == eFull || loadMode == eCollisionModel || loadMode == eFullVisAsCol)
                {
                    THROW_VR_EXCEPTION_IF(colProcessed,
                                          "Two collision tags defined in RobotNode '"
                                              << robotNodeName << "'." << endl);
                    collisionModel = processCollisionTag(node, robotNodeName, basePath);
                    colProcessed = true;
                }
                else if (loadMode == eStructureStore)
                {
                    std::stringstream ss;
                    ss << *node;
                    collisionModelXML = ss.str();
                }
            }
            else if (nodeName == "primitivemodel")
            {
                processPrimitiveModelTag(primitiveApproximation, node);
            }
            else if (nodeName == "child")
            {
                processChildNode(node, childrenNames);
            }
            else if (nodeName == "childfromrobot")
            {
                processChildFromRobotNode(node, robotNodeName, childrenFromRobot);
            }
            else if (nodeName == "joint")
            {
                THROW_VR_EXCEPTION_IF(jointNodeXML,
                                      "Two joint tags defined in RobotNode '" << robotNodeName
                                                                              << "'." << endl);
                jointNodeXML = node;
            }
            else if (nodeName == "physics")
            {
                THROW_VR_EXCEPTION_IF(physicsDefined,
                                      "Two physics tags defined in RobotNode '" << robotNodeName
                                                                                << "'." << endl);
                processPhysicsTag(node, robotNodeName, physics);
                physicsDefined = true;
            }
            else if (nodeName == "transform")
            {
                processTransformNode(robotNodeXMLNode, robotNodeName, transformMatrix);
            }
            else if (nodeName == "sensor")
            {
                sensorTags.push_back(node);
            }
            else if (nodeName == "graspset")
            {
                GraspSetPtr gs = processGraspSet(node, robotNodeName);
                THROW_VR_EXCEPTION_IF(!gs,
                                      "Invalid grasp set in '" << robotNodeName << "'." << endl);
                graspSets.push_back(gs);
            }
            else
            {
                THROW_VR_EXCEPTION("XML definition <"
                                   << nodeName << "> not supported in RobotNode <" << robotNodeName
                                   << ">." << endl);
            }

            node = node->next_sibling();
        }

        if (!colProcessed && visualizationNode && loadMode == eFullVisAsCol)
        {
            // Use collision model as visu model if not found
            std::string colModelName = robotNodeName;
            colModelName += "_VISU_ColModel";
            // clone model
            VisualizationNodePtr visualizationNodeClone = visualizationNode->clone();
            // todo: ID?
            collisionModel.reset(
                new CollisionModel(visualizationNodeClone, colModelName, CollisionCheckerPtr()));
        }

        //create joint from xml data
        robotNode = processJointNode(jointNodeXML,
                                     robotNodeName,
                                     robo,
                                     visualizationNode,
                                     collisionModel,
                                     physics,
                                     rntype,
                                     transformMatrix);
        robotNode->basePath = basePath;
        robotNode->visualizationModelXML = visualizationModelXML;
        robotNode->collisionModelXML = collisionModelXML;
        robotNode->setPrimitiveApproximation(primitiveApproximation);

        // process sensors
        for (auto& sensorTag : sensorTags)
        {
            processSensor(robotNode, sensorTag, loadMode, basePath);
        }

        for (const auto& graspSet : graspSets)
        {
            robotNode->addGraspSet(graspSet);
        }


        return robotNode;
    }

    VisualizationNodePtr
    RobotIO::checkUseAsColModel(rapidxml::xml_node<>* visuXMLNode,
                                const std::string& /*robotNodeName*/,
                                const std::string& basePath)
    {
        bool enableVisu = true;
        //bool coordAxis = false;
        //float coordAxisFactor = 1.0f;
        //std::string coordAxisText = "";
        //std::string visuCoordType = "";
        std::string visuFileType = "";
        std::string visuFile = "";
        rapidxml::xml_attribute<>* attr;
        VisualizationNodePtr visualizationNode;

        if (!visuXMLNode)
        {
            return visualizationNode;
        }

        attr = visuXMLNode->first_attribute("enable", 0, false);

        if (attr)
        {
            enableVisu = isTrue(attr->value());
        }

        if (enableVisu)
        {
            rapidxml::xml_node<>* visuFileXMLNode = visuXMLNode->first_node("file", 0, false);

            if (visuFileXMLNode)
            {
                attr = visuFileXMLNode->first_attribute("type", 0, false);
                THROW_VR_EXCEPTION_IF(!attr,
                                      "Missing 'type' attribute in <Visualization> tag." << endl);
                visuFileType = attr->value();
                getLowerCase(visuFileType);
                visuFile = processFileNode(visuFileXMLNode, basePath);
            }

            rapidxml::xml_node<>* useColModel =
                visuXMLNode->first_node("useascollisionmodel", 0, false);

            if (useColModel && visuFile != "")
            {
                VisualizationFactoryPtr visualizationFactory =
                    VisualizationFactory::fromName(visuFileType, nullptr);

                if (visualizationFactory)
                {
                    visualizationNode = visualizationFactory->getVisualizationFromFile(visuFile);
                }
                else
                {
                    VR_WARNING << "VisualizationFactory of type '" << visuFileType
                               << "' not present. Ignoring Visualization data." << std::endl;
                }
            }
        }

        return visualizationNode;
    }

    RobotPtr
    RobotIO::processRobot(rapidxml::xml_node<char>* robotXMLNode,
                          const std::string& basePath,
                          RobotDescription loadMode)
    {
        THROW_VR_EXCEPTION_IF(!robotXMLNode,
                              "No <Robot> tag in XML definition! base path = " << basePath);

        // process Attributes
        std::string robotRoot;
        RobotPtr robo;
        robo = processRobotAttributes(robotXMLNode, robotRoot);

        // process xml nodes
        std::map<RobotNodePtr, std::vector<ChildFromRobotDef>> childrenFromRobotFilesMap;
        std::vector<rapidxml::xml_node<char>*> robotNodeSetNodes;
        std::vector<rapidxml::xml_node<char>*> endeffectorNodes;

        NodeMapping nodeMapping;
        std::optional<HumanMapping> humanMapping;

        std::map<std::string, std::map<std::string, float>> configurations;

        std::map<std::string, std::vector<std::string>> attachments;

        processRobotChildNodes(robotXMLNode,
                               robo,
                               robotRoot,
                               basePath,
                               childrenFromRobotFilesMap,
                               robotNodeSetNodes,
                               endeffectorNodes,
                               nodeMapping,
                               humanMapping,
                               attachments,
                               configurations,
                               loadMode);

        // process childfromrobot tags
        for (auto& [node, childrenFromRobot] : childrenFromRobotFilesMap)
        {
            for (auto& child : childrenFromRobot)
            {
                std::filesystem::path filenameNew(child.filename);
                std::filesystem::path filenameBasePath(basePath);

                std::filesystem::path filenameNewComplete = filenameBasePath / filenameNew;

                try
                {
                    THROW_VR_EXCEPTION_IF(!std::filesystem::exists(filenameNewComplete),
                                          "File <" << filenameNewComplete.string()
                                                   << "> does not exist." << endl);
                }
                catch (...)
                {
                    THROW_VR_EXCEPTION("Error while processing file <"
                                       << filenameNewComplete.string() << ">." << endl);
                }

                RobotPtr r = loadRobot(filenameNewComplete.string(), loadMode);
                THROW_VR_EXCEPTION_IF(
                    !r,
                    "Could not add child-from-robot due to failed loading of robot from file"
                        << child.filename);
                RobotNodePtr root = r->getRootNode();
                THROW_VR_EXCEPTION_IF(!root,
                                      "Could not add child-from-robot. No root node in file"
                                          << child.filename);

                RobotNodePtr rootNew = root->clone(robo, true, node);
                THROW_VR_EXCEPTION_IF(!rootNew,
                                      "Clone failed. Could not add child-from-robot from file "
                                          << child.filename);

                std::vector<EndEffectorPtr> eefs;
                r->getEndEffectors(eefs);

                for (auto& eef : eefs)
                {
                    eef->clone(robo);
                }

                std::vector<RobotNodeSetPtr> nodeSets;
                r->getRobotNodeSets(nodeSets);

                for (auto& nodeSet : nodeSets)
                {
                    nodeSet->clone(robo);
                }

                // already performed in root->clone
                //node->attachChild(rootNew);
            }
        }

        { // handle attachments

            const auto robotNodes = robo->getRobotNodes();

            // extend children map with attachments
            for (const auto& [parentNodeName, childrenNodeNames] : attachments)
            {
                // find parent node
                const auto parentNodeIt = std::find_if(
                    robotNodes.begin(),
                    robotNodes.end(),
                    [&](const auto& robotNode) { return robotNode->getName() == parentNodeName; });

                if (parentNodeIt == robotNodes.end())
                {
                    THROW_VR_EXCEPTION("Robot node `"
                                       << parentNodeName
                                       << "` was defined as as parent node but is is not known!"
                                       << std::endl);
                }

                // add all children to the mapping
                const auto& parentNode = *parentNodeIt;

                for (const auto& childName : childrenNodeNames)
                {
                    robo->getRobotNode(childName)->initialize(parentNode);
                }
            }
        }

        for (auto& endeffectorNode : endeffectorNodes)
        {
            EndEffectorPtr eef = processEndeffectorNode(endeffectorNode, robo);
            robo->registerEndEffector(eef);
        }

        int rnsCounter = 0;
        for (auto& robotNodeSetNode : robotNodeSetNodes)
        {
            RobotNodeSetPtr rns =
                processRobotNodeSet(robotNodeSetNode, robo, robotRoot, rnsCounter);
            (void)rns;
        }

        {
            std::vector<RobotNodePtr> nodes;
            robo->getRobotNodes(nodes);
            RobotNodePtr rootNode = robo->getRootNode();
            for (RobotNodePtr& node : nodes)
            {
                if (not(node->getParent() or node == rootNode))
                {
                    VR_ERROR << "RobotNode '" << node->getName()
                             << "' is not connected to kinematic structure." << std::endl;
                }
            }
        }

        robo->registerNodeMapping(nodeMapping);

        if (humanMapping.has_value())
        {
            robo->registerHumanMapping(humanMapping.value());
        }

        for(const auto& [name, configuration]: configurations)
        {
            // VR_INFO << "Registering configuration `" << name << "`." << std::endl;
            robo->registerConfiguration(name, configuration);
        }

        return robo;
    }

    inline bool
    toBool(const std::string& strRepr) noexcept
    {
        try
        {
            const int passiveIntRepr = std::stoi(strRepr);
            return static_cast<bool>(passiveIntRepr);
        }
        catch (std::invalid_argument&)
        {
            // nothing to do here
        }

        return strRepr == "true";
    }

    RobotPtr
    RobotIO::processRobotAttributes(rapidxml::xml_node<char>* robotXMLNode, std::string& robotRoot)
    {
        std::string robotName;
        std::string robotType;
        bool passive{false}; // if true, robot joints will not be actuated

        // process attributes of robot
        rapidxml::xml_attribute<>* attr;
        attr = robotXMLNode->first_attribute("type", 0, false);

        if (!attr)
        {
            VR_WARNING << "Robot definition expects attribute 'type'" << std::endl;
            robotType = "not set";
        }
        else
        {
            robotType = attr->value();
        }

        // check name counter
        {
            std::scoped_lock lock(mutex);

            if (robot_name_counter.find(robotType) != robot_name_counter.end())
            {
                robot_name_counter[robotType] = robot_name_counter[robotType] + 1;
            }
            else
            {
                robot_name_counter[robotType] = 1;
            }
        }

        // STANDARD NAME
        attr = robotXMLNode->first_attribute("standardname", 0, false);

        if (!attr)
        {
            std::stringstream ss;
            {
                std::scoped_lock lock(mutex);

                if (robot_name_counter[robotType] == 1)
                {
                    ss << robotType; // first one
                }
                else
                {
                    ss << robotType << "_" << (robot_name_counter[robotType] - 1);
                }
            }
            robotName = ss.str();
        }
        else
        {
            robotName = attr->value();
        }

        // Root
        attr = robotXMLNode->first_attribute("rootnode", 0, false);
        THROW_VR_EXCEPTION_IF(!attr, "Robot definition needs attribute 'RootNode'");
        robotRoot = attr->value();


        attr = robotXMLNode->first_attribute("passive", 0, false);

        if (attr != nullptr)
        {
            const std::string passiveStrRep = attr->value();
            passive = toBool(passiveStrRep);

            // VR_INFO << "Robot is 'passive' according to config" << std::endl;
        }

        // build robot
        RobotPtr robo(new LocalRobot(robotName, robotType));
        /*attr = robotXMLNode->first_attribute("RadianToMMfactor", 0, false);
        if(attr)
        {
            robo->setRadianToMMfactor(atof(attr->value()));
        }*/

        if (passive)
        {
            robo->setPassive();
        }


        return robo;
    }

    void
    RobotIO::processRobotChildNodes(
        rapidxml::xml_node<char>* robotXMLNode,
        RobotPtr robo,
        const std::string& robotRoot,
        const std::string& basePath,
        std::map<RobotNodePtr, std::vector<ChildFromRobotDef>>& childrenFromRobotFilesMap,
        std::vector<rapidxml::xml_node<char>*>& robotNodeSetNodes,
        std::vector<rapidxml::xml_node<char>*>& endeffectorNodes,
        NodeMapping& nodeMapping,
        std::optional<HumanMapping>& humanMapping,
        std::map<std::string, std::vector<std::string>>& attachments,
        std::map<std::string, std::map<std::string, float>>& configurations,
        RobotDescription loadMode)
    {
        std::vector<RobotNodePtr> robotNodes;
        std::map<RobotNodePtr, std::vector<std::string>> childrenMap;
        RobotNodePtr rootNode;
        int robotNodeCounter = 0; // used for robotnodes without names

        //std::vector<rapidxml::xml_node<>* > robotNodeSetNodes;
        //std::vector<rapidxml::xml_node<>* > endeffectorNodes;
        rapidxml::xml_node<>* XMLNode = robotXMLNode->first_node(nullptr, 0, false);

        // allow nodes to be registered to parents

        while (XMLNode)
        {
            std::string nodeName_ = XMLNode->name();
            std::string nodeName = getLowerCase(XMLNode->name());

            if (nodeName == "robotnode" || nodeName == "jointnode" ||
                nodeName == "transformationnode" || nodeName == "bodynode" ||
                nodeName == "modelnode")
            {
                std::vector<ChildFromRobotDef> childrenFromRobot;
                std::vector<std::string> childrenNames;
                // check for type
                RobotNode::RobotNodeType rntype = RobotNode::Generic;

                if (nodeName == "jointnode")
                {
                    rntype = RobotNode::Joint;
                }

                if (nodeName == "bodynode")
                {
                    rntype = RobotNode::Body;
                }

                if (nodeName == "transformationnode")
                {
                    rntype = RobotNode::Transform;
                }

                RobotNodePtr n = processRobotNode(XMLNode,
                                                  robo,
                                                  basePath,
                                                  robotNodeCounter,
                                                  childrenNames,
                                                  childrenFromRobot,
                                                  loadMode,
                                                  rntype);

                if (!n)
                {
                    std::string failedNodeName = processNameAttribute(XMLNode);
                    THROW_VR_EXCEPTION("Failed to create robot node " << failedNodeName << endl);
                }
                else
                {
                    // double name check
                    for (auto& robotNode : robotNodes)
                    {
                        THROW_VR_EXCEPTION_IF((robotNode->getName() == n->getName()),
                                              "At least two RobotNodes with name <"
                                                  << n->getName()
                                                  << "> defined in robot definition");
                    }

                    childrenMap[n] = childrenNames;
                    robotNodes.push_back(n);
                    robo->registerRobotNode(n);

                    if (n->getName() == robotRoot)
                    {
                        rootNode = n;
                    }

                    if (!childrenFromRobot.empty())
                    {
                        childrenFromRobotFilesMap[n] = childrenFromRobot;
                    }
                }

                robotNodeCounter++;
            }
            else if (nodeName == "robotnodeset" || nodeName == "modelnodeset")
            {
                robotNodeSetNodes.push_back(XMLNode);
            }
            else if ("endeffector" == nodeName)
            {
                endeffectorNodes.push_back(XMLNode);
            }
            else if ("robotinfo" == nodeName)
            {
                //skip
            }
            else if ("nodemapping" == nodeName)
            {
                // VR_INFO << "Found robot node mapping";
                nodeMapping = processNodeMapping(XMLNode, robo);
            }
            else if ("humanmappings" == nodeName)
            {
                // VR_INFO << "Found human mappings";
                humanMapping = processHumanMapping(XMLNode, robo);
            }
            else if (nodeName == "attachment")
            {
                rapidxml::xml_node<>* parentNode = XMLNode->first_node("parent", 0, false);
                rapidxml::xml_node<>* childNode = XMLNode->first_node("child", 0, false);

                const std::string parent = processStringAttribute("name", parentNode, false);
                const std::string child = processStringAttribute("name", childNode, false);

                // implicit vector instantiation
                attachments[parent].push_back(child);
            }
            else if (nodeName == "configuration")
            {
                auto [name, configuration] = processConfigurationNode(XMLNode);

                THROW_VR_EXCEPTION_IF(configurations.count(name)> 0, "Configuration `" << name << "` specified multiple times!");
                configurations.emplace(name, configuration);
            }
            else
            {
                THROW_VR_EXCEPTION("XML node of type <"
                                   << nodeName_ << "> is not supported. Ignoring contents..."
                                   << endl);
            }

            XMLNode = XMLNode->next_sibling(nullptr, 0, false);
        }

        THROW_VR_EXCEPTION_IF(robotNodes.empty(), "No RobotNodes defined in Robot.");
        THROW_VR_EXCEPTION_IF(!rootNode, "Could not find root node <" << robotRoot << ">");

        if (!RobotFactory::initializeRobot(robo, robotNodes, childrenMap, rootNode))
        {
            THROW_VR_EXCEPTION("Error while initializing Robot" << endl);
        }
    }

    /**
     * This method parses the EndEffector which are child tags of the Robot tag.
     * Each EndEffector has a name, a base node, a list of static robot nodes and a list of actors.
     * Each actor itself consists of a list of robot nodes.
     *
     * The static parts and the actors are retrieved by delegating the processing
     * to RobotIO::processEndeffectorActorNode and RobotIO::processEndeffectorStaticNode.
     *
     * \return instance of VirtualRobot::EndEffector
     */
    EndEffectorPtr
    RobotIO::processEndeffectorNode(rapidxml::xml_node<char>* endeffectorXMLNode, RobotPtr robo)
    {
        std::string endeffectorName("");
        std::string baseNodeName;
        std::string tcpNodeName;
        std::string gcpNodeName;
        RobotNodePtr baseNode;
        RobotNodePtr tcpNode;
        RobotNodePtr gcpNode;

        // process attributes first
        rapidxml::xml_attribute<>* attr = endeffectorXMLNode->first_attribute();

        while (attr)
        {
            std::string attributeName = getLowerCase(attr->name());

            if ("name" == attributeName)
            {
                THROW_VR_EXCEPTION_IF(
                    !endeffectorName.empty(),
                    "Endeffector tag has more than one <name> tag. Value of the first one is: " +
                        endeffectorName);
                endeffectorName = attr->value();
                THROW_VR_EXCEPTION_IF(endeffectorName.empty(),
                                      "Endeffector tag does not specify a <name> tag.");
            }
            else if ("base" == attributeName)
            {
                THROW_VR_EXCEPTION_IF(
                    !baseNodeName.empty(),
                    "Endeffector tag has more than one <base> tag. Value of the first one is: " +
                        baseNodeName);
                baseNodeName = attr->value();
                THROW_VR_EXCEPTION_IF(baseNodeName.empty(),
                                      "Endeffector tag does not specify a <base> tag.");
                baseNode = robo->getRobotNode(baseNodeName);
                THROW_VR_EXCEPTION_IF(
                    !baseNode,
                    "base associated with <Endeffector> not available in the robot model.");
            }
            else if ("tcp" == attributeName)
            {
                THROW_VR_EXCEPTION_IF(
                    !tcpNodeName.empty(),
                    "Endeffector tag has more than one <tcp> tag. Value of the first one is: " +
                        tcpNodeName);
                tcpNodeName = attr->value();
                THROW_VR_EXCEPTION_IF(tcpNodeName.empty(),
                                      "Endeffector tag does not specify a <tcp> tag.");
                tcpNode = robo->getRobotNode(tcpNodeName);
                THROW_VR_EXCEPTION_IF(
                    !tcpNode,
                    "tcp associated with <Endeffector> not available in the robot model.");
            }
            else if ("gcp" == attributeName)
            {
                THROW_VR_EXCEPTION_IF(
                    !gcpNodeName.empty(),
                    "Endeffector tag has more than one <gcp> tag. Value of the first one is: " +
                        gcpNodeName);
                gcpNodeName = attr->value();
                THROW_VR_EXCEPTION_IF(gcpNodeName.empty(),
                                      "Endeffector tag does not specify a <gcp> tag.");
                gcpNode = robo->getRobotNode(gcpNodeName);
                THROW_VR_EXCEPTION_IF(
                    !gcpNode,
                    "gcp associated with <Endeffector> not available in the robot model.");
            }
            else
            {
                VR_WARNING << "Ignoring unknown attribute in EEF <" << endeffectorName
                           << "> definition:" << attributeName << std::endl;
            }

            attr = attr->next_attribute();
        }

        std::vector<RobotNodePtr> staticParts;
        std::vector<EndEffectorActorPtr> actors;
        std::vector<std::vector<RobotConfig::Configuration>> configDefinitions;
        std::vector<std::string> configNames;
        std::vector<std::string> tcpNames;
        rapidxml::xml_node<>* node = endeffectorXMLNode->first_node();

        while (node)
        {
            std::string nodeName = getLowerCase(node->name());

            if ("actor" == nodeName)
            {
                actors.push_back(processEndeffectorActorNode(node, robo));
            }
            else if ("static" == nodeName)
            {
                if (staticParts.empty())
                {
                    processEndeffectorStaticNode(node, robo, staticParts);
                }
                else
                {
                    VR_ERROR << "There should only be one <static> tag inside <endeffector> tags"
                             << std::endl;
                }
            }
            else if ("preshape" == nodeName)
            {
                bool cOK =
                    processConfigurationNodeList(node, configDefinitions, configNames, tcpNames);
                THROW_VR_EXCEPTION_IF(!cOK,
                                      "Invalid Preshape defined in robot's eef tag '"
                                          << nodeName << "'." << endl);
            }
            else
            {
                THROW_VR_EXCEPTION("XML tag <" << nodeName << "> not supported in endeffector <"
                                               << nodeName << ">");
            }

            node = node->next_sibling();
        }

        if (!tcpNode)
        {
            tcpNode = baseNode;
        }

        if (!gcpNode)
        {
            gcpNode = tcpNode;
        }

        EndEffectorPtr endEffector(
            new EndEffector(endeffectorName, actors, staticParts, baseNode, tcpNode, gcpNode));

        // create & register configs
        THROW_VR_EXCEPTION_IF(configDefinitions.size() != configNames.size(),
                              "Invalid Preshape definitions " << endl);

        for (size_t i = 0; i < configDefinitions.size(); i++)
        {
            RobotConfigPtr rc(new RobotConfig(robo, configNames[i], configDefinitions[i]));
            if (!tcpNames[i].empty())
            {
                rc->setTCP(tcpNames[i]);
            }
            endEffector->registerPreshape(rc);
        }

        return endEffector;
    }

    /**
     * This method processes the attributes and the children of an actor tag which
     * itself is a child of the endeffector tag.
     * First the name attribute is retrieved and afterwards the child nodes are
     * processed which make up the actor.
     */
    EndEffectorActorPtr
    RobotIO::processEndeffectorActorNode(rapidxml::xml_node<char>* endeffectorActorXMLNode,
                                         RobotPtr robo)
    {
        std::string actorName = processNameAttribute(endeffectorActorXMLNode);
        THROW_VR_EXCEPTION_IF(
            actorName.empty(),
            "<Actor> tag inside <Endeffector> does not specify a <name> attribute.");
        std::vector<EndEffectorActor::ActorDefinition> actors;
        processActorNodeList(endeffectorActorXMLNode, robo, actors);

        EndEffectorActorPtr actor(new EndEffectorActor(actorName, actors));
        return actor;
    }

    /**
     * This method processes the children of the static tag which
     * itself is a child of the endeffector tag.
     */
    void
    RobotIO::processEndeffectorStaticNode(rapidxml::xml_node<char>* endeffectorStaticXMLNode,
                                          RobotPtr robo,
                                          std::vector<RobotNodePtr>& staticNodesList)
    {
        processNodeList(endeffectorStaticXMLNode, robo, staticNodesList, false);
    }

    /**
     * This method processes the \p parentNode Tag and extracts a list of \<Node name="xyz" speed="0123" /\> tags.
     * All other child tags raise a VirtualRobot::VirtualRobotException.
     * The resulting nodes are stored in \p nodeList.
     *
     * If the parameter \p clearList is true all elements from \p nodeList are removed.
     */
    void
    RobotIO::processActorNodeList(rapidxml::xml_node<char>* parentNode,
                                  RobotPtr robot,
                                  std::vector<EndEffectorActor::ActorDefinition>& actorList,
                                  bool clearList /*= true*/)
    {
        if (clearList)
        {
            actorList.clear();
        }

        std::string parentName = processNameAttribute(parentNode, true);
        std::string speedname("direction");
        rapidxml::xml_node<>* node = parentNode->first_node();

        while (node)
        {
            std::string nodeName = getLowerCase(node->name());

            if (nodeName == "node")
            {
                EndEffectorActor::ActorDefinition actor;
                std::string nodeNameAttr = processNameAttribute(node, true);
                THROW_VR_EXCEPTION_IF(
                    nodeNameAttr.empty(),
                    "Missing name attribute for <Node> belonging to Robot node set " << parentName);
                actor.robotNode = robot->getRobotNode(nodeNameAttr);
                THROW_VR_EXCEPTION_IF(!actor.robotNode,
                                      "<node> tag with name '"
                                          << nodeNameAttr << "' not present in the current robot");
                actor.directionAndSpeed = processFloatAttribute(speedname, node, true);

                if (actor.directionAndSpeed == 0.0f)
                {
                    actor.directionAndSpeed = 1.0f;
                }

                actor.colMode = processEEFColAttributes(node, true);
                actorList.push_back(actor);
            }
            else
            {
                THROW_VR_EXCEPTION("XML definition <" << nodeName
                                                      << "> not supported in <Actor> with name "
                                                      << parentName);
            }

            node = node->next_sibling();
        }
    }

    EndEffectorActor::CollisionMode
    RobotIO::processEEFColAttributes(rapidxml::xml_node<char>* node,
                                     bool allowOtherAttributes /* = false */)
    {
        THROW_VR_EXCEPTION_IF(!node, "Can not process name attribute of NULL node" << endl);

        EndEffectorActor::CollisionMode result = EndEffectorActor::eNone;
        bool specified = false;
        std::string nodeNameAttr("");
        rapidxml::xml_attribute<char>* attr = node->first_attribute();

        while (attr)
        {
            std::string name = getLowerCase(attr->name());

            if ("considercollisions" == name)
            {
                std::string opt = getLowerCase(attr->value());
                specified = true;

                if (opt == "actors")
                {
                    result = EndEffectorActor::CollisionMode(result | EndEffectorActor::eActors);
                }
                else if (opt == "static")
                {
                    result = EndEffectorActor::CollisionMode(result | EndEffectorActor::eStatic);
                }
                else if (opt == "all")
                {
                    result = EndEffectorActor::eAll;
                }
                else if (opt == "none")
                {
                    result = EndEffectorActor::eNone;
                }
                else
                {
                    THROW_VR_EXCEPTION("<"
                                       << node->name()
                                       << "> considerCollisions attribute is unknowne: " << name);
                }
            }
            else
            {
                if (!allowOtherAttributes)
                {
                    THROW_VR_EXCEPTION("<" << node->name()
                                           << "> tag contains unknown attribute: " << attr->name());
                }
            }

            attr = attr->next_attribute();
        }

        // standard behavior: check collisions with all actors and static part of EEF
        if (!specified)
        {
            result = EndEffectorActor::eAll;
        }

        return result;
    }

    VirtualRobot::RobotPtr
    RobotIO::createRobotFromString(const std::string& xmlString,
                                   const std::string& basePath,
                                   RobotDescription loadMode)
    {
        // copy string content to char array
        char* y = new char[xmlString.size() + 1];
        strncpy(y, xmlString.c_str(), xmlString.size() + 1);

        VirtualRobot::RobotPtr robot;

        try
        {
            rapidxml::xml_document<char> doc; // character type defaults to char
            doc.parse<0>(y); // 0 means default parse flags
            rapidxml::xml_node<char>* robotXMLNode = doc.first_node("robot", 0, false);

            if (!robotXMLNode)
            {
                robotXMLNode = doc.first_node("model", 0, false);
            }

            robot = processRobot(robotXMLNode, basePath, loadMode);
        }
        catch (rapidxml::parse_error& e)
        {
            delete[] y;
            THROW_VR_EXCEPTION("Could not parse data in xml definition"
                               << endl
                               << "Error message:" << e.what() << endl
                               << "Position: " << endl
                               << e.where<char>() << endl);
            return RobotPtr();
        }
        catch (VirtualRobot::VirtualRobotException&)
        {
            delete[] y;
            // rethrow the current exception
            throw;
        }
        catch (std::exception& e)
        {
            delete[] y;
            THROW_VR_EXCEPTION("Error while parsing xml definition" << endl
                                                                    << "Error code:" << e.what()
                                                                    << endl);
            return RobotPtr();
        }
        catch (...)
        {
            delete[] y;
            THROW_VR_EXCEPTION("Error while parsing xml definition" << endl);
            return RobotPtr();
        }

        delete[] y;

        if (loadMode == RobotIO::eCollisionModel)
        {
            // use collision visualization to build main visualization
            robot->createVisualizationFromCollisionModels();
        }

        return robot;
    }

    VirtualRobot::RobotPtr
    RobotIO::loadRobot(const std::string& modelFile, RobotDescription loadMode)
    {
        std::string fullFile = modelFile;

        std::string fileType = fullFile.substr(fullFile.find_last_of(".") + 1);


        if (!RuntimeEnvironment::getDataFileAbsolute(fullFile))
        {
            VR_ERROR << "Could not open " + fileType + " file:" << modelFile << std::endl;
            return RobotPtr();
        }

        if (fileType == "xml")
        {
            std::ifstream in(fullFile.c_str());

            if (!in.is_open())
            {
                VR_ERROR << "Could not open XML file:" << fullFile << std::endl;
                return RobotPtr();
            }
            std::stringstream buffer;
            buffer << in.rdbuf();

            //i don't see big redunancies between the xml and the urdf branch.
            std::string robotXML(buffer.str());
            std::filesystem::path filenameBaseComplete(fullFile);
            std::filesystem::path filenameBasePath = filenameBaseComplete.parent_path();
            std::string basePath = filenameBasePath.string();

            in.close();

            VirtualRobot::RobotPtr res = createRobotFromString(robotXML, basePath, loadMode);

            if (!res)
            {
                VR_ERROR << "Error while parsing file " << fullFile << std::endl;
            }

            res->applyJointValues();
            res->setFilename(fullFile);
            return res;
        }
        else if (fileType == "urdf")
        {

            std::shared_ptr<RobotImporterFactory> factory =
                RobotImporterFactory::fromName("SimoxURDF", nullptr);
            if (!factory)
            {
                VR_ERROR << "Could not create RobotImporterFactory for SimoxURDF" << std::endl;
                return RobotPtr();
            }
            // to ensure that 3d model files can be loaded during converting we need to add the correct data path
            //#Question: Do we need this here as well? Is going up two directories still correct in the new context?
            std::filesystem::path tmppath = modelFile;
            tmppath = tmppath.parent_path();
            tmppath = tmppath / "/../..";
            std::string modelsBasePath = tmppath.generic_string();
            RuntimeEnvironment::addDataPath(modelsBasePath);

            //create VirtualRobot Object
            VirtualRobot::RobotPtr r = factory->loadFromFile(modelFile);

            return r;
        }
        else
        {
            std::cout << "File does not have correct file Type!" << std::endl;
            return RobotPtr();
        }
    }

    bool
    RobotIO::saveXML(RobotPtr robot,
                     const std::string& filename,
                     const std::string& basePath,
                     const std::string& modelDir,
                     bool storeEEF,
                     bool storeRNS,
                     bool storeSensors,
                     bool storeModelFiles)
    {
        THROW_VR_EXCEPTION_IF(!robot, "NULL data");


        std::filesystem::path p = simox::fs::remove_trailing_separator(basePath);
        std::filesystem::path fn = simox::fs::remove_trailing_separator(filename);
        std::filesystem::path pModelDir = simox::fs::remove_trailing_separator(modelDir);
        std::filesystem::path fnComplete = p / fn;
        std::filesystem::path modelDirComplete = p / pModelDir;

        if (std::filesystem::exists(modelDirComplete) &&
            !std::filesystem::is_directory(modelDirComplete))
        {
            VR_ERROR << "Could not create model directory (existing & !dir)  " << pModelDir.string()
                     << std::endl;
            return false;
        }

        if (!std::filesystem::is_directory(modelDirComplete))
        {
            if (!std::filesystem::create_directories(modelDirComplete))
            {
                VR_ERROR << "Could not create model dir  " << modelDirComplete.string()
                         << std::endl;
                return false;
            }
        }

        std::ofstream f(fnComplete.string().c_str());

        if (!f)
        {
            VR_ERROR << "Could not create file " << fnComplete.string() << std::endl;
            return false;
        }

        std::string xmlRob =
            robot->toXML(basePath, modelDir, storeEEF, storeRNS, storeSensors, storeModelFiles);
        f << xmlRob;
        f.close();

        if (storeModelFiles)
        {
            std::vector<RobotNodePtr> nodes = robot->getRobotNodes();

            for (auto& node : nodes)
            {
                node->saveModelFiles(modelDirComplete.string(), true);
            }
        }

        return true;
    }

    void
    RobotIO::saveMJCF(RobotPtr robot,
                      const std::string& filename,
                      const std::string& basePath,
                      const std::string& meshDir)
    {
        mujoco::RobotMjcf mjcf(robot);
        mjcf.setOutputPaths(std::filesystem::path(basePath) / filename, meshDir);
        mjcf.build(mujoco::WorldMountMode::FREE, mujoco::ActuatorType::MOTOR);
        mjcf.save();
    }


} // namespace VirtualRobot
