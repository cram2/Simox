/** @mainpage ColladaLight
 *
 * A light-weight library written in C++ for reading robot definitions from [COLLADA v1.5](http://www.khronos.org/collada) files.
 * It's main purpose is processing files created by the
 * [RobotEditor](https://github.com/StefanUlbrich/RobotEditor), a plugin for [Blender](http://www.blender.org) written in Python
 * that helps with the creation of robot models and finally exports it to COLLADA. For this reason, this library is specialized
 * to reading these files but should be easily modified to include more parts of the standard. For instance, all elements are expected
 * to be organized in libraries and referred to by instances rather than defining the element directly where it is needed.
 *
 * The base classes in [collada.h] (@ref collada.h) Collada::ColladaRobot and Collada::ColladaRobotNode fulfill the basic functions:
 * - Reading kinematic and dynamic constraints defined in `axis_info` elements.
 * - Reading the kinematic structure and initial values of joints
 * - In addition to standard COLLADA, position sensors created with the RobotEditor are placed on the model.
 *
 * The classes in [inventor.h](@ref inventor.h)---Collada::InventorRobot and Collada::InventorRobotNode---are derived from the base classes. They
 * translate the visuals defined in the file directly into [OpenInventor](http://de.wikipedia.org/wiki/Open_Inventor) object.
 * Therefore, it provide the additional functionality:
 * - Traversing the visual scene and prepare rendering its content.
 * - Reading and resolving additional weight distributions of the robot's body. (*Note* that this requires traversal of the visual scene graph. Although
 * this could be integrated into the the base classes, it is easier to integrate it at this place)
 * - In the future, it will provide a rudimentary simulation for robot kinematics.
 * Although the implementation is focused on OpenInventor, it should be very easy to adapt the classes to different render engines such as the free
 * [OpenSceneGraph](http://www.openscenegraph.org/),
 * [OGRE](http://www.ogre3d.org/) and [Panda3d](https://www.panda3d.org/) (*with Python/C++ support) libraries.
 *
 * The classes in [simox.h](@ref simox.h) build up a factory for creating robot nodes and complete robots for the [Simox](http://simox.sourceforge.net)
 * robot simulation framework that provides
 * a complete robot simulation environment. They are derived from the inventor classes and uses their mechanisms to travese the visual scene graph.
 *
 * Internally, ColladaLight relies (and includes) the excellent [pugixml](http://pugixml.org) lightweight XML library. To a small amount, it is necessary to
 * interact with XML-Elements when modifying this library. So make sure to browse over its [quick start guide](http://pugixml.googlecode.com/svn/tags/latest/docs/quickstart.html)
 * (Especially how to access text elements, children, and a rough idea how XPATH works). Further, this library relies heavily on the [boost](http://boost.org) libraries.
 */

#include "collada.h"

#include <cassert>
#include <cmath>
#include <iostream>
#include <map>
#include <vector>

#include <SimoxUtility/algorithm/string/string_tools.h>

#define IN_ARTICULATED_SYSTEMS "//library_articulated_systems"
#define IN_KINEMATICS_MODELS "//library_kinematics_models"
#define IN_JOINTS "//library_joints"
#define IN_PHYSICS_SCENES "//library_physics_scenes"
#define IN_PHYSICS_MODELS "//library_physics_models"
#define IN_VISUAL_SCENES "//library_visual_scenes"

#define IN_ARTICULATED_SYSTEMS_AND_KINEMATICS_MODELS_AND_JOINTS                                    \
    "//library_articulated_systems|//library_kinematics_models|//library_joints"

#define TARGETING_ATTRIBUTES "./@url|@source|@target"

namespace Collada
{

    std::vector<int>
    getIntVector(std::string text)
    {
        simox::alg::trim(text);
        std::vector<int> result;
        std::vector<std::string> splitted = simox::alg::split(text, "\t ");
        for (std::string const& number : splitted)
        {
            result.push_back(std::stoi(number));
        }

        return result;
    }

    std::vector<float>
    getFloatVector(std::string text)
    {
        simox::alg::trim(text);
        std::vector<float> result;
        std::vector<std::string> splitted = simox::alg::split(text, "\t ");
        for (std::string const& number : splitted)
        {
            result.push_back(std::stof(number));
        }

        return result;
    }

    std::ostream&
    operator<<(std::ostream& os, const ColladaRobotNode& node)
    {
        //std::cout
        os << "ColladaRobotNode: " << node.name << std::endl;
        return os;
    }

    pugi::xml_node
    resolveURL(pugi::xml_node node, std::string root)
    {
        pugi::xpath_variable_set vars;
        assert(node);
        vars.set("root", node.select_nodes(root.c_str()));
        vars.set("url", node.select_nodes(TARGETING_ATTRIBUTES, &vars));
        pugi::xml_node result =
            node.select_single_node("$root//*[@id=substring($url,2)]", &vars).node();
        assert(result);
        return result;
    }

    pugi::xml_node
    resolveSIDREF(pugi::xml_node node, std::string sidref, std::string root)
    {
        pugi::xpath_variable_set vars;
        vars.set("root", node.select_nodes(root.c_str()));
        std::vector<std::string> v = simox::alg::split(sidref, "/");
        vars.set("name", v.front().c_str());
        vars.set("context", node.select_nodes("$root//*[@id=$name]", &vars));

        for (std::vector<std::string>::const_iterator it = v.begin() + 1; it != v.end(); it++)
        {
            //cout << *it <<":" ;
            vars.set("name", it->c_str());
            vars.set("context", node.select_nodes("$context//*[@sid=$name]", &vars));
            vars.set(
                "context",
                node.select_nodes("$context | $root//*[@id=substring($context/@url,2)]", &vars));
        }

        pugi::xml_node result = node.select_single_node("$context", &vars).node();
        assert(result);
        return result;
    }

    struct ModelWalker : ColladaWalker
    {
        ModelWalker(StructureMap _structureMap) : ColladaWalker(_structureMap, XmlMap())
        {
        }
#ifdef COLLADA_IMPORT_USE_SENSORS
        void
        setSensorMap(XmlVectorMap _sensorMap)
        {
            this->sensorMap = _sensorMap;
        }
#endif
        bool
        for_each(pugi::xml_node& node) override
        {
            if (depth() + 1 > (int)stack.size())
            {
                stack.push_back(XmlNodeVector());
            }

            while (depth() + 1 < (int)stack.size())
            {
                stack.pop_back();
            }

            // The parent stack gets updated only even values of depth(). So it is half the size of stack:
            while (floor((depth() + 1) / 2.0) < parents.size())
            {
                parents.pop_back();
            }


            if (node.name() == std::string("attachment_full"))
            {
                pugi::xml_node instance =
                    resolveSIDREF(node, node.attribute("joint").value(), IN_KINEMATICS_MODELS);
                pugi::xml_node joint = resolveURL(instance, IN_JOINTS);
                ColladaRobotNodePtr robotNode = structureMap[joint];

                if (depth() > 0) // Not the root joint
                {
                    //robotNode->preJoint = stack[depth()-1];// QUICKFIX -- LOGIC ERROR IN ROBOT EDITOR
                    parents.back()->children.push_back(robotNode);
                    robotNode->parent = parents.back();
                }

                //BOOST_FOREACH(pugi::xml_node n, stack[depth()]) std::cout << "Ignored: " << n.name() << std::endl;

                parents.push_back(robotNode);
            }
            else
            {
                if (node.name() == std::string("link"))
                {
                    parents.back()->preJoint =
                        stack[depth()]; // QUICKFIX -- LOGIC ERROR IN ROBOT EDITOR
#ifdef COLLADA_IMPORT_USE_SENSORS

                    if (sensorMap.count(node))
                    {
                        parents.back()->sensors = sensorMap[node];
                    }

#endif
                }
                else // Can only be translation or rotation
                {
                    stack[depth()].push_back(node);
                }
            }

            return true;
        } //for_each

        ColladaRobotNodeSet parents;
        std::vector<XmlNodeVector> stack;
        TraversalStack<XmlNodeVector> travstack;
#ifdef COLLADA_IMPORT_USE_SENSORS
        XmlVectorMap sensorMap;
#endif
    };

    void
    ColladaRobot::parse(std::string fileName)
    {

        pugi::xml_document doc;

        pugi::xml_parse_result result = doc.load_file(fileName.c_str());

        if (!result)
        {

            std::cout << "Could not load:" << result.description() << std::endl;
        }


        std::cout << "Version: " << doc.child("COLLADA").attribute("version").as_float()
                  << std::endl;
        std::cout << "Scene: "
                  << doc.child("COLLADA")
                         .child("scene")
                         .child("instance_kinematics_scene")
                         .attribute("url")
                         .as_string()
                  << std::endl;

        pugi::xml_node collada = doc.child("COLLADA");
        pugi::xml_node scene = collada.child("scene");
        assert(scene);


        // There is probably ean error here ...  the // before scene should not be required
        //  std::cout << "url:" << collada.select_nodes("scene/instance_kinematics_scene/@url").begin()->node().value() << "KS: " << kinematics_scene.name() << std::endl;

        pugi::xml_node kinematics_scene =
            scene
                .select_single_node("//library_kinematics_scenes/kinematics_scene[@id=substring(//"
                                    "scene/instance_kinematics_scene/@url,2)]")
                .node();

        pugi::xpath_variable_set vars;
        vars.set("kinematicsScene",
                 scene.select_nodes("//library_kinematics_scenes/kinematics_scene[@id=substring(//"
                                    "scene/instance_kinematics_scene/@url,2)]"));

#ifdef COLLADA_IMPORT_USE_SENSORS
        XmlVectorMap sensorMap;
#endif

        // First step: gather all joint_axis_info
        std::map<pugi::xml_node, std::pair<pugi::xml_node, pugi::xml_node>> jointMap;
        std::vector<pugi::xml_node> kinematicsModels;

        for (pugi::xpath_node instance :
             collada.select_nodes("$kinematicsScene/instance_articulated_system", &vars))
        {
            pugi::xml_node motion_articulated_system =
                resolveURL(instance.node(), IN_ARTICULATED_SYSTEMS);
            for (pugi::xpath_node motion_axis_info :
                 motion_articulated_system.select_nodes(".//axis_info"))
            {
                pugi::xml_node kinematics_axis_info =
                    resolveSIDREF(collada,
                                  motion_axis_info.node().attribute("axis").value(),
                                  IN_ARTICULATED_SYSTEMS); /*TODO restrict research*/
                pugi::xml_node joint_axis =
                    resolveSIDREF(collada,
                                  kinematics_axis_info.attribute("axis").value(),
                                  IN_ARTICULATED_SYSTEMS_AND_KINEMATICS_MODELS_AND_JOINTS);
                jointMap[joint_axis] =
                    std::make_pair(motion_axis_info.node(), kinematics_axis_info);
            }

#ifdef COLLADA_IMPORT_USE_SENSORS
            // Get sensors
            for (pugi::xpath_node sensor :
                 motion_articulated_system.select_nodes(".//extra[@type='attach_sensor']"))
            {
                pugi::xml_node link = resolveSIDREF(
                    collada,
                    sensor.node().select_single_node(".//frame_origin/@link").attribute().value(),
                    IN_KINEMATICS_MODELS);
                sensorMap[link].push_back(sensor.node());
            }
#endif

            pugi::xml_node kinematics_articulated_system = resolveURL(
                motion_articulated_system.select_single_node(".//instance_articulated_system")
                    .node(),
                IN_ARTICULATED_SYSTEMS);
            kinematicsModels.push_back(resolveURL(
                kinematics_articulated_system.select_single_node(".//instance_kinematics_model")
                    .node(),
                IN_KINEMATICS_MODELS));
        }

        // Physics (center of mass, inertia and collision model)
        XmlMap physicsMap;
        for (pugi::xpath_node node : scene.select_nodes("./instance_physics_scene"))
        {
            pugi::xml_node physicsScene = resolveURL(node.node(), IN_PHYSICS_SCENES);
            for (pugi::xpath_node instance :
                 physicsScene.select_nodes("instance_physics_model/instance_rigid_body"))
            {
                pugi::xml_node rigidBody = resolveSIDREF(
                    collada, instance.node().attribute("body").value(), IN_PHYSICS_MODELS);
                pugi::xml_node visualNode = resolveURL(instance.node(), IN_VISUAL_SCENES);
                physicsMap[visualNode] = rigidBody;
            }
        }

        // Now get all joint_axes (these correspond to robot nodes) and connect them to the axis info (done via previous std::maps)
        StructureMap visualizationMap;
        StructureMap structureMap;
        pugi::xpath_node_set bindJointAxes =
            doc.select_nodes("COLLADA/scene/instance_kinematics_scene/bind_joint_axis");

        // pugixml had to be pathed to make xpath_node_set work with boost. non-inversive (verbose) alternative:
        // BOOST_FOREACH(xpath_node node, std::make_pair(set.begin(), set.end()))

        for (pugi::xpath_node bind : bindJointAxes)
        {
            ColladaRobotNodePtr robotNode = this->robotNodeFactory();
            {
                // Kinematic and dynamic properties
                vars.set("param", bind.node().select_nodes("axis/param"));
                pugi::xml_node param =
                    kinematics_scene
                        .select_single_node(".//bind[@symbol=$param/text()]/param", &vars)
                        .node();
                pugi::xml_node newparam_motion =
                    resolveSIDREF(collada, param.attribute("ref").value(), IN_ARTICULATED_SYSTEMS);
                pugi::xml_node newparam_kinematics = resolveSIDREF(
                    collada, newparam_motion.child_value("SIDREF"), IN_ARTICULATED_SYSTEMS);

                robotNode->joint_axis =
                    resolveSIDREF(collada,
                                  newparam_kinematics.child_value("SIDREF"),
                                  IN_ARTICULATED_SYSTEMS_AND_KINEMATICS_MODELS_AND_JOINTS);

                if (std::string("revolute") == robotNode->joint_axis.name())
                {
                    robotNode->type = ColladaRobotNode::eREVOLUTE;
                }
                else if (std::string("prismatic") == robotNode->joint_axis.name())
                {
                    robotNode->type = ColladaRobotNode::ePRISMATIC;
                }

                robotNode->name = robotNode->joint_axis.parent().attribute("name").value();
                //cout << "Name: " << robotNode->name << std::endl;

                robotNode->motion_info = jointMap[robotNode->joint_axis].first;
                robotNode->kinematics_info = jointMap[robotNode->joint_axis].second;
                //cout << robotNode->kinematics_info.child("limits").child("min").child_value("float") << std::endl;

                robotNodeSet.push_back(robotNode);

                std::string visualizationTarget = bind.node().attribute("target").value();
                visualizationMap[resolveSIDREF(collada, visualizationTarget, IN_VISUAL_SCENES)] =
                    robotNode;
                //cout << "Vizualization: " << visualizationTarget << "," << resolveSIDREF(collada,visualizationTarget,IN_VISUAL_SCENES).name() << std::endl;
                //cout << robotNode->joint_axis.parent().name() << std::endl;
                structureMap[robotNode->joint_axis.parent()] = robotNode;
            }
            {
                // initial value
                vars.set("param", bind.node().select_nodes("value/param"));
                pugi::xml_node param =
                    kinematics_scene
                        .select_single_node(".//bind[@symbol=$param/text()]/param", &vars)
                        .node();
                pugi::xml_node newparam_motion =
                    resolveSIDREF(collada, param.attribute("ref").value(), IN_ARTICULATED_SYSTEMS);
                pugi::xml_node newparam_kinematics = resolveSIDREF(
                    collada, newparam_motion.child_value("SIDREF"), IN_ARTICULATED_SYSTEMS);
                robotNode->value = std::stof(newparam_kinematics.child_value("float"));
            }
        }

        // Now optain the kinematic structure
        for (pugi::xml_node model : kinematicsModels)
        {
            std::cout << model.name() << std::endl;
            ModelWalker modelWalker(structureMap);
#ifdef COLLADA_IMPORT_USE_SENSORS
            modelWalker.setSensorMap(sensorMap);
#endif
            for (pugi::xpath_node link : model.select_nodes("./technique_common/link"))
            {
                link.node().traverse(modelWalker);
            }
        }

        // Parse the visual scene.
        assert(scene);
        pugi::xml_node visualScene =
            resolveURL(scene.child("instance_visual_scene"), IN_VISUAL_SCENES);
        ColladaWalkerPtr walker = this->visualSceneWalkerFactory(visualizationMap, physicsMap);
        visualScene.traverse(*walker);

        // Parse the collision models
        for (ColladaRobotNodePtr node : robotNodeSet)
        {
            for (pugi::xml_node body : node->rigidBodies)
            {
                this->addCollisionModel(node, body);
            }
        }

        // Initialize the nodes.
        for (ColladaRobotNodePtr node : robotNodeSet)
        {
            node->initialize();
        }
    }

    ColladaRobotNodePtr
    ColladaRobot::getRoot()
    {
        for (ColladaRobotNodePtr node : robotNodeSet)
        {
            if (!node->parent)
            {
                return node;
            }
        }

        return ColladaRobotNodePtr();
    }

    ColladaRobotNodePtr
    ColladaRobot::getNode(std::string name)
    {
        for (ColladaRobotNodePtr node : robotNodeSet)
        {
            if (name.compare(node->name) == 0)
            {
                return node;
            }
        }

        return ColladaRobotNodePtr();
    }

    ColladaRobotNodeSet
    ColladaRobot::getNodeSet()
    {
        return this->robotNodeSet;
    }

} // namespace Collada
