
#include "ColladaSimox.h"

#include <iostream>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <VirtualRobot/CollisionDetection/CollisionModel.h>
#include <VirtualRobot/Nodes/PositionSensor.h>
#include <VirtualRobot/Nodes/RobotNodeFixedFactory.h>
#include <VirtualRobot/Nodes/RobotNodePrismatic.h>
#include <VirtualRobot/Nodes/RobotNodePrismaticFactory.h>
#include <VirtualRobot/Nodes/RobotNodeRevolute.h>
#include <VirtualRobot/Nodes/RobotNodeRevoluteFactory.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RobotFactory.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationNode.h>

using namespace std;

#define DBG_NODE(NAME) (this->name.compare(NAME) == 0)

namespace Collada
{

    Eigen::Matrix4f
    getTransform(pugi::xml_node node, float scaleFactor = 1.0)
    {
        Eigen::Transform<float, 3, Eigen::Affine> t;
        std::vector<float> v = Collada::getFloatVector(node.child_value());

        if (std::string("rotate").compare(node.name()) == 0)
        {
            t = Eigen::AngleAxisf(v[3], Eigen::Vector3f(v[0], v[1], v[2]));
        }
        else if (std::string("translate").compare(node.name()) == 0)
        {
            t = Eigen::Translation3f(v[0] * scaleFactor, v[1] * scaleFactor, v[2] * scaleFactor);
        }
        else if (std::string("matrix").compare(node.name()) == 0)
        {
            t.matrix() << v[0], v[1], v[2], v[3] * scaleFactor, v[4], v[5], v[6],
                v[7] * scaleFactor, v[8], v[9], v[10], v[11] * scaleFactor, v[12], v[13], v[14],
                v[15];
        }

        return t.matrix();
    }

    Eigen::Vector3f
    getVector3f(pugi::xml_node node)
    {
        vector<float> v = getFloatVector(node.child_value());
        return Eigen::Vector3f(v[0], v[1], v[2]);
    }

    ColladaSimoxRobot::ColladaSimoxRobot(float scaleFactor) : scaleFactor(scaleFactor)
    {
        this->simoxRobot =
            VirtualRobot::RobotPtr(new VirtualRobot::LocalRobot("unnamed", "unknown type"));
        root = new SoSeparator;
        root->ref();
    }

    void
    ColladaSimoxRobot::initialize()
    {
        this->simoxRobot->setName(this->name);
        // How to initialize the robot?
        vector<VirtualRobot::RobotNodePtr> simoxRobotNodeSet;

        map<VirtualRobot::RobotNodePtr, vector<string>> childrenMap;
        VirtualRobot::RobotNodePtr root;

        std::vector<VirtualRobot::RobotNodePtr> simoxRevoluteJoints;
        std::vector<VirtualRobot::RobotNodePtr> simoxPrismaticJoints;

        for (ColladaRobotNodePtr node : this->robotNodeSet)
        {
            simoxRobotNodeSet.push_back(
                dynamic_pointer_cast<ColladaSimoxRobotNode>(node)->simoxRobotNode);

            if (node->type == ColladaRobotNode::eREVOLUTE)
            {
                simoxRevoluteJoints.push_back(
                    dynamic_pointer_cast<ColladaSimoxRobotNode>(node)->simoxRobotNode);
            }

            if (node->type == ColladaRobotNode::ePRISMATIC)
            {
                simoxPrismaticJoints.push_back(
                    dynamic_pointer_cast<ColladaSimoxRobotNode>(node)->simoxRobotNode);
            }

            vector<string> children;
            for (ColladaRobotNodePtr child : node->children)
                children.push_back(child->name);

            childrenMap[dynamic_pointer_cast<ColladaSimoxRobotNode>(node)->simoxRobotNode] =
                children;

            if (!node->parent)
            {
                root = dynamic_pointer_cast<ColladaSimoxRobotNode>(node)->simoxRobotNode;
            }
        }

        VirtualRobot::RobotFactory::initializeRobot(
            this->simoxRobot, simoxRobotNodeSet, childrenMap, root);
        VirtualRobot::RobotNodeSet::createRobotNodeSet(this->simoxRobot,
                                                       "All",
                                                       simoxRobotNodeSet,
                                                       VirtualRobot::RobotNodePtr(),
                                                       VirtualRobot::RobotNodePtr(),
                                                       true);

        if (!simoxRevoluteJoints.empty())
        {
            VirtualRobot::RobotNodeSet::createRobotNodeSet(this->simoxRobot,
                                                           "Joints_Revolute",
                                                           simoxRevoluteJoints,
                                                           VirtualRobot::RobotNodePtr(),
                                                           VirtualRobot::RobotNodePtr(),
                                                           true);
        }

        if (!simoxPrismaticJoints.empty())
        {
            VirtualRobot::RobotNodeSet::createRobotNodeSet(this->simoxRobot,
                                                           "Joints_Prismatic",
                                                           simoxPrismaticJoints,
                                                           VirtualRobot::RobotNodePtr(),
                                                           VirtualRobot::RobotNodePtr(),
                                                           true);
        }


        for (ColladaRobotNodePtr const& node : this->robotNodeSet)
        {
            (void)node;
            //dynamic_pointer_cast<ColladaSimoxRobotNode>(node)->simoxRobotNode->setJointValue(node->value);
        }
    }

    ColladaSimoxRobot::~ColladaSimoxRobot()
    {
        root->unref();
    }

    ColladaSimoxRobotNode::ColladaSimoxRobotNode(VirtualRobot::RobotPtr simoxRobot,
                                                 float scaleFactor) :
        simoxRobot(simoxRobot), scaleFactor(scaleFactor)
    {
        visualization = new SoSeparator;
        visualization->ref();
        collisionModel = new SoSeparator;
        collisionModel->ref();
    }

    ColladaSimoxRobotNode::~ColladaSimoxRobotNode()
    {
        visualization->unref();
        collisionModel->unref();
    }

    void
    ColladaSimoxRobotNode::initialize()
    {
        VirtualRobot::RobotNodeFactoryPtr revoluteNodeFactory =
            VirtualRobot::RobotNodeFactory::fromName(
                VirtualRobot::RobotNodeRevoluteFactory::getName(), NULL);
        VirtualRobot::RobotNodeFactoryPtr prismaticNodeFactory =
            VirtualRobot::RobotNodeFactory::fromName(
                VirtualRobot::RobotNodePrismaticFactory::getName(), NULL);
        float jointLimitLow = std::stof(
            this->kinematics_info.select_single_node("limits/min/float").node().child_value());
        float jointLimitHigh = std::stof(
            this->kinematics_info.select_single_node("limits/max/float").node().child_value());
        float acceleration = std::stof(
            this->motion_info.select_single_node("acceleration/float").node().child_value());
        float torque =
            std::stof(this->motion_info.select_single_node("jerk/float").node().child_value());
        float velocity =
            std::stof(this->motion_info.select_single_node("speed/float").node().child_value());
        float deceleration = std::stof(
            this->motion_info.select_single_node("deceleration/float").node().child_value());
        Eigen::Vector3f axis = getVector3f(this->joint_axis.child("axis"));
        float jointOffset = 0.0;
        Eigen::Matrix4f preJointTransformation = Eigen::Matrix4f::Identity();


        for (pugi::xml_node node : this->preJoint)
        {
            preJointTransformation = preJointTransformation * getTransform(node, scaleFactor);
        }

        if (DBG_NODE("RRKnee_Joint"))
        {
            std::cout << "Node: " << this->name;
            std::cout << jointLimitLow << "," << jointLimitHigh << "," << acceleration << ","
                      << deceleration << "," << velocity << "," << torque << std::endl;
            std::cout << this->joint_axis.name() << std::endl;
            preJointTransformation = Eigen::Matrix4f::Identity();
            for (pugi::xml_node node : this->preJoint)
            {
                std::cout << getTransform(node) << std::endl;
                preJointTransformation = preJointTransformation * getTransform(node, scaleFactor);
            }
            std::cout << preJointTransformation << std::endl;
        }

        /* Compute bounding box for debug reasons */
        //this->visualizeBoundingBox();

        VirtualRobot::VisualizationNodePtr visualizationNode(
            new VirtualRobot::CoinVisualizationNode(this->visualization));
        //cout << "node " << this->name << "#Faces" << visualizationNode->getNumFaces() << std::endl;
        VirtualRobot::CollisionModelPtr
            collisionModel; //(new VirtualRobot::CollisionModel(visualizationNode));

        // Check for rigid body dynamics and collision models.
        VirtualRobot::SceneObject::Physics physics;

        if (!rigidBodies.empty())
        {
            //assert(rigidBodies.size()==1);
            pugi::xml_node technique = rigidBodies[0].child("technique_common");
            float mass = std::stof(technique.child("mass").child_value());
            Eigen::Matrix4f massFrameTransformation = Eigen::Matrix4f::Identity();
            for (pugi::xpath_node trafo : technique.select_nodes(".//mass_frame/*"))
            {
                massFrameTransformation =
                    massFrameTransformation * getTransform(trafo.node(), 1000.0f);
            }
            Eigen::Matrix3f inertia = Eigen::Matrix3f::Identity();
            inertia./*block(0,0,3,3).*/ diagonal() = getVector3f(technique.child("inertia"));
            Eigen::Vector3f com = massFrameTransformation.block(0, 3, 3, 1);

            VirtualRobot::VisualizationNodePtr collisionNode(
                new VirtualRobot::CoinVisualizationNode(this->collisionModel));
            collisionModel.reset(new VirtualRobot::CollisionModel(collisionNode));
            //            std::cout << "Physics : " << name << endl << massFrameTransformation << endl << scaleFactor << massFrameTransformation.block<3,3>(0,0)*inertia << endl << mass << endl << com << std::endl;

            physics.comLocation = VirtualRobot::SceneObject::Physics::eCustom;
            physics.localCoM = com;
            physics.massKg = mass;
            physics.inertiaMatrix = massFrameTransformation.block<3, 3>(0, 0) * inertia;

            if (this->name.compare("RRKnee_joint") == 0)
            {
                std::cout << "Physics RRKnee_Joint: " << massFrameTransformation << endl
                          << scaleFactor << massFrameTransformation.block<3, 3>(0, 0) * inertia
                          << endl
                          << mass << endl
                          << com << std::endl;
            }
        }

        if (this->type == ColladaRobotNode::eREVOLUTE)
        {
            jointLimitLow = float(jointLimitLow / 180.0 * M_PI);
            jointLimitHigh = float(jointLimitHigh / 180.0 * M_PI);
            this->simoxRobotNode = revoluteNodeFactory->createRobotNode(simoxRobot,
                                                                        this->name,
                                                                        visualizationNode,
                                                                        collisionModel,
                                                                        jointLimitLow,
                                                                        jointLimitHigh,
                                                                        jointOffset,
                                                                        preJointTransformation,
                                                                        axis,
                                                                        Eigen::Vector3f::Zero(),
                                                                        physics);
        }
        else if (this->type == ColladaRobotNode::ePRISMATIC)
        {
            jointLimitLow = jointLimitLow * scaleFactor;
            jointLimitHigh = jointLimitHigh * scaleFactor;
            this->value *= scaleFactor;
            this->simoxRobotNode = prismaticNodeFactory->createRobotNode(simoxRobot,
                                                                         this->name,
                                                                         visualizationNode,
                                                                         collisionModel,
                                                                         jointLimitLow,
                                                                         jointLimitHigh,
                                                                         jointOffset,
                                                                         preJointTransformation,
                                                                         Eigen::Vector3f::Zero(),
                                                                         axis,
                                                                         physics);
        }

        if (!this->simoxRobotNode)
        {
            std::cout << "Node " << this->name << " not Created" << std::endl;
        }

#ifdef COLLADA_IMPORT_USE_SENSORS
        for (pugi::xml_node sensor : this->sensors)
        {
            string sensorName = sensor.attribute("name").value();
            Eigen::Matrix4f sensorTransformation = Eigen::Matrix4f::Identity();
            for (pugi::xpath_node trafo : sensor.select_nodes(".//frame_origin/*"))
            {
                sensorTransformation =
                    sensorTransformation * getTransform(trafo.node(), scaleFactor);
            }
            VirtualRobot::VisualizationNodePtr visualizationNode;
            VirtualRobot::PositionSensorPtr positionSensor(new VirtualRobot::PositionSensor(
                this->simoxRobotNode, sensorName, visualizationNode, sensorTransformation));
            this->simoxRobotNode->registerSensor(positionSensor);
        }
#endif

        this->simoxRobot->registerRobotNode(this->simoxRobotNode);
    }

    /*int main(int argc, char **argv ){
        SoDB::init();
        ColladaSimoxRobot robot;
        robot.parse("../RobotEditorArmar4.dae");
    }*/

} // namespace Collada
