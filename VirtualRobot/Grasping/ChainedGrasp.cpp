#include "ChainedGrasp.h"

#include "Nodes/RobotNodeFixed.h"
#include "VirtualRobot/MathTools.h"
#include "VirtualRobot/EndEffector/EndEffector.h"
#include "VirtualRobot/Robot.h"
#include "VirtualRobot/RobotNodeSet.h"
#include "VirtualRobot/Nodes/RobotNode.h"
#include "VirtualRobot/Nodes/RobotNodeRevolute.h"
#include "VirtualRobot/Nodes/RobotNodePrismatic.h"

namespace VirtualRobot
{

ChainedGrasp::ChainedGrasp(const std::string& name, const std::string& robotType, const std::string& eef,
                           const Eigen::Matrix4f& poseInTCPCoordSystem, const std::string& creation,
                           float quality, const std::string& eefPreshape) :
    Grasp(name, robotType, eef, poseInTCPCoordSystem, creation, quality, eefPreshape),
    x(VirtualJoint("x", false, 0)),
    y(VirtualJoint("y", false, 1)),
    z(VirtualJoint("z", false, 2)),
    roll(VirtualJoint("roll", true, 0)),
    pitch(VirtualJoint("pitch", true, 1)),
    yaw(VirtualJoint("yaw", true, 2)),
    graspableObjectCoordSysTransformation(Eigen::Matrix4f::Identity())
{
}

ChainedGrasp::~ChainedGrasp()
{
}

void ChainedGrasp::print(bool printDecoration) const
{
    Grasp::print(printDecoration);

    // TODO
}

RobotNodePtr ChainedGrasp::attachChain(RobotPtr robot, GraspableSensorizedObjectPtr object, bool addObjectVisualization) {
    if (!robot || !object) {
        return nullptr;
    }
    auto tcp = getTCP(robot);
    if (!tcp) {
        return nullptr;
    }
    auto virtual_object = getObjectNode(robot);
    if (virtual_object) {
        return virtual_object;
    }
    return attachChain(tcp, object, addObjectVisualization);
}

void ChainedGrasp::updateChain(RobotPtr robot) {
    if (!robot) { 
        return;
    }

    update(x, robot);
    update(y, robot);
    update(z, robot);
    update(roll, robot);
    update(pitch, robot);
    update(yaw, robot);

    if (robot->hasRobotNode(ROBOT_NODE_NAME("object"))) {
        auto virtual_object = robot->getRobotNode(ROBOT_NODE_NAME("object"));
        if (virtual_object->getLocalTransformation() != graspableObjectCoordSysTransformation) {
            virtual_object->setLocalTransformation(graspableObjectCoordSysTransformation);
            robot->updatePose();
        }
    }
}

void ChainedGrasp::detachChain(RobotPtr robot) {
    auto node = getObjectNode(robot);
    auto tcp = getTCP(robot);
    if (!node || !tcp) { 
        return;
    }
    while (node && node->getName() != tcp->getName()) {
        auto parent = node->getParent();
        parent->detachChild(node);
        robot->deregisterRobotNode(node);
        node = std::dynamic_pointer_cast<RobotNode>(parent);
    }
}

RobotNodePtr ChainedGrasp::getVirtualNode(RobotPtr robot, const std::string &virtualName) {
    if (robot) {
        if (robot->hasEndEffector(eef)) {
            auto name = ROBOT_NODE_NAME(virtualName);
            if (robot->hasRobotNode(name)) {
                return robot->getRobotNode(name);
            }
        }
    }
    return nullptr;
}

RobotNodePtr ChainedGrasp::getObjectNode(RobotPtr robot) {
    return getVirtualNode(robot, "object");
}

RobotNodePtr ChainedGrasp::attachChain(RobotNodePtr robotNode, GraspableSensorizedObjectPtr object, bool addObjectVisualization) {
    auto robot = robotNode->getRobot();

    auto virtual_x = attach(x, robotNode);
    auto virtual_y = attach(y, virtual_x);
    auto virtual_z = attach(z, virtual_y);
    auto virtual_roll = attach(roll, virtual_z);
    auto virtual_pitch = attach(pitch, virtual_roll);
    auto virtual_yaw = attach(yaw, virtual_pitch);

    RobotNodePtr virtual_object(new RobotNodeFixed(robot, ROBOT_NODE_NAME("object"), graspableObjectCoordSysTransformation, addObjectVisualization ? object->getVisualization() : nullptr,
                                                     addObjectVisualization ? object->getCollisionModel() : nullptr, object->getPhysics(), object->getCollisionChecker(), RobotNode::Generic));
    robot->registerRobotNode(virtual_object);
    virtual_yaw->attachChild(virtual_object);
    virtual_object->initialize(virtual_yaw);

    return virtual_object;
}

RobotNodePtr ChainedGrasp::attach(VirtualJoint joint, RobotNodePtr robotNode) {
    auto robot = robotNode->getRobot();
    RobotNodePtr virtualNode;
    if (joint.isRevolute) {
        virtualNode.reset(new RobotNodeRevolute(robot, ROBOT_NODE_NAME(joint.name), joint.min, joint.max, Eigen::Matrix4f::Identity(), joint.getAxis()));
        if (joint.isLimitless()) {
            virtualNode->setLimitless(true);
        }
    }
    else {
        virtualNode.reset(new RobotNodePrismatic(robot, ROBOT_NODE_NAME(joint.name), joint.min, joint.max, Eigen::Matrix4f::Identity(), joint.getAxis()));
    }
    robot->registerRobotNode(virtualNode);
    robotNode->attachChild(virtualNode);
    virtualNode->initialize(robotNode);
    virtualNode->setJointValue(joint.value);
    return virtualNode;
}

bool ChainedGrasp::update(VirtualJoint joint, RobotPtr robot) {
    auto robotNode = robot->getRobotNode(ROBOT_NODE_NAME(joint.name));
    if (!robotNode) { 
        return false;
    }
    robotNode->setJointValue(joint.value);
    robotNode->setJointLimits(joint.min, joint.max);
    if (joint.isRevolute) {
        robotNode->setLimitless(joint.isLimitless());
    }
    return true;
}

void ChainedGrasp::sampleGraspsUniform(std::vector<Eigen::Matrix4f> &grasps, RobotPtr robot, unsigned int grid)
{
    if (getObjectNode(robot)) {
        auto tcp = getTCP(robot);
        if (tcp) {
            sampleGraspsUniform(grasps, tcp->getName(), tcp, grid);
        }
    }
}

void ChainedGrasp::sampleGraspsUniform(std::vector<Eigen::Matrix4f> &grasps, const std::string &rootName, RobotNodePtr robotNode, unsigned int grid)
{
    float low = robotNode->getJointLimitLo();
    float high = robotNode->getJointLimitHigh();
    VR_INFO <<  robotNode->getName() << " " << low << " " << high << std::endl;
    auto children = robotNode->getChildren();
    unsigned int joint_grid = abs(high - low) > 1e-6 ? grid : 1;
    for (unsigned int i = 0; i < joint_grid; i++)
    {
        if (children.empty())
        {
            robotNode->getRobot()->updatePose();
            Eigen::Matrix4f globalPose = robotNode->getGlobalPose();
            Eigen::Matrix4f localPose = robotNode->getRobot()->getRobotNode(rootName)->toLocalCoordinateSystem(globalPose);
            Eigen::Matrix4f inverse = localPose.inverse();
            grasps.push_back(inverse);
        }
        else
        {
            double percentage = joint_grid == 1 ? 0.5 : ((double)i / (joint_grid - 1));
            robotNode->setJointValue(low + (high - low) * percentage);
            sampleGraspsUniform(grasps, rootName, std::dynamic_pointer_cast<RobotNode>(children[0]), grid);
        }
    }
}

Eigen::Matrix4f ChainedGrasp::getLocalPose() const {
    return simox::math::pos_rpy_to_mat4f(getPositionXYZ(), getOrientationRPY());
}

std::vector<RobotPtr> ChainedGrasp::sampleHandsUniform(RobotPtr robot, unsigned int grid) {
    std::vector<RobotPtr> hands;
    auto virtualObject = getObjectNode(robot);
    auto endEffector = getEndEffector(robot);
    if (virtualObject && endEffector) {
        std::vector<Eigen::Matrix4f> grasps;
        sampleGraspsUniform(grasps, robot->getRootNode()->getName(), robot->clone("clonedHand")->getEndEffector(endEffector->getName())->getTcp(), grid);
        auto exampleHand = endEffector->createEefRobot(name, name);
        detachChain(exampleHand);
        int i = 0;
        for (const auto& grasp : grasps) {
            auto name = "grasp" + std::to_string(i++);
            auto hand = exampleHand->clone();
            auto globalPose = virtualObject->toGlobalCoordinateSystem(grasp);
            hand->setGlobalPose(globalPose);
            hands.push_back(hand);
        }
    }
    return hands;
}

ChainedGrasp::VirtualJoint* ChainedGrasp::getVirtualJoint(const std::string &name) {
    if (name == x.name) return &x;
    else if (name == y.name) return &y;
    else if (name == z.name) return &z;
    else if (name == roll.name) return &roll;
    else if (name == pitch.name) return &pitch;
    else if (name == yaw.name) return &yaw;
    else
    {
        VR_INFO << "No virtual joint called " << name << " in grasp" << std::endl;
        return nullptr;
    }
}

std::vector<float> ChainedGrasp::getUsedVirtualLimits() const {
    std::vector<float> usedVirtualLimits;
    x.addLimits(usedVirtualLimits);
    y.addLimits(usedVirtualLimits);
    z.addLimits(usedVirtualLimits);
    roll.addLimits(usedVirtualLimits);
    pitch.addLimits(usedVirtualLimits);
    yaw.addLimits(usedVirtualLimits);
    return usedVirtualLimits;
}

Eigen::Vector3f ChainedGrasp::getPositionXYZ() const {
    return Eigen::Vector3f(x.value, y.value, z.value);
}

Eigen::Vector3f ChainedGrasp::getOrientationRPY() const {
    return Eigen::Vector3f(roll.value, pitch.value, yaw.value);
}

Eigen::Matrix4f ChainedGrasp::getTransformation() const {
    return getLocalPose() * graspableObjectCoordSysTransformation;
}

Eigen::Matrix4f ChainedGrasp::getLocalPoseGrasp() {
    return getTransformation().inverse();
}

void ChainedGrasp::setObjectTransformation(const Eigen::Matrix4f &graspableObjectCoordSysTransformation) {
    this->graspableObjectCoordSysTransformation = graspableObjectCoordSysTransformation;
}

std::string ChainedGrasp::getTransformationXML(const std::string &tabs) const {
    std::stringstream ss;
    std::string tt = tabs + "\t";
    std::string ttt = tabs + "\t\t";
    ss << tabs << "<ChainGrasp>\n";
    ss << tt << x.toXML();
    ss << tt << y.toXML();
    ss << tt << z.toXML();
    ss << tt << roll.toXML();
    ss << tt << pitch.toXML();
    ss << tt << yaw.toXML();
    if (graspableObjectCoordSysTransformation != Eigen::Matrix4f::Identity()) {
        ss << tt << "<Transform>\n";
        ss << MathTools::getTransformXMLString(graspableObjectCoordSysTransformation, ttt);
        ss << tt << "</Transform>\n";
    }
    ss << tabs << "</ChainGrasp>\n";
    return ss.str();
}

GraspPtr ChainedGrasp::clone() const {
    // TODO
    VR_INFO << "Cloning chained grasp not yet implemented. Calling Grasp::clone()" << std::endl;
    return Grasp::clone();
}

bool ChainedGrasp::visualizeRotationCoordSystem(RobotPtr robot, bool visible) {
    auto node = getVirtualNode(robot, yaw.name);
    if (node) {
        node->showCoordinateSystem(visible);
        return true;
    }
    return false;
}

std::string ChainedGrasp::ROBOT_NODE_NAME(const std::string &virtualJointName)
{
    std::stringstream ss;
    ss << "virtual_";
    ss << eef;
    ss << "_";
    ss << virtualJointName;
    return ss.str();
}

EndEffectorPtr ChainedGrasp::getEndEffector(RobotPtr robot) {
    if (robot->hasEndEffector(eef)) {
        return robot->getEndEffector(eef);
    } 
    
    return nullptr;
}

RobotNodePtr ChainedGrasp::getTCP(RobotPtr robot) {
    auto endEffector = getEndEffector(robot);
    if (endEffector) {
        return endEffector->getTcp();
    } 
    
    return nullptr;
}

std::vector<std::string> ChainedGrasp::getNames(bool onlyAdaptable) {
    std::vector<std::string> names;
    if (x.isUsed() || !onlyAdaptable) names.push_back(ROBOT_NODE_NAME(x.name));
    if (y.isUsed() || !onlyAdaptable) names.push_back(ROBOT_NODE_NAME(y.name));
    if (z.isUsed() || !onlyAdaptable) names.push_back(ROBOT_NODE_NAME(z.name));
    if (roll.isUsed() || !onlyAdaptable) names.push_back(ROBOT_NODE_NAME(roll.name));
    if (pitch.isUsed() || !onlyAdaptable) names.push_back(ROBOT_NODE_NAME(pitch.name));
    if (yaw.isUsed() || !onlyAdaptable) names.push_back(ROBOT_NODE_NAME(yaw.name));
    if (!onlyAdaptable) names.push_back(ROBOT_NODE_NAME("object"));
    return names;
}

RobotNodeSetPtr ChainedGrasp::createRobotNodeSet(RobotPtr robot, RobotNodeSetPtr rns) {
    auto virtualObject = getObjectNode(robot);
    if (virtualObject) {
        std::vector<std::string> nodeNames;
        for (const std::string &name : rns->getNodeNames()) nodeNames.push_back(name);
        //int index = nodeNames.size();
        for (const std::string &name : getNames(true)) nodeNames.push_back(name);
        /*SceneObjectPtr node = robot->getRobotNode(nodeNames.at(index));
        while (node != robot->getRobotNode(nodeNames.at(index-1))) {
            node = node->getParent();
            nodeNames.insert(nodeNames.begin() + index, node->getName());
        }*/
        return RobotNodeSet::createRobotNodeSet(robot, this->name, nodeNames, rns->getKinematicRoot()->getName(), virtualObject->getName());
    }
    return nullptr;
}


// VirtualJoint

bool ChainedGrasp::VirtualJoint::setValue(float value) {
    if (isRevolute && (value < -M_PI || value > M_PI)) {
        return false;
    }
    else if (min == max) {
        min = value;
        max = value;
    }
    else if (value < min || value > max) {
        return false;
    }
    this->value = value;
    return true;
}

void ChainedGrasp::VirtualJoint::resetLimits() {
    min = value;
    max = value;
}

bool ChainedGrasp::VirtualJoint::setLimitsValue(float min, float max) {
    if (setLimits(min, max)) {
        value = (max + min) / 2.0f;
        return true;
    }
    return false;
}

bool ChainedGrasp::VirtualJoint::setLimits(float min, float max) {
    if (min <= max) {
        if (isRevolute) {
            this->min = std::fmax(-M_PI, min);
            this->max = std::fmin(M_PI, max);
        }
        else {
            this->min = min;
            this->max = max;
        }
        return true;
    }
    return false;
}

bool ChainedGrasp::VirtualJoint::setLimitless() {
    if (isRevolute) {
        this->min = -M_PI;
        this->max = M_PI;
        return true;
    }
    return false;
}

bool ChainedGrasp::VirtualJoint::isLimitless() {
    return max - min > 2 * M_PI - 0.000001;
}

Eigen::Vector3f ChainedGrasp::VirtualJoint::getAxis() const {
    Eigen::Vector3f axis(Eigen::Vector3f::Zero());
    axis(this->axis) = 1;
    return axis;
}

float ChainedGrasp::VirtualJoint::getMin() const {
    return min;
}

float ChainedGrasp::VirtualJoint::getMax() const {
    return max;
}

float ChainedGrasp::VirtualJoint::isUsed() const {
    return std::abs(max - min) > 0.000001;
}

std::string ChainedGrasp::VirtualJoint::toXML() const {
    std::stringstream ss;
    ss << "<VirtualJoint name='" << name << "' value='" << value;
    if (isUsed())
        ss << "' min='" << min  << "' max='" << max;
    ss << "'/>\n";
    return ss.str();
}

ChainedGrasp::VirtualJoint::VirtualJoint(const std::string &name, bool isRevolute, int axis) :
    name(name),
    value(0.0f),
    min(0.0f),
    max(0.0f),
    isRevolute(isRevolute),
    axis(axis)
{
}

void ChainedGrasp::VirtualJoint::addLimits(std::vector<float> &usedLimits) const {
    if (isUsed()) {
        usedLimits.push_back(min);
        usedLimits.push_back(max);
    }
}

}
