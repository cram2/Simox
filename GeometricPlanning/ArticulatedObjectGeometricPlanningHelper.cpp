#include "ArticulatedObjectGeometricPlanningHelper.h"

#include <cmath>
#include <cstddef>
#include <iterator>
#include <numeric>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <SimoxUtility/algorithm/string/string_tools.h>
#include <VirtualRobot/MathTools.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/Nodes/RobotNodeFixed.h>
#include <VirtualRobot/Nodes/RobotNodeFixedFactory.h>
#include <VirtualRobot/Nodes/RobotNodePrismatic.h>
#include <VirtualRobot/Nodes/RobotNodeRevolute.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/math/AbstractFunctionR1R6.h>

#include "VirtualRobot/VirtualRobotException.h"
#include <GeometricPlanning/ParametricPath.h>
#include <GeometricPlanning/assert/assert.h>
#include <GeometricPlanning/path_primitives/CircleSegment.h>
#include <GeometricPlanning/path_primitives/Line.h>
#include <GeometricPlanning/path_primitives/PathPrimitive.h>
#include <GeometricPlanning/types.h>


namespace simox::geometric_planning
{

    ArticulatedObjectGeometricPlanningHelper::ArticulatedObjectGeometricPlanningHelper(
        const VirtualRobot::RobotPtr& articulatedObject) :
        articulatedObject(articulatedObject)
    {
    }

    ParametricPath
    ArticulatedObjectGeometricPlanningHelper::getPathForNode(const std::string& nodeName) const
    {
        const auto node = articulatedObject->getRobotNode(nodeName);
        REQUIRE(node != nullptr);

        const auto global_T_object_root = articulatedObject->getGlobalPose();
        articulatedObject->setGlobalPose(Eigen::Matrix4f::Identity());

        simox::geometric_planning::ArticulatedObjectGeometricPlanningHelper helper(
            articulatedObject);

        const auto parents = node->getAllParents();
        REQUIRE(not parents.empty());

        // VR_INFO << parents.size() << " parents for " << nodeName;

        /// assumption: only one parent is a movable joint

        const auto isJoint = [](const VirtualRobot::RobotNodePtr& node) -> bool
        {
            // // VR_INFO << parent->getName() << " " << parent->isJoint();
            // const std::size_t x = (parent->isJoint()) ? 1 : 0;

            // hack: the above is not working reliably of URDF models.
            // therefore, we only accept joints with name matching "***_joint"

            return simox::alg::ends_with(node->getName(), "joint");
        };

        // validate assumption
        [[maybe_unused]] const auto numberMovableParentJoints =
            std::accumulate(parents.begin(),
                            parents.end(),
                            0,
                            [&isJoint](const std::size_t& init, const auto& parent)
                            {
                                const std::size_t x = isJoint(parent) ? 1 : 0;

                                return init + x;
                            });

        REQUIRE(numberMovableParentJoints == 1);

        const auto parentJoint = std::find_if(parents.begin(), parents.end(), isJoint);
        const auto& joint = *parentJoint;
        REQUIRE(joint != nullptr);

        const auto parametricPath = helper.getPathForNode(node->getName(), joint->getName());

        // reset global pose
        articulatedObject->setGlobalPose(global_T_object_root);


        return parametricPath;
    }

    ParametricPath
    ArticulatedObjectGeometricPlanningHelper::createCircularPath(
        const VirtualRobot::RobotNodePtr& node,
        const VirtualRobot::RobotNodePtr& joint) const
    {

        const auto* revoluteJoint =
            dynamic_cast<const VirtualRobot::RobotNodeRevolute*>(joint.get());
        REQUIRE_MESSAGE(revoluteJoint != nullptr, "`joint` must be a revolute joint!");

        const float initialJointValue = joint->getJointValue();

        joint->setJointValue(joint->getJointLimitLow());

        // We define a reference frame that represents the joint with the axis into +z direction
        // const Pose global_T_joint_ref(joint->getGlobalPose());
        // ARMARX_DEBUG << VAROUT(revoluteJoint->getJointRotationAxis());
        const Pose global_T_joint_orig(revoluteJoint->getGlobalPose());

        const Pose joint_orig_R_global(global_T_joint_orig.inverse().linear());

        const Eigen::Vector3f localJointAxis =
            joint_orig_R_global * revoluteJoint->getJointRotationAxis();
        const Eigen::Vector3f localDesiredJointAxis = Eigen::Vector3f::UnitZ();

        const Pose joint_T_joint_ref(
            Eigen::Quaternionf::FromTwoVectors(localDesiredJointAxis, localJointAxis)
                .toRotationMatrix());
        // ARMARX_DEBUG << VAROUT(joint_T_joint_ref.linear());

        const Pose global_T_joint_ref = global_T_joint_orig * joint_T_joint_ref;
        // * Pose(Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f::UnitZ(), revoluteJoint->getJointRotationAxis()).toRotationMatrix());

        const Pose global_T_node(node->getGlobalPose());

        // this is the node within the joint frame
        // joint_ref_T_node = joint_ref_T_global * global_T_node
        const Pose joint_ref_T_node = global_T_joint_ref.inverse() * global_T_node;

        // ARMARX_DEBUG << "relative position: " << joint_T_node.translation();

        // the plane in which the node moves is given by the following (x, y coordinates = 0 for movement plane)
        const Pose global_T_joint_plane =
            global_T_joint_ref * Eigen::Translation3f{0, 0, joint_ref_T_node.translation().z()};

        // and the radius of the movement is given by the xy coordinate
        const float radius = joint_ref_T_node.translation().head<2>().norm();
        // ARMARX_DEBUG << "Radius: " << radius;

        REQUIRE_MESSAGE(joint->getJointLimitHigh() > joint->getJointLimitLow(),
                        "Not implemented yet. Do so by flipping the z axis of the joint.");

        const ParameterRange parameterRange{
            .min = 0, .max = joint->getJointLimitHigh() - joint->getJointLimitLow()};

        // ARMARX_DEBUG << "radius is " << radius;

        // Pose postTransform; //(joint_R_node);
        // postTransform.setIdentity(); // FIXME ???

        //
        auto subpart = articulatedObject;

        const std::string nodeJointReference = node->getName() + "_joint_reference";

        // joint_T_joint_plane = joint_ref_T_global * global_t_joint_plane
        const Pose joint_ref_T_joint_plane = global_T_joint_ref.inverse() * global_T_joint_plane;

        // now we make sure that the joint coordinate system is oriented such that the x axis points towards the initial node position
        const Eigen::Vector3f global__joint_plane_P_node_initial =
            global_T_node.translation() - global_T_joint_plane.translation();

        // this is the vector in the joint plane. The z coordinate should be 0
        const Eigen::Vector3f joint_plane__joint_plane_P_node_initial =
            global_T_joint_plane.rotation().inverse() * global__joint_plane_P_node_initial;

        const float yaw = std::atan2(joint_plane__joint_plane_P_node_initial.y(),
                                     joint_plane__joint_plane_P_node_initial.x());
        // ARMARX_DEBUG << VAROUT(yaw);

        const Pose joint_plane_T_joint_reference(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));

        subpart->setJointValue(joint->getName(), joint->getJointLimitLow());

        const Pose global_T_root(subpart->getGlobalPose());
        const Pose root_T_global = global_T_root.inverse();
        const Pose root_T_joint_ref = root_T_global * global_T_joint_ref;

        const Pose root_T_joint_reference(root_T_joint_ref * joint_ref_T_joint_plane *
                                          joint_plane_T_joint_reference);

        const auto jointReferenceNode = std::make_shared<VirtualRobot::RobotNodeFixed>(
            subpart, nodeJointReference, root_T_joint_reference.matrix());

        // TODO check if node already exists and unregister if needed

        subpart->registerRobotNode(jointReferenceNode);
        CHECK_MESSAGE(jointReferenceNode->initialize(subpart->getRootNode()),
                      "Failed to initialize node `",
                      jointReferenceNode->getName(),
                      "`!");

        // reset joint state
        joint->setJointValue(initialJointValue);

        // we need the transformation from root_T_joint_reference to node, so that the orientations match up.
        // without the orientation adaption, this will output a coordinate system with x pointing in the
        // direction from global_T_plane to node, and z pointing upward.
        // therefore:
        // global_T_root * root_T_joint_reference = global_T_joint_reference
        // we just need the linear transformation though (i think), because the translation was already covered before
        Pose joint_reference_T_node(
            ((global_T_root * root_T_joint_reference).inverse() * global_T_node).linear());

        return {jointReferenceNode,
                std::make_unique<CircleSegment>(radius, parameterRange),
                joint_reference_T_node};
    }

    ParametricPath
    ArticulatedObjectGeometricPlanningHelper::createLinearPath(
        const VirtualRobot::RobotNodePtr& node,
        const VirtualRobot::RobotNodePtr& joint) const
    {

        const Pose relativePose(node->getPoseInFrame(joint));

        // ARMARX_DEBUG << "Relative pose" << relativePose.translation();

        const ParameterRange parameterRange{.min = joint->getJointLimitLow(),
                                            .max = joint->getJointLimitHigh()};

        // ARMARX_DEBUG << "linear param range " << parameterRange.min << ", " << parameterRange.max;
        // ARMARX_DEBUG << "Joint in root frame " << joint->getPoseInRootFrame();

        // Info: extractSubPart(joint, "", ""); instead of cloning the full robot is not working properly ...
        // Compared to a full clone, node->getName())->getPoseInFrame(joint) will be constant for all possible
        // joint values.
        // auto subpart = articulatedObject->clone();
        auto subpart = articulatedObject;

        subpart->setJointValue(joint->getName(), joint->getJointLimitLow());
        const Pose poseMin(subpart->getRobotNode(node->getName())->getPoseInFrame(joint));

        subpart->setJointValue(joint->getName(), joint->getJointLimitHigh());
        const Pose poseMax(subpart->getRobotNode(node->getName())->getPoseInFrame(joint));

        // ARMARX_DEBUG << poseMin.translation() << "to" << poseMax.translation();

        // The line parameter should directly relate to the joint state
        // Therefore, a virtual joint is introduced that is at the same position as the movable node.
        // The origin of the virtual joint is where the orinal joint state would be 0.
        Pose subframe = Pose::Identity();
        subframe.translation() =
            poseMin.translation() -
            (poseMax.translation() - poseMin.translation()) * joint->getJointLimitLow();

        const Eigen::Vector3f translationDirection =
            dynamic_cast<VirtualRobot::RobotNodePrismatic*>(joint.get())
                ->getJointTranslationDirectionJointCoordSystem();
        subframe.linear() =
            Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f::UnitX(), translationDirection)
                .toRotationMatrix();

        // ARMARX_DEBUG << "linear part" << subframe.linear();
        // ARMARX_DEBUG << "translation direction" << translationDirection;

        const std::string nodeJointReference = node->getName() + "_joint_reference";

        // FIXME attach to parent of joint

        subpart->setJointValue(joint->getName(), joint->getJointLimitLow());
        const Pose root_T_joint_reference(Pose(joint->getPoseInRootFrame()) * subframe);

        const auto jointReferenceNode = std::make_shared<VirtualRobot::RobotNodeFixed>(
            subpart, nodeJointReference, root_T_joint_reference.matrix());

        // TODO check if node already exists and unregister if needed

        subpart->registerRobotNode(jointReferenceNode);
        CHECK(jointReferenceNode->initialize(subpart->getRootNode()));


        // ARMARX_DEBUG << "registered robot node `" << jointReferenceNode->getName() << "`";
        const Eigen::Isometry3f global_T_node(node->getGlobalPose());
        const Eigen::Isometry3f global_T_jointReference(jointReferenceNode->getGlobalPose());

        const Eigen::Isometry3f jointReference_T_node = global_T_jointReference.inverse() * global_T_node;
        // jointRef_T_node = jointRef_T_global * global_T_node

        return ParametricPath(
            jointReferenceNode, std::make_unique<Line>(parameterRange), jointReference_T_node);
    }

    ParametricPath
    ArticulatedObjectGeometricPlanningHelper::getPathForNode(const std::string& node,
                                                             const std::string& joint) const
    {
        REQUIRE(articulatedObject->hasRobotNode(node));
        REQUIRE(articulatedObject->hasRobotNode(joint));

        const auto movableNode = articulatedObject->getRobotNode(node);
        const auto jointNode = articulatedObject->getRobotNode(joint);

        if (jointNode->isRotationalJoint())
        {
            // ARMARX_DEBUG << "joint " << joint << " is rotational joint";
            return createCircularPath(movableNode, jointNode);
        }

        if (jointNode->isTranslationalJoint())
        {
            // ARMARX_DEBUG << "joint " << joint << " is translational joint";
            return createLinearPath(movableNode, jointNode);
        }

        throw VirtualRobot::VirtualRobotException("Unknown joint type!");
    }

    ParametricPath
    ArticulatedObjectGeometricPlanningHelper::getPathForNode(
        const VirtualRobot::RobotNodePtr& node,
        const VirtualRobot::RobotNodePtr& joint) const
    {
        REQUIRE(articulatedObject->hasRobotNode(node));
        REQUIRE(articulatedObject->hasRobotNode(joint));

        if (joint->isRotationalJoint())
        {
            // ARMARX_DEBUG << "joint " << joint << " is rotational joint";
            return createCircularPath(node, joint);
        }

        if (joint->isTranslationalJoint())
        {
            // ARMARX_DEBUG << "joint " << joint << " is translational joint";
            return createLinearPath(node, joint);
        }

        throw VirtualRobot::VirtualRobotException("Unknown joint type!");
    }

} // namespace simox::geometric_planning
