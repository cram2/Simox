
#include "Dynamics.h"

#include <math.h>

#include <iostream>
#include <string>

#include <Eigen/Dense>

#include <VirtualRobot/MathTools.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/Nodes/RobotNodeFactory.h>
#include <VirtualRobot/Nodes/RobotNodePrismatic.h>
#include <VirtualRobot/Nodes/RobotNodeRevolute.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <VirtualRobot/Units.h>
#include <VirtualRobot/XML/RobotIO.h>

#include "Logging.h"
#include "VirtualRobotException.h"
#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>

#define VERBOSE_OUT                                                                                \
    if (verbose)                                                                                   \
    std::cout


using std::cin;
using std::cout;

namespace VirtualRobot
{
    using std::cout;
    using std::endl;

    VirtualRobot::Dynamics::Dynamics(RobotNodeSetPtr rns, RobotNodeSetPtr rnsBodies, bool verbose) :
        rns(rns), rnsBodies(rnsBodies), verbose(verbose)
    {
        if (!rns)
        {
            THROW_VR_EXCEPTION("RobotNodeSetPtr for the joints is zero pointer");
        }
        VERBOSE_OUT << "joint values:\n" << rns->getJointValuesEigen() << std::endl;
        gravity = RigidBodyDynamics::Math::Vector3d(0, 0, -9.81);
        model = std::shared_ptr<RigidBodyDynamics::Model>(new RigidBodyDynamics::Model());

        model->gravity = gravity;

        if (!rns->isKinematicChain())
        {
            THROW_VR_EXCEPTION("RobotNodeSet is not a kinematic chain!")
        }

        RobotNodePtr root = rns->getKinematicRoot();

        //VERBOSE_OUT << "Root name: "<<root->getName()<<endl;

        //Dynamics::toRBDLRecursive(model, root, root->getGlobalPose(), Eigen::Matrix4f::Identity());
        Dynamics::toRBDL(model, rns->getNode(0), rns);
        zeroVec = Eigen::VectorXd::Zero(model->dof_count);
    }

    Eigen::VectorXd
    Dynamics::getInverseDynamics(const Eigen::VectorXd& q,
                                 const Eigen::VectorXd& qdot,
                                 const Eigen::VectorXd& qddot)
    {
        Eigen::VectorXd tau = Eigen::VectorXd::Zero(Dynamics::model->dof_count);
        getInverseDynamics(q, qdot, qddot, tau);
        return tau;
    }

    void
    Dynamics::getInverseDynamics(const Eigen::VectorXd& q,
                                 const Eigen::VectorXd& qdot,
                                 const Eigen::VectorXd& qddot,
                                 Eigen::VectorXd& tau)
    {
        tau.setZero();
        VR_ASSERT(tau.rows() == q.rows());
        InverseDynamics(*model.get(), q, qdot, qddot, tau);
    }

    Eigen::VectorXd
    Dynamics::getForwardDynamics(const Eigen::VectorXd& q,
                                 const Eigen::VectorXd& qdot,
                                 Eigen::VectorXd tau)
    {
        Eigen::VectorXd qddot = Eigen::VectorXd::Zero(Dynamics::model->dof_count);
        ForwardDynamics(*model.get(), q, qdot, tau, qddot);
        return qddot;
    }

    Eigen::VectorXd
    Dynamics::getGravityMatrix(const Eigen::VectorXd& q)
    {
        Eigen::VectorXd tauGravity = Eigen::VectorXd::Zero(Dynamics::model->dof_count);
        getGravityMatrix(q, tauGravity);
        return tauGravity;
    }

    void
    Dynamics::getGravityMatrix(const Eigen::VectorXd& q, Eigen::VectorXd& tau)
    {
        VR_ASSERT(q.rows() == Dynamics::model->dof_count);
        InverseDynamics(*model.get(), q, zeroVec, zeroVec, tau);
    }

    Eigen::VectorXd
    Dynamics::getCoriolisMatrix(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot)
    {
        //    Eigen::VectorXd qddot = Eigen::VectorXd::Zero(Dynamics::model->dof_count);
        Eigen::VectorXd tauGravity = getGravityMatrix(q);
        Eigen::VectorXd tau = Eigen::VectorXd::Zero(Dynamics::model->dof_count);

        Eigen::MatrixXd tauCoriolis = Eigen::VectorXd::Zero(Dynamics::model->dof_count);
        InverseDynamics(*model.get(), q, qdot, zeroVec, tau);
        tauCoriolis = tau - tauGravity;
        return tauCoriolis;
    }

    Eigen::MatrixXd
    Dynamics::getInertiaMatrix(const Eigen::VectorXd& q, bool updateKinematics)
    {
        Eigen::MatrixXd inertia = Eigen::MatrixXd::Zero(model->dof_count, model->dof_count);
        getInertiaMatrix(q, inertia, updateKinematics);
        return inertia;
    }

    void
    Dynamics::getInertiaMatrix(const Eigen::VectorXd& q,
                               Eigen::MatrixXd& inertiaMatrix,
                               bool updateKinematics)
    {
        CompositeRigidBodyAlgorithm(*model.get(), q, inertiaMatrix, updateKinematics);
    }

    void
    Dynamics::setGravity(const Eigen::Vector3d& gravity)
    {
        model->gravity = gravity;
    }

    int
    Dynamics::getnDoF()
    {
        return model->dof_count;
    }

    void
    Dynamics::print()
    {
        std::string result = RigidBodyDynamics::Utils::GetModelHierarchy(*model.get());
        std::cout << "RBDL hierarchy of RNS:" << rns->getName() << std::endl;
        std::cout << result;
        result = RigidBodyDynamics::Utils::GetNamedBodyOriginsOverview(*model.get());
        std::cout << "RBDL origins of RNS:" << rns->getName() << std::endl;
        std::cout << result;
        result = RigidBodyDynamics::Utils::GetModelDOFOverview(*model.get());
        std::cout << "RBDL DoF of RNS:" << rns->getName() << std::endl;
        std::cout << result;
    }

    std::tuple<Eigen::Matrix3d, Eigen::Vector3d, double>
    Dynamics::computeCombinedPhysics(const std::set<RobotNodePtr>& nodes,
                                     const RobotNodePtr& referenceNode)
    {
        Eigen::Matrix3f combinedInertia;
        combinedInertia.setZero();
        Eigen::Vector3f combinedCoM;
        combinedCoM.setZero();
        double massSum = referenceNode->getMass();
        for (auto& node : nodes)
        {
            massSum = node->getMass();
        }

        Eigen::Matrix3f rotation = referenceNode->getGlobalPose().block<3, 3>(0, 0);
        Eigen::Vector3f comGlobalMeters = referenceNode->getCoMGlobal() / 1000;
        Eigen::Matrix3f inertiaInGlobal =
            referenceNode->getInertiaMatrix(comGlobalMeters, rotation);
        combinedInertia += inertiaInGlobal;
        combinedCoM += referenceNode->getCoMGlobal() * referenceNode->getMass() / massSum;

        for (auto& node : nodes)
        {
            Eigen::Matrix3f rotation = node->getGlobalPose().block<3, 3>(0, 0);
            Eigen::Vector3f comGlobalMeters = node->getCoMGlobal() / 1000;
            Eigen::Matrix3f inertiaInGlobal = node->getInertiaMatrix(comGlobalMeters, rotation);
            combinedInertia += inertiaInGlobal;
            combinedCoM += node->getCoMGlobal() * node->getMass() / massSum;
        }
        combinedInertia = SceneObject::shiftInertia(combinedInertia, -combinedCoM / 1000, massSum);
        rotation = referenceNode->getGlobalPose().block<3, 3>(0, 0).inverse();
        combinedInertia = rotation * combinedInertia * rotation.transpose();
        return std::make_tuple(combinedInertia.cast<double>(), combinedCoM.cast<double>(), massSum);
    }

    std::shared_ptr<RigidBodyDynamics::Body>
    Dynamics::computeCombinedBody(const std::set<RobotNodePtr>& nodes,
                                  const RobotNodePtr& referenceNode) const
    {
        //    VR_ASSERT(!nodes.empty());
        Eigen::Vector3d CoM = referenceNode->getCoMLocal().cast<double>() / 1000;
        RigidBodyDynamics::Math::Matrix3d inertia =
            referenceNode->getInertiaMatrix().cast<double>();

        auto mainBody = rnsBodies && rnsBodies->hasRobotNode(referenceNode)
                            ? std::make_shared<RigidBodyDynamics::Body>(referenceNode->getMass(), CoM, inertia)
                            : std::make_shared<RigidBodyDynamics::Body>();

        for (auto node : nodes)
        {
            Eigen::Vector3d CoM = node->getCoMLocal().cast<double>() / 1000;
            RigidBodyDynamics::Math::Matrix3d inertia = node->getInertiaMatrix().cast<double>();

            auto otherBody = RigidBodyDynamics::Body(node->getMass(), CoM, inertia);
            Eigen::Matrix4f transform = referenceNode->getTransformationTo(node);
            RigidBodyDynamics::Math::SpatialTransform rbdlTransform(
                transform.block<3, 3>(0, 0).cast<double>(),
                transform.block<3, 1>(0, 3).cast<double>() / 1000);
            mainBody->Join(rbdlTransform, otherBody);
        }
        return mainBody;
    }

    bool
    Dynamics::getVerbose() const
    {
        return verbose;
    }

    void
    Dynamics::setVerbose(bool value)
    {
        verbose = value;
    }

    std::shared_ptr<RigidBodyDynamics::Model>
    Dynamics::getModel() const
    {
        return model;
    }

    // this method just selects the first node with an attached mass that is no RigidBodyDynamics::Joint
    RobotNodePtr
    Dynamics::checkForConnectedMass(RobotNodePtr node)
    {
        if (!node)
        {
            return RobotNodePtr();
        }

        for (SceneObjectPtr const& child : node->getChildren())
        {
            RobotNodePtr childPtr = std::dynamic_pointer_cast<RobotNode>(child);

            if (childPtr != 0 && // existing
                childPtr->getMass() > 0 && // has mass
                (childPtr->isTranslationalJoint() // is translational joint
                 || (!childPtr->isTranslationalJoint() &&
                     !childPtr->isRotationalJoint()))) // or fixed joint
            {
                return childPtr;
            }
        }
        for (SceneObjectPtr const& child : node->getChildren())
        {
            RobotNodePtr rnPtr = std::dynamic_pointer_cast<RobotNode>(child);
            RobotNodePtr childPtr;

            if (rnPtr && !rnPtr->isRotationalJoint()) // break recursion if child is rot joint
            {
                childPtr = checkForConnectedMass(rnPtr);
            }

            if (childPtr)
            {
                return childPtr;
            }
        }
        return RobotNodePtr();
    }

    std::set<RobotNodePtr>
    Dynamics::getChildrenWithMass(const RobotNodePtr& node, const RobotNodeSetPtr& nodeSet) const
    {
        std::set<RobotNodePtr> result;
        for (auto& obj : node->getChildren())
        {
            auto node = std::dynamic_pointer_cast<VirtualRobot::RobotNode>(obj);

            if (node && nodeSet->hasRobotNode(node))
            {
                continue;
            }
            if (node && obj->getMass() > 0.01 && (!rnsBodies || rnsBodies->hasRobotNode(node)))
            {
                result.insert(node);
            }
            if (node)
            {
                auto tmp = getChildrenWithMass(node, nodeSet);
                result.insert(tmp.begin(), tmp.end());
            }
        }
        return result;
    }

    // rbdl: (trafo->joint->body) -> (trafo->joint->body) -> (trafo->joint->body) ...
    void
    Dynamics::toRBDLRecursive(std::shared_ptr<RigidBodyDynamics::Model> model,
                              RobotNodePtr node,
                              Eigen::Matrix4f accumulatedTransformPreJoint,
                              Eigen::Matrix4f accumulatedTransformPostJoint,
                              RobotNodePtr jointNode,
                              int parentID)
    {
        VR_ASSERT(model);
        VR_ASSERT(node);
        int nodeID = parentID; // might be overwritten when adding new body
        float mass = node->getMass();


        if (!jointNode)
        {
            accumulatedTransformPreJoint *= node->getLocalTransformation();

            if (Dynamics::rns->hasRobotNode(node) &&
                (node->isRotationalJoint() || node->isTranslationalJoint()))
            {
                jointNode = node;
            }
        }
        else
        {
            accumulatedTransformPostJoint *= node->getLocalTransformation();

            if (Dynamics::rns->hasRobotNode(node) &&
                (node->isRotationalJoint() || node->isTranslationalJoint()))
            {
                VR_ERROR << "Skipping joint " << node->getName()
                         << ": multiple joints in row without masses inbetween..." << std::endl;
            }
        }


        if (mass > 0 && Dynamics::rns->hasRobotNode(node))
        {
            // create a body
            //RigidBodyDynamics::Math::Vector3d com = node->getCoMLocal().cast<double>() / 1000; // divide by 1000 because Simox defines lengths in mm while the RBDL defines lengths in m
            //RigidBodyDynamics::Math::Matrix3d inertia = node->getInertiaMatrix().cast<double>();

            // apply postJoint transform
            Eigen::Vector4f comTr;
            comTr.head(3) = node->getCoMLocal();
            comTr(3) = 1.0f;
            RigidBodyDynamics::Math::Vector3d com =
                (accumulatedTransformPostJoint * comTr).head(3).cast<double>() / 1000.0;

            // convert inertia from node to jointFrame (I_new = R * I_old * R^T)
            Eigen::Matrix3f trafo = accumulatedTransformPostJoint.block(0, 0, 3, 3);
            Eigen::Matrix3f inertia2 = trafo * node->getInertiaMatrix() * trafo.transpose();
            RigidBodyDynamics::Math::Matrix3d inertia = inertia2.cast<double>();


            RigidBodyDynamics::Body body = RigidBodyDynamics::Body(mass, com, inertia);

            RigidBodyDynamics::Math::Matrix3d spatial_rotation =
                accumulatedTransformPreJoint.block(0, 0, 3, 3).cast<double>();
            RigidBodyDynamics::Math::Vector3d spatial_translation =
                accumulatedTransformPreJoint.col(3).head(3).cast<double>() / 1000.0;
            RigidBodyDynamics::Math::SpatialTransform spatial_transform =
                RigidBodyDynamics::Math::SpatialTransform(spatial_rotation, spatial_translation);
            VERBOSE_OUT << "****** spatial_translation: " << spatial_translation.transpose()
                        << std::endl;

            // joint
            RigidBodyDynamics::Joint joint =
                RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeFixed);

            if (jointNode && jointNode->isRotationalJoint())
            {
                RigidBodyDynamics::JointType joint_type = RigidBodyDynamics::JointTypeRevolute;
                std::shared_ptr<RobotNodeRevolute> rev =
                    std::dynamic_pointer_cast<RobotNodeRevolute>(jointNode);
                VR_ASSERT(rev);
                RigidBodyDynamics::Math::Vector3d joint_axis =
                    rev->getJointRotationAxisInJointCoordSystem().cast<double>();

                joint = RigidBodyDynamics::Joint(joint_type, joint_axis);

                VR_INFO << "Adding Rotational RigidBodyDynamics::Joint" << std::endl;
            }
            else if (jointNode && jointNode->isTranslationalJoint())
            {
                RigidBodyDynamics::JointType joint_type = RigidBodyDynamics::JointTypePrismatic;
                std::shared_ptr<RobotNodePrismatic> prism =
                    std::dynamic_pointer_cast<RobotNodePrismatic>(jointNode);
                RigidBodyDynamics::Math::Vector3d joint_axis =
                    prism->getJointTranslationDirectionJointCoordSystem().cast<double>();

                joint = RigidBodyDynamics::Joint(joint_type, joint_axis);

                VR_INFO << "Adding Translational RigidBodyDynamics::Joint" << std::endl;
            }

            std::string nodeName;

            if (jointNode)
            {
                nodeName = jointNode->getName();
            }
            else
            {
                nodeName = node->getName();
            }

            nodeID = model->AddBody(parentID, spatial_transform, joint, body, nodeName);
            this->identifierMap[nodeName] = nodeID;

            VERBOSE_OUT << "New body:" << node->getName() << ", " << nodeID << " :"
                        << std::endl; // Debugging Info
            VERBOSE_OUT << "** SPATIAL TRAFO: " << endl
                        << spatial_transform.toMatrix() << std::endl;
            VERBOSE_OUT << "** MASS: " << body.mMass << std::endl;
            VERBOSE_OUT << "** COM: " << body.mCenterOfMass.transpose() << std::endl;
            VERBOSE_OUT << "** INERTIA: " << endl << body.mInertia << std::endl;
            VERBOSE_OUT << "** mIsVirtual: " << body.mIsVirtual << std::endl;

            if (jointNode)
            {
                VERBOSE_OUT << "** RigidBodyDynamics::Joint: " << jointNode->getName() << std::endl;
            }
            else
            {
                VERBOSE_OUT << "** RigidBodyDynamics::Joint: none" << std::endl;
            }

            if (joint.mJointAxes)
            {
                VERBOSE_OUT << "** JOINT AXES " << endl << *(joint.mJointAxes) << std::endl;
            }

            // reset pre and post trafos and jointNode
            accumulatedTransformPreJoint = Eigen::Matrix4f::Identity();
            accumulatedTransformPostJoint = Eigen::Matrix4f::Identity();
            jointNode.reset();

        } /*else

    {

        // node without mass, check how to proceed

    }*/

        std::vector<SceneObjectPtr> children = node->getChildren();

        for (SceneObjectPtr const& child : children)
        {
            std::shared_ptr<RobotNode> childRobotNode = std::dynamic_pointer_cast<RobotNode>(child);

            if (childRobotNode) // if cast returns 0 pointer, child is a sensor and can be omitted. also, child must be contained in the robotnodeset
            {
                //if (Dynamics::rns->hasRobotNode(childRobotNode))
                //{
                toRBDLRecursive(model,
                                childRobotNode,
                                accumulatedTransformPreJoint,
                                accumulatedTransformPostJoint,
                                jointNode,
                                nodeID);
                //} else
                //{
                //    VR_INFO << "skipping RN " << childRobotNode->getName() << " since it is not part of RNS" << std::endl;
                //}
            }
        }
    }

    void
    Dynamics::toRBDL(std::shared_ptr<RigidBodyDynamics::Model> model,
                     RobotNodePtr node,
                     RobotNodeSetPtr nodeSet,
                     RobotNodePtr parentNode,
                     int parentID)
    {
        VERBOSE_OUT << "#######ADDING NODE " << node->getName() << std::endl;
        RobotNodePtr physicsFromChild;
        int nodeID = parentID;
        // need to define body, joint and spatial transform
        // body first
        auto relevantChildNodes = getChildrenWithMass(node, nodeSet);
        for (auto node : relevantChildNodes)
        {
            VERBOSE_OUT << "Additional child: " << node->getName() << " - " << node->getMass()
                        << " kg" << std::endl;
        }
        auto combinedBody = computeCombinedBody(relevantChildNodes, node);


        RigidBodyDynamics::Body body = *combinedBody.get(); //Body(mass, com, inertia);


        // spatial transform next
        Eigen::Matrix4d trafo = Eigen::Matrix4d::Identity();

        if (parentNode)
        {
            trafo = node->getTransformationFrom(parentNode).cast<double>();
        }
        else if (!parentNode)
        {
            trafo =
                node->getGlobalPose()
                    .cast<
                        double>(); //node->getTransformationFrom(rns->getKinematicRoot()).cast<double>();
        }

        RigidBodyDynamics::Math::Matrix3d spatial_rotation = trafo.block(0, 0, 3, 3);
        RigidBodyDynamics::Math::Vector3d spatial_translation = trafo.col(3).head(3) / 1000;

        VERBOSE_OUT << "****** spatial_translation: " << spatial_translation.transpose()
                    << std::endl;

        RigidBodyDynamics::Math::SpatialTransform spatial_transform =
            RigidBodyDynamics::Math::SpatialTransform(spatial_rotation.transpose(),
                                                      spatial_translation);

        // last, joint
        RigidBodyDynamics::Joint joint =
            RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeFixed);

        if (node->isRotationalJoint())
        {
            RigidBodyDynamics::JointType joint_type = RigidBodyDynamics::JointTypeRevolute;
            std::shared_ptr<RobotNodeRevolute> rev =
                std::dynamic_pointer_cast<RobotNodeRevolute>(node);
            RigidBodyDynamics::Math::Vector3d joint_axis =
                rev->getJointRotationAxisInJointCoordSystem().cast<double>();

            joint = RigidBodyDynamics::Joint(joint_type, joint_axis);

            VERBOSE_OUT << "Rotational RigidBodyDynamics::Joint added:" << std::endl;
        }
        else if (node->isTranslationalJoint())
        {
            RigidBodyDynamics::JointType joint_type = RigidBodyDynamics::JointTypePrismatic;
            std::shared_ptr<RobotNodePrismatic> prism =
                std::dynamic_pointer_cast<RobotNodePrismatic>(node);
            RigidBodyDynamics::Math::Vector3d joint_axis =
                prism->getJointTranslationDirectionJointCoordSystem().cast<double>();

            joint = RigidBodyDynamics::Joint(joint_type, joint_axis);

            VERBOSE_OUT << "translational RigidBodyDynamics::Joint added" << std::endl;
        }

        if (joint.mJointType != RigidBodyDynamics::JointTypeFixed)
        {
            nodeID = model->AddBody(parentID, spatial_transform, joint, body, node->getName());
            this->identifierMap[node->getName()] = nodeID;
            Eigen::VectorXd QDDot = Eigen::VectorXd::Zero(identifierMap.size());

            VERBOSE_OUT << "New body:" << node->getName() << ", " << nodeID << " :"
                        << std::endl; // Debugging Info
            VERBOSE_OUT << "** SPATIAL TRAFO: " << endl
                        << spatial_transform.r << endl
                        << spatial_transform.E << std::endl;
            VERBOSE_OUT << "** MASS: " << body.mMass << std::endl;
            VERBOSE_OUT << "** COM: " << body.mCenterOfMass.transpose() << std::endl;
            VERBOSE_OUT << "** INERTIA: " << endl << body.mInertia << std::endl;
            VERBOSE_OUT << "** mIsVirtual: " << body.mIsVirtual << std::endl;
            Eigen::Vector3d bodyPosition = RigidBodyDynamics::CalcBodyToBaseCoordinates(
                *model,
                Eigen::VectorXd::Zero(identifierMap.size()),
                nodeID,
                Eigen::Vector3d::Zero(),
                true);
            Eigen::Vector3f positionInVirtualRobot =
                node->getGlobalPosition(); //rns->getKinematicRoot()->transformTo(node, zeroVec);
            auto diff = (positionInVirtualRobot - bodyPosition.cast<float>() * 1000).norm();
            if (diff > 0.01)
            {
                VR_ERROR << "forward kinematics between virtual robot and rbdl differ: " << diff
                         << " mm";
                throw VirtualRobotException(
                    "forward kinematics between virtual robot and rbdl differ!");
            }
            //        VERBOSE_OUT << "** position:\n" << bodyPosition << "\nposition in virtual robot:\n" << positionInVirtualRobot << endl << std::endl;

            if (joint.mJointAxes)
            {
                VERBOSE_OUT << "** JOINT AXES " << endl << *(joint.mJointAxes) << std::endl;
            }
        }

        for (size_t i = 0; i < nodeSet->getSize(); ++i)
        {
            if (nodeSet->getNode(i) == node && i + 1 < nodeSet->getSize())
            {
                toRBDL(model, nodeSet->getNode(i + 1), rns, node, nodeID);
            }
        }

        return;
    }
} // namespace VirtualRobot
