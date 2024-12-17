#include "BulletRobot.h"

#include <unordered_set>

#include <Eigen/Dense>

#include <VirtualRobot/Nodes/ContactSensor.h>
#include <VirtualRobot/Nodes/ForceTorqueSensor.h>
#include <VirtualRobot/Nodes/RobotNodeFixed.h>
#include <VirtualRobot/Nodes/RobotNodePrismatic.h>
#include <VirtualRobot/Nodes/RobotNodeRevolute.h>
#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <VirtualRobot/VirtualRobot.h>

#include "../../DynamicsWorld.h"
#include "../DynamicsObject.h"
#include "BulletEngine.h"
#include "BulletEngineFactory.h"
#include "DetectBulletVersion.h"
#include "VirtualRobot/MathTools.h"

//#define DEBUG_FIXED_OBJECTS
//#define DEBUG_SHOW_LINKS
using namespace VirtualRobot;
using namespace std;

namespace SimDynamics
{

    BulletRobot::BulletRobot(VirtualRobot::RobotPtr rob) :
        DynamicsRobot(rob)
        // should be enough for up to 10ms/step
        ,
        bulletMaxMotorImulse(30 * BulletObject::ScaleFactor)
    {
        ignoreTranslationalJoints = false;

        const bool enableJointMotors = not rob->isPassive();

        if (not enableJointMotors)
        {
            VR_INFO << "Robot " << rob->getName() << " is passive (no joint motors)";
        }

        buildBulletModels(enableJointMotors);

        // activate force torque sensors
        for (auto it = sensors.begin(); it != sensors.end(); it++)
        {
            ForceTorqueSensorPtr ftSensor = std::dynamic_pointer_cast<ForceTorqueSensor>(*it);

            if (ftSensor)
            {
                VirtualRobot::RobotNodePtr node = ftSensor->getRobotNode();
                THROW_VR_EXCEPTION_IF(!node, "parent of sensor could not be casted to RobotNode");

                if (!hasLink(node))
                {
                    VR_WARNING << "Ignoring FT sensor " << ftSensor->getName()
                               << ". Must be linked to a joint" << std::endl;
                }
                else
                {
                    const LinkInfo& link = getLink(node);
                    enableForceTorqueFeedback(link);
                    std::cout << "Found force torque sensor: " << node->getName() << std::endl;
                }
            }
        }
    }

    BulletRobot::~BulletRobot() = default;

    void
    BulletRobot::buildBulletModels(const bool enableJointMotors)
    {
        MutexLockPtr lock = getScopedLock();

        if (!robot)
        {
            return;
        }

        robotNodes = robot->getRobotNodes();
        for (auto rn : robotNodes)
        {
            CollisionModelPtr colModel = rn->getCollisionModel();
            if (colModel)
            {
                addIgnoredCollisionModels(rn);
                // search joint and connected model
                RobotNodePtr bodyA;
                RobotNodePtr bodyB = rn;
                RobotNodePtr joint;
                //RobotNodePtr joint2;

                // TODO: Enable new joint types
                if ((rn->isTranslationalJoint() && !ignoreTranslationalJoints) or
                    rn->isRotationalJoint())
                {
                    joint = rn;
                }

                RobotNodePtr parent = std::dynamic_pointer_cast<RobotNode>(rn->getParent());
                while (parent and bodyA == nullptr)
                {
                    // TODO: Enable new joint types.
                    if (not parent->getCollisionModel() and
                        ((parent->isTranslationalJoint() and not ignoreTranslationalJoints) or
                         parent->isRotationalJoint()))
                    {
                        if (!joint)
                        {
                            joint = parent;
                        }
                        else
                        {
                            VR_WARNING << "No body between " << parent->getName() << " and "
                                       << joint->getName() << ", skipping " << parent->getName()
                                       << std::endl;
                        }
                        /*else
                        {
                            // check for hinge2 joint
                            THROW_VR_EXCEPTION_IF(joint2, "three joints in a row not supported:" << joint->getName() << ", " << joint2->getName() << "," << parent->getName());
                            joint2 = parent;
                            Eigen::Matrix4f p1 = joint->getGlobalPose();
                            Eigen::Matrix4f p2 = joint2->getGlobalPose();

                            double d = (p1.block(0, 3, 3, 1) - p2.block(0, 3, 3, 1)).norm();
                            THROW_VR_EXCEPTION_IF((d > 1e-6), "Could not create hinge2 joint: Joint coord systems must be located at the same position:" << joint->getName() << ", " << joint2->getName());
                            RobotNodeRevolutePtr rev1 = std::dynamic_pointer_cast<RobotNodeRevolute>(joint);
                            RobotNodeRevolutePtr rev2 = std::dynamic_pointer_cast<RobotNodeRevolute>(joint2);
                            THROW_VR_EXCEPTION_IF(!rev1 || !rev2 , "Could not create hinge2 joint: Joints must be revolute nodes:" << joint->getName() << ", " << joint2->getName());
                            Eigen::Vector3f ax1 = rev1->getJointRotationAxis();
                            Eigen::Vector3f ax2 = rev2->getJointRotationAxis();
                            double ang = MathTools::getAngle(ax1, ax2);
                            THROW_VR_EXCEPTION_IF(fabs(fabs(ang) - M_PI_2) > 1e-6, "Could not create hinge2 joint: Joint axes must be orthogonal to each other:" << joint->getName() << ", " << joint2->getName());
                        }*/
                    }

                    if (parent->getCollisionModel())
                    {
                        bodyA = parent;
                        break;
                    }

                    parent = std::dynamic_pointer_cast<RobotNode>(parent->getParent());
                }

                if (!bodyA)
                {
                    bodyA = robot->getRootNode();
                }

                // check for fixed joint
                if (!joint)
                {
                    joint = bodyB;
                }

                if (bodyA != bodyB)
                {
                    createLink(bodyA, joint, /*joint2,*/ bodyB, enableJointMotors);
                }
            }
        }
    }

    void
    BulletRobot::addIgnoredCollisionModels(RobotNodePtr rn)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(rn);

        if (!rn->getCollisionModel())
        {
            return; // nothing to do: no col model -> no bullet model -> no collisions
        }

        createDynamicsNode(rn);
        std::vector<std::string> ic = rn->getIgnoredCollisionModels();
        RobotPtr robot = rn->getRobot();
        BulletObjectPtr drn1 = std::dynamic_pointer_cast<BulletObject>(dynamicRobotNodes[rn]);
        VR_ASSERT(drn1);

        for (const auto& i : ic)
        {
            RobotNodePtr rn2 = robot->getRobotNode(i);

            if (!rn2)
            {
                VR_ERROR << "Error while processing robot node <" << rn->getName()
                         << ">: Ignored collision model <" << i << "> is not part of robot..."
                         << std::endl;
            }
            else
            {
                createDynamicsNode(rn2);
                BulletObjectPtr drn2 =
                    std::dynamic_pointer_cast<BulletObject>(dynamicRobotNodes[rn2]);
                VR_ASSERT(drn2);
                DynamicsWorld::GetWorld()->getEngine()->disableCollision(drn1.get(), drn2.get());
            }
        }
    }

    bool
    BulletRobot::removeLink(const BulletRobot::LinkInfo& l)
    {
        bool result = false;
        std::vector<LinkInfo>::iterator it = links.begin();

        while (it != links.end())
        {
            if (it->dynNode1 == l.dynNode1 && it->dynNode2 == l.dynNode2)
            {
                it = links.erase(it);
                result = true;
            }
            else
            {
                it++;
            }
        }

        return result;
    }

    std::shared_ptr<btTypedConstraint>
    BulletRobot::createHingeJoint(std::shared_ptr<btRigidBody> btBody1,
                                  std::shared_ptr<btRigidBody> btBody2,
                                  Eigen::Matrix4f& coordSystemNode1,
                                  Eigen::Matrix4f& coordSystemNode2,
                                  Eigen::Matrix4f& anchor_inNode1,
                                  Eigen::Matrix4f& anchor_inNode2,
                                  Eigen::Vector3f& /*axisGlobal*/,
                                  Eigen::Vector3f& axisLocal,
                                  Eigen::Matrix4f& coordSystemJoint,
                                  double limMinBT,
                                  double limMaxBT)
    {
        // HINGE joint
        /*Eigen::Matrix4f tmpGp1 = coordSystemNode1;
        tmpGp1.block(0,3,3,1).setZero();
        Eigen::Matrix4f tmpGp2 = coordSystemNode2;
        tmpGp2.block(0,3,3,1).setZero();*/
        //Eigen::Vector3f axis_inLocal1 = (tmpGp1.inverse() * axisGlobal).block(0,0,3,1);
        //Eigen::Vector3f axis_inLocal2 = (tmpGp2.inverse() * axisGlobal).block(0,0,3,1);
        //Eigen::Vector3f axis_inLocal2 = rnRev2->getJointRotationAxisInJointCoordSystem();

        btVector3 pivot1 = BulletEngine::getVecBullet(anchor_inNode1.block(0, 3, 3, 1));
        btVector3 pivot2 = BulletEngine::getVecBullet(anchor_inNode2.block(0, 3, 3, 1));

        // we need to align coord system joint, so that z-axis is rotation axis

        MathTools::Quaternion q1 = MathTools::getRotation(Eigen::Vector3f::UnitZ(), axisLocal);
        Eigen::Matrix4f rotationzAlignment = MathTools::quat2eigen4f(q1);
        Eigen::Matrix4f coordSystemJoint_zAligned = coordSystemJoint * rotationzAlignment;

        // get transformation coord1 -> joint coord
        Eigen::Matrix4f trafoNode1 = coordSystemNode1.inverse() * coordSystemJoint_zAligned;
        // get transformation coord2 -> joint coord
        Eigen::Matrix4f trafoNode2 = coordSystemNode2.inverse() * coordSystemJoint_zAligned;

        // now we need to pivot points in local coord systems
        btTransform tr1 = BulletEngine::getPoseBullet(trafoNode1);
        btTransform tr2 = BulletEngine::getPoseBullet(trafoNode2);
        tr1.getOrigin() = pivot1;
        tr2.getOrigin() = pivot2;

        std::shared_ptr<btHingeConstraint> hinge(
            new btHingeConstraint(*btBody1, *btBody2, tr1, tr2, true));
        //        hinge->setDbgDrawSize(0.15);

        // todo: check effects of parameters...
        //        hinge->setParam(BT_CONSTRAINT_STOP_ERP, 2.0f);
        //        hinge->setParam(BT_CONSTRAINT_ERP, 2.f);


        //        hinge->setParam(BT_CONSTRAINT_CFM,0);
        //        hinge->setParam(BT_CONSTRAINT_STOP_CFM,0.f);
        //hinge->setLimit(limMin,limMax);//,btScalar(1.0f));
        //        hinge->setParam(BT_CONSTRAINT_CFM,0.0f);
        hinge->setLimit(btScalar(limMinBT), btScalar(limMaxBT));
        return hinge;
    }

    std::shared_ptr<btTypedConstraint>
    BulletRobot::createTranslationalJoint(std::shared_ptr<btRigidBody> btBody1,
                                          std::shared_ptr<btRigidBody> btBody2,
                                          Eigen::Matrix4f& coordSystemNode1,
                                          Eigen::Matrix4f& coordSystemNode2,
                                          Eigen::Matrix4f& anchor_inNode1,
                                          Eigen::Matrix4f& anchor_inNode2,
                                          Eigen::Vector3f& directionLocal,
                                          Eigen::Matrix4f& coordSystemJoint,
                                          double limMinBT,
                                          double limMaxBT)
    {
        // we need to align coord system joint, so that z-axis is rotation axis

        btVector3 pivot1 = BulletEngine::getVecBullet(anchor_inNode1.block(0, 3, 3, 1));
        btVector3 pivot2 = BulletEngine::getVecBullet(anchor_inNode2.block(0, 3, 3, 1));

        MathTools::Quaternion q1 =
            MathTools::getRotation(Eigen::Vector3f::UnitX(),
                                   directionLocal); // dont ask me why this has to be the X axis...
        Eigen::Matrix4f rotationzAlignment = MathTools::quat2eigen4f(q1);
        Eigen::Matrix4f coordSystemJoint_zAligned = coordSystemJoint * rotationzAlignment;

        // now we need to pivot points in local coord systems
        btTransform tr1 =
            BulletEngine::getPoseBullet(coordSystemNode1.inverse() * coordSystemJoint_zAligned);
        btTransform tr2 =
            BulletEngine::getPoseBullet(coordSystemNode2.inverse() * coordSystemJoint_zAligned);
        tr1.getOrigin() = pivot1;
        tr2.getOrigin() = pivot2;

        std::shared_ptr<btSliderConstraint> joint(
            new btSliderConstraint(*btBody1, *btBody2, tr1, tr2, true));

        // disable agular part
        joint->setLowerAngLimit(btScalar(0));
        joint->setUpperAngLimit(btScalar(0));

        joint->setLowerLinLimit(btScalar(limMinBT));
        joint->setUpperLinLimit(btScalar(limMaxBT));

        return joint;
    }

    std::shared_ptr<btTypedConstraint>
    BulletRobot::createFixedJoint(std::shared_ptr<btRigidBody> btBody1,
                                  std::shared_ptr<btRigidBody> btBody2,
                                  Eigen::Matrix4f& anchor_inNode1,
                                  Eigen::Matrix4f& anchor_inNode2)
    {
        btTransform localA, localB;
        localA = BulletEngine::getPoseBullet(anchor_inNode1);
        localB = BulletEngine::getPoseBullet(anchor_inNode2);
        std::shared_ptr<btGeneric6DofConstraint> generic6Dof(
            new btGeneric6DofConstraint(*btBody1, *btBody2, localA, localB, true));
        generic6Dof->setOverrideNumSolverIterations(100);

        for (int i = 0; i < 6; i++)
        {
            generic6Dof->setLimit(i, 0, 0);
        }

        /*generic6Dof->setParam(BT_CONSTRAINT_STOP_CFM,0.01f,0);
        generic6Dof->setParam(BT_CONSTRAINT_STOP_CFM,0.01f,1);
        generic6Dof->setParam(BT_CONSTRAINT_STOP_CFM,0.01f,2);
        generic6Dof->setParam(BT_CONSTRAINT_STOP_CFM,0.01f,3);
        generic6Dof->setParam(BT_CONSTRAINT_STOP_CFM,0.01f,4);
        generic6Dof->setParam(BT_CONSTRAINT_STOP_CFM,0.01f,5);*/
        /*generic6Dof->setParam(BT_CONSTRAINT_STOP_ERP,0.9f,0);
        generic6Dof->setParam(BT_CONSTRAINT_STOP_ERP,0.9f,1);
        generic6Dof->setParam(BT_CONSTRAINT_STOP_ERP,0.9f,2);
        generic6Dof->setParam(BT_CONSTRAINT_STOP_ERP,0.9f,3);
        generic6Dof->setParam(BT_CONSTRAINT_STOP_ERP,0.9f,4);
        generic6Dof->setParam(BT_CONSTRAINT_STOP_ERP,0.9f,5);*/
        return generic6Dof;
    }

    void
    BulletRobot::createLink(VirtualRobot::RobotNodePtr bodyA,
                            VirtualRobot::RobotNodePtr joint,
                            /*VirtualRobot::RobotNodePtr joint2,*/ VirtualRobot::RobotNodePtr bodyB,
                            const bool enableJointMotors)
    {
        MutexLockPtr lock = getScopedLock();

        // ensure dynamics nodes are created
        createDynamicsNode(bodyA);
        createDynamicsNode(bodyB);
        if (hasLink(bodyA, bodyB))
        {
            THROW_VR_EXCEPTION("Joints are already connected:" << bodyA->getName() << ","
                                                               << bodyB->getName());
        }

        BulletObjectPtr drn1 = std::dynamic_pointer_cast<BulletObject>(dynamicRobotNodes[bodyA]);
        BulletObjectPtr drn2 = std::dynamic_pointer_cast<BulletObject>(dynamicRobotNodes[bodyB]);
        VR_ASSERT(drn1);
        VR_ASSERT(drn2);
        std::shared_ptr<btRigidBody> btBody1 = drn1->getRigidBody();
        std::shared_ptr<btRigidBody> btBody2 = drn2->getRigidBody();
        VR_ASSERT(btBody1);
        VR_ASSERT(btBody2);
        DynamicsWorld::GetWorld()->getEngine()->disableCollision(drn1.get(), drn2.get());

        if (joint->getJointValue() != 0.0f)
        {
            VR_WARNING
                << joint->getName()
                << ": joint values != 0 may produce a wrong setup, setting joint value to zero"
                << std::endl;
            joint->setJointValue(0);
        }

        Eigen::Matrix4f coordSystemNode1 =
            bodyA->getGlobalPose(); // todo: what if joint is not at 0 ?!
        Eigen::Matrix4f coordSystemNode2 = bodyB->getGlobalPose();
        Eigen::Matrix4f coordSystemJoint = joint->getGlobalPose();

        Eigen::Matrix4f anchorPointGlobal =
            joint
                ->getGlobalPose(); //node1->getGlobalPose() * node2->getPreJointTransformation(); //

        Eigen::Matrix4f anchor_inNode1 = coordSystemNode1.inverse() * anchorPointGlobal;
        Eigen::Matrix4f anchor_inNode2 = coordSystemNode2.inverse() * anchorPointGlobal;


        // The bullet model was adjusted, so that origin is at local com
        // since we computed the anchor in from simox models, we must re-adjust the anchor, in order to consider the com displacement
        Eigen::Matrix4f com1;
        com1.setIdentity();
        com1.block(0, 3, 3, 1) = -drn1->getCom();
        anchor_inNode1 = com1 * anchor_inNode1;

        Eigen::Matrix4f com2;
        com2.setIdentity();
        com2.block(0, 3, 3, 1) = -drn2->getCom();
        anchor_inNode2 = com2 * anchor_inNode2;

        std::shared_ptr<btTypedConstraint> jointbt;

        double vr2bulletOffset = 0.0f;


        if (joint->isTranslationalJoint() && !ignoreTranslationalJoints)
        {
            std::shared_ptr<RobotNodePrismatic> rnPrisJoint =
                std::dynamic_pointer_cast<RobotNodePrismatic>(joint);

            double limMin, limMax;
            btScalar diff = joint->getJointValueOffset();
            limMin = (joint->getJointLimitLo() + diff) / 1000 * BulletObject::ScaleFactor; //mm -> m
            limMax = (joint->getJointLimitHi() + diff) / 1000 * BulletObject::ScaleFactor; //mm -> m

            Eigen::Vector3f directionLocal =
                rnPrisJoint->getJointTranslationDirectionJointCoordSystem();

            jointbt = createTranslationalJoint(btBody1,
                                               btBody2,
                                               coordSystemNode1,
                                               coordSystemNode2,
                                               anchor_inNode1,
                                               anchor_inNode2,
                                               directionLocal,
                                               coordSystemJoint,
                                               limMin,
                                               limMax);

            vr2bulletOffset = diff;
        }
        else if (joint->isRotationalJoint())
        {
            // create joint
            std::shared_ptr<RobotNodeRevolute> rnRevJoint =
                std::dynamic_pointer_cast<RobotNodeRevolute>(joint);

            // transform axis direction (not position!)
            Eigen::Vector4f axisLocalJoint = Eigen::Vector4f::Zero();
            axisLocalJoint.block(0, 0, 3, 1) = rnRevJoint->getJointRotationAxisInJointCoordSystem();
            Eigen::Matrix4f tmpGpJoint = coordSystemJoint;
            tmpGpJoint.block(0, 3, 3, 1).setZero(); // coordSystemJoint
            //Eigen::Vector4f axisGlobal = tmpGpJoint * axisLocalJoint;

            double limMin, limMax;
            limMin = joint->getJointLimitLo();
            limMax = joint->getJointLimitHi();

            Eigen::Vector3f axisGlobal = rnRevJoint->getJointRotationAxis();
            Eigen::Vector3f axisLocal = rnRevJoint->getJointRotationAxisInJointCoordSystem();
            btScalar limMinBT, limMaxBT;
            btScalar diff = joint->getJointValueOffset(); //startAngleBT + startAngle);
            limMinBT = btScalar(limMin) + diff; //diff - limMax;//
            limMaxBT = btScalar(limMax) + diff; //diff - limMin;//
            jointbt = createHingeJoint(btBody1,
                                       btBody2,
                                       coordSystemNode1,
                                       coordSystemNode2,
                                       anchor_inNode1,
                                       anchor_inNode2,
                                       axisGlobal,
                                       axisLocal,
                                       coordSystemJoint,
                                       limMinBT,
                                       limMaxBT);

            vr2bulletOffset = diff;
        }
        else
        {
            VR_WARNING << "Creating fixed joint between " << bodyA->getName() << " and "
                       << bodyB->getName()
                       << ". This might result in some artefacts (e.g. no strict rigid connection)"
                       << std::endl;
            // create fixed joint
            jointbt = createFixedJoint(btBody1, btBody2, anchor_inNode1, anchor_inNode2);
        }

        LinkInfo i;
        i.nodeA = bodyA;
        i.nodeB = bodyB;
        i.dynNode1 = drn1;
        i.dynNode2 = drn2;
        i.nodeJoint = joint;
        i.joint = jointbt;
        i.jointValueOffset = vr2bulletOffset;

        // activate joint feedback for translational joints (somehow, the jumping positions issue seems sometimes to be solved with this?!)
        if (joint && joint->isTranslationalJoint() && !ignoreTranslationalJoints)
        {
            enableForceTorqueFeedback(i);
        }

        // disable col model
        i.disabledCollisionPairs.push_back(std::pair<DynamicsObjectPtr, DynamicsObjectPtr>(
            std::dynamic_pointer_cast<DynamicsObject>(drn1),
            std::dynamic_pointer_cast<DynamicsObject>(drn2)));

        links.push_back(i);
#ifndef DEBUG_FIXED_OBJECTS

        if (enableJointMotors && (joint->isRotationalJoint() ||
                                  (joint->isTranslationalJoint() && !ignoreTranslationalJoints)))
        {
            // start standard actuator
            actuateNode(joint, joint->getJointValue());
        }
        else
        {
            addJointFriction(joint);
        }

#endif
    }

    bool
    BulletRobot::hasLink(VirtualRobot::RobotNodePtr node1, VirtualRobot::RobotNodePtr node2)
    {
        MutexLockPtr lock = getScopedLock();

        for (auto& link : links)
        {
            if (link.nodeA == node1 && link.nodeB == node2)
            {
                return true;
            }
        }

        return false;
    }

    std::vector<BulletRobot::LinkInfo>
    BulletRobot::getLinks()
    {
        return links;
    }

    void
    BulletRobot::ensureKinematicConstraints()
    {
        // results in strange behavior?!
    }

    void
    BulletRobot::actuateJoints(double dt)
    {
        MutexLockPtr lock = getScopedLock();
        //cout << "=== === BulletRobot: actuateJoints() 1 === " << this << std::endl;

        //int jointCounter = 0;
        //cout << "**** Control Values: ";

        for (auto& pair : actuationTargets)
        {
            VirtualRobot::RobotNodePtr const& node = pair.first;
            robotNodeActuationTarget& target = pair.second;

            //cout << "it:" << it->first << ", name: " << it->first->getName() << std::endl;
            VelocityMotorController& controller = actuationControllers[node];

            if (!target.node->isRotationalJoint() && !target.node->isTranslationalJoint())
            {
                continue;
            }

            LinkInfo link = getLink(target.node);

            const ActuationMode& actuation = target.actuation;

            // CHECK FOR DISABLED MOTORS
            if (actuation.mode == 0)
            {
                if (target.node->isRotationalJoint())
                {
                    std::shared_ptr<btHingeConstraint> hinge =
                        std::dynamic_pointer_cast<btHingeConstraint>(link.joint);
                    hinge->enableMotor(false);
                    continue;
                }
                else if (target.node->isTranslationalJoint() && !ignoreTranslationalJoints)
                {
                    std::shared_ptr<btSliderConstraint> slider =
                        std::dynamic_pointer_cast<btSliderConstraint>(link.joint);
                    slider->setPoweredLinMotor(false);
                    continue;
                }
            }


            if (actuation.modes.torque)
            {
                // TORQUE MODES
                double torque = target.jointTorqueTarget;

                // node->getMaxTorque is -1 if it is not set at all
                // => Just ignore the torque limit in this case
                float maxTorque = node->getMaxTorque();
                if (maxTorque > 0)
                {
                    if (torque < -maxTorque)
                    {
                        torque = -maxTorque;
                    }
                    if (torque > maxTorque)
                    {
                        torque = maxTorque;
                    }
                }

                if (target.node->isRotationalJoint())
                {
                    btHingeConstraint& hinge = dynamic_cast<btHingeConstraint&>(*link.joint);
                    btVector3 hingeAxisLocalA = hinge.getFrameOffsetA().getBasis().getColumn(2);
                    btVector3 hingeAxisLocalB = hinge.getFrameOffsetB().getBasis().getColumn(2);
                    btVector3 hingeAxisWorldA =
                        hinge.getRigidBodyA().getWorldTransform().getBasis() * hingeAxisLocalA;
                    btVector3 hingeAxisWorldB =
                        hinge.getRigidBodyB().getWorldTransform().getBasis() * hingeAxisLocalB;

                    btVector3 hingeTorqueA = -torque * hingeAxisWorldA;
                    btVector3 hingeTorqueB = torque * hingeAxisWorldB;
                    hinge.enableMotor(false);
                    hinge.getRigidBodyA().applyTorque(hingeTorqueA);
                    hinge.getRigidBodyB().applyTorque(hingeTorqueB);
                }
                else
                {
                    // The slider constraint works along the local x-axis
                    // So we have to apply force along the column 0 of the frame
                    btSliderConstraint& slider = dynamic_cast<btSliderConstraint&>(*link.joint);
                    btVector3 sliderAxisLocalA = slider.getFrameOffsetA().getBasis().getColumn(0);
                    btVector3 sliderAxisLocalB = slider.getFrameOffsetB().getBasis().getColumn(0);
                    btVector3 sliderAxisWorldA =
                        slider.getRigidBodyA().getWorldTransform().getBasis() * sliderAxisLocalA;
                    btVector3 sliderAxisWorldB =
                        slider.getRigidBodyB().getWorldTransform().getBasis() * sliderAxisLocalB;

                    btVector3 sliderTorqueA = -torque * sliderAxisWorldA;
                    btVector3 sliderTorqueB = torque * sliderAxisWorldB;
                    slider.setPoweredLinMotor(false);
                    slider.btTypedConstraint::getRigidBodyA().applyCentralForce(sliderTorqueA);
                    slider.btTypedConstraint::getRigidBodyB().applyCentralForce(sliderTorqueB);
                }
            }
            else if (actuation.modes.position || actuation.modes.velocity)
            {
                // POSITION, VELOCITY OR POSITION&VELOCITY MODE

                btScalar velActual = btScalar(getJointSpeed(node));
                btScalar velocityTarget = btScalar(target.jointVelocityTarget);

                if (actuation.modes.velocity && !actuation.modes.position)
                {
                    // bullet is buggy here and cannot reach velocity targets for some joints, use a position-velocity mode as workaround
                    target.jointValueTarget += velocityTarget * dt;
                }

                btScalar posTarget = btScalar(target.jointValueTarget + link.jointValueOffset);
                btScalar posActual = btScalar(getJointAngle(node));
                controller.setName(node->getName());

                double targetVelocity;
                float deltaPos = target.node->getDelta(posTarget);
                if (target.node->isTranslationalJoint())
                {
                    posTarget *= 0.001;
                    posActual *= 0.001;
                    velActual *= 0.001;
                    velocityTarget *= 0.001;
                    deltaPos *= 0.001f;
                }
                if (!actuation.modes.position)
                {
                    // we always use position or position-velocity mode
                    ActuationMode tempAct = actuation;
                    tempAct.modes.position = 1;
                    targetVelocity = controller.update(
                        /*posTarget - posActual*/ deltaPos, velocityTarget, tempAct, btScalar(dt));
                }
                else
                {
                    targetVelocity = controller.update(/*posTarget - posActual*/ deltaPos,
                                                       velocityTarget,
                                                       actuation,
                                                       btScalar(dt));
                }
                /*if (it->second.node->getName() == "ArmL6_Elb2")
                {
                    std::cout << "ELBOW - pos act:" << posActual << ",\t posTarget: " << posTarget << ",\t delta: " << deltaPos << ",\t , vel target:" << velocityTarget << ",\t pid vel:" << targetVelocity << std::endl;
                }*/

                btScalar maxImpulse = bulletMaxMotorImulse;
                //                controller.setCurrentVelocity(velActual);
                if (target.node->getMaxTorque() > 0)
                {
                    maxImpulse = target.node->getMaxTorque() * btScalar(dt) *
                                 BulletObject::ScaleFactor * BulletObject::ScaleFactor *
                                 BulletObject::ScaleFactor;
                    //cout << "node:" << it->second.node->getName() << ", max impulse: " << maxImpulse << ", dt:" << dt << ", maxImp:" << it->second.node->getMaxTorque() << std::endl;
                }
                if (fabs(targetVelocity) > 0.00001)
                {
                    link.dynNode1->getRigidBody()->activate();
                    link.dynNode2->getRigidBody()->activate();
                }
                if (target.node->isRotationalJoint())
                {
                    std::shared_ptr<btHingeConstraint> hinge =
                        std::dynamic_pointer_cast<btHingeConstraint>(link.joint);
                    hinge->enableAngularMotor(true, btScalar(targetVelocity), maxImpulse);
                }
                else if (target.node->isTranslationalJoint() && !ignoreTranslationalJoints)
                {
                    std::shared_ptr<btSliderConstraint> slider =
                        std::dynamic_pointer_cast<btSliderConstraint>(link.joint);
                    slider->setMaxLinMotorForce(maxImpulse * 1000); // Magic number!!!
                    slider->setTargetLinMotorVelocity(btScalar(targetVelocity));
                    slider->setPoweredLinMotor(true);
                }
            }
        }

        //cout << std::endl;
        setPoseNonActuatedRobotNodes();
    }

    void
    BulletRobot::updateSensors(double dt)
    {
        MutexLockPtr lock = getScopedLock();
        std::unordered_set<std::string> contactObjectNames;

        // this seems stupid and it is, but that is abstract interfaces for you.
        for (auto& sensor : sensors)
        {
            ContactSensorPtr contactSensor = std::dynamic_pointer_cast<ContactSensor>(sensor);

            if (contactSensor)
            {
                contactObjectNames.insert(contactSensor->getRobotNode()->getName());
            }
        }

        DynamicsWorldPtr world = DynamicsWorld::GetWorld();
        std::vector<SimDynamics::DynamicsEngine::DynamicsContactInfo> contacts =
            world->getEngine()->getContacts();
        std::unordered_map<std::string, VirtualRobot::ContactSensor::ContactFrame> frameMap;

        for (auto& contact : contacts)
        {
            if (contact.objectAName.empty() || contact.objectBName.empty())
            {
                continue;
            }

            float sign;
            std::string key;
            std::string contactBody;

            if (contactObjectNames.find(contact.objectAName) != contactObjectNames.end())
            {
                sign = 1.0f;
                key = contact.objectAName;
                contactBody = contact.objectBName;
            }
            else if (contactObjectNames.find(contact.objectBName) != contactObjectNames.end())
            {
                sign = -1.0f;
                key = contact.objectBName;
                contactBody = contact.objectAName;
            }
            else
            {
                continue;
            }

            VirtualRobot::ContactSensor::ContactFrame& frame = frameMap[key];
            double zForce = sign * contact.normalGlobalB.z() * contact.appliedImpulse;
            VirtualRobot::ContactSensor::ContactForce cf;
            cf.contactPoint = contact.posGlobalB;
            cf.zForce = zForce;
            cf.bodyName = contactBody;
            frame.forces.push_back(cf);
        }

        // Update forces and torques
        for (auto& sensor : sensors)
        {
            ForceTorqueSensorPtr ftSensor = std::dynamic_pointer_cast<ForceTorqueSensor>(sensor);

            if (ftSensor)
            {
                VirtualRobot::RobotNodePtr node = ftSensor->getRobotNode();
                THROW_VR_EXCEPTION_IF(!node, "parent of sensor could not be casted to RobotNode");

                if (hasLink(node))
                {
                    const LinkInfo& link = getLink(node);
                    Eigen::VectorXf forceTorques = getJointForceTorqueGlobal(link);
                    ftSensor->updateSensors(forceTorques);
                }
            }
            else
            {
                ContactSensorPtr contactSensor = std::dynamic_pointer_cast<ContactSensor>(sensor);

                if (contactSensor)
                {
                    VirtualRobot::RobotNodePtr node = contactSensor->getRobotNode();
                    THROW_VR_EXCEPTION_IF(!node,
                                          "parent of sensor could not be casted to RobotNode")
                    const VirtualRobot::ContactSensor::ContactFrame& frame =
                        frameMap[node->getName()];
                    contactSensor->updateSensors(frame, dt);
                }
            }
        }
    }

    BulletRobot::LinkInfo
    BulletRobot::getLink(VirtualRobot::RobotNodePtr node)
    {
        MutexLockPtr lock = getScopedLock();

        for (auto& link : links)
        {
            if (link.nodeJoint == node /*|| links[i].nodeJoint2 == node*/)
            {
                return link;
            }
        }

        THROW_VR_EXCEPTION("No link with node " << node->getName());
        return LinkInfo();
    }

    BulletRobot::LinkInfo
    BulletRobot::getLink(BulletObjectPtr object1, BulletObjectPtr object2)
    {
        MutexLockPtr lock = getScopedLock();

        for (auto& link : links)
        {
            if ((link.dynNode1 == object1 && link.dynNode2 == object2) ||
                (link.dynNode1 == object2 && link.dynNode2 == object1))
            {
                return link;
            }
        }

        VR_WARNING << "No link with nodes: " << object1->getName() << " and " << object2->getName()
                   << std::endl;
        return LinkInfo();
    }

    std::vector<BulletRobot::LinkInfo>
    BulletRobot::getLinks(VirtualRobot::RobotNodePtr node)
    {
        MutexLockPtr lock = getScopedLock();
        std::vector<BulletRobot::LinkInfo> result;

        for (auto& link : links)
        {
            if (link.nodeJoint == node /*|| links[i].nodeJoint2 == node*/ || link.nodeA == node ||
                link.nodeB == node)
            {
                result.push_back(link);
            }
        }

        return result;
    }

    std::vector<BulletRobot::LinkInfo>
    BulletRobot::getLinks(BulletObjectPtr node)
    {
        MutexLockPtr lock = getScopedLock();
        std::vector<BulletRobot::LinkInfo> result;

        for (auto& link : links)
        {
            if (link.dynNode1 == node || link.dynNode2 == node)
            {
                result.push_back(link);
            }
        }

        return result;
    }

    bool
    BulletRobot::attachObject(const string& nodeName, DynamicsObjectPtr object)
    {
        BulletRobot::LinkInfoPtr li = attachObjectLink(nodeName, object);
        return bool(li);
    }

    BulletRobot::LinkInfoPtr
    BulletRobot::attachObjectLink(const string& nodeName, DynamicsObjectPtr object)
    {
        BulletRobot::LinkInfoPtr result;

        if (!robot || !robot->hasRobotNode(nodeName))
        {
            VR_ERROR << "no node with name " << nodeName << std::endl;
            return result;
        }

        if (!object)
        {
            VR_ERROR << "no object " << std::endl;
            return result;
        }

        BulletObjectPtr bo = std::dynamic_pointer_cast<BulletObject>(object);

        if (!bo)
        {
            VR_ERROR << "no bullet object " << std::endl;
            return result;
        }

        RobotNodePtr nodeA = robot->getRobotNode(nodeName);

        DynamicsObjectPtr drn = getDynamicsRobotNode(nodeA);

        if (!drn)
        {
            while (nodeA && !drn)
            {
                SceneObjectPtr ts = nodeA->getParent();
                nodeA = std::dynamic_pointer_cast<RobotNode>(ts);

                if (nodeA)
                {
                    drn = getDynamicsRobotNode(nodeA);
                }
            }

            if (!drn)
            {
                VR_ERROR << "No dynamics object..." << std::endl;
                return result;
            }
        }

        BulletObjectPtr bdrn = std::dynamic_pointer_cast<BulletObject>(drn);

        if (!bo)
        {
            VR_ERROR << "no bullet robot object " << std::endl;
            return result;
        }

        // create bullet joint
        std::shared_ptr<btRigidBody> btBody1 = bdrn->getRigidBody();
        std::shared_ptr<btRigidBody> btBody2 = bo->getRigidBody();

        Eigen::Matrix4f coordSystemNode1 =
            bdrn->getComGlobal(); // todo: what if joint is not at 0 ?!
        Eigen::Matrix4f coordSystemNode2 = bo->getComGlobal();
        //Eigen::Matrix4f coordSystemJoint = bdrn->getComGlobal();

        Eigen::Matrix4f anchorPointGlobal =
            bdrn->getComGlobal(); //node1->getGlobalPose() * node2->getPreJointTransformation(); //

        Eigen::Matrix4f anchor_inNode1 = coordSystemNode1.inverse() * anchorPointGlobal;
        Eigen::Matrix4f anchor_inNode2 = coordSystemNode2.inverse() * anchorPointGlobal;


        // The bullet model was adjusted, so that origin is at local com
        // since we computed the anchor in from simox models, we must re-adjust the anchor, in order to consider the com displacement
        /*Eigen::Matrix4f com1;
        com1.setIdentity();
        com1.block(0,3,3,1) = -drn1->getCom();
        anchor_inNode1 = com1 * anchor_inNode1;

        Eigen::Matrix4f com2;
        com2.setIdentity();
        com2.block(0,3,3,1) = -drn2->getCom();
        anchor_inNode2 = com2 * anchor_inNode2;*/

        std::shared_ptr<btTypedConstraint> jointbt =
            createFixedJoint(btBody1, btBody2, anchor_inNode1, anchor_inNode2);

        result.reset(new LinkInfo());
        result->nodeA = nodeA;
        //i.nodeB = ;
        result->dynNode1 = bdrn;
        result->dynNode2 = bo;
        result->nodeJoint = nodeA;
        //i.nodeJoint2 = joint2;
        result->joint = jointbt;
        result->jointValueOffset = 0;

        // disable col model
        result->disabledCollisionPairs.push_back(
            std::pair<DynamicsObjectPtr, DynamicsObjectPtr>(drn, object));

        links.push_back(*result);

        VR_INFO << "Attached object " << object->getName() << " to node " << nodeName << std::endl;
        return result;
    }

    bool
    BulletRobot::detachObject(DynamicsObjectPtr object)
    {
        BulletRobot::LinkInfoPtr result;

        if (!robot)
        {
            VR_ERROR << "no robot " << std::endl;
            return false;
        }

        if (!object)
        {
            VR_ERROR << "no object " << std::endl;
            return false;
        }

        BulletObjectPtr bo = std::dynamic_pointer_cast<BulletObject>(object);

        if (!bo)
        {
            VR_ERROR << "no bullet object " << std::endl;
            return false;
        }

        bool res = true;
        std::vector<LinkInfo> ls = getLinks(bo);

        if (ls.size() == 0)
        {
            VR_ERROR << "No link with object " << object->getName() << std::endl;
            return true; // not a failure, object is not attached
        }

        for (const auto& l : ls)
        {
            res = res & removeLink(l);
        }

        VR_INFO << "Detached object " << object->getName() << " from robot " << robot->getName()
                << std::endl;
        return res;
    }

    bool
    BulletRobot::hasLink(VirtualRobot::RobotNodePtr node)
    {
        MutexLockPtr lock = getScopedLock();

        for (auto& link : links)
        {
            if (link.nodeJoint == node /*|| links[i].nodeJoint2 == node*/)
            {
                return true;
            }
        }

        return false;
    }

    void
    BulletRobot::actuateNode(VirtualRobot::RobotNodePtr node, double jointValue)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(node);

        if (node->isRotationalJoint() ||
            (node->isTranslationalJoint() && !ignoreTranslationalJoints))
        {
            if (!hasLink(node))
            {
                VR_ERROR << "No link for node " << node->getName() << std::endl;
                return;
            }

            DynamicsRobot::actuateNode(node, jointValue);
        }
        else
        {
            if (node->isTranslationalJoint() && ignoreTranslationalJoints)
            {
                VR_WARNING << "Translational joints ignored. (ignoring node " << node->getName()
                           << ")." << std::endl;
            }
            else
            {
                VR_ERROR
                    << "Only Revolute and translational joints implemented so far (ignoring node "
                    << node->getName() << ")." << std::endl;
            }
        }
    }

    void
    BulletRobot::addJointFriction(VirtualRobot::RobotNodePtr node)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(node);

        if (node->isRotationalJoint() ||
            (node->isTranslationalJoint() && !ignoreTranslationalJoints))
        {
            if (!hasLink(node))
            {
                VR_ERROR << "No link for node " << node->getName() << std::endl;
                return;
            }

            DynamicsRobot::addJointFriction(node);
        }
        else
        {
            if (node->isTranslationalJoint() && ignoreTranslationalJoints)
            {
                VR_WARNING << "Translational joints ignored. (ignoring node " << node->getName()
                           << ")." << std::endl;
            }
            else
            {
                VR_ERROR
                    << "Only Revolute and translational joints implemented so far (ignoring node "
                    << node->getName() << ")." << std::endl;
            }
        }
    }

    void
    BulletRobot::actuateNodeVel(RobotNodePtr node, double jointVelocity)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(node);

        if (!hasLink(node))
        {
            VR_ERROR << "No link for node " << node->getName() << std::endl;
            return;
        }

        LinkInfo link = getLink(node);

        if (node->isRotationalJoint() || node->isTranslationalJoint())
        {
            DynamicsRobot::actuateNodeVel(node,
                                          jointVelocity); // inverted joint direction in bullet
        }
        else
        {
            VR_ERROR << "Only Revolute and Prismatic joints implemented so far (node: "
                     << node->getName() << ")..." << std::endl;
        }
    }

    double
    BulletRobot::getJointAngle(VirtualRobot::RobotNodePtr rn)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(rn);

        if (!hasLink(rn))
        {
            //VR_ERROR << "No link with node " << rn->getName() << std::endl;
            return 0.0f;
        }

        LinkInfo link = getLink(rn);

        if (rn->isRotationalJoint())
        {
            std::shared_ptr<btHingeConstraint> hinge =
                std::dynamic_pointer_cast<btHingeConstraint>(link.joint);
            VR_ASSERT(hinge);

            return (hinge->getHingeAngle() -
                    link.jointValueOffset); // inverted joint direction in bullet
        }
        else if (rn->isTranslationalJoint())
        {
            std::shared_ptr<btSliderConstraint> slider =
                std::dynamic_pointer_cast<btSliderConstraint>(link.joint);
            VR_ASSERT(slider);

            return (slider->getLinearPos() - link.jointValueOffset) * 1000 /
                   BulletObject::ScaleFactor; // m -> mm
        }
        else
        {
            return 0.0;
        }
    }

    double
    BulletRobot::getJointTargetSpeed(VirtualRobot::RobotNodePtr rn)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(rn);

        if (!hasLink(rn))
        {
            //VR_ERROR << "No link with node " << rn->getName() << std::endl;
            return 0.0f;
        }

        LinkInfo link = getLink(rn);

        if (rn->isRotationalJoint())
        {
            std::shared_ptr<btHingeConstraint> hinge =
                std::dynamic_pointer_cast<btHingeConstraint>(link.joint);
#ifdef SIMOX_USES_OLD_BULLET
            return hinge->getMotorTargetVelosity();
#else
            return hinge->getMotorTargetVelocity();
#endif
        }
        else if (rn->isTranslationalJoint())
        {
            std::shared_ptr<btSliderConstraint> slider =
                std::dynamic_pointer_cast<btSliderConstraint>(link.joint);

            return slider->getTargetLinMotorVelocity() *
                   1000; // / BulletObject::ScaleFactor; m -> mm
        }
        return 0;
    }

    double
    BulletRobot::getJointSpeed(VirtualRobot::RobotNodePtr rn)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(rn);

        if (!hasLink(rn))
        {
            //VR_ERROR << "No link with node " << rn->getName() << std::endl;
            return 0.0f;
        }

        if (rn->isRotationalJoint())
        {
            LinkInfo link = getLink(rn);
            std::shared_ptr<btHingeConstraint> hinge =
                std::dynamic_pointer_cast<btHingeConstraint>(link.joint);

            VR_ASSERT(hinge);

            std::shared_ptr<RobotNodeRevolute> rnRevJoint =
                std::dynamic_pointer_cast<RobotNodeRevolute>(link.nodeJoint);

            Eigen::Vector3f deltaVel =
                link.dynNode2->getAngularVelocity() - link.dynNode1->getAngularVelocity();
            double speed = deltaVel.dot(rnRevJoint->getJointRotationAxis());
            return speed; //hinge->getMotorTargetVelosity();
        }
        else if (rn->isTranslationalJoint())
        {
            LinkInfo link = getLink(rn);
            std::shared_ptr<btSliderConstraint> slider =
                std::dynamic_pointer_cast<btSliderConstraint>(link.joint);

            VR_ASSERT(slider);

            std::shared_ptr<RobotNodePrismatic> rnPrisJoint =
                std::dynamic_pointer_cast<RobotNodePrismatic>(link.nodeJoint);

            Eigen::Vector3f jointDirection =
                rnPrisJoint->getGlobalPose().block<3, 3>(0, 0) *
                rnPrisJoint->getJointTranslationDirectionJointCoordSystem();
            Eigen::Vector3f deltaVel =
                (link.dynNode2->getLinearVelocity() - link.dynNode1->getLinearVelocity());
            return jointDirection.dot(deltaVel) / jointDirection.norm();
        }
        else
        {
            VR_WARNING << "Only translational and rotational joints implemented." << std::endl;
            return 0.0;
        }
    }

    double
    BulletRobot::getNodeTarget(VirtualRobot::RobotNodePtr node)
    {
        MutexLockPtr lock = getScopedLock();
        return DynamicsRobot::getNodeTarget(node);
    }

    Eigen::Vector3f
    BulletRobot::getJointTorques(RobotNodePtr rn)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(rn);
        Eigen::Vector3f result;
        result.setZero();

        if (!hasLink(rn))
        {
            //VR_ERROR << "No link with node " << rn->getName() << std::endl;
            return result;
        }

        LinkInfo link = getLink(rn);

        if (rn->isRotationalJoint() || rn->isTranslationalJoint())
        {
            enableForceTorqueFeedback(link, true);
            result = getJointForceTorqueGlobal(link).tail(3);
        }
        else
        {
            VR_WARNING << "Only translational and rotational joints implemented." << std::endl;
        }

        return result;
    }

    double
    BulletRobot::getJointTorque(RobotNodePtr rn)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(rn);

        if (!hasLink(rn))
        {
            //VR_ERROR << "No link with node " << rn->getName() << std::endl;
            return 0.0;
        }

        LinkInfo link = getLink(rn);

        if (rn->isRotationalJoint() || rn->isTranslationalJoint())
        {
            enableForceTorqueFeedback(link, true);
            Eigen::Vector3f torqueVector = getJointForceTorqueGlobal(link).tail(3);

            // project onto joint axis
            double troque =
                (torqueVector.adjoint() * link.nodeJoint->getGlobalPose().block(0, 2, 3, 1))(0, 0);
            return troque;
        }
        else
        {
            VR_WARNING << "Only translational and rotational joints implemented." << std::endl;
        }
        return 0.0;
    }

    Eigen::Vector3f
    BulletRobot::getJointForces(RobotNodePtr rn)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(rn);
        Eigen::Vector3f result;
        result.setZero();

        if (!hasLink(rn))
        {
            //VR_ERROR << "No link with node " << rn->getName() << std::endl;
            return result;
        }

        LinkInfo link = getLink(rn);

        if (rn->isRotationalJoint() || rn->isTranslationalJoint())
        {
            enableForceTorqueFeedback(link, true);
            result = getJointForceTorqueGlobal(link).head(3);
        }
        else
        {
            VR_WARNING << "Only translational and rotational joints implemented." << std::endl;
        }

        return result;
    }

    Eigen::Matrix4f
    BulletRobot::getComGlobal(const VirtualRobot::RobotNodePtr& rn)
    {
        MutexLockPtr lock = getScopedLock();
        BulletObjectPtr bo = std::dynamic_pointer_cast<BulletObject>(getDynamicsRobotNode(rn));

        if (!bo)
        {
            VR_ERROR << "Could not cast object..." << std::endl;
            return Eigen::Matrix4f::Identity();
        }

        return bo->getComGlobal();
    }

    Eigen::Vector3f
    BulletRobot::getComGlobal(const VirtualRobot::RobotNodeSetPtr& set)
    {
        MutexLockPtr lock = getScopedLock();
        Eigen::Vector3f com = Eigen::Vector3f::Zero();
        double totalMass = 0.0;

        for (unsigned int i = 0; i < set->getSize(); i++)
        {
            VirtualRobot::RobotNodePtr node = (*set)[i];
            BulletObjectPtr bo =
                std::dynamic_pointer_cast<BulletObject>(getDynamicsRobotNode(node));
            Eigen::Matrix4f pose = bo->getComGlobal();
            com += node->getMass() * pose.block(0, 3, 3, 1);
            totalMass += node->getMass();
        }

        com *= float(1.0f / totalMass);
        return com;
    }

    Eigen::Vector3f
    BulletRobot::getComVelocityGlobal(const VirtualRobot::RobotNodeSetPtr& set)
    {
        MutexLockPtr lock = getScopedLock();
        Eigen::Vector3f com = Eigen::Vector3f::Zero();
        double totalMass = 0.0;

        for (unsigned int i = 0; i < set->getSize(); i++)
        {
            VirtualRobot::RobotNodePtr node = (*set)[i];
            BulletObjectPtr bo =
                std::dynamic_pointer_cast<BulletObject>(getDynamicsRobotNode(node));
            Eigen::Vector3f vel = bo->getLinearVelocity();

            if (std::isnan(vel(0)) || std::isnan(vel(1)) || std::isnan(vel(2)))
            {
                VR_ERROR << "NAN result: getLinearVelocity:" << bo->getName() << ", i:" << i
                         << std::endl;
                node->print();
                VR_ERROR << "BULLETOBJECT com:" << bo->getCom() << std::endl;
                VR_ERROR << "BULLETOBJECT: ang vel:" << bo->getAngularVelocity() << std::endl;
                VR_ERROR << "BULLETOBJECT:" << bo->getRigidBody()->getWorldTransform().getOrigin()
                         << std::endl;
            }

            com += node->getMass() * vel;
            totalMass += node->getMass();
        }

        if (fabs(totalMass) < 1e-5)
        {
            VR_ERROR << "Little mass: " << totalMass << ". Could not compute com velocity..."
                     << std::endl;
        }
        else
        {
            com *= float(1.0f / totalMass);
        }

        return com;
    }

    Eigen::Vector3f
    BulletRobot::getLinearMomentumGlobal(const VirtualRobot::RobotNodeSetPtr& set)
    {
        MutexLockPtr lock = getScopedLock();
        Eigen::Vector3f linMomentum = Eigen::Vector3f::Zero();

        for (unsigned int i = 0; i < set->getSize(); i++)
        {
            VirtualRobot::RobotNodePtr node = (*set)[i];
            BulletObjectPtr bo =
                std::dynamic_pointer_cast<BulletObject>(getDynamicsRobotNode(node));
            Eigen::Vector3f vel = bo->getLinearVelocity() / 1000.0 * BulletObject::ScaleFactor;

            linMomentum += node->getMass() * vel;
        }

        return linMomentum;
    }

    Eigen::Vector3f
    BulletRobot::getAngularMomentumGlobal(const VirtualRobot::RobotNodeSetPtr& set)
    {
        MutexLockPtr lock = getScopedLock();
        Eigen::Vector3f angMomentum = Eigen::Vector3f::Zero();

        for (unsigned int i = 0; i < set->getSize(); i++)
        {
            VirtualRobot::RobotNodePtr node = (*set)[i];
            BulletObjectPtr bo =
                std::dynamic_pointer_cast<BulletObject>(getDynamicsRobotNode(node));
            Eigen::Vector3f vel = bo->getLinearVelocity() / 1000.0 * BulletObject::ScaleFactor;
            Eigen::Vector3f ang = bo->getAngularVelocity() / 1000.0 * BulletObject::ScaleFactor;
            Eigen::Vector3f com =
                bo->getComGlobal().block(0, 3, 3, 1) / 1000.0 * BulletObject::ScaleFactor;
            double mass = node->getMass();

            std::shared_ptr<btRigidBody> body = bo->getRigidBody();
            Eigen::Matrix3f intertiaWorld =
                BulletEngine::getRotMatrix(body->getInvInertiaTensorWorld())
                    .inverse()
                    .block(0, 0, 3, 3);

            angMomentum += com.cross(mass * vel) + intertiaWorld * ang;
        }

        return angMomentum;
    }

    Eigen::Vector3f
    BulletRobot::getAngularMomentumLocal(const VirtualRobot::RobotNodeSetPtr& set)
    {
        MutexLockPtr lock = getScopedLock();
        Eigen::Vector3f angMomentum = Eigen::Vector3f::Zero();
        Eigen::Vector3f com = getComGlobal(set) / 1000.0 * BulletObject::ScaleFactor;
        Eigen::Vector3f comVel = getComVelocityGlobal(set) / 1000;

        for (unsigned int i = 0; i < set->getSize(); i++)
        {
            VirtualRobot::RobotNodePtr node = (*set)[i];
            BulletObjectPtr bo =
                std::dynamic_pointer_cast<BulletObject>(getDynamicsRobotNode(node));
            Eigen::Vector3f bodyVel = bo->getLinearVelocity() / 1000.0 * BulletObject::ScaleFactor;
            Eigen::Vector3f ang = bo->getAngularVelocity() / 1000.0 * BulletObject::ScaleFactor;
            Eigen::Vector3f bodyCoM =
                bo->getComGlobal().block(0, 3, 3, 1) / 1000.0 * BulletObject::ScaleFactor;
            double mass = node->getMass();

            std::shared_ptr<btRigidBody> body = bo->getRigidBody();

            btVector3 invIntertiaDiag = body->getInvInertiaDiagLocal();
            Eigen::Matrix3f intertiaLocal = Eigen::Matrix3f::Zero();
            intertiaLocal(0, 0) = 1 / invIntertiaDiag.getX();
            intertiaLocal(1, 1) = 1 / invIntertiaDiag.getY();
            intertiaLocal(2, 2) = 1 / invIntertiaDiag.getZ();

            angMomentum += mass * (bodyCoM - com).cross(bodyVel - comVel) + intertiaLocal * ang;
        }

        return angMomentum;
    }

    void
    BulletRobot::setPoseNonActuatedRobotNodes()
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(robot);
        std::vector<RobotNodePtr> rns = robot->getRobotNodes();
        std::vector<RobotNodePtr> actuatedNodes;
        std::vector<RobotNodePtr> notActuatedNodes;
        size_t i;

        // only objects with collisionmodel are processed by bullet
        for (i = 0; i < rns.size(); i++)
        {
            if (rns[i]->getCollisionModel())
            {
                actuatedNodes.push_back(rns[i]);
            }
            else
            {
                notActuatedNodes.push_back(rns[i]);
            }
        }

        size_t lastSize = notActuatedNodes.size();

        while (notActuatedNodes.size() > 0)
        {
            vector<RobotNodePtr>::iterator it = notActuatedNodes.begin();

            while (it != notActuatedNodes.end())
            {
                SceneObjectPtr parent = (*it)->getParent();

                if (!parent ||
                    find(actuatedNodes.begin(), actuatedNodes.end(), parent) != actuatedNodes.end())
                {
                    // parent is at correct pose, we can update *it
                    if (parent)
                    {
                        (*it)->updatePose(false);
                    }

                    // if root, we also have to delete node from list
                    actuatedNodes.push_back(*it);
                    it = notActuatedNodes.erase(it);
                }
                else
                {
                    it++;
                }
            }


            // just a sanity check
            if (lastSize == notActuatedNodes.size())
            {
                VR_ERROR << "Internal error?!" << std::endl;
                return;
            }
            else
            {
                lastSize = notActuatedNodes.size();
            }
        }
    }

    void
    BulletRobot::enableForceTorqueFeedback(const LinkInfo& link, bool enable)
    {
        MutexLockPtr lock = getScopedLock();

        if (!link.joint->needsFeedback() && enable)
        {
            link.joint->enableFeedback(true);
            btJointFeedback* feedback = new btJointFeedback;
            feedback->m_appliedForceBodyA = btVector3(0, 0, 0);
            feedback->m_appliedForceBodyB = btVector3(0, 0, 0);
            feedback->m_appliedTorqueBodyA = btVector3(0, 0, 0);
            feedback->m_appliedTorqueBodyB = btVector3(0, 0, 0);
            link.joint->setJointFeedback(feedback);
        }
        else if (link.joint->needsFeedback() && !enable)
        {
            link.joint->enableFeedback(false);
        }
    }

    Eigen::VectorXf
    BulletRobot::getForceTorqueFeedbackA(const LinkInfo& link)
    {
        MutexLockPtr lock = getScopedLock();
        Eigen::VectorXf r(6);
        r.setZero();

        if (!link.joint || !link.joint->needsFeedback())
        {
            return r;
        }

        btJointFeedback* feedback = link.joint->getJointFeedback();

        if (!feedback)
        {
            return r;
        }

        r << feedback->m_appliedForceBodyA[0], feedback->m_appliedForceBodyA[1],
            feedback->m_appliedForceBodyA[2], feedback->m_appliedTorqueBodyA[0],
            feedback->m_appliedTorqueBodyA[1], feedback->m_appliedTorqueBodyA[2];
        return r;
    }

    Eigen::VectorXf
    BulletRobot::getForceTorqueFeedbackB(const LinkInfo& link)
    {
        MutexLockPtr lock = getScopedLock();
        Eigen::VectorXf r(6);
        r.setZero();

        if (!link.joint || !link.joint->needsFeedback())
        {
            return r;
        }

        btJointFeedback* feedback = link.joint->getJointFeedback();

        if (!feedback)
        {
            return r;
        }

        r << feedback->m_appliedForceBodyB[0], feedback->m_appliedForceBodyB[1],
            feedback->m_appliedForceBodyB[2], feedback->m_appliedTorqueBodyB[0],
            feedback->m_appliedTorqueBodyB[1], feedback->m_appliedTorqueBodyB[2];
        return r;
    }

    Eigen::VectorXf
    BulletRobot::getJointForceTorqueGlobal(const BulletRobot::LinkInfo& link)
    {
        MutexLockPtr lock = getScopedLock();
        Eigen::VectorXf ftA = getForceTorqueFeedbackA(link);
        Eigen::VectorXf ftB = getForceTorqueFeedbackB(link);

        Eigen::Vector3f jointGlobal = link.nodeJoint->getGlobalPose().block(0, 3, 3, 1);
        Eigen::Vector3f comBGlobal = link.nodeB->getCoMGlobal();

        // force that is applied on objectA by objectB -> so the force that object B applies on the joint
        Eigen::Vector3f forceOnBGlobal = ftB.head(3);

        Eigen::Vector3f torqueBGlobal = ftB.tail(3);

        // the lever from Object B CoM to Joint
        Eigen::Vector3f leverOnJoint =
            (comBGlobal - jointGlobal) * 0.001f * BulletObject::ScaleFactor;
        // Calculate the torque in Joint by taking the torque that presses on the CoM of BodyB and the Torque of BodyB on the joint
        // forceOnBGlobal is inverted in next line because it is the force of A on B to hold it in position
        // torqueBGlobal is inverted in next line because it is the torque on B from A to compensate torque of other objects (which is the torque we would like) to hold it in place and therefore needs to be inverted as well
        Eigen::Vector3f torqueJointGlobal =
            (leverOnJoint).cross(-forceOnBGlobal) + (-1) * torqueBGlobal;
        Eigen::VectorXf result(6);
        result.head(3) = ftA.head(3); // force in joint is same as force on CoM of A
        result.tail(3) = torqueJointGlobal;
        return result / BulletObject::MassFactor;
    }

    void
    BulletRobot::setMaximumMotorImpulse(double maxImpulse)
    {
        bulletMaxMotorImulse = (btScalar)maxImpulse;
    }

    double
    BulletRobot::getMaximumMotorImpulse() const
    {
        return static_cast<double>(bulletMaxMotorImulse);
    }

} // namespace SimDynamics
