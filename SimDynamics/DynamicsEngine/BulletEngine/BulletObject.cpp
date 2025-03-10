#include "BulletObject.h"

#include <Eigen/Dense>

#include <SimoxUtility/math/pose/pose.h>
#include <VirtualRobot/CollisionDetection/CollisionModel.h>
#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/Primitive.h>
#include <VirtualRobot/SceneObject.h>
#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <VirtualRobot/Visualization/VisualizationNode.h>

#include "../../DynamicsWorld.h"
#include "BulletEngine.h"
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletCollision/CollisionShapes/btCylinderShape.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>

//#define DEBUG_FIXED_OBJECTS
//#define USE_BULLET_GENERIC_6DOF_CONSTRAINT

#include <typeinfo>

using namespace VirtualRobot;

namespace SimDynamics
{

    float BulletObject::ScaleFactor = 2.0f;
    float BulletObject::MassFactor = 0.1f;

    BulletObject::BulletObject(VirtualRobot::SceneObjectPtr o) : DynamicsObject(o)
    {
        btScalar interatiaFactor = btScalar(1.0);
#ifdef USE_BULLET_GENERIC_6DOF_CONSTRAINT
        interatiaFactor = 5.0f;
#endif

        btMargin = (btScalar)(0.0001);
        com.setZero();
        THROW_VR_EXCEPTION_IF(!o, "NULL object");
        CollisionModelPtr colModel = o->getCollisionModel();
        if (!colModel)
        {
            VR_WARNING << "Building empty collision shape for object " << o->getName() << std::endl;
            collisionShape.reset(new btEmptyShape());
        }
        else
        {
            THROW_VR_EXCEPTION_IF(!colModel,
                                  "No CollisionModel, could not create dynamics model...");

            if (o->getName() != "Floor")
            {
                std::vector<Primitive::PrimitivePtr> primitives =
                    colModel->getVisualization()->primitives;

                if (primitives.size() > 0)
                {
                    //cout << "Object:" << o->getName() << std::endl;
                    //o->print();

                    btCompoundShape* compoundShape = new btCompoundShape(true);

                    Eigen::Matrix4f currentTransform = Eigen::Matrix4f::Identity();
                    simox::math::position(currentTransform) = -o->getCoMLocal();
                    //cout << "currentTransform:\n" << currentTransform << std::endl;

                    for (auto it = primitives.begin(); it != primitives.end(); it++)
                    {
                        currentTransform = currentTransform * (*it)->transform;
                        //currentTransform = localComTransform * (*it)->transform;
                        //cout << "primitive: (*it)->transform:\n" << (*it)->transform << std::endl;
                        //cout << "primitive: currentTransform:\n" << currentTransform << std::endl;

                        compoundShape->addChildShape(BulletEngine::getPoseBullet(currentTransform),
                                                     getShapeFromPrimitive(*it));
                    }

                    collisionShape.reset(compoundShape);

                    // update com
                    Eigen::Matrix4f comLoc = Eigen::Matrix4f::Identity();
                    simox::math::position(comLoc) = o->getCoMGlobal();
                    comLoc = o->getGlobalPose().inverse() * comLoc;
                    com = simox::math::position(comLoc);
                }
                else
                {
                    TriMeshModelPtr trimesh;
                    trimesh = colModel->getTriMeshModel();
                    THROW_VR_EXCEPTION_IF((!trimesh || trimesh->faces.size() == 0),
                                          "No TriMeshModel, could not create dynamics model...");
                    collisionShape.reset(createConvexHullShape(trimesh));
                }
            }
            else
            {
                // the floor needs a primitive shape, works better with collision handling
                VirtualRobot::BoundingBox bb = colModel->getBoundingBox();
                Eigen::Vector3f half_size = (bb.getMax() - bb.getMin()) / 1000.0 * ScaleFactor / 2;
                btBoxShape* box =
                    new btBoxShape(btVector3(half_size.x(), half_size.y(), half_size.z()));
                collisionShape.reset(box);
            }
        }

        //collisionShape->setMargin(btMargin);

        btScalar mass = o->getMass();
        btVector3 localInertia;

        if (mass <= 0 && (o->getSimulationType() == VirtualRobot::SceneObject::Physics::eDynamic ||
                          o->getSimulationType() == VirtualRobot::SceneObject::Physics::eUnknown))
        {
            //THROW_VR_EXCEPTION ("mass == 0 -> SimulationType must not be eDynamic! ");
            mass = btScalar(1.0f); // give object a dummy mass
#ifdef USE_BULLET_GENERIC_6DOF_CONSTRAINT
            mass = btScalar(1.0f);
#endif

            //type = eKinematic;
            if (colModel)
            {
                VR_WARNING
                    << "Object:" << o->getName()
                    << ": mass == 0 -> SimulationType must not be eDynamic! Setting mass to 1"
                    << std::endl;
            }
        }

#ifdef DEBUG_FIXED_OBJECTS
        std::cout << "TEST" << std::endl;
        mass = 0;
        localInertia.setValue(0.0f, 0.0f, 0.0f);
#else

        if (o->getSimulationType() != VirtualRobot::SceneObject::Physics::eDynamic &&
            o->getSimulationType() != VirtualRobot::SceneObject::Physics::eUnknown)
        {
            mass = 0;
            localInertia.setValue(0.0f, 0.0f, 0.0f);
        }
        else
        {
            if (colModel)
            {
                collisionShape->calculateLocalInertia(mass, localInertia);
            }
            else
#ifndef USE_BULLET_GENERIC_6DOF_CONSTRAINT
                localInertia.setValue(
                    btScalar(1), btScalar(1), btScalar(1)); // give Object a dummy inertia matrix

#else
                localInertia.setValue(
                    btScalar(1), btScalar(1), btScalar(1)); // give Object a dummy inertia matrix
#endif
        }

#endif
        localInertia *= interatiaFactor;
        motionState = new SimoxMotionState(o);
        btRigidBody::btRigidBodyConstructionInfo btRBInfo(
            mass * MassFactor, motionState, collisionShape.get(), localInertia);
        //btRBInfo.m_additionalDamping = true;

        rigidBody.reset(new btRigidBody(btRBInfo));
        rigidBody->setUserPointer((void*)(this));

        setPoseIntern(o->getGlobalPose());
    }

    BulletObject::~BulletObject()
    {
        rigidBody.reset();
        delete motionState;
    }

    btCollisionShape*
    BulletObject::getShapeFromPrimitive(VirtualRobot::Primitive::PrimitivePtr primitive)
    {
        btCollisionShape* result;

        if (primitive->type == Primitive::Box::TYPE)
        {
            Primitive::Box* box = std::dynamic_pointer_cast<Primitive::Box>(primitive).get();
            // w/h/d have to be halved
            btBoxShape* boxShape = new btBoxShape(btVector3(box->width / 1000.f * ScaleFactor / 2,
                                                            box->height / 1000.f * ScaleFactor / 2,
                                                            box->depth / 1000.f * ScaleFactor / 2));
            result = boxShape;
        }
        else if (primitive->type == Primitive::Sphere::TYPE)
        {
            Primitive::Sphere* sphere =
                std::dynamic_pointer_cast<Primitive::Sphere>(primitive).get();
            btSphereShape* sphereShape =
                new btSphereShape(btScalar(sphere->radius / 1000.0 * ScaleFactor));
            result = sphereShape;
        }
        else if (primitive->type == Primitive::Cylinder::TYPE)
        {
            Primitive::Cylinder* cyl =
                std::dynamic_pointer_cast<Primitive::Cylinder>(primitive).get();
            // height has to be halved
            btCylinderShape* cylShape =
                new btCylinderShape(btVector3(cyl->radius / 1000.0 * ScaleFactor,
                                              cyl->height / 1000.0 * ScaleFactor / 2,
                                              cyl->radius / 1000.0 * ScaleFactor));
            result = cylShape;
        }
        else
        {
            VR_ERROR << "Unsupported shape type " << primitive->type << std::endl;
            result = new btEmptyShape();
        }

        return result;
    }

    btCollisionShape*
    BulletObject::createConvexHullShape(VirtualRobot::TriMeshModelPtr trimesh)
    {
        VR_ASSERT(trimesh);
        // create triangle shape
        btTrimesh.reset(new btTriangleMesh());

        //com = trimesh->getCOM();

        Eigen::Matrix4f comLoc;
        comLoc.setIdentity();
        comLoc.block<3, 1>(0, 3) = sceneObject->getCoMGlobal();
        comLoc = (sceneObject->getGlobalPose().inverse() * comLoc);
        com = comLoc.block<3, 1>(0, 3);

        double sc = ScaleFactor;

        if (DynamicsWorld::convertMM2M)
        {
            sc = 0.001f * ScaleFactor;
        }

        for (auto& face : trimesh->faces)
        {
            auto& vertex1 = trimesh->vertices.at(face.id1);
            auto& vertex2 = trimesh->vertices.at(face.id2);
            auto& vertex3 = trimesh->vertices.at(face.id3);
            btVector3 v1(btScalar((vertex1[0] - com[0]) * sc),
                         btScalar((vertex1[1] - com[1]) * sc),
                         btScalar((vertex1[2] - com[2]) * sc));
            btVector3 v2(btScalar((vertex2[0] - com[0]) * sc),
                         btScalar((vertex2[1] - com[1]) * sc),
                         btScalar((vertex2[2] - com[2]) * sc));
            btVector3 v3(btScalar((vertex3[0] - com[0]) * sc),
                         btScalar((vertex3[1] - com[1]) * sc),
                         btScalar((vertex3[2] - com[2]) * sc));
            btTrimesh->addTriangle(v1, v2, v3);
        }

        // convert COM to visualization frame (->no, the trimesh points are given in local visu frame!)
        /*Eigen::Matrix4f comLoc;
        comLoc.setIdentity();
        comLoc.block(0,3,3,1) = com;
        Eigen::Matrix4f comConv = sceneObject->getGlobalPoseVisualization() * comLoc;
        com = comConv.block(0,3,3,1);*/


        if (sceneObject->getSimulationType() == VirtualRobot::SceneObject::Physics::eKinematic)
        {
            // Support concave objects if they are not dynamic
            return new btBvhTriangleMeshShape(btTrimesh.get(), false);
        }
        else
        {
            // build convex hull
            std::shared_ptr<btConvexShape> btConvexShape(
                new btConvexTriangleMeshShape(btTrimesh.get()));
            btConvexShape->setMargin(btMargin);

            std::shared_ptr<btShapeHull> btHull(new btShapeHull(btConvexShape.get()));
            btHull->buildHull(btMargin);
            btConvexHullShape* btConvex = new btConvexHullShape();
            btConvex->setLocalScaling(btVector3(1, 1, 1));

            for (int i = 0; i < btHull->numVertices(); i++)
            {
                btConvex->addPoint(btHull->getVertexPointer()[i]);
            }

            btConvex->setMargin(btMargin);

            // Trimesh no longer needed
            btTrimesh.reset();
            return btConvex;
        }
    }

    std::shared_ptr<btRigidBody>
    BulletObject::getRigidBody()
    {
        return rigidBody;
    }

    void
    BulletObject::setPosition(const Eigen::Vector3f& posMM)
    {
        MutexLockPtr lock = getScopedLock();
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        pose.block(0, 3, 3, 1) = posMM;
        setPose(pose);
    }

    void
    BulletObject::setPoseIntern(const Eigen::Matrix4f& pose)
    {
        MutexLockPtr lock = getScopedLock();
        /* convert to local coord system, apply comoffset and convert back*/
        Eigen::Matrix4f poseLocal = Eigen::Matrix4f::Identity();
        poseLocal.block(0, 3, 3, 1) += com;
        Eigen::Matrix4f poseGlobal = pose * poseLocal;
        this->rigidBody->setWorldTransform(BulletEngine::getPoseBullet(poseGlobal));

        // notify motionState of non-robot nodes
        if (!std::dynamic_pointer_cast<VirtualRobot::RobotNode>(sceneObject))
        {
            motionState->setGlobalPose(pose);
        }
    }

    void
    BulletObject::setPose(const Eigen::Matrix4f& pose)
    {
        MutexLockPtr lock = getScopedLock();
        DynamicsObject::setPose(pose);
        setPoseIntern(pose);
    }

    Eigen::Vector3f
    BulletObject::getLinearVelocity()
    {
        MutexLockPtr lock = getScopedLock();

        if (!rigidBody)
        {
            return Eigen::Vector3f::Zero();
        }

        return (BulletEngine::getVecEigen(rigidBody->getLinearVelocity()));
    }

    Eigen::Vector3f
    BulletObject::getAngularVelocity()
    {
        MutexLockPtr lock = getScopedLock();

        if (!rigidBody)
        {
            return Eigen::Vector3f::Zero();
        }

        return (BulletEngine::getVecEigen(rigidBody->getAngularVelocity(), false));
    }

    void
    BulletObject::setLinearVelocity(const Eigen::Vector3f& vel)
    {
        MutexLockPtr lock = getScopedLock();

        if (!rigidBody)
        {
            return;
        }

        btVector3 btVel = BulletEngine::getVecBullet(vel, false);
        rigidBody->activate();
        rigidBody->setLinearVelocity(btVel);
    }

    void
    BulletObject::setAngularVelocity(const Eigen::Vector3f& vel)
    {
        MutexLockPtr lock = getScopedLock();

        if (!rigidBody)
        {
            return;
        }

        btVector3 btVel = BulletEngine::getVecBullet(vel, false);
        rigidBody->activate();
        rigidBody->setAngularVelocity(btVel);
    }

    Eigen::Matrix4f
    BulletObject::getComGlobal()
    {
        MutexLockPtr lock = getScopedLock();
        btTransform tr;
        motionState->getWorldTransform(tr);
        return BulletEngine::getPoseEigen(tr);
    }

    void
    BulletObject::applyForce(const Eigen::Vector3f& force)
    {
        MutexLockPtr lock = getScopedLock();

        if (!rigidBody)
        {
            return;
        }

        btVector3 btVel = BulletEngine::getVecBullet(force, false);
        rigidBody->activate();
        rigidBody->applyCentralForce(btVel);
    }

    void
    BulletObject::applyTorque(const Eigen::Vector3f& torque)
    {
        MutexLockPtr lock = getScopedLock();

        if (!rigidBody)
        {
            return;
        }

        btVector3 btVel = BulletEngine::getVecBullet(torque, false) * BulletObject::ScaleFactor;
        rigidBody->activate();
        rigidBody->applyTorque(btVel);
    }

    void
    BulletObject::setSimType(VirtualRobot::SceneObject::Physics::SimulationType s)
    {
        btVector3 localInertia;
        localInertia.setZero();
        CollisionModelPtr colModel = sceneObject->getCollisionModel();


        int btColFlag = btCollisionObject::CF_STATIC_OBJECT;
        float mass = 0.0f;
        switch (s)
        {
            case VirtualRobot::SceneObject::Physics::eStatic:
                btColFlag = btCollisionObject::CF_STATIC_OBJECT;
                mass = 0;
                break;

            case VirtualRobot::SceneObject::Physics::eKinematic:
                btColFlag = btCollisionObject::CF_KINEMATIC_OBJECT;
                mass = 0;
                break;

            case VirtualRobot::SceneObject::Physics::eDynamic:
            case VirtualRobot::SceneObject::Physics::eUnknown:
                if (colModel)
                {
                    collisionShape->calculateLocalInertia(sceneObject->getMass(), localInertia);
                }
                else
                {
                    localInertia.setValue(btScalar(1),
                                          btScalar(1),
                                          btScalar(1)); // give Object a dummy inertia matrix
                }
                mass = sceneObject->getMass();
                btColFlag = 0;
                break;

            default:
                break;
        }

        rigidBody->setMassProps(mass, localInertia);
        rigidBody->setCollisionFlags(btColFlag);

        DynamicsObject::setSimType(s);
    }

    void
    BulletObject::activate()
    {
        MutexLockPtr lock = getScopedLock();

        if (!rigidBody)
        {
            return;
        }

        rigidBody->activate();
    }


} // namespace SimDynamics
