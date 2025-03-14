#include "BulletEngine.h"

#include <VirtualRobot/MathTools.h>
#include <VirtualRobot/Obstacle.h>

#include "../../DynamicsWorld.h"
#include "BulletObject.h"
#include "DetectBulletVersion.h"
#include "SimoxCollisionDispatcher.h"

//#define DEBUG_FIXED_OBJECTS

namespace SimDynamics
{

    BulletEngineConfig::BulletEngineConfig() : DynamicsEngineConfig()
    {
        bulletObjectRestitution = btScalar(0.0);
        bulletObjectFriction = btScalar(0.5f);
        bulletObjectDampingLinear = btScalar(0.05f);
        bulletObjectDampingAngular = btScalar(0.85f);
        //            bulletObjectDampingAngular = btScalar(0.1f);
        bulletObjectDeactivation = btScalar(5.0); //1.0);
        bulletObjectSleepingThresholdLinear = btScalar(0.5f * BulletObject::ScaleFactor);
        bulletObjectSleepingThresholdAngular = btScalar(0.5f); //2.5);

        bulletSolverIterations = 250;
        bulletSolverGlobalContactForceMixing = 0.0;
        bulletSolverGlobalErrorReductionParameter = btScalar(0.6);
#ifdef SIMOX_USES_OLD_BULLET
        bulletSolverSuccessiveOverRelaxation = btScalar(0.0);
#else
        bulletSolverSuccessiveOverRelaxation = btScalar(1.0);
#endif
        //bulletSolverContactSurfaceLayer = btScalar(0.001);
        bulletSolverSplitImpulsePenetrationThreshold = btScalar(-0.01);
    }

    BulletEngine::BulletEngine(std::shared_ptr<std::recursive_mutex> engineMutex) :
        DynamicsEngine(engineMutex)
    {
        collision_config = nullptr;
        dispatcher = nullptr;
        overlappingPairCache = nullptr;
        constraintSolver = nullptr;
        dynamicsWorld = nullptr;
        simTime = 0;
    }

    BulletEngine::~BulletEngine()
    {
        cleanup();
    }

    bool
    BulletEngine::init(DynamicsEngineConfigPtr config)
    {
        // first check if config is of type BulletEngineConfig
        BulletEngineConfigPtr test = std::dynamic_pointer_cast<BulletEngineConfig>(config);

        if (!config || !test)
        {
            BulletEngineConfigPtr c(new BulletEngineConfig());

            if (config)
            {
                c->gravity = config->gravity;
            }

            return init(c);
        }
        else
        {
            return init(test);
        }
    }

    bool
    BulletEngine::init(BulletEngineConfigPtr config)
    {
        MutexLockPtr lock = getScopedLock();
        DynamicsEngine::init(config);
        bulletConfig = config;

        // Setup the bullet world
        collision_config = new btDefaultCollisionConfiguration();
        dispatcher = new SimoxCollisionDispatcher(this, collision_config);

        /*
        btVector3 worldAabbMin(-10000,-10000,-10000);
        btVector3 worldAabbMax(10000,10000,10000);
        overlappingPairCache = new btAxisSweep3 (worldAabbMin, worldAabbMax);
        */
        overlappingPairCache = new btDbvtBroadphase();
        //overlappingPairCache = new btSimpleBroadphase();


        constraintSolver = new btSequentialImpulseConstraintSolver;

        dynamicsWorld = new btDiscreteDynamicsWorld(
            dispatcher, overlappingPairCache, constraintSolver, collision_config);

        dynamicsWorld->setGravity(
            btVector3(btScalar(config->gravity[0] * BulletObject::ScaleFactor),
                      btScalar(config->gravity[1] * BulletObject::ScaleFactor),
                      btScalar(config->gravity[2] * BulletObject::ScaleFactor)));

        collisionFilterCallback = new BulletEngine::CustomCollisionCallback(this);
        dynamicsWorld->getPairCache()->setOverlapFilterCallback(collisionFilterCallback);

        btContactSolverInfo& solverInfo = dynamicsWorld->getSolverInfo();
        solverInfo.m_numIterations = config->bulletSolverIterations;
        solverInfo.m_globalCfm = config->bulletSolverGlobalContactForceMixing;
        solverInfo.m_erp = config->bulletSolverGlobalErrorReductionParameter;
        solverInfo.m_solverMode |= SOLVER_USE_2_FRICTION_DIRECTIONS;
        solverInfo.m_sor = config->bulletSolverSuccessiveOverRelaxation;

        /*
        By default, Bullet solves positional constraints and velocity constraints coupled together.
        This works well in many cases, but the error reduction of position coupled to velocity introduces extra energy (noticeable as 'bounce').
        Instead of coupled positional and velocity constraint solving, the two can be solved separately using the 'split impulse' option.
        This means that recovering from deep penetrations doesn't add any velocity. You can enable the option using:
        */
        solverInfo.m_splitImpulse = 1; //enable split impulse feature
        //optionally set the m_splitImpulsePenetrationThreshold (only used when m_splitImpulse  is enabled)
        //only enable split impulse position correction when the penetration is deeper than this m_splitImpulsePenetrationThreshold, otherwise use the regular velocity/position constraint coupling (Baumgarte).
        solverInfo.m_splitImpulsePenetrationThreshold =
            config->bulletSolverSplitImpulsePenetrationThreshold;

        //        dynamicsWorld->setInternalTickCallback(externalCallbacks, this, true);
        dynamicsWorld->addAction(this);
        return true;
    }

    bool
    BulletEngine::cleanup()
    {
        MutexLockPtr lock = getScopedLock();

        while (robots.size() > 0)
        {
            removeRobot(robots[0]);
#ifndef NDEBUG
            size_t start = robots.size();
#endif
            VR_ASSERT(robots.size() < start);
        }

        while (objects.size() > 0)
        {
#ifndef NDEBUG
            size_t start = objects.size();
#endif
            removeObject(objects[0]);
            VR_ASSERT(objects.size() < start);
        }

        delete dynamicsWorld;
        dynamicsWorld = nullptr;
        delete collision_config;
        collision_config = nullptr;
        delete dispatcher;
        dispatcher = nullptr;
        delete overlappingPairCache;
        overlappingPairCache = nullptr;
        delete constraintSolver;
        constraintSolver = nullptr;
        delete collisionFilterCallback;
        collisionFilterCallback = nullptr;
        return true;
    }

    void
    BulletEngine::updateConfig(BulletEngineConfigPtr newConfig)
    {
        MutexLockPtr lock = getScopedLock();

        bulletConfig = newConfig;

        dynamicsWorld->setGravity(
            btVector3(btScalar(newConfig->gravity[0] * BulletObject::ScaleFactor),
                      btScalar(newConfig->gravity[1] * BulletObject::ScaleFactor),
                      btScalar(newConfig->gravity[2] * BulletObject::ScaleFactor)));

        btContactSolverInfo& solverInfo = dynamicsWorld->getSolverInfo();
        solverInfo.m_numIterations = newConfig->bulletSolverIterations;
        solverInfo.m_globalCfm = newConfig->bulletSolverGlobalContactForceMixing;
        solverInfo.m_erp = newConfig->bulletSolverGlobalErrorReductionParameter;

        std::vector<DynamicsObjectPtr> objects = getObjects();

        for (std::vector<DynamicsObjectPtr>::const_iterator i = objects.begin(); i != objects.end();
             ++i)
        {
            BulletObjectPtr btObject = std::dynamic_pointer_cast<BulletObject>(*i);

            if (!btObject)
            {
                VR_ERROR << "Skipping non-BULLET object " << (*i)->getName() << "!" << std::endl;
                continue;
            }

            auto friction = btObject->getSceneObject()->getPhysics().friction;
            // TODO const auto damping = btObject->getSceneObject()->getPhysics().damping;

            btObject->getRigidBody()->setRestitution(bulletConfig->bulletObjectRestitution);
            btObject->getRigidBody()->setFriction(
                friction > 0.0 ? friction : bulletConfig->bulletObjectFriction);
            btObject->getRigidBody()->setFriction(bulletConfig->bulletObjectFriction);
            btObject->getRigidBody()->setDamping(bulletConfig->bulletObjectDampingLinear,
                                                 bulletConfig->bulletObjectDampingAngular);
            btObject->getRigidBody()->setDeactivationTime(bulletConfig->bulletObjectDeactivation);
            btObject->getRigidBody()->setSleepingThresholds(
                bulletConfig->bulletObjectSleepingThresholdLinear,
                bulletConfig->bulletObjectSleepingThresholdAngular);
        }
    }

    bool
    BulletEngine::addObject(DynamicsObjectPtr o)
    {
        MutexLockPtr lock = getScopedLock();
        BulletObjectPtr btObject = std::dynamic_pointer_cast<BulletObject>(o);

        if (!btObject)
        {
            VR_ERROR << "Could only handle BULLET objects?! <" << o->getName() << ">" << std::endl;
            return false;
        }

        int btColFlag;

        switch (o->getSimType())
        {
            case VirtualRobot::SceneObject::Physics::eStatic:
                btColFlag = btCollisionObject::CF_STATIC_OBJECT;
                break;

            case VirtualRobot::SceneObject::Physics::eKinematic:
                btColFlag = btCollisionObject::CF_KINEMATIC_OBJECT;
                break;

            case VirtualRobot::SceneObject::Physics::eDynamic:
            case VirtualRobot::SceneObject::Physics::eUnknown:
                btColFlag = 0;
                break;

            default:
                // Dynamic Object
                btColFlag = 0;
                break;
        }
        auto friction = o->getSceneObject()->getPhysics().friction;
        btObject->getRigidBody()->setCollisionFlags(btColFlag);
        btObject->getRigidBody()->setRestitution(bulletConfig->bulletObjectRestitution);
        btObject->getRigidBody()->setFriction(friction > 0.0 ? friction
                                                             : bulletConfig->bulletObjectFriction);
        btObject->getRigidBody()->setDamping(bulletConfig->bulletObjectDampingLinear,
                                             bulletConfig->bulletObjectDampingAngular);
        btObject->getRigidBody()->setDeactivationTime(
            bulletConfig->bulletObjectDeactivation); //5.0f);
        btObject->getRigidBody()->setSleepingThresholds(
            bulletConfig->bulletObjectSleepingThresholdLinear,
            bulletConfig->bulletObjectSleepingThresholdAngular); //0.05f, 0.05f);

        //btScalar defaultContactProcessingThreshold = BT_LARGE_FLOAT;
        //btObject->getRigidBody()->setContactProcessingThreshold(defaultContactProcessingThreshold);
        dynamicsWorld->addRigidBody(btObject->getRigidBody().get());
        btObject->getRigidBody()->setAngularVelocity(btVector3(0, 0, 0));
        btObject->getRigidBody()->setLinearVelocity(btVector3(0, 0, 0));
        btObject->getRigidBody()->activate(true);

        return DynamicsEngine::addObject(o);
    }

    bool
    BulletEngine::removeObject(DynamicsObjectPtr o)
    {
        MutexLockPtr lock = getScopedLock();
        BulletObjectPtr btObject = std::dynamic_pointer_cast<BulletObject>(o);

        if (!btObject)
        {
            VR_ERROR << "Could only handle BULLET objects?! <" << o->getName() << ">" << std::endl;
            return false;
        }

        dynamicsWorld->removeRigidBody(btObject->getRigidBody().get());
        btObject->getRigidBody()->setBroadphaseHandle(nullptr);
        return DynamicsEngine::removeObject(o);
    }

    bool
    BulletEngine::removeLink(BulletRobot::LinkInfo& l)
    {
        MutexLockPtr lock = getScopedLock();
        dynamicsWorld->removeConstraint(l.joint.get());
        this->enableCollision(static_cast<DynamicsObject*>(l.dynNode1.get()),
                              static_cast<DynamicsObject*>(l.dynNode2.get()));
        //this->resetCollisions(static_cast<DynamicsObject*>(l.dynNode1.get()));
        //this->resetCollisions(static_cast<DynamicsObject*>(l.dynNode2.get()));
        return true;
    }

    btDynamicsWorld*
    BulletEngine::getBulletWorld()
    {
        return dynamicsWorld;
    }

    void
    BulletEngine::createFloorPlane(const Eigen::Vector3f& pos,
                                   const Eigen::Vector3f& up,
                                   float friction)
    {
        MutexLockPtr lock = getScopedLock();

        if (friction <= 0)
        {
            friction = bulletConfig->bulletObjectFriction;
        }
        DynamicsEngine::createFloorPlane(pos, up);
        float size = float(floorExtendMM); //50000.0f; // mm
        float sizeSmall = float(floorDepthMM); // 500.0f;
        float w = size;
        float h = size;
        float d = sizeSmall;

        if (up(1) == 0 && up(2) == 0)
        {
            w = sizeSmall;
            h = size;
            d = size;
        }
        else if (up(0) == 0 && up(2) == 0)
        {
            w = size;
            h = sizeSmall;
            d = size;
        }

        groundObject = VirtualRobot::Obstacle::createBox(
            w, h, d, VirtualRobot::VisualizationFactory::Color::Gray());
        std::string name("Floor");
        groundObject->setName(name);
        Eigen::Matrix4f gp;
        gp.setIdentity();
        gp(2, 3) = -sizeSmall * 0.5f;
        groundObject->setGlobalPose(gp);

        groundObject->getVisualization();
        groundObject->setSimulationType(VirtualRobot::SceneObject::Physics::eStatic);

        groundObject->setFriction(friction);

        BulletObjectPtr groundObjectBt(new BulletObject(groundObject));
        floor = groundObjectBt;

        addObject(groundObjectBt);
    }

    void
    BulletEngine::removeFloorPlane()
    {
        MutexLockPtr lock = getScopedLock();
        groundObject.reset();
        DynamicsEngine::removeFloorPlane();
    }

    btMatrix3x3
    BulletEngine::getRotMatrix(const Eigen::Matrix4f& pose)
    {
        btMatrix3x3 rot(pose(0, 0),
                        pose(0, 1),
                        pose(0, 2),
                        pose(1, 0),
                        pose(1, 1),
                        pose(1, 2),
                        pose(2, 0),
                        pose(2, 1),
                        pose(2, 2));
        return rot;
    }

    Eigen::Matrix4f
    BulletEngine::getRotMatrix(const btMatrix3x3& pose)
    {
        Eigen::Matrix4f rot = Eigen::Matrix4f::Identity();

        for (int a = 0; a < 3; a++)
            for (int b = 0; b < 3; b++)
            {
                rot(a, b) = pose[a][b];
            }

        return rot;
    }

    btTransform
    BulletEngine::getPoseBullet(const Eigen::Matrix4f& pose, bool scaling)
    {
        btTransform res;
        btScalar sc = btScalar(1.0f);

        if (scaling && DynamicsWorld::convertMM2M)
        {
            sc = 0.001f * BulletObject::ScaleFactor; // mm -> m
        }

        btVector3 pos(pose(0, 3) * sc, pose(1, 3) * sc, pose(2, 3) * sc);
        res.setOrigin(pos);
        btMatrix3x3 rot = getRotMatrix(pose);
        //VirtualRobot::MathTools::Quaternion q = VirtualRobot::MathTools::eigen4f2quat(pose);
        //btQuaternion rot(q.x,q.y,q.z,q.w);
        res.setBasis(rot);
        return res;
    }

    Eigen::Matrix4f
    BulletEngine::getPoseEigen(const btTransform& pose, bool scaling)
    {
        double sc = 1.0f;

        if (scaling && DynamicsWorld::convertMM2M)
        {
            sc = 1000.0f / BulletObject::ScaleFactor; // m -> mm
        }

        /*btQuaternion q = pose.getRotation();
        VirtualRobot::MathTools::Quaternion qvr;
        qvr.x = q.getX();
        qvr.y = q.getY();
        qvr.z = q.getZ();
        qvr.w = q.getW();
        Eigen::Matrix4f res = VirtualRobot::MathTools::quat2eigen4f(qvr);*/
        Eigen::Matrix4f res = getRotMatrix(pose.getBasis());
        res(0, 3) = float(pose.getOrigin().getX() * sc);
        res(1, 3) = float(pose.getOrigin().getY() * sc);
        res(2, 3) = float(pose.getOrigin().getZ() * sc);
        return res;
    }

    btVector3
    BulletEngine::getVecBullet(const Eigen::Vector3f& vec, bool scaling)
    {
        btTransform res;
        btScalar sc = 1.0f;

        if (scaling && DynamicsWorld::convertMM2M)
        {
            sc = 0.001f * BulletObject::ScaleFactor; // mm -> m
        }

        btVector3 pos(vec(0) * sc, vec(1) * sc, vec(2) * sc);
        return pos;
    }

    Eigen::Vector3f
    BulletEngine::getVecEigen(const btVector3& vec, bool scaling)
    {
        double sc = 1.0f;

        if (scaling && DynamicsWorld::convertMM2M)
        {
            sc = 1000.0f / BulletObject::ScaleFactor; // m -> mm
        }

        Eigen::Vector3f res;
        res(0) = float(vec.getX() * sc);
        res(1) = float(vec.getY() * sc);
        res(2) = float(vec.getZ() * sc);

        return res;
    }

    bool
    BulletEngine::addRobot(DynamicsRobotPtr r)
    {
        MutexLockPtr lock = getScopedLock();
        BulletRobotPtr btRobot = std::dynamic_pointer_cast<BulletRobot>(r);

        if (!btRobot)
        {
            VR_ERROR << "Could only handle BULLET objects?! <" << r->getName() << ">" << std::endl;
            return false;
        }

        std::vector<BulletRobot::LinkInfo> links = btRobot->getLinks();
        std::vector<DynamicsObjectPtr> nodes = btRobot->getDynamicsRobotNodes();

        for (const auto& node : nodes)
        {
            addObject(node);
        }

        for (auto& link : links)
        {
            addLink(link);
        }

        return DynamicsEngine::addRobot(r);
    }

    void
    BulletEngine::addExternalCallback(BulletStepCallback function, void* data)
    {
        MutexLockPtr lock = getScopedLock();

        callbacks.push_back(ExCallbackData(function, data));
    }

    void
    BulletEngine::externalCallbacks(btDynamicsWorld* world, btScalar timeStep)
    {
        BulletEngine* e = static_cast<BulletEngine*>(world->getWorldUserInfo());

        if (!e)
        {
            return;
        }

        // apply lock
        MutexLockPtr lock = e->getScopedLock();

        e->updateRobots(timeStep);

        for (auto& callback : e->callbacks)
        {
            callback.first(callback.second, timeStep);
        }
    }

    void
    BulletEngine::updateRobots(btScalar timeStep)
    {
        for (auto& robot : robots)
        {
            robot->actuateJoints(static_cast<double>(timeStep));
            robot->updateSensors(static_cast<double>(timeStep));
        }
    }

    bool
    BulletEngine::removeRobot(DynamicsRobotPtr r)
    {
        MutexLockPtr lock = getScopedLock();
        BulletRobotPtr btRobot = std::dynamic_pointer_cast<BulletRobot>(r);

        if (!btRobot)
        {
            VR_ERROR << "Could only handle BULLET objects?! <" << r->getName() << ">" << std::endl;
            return false;
        }

        std::vector<BulletRobot::LinkInfo> links = btRobot->getLinks();
        std::vector<DynamicsObjectPtr> nodes = btRobot->getDynamicsRobotNodes();


        for (auto& link : links)
        {
            removeLink(link);
        }

        for (const auto& node : nodes)
        {
            removeObject(node);
        }

        return DynamicsEngine::removeRobot(r);
    }

    bool
    BulletEngine::addLink(BulletRobot::LinkInfo& l)
    {
        MutexLockPtr lock = getScopedLock();
#ifdef DEBUG_FIXED_OBJECTS
        std::cout << "TEST2" << std::endl;
#else
        dynamicsWorld->addConstraint(l.joint.get(), true);
#endif

        for (auto& disabledCollisionPair : l.disabledCollisionPairs)
        {
            this->disableCollision(
                static_cast<DynamicsObject*>(disabledCollisionPair.first.get()),
                static_cast<DynamicsObject*>(disabledCollisionPair.second.get()));
        }

        return true;
    }

    void
    BulletEngine::print()
    {
        MutexLockPtr lock = getScopedLock();
        std::cout << "------------------ Bullet Engine ------------------" << std::endl;

        for (size_t i = 0; i < objects.size(); i++)
        {
            std::cout << "++ Object " << i << ":" << objects[i]->getName() << std::endl;
            Eigen::Matrix4f m = objects[i]->getSceneObject()->getGlobalPose();
            std::cout << "   pos (simox)  " << m(0, 3) << "," << m(1, 3) << "," << m(2, 3)
                      << std::endl;
            BulletObjectPtr bo = std::dynamic_pointer_cast<BulletObject>(objects[i]);
            std::shared_ptr<btRigidBody> rb = bo->getRigidBody();
            btVector3 v = rb->getWorldTransform().getOrigin();
            std::cout << "   pos (bullet) " << v[0] << "," << v[1] << "," << v[2] << std::endl;
            btVector3 va = rb->getAngularVelocity();
            btVector3 vl = rb->getLinearVelocity();
            std::cout << "   ang vel (bullet) " << va[0] << "," << va[1] << "," << va[2]
                      << std::endl;
            std::cout << "   lin vel (bullet) " << vl[0] << "," << vl[1] << "," << vl[2]
                      << std::endl;
        }

        for (size_t i = 0; i < robots.size(); i++)
        {
            std::cout << "++ Robot " << i << ":" << objects[i]->getName() << std::endl;
            BulletRobotPtr br = std::dynamic_pointer_cast<BulletRobot>(robots[i]);
            std::vector<BulletRobot::LinkInfo> links = br->getLinks();

            for (size_t j = 0; j < links.size(); j++)
            {
                std::cout << "++++ Link " << j << ":" << links[j].nodeJoint->getName();
                std::cout << "++++ - ColModelA " << j << ":" << links[j].nodeA->getName();
                std::cout << "++++ - ColModelB " << j << ":" << links[j].nodeB->getName();

                std::cout << "     enabled:" << links[j].joint->isEnabled() << std::endl;
                std::shared_ptr<btHingeConstraint> hinge =
                    std::dynamic_pointer_cast<btHingeConstraint>(links[j].joint);

                if (hinge)
                {
                    std::cout << "     hinge motor enabled:" << hinge->getEnableAngularMotor()
                              << std::endl;
                    std::cout << "     hinge angle :" << hinge->getHingeAngle() << std::endl;
                    std::cout << "     hinge max motor impulse :" << hinge->getMaxMotorImpulse()
                              << std::endl;
#ifdef SIMOX_USES_OLD_BULLET
                    std::cout << "     hinge motor target vel :" << hinge->getMotorTargetVelosity()
                              << std::endl;
#else
                    std::cout << "     hinge motor target vel :" << hinge->getMotorTargetVelocity()
                              << std::endl;
#endif
                }

                std::shared_ptr<btGeneric6DofConstraint> dof =
                    std::dynamic_pointer_cast<btGeneric6DofConstraint>(links[j].joint);

                if (dof)
                {
                    btRotationalLimitMotor* m = dof->getRotationalLimitMotor(2);
                    VR_ASSERT(m);
                    std::cout << "     generic_6DOF_joint: axis 5 (z)" << std::endl;
                    std::cout << "     generic_6DOF_joint motor enabled:" << m->m_enableMotor
                              << std::endl;
                    std::cout << "     generic_6DOF_joint angle :" << m->m_currentPosition
                              << std::endl;
                    std::cout << "     generic_6DOF_joint max motor force :" << m->m_maxMotorForce
                              << std::endl;
                    std::cout << "     higeneric_6DOF_jointnge motor target vel :"
                              << m->m_targetVelocity << std::endl;
                }
            }
        }

        std::cout << "------------------ Bullet Engine ------------------" << std::endl;
    }

    std::vector<DynamicsEngine::DynamicsContactInfo>
    BulletEngine::getContacts()
    {
        MutexLockPtr lock = getScopedLock();
        //Assume world->stepSimulation or world->performDiscreteCollisionDetection has been called

        std::vector<DynamicsEngine::DynamicsContactInfo> result;

        int numManifolds = dynamicsWorld->getDispatcher()->getNumManifolds();

        for (int i = 0; i < numManifolds; i++)
        {
            btPersistentManifold* contactManifold =
                dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
            const btCollisionObject* obA =
                static_cast<const btCollisionObject*>(contactManifold->getBody0());
            const btCollisionObject* obB =
                static_cast<const btCollisionObject*>(contactManifold->getBody1());
            SimDynamics::BulletObject* dynObjA =
                static_cast<SimDynamics::BulletObject*>(obA->getUserPointer());
            SimDynamics::BulletObject* dynObjB =
                static_cast<SimDynamics::BulletObject*>(obB->getUserPointer());
            int numContacts = contactManifold->getNumContacts();

            for (int j = 0; j < numContacts; j++)
            {
                btManifoldPoint& pt = contactManifold->getContactPoint(j);
                DynamicsContactInfo i;
                if (dynObjA)
                {
                    i.objectAName = dynObjA->getName();
                }
                if (dynObjB)
                {
                    i.objectBName = dynObjB->getName();
                }
                const btVector3& ptA = pt.getPositionWorldOnA();
                const btVector3& ptB = pt.getPositionWorldOnB();
                const btVector3& normalOnB = pt.m_normalWorldOnB;
                i.posGlobalA = getVecEigen(ptA);
                i.posGlobalB = getVecEigen(ptB);
                i.normalGlobalB(0) = normalOnB.x();
                i.normalGlobalB(1) = normalOnB.y();
                i.normalGlobalB(2) = normalOnB.z();
                i.combinedFriction = pt.m_combinedFriction;
                i.combinedRestitution = pt.m_combinedRestitution;
                i.appliedImpulse = pt.m_appliedImpulse;
                i.frictionDir1.x() = pt.m_lateralFrictionDir1.x();
                i.frictionDir1.y() = pt.m_lateralFrictionDir1.y();
                i.frictionDir1.z() = pt.m_lateralFrictionDir1.z();
                i.frictionDir2.x() = pt.m_lateralFrictionDir2.x();
                i.frictionDir2.y() = pt.m_lateralFrictionDir2.y();
                i.frictionDir2.z() = pt.m_lateralFrictionDir2.z();
                i.distance = pt.getDistance();
                result.push_back(i);
            }
        }

        return result;
    }

    void
    BulletEngine::stepSimulation(double dt, int maxSubSteps, double fixedTimeStep)
    {
        MutexLockPtr lock = getScopedLock();
        simTime += dt;
        dynamicsWorld->stepSimulation(btScalar(dt), maxSubSteps, btScalar(fixedTimeStep));
    }

    double
    BulletEngine::getSimTime()
    {
        return simTime;
    }

    bool
    BulletEngine::attachObjectToRobot(DynamicsRobotPtr r,
                                      const std::string& nodeName,
                                      DynamicsObjectPtr object)
    {
        MutexLockPtr lock = getScopedLock();

        if (!r)
        {
            return false;
        }


        BulletRobotPtr br = std::dynamic_pointer_cast<BulletRobot>(r);

        if (!br)
        {
            VR_ERROR << "no bullet robot" << std::endl;
            return false;
        }

        if (object->getSimType() != VirtualRobot::SceneObject::Physics::eDynamic)
        {
            VR_WARNING << "Sim type of object " << object->getName()
                       << "!=eDynamic. Is this intended?" << std::endl;
        }

        BulletRobot::LinkInfoPtr link = br->attachObjectLink(nodeName, object);

        if (!link)
        {
            VR_ERROR << "Failed to create bullet robot link" << std::endl;
            return false;
        }

        return addLink(*link);
    }

    bool
    BulletEngine::detachObjectFromRobot(DynamicsRobotPtr r, DynamicsObjectPtr object)
    {
        MutexLockPtr lock = getScopedLock();

        if (!r)
        {
            return false;
        }


        BulletRobotPtr br = std::dynamic_pointer_cast<BulletRobot>(r);

        if (!br)
        {
            VR_ERROR << "no bullet robot" << std::endl;
            return false;
        }

        BulletObjectPtr bo = std::dynamic_pointer_cast<BulletObject>(object);

        if (!bo)
        {
            VR_ERROR << "no bullet object" << std::endl;
            return false;
        }

        std::vector<BulletRobot::LinkInfo> links = br->getLinks(bo);

        for (auto& link : links)
        {
            removeLink(link);
        }

        bool res = br->detachObject(object);

        if (!res)
        {
            VR_ERROR << "Failed to detach object" << std::endl;
            return false;
        }

        return true;
    }

} // namespace SimDynamics

#include <chrono>

void
SimDynamics::BulletEngine::updateAction(btCollisionWorld* /*collisionWorld*/,
                                        btScalar deltaTimeStep)
{
    auto start = std::chrono::system_clock::now();
    // apply lock
    MutexLockPtr lock = getScopedLock();

    updateRobots(deltaTimeStep);

    for (auto& callback : callbacks)
    {
        callback.first(callback.second, deltaTimeStep);
    }
    std::chrono::duration<double> diff = (std::chrono::system_clock::now() - start);
    //    std::cout << "duration: " << diff.count() << std::endl;
}

void
SimDynamics::BulletEngine::debugDraw(btIDebugDraw* /*debugDrawer*/)
{
}
