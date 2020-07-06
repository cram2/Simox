/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @package    SimDynamics
* @author     Nikolaus Vahrenkamp
* @copyright  2012 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#pragma once

#include <VirtualRobot/SceneObject.h>
#include "../DynamicsEngine.h"
#include "BulletRobot.h"

#include "btBulletDynamicsCommon.h"

namespace internal
{
    inline void suppressUnusedVariableBtInfinityMask()
    {
        (void)sizeof(btInfinityMask);
    }
}

namespace SimDynamics
{

    typedef void (*BulletStepCallback)(void* data, btScalar timeStep);

    class SIMDYNAMICS_IMPORT_EXPORT BulletEngineConfig : public DynamicsEngineConfig
    {
    public:
        BulletEngineConfig();

        ~BulletEngineConfig() override {}

        // global setup values
        btScalar bulletObjectRestitution;
        btScalar bulletObjectFriction;
        btScalar bulletObjectDampingLinear;
        btScalar bulletObjectDampingAngular;
        btScalar bulletObjectSleepingThresholdLinear;
        btScalar bulletObjectSleepingThresholdAngular;
        btScalar bulletObjectDeactivation;
        int bulletSolverIterations;
        btScalar bulletSolverGlobalContactForceMixing; // allow to violate constraints (eg joint limits). A value>0 may increase stablity. (standard:0)
        btScalar bulletSolverGlobalErrorReductionParameter; // How hard should the solver try to correct misaligned joints/constraints/links. (standard 0.2
        btScalar bulletSolverSuccessiveOverRelaxation;
        //btScalar bulletSolverContactSurfaceLayer;
        btScalar bulletSolverSplitImpulsePenetrationThreshold;
    };

    typedef std::shared_ptr<BulletEngineConfig> BulletEngineConfigPtr;

    /*!
        This class encapsulates all calls to the bullet physics engine.
        Usually there is no need to instantiate this object by your own, it is automatically created when calling DynamicsWorld::Init().
    */
    class SIMDYNAMICS_IMPORT_EXPORT BulletEngine : public DynamicsEngine, public btActionInterface, public std::enable_shared_from_this<BulletEngine>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        friend class BulletObject;

        /*!
            Constructor
            \param engineMutex Optionally, all engine access methods can be protected by an external mutex. If not set, an internal mutex is creeated.
        */
        BulletEngine(std::shared_ptr <std::recursive_mutex> engineMutex = std::shared_ptr<std::recursive_mutex>());

        /*!
        */
        ~BulletEngine() override;

        bool addObject(DynamicsObjectPtr o) override;
        bool removeObject(DynamicsObjectPtr o) override;

        bool addRobot(DynamicsRobotPtr r) override;
        bool removeRobot(DynamicsRobotPtr r) override;

        /*!
            Initialize the engine with this configuration.
            \param config Either a standard init (could be NULL), or if config is of type BulletEngineConfig, Bullet specific parameters will be considered.
        */
        bool init(DynamicsEngineConfigPtr config) override;
        virtual bool init(BulletEngineConfigPtr config);

        virtual bool cleanup();

        void updateConfig(BulletEngineConfigPtr newConfig);  /* Currently not available for DynamicsEngine */

        /*!
            Set floor
            \param friction If <=0.0, the standard friction parameter for novel objects is used.
        */
        void createFloorPlane(const Eigen::Vector3f& pos, const Eigen::Vector3f& up, float friction = 0.0f) override;

        void removeFloorPlane() override;
        /*!
            dt and fixedTimeStep are given in seconds.
        */
        virtual void stepSimulation(double dt, int maxSubSteps, double fixedTimeStep);

        btDynamicsWorld* getBulletWorld();

        std::vector<DynamicsEngine::DynamicsContactInfo> getContacts() override;

        void print();

        /*!
         * Adds callback that is called each time bullet steps the engine.
         */
        void addExternalCallback(BulletStepCallback function, void* data);

        /*!
            The simulated time starts with 0 (on creation) and is updated on every steSimulation call.
            \return simulated time in seconds.
        */
        double getSimTime();

        bool attachObjectToRobot(DynamicsRobotPtr r, const std::string& nodeName, DynamicsObjectPtr object) override;
        bool detachObjectFromRobot(DynamicsRobotPtr r, DynamicsObjectPtr object) override;


        /*!
            Transforms pose to bullet.
            Translation is scaled from mm to m.
        */
        static btTransform getPoseBullet(const Eigen::Matrix4f& pose, bool scaling = true);
        static Eigen::Matrix4f getPoseEigen(const btTransform& pose, bool scaling = true);
        static btVector3 getVecBullet(const Eigen::Vector3f& vec, bool scaling = true);
        static Eigen::Vector3f getVecEigen(const btVector3& vec, bool scaling = true);
        static btMatrix3x3 getRotMatrix(const Eigen::Matrix4f& pose);
        static Eigen::Matrix4f getRotMatrix(const btMatrix3x3& pose);
    protected:
        // callback called each tick by bullet callback
        void updateRobots(btScalar timeStep);

        class CustomCollisionCallback : public btOverlapFilterCallback
        {
        public:
            CustomCollisionCallback(BulletEngine* e)
            {
                engine = e;
            }
            bool needBroadphaseCollision(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) const override
            {
                VR_ASSERT(engine);
                VR_ASSERT(static_cast<btCollisionObject*>(proxy0->m_clientObject));
                VR_ASSERT(static_cast<btCollisionObject*>(proxy1->m_clientObject));
                btCollisionObject* bt0 = static_cast<btCollisionObject*>(proxy0->m_clientObject);
                btCollisionObject* bt1 = static_cast<btCollisionObject*>(proxy1->m_clientObject);
                SimDynamics::BulletObject* o0 = static_cast<SimDynamics::BulletObject*>(bt0->getUserPointer());
                SimDynamics::BulletObject* o1 = static_cast<SimDynamics::BulletObject*>(bt1->getUserPointer());
                return engine->checkCollisionEnabled(o0, o1);
                //return true;//btOverlapFilterCallback::needBroadphaseCollision(proxy0,proxy1);
            }
        protected:
            BulletEngine* engine;
        };

        typedef std::pair<BulletStepCallback, void*> ExCallbackData;
        std::vector<ExCallbackData> callbacks;
        static void externalCallbacks(btDynamicsWorld* world, btScalar timeStep);

        virtual bool addLink(BulletRobot::LinkInfo& l);
        virtual bool removeLink(BulletRobot::LinkInfo& l);

        btDynamicsWorld* dynamicsWorld;

        btBroadphaseInterface* overlappingPairCache;
        btConstraintSolver* constraintSolver;
        btCollisionDispatcher* dispatcher;
        btDefaultCollisionConfiguration* collision_config;

        btOverlapFilterCallback* collisionFilterCallback;

        VirtualRobot::ObstaclePtr groundObject;

        BulletEngineConfigPtr bulletConfig;

        double simTime;

        // btActionInterface interface
    public:
        void updateAction(btCollisionWorld* collisionWorld, btScalar deltaTimeStep) override;
        void debugDraw(btIDebugDraw* debugDrawer) override;
    };

    typedef std::shared_ptr<BulletEngine> BulletEnginePtr;

} // namespace SimDynamics

