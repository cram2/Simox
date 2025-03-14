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
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#pragma once

#include <vector>

#include "VirtualRobot/MathTools.h"
#include "VirtualRobot/VirtualRobot.h"

#if defined(VR_COLLISION_DETECTION_PQP)
#include "PQP/CollisionCheckerPQP.h"
#else
#include "Dummy/CollisionCheckerDummy.h"
#endif

namespace VirtualRobot
{

    /*!
        A CollisionChecker is an instance that handles collision and distance queries.
        Internally the requests are passed to the underlying engine (e.g. the PQP library).
        All objects that should be considered for collision detection (called CollisionModels)
        must be registered here. Usually the objects take care of registering on their own.

        When collision detection should not be performed in parallel, the global CollisionChecker
        singleton can be used. It can be retrieved with CollisionChecker::getGlobalCollisionChecker().


    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT CollisionChecker :
        public std::enable_shared_from_this<CollisionChecker>
    {
    public:
        CollisionChecker();
        virtual ~CollisionChecker();

        /*!
            Returns distance of the collision models.
            Returns -1.0 if no distance calculation lib was specified (-> Dummy Col Checker)
        */
        virtual float calculateDistance(const SceneObjectSetPtr& model1,
                                        const SceneObjectSetPtr& model2);
        virtual float calculateDistance(const CollisionModelPtr& model1,
                                        const SceneObjectSetPtr& model2);
        virtual float calculateDistance(const CollisionModelPtr& model1,
                                        const CollisionModelPtr& model2);
        virtual float calculateDistance(const SceneObjectSetPtr& model1,
                                        const SceneObjectSetPtr& model2,
                                        Eigen::Vector3f& P1,
                                        Eigen::Vector3f& P2,
                                        int* trID1 = NULL,
                                        int* trID2 = NULL);
        virtual float calculateDistance(const CollisionModelPtr& model1,
                                        const SceneObjectSetPtr& model2,
                                        Eigen::Vector3f& P1,
                                        Eigen::Vector3f& P2,
                                        int* trID1 = NULL,
                                        int* trID2 = NULL);
        virtual float calculateDistance(const CollisionModelPtr& model1,
                                        const CollisionModelPtr& model2,
                                        Eigen::Vector3f& P1,
                                        Eigen::Vector3f& P2,
                                        int* trID1 = NULL,
                                        int* trID2 = NULL);

        /*!
            Test if the two models are colliding.
            Returns true on collision.
        */
        virtual bool checkCollision(const SceneObjectSetPtr& model1,
                                    const SceneObjectSetPtr& model2);

        /*!
            Test if the two models are colliding.
            Returns true on collision.
        */
        virtual bool checkCollision(const std::vector<CollisionModelPtr>& model1,
                                    const CollisionModelPtr& model2);
        /*!
            Test if the two models are colliding.
            Returns true on collision.
        */
        virtual bool checkCollision(const CollisionModelPtr& model1,
                                    const SceneObjectSetPtr& model2);
        /*!
            Test if the two models are colliding.
            Returns true on collision.
        */
        virtual bool
        checkCollision(const CollisionModelPtr& model1,
                       const CollisionModelPtr& model2); //, Eigen::Vector3f *storeContact = NULL);
        //virtual bool getAllCollisonTriangles (SceneObjectSetPtr model1, SceneObjectSetPtr model2, std::vector<int> &storePairs);

        /*!
            Store all vertices of colModel whose distance to p is smaller than maxDist.
        */
        virtual void getContacts(const MathTools::Plane& p,
                                 const CollisionModelPtr& colModel,
                                 std::vector<MathTools::ContactPoint>& storeContatcs,
                                 float maxDist = 1.0f);

        /*!
            If continuous collision detection (CCD) is supported, this method can be used to detect collisions on the path
            from the current pose of the collision models to the goal poses.
            true -> collision (then the time of contact [0..1] is stored to fStoreTOC)
        */
        //bool CheckContinuousCollision (CollisionModelPtr model1, SbMatrix &mGoalPose1, CollisionModelPtr model2, SbMatrix &mGoalPose2, float &fSToreTOC);

        MultiCollisionResult checkMultipleCollisions(CollisionModelPtr const& model1,
                                                     CollisionModelPtr const& model2);

        inline bool
        isInitialized()
        {
            return initialized;
        };

        /*!
        Activates / Deactivates the automatic size check on col model creation.
        The size check can be useful when the UNITS definitions in inventor files result in different scalings of the 3D models.
        (Standard: true)
        */
        void setAutomaticSizeCheck(bool checkSizeOnColModelCreation);

        void
        enableDebugOutput(bool e)
        {
            debugOutput = e;
            collisionCheckerImplementation->enableDebugOutput(e);
        }

        bool debugOutput;

        //! This is the global collision checker singleton
        static CollisionCheckerPtr getGlobalCollisionChecker();

        /*!
            Convenient methods
        */
        virtual float calculateDistance(const SceneObjectPtr& model1,
                                        const SceneObjectSetPtr& model2);
        virtual float calculateDistance(const SceneObjectPtr& model1, const SceneObjectPtr& model2);
        virtual float calculateDistance(const SceneObjectPtr& model1,
                                        const SceneObjectSetPtr& model2,
                                        Eigen::Vector3f& P1,
                                        Eigen::Vector3f& P2,
                                        int* trID1 = NULL,
                                        int* trID2 = NULL);
        virtual float calculateDistance(const SceneObjectPtr& model1,
                                        const SceneObjectPtr& model2,
                                        Eigen::Vector3f& P1,
                                        Eigen::Vector3f& P2,
                                        int* trID1 = NULL,
                                        int* trID2 = NULL);

        /*!
            Test if the two models are colliding.
            Returns true on collision.
        */
        virtual bool checkCollision(const SceneObjectPtr& model1, const SceneObjectSetPtr& model2);
        /*!
            Test if the two models are colliding.
            Returns true on collision.
        */
        virtual bool checkCollision(const SceneObjectPtr& model1, const SceneObjectPtr& model2);

        /**
         * @brief checks collision between a sphere (modeled by point + tolerance).
         * @node The check only performs
         * a collision check of triangles, not if the sphere is inside the given model!
         * @param model
         * @param point
         * @param tolerance
         * @return true, if the sphere is in collision with the model
         */
        virtual bool checkCollision(const CollisionModelPtr& model,
                                    const Eigen::Vector3f& point,
                                    float tolerance);
        virtual bool checkCollision(const SceneObjectSetPtr& modelSet,
                                    const Eigen::Vector3f& point,
                                    float tolerance);

        /*!
            Does the underlying collision detection library support discrete collision detection.
        */
        static bool IsSupported_CollisionDetection();

        /*!
            Does the underlying collision detection library support continuous collision detection.
        */
        static bool IsSupported_ContinuousCollisionDetection();

        /*!
            Does the underlying collision detection library support distance calculations.
        */
        static bool IsSupported_DistanceCalculations();

        /*!
            Does the underlying collision detection library support threadsafe access.
            E.g. multiple threads query the collision checker asynchronously.
        */
        static bool IsSupported_Multithreading_Threadsafe();

        /*!
            Does the underlying collision detection library support multiple instances of the collision checker.
            E.g. one per thread.
        */
        static bool IsSupported_Multithreading_MultipleColCheckers();

#if defined(VR_COLLISION_DETECTION_PQP)
        std::shared_ptr<CollisionCheckerPQP>
        getCollisionCheckerImplementation()
        {
            return collisionCheckerImplementation;
        }
#else
        std::shared_ptr<CollisionCheckerDummy>
        getCollisionCheckerImplementation()
        {
            return collisionCheckerImplementation;
        }
#endif

    protected:
        SceneObjectSetPtr getRobotModels(const RobotPtr& r);

    private:
        // see http://en.wikipedia.org/wiki/Singleton_pattern for details about correct implementations of singletons in C++
        friend class Cleanup;

        class Cleanup
        {
        public:
            ~Cleanup();
        };

        bool initialized;

        static CollisionCheckerPtr globalCollisionChecker;

        bool automaticSizeCheck;

        Eigen::Vector3f tmpV1;
        Eigen::Vector3f tmpV2;


#if defined(VR_COLLISION_DETECTION_PQP)
        std::shared_ptr<CollisionCheckerPQP> collisionCheckerImplementation;
#else
        std::shared_ptr<CollisionCheckerDummy> collisionCheckerImplementation;
#endif
    };
} // namespace VirtualRobot
