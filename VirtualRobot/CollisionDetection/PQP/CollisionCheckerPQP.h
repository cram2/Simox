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

#include "../../VirtualRobot.h"
#include "../CollisionCheckerImplementation.h"

// #include "PQP++/PQP_Compile.h"
// #include "PQP++/PQP.h"

#include <eigen3/Eigen/Core>
#include <Eigen/Geometry>

namespace PQP
{
    class PQP_Model;
    class PQP_Checker;
    class PQP_DistanceResult;
} // namespace PQP

namespace VirtualRobot
{
    /*!
        This implementation encapsulates the PQP collision checker.
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT CollisionCheckerPQP : public CollisionCheckerImplementation
    {
    public:
        friend class CollisionChecker;

        CollisionCheckerPQP();
        ~CollisionCheckerPQP() override;

        float calculateDistance(const CollisionModelPtr& model1,
                                const CollisionModelPtr& model2,
                                Eigen::Vector3f& P1,
                                Eigen::Vector3f& P2,
                                int* trID1 = NULL,
                                int* trID2 = NULL) final;
        bool checkCollision(const CollisionModelPtr& model1, const CollisionModelPtr& model2)
            final; //, Eigen::Vector3f *storeContact = NULL);
        bool checkCollision(const CollisionModelPtr& model1,
                            const Eigen::Vector3f& point,
                            float tolerance = 0.0f) final;

        MultiCollisionResult checkMultipleCollisions(CollisionModelPtr const& model1,
                                                     CollisionModelPtr const& model2);

        /*!
        If continuous collision detection (CCD) is supported, this method can be used to detect collisions on the path
        from the current pose of the collision models to the goal poses.
        true -> collision (then the time of contact [0..1] is stored to fStoreTOC)
        */
        //bool CheckContinuousCollision (CollisionModel *model1, Eigen::Matrix4f &mGoalPose1, CollisionModel *model2, Eigen::Matrix4f &mGoalPose2, float &fStoreTOC);


        float getMinDistance(std::shared_ptr<PQP::PQP_Model> m1,
                             std::shared_ptr<PQP::PQP_Model> m2,
                             const Eigen::Matrix4f& mat1,
                             const Eigen::Matrix4f& mat2);
        float getMinDistance(std::shared_ptr<PQP::PQP_Model> m1,
                             std::shared_ptr<PQP::PQP_Model> m2,
                             const Eigen::Matrix4f& mat1,
                             const Eigen::Matrix4f& mat2,
                             Eigen::Vector3f& storeP1,
                             Eigen::Vector3f& storeP2,
                             int* storeID1,
                             int* storeID2);

        void GetPQPDistance(const std::shared_ptr<PQP::PQP_Model>& model1,
                            const std::shared_ptr<PQP::PQP_Model>& model2,
                            const Eigen::Matrix4f& matrix1,
                            const Eigen::Matrix4f& matrix2,
                            PQP::PQP_DistanceResult& pqpResult);

        /*!
        Does the underlying collision detection library support discrete collision detection.
        */
        static bool
        IsSupported_CollisionDetection()
        {
            return true;
        }

        /*!
        Does the underlying collision detection library support continuous collision detection.
        */
        static bool
        IsSupported_ContinuousCollisionDetection()
        {
            return false;
        }

        /*!
        Does the underlying collision detection library support distance calculations.
        */
        static bool
        IsSupported_DistanceCalculations()
        {
            return true;
        }

        /*!
        Does the underlying collision detection library support threadsafe access.
        E.g. multiple threads query the collision checker asynchronously.
        */
        static bool
        IsSupported_Multithreading_Threadsafe()
        {
            return false;
        }

        /*!
        Does the underlying collision detection library support multiple instances of the collision checker.
        E.g. one per thread.
        */
        static bool
        IsSupported_Multithreading_MultipleColCheckers()
        {
            return true;
        }

    protected:
        PQP::PQP_Checker* pqpChecker;
        std::unique_ptr<PQP::PQP_Model> pointModel;
    };

} // namespace VirtualRobot
