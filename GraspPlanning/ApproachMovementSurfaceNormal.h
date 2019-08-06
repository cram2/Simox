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
* @package    GraspStudio
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#pragma once

#include "GraspStudio.h"
#include "ApproachMovementGenerator.h"

#include <VirtualRobot/SceneObject.h>

#include <memory>
#include <random>
#include <vector>


namespace GraspStudio
{
    /*!
     * This class generates grasping configs by sampling a random surface
     * position of the object and setting the EEF to a surface normal aligned
     * position.
     * The remaining free DoF (the rotation around the normal) is set randomly.
     * Then the EEF is moved along the normal until a collision is detected
     * or the GCP hits the object.
     * If needed, the EEF is moved back until a collision-free pose is found.
     *
     * Internally the EEF is cloned.
     */
    class GRASPSTUDIO_IMPORT_EXPORT ApproachMovementSurfaceNormal : public ApproachMovementGenerator
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
         * To generate approach movements an object and an end effector has to be specified.
         * \param object The object.
         * \param eef The end effector.
         * \param graspPreshape An optional preshape that can be used in order to "open" the eef.
         * \param maxRandDist
         *      If >0, the resulting apporach pose is randomly moved in the approach direction
         *      (away from the object) in order to create different distances to the object.
         * \param usefaceAreaDistribution
         *      If true, the probability of a face being selected is proportional to its area.
         *      If false, all faces are selected with equal probability.
        */
        ApproachMovementSurfaceNormal(VirtualRobot::SceneObjectPtr object, VirtualRobot::EndEffectorPtr eef,
                                      const std::string& graspPreshape = "", float maxRetreatDist = 0.0f,
                                      bool useFaceAreaDistribution = false);
        //! Destructor
        virtual ~ApproachMovementSurfaceNormal() override;

        //! Creates a new pose for approaching
        Eigen::Matrix4f createNewApproachPose() override;

        //! Returns a position with normal on the surface of the object.
        bool getPositionOnObject(Eigen::Vector3f& storePos, Eigen::Vector3f& storeApproachDir);

        //! Sets EEF to a position so that the Z component of the GCP coord system is aligned with -approachDir
        bool virtual setEEFToApproachPose(const Eigen::Vector3f& position, const Eigen::Vector3f& approachDir);

        void moveEEFAway(const Eigen::Vector3f& approachDir, float step, int maxLoops = 1000);

        Eigen::Matrix4f getEEFPose();
        bool setEEFPose(const Eigen::Matrix4f& pose);


    protected:

        /// The random engine.
        std::default_random_engine randomEngine { std::random_device{}() };

        /// Uniform distribiton over face indices.
        std::uniform_int_distribution<std::size_t> distribUniform;

        /// Indicates whether distribFaceAreas shall be used.
        bool useFaceAreasDistrib = false;

        /// Distribution with probability of a face proportional to its area.
        /// Only initialized if useFaceAreasDistrib is true.
        std::discrete_distribution<std::size_t> distribFaceAreas;


        /// Distribution to draw random retreat distances from.
        std::uniform_real_distribution<float> distribRetreatDistance;

    };
}

