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

#include <vector>

#include <eigen3/Eigen/Core>

#include <VirtualRobot/EndEffector/EndEffector.h>
#include <VirtualRobot/Grasping/BasicGraspQualityMeasure.h>
#include <VirtualRobot/MathTools.h>

#include "../ContactConeGenerator.h"
#include "../ConvexHullGenerator.h"
#include "../GraspStudio.h"

namespace GraspStudio
{
    /*!
        \brief An interface class for grasp quality algorithms that offer a force closure test.

        @see GraspQualityMeasureWrenchSpace
        @see VirtualRobot::BasicGraspQualityMeasure
    */
    class GRASPSTUDIO_IMPORT_EXPORT GraspQualityMeasure :
        public VirtualRobot::BasicGraspQualityMeasure
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /// Constructor.
        GraspQualityMeasure(VirtualRobot::SceneObjectPtr object,
                            float unitForce = 1.0f,
                            float frictionConeCoeff = 0.35f,
                            int frictionConeSamples = 8);

        //! Destructor
        virtual ~GraspQualityMeasure() override;


        //! Indicate whether the grasp is in force closure.
        virtual bool isGraspForceClosure() = 0;


        //! This method is used to compute a reference value that describes a perfect grasp
        virtual bool calculateObjectProperties() = 0;

        virtual VirtualRobot::MathTools::ContactPoint getSampledObjectPointsCenter();

        std::string getName() override;

        bool isValid() override;

        virtual ContactConeGeneratorPtr getConeGenerator();

    protected:
        // Methods
        bool sampleObjectPoints(int nMaxFaces = 400);


        // Friction cone relevant parameters
        float unitForce;
        float frictionCoeff;
        int frictionConeSamples;
        ContactConeGeneratorPtr coneGenerator;

        // For Object and Grasp Wrench Space Calculation
        std::vector<VirtualRobot::MathTools::ContactPoint> sampledObjectPoints; // in MM
        std::vector<VirtualRobot::MathTools::ContactPoint> sampledObjectPointsM; // converted to M
    };

} // namespace GraspStudio
