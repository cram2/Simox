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
* @author     Peter Kaiser, Nikolaus Vahrenkamp
* @copyright  2011 Peter Kaiser, Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#pragma once

#include "../VirtualRobot.h"
#include "WorkspaceRepresentation.h"

namespace VirtualRobot
{


    class VIRTUAL_ROBOT_IMPORT_EXPORT NaturalPosture :
        public WorkspaceRepresentation,
        public std::enable_shared_from_this<NaturalPosture>
    {
    public:
        friend class CoinVisualizationFactory;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        NaturalPosture(RobotPtr robot);

        void customInitialize() override;


        void addPose(const Eigen::Matrix4f& pose) override;

        // /*!
        //     Returns true, if the corresponding NaturalPosture entry is non zero.
        // */
        // bool isReachable(const Eigen::Matrix4f& globalPose);

        // /*!
        //     Returns all reachable grasps that can be applied at the current position of object.
        // */
        // GraspSetPtr getReachableGrasps(GraspSetPtr grasps, ManipulationObjectPtr object);


        // //! returns a random pose that is covered by the workspace data.
        // Eigen::Matrix4f sampleReachablePose();

        // /*!
        //     Creates a deep copy of this data structure. A NaturalPosturePtr is returned.
        // */
        // WorkspaceRepresentationPtr clone() override;


    protected:
        float evaluate();
    };


} // namespace VirtualRobot
