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

#include <VirtualRobot/EndEffector/EndEffector.h>
#include <VirtualRobot/Grasping/GraspSet.h>
#include <VirtualRobot/SceneObject.h>

#include <GraspPlanning/GraspPlanner/GraspPlannerEvaluation.h>
#include <GraspPlanning/GraspStudio.h>

namespace GraspStudio
{
    /*!
    *
    * \brief An interface for grasp planners.
    *
    */
    class GRASPSTUDIO_IMPORT_EXPORT GraspPlanner
    {
    public:
        /*!
            Constructor
            \param graspSet Append planned grasps to this set.
        */
        GraspPlanner(VirtualRobot::GraspSetPtr graspSet);

        //! destructor
        virtual ~GraspPlanner();

        void setVerbose(bool enable);


        /*!
            Creates new grasps.
            \param nrGrasps The number of grasps to be planned.
            \param timeOutMS The time out in milliseconds. Planning is stopped when this time is exceeded. Disabled when zero.
            \param obstacles
            \return Number of planned grasps.
        */
        virtual int
        plan(int nrGrasps, int timeOutMS = 0, VirtualRobot::SceneObjectSetPtr obstacles = {}) = 0;

        /*!
            Returns all grasps that have been generated. These grasps are also stored in the graspSet that was specified on construction.
        */
        std::vector<VirtualRobot::GraspPtr> getPlannedGrasps();

        /*!
         * \brief getEvaluation
         * \return The current evaluation of the grasp planner.
         */
        GraspPlannerEvaluation getEvaluation() const;
        /// Clear the evaluation.
        void clearEvaluation();


    protected:
        bool verbose;
        VirtualRobot::GraspSetPtr graspSet;
        std::vector<VirtualRobot::GraspPtr> plannedGrasps;

        GraspPlannerEvaluation eval;
    };
} // namespace GraspStudio
