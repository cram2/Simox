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
* @author     Andre Meixner
* @copyright  2020 Andre Meixner
*             GNU Lesser General Public License
*
*/

#pragma once

#include "SceneObject.h"

namespace VirtualRobot
{

    class GraspableSensorizedObject;

    using GraspableSensorizedObjectPtr = std::shared_ptr<GraspableSensorizedObject>;

    class GraspableSensorizedObject : public SceneObject
    {
    public:
        GraspableSensorizedObject(const std::string& name,
                                  VisualizationNodePtr visualization = VisualizationNodePtr(),
                                  CollisionModelPtr collisionModel = CollisionModelPtr(),
                                  const Physics& p = Physics(),
                                  CollisionCheckerPtr colChecker = CollisionCheckerPtr());

        bool initialize(
            SceneObjectPtr parent = SceneObjectPtr(),
            const std::vector<SceneObjectPtr>& children = std::vector<SceneObjectPtr>()) override;

        bool hasGraspSet(GraspSetPtr graspSet);
        bool hasGraspSet(const std::string& robotType, const std::string& eef);

        /*!
        Appends a grasp set. Note, that only one grasp set per EEF is allowed.
    */
        void addGraspSet(GraspSetPtr graspSet);

        /*!
     * \brief includeGraspSet
     * \param graspSet
     */
        void includeGraspSet(GraspSetPtr graspSet);

        /*!
        Get grasp set for the given end effector. In case multiple grasp sets for the eef are present, the first one is returned.
        An empty GraspSetPtr is returned when no GraspSet for eef is found.
    */
        GraspSetPtr getGraspSet(EndEffectorPtr eef);

        /*!
        Get grasp set for the given robotType and end effector. In case multiple grasp sets for the robot/eef combination are present, the first one is returned.
        An empty GraspSetPtr is returned when no GraspSet for robot&eef is found.
    */
        GraspSetPtr getGraspSet(const std::string& robotType, const std::string& eefName);

        /*!
        Get grasp set by name.
        \param name The name of the grasp set.
        \return An empty GraspSetPtr is returned when no GraspSet with the given name is found.
    */
        GraspSetPtr getGraspSet(const std::string& name);

        /*!
        Get grasp set vector
    */
        const std::vector<GraspSetPtr>& getAllGraspSets();

        virtual SensorPtr getSensor(const std::string& name) const;
        virtual bool hasSensor(const std::string& name) const;
        virtual std::vector<SensorPtr> getSensors() const;
        virtual bool registerSensor(SensorPtr sensor);

        /*! Clones this grasps sets and appends to other */
        void appendGraspSetsTo(GraspableSensorizedObjectPtr other) const;

        /*! Clones this sensors and appends to other*/
        void appendSensorsTo(GraspableSensorizedObjectPtr other) const;

        void printGrasps() const;

    protected:
        GraspableSensorizedObject()
        {
        }

        std::string getGraspableSensorizedObjectXML(const std::string& modelPathRelative = "models",
                                                    bool storeSensors = true,
                                                    int tabs = 0);

        std::vector<GraspSetPtr> graspSets;
        std::vector<SensorPtr> sensors;
    };

} // namespace VirtualRobot
