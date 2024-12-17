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
* @copyright  2013 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>

#include "../SceneObject.h"
#include "../VirtualRobot.h"

namespace VirtualRobot
{

    class Sensor;
    typedef std::shared_ptr<Sensor> SensorPtr;

    /*!
        A sensor can be attached to a GraspableSensorizedObject.
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT Sensor : public SceneObject
    {
    public:
        friend class Robot;
        friend class RobotIO;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
            Constructor with settings.
        */
        Sensor(GraspableSensorizedObjectWeakPtr parentNode,
               const std::string& name,
               VisualizationNodePtr visualization = VisualizationNodePtr(),
               const Eigen::Matrix4f& rnTrafo = Eigen::Matrix4f::Identity());

        /*!
        */
        ~Sensor() override;

        /*! returns the parent as robot node if it is a robot node otherwise a nullptr */
        RobotNodePtr getRobotNode() const;

        GraspableSensorizedObjectPtr getParentNode() const;

        /*!
            The transformation that specifies the pose of the sensor relatively to the pose of the parent node.
        */
        virtual Eigen::Matrix4f
        getParentNodeToSensorTransformation()
        {
            return rnTransformation;
        }

        /*!
            Set the local transformation.
        */
        virtual void setRobotNodeToSensorTransformation(const Eigen::Matrix4f& t);

        /*!
            Calling this SceneObject method will cause an exception, since Sensors are controlled via their node parent.
        */
        void setGlobalPose(const Eigen::Matrix4f& pose) override;

        /*!
            Print status information.
        */
        void print(bool printChildren = false, bool printDecoration = true) const override;


        /*!
            Clone this Sensor.
            \param newNode The newly created Sensor belongs to newNode.
            \param scaling Scales the visualization and transformation data.
        */
        virtual SensorPtr clone(GraspableSensorizedObjectPtr newNode, float scaling = 1.0f);


        //! Forbid cloning method from SceneObject. We need to know the new parent node for cloning
        SceneObjectPtr clone(const std::string& /*name*/,
                             CollisionCheckerPtr /*colChecker*/ = CollisionCheckerPtr(),
                             float /*scaling*/ = 1.0f) const;

        /*!
            Compute/Update the transformations of this sensor. Therefore the parent is queried for its pose.
        */
        void updatePose(bool updateChildren = true) override;

        bool initialize(
            SceneObjectPtr parent = SceneObjectPtr(),
            const std::vector<SceneObjectPtr>& children = std::vector<SceneObjectPtr>()) override;

        virtual std::string toXML(const std::string& modelPath, int tabs = 1);

    protected:
        /*!
            Update the pose according to parent pose
        */
        void updatePose(const Eigen::Matrix4f& parentPose, bool updateChildren = true) override;

        Sensor(){};


        Eigen::Matrix4f
            rnTransformation; //<! Transformation from parent's coordinate system to this sensor

        GraspableSensorizedObjectWeakPtr parentNode;

        /*!
            Derived classes must implement their clone method here.
            The visualization is already scaled, the kinematic information (i.e. transformations) have to be scaled by derived implementations.
        */
        virtual SensorPtr _clone(const GraspableSensorizedObjectPtr newNode,
                                 const VisualizationNodePtr visualizationModel,
                                 float scaling) = 0;

        SceneObject* _clone(const std::string& /*name*/,
                            CollisionCheckerPtr /*colChecker*/ = CollisionCheckerPtr(),
                            float /*scaling*/ = 1.0f) const override;
    };

} // namespace VirtualRobot
