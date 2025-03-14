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

#include "Sensor.h"

namespace VirtualRobot
{

    class PositionSensor;
    typedef std::shared_ptr<PositionSensor> PositionSensorPtr;

    /*!
        This is a dummy sensor that can be queried for positions (Via the getGlobalPose method, defined in SceneObject)
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT PositionSensor : public Sensor
    {
    public:
        friend class Robot;
        friend class RobotIO;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
            Constructor with settings.
        */
        PositionSensor(GraspableSensorizedObjectWeakPtr parentNode,
                       const std::string& name,
                       VisualizationNodePtr visualization = VisualizationNodePtr(),
                       const Eigen::Matrix4f& rnTrafo = Eigen::Matrix4f::Identity());

        /*!
        */
        ~PositionSensor() override;


        /*!
            Print status information.
        */
        void print(bool printChildren = false, bool printDecoration = true) const override;


        std::string toXML(const std::string& modelPath, int tabs) override;

    protected:
        PositionSensor(){};

        /*!
        Derived classes must implement their clone method here.
        */
        SensorPtr _clone(const GraspableSensorizedObjectPtr newRobotNode,
                         const VisualizationNodePtr visualizationModel,
                         float scaling) override;
    };

    typedef std::shared_ptr<PositionSensor> PositionSensorPtr;

} // namespace VirtualRobot
