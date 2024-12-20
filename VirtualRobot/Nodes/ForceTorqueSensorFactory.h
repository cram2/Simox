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

#include "../VirtualRobot.h"
#include "SensorFactory.h"

namespace VirtualRobot
{
    class Sensor;

    class VIRTUAL_ROBOT_IMPORT_EXPORT ForceTorqueSensorFactory : public SensorFactory
    {
    public:
        ForceTorqueSensorFactory();
        ~ForceTorqueSensorFactory() override;

        //! Standard init method
        SensorPtr
        createSensor(GraspableSensorizedObjectPtr node,
                     const std::string& name,
                     VisualizationNodePtr visualization = VisualizationNodePtr(),
                     const Eigen::Matrix4f& rnTrafo = Eigen::Matrix4f::Identity()) const override;

        /*!
            Create sensor from XML tag.
        */
        SensorPtr createSensor(GraspableSensorizedObjectPtr node,
                               const rapidxml::xml_node<char>* sensorXMLNode,
                               BaseIO::RobotDescription loadMode = RobotIO::eFull,
                               const std::string basePath = std::string()) const override;

        // AbstractFactoryMethod
    public:
        static std::string getName();
        static std::shared_ptr<SensorFactory> createInstance(void*);

    private:
        static SubClassRegistry registry;
    };

} // namespace VirtualRobot
