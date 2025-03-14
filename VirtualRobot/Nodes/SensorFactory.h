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

#include <eigen3/Eigen/Core>

#include "../AbstractFactoryMethod.h"
#include "../VirtualRobot.h"
#include "../XML/RobotIO.h"
#include "GraspableSensorizedObject.h"
#include "Sensor.h"

// using forward declarations here, so that the rapidXML header does not have to be parsed when this file is included
namespace rapidxml
{
    template <class Ch>
    class xml_node;
}

namespace VirtualRobot
{

    class VIRTUAL_ROBOT_IMPORT_EXPORT SensorFactory :
        public AbstractFactoryMethod<SensorFactory, void*>
    {
    public:
        SensorFactory()
        {
            ;
        }

        virtual ~SensorFactory()
        {
            ;
        }

        //! Standard init method
        virtual SensorPtr
        createSensor(GraspableSensorizedObjectPtr /*node*/,
                     const std::string& /*name*/,
                     VisualizationNodePtr /*visualization*/ = VisualizationNodePtr(),
                     const Eigen::Matrix4f& /*rnTrafo*/ = Eigen::Matrix4f::Identity()) const
        {
            return SensorPtr();
        }

        /*!
            Create sensor from XML tag. Factories of custom sensors can initialize with this method.
        */
        virtual SensorPtr
        createSensor(GraspableSensorizedObjectPtr /*node*/,
                     const rapidxml::xml_node<char>* /*sensorXMLNode*/,
                     BaseIO::RobotDescription /*loadMode*/ = RobotIO::eFull,
                     const std::string /*basePath*/ = std::string()) const
        {
            return SensorPtr();
        }
    };

    typedef std::shared_ptr<SensorFactory> SensorFactoryPtr;

} // namespace VirtualRobot
