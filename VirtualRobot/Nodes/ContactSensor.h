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
* @author     Patrick Niklaus
* @copyright  2014 Patrick Niklaus
*             GNU Lesser General Public License
*
*/
#pragma once

#include "Sensor.h"

namespace VirtualRobot
{

    class ContactSensor;
    typedef std::shared_ptr<ContactSensor> ContactSensorPtr;

    /*!
     *  This ia a contact sensor that reports all contact points with other objects.
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT ContactSensor : public Sensor
    {
    public:
        friend class Robot;
        friend class RobotIO;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        struct ContactForce
        {
            std::string bodyName;
            Eigen::Vector3f contactPoint;
            double zForce;

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };
        struct ContactFrame
        {
            std::vector<ContactForce> forces;
        };

        /*!
          Constructor with settings.
          */
        ContactSensor(GraspableSensorizedObjectWeakPtr parentNode, const std::string& name);

        /*!
        */
        ~ContactSensor() override;

        /*!
         * Set the new contact frame for this node.
         */
        void updateSensors(const ContactSensor::ContactFrame& frame, double dt);

        /*!
         * Returns contact frame
         */
        const ContactSensor::ContactFrame& getContacts()
        {
            return frame;
        }

        /*!
         * Returns true of this node as contact with another object.
         */
        bool hasContact()
        {
            return (frame.forces.size() > 0);
        }

        /*!
          Print status information.
          */
        void print(bool printChildren = false, bool printDecoration = true) const override;


        std::string toXML(const std::string& modelPath, int tabs) override;

    protected:

        ContactSensor() {}

        /*!
        Derived classes must implement their clone method here.
        */
        SensorPtr _clone(const GraspableSensorizedObjectPtr newParentNode, const VisualizationNodePtr visualizationModel, float scaling) override;

        ContactFrame frame;
        double timestamp;
    };

} // namespace VirtualRobot

