
#include "ContactSensor.h"

#include "../XML/BaseIO.h"
#include "ContactSensorFactory.h"

namespace VirtualRobot
{

    ContactSensor::ContactSensor(GraspableSensorizedObjectWeakPtr parentNode,
                                 const std::string& name) :
        Sensor(parentNode, name), timestamp(0.0)
    {
    }

    ContactSensor::~ContactSensor() = default;

    void
    ContactSensor::print(bool printChildren, bool printDecoration) const
    {
        if (printDecoration)
        {
            std::cout << "******** ContactSensor ********" << std::endl;
        }

        Sensor::print(printChildren, false);
    }

    SensorPtr
    ContactSensor::_clone(const GraspableSensorizedObjectPtr newParentNode,
                          const VisualizationNodePtr /*visualizationModel*/,
                          float /*scaling*/)
    {
        SensorPtr result(new ContactSensor(newParentNode, name /*, rnTransformation*/));
        return result;
    }

    std::string
    ContactSensor::toXML(const std::string& /*modelPath*/, int tabs)
    {
        std::stringstream ss;
        std::string t = "\t";
        std::string pre = "";

        for (int i = 0; i < tabs; i++)
        {
            pre += t;
        }

        ss << pre << "<Sensor type='" << ContactSensorFactory::getName() << "'/>" << std::endl;
        std::string pre2 = pre + t;

        return ss.str();
    }

    void
    ContactSensor::updateSensors(const ContactSensor::ContactFrame& frame, double dt)
    {
        this->frame = frame;
        timestamp += dt;
    }

} // namespace VirtualRobot
