
#include "PositionSensor.h"
#include <iostream>

#include <VirtualRobot/Visualization/VisualizationNode.h>

#include "../XML/BaseIO.h"
#include "PositionSensorFactory.h"
#include "VirtualRobotException.h"

namespace VirtualRobot
{

    PositionSensor::PositionSensor(GraspableSensorizedObjectWeakPtr parentNode,
                                   const std::string& name,
                                   VisualizationNodePtr visualization,
                                   const Eigen::Matrix4f& rnTrafo) :
        Sensor(parentNode, name, visualization, rnTrafo)
    {
    }

    PositionSensor::~PositionSensor() = default;

    void
    PositionSensor::print(bool printChildren, bool printDecoration) const
    {
        if (printDecoration)
        {
            std::cout << "******** PositionSensor ********" << std::endl;
        }

        Sensor::print(printChildren, false);
    }

    SensorPtr
    PositionSensor::_clone(const GraspableSensorizedObjectPtr parentNode,
                           const VisualizationNodePtr visualizationModel,
                           float scaling)
    {
        THROW_VR_EXCEPTION_IF(scaling < 0, "Scaling must be >0");
        Eigen::Matrix4f rnt = rnTransformation;
        rnt.block(0, 3, 3, 1) *= scaling;
        SensorPtr result(new PositionSensor(parentNode, name, visualizationModel, rnt));
        return result;
    }

    std::string
    PositionSensor::toXML(const std::string& modelPath, int tabs)
    {
        std::stringstream ss;
        std::string t = "\t";
        std::string pre = "";

        for (int i = 0; i < tabs; i++)
        {
            pre += t;
        }

        ss << pre << "<Sensor type='" << PositionSensorFactory::getName() << "' name='" << name
           << "'>" << std::endl;
        std::string pre2 = pre + t;
        std::string pre3 = pre2 + t;
        ss << pre2 << "<Transform>" << std::endl;
        ss << BaseIO::toXML(rnTransformation, pre3);
        ss << pre2 << "</Transform>" << std::endl;

        if (visualizationModel)
        {
            ss << visualizationModel->toXML(modelPath, tabs + 1);
        }

        ss << pre << "</Sensor>" << std::endl;
        return ss.str();
    }

} // namespace VirtualRobot
