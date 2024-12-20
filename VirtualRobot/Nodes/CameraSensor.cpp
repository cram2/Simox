
#include "CameraSensor.h"

#include <iostream>

#include <VirtualRobot/Visualization/VisualizationNode.h>

#include "CameraSensorFactory.h"
#include "VirtualRobotException.h"

namespace VirtualRobot
{

    CameraSensor::CameraSensor(GraspableSensorizedObjectWeakPtr parentNode,
                               const std::string& name,
                               VisualizationNodePtr visualization,
                               const Eigen::Matrix4f& rnTrafo) :
        Sensor(parentNode, name, visualization, rnTrafo)
    {
    }

    CameraSensor::~CameraSensor() = default;

    void
    CameraSensor::print(bool printChildren, bool printDecoration) const
    {
        if (printDecoration)
        {
            std::cout << "******** CameraSensor ********" << std::endl;
        }

        Sensor::print(printChildren, false);
    }

    SensorPtr
    CameraSensor::_clone(const GraspableSensorizedObjectPtr newParentNode,
                         const VisualizationNodePtr visualizationModel,
                         float scaling)
    {
        THROW_VR_EXCEPTION_IF(scaling < 0, "Scaling must be >0");
        Eigen::Matrix4f rnt = rnTransformation;
        rnt.block(0, 3, 3, 1) *= scaling;
        SensorPtr result(new CameraSensor(newParentNode, name, visualizationModel, rnt));
        return result;
    }

    std::string
    CameraSensor::toXML(const std::string& modelPath, int tabs)
    {
        std::stringstream ss;
        std::string t = "\t";
        std::string pre = "";

        for (int i = 0; i < tabs; i++)
        {
            pre += t;
        }

        ss << pre << "<Sensor type='" << CameraSensorFactory::getName() << "' name='" << name
           << "'>" << std::endl;
        std::string pre2 = pre + t;
        ss << pre << "<Transform>" << std::endl;
        ss << BaseIO::toXML(rnTransformation, pre2);
        ss << pre << "</Transform>" << std::endl;

        if (visualizationModel)
        {
            ss << visualizationModel->toXML(modelPath, tabs + 1);
        }

        ss << pre << "</Sensor>" << std::endl;
        return ss.str();
    }

} // namespace VirtualRobot
