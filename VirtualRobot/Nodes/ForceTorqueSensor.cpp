
#include "ForceTorqueSensor.h"
#include "ForceTorqueSensorFactory.h"
#include "../XML/BaseIO.h"
#include "VirtualRobotException.h"

namespace VirtualRobot
{
    using std::cout;
    using std::endl;

    ForceTorqueSensor::ForceTorqueSensor(GraspableSensorizedObjectWeakPtr parentNode,
                                         const std::string& name,
                                         const Eigen::Matrix4f& rnTrafo) :
        Sensor(parentNode, name, VisualizationNodePtr(), rnTrafo),
        forceTorqueValues(6)
    {
        forceTorqueValues.setZero();
    }


    ForceTorqueSensor::~ForceTorqueSensor()
    = default;

    Eigen::Vector3f ForceTorqueSensor::getForce() const
    {
        return forceTorqueValues.head(3);
    }

    Eigen::Vector3f ForceTorqueSensor::getTorque() const
    {
        return forceTorqueValues.tail(3);
    }

    const Eigen::VectorXf& ForceTorqueSensor::getForceTorque()
    {
        return forceTorqueValues;
    }

    Eigen::Vector3f ForceTorqueSensor::getAxisTorque()
    {
        Eigen::Vector3f torqueVector = forceTorqueValues.tail(3);

        // project onto joint axis
        Eigen::Vector3f zAxis = this->globalPose.block(0, 2, 3, 1);
        Eigen::Vector3f axisTorque = (torqueVector.dot(zAxis)) * zAxis;

        return axisTorque;
    }



    void ForceTorqueSensor::print(bool printChildren, bool printDecoration) const
    {
        if (printDecoration)
        {
            std::cout << "******** ForceTorqueSensor ********" << std::endl;
        }

        Sensor::print(printChildren, false);
    }


    SensorPtr ForceTorqueSensor::_clone(const GraspableSensorizedObjectPtr newParentNode, const VisualizationNodePtr /*visualizationModel*/, float scaling)
    {
        THROW_VR_EXCEPTION_IF(scaling < 0, "Scaling must be >0");
        Eigen::Matrix4f rnt = rnTransformation;
        rnt.block(0, 3, 3, 1) *= scaling;
        ForceTorqueSensorPtr result(new ForceTorqueSensor(newParentNode, name, rnt));
        result->updateSensors(forceTorqueValues);
        return result;
    }


    std::string ForceTorqueSensor::toXML(const std::string& /*modelPath*/, int tabs)
    {
        std::stringstream ss;
        std::string t = "\t";
        std::string pre = "";

        for (int i = 0; i < tabs; i++)
        {
            pre += t;
        }

        ss << pre << "<Sensor type='" << ForceTorqueSensorFactory::getName() << "' name='" << name << "'>" << std::endl;
        std::string pre2 = pre + t;
        ss << pre << "<Transform>" << std::endl;
        ss << BaseIO::toXML(rnTransformation, pre2);
        ss << pre << "</Transform>" << std::endl;


        ss << pre << "</Sensor>" << std::endl;
        return ss.str();
    }

    void ForceTorqueSensor::updateSensors(const Eigen::VectorXf& newForceTorque)
    {
        forceTorqueValues = newForceTorque;
    }

} // namespace VirtualRobot
