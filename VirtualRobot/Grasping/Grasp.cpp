
#include "Grasp.h"
#include "../RobotConfig.h"
#include "..//Robot.h"
#include "../VirtualRobotException.h"
#include "EndEffector/EndEffector.h"
#include <VirtualRobot/MathTools.h>
#include "VirtualRobot/Nodes/RobotNode.h"

#include <Eigen/Dense>

#include <iomanip>

namespace VirtualRobot
{
    using std::cout;
    using std::endl;

    Grasp::Grasp(const std::string& name, const std::string& robotType, const std::string& eef,
                 const Eigen::Matrix4f& poseInTCPCoordSystem, const std::string& creation,
                 float quality, const std::string& eefPreshape)
        : poseTcp(poseInTCPCoordSystem), robotType(robotType), eef(eef), name(name),
          creation(creation), quality(quality), preshape(eefPreshape)
    {}

    Grasp::~Grasp()
        = default;

    void Grasp::print(bool printDecoration) const
    {
        if (printDecoration)
        {
            std::cout << "**** Grasp ****" << std::endl;
            std::cout << " * Robot type: " << robotType << std::endl;
            std::cout << " * End Effector: " << eef << std::endl;
            std::cout << " * EEF Preshape: " << preshape << std::endl;
        }

        std::cout << " * Name: " << name << std::endl;
        std::cout << " * Creation Method: " << creation << std::endl;
        std::cout << " * Quality: " << quality << std::endl;
        {
            // scope
            std::ostringstream sos;
            sos << std::setiosflags(std::ios::fixed);
            sos << " * Pose in EEF-TCP coordinate system:" << endl << getTransformation() << std::endl;
            std::cout << sos.str() << std::endl;
        } // scope

        if (printDecoration)
        {
            std::cout << std::endl;
        }
    }

    std::string Grasp::getRobotType() const
    {
        return robotType;
    }

    std::string Grasp::getEefName() const
    {
        return eef;
    }

    Eigen::Matrix4f Grasp::getTargetPoseGlobal(RobotPtr robot) const
    {
        THROW_VR_EXCEPTION_IF(!robot, "Null data");
        THROW_VR_EXCEPTION_IF(robot->getType() != robotType, "Robot types are not compatible: " << robot->getType() << " != " << robotType);
        EndEffectorPtr eefPtr = robot->getEndEffector(eef);

        if (!eefPtr)
        {
            VR_ERROR << "No EndEffector with name " << eef << " stored in robot " << robot->getName() << std::endl;
            return Eigen::Matrix4f::Identity();
        }

        RobotNodePtr tcpNode = eefPtr->getTcp();

        if (!tcpNode)
        {
            VR_ERROR << "No tcp with name " << eefPtr->getTcpName() << " in EndEffector " << eef << " in robot " << robot->getName() << std::endl;
            return Eigen::Matrix4f::Identity();
        }

        return tcpNode->toGlobalCoordinateSystem(getTransformation());
    }

    std::string Grasp::getName() const
    {
        return name;
    }

    std::string Grasp::getPreshapeName() const
    {
        return preshape;
    }

    Eigen::Matrix4f Grasp::getTransformation() const
    {
        return poseTcp;
    }

    void Grasp::setName(const std::string& name)
    {
        this->name = name;
    }

    std::string Grasp::toXML(int tabs) const
    {
        std::stringstream ss;
        std::string t;

        for (int i = 0; i < tabs; i++)
        {
            t += "\t";
        }

        std::string tt = t;
        tt += "\t";
        std::string ttt = tt;
        ttt += "\t";
        ss << t << "<Grasp name='" << name << "' quality='" << quality << "' Creation='" << creation;

        if (preshape.empty())
        {
            ss << "'>\n";
        }
        else
        {
            ss << "' Preshape='" << preshape << "'>\n";
        }

        ss << getTransformationXML(tt);

        if (eefConfiguration.size() > 0)
        {
            std::string tcpName;
            ss << RobotConfig::createXMLString(eefConfiguration, name, tcpName, tabs + 1);
        }

        ss << t << "</Grasp>\n";

        return ss.str();
    }

    std::string Grasp::getTransformationXML(const std::string &tabs) const {
        std::stringstream ss;
        ss << tabs << "<Transform>\n";
        ss << MathTools::getTransformXMLString(poseTcp, tabs + "\t");
        ss << tabs << "</Transform>\n";
        return ss.str();
    }

    void Grasp::setTransformation(const Eigen::Matrix4f& tcp2Object)
    {
        poseTcp = tcp2Object;
    }

    Eigen::Matrix4f Grasp::getTcpPoseGlobal(const Eigen::Matrix4f& objectPose) const
    {
        Eigen::Matrix4f result = objectPose * getTransformation().inverse();
        return result;
    }

    Eigen::Matrix4f Grasp::getTcpPoseRobotRoot(const Eigen::Matrix4f& objectPose, const RobotPtr& robot) const
    {
        return robot->getGlobalPose().inverse() * getTcpPoseGlobal(objectPose);
    }

    Eigen::Matrix4f Grasp::getObjectTargetPoseGlobal(const Eigen::Matrix4f& graspingPose) const
    {
        Eigen::Matrix4f result = graspingPose * getTransformation();
        return result;
    }

    VirtualRobot::GraspPtr Grasp::clone() const
    {
        GraspPtr result(new Grasp(name, robotType, eef, getTransformation(), creation, quality, preshape));
        result->setConfiguration(eefConfiguration);
        return result;
    }

    void Grasp::setConfiguration(const std::map< std::string, float >& config)
    {
        eefConfiguration = config;
    }

    std::map< std::string, float > Grasp::getConfiguration() const
    {
        return eefConfiguration;
    }

    void Grasp::setPreshape(const std::string& preshapeName)
    {
        preshape = preshapeName;
    }

    float Grasp::getQuality() const
    {
        return quality;
    }

    void Grasp::setQuality(float q)
    {
        quality = q;
    }

    std::string Grasp::getCreationMethod() const
    {
        return creation;
    }


} //  namespace
