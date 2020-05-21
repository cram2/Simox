#include "ApproachMovementGenerator.h"
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/SceneObject.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/CollisionDetection/CollisionModel.h>
#include <VirtualRobot/EndEffector/EndEffector.h>
#include <VirtualRobot/MathTools.h>
#include <VirtualRobot/RobotConfig.h>
#include <VirtualRobot/math/Helpers.h>
#include <iostream>

using namespace std;

namespace GraspStudio
{
    void ApproachMovementGenerator::setVerbose(bool v)
    {
        verbose = v;
    }

    ApproachMovementGenerator::ApproachMovementGenerator(VirtualRobot::SceneObjectPtr object, VirtualRobot::EndEffectorPtr eef, const std::string& graspPreshape)
        : object(object), eef(eef), graspPreshape(graspPreshape)
    {
        name = "ApproachMovementGenerator";
        THROW_VR_EXCEPTION_IF(!object, "NULL object?!");
        THROW_VR_EXCEPTION_IF(!object->getCollisionModel(), "No collision model for object " << object->getName());
        THROW_VR_EXCEPTION_IF(!eef, "NULL eef?!");
        THROW_VR_EXCEPTION_IF(!eef->getGCP(), "Need a GraspCenterPoint Node defined in EEF " << eef->getName());

        objectModel = object->getCollisionModel()->getTriMeshModel();
        THROW_VR_EXCEPTION_IF(!objectModel, "NULL trimeshmodel of object " << object->getName());
        THROW_VR_EXCEPTION_IF(objectModel->faces.size() == 0, "no faces in trimeshmodel of object " << object->getName());

        eefRobot = eef->createEefRobot(eef->getName(), eef->getName());
        THROW_VR_EXCEPTION_IF(!eefRobot, "Failed cloning EEF " << eef->getName());


        eef_cloned = eefRobot->getEndEffector(eef->getName());
        THROW_VR_EXCEPTION_IF(!eef_cloned, "No EEF with name " << eef->getName() << " in cloned robot?!");
        THROW_VR_EXCEPTION_IF(!eef_cloned->getGCP(), "No GCP in EEF with name " << eef->getName());

        if (!graspPreshape.empty())
        {
            THROW_VR_EXCEPTION_IF(!eef_cloned->hasPreshape(graspPreshape), "Preshape with name " << graspPreshape << " not present in EEF");
            eef_cloned->setPreshape(graspPreshape);
        }
        approachDirGlobal << 1.0f, 0, 0;
    }

    ApproachMovementGenerator::~ApproachMovementGenerator()
        = default;


    VirtualRobot::RobotPtr ApproachMovementGenerator::getEEFRobotClone()
    {
        return eefRobot;
    }

    bool ApproachMovementGenerator::setEEFPose(const Eigen::Matrix4f& pose)
    {
        VirtualRobot::RobotNodePtr tcp;
        if (!graspPreshape.empty()
            && eef_cloned->hasPreshape(graspPreshape)
            && eef_cloned->getPreshape(graspPreshape)->getTCP())
        {
            tcp = eef_cloned->getPreshape(graspPreshape)->getTCP();
        }
        else
        {
            tcp = eef_cloned->getGCP();
        }
        eefRobot->setGlobalPoseForRobotNode(tcp, pose);
        return true;
    }

    bool ApproachMovementGenerator::updateEEFPose(const Eigen::Vector3f& deltaPosition)
    {
        Eigen::Matrix4f pose = eef_cloned->getGCP()->getGlobalPose();
        math::Helpers::Position(pose) += deltaPosition;
        return setEEFPose(pose);
    }

    bool ApproachMovementGenerator::updateEEFPose(const Eigen::Matrix4f& deltaPose)
    {
        Eigen::Matrix4f pose = eef_cloned->getGCP()->getGlobalPose();
        pose = deltaPose * pose;
        return setEEFPose(pose);
    }

    Eigen::Matrix4f ApproachMovementGenerator::getEEFPose()
    {
        VirtualRobot::RobotNodePtr tcp;
        if (!graspPreshape.empty() && eef_cloned->hasPreshape(graspPreshape) && eef_cloned->getPreshape(graspPreshape)->getTCP())
        {
            tcp = eef_cloned->getPreshape(graspPreshape)->getTCP();
        }
        else
        {
            tcp = eef_cloned->getGCP();
        }
        return tcp->getGlobalPose();
    }

    bool ApproachMovementGenerator::setEEFToRandomApproachPose()
    {
        Eigen::Matrix4f pose = createNewApproachPose();
        return setEEFPose(pose);
    }

    std::string ApproachMovementGenerator::getGCPJoint()
    {
        return eef_cloned->getGCP()->getName();
    }

    VirtualRobot::SceneObjectPtr ApproachMovementGenerator::getObject()
    {
        return object;
    }

    VirtualRobot::EndEffectorPtr ApproachMovementGenerator::getEEF()
    {
        return eef_cloned;
    }

    VirtualRobot::EndEffectorPtr ApproachMovementGenerator::getEEFOriginal()
    {
        return eef;
    }

    Eigen::Vector3f ApproachMovementGenerator::getApproachDirGlobal()
    {
        return approachDirGlobal;
    }

    std::string ApproachMovementGenerator::getName()
    {
        return name;
    }


    void ApproachMovementGenerator::openHand()
    {
        if (eef_cloned)
        {
            if (!graspPreshape.empty())
            {
                eef_cloned->setPreshape(graspPreshape);
            }
            else
            {
                eef_cloned->openActors();
            }
        }
    }

}
