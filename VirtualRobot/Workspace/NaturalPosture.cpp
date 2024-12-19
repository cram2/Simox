#include "NaturalPosture.h"

#include <cfloat>
#include <climits>
#include <cmath>
#include <fstream>
#include <stdexcept>

#include <VirtualRobot/IK/CompositeDiffIK/Soechting.h>
#include <VirtualRobot/IK/CompositeDiffIK/SoechtingNullspaceGradient.h>

#include "../Grasping/Grasp.h"
#include "../Grasping/GraspSet.h"
#include "../ManipulationObject.h"
#include "../Nodes/RobotNode.h"
#include "../Robot.h"
#include "../RobotNodeSet.h"
#include "../VirtualRobotException.h"
#include "IK/CompositeDiffIK/CompositeDiffIK.h"
#include "IK/CompositeDiffIK/SoechtingNullspaceGradient.h"
#include "IK/CompositeDiffIK/SoechtingNullspaceGradientWithWrist.h"
#include "Logging.h"
#include "VirtualRobot.h"
#include "WorkspaceData.h"

namespace VirtualRobot
{

    NaturalPosture::NaturalPosture(RobotPtr robot) : WorkspaceRepresentation(robot)
    {
        type = "NaturalPosture";
    }

    void
    NaturalPosture::customInitialize()
    {
        // auto target1 = ik->addTarget(robot->getRobotNode(tcpNode->getName()), VirtualRobot::IKSolver::All);
    }

    float
    NaturalPosture::evaluate()
    {
        SoechtingNullspaceGradientWithWristPtr soechtingNullspaceGradient;

        CompositeDiffIK::Parameters params;

        const CompositeDiffIK::TargetPtr target(new CompositeDiffIK::Target(
            tcpNode, tcpNode->getGlobalPose(), VirtualRobot::IKSolver::All));

        if (robot->getName() == "Armar6" && nodeSet->getName() == "RightArm")
        {
            VirtualRobot::SoechtingNullspaceGradientWithWrist::ArmJointsWithWrist armjoints;
            armjoints.clavicula = robot->getRobotNode("ArmR1_Cla1");
            armjoints.shoulder1 = robot->getRobotNode("ArmR2_Sho1");
            armjoints.shoulder2 = robot->getRobotNode("ArmR3_Sho2");
            armjoints.shoulder3 = robot->getRobotNode("ArmR4_Sho3");
            armjoints.elbow = robot->getRobotNode("ArmR5_Elb1");
            armjoints.forearm = robot->getRobotNode("ArmR6_Elb2");
            armjoints.wristAdduction = robot->getRobotNode("ArmR7_Wri1");
            armjoints.wristExtension = robot->getRobotNode("ArmR8_Wri2");

            soechtingNullspaceGradient.reset(new VirtualRobot::SoechtingNullspaceGradientWithWrist(
                target, "ArmR2_Sho1", VirtualRobot::Soechting::ArmType::Right, armjoints));
            soechtingNullspaceGradient->kP = 1.0;
        }
        else if (robot->getName() == "Armar6" && nodeSet->getName() == "LeftArm")
        {
            VirtualRobot::SoechtingNullspaceGradientWithWrist::ArmJointsWithWrist armjoints;
            armjoints.clavicula = robot->getRobotNode("ArmL1_Cla1");
            armjoints.shoulder1 = robot->getRobotNode("ArmL2_Sho1");
            armjoints.shoulder2 = robot->getRobotNode("ArmL3_Sho2");
            armjoints.shoulder3 = robot->getRobotNode("ArmL4_Sho3");
            armjoints.elbow = robot->getRobotNode("ArmL5_Elb1");
            armjoints.forearm = robot->getRobotNode("ArmL6_Elb2");
            armjoints.wristAdduction = robot->getRobotNode("ArmL7_Wri1");
            armjoints.wristExtension = robot->getRobotNode("ArmL8_Wri2");

            soechtingNullspaceGradient.reset(new VirtualRobot::SoechtingNullspaceGradientWithWrist(
                target, "ArmL2_Sho1", VirtualRobot::Soechting::ArmType::Left, armjoints));
            soechtingNullspaceGradient->kP = 1.0;
        }
        else
        {
            throw std::runtime_error("Unknown robot");
        }

        const auto weightedDiff = soechtingNullspaceGradient->getGradient(params, -1);
        const float e1 = weightedDiff.squaredNorm();

        return std::sqrt(e1);
    }

    void
    NaturalPosture::addPose(const Eigen::Matrix4f& pose)
    {
        Eigen::Matrix4f p = pose;
        toLocal(p);

        std::array<float, 6> x;
        matrix2Vector(p, x.data());

        // check for achieved values
        for (int i = 0; i < 6; i++)
        {
            if (x[i] < achievedMinValues[i])
            {
                achievedMinValues[i] = x[i];
            }

            if (x[i] > achievedMaxValues[i])
            {
                achievedMaxValues[i] = x[i];
            }
        }

        // get voxels
        std::array<unsigned int, 6> v;
        if (getVoxelFromPose(x.data(), v.data()))
        {
            // value corresponds to the norm of the gradient => costs
            const float value = evaluate();

            // scale and clip costs
            const float scaledCosts = value / 10 * 6;
            if (scaledCosts > 1.0)
            {
                VR_WARNING << "Costs too large: " << scaledCosts;
            }

            const float costs = std::clamp(scaledCosts, 0.F, 1.0F); // [0,1]

            // convert costs to objective / quality measure (similar to manipulability; the higher, the better)
            const float objective = 1 - costs; // [0,1]

            // convert to fixed size type
            const auto cellValue =
                static_cast<unsigned char>(objective * static_cast<float>(UCHAR_MAX) + 0.5F);

            // update grid
            const auto oldCellValue = data->get(v.data());
            if (oldCellValue == 0) // if unset
            {
                data->setDatum(v.data(), cellValue);
            }
            else
            {
                if (cellValue < oldCellValue)
                {
                    data->setDatum(v.data(), cellValue);
                }
            }
        }
        else
        {
            VR_WARNING << "Could not get voxel from pose!";
        }

        buildUpLoops++;
    }

    /*
        void NaturalPosture::addRandomTCPPoses(unsigned int loops, bool checkForSelfCollisions)
        {
            THROW_VR_EXCEPTION_IF(!data || !nodeSet || !tcpNode, "NaturalPosture data not initialized");

            std::vector<float> c;
            nodeSet->getJointValues(c);
            bool visuSate = robot->getUpdateVisualizationStatus();
            robot->setUpdateVisualization(false);

            for (unsigned int i = 0; i < loops; i++)
            {
                if (setRobotNodesToRandomConfig(nodeSet, checkForSelfCollisions))
                {
                    addCurrentTCPPose();
                }
                else
                {
                    VR_WARNING << "Could not find collision-free configuration...";
                }
            }

            robot->setUpdateVisualization(visuSate);
            robot->setJointValues(nodeSet, c);
        }*/

    // bool NaturalPosture::isReachable(const Eigen::Matrix4f& globalPose)
    // {
    //     return isCovered(globalPose);
    // }

    // VirtualRobot::GraspSetPtr NaturalPosture::getReachableGrasps(GraspSetPtr grasps, ManipulationObjectPtr object)
    // {
    //     THROW_VR_EXCEPTION_IF(!object, "no object");
    //     THROW_VR_EXCEPTION_IF(!grasps, "no grasps");

    //     GraspSetPtr result(new GraspSet(grasps->getName(), grasps->getRobotType(), grasps->getEndEffector()));

    //     for (unsigned int i = 0; i < grasps->getSize(); i++)
    //     {
    //         Eigen::Matrix4f m = grasps->getGrasp(i)->getTcpPoseGlobal(object->getGlobalPose());

    //         if (isReachable(m))
    //         {
    //             result->addGrasp(grasps->getGrasp(i));
    //         }
    //     }

    //     return result;
    // }

    // Eigen::Matrix4f NaturalPosture::sampleReachablePose()
    // {
    //     return sampleCoveredPose();
    // }

    // VirtualRobot::WorkspaceRepresentationPtr NaturalPosture::clone()
    // {
    //     VirtualRobot::NaturalPosturePtr res(new NaturalPosture(robot));
    //     res->setOrientationType(this->orientationType);
    //     res->versionMajor = this->versionMajor;
    //     res->versionMinor = this->versionMinor;
    //     res->nodeSet = this->nodeSet;
    //     res->type = this->type;

    //     res->baseNode = this->baseNode;
    //     res->tcpNode = this->tcpNode;
    //     res->staticCollisionModel = this->staticCollisionModel;
    //     res->dynamicCollisionModel = this->dynamicCollisionModel;
    //     res->buildUpLoops = this->buildUpLoops;
    //     res->collisionConfigs = this->collisionConfigs;
    //     res->discretizeStepTranslation = this->discretizeStepTranslation;
    //     res->discretizeStepRotation = this->discretizeStepRotation;
    //     memcpy(res->minBounds, this->minBounds, sizeof(float) * 6);
    //     memcpy(res->maxBounds, this->maxBounds, sizeof(float) * 6);
    //     memcpy(res->numVoxels, this->numVoxels, sizeof(float) * 6);
    //     memcpy(res->achievedMinValues, this->achievedMinValues, sizeof(float) * 6);
    //     memcpy(res->achievedMaxValues, this->achievedMaxValues, sizeof(float) * 6);
    //     memcpy(res->spaceSize, this->spaceSize, sizeof(float) * 6);

    //     res->adjustOnOverflow = this->adjustOnOverflow;
    //     res->data.reset(this->data->clone());

    //     return res;
    // }

} // namespace VirtualRobot
