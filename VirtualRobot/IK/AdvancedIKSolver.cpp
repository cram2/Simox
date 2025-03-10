#include "AdvancedIKSolver.h"

#include <algorithm>

#include <Eigen/Geometry>

#include "../CollisionDetection/CDManager.h"
#include "../CollisionDetection/CollisionChecker.h"
#include "../EndEffector/EndEffector.h"
#include "../Grasping/Grasp.h"
#include "../Grasping/GraspSet.h"
#include "../ManipulationObject.h"
#include "../Nodes/RobotNodePrismatic.h"
#include "../Nodes/RobotNodeRevolute.h"
#include "../Obstacle.h"
#include "../Robot.h"
#include "../RobotConfig.h"
#include "../VirtualRobotException.h"
#include "../Workspace/Reachability.h"
#include "Logging.h"

namespace VirtualRobot
{
    using std::endl;

    AdvancedIKSolver::AdvancedIKSolver(RobotNodeSetPtr rns) : IKSolver(), rns(rns)
    {
        verbose = false;
        THROW_VR_EXCEPTION_IF(!rns, "Null data");
        tcp = rns->getTCP();
        THROW_VR_EXCEPTION_IF(!tcp, "no tcp");
        setMaximumError();
    }

    void
    AdvancedIKSolver::collisionDetection(SceneObjectPtr avoidCollisionsWith)
    {
        cdm.reset();

        if (avoidCollisionsWith)
        {
            cdm.reset(new CDManager(avoidCollisionsWith->getCollisionChecker()));
            cdm->addCollisionModel(avoidCollisionsWith);
            cdm->addCollisionModel(rns);
        }
    }

    void
    AdvancedIKSolver::collisionDetection(ObstaclePtr avoidCollisionsWith)
    {
        SceneObjectPtr so = std::dynamic_pointer_cast<SceneObject>(avoidCollisionsWith);
        collisionDetection(so);
    }

    void
    AdvancedIKSolver::collisionDetection(SceneObjectSetPtr avoidCollisionsWith)
    {
        cdm.reset();

        if (avoidCollisionsWith)
        {
            cdm.reset(new CDManager(avoidCollisionsWith->getCollisionChecker()));
            cdm->addCollisionModel(avoidCollisionsWith);
            cdm->addCollisionModel(rns);
        }
    }

    void
    AdvancedIKSolver::collisionDetection(CDManagerPtr avoidCollisions)
    {
        cdm = avoidCollisions;
    }

    std::vector<float>
    AdvancedIKSolver::solveNoRNSUpdate(const Eigen::Matrix4f& globalPose,
                                       CartesianSelection selection)
    {
        std::vector<float> result;
        std::vector<float> v;
        rns->getJointValues(v);

        if (solve(globalPose, selection))
        {
            rns->getJointValues(result);
        }

        RobotPtr rob = rns->getRobot();
        rob->setJointValues(rns, v);
        return result;
    }

    bool
    AdvancedIKSolver::solve(const Eigen::Vector3f& globalPosition)
    {
        Eigen::Matrix4f t;
        t.setIdentity();
        t.block(0, 3, 3, 1) = globalPosition;
        return solve(t, Position);
    }

    GraspPtr
    AdvancedIKSolver::solve(ManipulationObjectPtr object,
                            CartesianSelection selection /*= All*/,
                            int maxLoops)
    {
        THROW_VR_EXCEPTION_IF(!object, "NULL object");
        // first get a compatible EEF
        RobotPtr robot = rns->getRobot();
        THROW_VR_EXCEPTION_IF(!robot, "NULL robot");
        std::vector<EndEffectorPtr> eefs;
        robot->getEndEffectors(eefs);
        EndEffectorPtr eef;

        for (auto& i : eefs)
        {
            if (i->getTcp() == rns->getTCP())
            {
                if (eef)
                {
                    VR_ERROR << " Two end effectors with tcp " << rns->getTCP()->getName()
                             << " defined in robot " << robot->getName()
                             << ". taking the first one?!" << std::endl;
                }
                else
                {
                    eef = i;
                }
            }
        }

        if (!eef)
        {
            VR_ERROR << " No end effector with tcp " << rns->getTCP()->getName()
                     << " defined in robot " << robot->getName() << ". Aborting..." << std::endl;
            return GraspPtr();
        }

        GraspSetPtr gs = object->getGraspSet(eef)->clone();

        if (!gs || gs->getSize() == 0)
        {
            VR_ERROR << " No grasps defined for eef " << eef->getName() << " defined in object "
                     << object->getName() << ". Aborting..." << std::endl;
            return GraspPtr();
        }

        bool updateStatus = robot->getUpdateVisualizationStatus();
        robot->setUpdateVisualization(false);

        // check all grasps if there is an IK solution
        while (gs->getSize() > 0)
        {
            GraspPtr g = sampleSolution(object, gs, selection, true, maxLoops);

            if (g)
            {
                robot->setUpdateVisualization(updateStatus);
                robot->applyJointValues();
                return g;
            }
        }

        robot->setUpdateVisualization(updateStatus);

        // when here, no grasp was successful
        return GraspPtr();
    }

    bool
    AdvancedIKSolver::solve(ManipulationObjectPtr object,
                            GraspPtr grasp,
                            CartesianSelection selection /*= All*/,
                            int maxLoops)
    {
        THROW_VR_EXCEPTION_IF(!object, "NULL object");
        THROW_VR_EXCEPTION_IF(!grasp, "NULL grasp");
        RobotPtr robot = rns->getRobot();
        THROW_VR_EXCEPTION_IF(!robot, "NULL robot");
        bool updateStatus = robot->getUpdateVisualizationStatus();
        robot->setUpdateVisualization(false);

        std::vector<float> v;
        rns->getJointValues(v);
        Eigen::Matrix4f m = grasp->getTcpPoseGlobal(object->getGlobalPose());

        if (_sampleSolution(m, selection, maxLoops))
        {
            robot->setUpdateVisualization(updateStatus);
            robot->applyJointValues();
            return true;
        }

        robot->setJointValues(rns, v);
        robot->setUpdateVisualization(updateStatus);
        return false;
    }

    GraspPtr
    AdvancedIKSolver::sampleSolution(ManipulationObjectPtr object,
                                     GraspSetPtr graspSet,
                                     CartesianSelection selection /*= All*/,
                                     bool removeGraspFromSet,
                                     int maxLoops)
    {
        THROW_VR_EXCEPTION_IF(!object, "NULL object");
        //THROW_VR_EXCEPTION_IF(!eef,"NULL eef");
        THROW_VR_EXCEPTION_IF(!graspSet, "NULL graspSet");

        if (graspSet->getSize() == 0)
        {
            return GraspPtr();
        }

        std::vector<float> v;
        rns->getJointValues(v);

        int pos = rand() % graspSet->getSize();
        GraspPtr g = graspSet->getGrasp(pos);
        Eigen::Matrix4f m = g->getTcpPoseGlobal(object->getGlobalPose());

        if (_sampleSolution(m, selection, maxLoops))
        {
            return g;
        }

        // did not succeed, reset joint values and remove grasp from temporary set
        RobotPtr rob = rns->getRobot();
        rob->setJointValues(rns, v);

        if (removeGraspFromSet)
        {
            graspSet->removeGrasp(g);
        }

        return GraspPtr();
    }

    void
    AdvancedIKSolver::setMaximumError(float maxErrorPositionMM /*= 1.0f*/,
                                      float maxErrorOrientationRad /*= 0.02*/)
    {
        this->maxErrorPositionMM = maxErrorPositionMM;
        this->maxErrorOrientationRad = maxErrorOrientationRad;
    }

    void
    AdvancedIKSolver::setReachabilityCheck(ReachabilityPtr reachabilitySpace)
    {
        this->reachabilitySpace = reachabilitySpace;

        if (reachabilitySpace)
        {
            if (reachabilitySpace->getTCP() != tcp)
            {
                VR_ERROR << "Reachability representation has different tcp RobotNode ("
                         << reachabilitySpace->getTCP()->getName() << ") than IK solver ("
                         << tcp->getName() << ") ?! " << endl
                         << "Reachability results may not be valid!" << std::endl;
            }

            if (reachabilitySpace->getNodeSet() != rns)
            {
                VR_ERROR << "Reachability representation is defined for a different RobotNodeSet ("
                         << reachabilitySpace->getNodeSet()->getName() << ") than IK solver uses ("
                         << rns->getName() << ") ?! " << endl
                         << "Reachability results may not be valid!" << std::endl;
            }
        }
    }

    bool
    AdvancedIKSolver::checkReachable(const Eigen::Matrix4f& globalPose)
    {
        if (!reachabilitySpace)
        {
            return true;
        }

        return reachabilitySpace->isReachable(globalPose);
    }

    VirtualRobot::RobotNodePtr
    AdvancedIKSolver::getTcp()
    {
        return tcp;
    }

    VirtualRobot::RobotNodeSetPtr
    AdvancedIKSolver::getRobotNodeSet()
    {
        return rns;
    }

    void
    AdvancedIKSolver::setVerbose(bool enable)
    {
        verbose = enable;
    }


} // namespace VirtualRobot
