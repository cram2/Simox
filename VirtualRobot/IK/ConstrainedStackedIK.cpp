#include "ConstrainedStackedIK.h"

using namespace VirtualRobot;

ConstrainedStackedIK::ConstrainedStackedIK(RobotPtr &robot, const RobotNodeSetPtr &nodeSet, JacobiProvider::InverseJacobiMethod method) :
    ConstrainedIK(robot),
    nodeSet(nodeSet),
    method(method)
{
}

bool ConstrainedStackedIK::initialize()
{
    ik.reset(new StackedIK(nodeSet, method));
    jacobians.clear();

    for(auto &constraint : constraints)
    {
        jacobians.push_back(constraint);
    }

    return ConstrainedIK::initialize();
}

bool ConstrainedStackedIK::solveStep()
{
    Eigen::VectorXf jointValues;
    Eigen::VectorXf delta = ik->computeStep(jacobians);

    nodeSet->getJointValues(jointValues);
    nodeSet->setJointValues(jointValues + delta);

    return true;
}
