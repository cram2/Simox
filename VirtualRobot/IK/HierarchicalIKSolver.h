#pragma once

#include "HierarchicalIK.h"

using namespace VirtualRobot;

class HierarchicalIKSolver : public HierarchicalIK, public std::enable_shared_from_this<HierarchicalIKSolver>
{

public:
    HierarchicalIKSolver(RobotNodeSetPtr allRobotNodes);
    bool solveIK(float stepSize = 0.2, float minChange = 0.0, int maxSteps = 50);
    bool computeSteps(float stepSize, float minChange, int maxSteps);
    void addIK(JacobiProviderPtr jacProvider);

    bool checkTolerances();
    void clearIKs();
protected:

    std::vector<JacobiProviderPtr> jacobies;
};

typedef std::shared_ptr<HierarchicalIKSolver> HierarchicalIKSolverPtr;
