#include "Constraint.h"

using namespace VirtualRobot;

Constraint::Constraint(const RobotNodeSetPtr& nodeSet) :
    JacobiProvider(nodeSet, JacobiProvider::eSVDDamped),
    lastError(-1),
    lastLastError(-1),
    optimizationFunctionFactor(1)
{
    name ="Constraint";
    initialized = true;
}

void Constraint::initialize()
{
    lastError = -1;
    lastLastError = -1;
    optimizationFunctionFactor = 1;
}

bool Constraint::getRobotPoseForConstraint(Eigen::Matrix4f& /*pose*/)
{
    // No change in global pose required
    return false;
}

float Constraint::getErrorDifference()
{
    Eigen::VectorXf e = getError(1);
    lastLastError = lastError;
    lastError = e.norm();

    if (lastLastError <= 0)
    {
        return 0;
    }

    return lastLastError - lastError;
}

const std::vector<OptimizationFunctionSetup> &Constraint::getEqualityConstraints()
{
    return equalityConstraints;
}

const std::vector<OptimizationFunctionSetup> &Constraint::getInequalityConstraints()
{
    return inequalityConstraints;
}

const std::vector<OptimizationFunctionSetup> &Constraint::getOptimizationFunctions()
{
    return optimizationFunctions;
}

void Constraint::setOptimizationFunctionFactor(float factor)
{
    optimizationFunctionFactor = factor;
}

float Constraint::getOptimizationFunctionFactor()
{
    return optimizationFunctionFactor;
}

double Constraint::optimizationFunction(unsigned int /*id*/)
{
    THROW_VR_EXCEPTION("Constraint does not support NLopt-based solvers.");
}

Eigen::VectorXf Constraint::optimizationGradient(unsigned int /*id*/)
{
    THROW_VR_EXCEPTION("Constraint does not support NLopt-based solvers.");
}

Eigen::MatrixXf Constraint::getJacobianMatrix()
{
    THROW_VR_EXCEPTION("Constraint does not support Jacobian-based solvers.");
}

Eigen::MatrixXf Constraint::getJacobianMatrix(SceneObjectPtr /*tcp*/)
{
    THROW_VR_EXCEPTION("Constraint does not support Jacobian-based solvers.");
}

Eigen::VectorXf Constraint::getError(float /*stepSize*/)
{
    THROW_VR_EXCEPTION("Constraint does not support Jacobian-based solvers.");
}

bool Constraint::checkTolerances()
{
    THROW_VR_EXCEPTION("Constraint does not support Jacobian-based solvers.");
}

bool Constraint::usingCollisionModel()
{
    return false;
}

void Constraint::addEqualityConstraint(unsigned int id, bool soft)
{
    OptimizationFunctionSetup setup;
    setup.id = id;
    setup.constraint = this;
    setup.soft = soft;
    equalityConstraints.push_back(setup);
}

void Constraint::addInequalityConstraint(unsigned int id, bool soft)
{
    OptimizationFunctionSetup setup;
    setup.id = id;
    setup.constraint = this;
    setup.soft = soft;
    inequalityConstraints.push_back(setup);
}

void Constraint::addOptimizationFunction(unsigned int id, bool soft)
{
    OptimizationFunctionSetup setup;
    setup.id = id;
    setup.constraint = this;
    setup.soft = soft;
    optimizationFunctions.push_back(setup);
}
