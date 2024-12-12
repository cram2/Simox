/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @author      ()
* @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
*             GNU General Public License
*/

#include "CartesianVelocityController.h"

#include "RobotNodeSet.h"
#include "VirtualRobot/Robot.h"
#include "VirtualRobot/math/Helpers.h"
#include "VirtualRobot/IK/DifferentialIK.h"
#include "VirtualRobot/Nodes/RobotNode.h"

namespace VirtualRobot
{

CartesianVelocityController::CartesianVelocityController(const RobotNodeSetPtr& rns, const RobotNodePtr& tcp, const JacobiProvider::InverseJacobiMethod invJacMethod, bool considerJointLimits)
    : rns(rns),
      _considerJointLimits(considerJointLimits)
{
    ik.reset(new DifferentialIK(rns, rns->getRobot()->getRootNode(), invJacMethod));
    _tcp = tcp ? tcp : rns->getTCP();

    _cartesianMMRegularization = 100;
    _cartesianRadianRegularization = 1;
    _jointCosts = Eigen::VectorXf::Constant(rns->getSize(), 1);
}

void CartesianVelocityController::calculateJacobis(IKSolver::CartesianSelection mode)
{
    jacobi = ik->getJacobianMatrix(_tcp, mode);
    _jacobiWithCosts = Eigen::MatrixXf(jacobi.rows(), jacobi.cols());
    for (int r = 0; r < jacobi.rows(); r++)
    {
        for (int c = 0; c < jacobi.cols(); c++)
        {
            _jacobiWithCosts(r, c) = jacobi(r, c) / _jointCosts(c);
        }
    }
    _inv = ik->computePseudoInverseJacobianMatrix(_jacobiWithCosts, ik->getJacobiRegularization(mode));
}

Eigen::VectorXf CartesianVelocityController::calculate(const Eigen::VectorXf& cartesianVel, IKSolver::CartesianSelection mode)
{
    return calculate(cartesianVel, Eigen::VectorXf::Zero(0), mode);
    /*calculateJacobis(mode);

    if (_considerJointLimits)
    {
        clampJacobiAtJointLimits(mode, cartesianVel, _jacobiWithCosts, _inv);
    }

    Eigen::VectorXf jointVel = _inv * cartesianVel;
    jointVel += nsv;
    for (int r = 0; r < jointVel.rows(); r++)
    {
        jointVel(r) = jointVel(r) / _jointCosts(r);
    }

    if (maximumJointVelocities.rows() > 0)
    {
        jointVel = ::math::Helpers::LimitVectorLength(jointVel, maximumJointVelocities);
    }

    return jointVel;*/
}

Eigen::VectorXf CartesianVelocityController::calculate(const Eigen::VectorXf& cartesianVel, float KpJointLimitAvoidanceScale, IKSolver::CartesianSelection mode)
{
    Eigen::VectorXf nullspaceVel = calculateNullspaceVelocity(cartesianVel, KpJointLimitAvoidanceScale, mode);
    return calculate(cartesianVel, nullspaceVel, mode);
}

Eigen::VectorXf CartesianVelocityController::calculate(const Eigen::VectorXf& cartesianVel, const Eigen::VectorXf& nullspaceVel, IKSolver::CartesianSelection mode)
{

    calculateJacobis(mode);


    Eigen::VectorXf nsv = Eigen::VectorXf::Zero(rns->getSize());

    if (nullspaceVel.rows() > 0)
    {
        Eigen::FullPivLU<Eigen::MatrixXf> lu_decomp(_jacobiWithCosts);
        Eigen::MatrixXf nullspace = lu_decomp.kernel();

        for (int i = 0; i < nullspace.cols(); i++)
        {
            float squaredNorm = nullspace.col(i).squaredNorm();
            // Prevent division by zero
            if (squaredNorm > 1.0e-32f)
            {
                nsv += nullspace.col(i) * nullspace.col(i).dot(nullspaceVel) / nullspace.col(i).squaredNorm();
            }
        }
    }


    if (_considerJointLimits)
    {
        clampJacobiAtJointLimits(mode, cartesianVel, _jacobiWithCosts, _inv);
    }


    Eigen::VectorXf jointVel = _inv * cartesianVel;
    jointVel += nsv;
    for (int r = 0; r < jointVel.rows(); r++)
    {
        jointVel(r) = jointVel(r) / _jointCosts(r);
    }

    if (maximumJointVelocities.rows() > 0)
    {
        jointVel = math::Helpers::LimitVectorLength(jointVel, maximumJointVelocities);
    }

    return jointVel;
}

Eigen::VectorXf CartesianVelocityController::calculateJointLimitAvoidance()
{
    Eigen::VectorXf r(rns->getSize());
    for (size_t i = 0; i < rns->getSize(); i++)
    {
        RobotNodePtr rn = rns->getNode(i);
        if (rn->isLimitless())
        {
            r(i) = 0;
        }
        else
        {
            float f = math::Helpers::ILerp(rn->getJointLimitLo(), rn->getJointLimitHi(), rn->getJointValue());
            r(i) = cos(f * M_PI);
            //r(i) = math::MathUtils::Lerp(1.f, -1.f, f);
        }
    }
    return r;
}

/**
 * @brief CartesianVelocityController::calculateJointLimitAvoidanceWithMargins
 * @param margins Vector with same size as rns. Values between 0 (no margin) and 1 (50% low and 50% high margin).
 * @return
 */
Eigen::VectorXf CartesianVelocityController::calculateJointLimitAvoidanceWithMargins(const Eigen::VectorXf& margins)
{
    Eigen::VectorXf r(rns->getSize());
    for (size_t i = 0; i < rns->getSize(); i++)
    {
        RobotNodePtr rn = rns->getNode(i);
        if (rn->isLimitless() || margins(i) <= 0)
        {
            r(i) = 0;
        }
        else
        {
            float lo = rn->getJointLimitLo();
            float hi = rn->getJointLimitHi();
            float range = hi - lo;
            float mrg = math::Helpers::Clamp(0.f, 1.f, margins(i) * range / 2);
            r(i) = math::Helpers::Clamp(0.f, 1.f, math::Helpers::ILerp(lo + mrg, lo, rn->getJointValue()))
                   + math::Helpers::Clamp(0.f, 1.f, math::Helpers::ILerp(hi, hi - mrg, rn->getJointValue()));
        }
    }
    return r;
}

Eigen::VectorXf CartesianVelocityController::calculateNullspaceVelocity(const Eigen::VectorXf& cartesianVel, float KpScale, IKSolver::CartesianSelection mode)
{
    Eigen::VectorXf regularization = calculateRegularization(mode);
    // Eigen does not allow product between two column vectors (this fails in Debug mode)
    // In Release this causes cartesianVel to be only multiplied by the first element of regularization
    // Eigen::VectorXf vel = cartesianVel * regularization;
    Eigen::VectorXf vel = cartesianVel.cwiseProduct(regularization);

    return KpScale * vel.norm() * calculateJointLimitAvoidance();

}

void CartesianVelocityController::setCartesianRegularization(float cartesianMMRegularization, float cartesianRadianRegularization)
{
    _cartesianMMRegularization = cartesianMMRegularization;
    _cartesianRadianRegularization = cartesianRadianRegularization;
}

Eigen::VectorXf CartesianVelocityController::calculateRegularization(IKSolver::CartesianSelection mode)
{
    Eigen::VectorXf regularization(6);

    int i = 0;

    if (mode & IKSolver::X)
    {
        regularization(i++) = 1 / _cartesianMMRegularization;
    }

    if (mode & IKSolver::Y)
    {
        regularization(i++) = 1 / _cartesianMMRegularization;
    }

    if (mode & IKSolver::Z)
    {
        regularization(i++) = 1 / _cartesianMMRegularization;
    }

    if (mode & IKSolver::Orientation)
    {
        regularization(i++) = 1 / _cartesianRadianRegularization;
        regularization(i++) = 1 / _cartesianRadianRegularization;
        regularization(i++) = 1 / _cartesianRadianRegularization;
    }
    return regularization.topRows(i);
}

bool CartesianVelocityController::clampJacobiAtJointLimits(IKSolver::CartesianSelection mode, const Eigen::VectorXf& cartesianVel, Eigen::MatrixXf& jacobi, Eigen::MatrixXf& inv, float jointLimitCheckAccuracy)
{
    bool modifiedJacobi = false;

    Eigen::VectorXf jointVel = inv * cartesianVel;
    size_t size = rns->getSize();

    for (size_t i = 0; i < size; ++i)
    {
        auto& node = rns->getNode(static_cast<int>(i));

        if (node->isLimitless() || // limitless joint cannot be out of limits
            std::abs(jointVel(i)) < 0.001f // If it the jacobi doesnt want this joint to move anyway, there is no point in recalculating the inverse jacobi
           )
        {
            continue;
        }

        if ((node->getJointValue() >= node->getJointLimitHigh() - jointLimitCheckAccuracy && jointVel(i) > 0)
            || (node->getJointValue() <= node->getJointLimitLow() + jointLimitCheckAccuracy && jointVel(i) < 0))
        {
            for (int k = 0; k < jacobi.rows(); ++k) // memory allocation free resetting of column
            {
                jacobi(k, i) = 0.0f;
            }
            modifiedJacobi = true;
        }
    }
    if (modifiedJacobi)
    {
        inv = ik->computePseudoInverseJacobianMatrix(jacobi, ik->getJacobiRegularization(mode));
    }


    return modifiedJacobi;
}

bool CartesianVelocityController::getConsiderJointLimits() const
{
    return _considerJointLimits;
}

void CartesianVelocityController::setConsiderJointLimits(bool value)
{
    _considerJointLimits = value;
}

void CartesianVelocityController::setJointCosts(const std::vector<float>& jointCosts)
{
    VR_ASSERT((int)jointCosts.size() == _jointCosts.rows());
    for (size_t i = 0; i < jointCosts.size(); i++)
    {
        _jointCosts(i) = jointCosts.at(i);
    }
}

}
