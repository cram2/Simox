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
* @author     Simon Ottenhaus (simon dot ottenhaus at kit dot edu)
* @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
*             GNU General Public License
*/

#include "CompositeDiffIK.h"
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/math/Helpers.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <cfloat>
#include <SimoxUtility/error/SimoxError.h>

namespace VirtualRobot {

CompositeDiffIK::CompositeDiffIK(const RobotNodeSetPtr& rns)
    : rns(rns), robot(rns->getRobot())
{
    ik.reset(new DifferentialIK(rns, rns->getRobot()->getRootNode(), JacobiProvider::eSVDDampedDynamic));
}

void CompositeDiffIK::addTarget(const TargetPtr& target)
{
    targets.emplace_back(target);
}

CompositeDiffIK::TargetPtr CompositeDiffIK::addTarget(const RobotNodePtr& tcp, const Eigen::Matrix4f& target, IKSolver::CartesianSelection mode)
{
    TargetPtr t = std::make_shared<Target>(tcp, target, mode);
    addTarget(t);
    return t;
}

void CompositeDiffIK::addNullspaceGradient(const CompositeDiffIK::NullspaceGradientPtr& gradient)
{
    nullspaceGradients.emplace_back(gradient);
    gradient->setMapping(rns);
}

CompositeDiffIK::NullspaceTargetPtr CompositeDiffIK::addNullspacePositionTarget(const RobotNodePtr& tcp, const Eigen::Vector3f& target)
{
    CompositeDiffIK::NullspaceTargetPtr nst(new CompositeDiffIK::NullspaceTarget(rns, tcp, target));
    addNullspaceGradient(nst);
    return nst;
}


CompositeDiffIK::Result CompositeDiffIK::solve(Parameters params) {
    SolveState s;
    return solve(params, s);
}

CompositeDiffIK::Result CompositeDiffIK::solve(Parameters params, SolveState &s)
{
    if (params.resetRnsValues)
    {
        for (RobotNodePtr rn : rns->getAllRobotNodes())
        {
            if (rn->isLimitless())
            {
                rn->setJointValue(0);
            }
            else
            {
                rn->setJointValue((rn->getJointLimitHi() + rn->getJointLimitLo()) / 2);
            }
        }
    }

    s.rows = 0;
    s.cols = rns->getSize();

    for (const TargetPtr& target : targets)
    {
        s.rows += CartesianSelectionToSize(target->mode);
    }

    s.jointRegularization = Eigen::VectorXf::Zero(s.cols);
    for (size_t i = 0; i < rns->getSize(); i++)
    {
        float regularization = 1;
        if (rns->getNode(i)->isTranslationalJoint())
        {
            regularization = params.jointRegularizationTranslation;
        }
        else if (rns->getNode(i)->isRotationalJoint())
        {
            regularization = params.jointRegularizationRotation;
        }
        else if (rns->getNode(i)->isHemisphereJoint())
        {
            // FIXME: ToDo?
            regularization = params.jointRegularizationRotation;
        }
        else if (rns->getNode(i)->isFourBarJoint())
        {
            // FIXME: ToDo?
            regularization = params.jointRegularizationRotation;
        }

        s.jointRegularization(i) = regularization;
    }

    s.cartesianRegularization = Eigen::VectorXf::Zero(s.rows);
    {
        CartesianVelocityController tmpVC(rns);
        int row = 0;
        for (const TargetPtr& target : targets)
        {
            int h = CartesianSelectionToSize(target->mode);
            target->jacobi = Eigen::MatrixXf::Zero(h, s.cols);
            s.cartesianRegularization.block(row, 0, h, 1) = tmpVC.calculateRegularization(target->mode);
            row += h;
        }
    }

    s.jointValues = rns->getJointValuesEigen();

    for (size_t i = 0; i < params.steps; i++)
    {
        step(params, s, i);
    }

    return getLastResult();
}

CompositeDiffIK::Result CompositeDiffIK::getLastResult() {
    bool allReached = true;
    for (const TargetPtr& target : targets)
    {
        allReached = allReached && target->isReached();
    }

    Result result;
    result.jointValues = rns->getJointValuesEigen();
    result.reached = allReached;
    result.jointLimitMargins = Eigen::VectorXf::Zero(rns->getSize());
    result.minimumJointLimitMargin = FLT_MAX;
    for (size_t i = 0; i < rns->getSize(); i++)
    {
        RobotNodePtr rn = rns->getNode(i);
        if (rn->isLimitless())
        {
            result.jointLimitMargins(i) = M_PI;
        }
        else
        {
            result.jointLimitMargins(i) = std::min(rn->getJointValue() - rn->getJointLimitLo(), rn->getJointLimitHi() - rn->getJointValue());
            result.minimumJointLimitMargin = std::min(result.minimumJointLimitMargin, result.jointLimitMargins(i));
        }
    }

    return result;
}

Eigen::MatrixXf CompositeDiffIK::CalculateNullspaceSVD(const Eigen::Matrix4f& jacobi)
{
    Eigen::FullPivLU<Eigen::MatrixXf> lu_decomp(jacobi);
    Eigen::MatrixXf nullspaceLU = lu_decomp.kernel();
    return nullspaceLU;
}

Eigen::MatrixXf CompositeDiffIK::CalculateNullspaceLU(const Eigen::Matrix4f& jacobi)
{
    // cols >= rows
    int rows = jacobi.rows();
    int cols = jacobi.cols();
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(jacobi, Eigen::ComputeFullU | Eigen::ComputeFullV);
    ///
    /// \brief V contains right singular vector and nullspace:
    /// V.shape: (cols,cols)
    /// singular vectors: V[:,0:rows]
    /// nullspace: V[:,rows:cols-rows]
    ///
    Eigen::MatrixXf V = svd.matrixV();
    Eigen::MatrixXf nullspaceSVD = V.block(0, rows, cols, cols - rows);
    return nullspaceSVD;
}

void CompositeDiffIK::step(CompositeDiffIK::Parameters& params, SolveState& s, int stepNr)
{
    s.jacobi = Eigen::MatrixXf::Zero(s.rows, s.cols);
    s.invJac = Eigen::MatrixXf::Zero(s.cols, s.rows);

    Eigen::VectorXf cartesianVel = Eigen::VectorXf::Zero(s.rows);
    {
        int row = 0;
        for (const TargetPtr& target : targets)
        {
            ik->updateJacobianMatrix(target->jacobi, target->tcp, target->mode);
            int h = CartesianSelectionToSize(target->mode);
            s.jacobi.block(row, 0, h, s.cols) = target->jacobi;
            Eigen::Matrix4f targetPose = target->target;
            Eigen::VectorXf cv = target->pCtrl.calculate(targetPose, target->mode);
            cartesianVel.block(row, 0, h, 1) = cv;
            row += h;
            if (params.returnIKSteps)
            {
                TargetStep step;
                step.tcpPose = target->tcp->getPoseInRootFrame();
                step.cartesianVel = cv;
                step.posDiff = target->pCtrl.getPositionDiff(targetPose);
                step.oriDiff = target->pCtrl.getOrientationDiff(targetPose);
                target->ikSteps.emplace_back(step);
            }
        }
    }

    for (int row = 0; row < s.rows; row++)
    {
        s.jacobi.block(row, 0, 1, s.cols) = s.jacobi.block(row, 0, 1, s.cols).cwiseProduct(s.jointRegularization.transpose());
    }

    ik->updatePseudoInverseJacobianMatrix(s.invJac, s.jacobi, 0, s.cartesianRegularization);

    Eigen::VectorXf jv = s.invJac * cartesianVel;

    if (s.cols > s.rows)
    {
        Eigen::VectorXf nullspaceVel = Eigen::VectorXf::Zero(s.cols);

        for (const NullspaceGradientPtr& nsGradient : nullspaceGradients)
        {
            Eigen::VectorXf nsgrad = nsGradient->kP * nsGradient->getGradientAdjusted(params, stepNr);
            nullspaceVel += nsgrad;
        }

        //LimitInfNormTo(nullspaceVel, params.maxJointAngleStep);

        Eigen::JacobiSVD<Eigen::MatrixXf> svd(s.jacobi, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::MatrixXf V = svd.matrixV();

        Eigen::MatrixXf nullspaceSVD = V.block(0, s.rows, s.cols, s.cols - s.rows);

        s.nullspace = nullspaceSVD; // CalculateNullspaceSVD(s.jacobi);

        Eigen::VectorXf nsv = Eigen::VectorXf::Zero(s.cols);
        for (int i = 0; i < s.nullspace.cols(); i++)
        {
            float squaredNorm = s.nullspace.col(i).squaredNorm();
            // Prevent division by zero
            if (squaredNorm > 1.0e-32f)
            {
                nsv += s.nullspace.col(i) * s.nullspace.col(i).dot(nullspaceVel) / s.nullspace.col(i).squaredNorm();
            }
        }
        jv = jv + nsv;
    }

    jv = jv * params.stepSize;
    jv = LimitInfNormTo(jv, params.maxJointAngleStep, params.maxJointAngleStepIgnore);
    jv = jv.cwiseProduct(s.jointRegularization);

    Eigen::VectorXf newJointValues = s.jointValues + jv;

    rns->setJointValues(newJointValues);
    s.jointValues = newJointValues;
}

int CompositeDiffIK::CartesianSelectionToSize(IKSolver::CartesianSelection mode)
{
    switch (mode)
    {
        case IKSolver::CartesianSelection::Position:
            return 3;
        case IKSolver::CartesianSelection::Orientation:
            return 3;
        case IKSolver::CartesianSelection::All:
            return 6;
        default:
            throw std::runtime_error("mode not supported");
    }
}

RobotNodeSetPtr CompositeDiffIK::getRobotNodeSet() {
    return rns;
}

RobotPtr CompositeDiffIK::getRobot() {
    return robot;
}

CompositeDiffIK::Target::Target(const RobotNodePtr& tcp, const Eigen::Matrix4f& target, IKSolver::CartesianSelection mode)
    : tcp(tcp), target(target), mode(mode), pCtrl(tcp)
{
    jacobi = Eigen::MatrixXf::Zero(0, 0);
}

CompositeDiffIK::NullspaceTarget::NullspaceTarget(const RobotNodeSetPtr& rns, const RobotNodePtr& tcp, const Eigen::Matrix4f& target, IKSolver::CartesianSelection mode)
    : NullspaceGradient(rns->getNodeNames()), tcp(tcp), target(target), mode(mode), pCtrl(tcp), vCtrl(rns, tcp)
{
}

CompositeDiffIK::NullspaceTarget::NullspaceTarget(const RobotNodeSetPtr& rns, const RobotNodePtr& tcp, const Eigen::Vector3f& target)
    : CompositeDiffIK::NullspaceTarget(rns, tcp, math::Helpers::CreatePose(target, Eigen::Matrix3f::Identity()), IKSolver::CartesianSelection::Position)
{
}

void CompositeDiffIK::NullspaceTarget::init(Parameters&)
{
    ikSteps.clear();
}

Eigen::VectorXf CompositeDiffIK::NullspaceTarget::getGradient(Parameters& params, int /*stepNr*/)
{
    Eigen::VectorXf cv = pCtrl.calculate(target, mode);
    Eigen::VectorXf jv = vCtrl.calculate(cv, mode);
    if (params.returnIKSteps)
    {
        NullspaceTargetStep step;
        step.tcpPose = tcp->getPoseInRootFrame();
        step.posDiff = pCtrl.getPositionDiff(target);
        step.oriDiff = pCtrl.getOrientationDiff(target);
        step.cartesianVel = cv;
        step.jointVel = jv;
        ikSteps.emplace_back(step);
    }
    return jv;
}

CompositeDiffIK::AdaptableNullspaceTarget::AdaptableNullspaceTarget(const RobotNodeSetPtr& rns, const RobotNodePtr& tcp, const Eigen::Matrix4f& target, IKSolver::CartesianSelection mode, int startStepNr, int endStepNr) :
    NullspaceTarget(rns, tcp, target, mode),
    startStepNr(startStepNr),
    endStepNr(endStepNr)
{
    assert(startStepNr < endStepNr);
}

CompositeDiffIK::AdaptableNullspaceTarget::AdaptableNullspaceTarget(const RobotNodeSetPtr& rns, const RobotNodePtr& tcp, const Eigen::Vector3f& target, int startStepNr, int endStepNr) :
    NullspaceTarget(rns, tcp, target),
    startStepNr(startStepNr),
    endStepNr(endStepNr)
{
    assert(startStepNr < endStepNr);
}

Eigen::VectorXf CompositeDiffIK::AdaptableNullspaceTarget::getGradient(Parameters& params, int stepNr) {
    Eigen::VectorXf gradient = NullspaceTarget::getGradient(params, stepNr);
    float factor = 0;
    if ((int)params.steps < (endStepNr - startStepNr) || stepNr >= (int)params.steps - startStepNr)
        factor = 1;
    else if (stepNr >= startStepNr) {
        factor = ((float)(stepNr - startStepNr)) / ((int)params.steps - (endStepNr - startStepNr));
    }
    return gradient * factor;
}

Eigen::VectorXf CompositeDiffIK::LimitInfNormTo(Eigen::VectorXf vec, float maxValue, const std::set<std::string> &ignore)
{
    Eigen::VectorXf newVec(vec.rows() - ignore.size());
    int j = 0;
    for (size_t i = 0; i < rns->getSize(); i++)
    {
        auto node = rns->getNode(i);
        if (ignore.find(node->getName()) == ignore.end()) {
            newVec(j) = vec(i);
            j++;
        }
    }

    float infNorm = newVec.lpNorm<Eigen::Infinity>();
    if (infNorm > maxValue)
    {
        for (int i = 0; i < vec.rows(); i++)
            vec(i) = vec(i) / infNorm * maxValue;
    }
    return vec;
}


Eigen::VectorXf CompositeDiffIK::LimitInfNormTo(Eigen::VectorXf vec, float maxValue)
{
    float infNorm = vec.lpNorm<Eigen::Infinity>();
    if (infNorm > maxValue)
    {
        vec = vec / infNorm * maxValue;
    }
    return vec;
}

CompositeDiffIK::NullspaceJointTarget::NullspaceJointTarget(const RobotNodeSetPtr& rns)
    : NullspaceGradient(rns->getNodeNames()), rns(rns), target(rns->getJointValuesEigen()), weight(Eigen::VectorXf::Zero(rns->getSize()))
{

}

CompositeDiffIK::NullspaceJointTarget::NullspaceJointTarget(const RobotNodeSetPtr& rns, const Eigen::VectorXf& target, const Eigen::VectorXf& weight)
    : NullspaceGradient(rns->getNodeNames()), rns(rns), target(target), weight(weight)
{

}

void CompositeDiffIK::NullspaceJointTarget::set(int index, float target, float weight)
{
    this->target(index) = target;
    this->weight(index) = weight;
}

void CompositeDiffIK::NullspaceJointTarget::set(const std::string& jointName, float target, float weight)
{
    int index = rns->getRobotNodeIndex(jointName);
    if (index < 0)
    {
        throw std::runtime_error("RobotNodeSet has no joint " + jointName);
    }
    set(index, target, weight);
}

void CompositeDiffIK::NullspaceJointTarget::set(const RobotNodePtr& rn, float target, float weight)
{
    int index = rns->getRobotNodeIndex(rn);
    if (index < 0)
    {
        throw std::runtime_error("RobotNodeSet has no joint " + rn->getName());
    }
    set(index, target, weight);
}

void CompositeDiffIK::NullspaceJointTarget::init(CompositeDiffIK::Parameters&)
{

}

Eigen::VectorXf CompositeDiffIK::NullspaceJointTarget::getGradient(Parameters& /*params*/, int /*stepNr*/)
{
    return (target - rns->getJointValuesEigen()).cwiseProduct(weight);
}

CompositeDiffIK::NullspaceJointLimitAvoidance::NullspaceJointLimitAvoidance(const RobotNodeSetPtr& rns)
    : NullspaceGradient(rns->getNodeNames()), rns(rns), weight(Eigen::VectorXf::Constant(rns->getSize(), 1))
{

}

CompositeDiffIK::NullspaceJointLimitAvoidance::NullspaceJointLimitAvoidance(const RobotNodeSetPtr& rns, const Eigen::VectorXf& weight)
    : NullspaceGradient(rns->getNodeNames()), rns(rns), weight(weight)
{

}

void CompositeDiffIK::NullspaceJointLimitAvoidance::setWeight(int index, float weight)
{
    this->weight(index) = weight;
}

void CompositeDiffIK::NullspaceJointLimitAvoidance::setWeight(const std::string& jointName, float weight)
{
    int index = rns->getRobotNodeIndex(jointName);
    if (index < 0)
    {
        throw std::runtime_error("RobotNodeSet has no joint " + jointName);
    }
    setWeight(index, weight);
}

void CompositeDiffIK::NullspaceJointLimitAvoidance::setWeight(const RobotNodePtr& rn, float weight)
{
    int index = rns->getRobotNodeIndex(rn);
    if (index < 0)
    {
        throw std::runtime_error("RobotNodeSet has no joint " + rn->getName());
    }
    setWeight(index, weight);
}

void CompositeDiffIK::NullspaceJointLimitAvoidance::setWeights(const RobotNodeSetPtr& rns, float weight)
{
    for (const RobotNodePtr& rn : rns->getAllRobotNodes())
    {
        setWeight(rn, weight);
    }
}

void CompositeDiffIK::NullspaceJointLimitAvoidance::init(CompositeDiffIK::Parameters&)
{
    // Do nothing
}

Eigen::VectorXf CompositeDiffIK::NullspaceJointLimitAvoidance::getGradient(Parameters& /*params*/, int /*stepNr*/)
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
        }
    }
    return r.cwiseProduct(weight);
}

CompositeDiffIK::NullspaceGradient::NullspaceGradient(const std::vector<std::string> &jointNames) : jointNames(jointNames)
{
}

Eigen::VectorXf CompositeDiffIK::NullspaceGradient::getGradientAdjusted(CompositeDiffIK::Parameters &params, int stepNr) {
    Eigen::VectorXf gradient = getGradient(params, stepNr);
    if (mapping.empty()) return gradient;
    else {
        Eigen::VectorXf adjustedGradient = Eigen::VectorXf::Zero(ikSize);
        for (const auto &m : mapping) {
            adjustedGradient(m.second) = gradient(m.first);
        }
        return adjustedGradient;
    }
}

void CompositeDiffIK::NullspaceGradient::setMapping(const RobotNodeSetPtr &rns) {
    ikSize = rns->getSize();
    mapping = getMapping(rns);
}

std::map<int, int> CompositeDiffIK::NullspaceGradient::getMapping(const RobotNodeSetPtr &rns) {
    std::map<int, int> mapping;
    if (this->jointNames != rns->getNodeNames()) {
        for (unsigned int i = 0; i < jointNames.size(); i++) {
            int index = rns->getRobotNodeIndex(jointNames.at(i));
            if (index >= 0) {
                mapping[i] = index;
            }
            else std::cout << "Ignoring null space node " << jointNames.at(i) << " which is not part of ik" << std::endl;
        }
    }
    return mapping;
}

std::vector<std::string> CompositeDiffIK::NullspaceGradient::getJointNames() const {
    return jointNames;
}

}
