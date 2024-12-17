#include "RobotPoseDifferentialIK.h"

#include <algorithm>
#include <cfloat>

#include <Eigen/Geometry>
#include <Eigen/QR>

#include <VirtualRobot/Robot.h>

#include "../Nodes/RobotNode.h"
#include "../Robot.h"
#include "../RobotNodeSet.h"

namespace VirtualRobot
{

    RobotPoseDifferentialIK::RobotPoseDifferentialIK(
        RobotPtr robot,
        RobotNodeSetPtr _rns,
        RobotNodePtr _coordSystem,
        JacobiProvider::InverseJacobiMethod invJacMethod) :
        DifferentialIK(_rns, _coordSystem, invJacMethod)
    {
        this->robot = robot;
        checkImprovement = true;
        considerBoxConstraints = true;
        // saving joint limits
        _uLimits.resize(rns->getSize());
        _lLimits.resize(rns->getSize());
        for (unsigned int i = 0; i < rns->getSize(); i++)
        {
            RobotNodePtr n = rns->getNode(i);
            _lLimits[i] = n->getJointLimitLow();
            _uLimits[i] = n->getJointLimitHigh();
        }
    }

    Eigen::MatrixXf
    RobotPoseDifferentialIK::getJacobianMatrix()
    {
        if (nRows == 0)
            this->setNRows();
        size_t nDoF = nodes.size();

        nDoF += 6; // add 6 DoF for robot pose

        Eigen::MatrixXf Jacobian = Eigen::MatrixXf::Constant(nRows, nDoF, 0.0f);

        size_t index = 0;
        for (size_t i = 0; i < tcp_set.size(); i++)
        {
            SceneObjectPtr tcp = tcp_set[i];
            if (this->targets.find(tcp) != this->targets.end())
            {
                IKSolver::CartesianSelection mode = this->modes[tcp];
                Eigen::MatrixXf partJacobian = this->getJacobianMatrix(tcp, mode);
                this->localJacobians[i] = partJacobian;

                Jacobian.block(index, 0, partJacobian.rows(), nDoF) =
                    partJacobian.block(0, 0, partJacobian.rows(), nDoF);
                if (mode & IKSolver::X)
                    index++;
                if (mode & IKSolver::Y)
                    index++;
                if (mode & IKSolver::Z)
                    index++;
                if (mode & IKSolver::Orientation)
                    index += 3;
            }
            else
                VR_ERROR << "Internal error?!\n"; // Error
        }
        return Jacobian;
    }

    Eigen::MatrixXf
    RobotPoseDifferentialIK::getJacobianMatrix(SceneObjectPtr tcp,
                                               IKSolver::CartesianSelection mode)
    {
        // Get number of degrees of freedom
        size_t nDoF = nodes.size();

        // using robot position and rotation as dof
        nDoF += 6;

        // obtain the size of the matrix.
        unsigned int size = 0;
        if (mode & IKSolver::X)
            size++;
        if (mode & IKSolver::Y)
            size++;
        if (mode & IKSolver::Z)
            size++;
        if (mode & IKSolver::Orientation)
            size += 3;

        Eigen::MatrixXf jacJoints = DifferentialIK::getJacobianMatrix(tcp, mode);
        Eigen::MatrixXf jacRobot(size, 6);

        Eigen::MatrixXf positionRobot = Eigen::MatrixXf::Zero(3, 6);
        Eigen::MatrixXf orientationRobot = Eigen::MatrixXf::Zero(3, 6);
        // Add DoF related to robot pose
        for (size_t i = 0; i < 6; i++)
        {

            // robot pose
            if (i < 3)
            {
                // pos

                Eigen::Vector3f axis;
                if (i == 0)
                    axis = Eigen::Vector3f::UnitX();
                else if (i == 1)
                    axis = Eigen::Vector3f::UnitY();
                else if (i == 2)
                    axis = Eigen::Vector3f::UnitZ();
                else
                {
                    std::cout << "int. err. " << std::endl;
                    continue;
                }

                if (coordSystem)
                {
                    // convert Vector to local coord system
                    Eigen::Vector4f result4f = Eigen::Vector4f::Zero();
                    result4f.segment(0, 3) = axis;
                    result4f = coordSystem->getGlobalPose().inverse() * result4f;
                    axis = result4f.head(3);
                }
                // if necessary calculate the position part of the Jacobian
                if (mode & IKSolver::Position)
                    positionRobot.block(0, i, 3, 1) = axis;
                // no orientation part required with prismatic joints
            }
            else
            {
                // ori

                Eigen::Vector3f axis;
                if (i == 3)
                    axis = Eigen::Vector3f::UnitX();
                else if (i == 4)
                    axis = Eigen::Vector3f::UnitY();
                else if (i == 5)
                    axis = Eigen::Vector3f::UnitZ();
                else
                {
                    std::cout << "int. err. " << std::endl;
                    continue;
                }
                if (coordSystem)
                {
                    // convert Vector to local coord system
                    Eigen::Vector4f result4f = Eigen::Vector4f::Zero();
                    result4f.segment(0, 3) = axis;
                    result4f = coordSystem->getGlobalPose().inverse() * result4f;
                    axis = result4f.head(3);
                }
                // if necessary calculate the position part of the Jacobian
                if (mode & IKSolver::Position)
                {
                    Eigen::Vector3f toTCP;
                    if (coordSystem)
                    {
                        toTCP = coordSystem->toLocalCoordinateSystem(tcp->getGlobalPose())
                                    .block(0, 3, 3, 1) -
                                coordSystem->toLocalCoordinateSystem(robot->getGlobalPose())
                                    .block(0, 3, 3, 1);
                    }
                    else
                    {
                        toTCP = tcp->getGlobalPose().block(0, 3, 3, 1) -
                                robot->getGlobalPose().block(0, 3, 3, 1);
                    }
                    if (convertMMtoM)
                        toTCP /= 1000.0f;
                    /*cout << "toTCP: " << tcp->getName() << endl;
                    cout << axis << endl;
                    cout << toTCP << endl;*/
                    Eigen::Vector3f r = axis.cross(toTCP);
                    //cout << r << endl;
                    positionRobot.block(0, i, 3, 1) = r;
                    if (r.norm() > 1e10)
                    {
                        std::cout << "posRobot error" << std::endl;
                    }
                }
                // and the orientation part
                if (mode & IKSolver::Orientation)
                    orientationRobot.block(0, i, 3, 1) = axis;
            }
        }

        Eigen::MatrixXf result(size, nDoF);
        result.block(0, 0, size, nDoF - 6) = jacJoints;

        // copy only what is required (and was previously calculated)
        unsigned int index = 0;
        if (mode & IKSolver::X)
        {
            result.block(index, nDoF - 6, 1, 6) = positionRobot.row(0);
            //result.block(0, nDoF - 6 + index, size, 1) = positionRobot.col(0);
            index++;
        }
        if (mode & IKSolver::Y)
        {
            result.block(index, nDoF - 6, 1, 6) = positionRobot.row(1);
            index++;
        }
        if (mode & IKSolver::Z)
        {
            result.block(index, nDoF - 6, 1, 6) = positionRobot.row(2);
            index++;
        }
        if (mode & IKSolver::Orientation)
        {
            result.block(index, nDoF - 6, 3, 6) = orientationRobot;
        }
        return result;
    }

    Eigen::MatrixXd
    RobotPoseDifferentialIK::computePseudoInverseJacobianMatrixDampedD(const Eigen::MatrixXd& m)
    {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(m, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::MatrixXd U = svd.matrixU();
        Eigen::MatrixXd V = svd.matrixV();
        Eigen::VectorXd sv = svd.singularValues();

        //float lambda = 1.0f;

        double epsilon = std::numeric_limits<double>::epsilon();
        double tol = epsilon * std::max(m.rows(), m.cols()) * m.norm();
        //tol = 0.001;
        //MMM_INFO << "tol" <<  tol << endl;
        for (int i = 0; i < sv.rows(); i++)
        {
            if (sv(i) > tol)
                sv(i) = 1.0f / sv(i);
            //            sv(i) = sv(i) / (sv(i)*sv(i) + lambda*lambda);
            else
                sv(i) = 0.0f;
        }

        /*if (sv(i) > tol)
        sv(i) = 1.0f / sv(i);
        else sv(i) = 0;*/

        return (V * sv.asDiagonal() * U.transpose());
    }

    Eigen::VectorXf
    RobotPoseDifferentialIK::computeStep(float stepSize)
    {
        const bool considerMaxAngle = true;

        if (nRows == 0)
            this->setNRows();
        size_t nDoF = nodes.size();

        // consider robot pose as 6d vector
        nDoF += 6;
        //this->inverseMethod = DifferentialIK::eSVDDamped;
        bool jVerbose = verbose;
        jVerbose = false;

        // init
        bool bAbort = false;
        Eigen::VectorXf thetaOld(nDoF);
        Eigen::VectorXf theta(nDoF);
        Eigen::VectorXf blockedJointDeltas(nDoF);
        Eigen::VectorXf blockedJointValues(nDoF);
        bool bViolation;
        std::vector<int> blockedJoints;
        Eigen::VectorXf dTheta(nDoF);
        //Eigen::MatrixXf pseudo;
        float maximal = float(10 * M_PI / 180.0);
        float scale;
        Eigen::VectorXf error;
        Eigen::VectorXf angularDiff(nDoF - 3);
        Eigen::MatrixXf Jacobian;
        float maxAngularDifference; // , maxAngularDifference1, maxAngularDifference2;


        if (considerBoxConstraints)
        {

            // get joint positions
            rns->getJointValues(thetaOld);
            blockedJointValues = thetaOld;
            blockedJointDeltas.setZero();
            // main loop
            int counter = 0;
            while (!bAbort && counter < 30)
            {
                counter++;
                scale = 1.0;
                error = getError(stepSize);
                Jacobian = getJacobianMatrix();

                int rows = Jacobian.rows();
                for (int blockedJoint : blockedJoints)
                {
                    Jacobian.block(0, blockedJoint, rows, 1).setZero();
                }


                /*
                  MatrixXd JacobianXd = Jacobian.cast<double>();
                  MatrixXd test = Jacobian.transpose().cast<double>()*Jacobian.cast<double>();
                  FullPivLU<MatrixXd> lu_decomp(test);
                  cout << "The rank of A is " << lu_decomp.rank() << " (" << test.rows() << "x" << test.cols() << ")" << endl;
                  */

                Eigen::MatrixXf pseudoXf = computePseudoInverseJacobianMatrix(Jacobian);
                Eigen::VectorXf dThetaXf = pseudoXf * error;
                //            Eigen::VectorXf errorXf = Jacobian * dThetaXf;
                //            MMM_INFO << error.norm() << ", " << errorXf.norm() << endl;
                dTheta = dThetaXf;

                /*
                MatrixXd pseudoXd = computePseudoInverseJacobianMatrixDampedD(JacobianXd);
                VectorXd dThetaXd = pseudoXd * error.cast<double>();
                dTheta = dThetaXd.cast<float>();
                Eigen::VectorXd errorXd = JacobianXd * dThetaXd;
                */


                /* Debug Stuff */
                //            MMM_INFO << error.norm() << ", " << errorXd.norm() << endl;
                //MMM_INFO << dThetaXd.transpose() << endl << dThetaXf.transpose() << endl;


                //cout << "error:" << error << endl;
                //cout << "THETA:" << dTheta << endl;
                if (considerMaxAngle)
                {
                    scale = 1.0f;
                    angularDiff.block(0, 0, nDoF - 6, 1) = dTheta.head(nDoF - 6);
                    angularDiff.block(nDoF - 6, 0, 3, 1) = dTheta.tail(3);
                    //cout << "angularDiff:" << angularDiff << endl;
                    maxAngularDifference = angularDiff.array().abs().maxCoeff();
                    //cout << "maxAngularDifference:" << maxAngularDifference << endl;
                    //if (angularDiff.norm() > maximal) // why norm?!
                    if (maxAngularDifference > maximal)
                        //scale = maximal / angularDiff.norm(); // * stepSize;
                        scale = float(maximal / maxAngularDifference) * stepSize;
                    dTheta *= scale;
                }

                // calculate new joint positions after applying deltas
                theta = thetaOld + dTheta.block(0, 0, nDoF - 6, 1);
                // check for violation
                bViolation = false;
                for (unsigned int i = 0; i < nDoF - 6; i++)
                {
                    // lower limit violated?
                    if (theta[i] < _lLimits[i])
                    {
                        // save delta, and apply limit
                        blockedJointDeltas[i] = _lLimits[i] - thetaOld[i];
                        theta[i] = _lLimits[i];
                        blockedJointValues[i] = _lLimits[i];
                    }
                    // upper limit violated?
                    else if (theta[i] > _uLimits[i])
                    {
                        // save delta, and apply limit
                        blockedJointDeltas[i] = _uLimits[i] - thetaOld[i];
                        theta[i] = _uLimits[i];
                        blockedJointValues[i] = _uLimits[i];
                    }
                    else
                        // no violation? -> next
                        continue;
                    // one limit was violated, so mark this joint as fixed
                    bViolation = true;
                    blockedJoints.push_back(i);
                    // blocking one joint at a time
                    break;
                }
                // abort if no new joints were locked, rerun otherwise
                if (!bViolation)
                {
                    bAbort = true;
                }
                /*else {
                    // todo: check if necessary
                    rns->setJointValues(blockedJointValues);
                }*/
            }
            // restore original robot configuration
            //rns->setJointValues(thetaOld);
            // restore real delta theta
            if (jVerbose)
                if (blockedJoints.size() != 0)
                    std::cout << "These joints were blocked at their limits: ";
            for (int index : blockedJoints)
            {
                dTheta[index] = blockedJointDeltas[index];
                if (jVerbose)
                    std::cout << index << " " << rns->getNode(index)->getName() << "\t";
            }
            if (jVerbose)
                if (blockedJoints.size() != 0)
                    std::cout << std::endl;
            //cout << "counter:" << counter << endl;
            return dTheta;
        }
        else
        {
            // no box constraints
            error = getError(stepSize);
            if (jVerbose)
                std::cout << "Error Cartesian:" << error.transpose() << std::endl << std::endl;
            Jacobian = getJacobianMatrix();
            Eigen::MatrixXf pseudoXf = computePseudoInverseJacobianMatrix(Jacobian);
            if (jVerbose)
                std::cout << "PseudoInv min/max:" << std::endl
                          << pseudoXf.minCoeff() << "," << pseudoXf.maxCoeff() << std::endl
                          << std::endl;
            Eigen::VectorXf dThetaXf = pseudoXf * error;
            dTheta = dThetaXf;
            if (jVerbose)
                std::cout << "dTheta:" << dTheta.transpose() << std::endl << std::endl;

            if (considerMaxAngle)
            {
                scale = 1.0f;
                angularDiff.block(0, 0, nDoF - 6, 1) = dTheta.head(nDoF - 6);
                angularDiff.block(nDoF - 6, 0, 3, 1) = dTheta.tail(3);

                //cout << "angularDiff:" << angularDiff << endl;
                maxAngularDifference = angularDiff.array().abs().maxCoeff();
                //maxAngularDifference = angularDiff.norm();
                //cout << "maxAngularDifference:" << maxAngularDifference << endl;
                if (maxAngularDifference > maximal)
                //if (angularDiff.norm() > maximal)
                {
                    scale = maximal / maxAngularDifference;
                    if (jVerbose)
                        std::cout << "Cutting, maxAngularDifference=" << maxAngularDifference
                                  << ", scale = " << scale << std::endl
                                  << std::endl;
                }
                dTheta *= scale;
            }
            return dTheta;
        }
    }

    bool
    RobotPoseDifferentialIK::checkTolerances()
    {
        for (auto tcp : tcp_set)
        {
            if (getErrorPosition(tcp) > tolerancePosition[tcp] ||
                getErrorRotation(tcp) > toleranceRotation[tcp])
            {
                return false;
            }
        }
        return true;
    }

    void
    RobotPoseDifferentialIK::boxConstraints(bool enable)
    {
        considerBoxConstraints = enable;
    }

    bool
    RobotPoseDifferentialIK::computeSteps(float stepSize,
                                          float minChange,
                                          int maxNStep,
                                          bool performMinOneStep)
    {
        VR_ASSERT(rns);
        VR_ASSERT(nodes.size() == rns->getSize());
        VR_ASSERT(robot);
        std::vector<float> jv(nodes.size(), 0.0f);
        std::vector<float> jvBest = rns->getJointValues();
        Eigen::Matrix4f bestPose = robot->getGlobalPose();
        int step = 0;
        checkTolerances();
        float lastDist = getMeanErrorPosition();
        float bestDist = lastDist;
        int nrImprovements = 0;
        this->localJacobians.resize(this->tcp_set.size());

        //cout << verbose << endl;
        while (step < maxNStep)
        {
            Eigen::VectorXf dTheta = this->computeStep(stepSize);

            for (unsigned int i = 0; i < nodes.size(); i++)
            {
                jv[i] = (nodes[i]->getJointValue() + dTheta[i]);
                if (std::isnan(jv[i]) || std::isinf(jv[i]))
                {
                    VR_WARNING << "Aborting, invalid joint value (nan)" << std::endl;
                    return false;
                }
            }

            // update pose
            Eigen::Matrix3f m;
            m = Eigen::AngleAxisf(dTheta[nodes.size() + 3], Eigen::Vector3f::UnitX()) *
                Eigen::AngleAxisf(dTheta[nodes.size() + 4], Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf(dTheta[nodes.size() + 5], Eigen::Vector3f::UnitZ());
            Eigen::Vector3f pos(
                dTheta[nodes.size() + 0], dTheta[nodes.size() + 1], dTheta[nodes.size() + 2]);
            Eigen::Matrix4f deltaPose = Eigen::Matrix4f::Identity();
            deltaPose.block(0, 0, 3, 3) = m;
            deltaPose.block(0, 3, 3, 1) = pos;

            Eigen::Matrix4f resPose = deltaPose * robot->getGlobalPose();
            robot->setGlobalPose(resPose);

            if (considerBoxConstraints && !rns->checkJointLimits(jv))
            {
                VR_ERROR << "Joint limit violated" << std::endl;
            }
            robot->setJointValues(rns, jv);


            // check tolerances
            if (checkTolerances())
            {
                if (verbose)
                    VR_INFO << "Tolerances ok, loop:" << step << std::endl;
                return true;
            }

            float posDist = getMeanErrorPosition();

            // ensure that at least one step is performed (step==0 -> store best solution)
            if ((performMinOneStep && step == 0) || posDist < bestDist)
            {
                if (verbose && step != 0)
                    VR_INFO << "Loop:" << step << ", best IK dist: " << posDist << std::endl;

                bestPose = robot->getGlobalPose();
                jvBest = jv;
                bestDist = posDist;
                nrImprovements++;
            }
            if (checkImprovement && posDist > lastDist)
            {
                if (verbose)
                    VR_INFO << "Could not improve result any more (current position error="
                            << posDist << ", last loop's error:" << lastDist << "), loop:" << step
                            << std::endl;
                robot->setGlobalPose(bestPose);
                robot->setJointValues(rns, jvBest);
                return false;
            }
            float d = dTheta.norm();
            if (dTheta.norm() < minChange)
            {
                if (verbose)
                    VR_INFO << "Could not improve result any more (dTheta.norm()=" << d
                            << "), loop:" << step << std::endl;

                // set best result
                robot->setGlobalPose(bestPose);
                robot->setJointValues(rns, jvBest);
                return false; // local minimum
            }

            lastDist = posDist;
            step++;
        }

        // set best result
        robot->setGlobalPose(bestPose);
        robot->setJointValues(rns, jvBest);
        if (verbose && maxNStep > 1)
        {
            VR_INFO << "IK failed, improvementSteps:" << nrImprovements << ", loop:" << step
                    << std::endl;
            VR_INFO << "pos error:" << getMeanErrorPosition() << std::endl;
            VR_INFO << "rot error (tcp 0):" << getErrorRotation(tcp_set[0]) << std::endl;
        }
        return false;
    }

    bool
    RobotPoseDifferentialIK::computeSteps(float stepSize, float minChange, int maxSteps)
    {
        return computeSteps(stepSize, minChange, maxSteps, true);
    }

} // namespace VirtualRobot
