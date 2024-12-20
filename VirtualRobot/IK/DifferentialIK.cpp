#include "DifferentialIK.h"

#include <algorithm>
#include <cfloat>

#include <Eigen/Geometry>
#include <Eigen/src/Core/Matrix.h>

#include <VirtualRobot/Nodes/RobotNodeFourBar.h>
#include <VirtualRobot/Nodes/RobotNodeHemisphere.h>

#include "../CollisionDetection/CollisionChecker.h"
#include "../Nodes/RobotNodePrismatic.h"
#include "../Nodes/RobotNodeRevolute.h"
#include "../Robot.h"
#include "../VirtualRobotException.h"
#include "VirtualRobot/Logging.h"
#include "VirtualRobot/Nodes/RobotNode.h"
#include "VirtualRobot/RobotNodeSet.h"

//#define CHECK_PERFORMANCE

namespace VirtualRobot
{
    using std::cout;
    using std::endl;

    DifferentialIK::DifferentialIK(RobotNodeSetPtr _rns,
                                   RobotNodePtr _coordSystem,
                                   JacobiProvider::InverseJacobiMethod invJacMethod,
                                   float invParam) :
        JacobiProvider(_rns, invJacMethod), invParam(invParam), coordSystem(_coordSystem), nRows(0)
    {
        name = "DifferentialIK";

        if (!rns)
        {
            THROW_VR_EXCEPTION("Null data");
        }

        checkImprovement = false;
        nodes = rns->getAllRobotNodes();

        for (auto& node : nodes)
        {
            std::vector<RobotNodePtr> p = node->getAllParents(rns);
            p.push_back(
                node); // if the tcp is not fixed, it must be considered for calculating the Jacobian
            parents[node] = p;
        }

        convertMMtoM = false;
        verbose = false;
        positionMaxStep = -1.0f;

        tmpUpdateErrorDelta.resize(6);

        name = "DifferentialIK";
    }

    void
    DifferentialIK::setGoal(const Eigen::Matrix4f& goal,
                            SceneObjectPtr tcp,
                            IKSolver::CartesianSelection mode,
                            float tolerancePosition,
                            float toleranceRotation,
                            bool performInitialization)
    {
        if (!tcp)
        {
            tcp = this->getDefaultTCP();
        }

        // tcp not in list yet?
        if (find(tcp_set.begin(), tcp_set.end(), tcp) == tcp_set.end())
        {
            tcp_set.push_back(tcp);
        }

        this->targets[tcp] = goal;
        this->modes[tcp] = mode;
        this->tolerancePosition[tcp] = tolerancePosition;
        this->toleranceRotation[tcp] = toleranceRotation;

        RobotNodePtr tcpRN = std::dynamic_pointer_cast<RobotNode>(tcp);

        if (!tcpRN)
        {
            if (!tcp->getParent())
            {
                VR_ERROR << "tcp not linked to a parent!!!" << std::endl;
                return;
            }

            tcpRN = std::dynamic_pointer_cast<RobotNode>(tcp->getParent());

            if (!tcpRN)
            {
                VR_ERROR << "tcp not linked to robotNode!!!" << std::endl;
                return;
            }
        }

        // check if we already computed the parents for tcp
        if (parents.find(tcpRN) == parents.end())
        {
            parents[tcpRN] = tcpRN->getAllParents(rns);
            parents[tcpRN].push_back(tcpRN);
        }

        // tcp not in list yet?
        /*if (find(tcp_set.begin(), tcp_set.end(), tcp) == tcp_set.end())
        {
            tcp_set.push_back(tcp);
        }*/
        if (performInitialization)
        {
            initialize();
        }
    }

    Eigen::MatrixXf
    DifferentialIK::getJacobianMatrix()
    {
        updateJacobianMatrix(currentJacobian);
        return currentJacobian;
    }

    void
    DifferentialIK::updateJacobianMatrix(Eigen::MatrixXf& jacobian)
    {
        VR_ASSERT(initialized);

#ifdef ALLOW_RESIZE

        if (jacobian.rows() != nRows || jacobian.cols() != nDoF)
        {
            jacobian.resize(nRows, nDoF);
        }

#endif
        VR_ASSERT(static_cast<std::size_t>(jacobian.rows()) == nRows &&
                  static_cast<std::size_t>(jacobian.cols()) == nodes.size());

        size_t index = 0;

        for (auto tcp : tcp_set)
        {
            if (this->targets.find(tcp) != this->targets.end())
            {
                auto& mode = this->modes[tcp];
                updateJacobianMatrix(partJacobians[tcp], tcp, mode);
                //updatePartJacobian(tcp, mode, jacobian.block(index, 0,  partJacobians[tcp].rows(), nDoF));
                //Eigen::MatrixXf partJacobian = this->getJacobianMatrix(tcp, mode);
                jacobian.block(index, 0, partJacobians[tcp].rows(), nDoF) = partJacobians[tcp];

                if (mode & IKSolver::X)
                {
                    index++;
                }

                if (mode & IKSolver::Y)
                {
                    index++;
                }

                if (mode & IKSolver::Z)
                {
                    index++;
                }

                if (mode & IKSolver::Orientation)
                {
                    index += 3;
                }
            }
            else
            {
                VR_ERROR << "Internal error?!" << std::endl; // Error
            }
        }
    }

    Eigen::VectorXf
    DifferentialIK::getError(float stepSize)
    {
        updateError(currentError, stepSize);
        return currentError;
    }

    void
    DifferentialIK::updateError(Eigen::VectorXf& error, float stepSize)
    {
        VR_ASSERT(initialized);

#ifdef ALLOW_RESIZE

        if (error.rows() != nRows)
        {
            error.resize(nRows);
        }

#endif
        VR_ASSERT(static_cast<std::size_t>(error.rows()) == nRows);

        //Eigen::VectorXf error(nRows);

        // compute error
        size_t index = 0;

        for (auto tcp : tcp_set)
        {
            if (this->targets.find(tcp) != this->targets.end())
            {
                //Eigen::VectorXf delta =
                updateDeltaToGoal(tmpUpdateErrorDelta, tcp);
                IKSolver::CartesianSelection mode = this->modes[tcp];
                tmpUpdateErrorPosition = tmpUpdateErrorDelta.head(3);
                tmpUpdateErrorPosition *= stepSize;

                if (positionMaxStep > 0)
                {
                    if (tmpUpdateErrorPosition.norm() > positionMaxStep)
                    {
                        tmpUpdateErrorPosition *= positionMaxStep / tmpUpdateErrorPosition.norm();
                    }
                }

                if (mode & IKSolver::X)
                {
                    error(index) = tmpUpdateErrorPosition(0);
                    index++;
                }

                if (mode & IKSolver::Y)
                {
                    error(index) = tmpUpdateErrorPosition(1);
                    index++;
                }

                if (mode & IKSolver::Z)
                {
                    error(index) = tmpUpdateErrorPosition(2);
                    index++;
                }

                if (mode & IKSolver::Orientation)
                {
                    error.segment(index, 3) = tmpUpdateErrorDelta.tail(3) * stepSize;
                    index += 3;
                }
            }
            else
            {
                VR_ERROR << "Internal error?!" << std::endl; // Error
            }
        }
    }

    Eigen::MatrixXf
    DifferentialIK::getJacobianMatrix(SceneObjectPtr tcp)
    {
        return getJacobianMatrix(tcp, IKSolver::All);
    }

    Eigen::MatrixXf
    DifferentialIK::getJacobianMatrix(IKSolver::CartesianSelection mode)
    {
        return getJacobianMatrix(SceneObjectPtr(), mode);
    }

    Eigen::MatrixXf
    DifferentialIK::getJacobianMatrix(SceneObjectPtr tcp, IKSolver::CartesianSelection mode)
    {
        if (!initialized)
        {
            initialize();
        }

        int partSize = 0;

        if (mode & IKSolver::X)
        {
            partSize++;
        }

        if (mode & IKSolver::Y)
        {
            partSize++;
        }

        if (mode & IKSolver::Z)
        {
            partSize++;
        }

        if (mode & IKSolver::Orientation)
        {

            partSize += 3;
        }

        Eigen::MatrixXf jac(partSize, nDoF);
        updateJacobianMatrix(jac, tcp, mode);
        return jac;
    }

    void
    DifferentialIK::updateJacobianMatrix(Eigen::MatrixXf& jac,
                                         SceneObjectPtr tcp,
                                         IKSolver::CartesianSelection mode)
    {

        if (!initialized)
        {
            initialize();
        }

        // obtain the size of the matrix.
        unsigned int size = 0;

        if (mode & IKSolver::X)
        {
            size++;
        }

        if (mode & IKSolver::Y)
        {
            size++;
        }

        if (mode & IKSolver::Z)
        {
            size++;
        }

        if (mode & IKSolver::Orientation)
        {
            size += 3;
        }

#ifdef ALLOW_RESIZE

        if (jac.rows() != size || jac.cols() != nDoF)
        {
            jac.resize(size, nDof);
        }

#endif
        VR_ASSERT(static_cast<std::size_t>(jac.rows()) == size &&
                  static_cast<std::size_t>(jac.cols()) == nDoF);

        // Create matrices for the position and the orientation part of the jacobian.

        if (!tcp)
        {
            tcp = this->getDefaultTCP();
        }


        //  THROW_VR_EXCEPTION_IF(!tcp,boost::format("No tcp defined in node set \"%1%\" of robot %2% (DifferentialIK::%3% )") % this->rns->getName() % this->rns->getRobot()->getName() % BOOST_CURRENT_FUNCTION);

        RobotNodePtr tcpRN = std::dynamic_pointer_cast<RobotNode>(tcp);

        if (!tcpRN)
        {
            if (!tcp->getParent())
            {
                VR_ERROR << "tcp not linked to a parent!!!" << std::endl;
                jac.setZero();
                return;
            }

            tcpRN = std::dynamic_pointer_cast<RobotNode>(tcp->getParent());

            if (!tcpRN)
            {
                VR_ERROR << "tcp not linked to robotNode!!!" << std::endl;
                jac.setZero();
                return;
            }
        }

        if (parents.find(tcpRN) == parents.end())
        {
            parents[tcpRN] = tcpRN->getAllParents(rns);
            parents[tcpRN].push_back(tcpRN);
        }

        Eigen::Vector3f axis;
        Eigen::Vector3f toTCP;
        tmpUpdateJacobianPosition.setZero();
        tmpUpdateJacobianOrientation.setZero();


        // Iterate over all degrees of freedom
        for (size_t i = 0; i < nDoF; i++)
        {

#ifdef CHECK_PERFORMANCE
            clock_t startT = clock();
#endif

            const RobotNodePtr& dof = this->nodes[i];
            //std::vector<RobotNodePtr> parents = parents[tcp];//tcp->getAllParents(this->rns);

            //check if the tcp is affected by this DOF
            auto p = parents[tcpRN];
            const bool isParent = std::find(p.begin(), p.end(), dof) != p.end();
            if (isParent)
            {
                // Calculus for rotational joints is different as for prismatic joints.
                if (dof->isRotationalJoint())
                {
                    // todo: find a better way of handling different joint types
                    std::shared_ptr<RobotNodeRevolute> revolute =
                        std::dynamic_pointer_cast<RobotNodeRevolute>(dof);
                    THROW_VR_EXCEPTION_IF(!revolute, "Internal error: expecting revolute joint");

                    // get axis
                    axis = revolute->getJointRotationAxis(coordSystem);

                    // if necessary calculate the position part of the Jacobian
                    if (mode & IKSolver::Position)
                    {

                        if (coordSystem)
                        {
                            toTCP = coordSystem->toLocalCoordinateSystem(tcp->getGlobalPose())
                                        .block(0, 3, 3, 1) -
                                    coordSystem->toLocalCoordinateSystem(dof->getGlobalPose())
                                        .block(0, 3, 3, 1);
                        }
                        else
                        {
                            toTCP = tcp->getGlobalPose().block(0, 3, 3, 1) -
                                    dof->getGlobalPose().block(0, 3, 3, 1);
                        }

                        if (convertMMtoM)
                        {
                            toTCP /= 1000.0f;
                        }

                        //cout << "toTCP: " << tcp->getName() << std::endl;
                        //cout << toTCP << std::endl;
                        tmpUpdateJacobianPosition.block(0, i, 3, 1) = axis.cross(toTCP);
                    }

                    // and the orientation part
                    if (mode & IKSolver::Orientation)
                    {
                        tmpUpdateJacobianOrientation.block(0, i, 3, 1) = axis;
                    }
                }
                else if (dof->isTranslationalJoint())
                {
                    // -> prismatic joint
                    std::shared_ptr<RobotNodePrismatic> prismatic =
                        std::dynamic_pointer_cast<RobotNodePrismatic>(dof);
                    THROW_VR_EXCEPTION_IF(!prismatic, "Internal error: expecting prismatic joint");
                    // todo: find a better way of handling different joint types
                    axis = prismatic->getJointTranslationDirection(coordSystem);

                    //if (!convertMMtoM)
                    //  axis *= 1000.0f; // we have a mm jacobian -> no, we say how much the joint moves when applying 1 'unit', this can be mm or m and depends only on the error vector
                    // if necessary calculate the position part of the Jacobian
                    if (mode & IKSolver::Position)
                    {
                        tmpUpdateJacobianPosition.block(0, i, 3, 1) = axis;
                    }

                    // no orientation part required with prismatic joints
                }
                else if (dof->isHemisphereJoint())
                {
                    RobotNodeHemispherePtr hemisphere =
                        std::dynamic_pointer_cast<RobotNodeHemisphere>(dof);

                    if (hemisphere->isSecondHemisphereJointNode())
                    {
                        // Set Jacobian for both DoFs.
                        RobotNodeHemisphere::SecondData& second = hemisphere->getSecondData();
                        const hemisphere::Maths::Jacobian jacobian = second.getJacobian();

                        using Matrix3x2f = Eigen::Matrix<float, 3, 2>;

                        const Matrix3x2f velocities = jacobian.block<3, 2>(0, 0).cast<float>();

                        const Matrix3x2f rotAxes = jacobian.block<3, 2>(3, 0).cast<float>();

                        const Eigen::Matrix3f globalOri = second.firstNode->getGlobalOrientation();

                        Matrix3x2f velocitiesCoord = globalOri * velocities;
                        Matrix3x2f rotAxesCoord = globalOri * rotAxes;
                        if (coordSystem)
                        {
                            auto coordSystemInverseGlobalRot =
                                coordSystem->getGlobalOrientation().transpose();

                            velocitiesCoord = coordSystemInverseGlobalRot * velocitiesCoord;
                            rotAxesCoord = coordSystemInverseGlobalRot * rotAxesCoord;
                        }

                        tmpUpdateJacobianPosition.block<3, 2>(0, i - 1) = velocitiesCoord;

                        if (convertMMtoM)
                        {
                            tmpUpdateJacobianPosition.block<3, 2>(0, i - 1) /= 1000.f;
                        }

                        tmpUpdateJacobianOrientation.block<3, 2>(0, i - 1) = rotAxesCoord;
                    }
                    else
                    {
                        // Nothing to do - everything is handled by second DoF.
                    }
                }
                else if (dof->isFourBarJoint())
                {
                    VR_ASSERT(coordSystem == nullptr); // NOT implemented yet ...

                    const RobotNodeFourBar* fourBarJoint =
                        dynamic_cast<RobotNodeFourBar*>(dof.get());

                    const Eigen::Vector3f global_P_tcp = tcp->getGlobalPosition().cast<float>();

                    const four_bar::Joint::Jacobian jacobianInBaseD =
                        fourBarJoint->getJacobian(global_P_tcp);
                    Eigen::Vector3f jacobianInBase = jacobianInBaseD.cast<float>();

                    axis = fourBarJoint->getJointRotationAxis(coordSystem);

                    // if necessary calculate the position part of the Jacobian
                    if (mode & IKSolver::Position)
                    {

                        const Eigen::Matrix3f global_R_base =
                            fourBarJoint->baseFrame(nullptr).block<3, 3>(0, 0);

                        Eigen::Matrix3f ref_R_base;

                        if (coordSystem)
                        {
                            const Eigen::Matrix3f global_R_ref =
                                coordSystem->getGlobalOrientation();

                            ref_R_base = global_R_ref.transpose() * global_R_base;

                            toTCP = coordSystem->toLocalCoordinateSystem(tcp->getGlobalPose())
                                        .block(0, 3, 3, 1) -
                                    coordSystem->toLocalCoordinateSystem(dof->getGlobalPose())
                                        .block(0, 3, 3, 1);
                        }
                        else
                        {
                            ref_R_base = global_R_base;

                            toTCP = tcp->getGlobalPose().block(0, 3, 3, 1) -
                                    dof->getGlobalPose().block(0, 3, 3, 1);
                        }

                        const Eigen::Vector3f jacobianPosInBaseFrame{
                            jacobianInBase(0), jacobianInBase(1), 0};
                        const Eigen::Vector3f jacobianPosInRefFrame =
                            ref_R_base * jacobianPosInBaseFrame;

                        tmpUpdateJacobianPosition.block<3, 1>(0, i) = jacobianPosInRefFrame;

                        if (convertMMtoM) // positional part
                        {
                            tmpUpdateJacobianPosition.block<3, 1>(0, i) /= 1000.f;
                        }
                    }

                    // and the orientation part
                    if (mode & IKSolver::Orientation)
                    {
                        tmpUpdateJacobianOrientation.block(0, i, 3, 1) = axis * jacobianInBase(2);
                    }
                }
            }

#ifdef CHECK_PERFORMANCE
            clock_t endT = clock();
            float diffClock = (float)(((float)(endT - startT) / (float)CLOCKS_PER_SEC) * 1000.0f);

            if (diffClock > 0.0f)
            {
                std::cout << "Jacobi Loop " << i << ": RobotNode: " << dof->getName()
                          << ", time:" << diffClock << std::endl;
            }

#endif
        }

#ifdef CHECK_PERFORMANCE
        clock_t startT = clock();
#endif


        // copy only what is required (and was previously calculated)
        unsigned int index = 0;

        if (mode & IKSolver::X)
        {
            jac.row(index) = tmpUpdateJacobianPosition.row(0);
            index++;
        }

        if (mode & IKSolver::Y)
        {
            jac.row(index) = tmpUpdateJacobianPosition.row(1);
            index++;
        }

        if (mode & IKSolver::Z)
        {
            jac.row(index) = tmpUpdateJacobianPosition.row(2);
            index++;
        }

        if (mode & IKSolver::Orientation)
        {
            jac.block(index, 0, 3, nDoF) = tmpUpdateJacobianOrientation;
        }

        //cout << "partial JACOBIAN: (row 1-3)" << std::endl;
        //cout << result.block(0,0,3,3) << std::endl;

#ifdef CHECK_PERFORMANCE
        clock_t endT = clock();
        float diffClock = (float)(((float)(endT - startT) / (float)CLOCKS_PER_SEC) * 1000.0f);

        if (diffClock > 1.0f)
        {
            std::cout << "Jacobirest time:" << diffClock << std::endl;
        }

#endif

        /*if (jointWeights.rows() == nDoF)
        {
            Eigen::MatrixXf W = jointWeights.asDiagonal();
            //Eigen::MatrixXf W_1 = W.inverse();
            result = result * W;
        }*/
    }

    Eigen::MatrixXf
    DifferentialIK::getPseudoInverseJacobianMatrix()
    {
        return getPseudoInverseJacobianMatrix(SceneObjectPtr());
    }

    Eigen::MatrixXf
    DifferentialIK::getPseudoInverseJacobianMatrix(IKSolver::CartesianSelection mode)
    {
        return getPseudoInverseJacobianMatrix(SceneObjectPtr(), mode);
    }

    Eigen::MatrixXf
    DifferentialIK::getPseudoInverseJacobianMatrix(SceneObjectPtr tcp,
                                                   IKSolver::CartesianSelection mode)
    {
#ifdef CHECK_PERFORMANCE
        clock_t startT = clock();
#endif
        Eigen::MatrixXf Jacobian = this->getJacobianMatrix(tcp, mode);
#ifdef CHECK_PERFORMANCE
        clock_t startT2 = clock();
#endif
        Eigen::MatrixXf res = computePseudoInverseJacobianMatrix(Jacobian, invParam);
#ifdef CHECK_PERFORMANCE
        clock_t endT = clock();
        float diffClock1 = (float)(((float)(startT2 - startT) / (float)CLOCKS_PER_SEC) * 1000.0f);
        float diffClock2 = (float)(((float)(endT - startT2) / (float)CLOCKS_PER_SEC) * 1000.0f);
        std::cout << "getPseudoInverseJacobianMatrix time1:" << diffClock1
                  << ", time2: " << diffClock2 << std::endl;
#endif
        return res;
    }

    void
    DifferentialIK::setNRows()
    {
        initialize();
    }

    void
    DifferentialIK::initialize()
    {
        this->nRows = 0;
        nDoF = nodes.size();

        partJacobians.clear();

        for (auto tcp : tcp_set)
        {
            int partSize = 0;

            if (this->modes[tcp] & IKSolver::X)
            {
                partSize++;
            }

            if (this->modes[tcp] & IKSolver::Y)
            {
                partSize++;
            }

            if (this->modes[tcp] & IKSolver::Z)
            {
                partSize++;
            }

            if (this->modes[tcp] & IKSolver::Orientation)
            {

                partSize += 3;
            }

            nRows += partSize;

            Eigen::MatrixXf jac(partSize, nDoF);
            partJacobians[tcp] = jac;
        }

        currentError.resize(nRows);
        currentJacobian.resize(nRows, nDoF);
        currentInvJacobian.resize(nDoF, nRows);

        tmpUpdateJacobianPosition = Eigen::MatrixXf::Zero(3, nDoF);
        tmpUpdateJacobianOrientation = Eigen::MatrixXf::Zero(3, nDoF);

        tmpComputeStepTheta.resize(nDoF);

        initialized = true;
    }

    void
    DifferentialIK::print()
    {
        JacobiProvider::print();

        if (coordSystem)
        {
            std::cout << "Coordsystem: " << coordSystem->getName() << std::endl;
        }
        else
        {
            std::cout << "Coordsystem: global" << std::endl;
        }

        std::cout << "TCPs:" << std::endl;

        for (auto tcp : tcp_set)
        {
            RobotNodePtr tcpRN = std::dynamic_pointer_cast<RobotNode>(tcp);

            if (!tcpRN)
            {
                continue;
            }

            std::cout << "* " << tcpRN->getName() << std::endl;
            std::cout << "** Target: " << endl << this->targets[tcp] << std::endl;
            std::cout << "** Error: " << getDeltaToGoal(tcp).transpose() << std::endl;
            std::cout << "** mode:";

            if (this->modes[tcp] == IKSolver::All)
            {
                std::cout << "all" << std::endl;
            }
            else if (this->modes[tcp] == IKSolver::Position)
            {
                std::cout << "position" << std::endl;
            }
            else if (this->modes[tcp] == IKSolver::Orientation)
            {
                std::cout << "orientation" << std::endl;
            }
            else
            {
                std::cout << "unknown" << std::endl;
            }

            std::cout << "** tolerances pos: " << this->tolerancePosition[tcp]
                      << ", rot:" << this->toleranceRotation[tcp] << std::endl;
            std::cout << "** Nodes:";

            // Iterate over all degrees of freedom
            for (size_t i = 0; i < nDoF; i++)
            {
                RobotNodePtr dof = this->nodes[i];

                //check if the tcp is affected by this DOF
                if (find(parents[tcpRN].begin(), parents[tcpRN].end(), dof) != parents[tcpRN].end())
                {
                    std::cout << dof->getName() << ",";
                }
            }

            std::cout << std::endl;
        }
    }

    void
    DifferentialIK::setGoal(const Eigen::Vector3f& goal,
                            SceneObjectPtr tcp,
                            IKSolver::CartesianSelection mode,
                            float tolerancePosition,
                            float toleranceRotation,
                            bool performInitialization)
    {
        Eigen::Matrix4f trafo;
        trafo.setIdentity();
        trafo.block(0, 3, 3, 1) = goal;
        this->setGoal(
            trafo, tcp, mode, tolerancePosition, toleranceRotation, performInitialization);
    }

    RobotNodePtr
    DifferentialIK::getDefaultTCP()
    {
        return rns->getTCP();
    }

    Eigen::VectorXf
    DifferentialIK::getDeltaToGoal(SceneObjectPtr tcp)
    {
        Eigen::VectorXf result(6);
        updateDeltaToGoal(result, tcp);
        return result;
    }

    Eigen::VectorXf
    DifferentialIK::getDelta(const Eigen::Matrix4f& current,
                             const Eigen::Matrix4f& goal,
                             IKSolver::CartesianSelection mode)
    {
        Eigen::VectorXf result(6);
        updateDelta(result, current, goal, mode);
        return result;
    }

    void
    DifferentialIK::updateDeltaToGoal(Eigen::VectorXf& delta, SceneObjectPtr tcp)
    {
        if (!tcp)
        {
            tcp = getDefaultTCP();
        }

        VR_ASSERT(tcp);
        updateDelta(delta, tcp->getGlobalPose(), this->targets[tcp], this->modes[tcp]);
    }

    void
    DifferentialIK::updateDelta(Eigen::VectorXf& delta,
                                const Eigen::Matrix4f& current,
                                const Eigen::Matrix4f& goal,
                                IKSolver::CartesianSelection mode)
    {
#ifdef ALLOW_RESIZE

        if (delta.rows() != 6)
        {
            delta.resize(6);
        }

#endif
        VR_ASSERT(delta.rows() == 6);

        delta.setZero();

        Eigen::Vector3f position = goal.block(0, 3, 3, 1) - current.block(0, 3, 3, 1);

        if (mode & IKSolver::X)
        {
            delta(0) = position(0);
        }

        if (mode & IKSolver::Y)
        {
            delta(1) = position(1);
        }

        if (mode & IKSolver::Z)
        {
            delta(2) = position(2);
        }

        if (mode & IKSolver::Orientation)
        {
            tmpDeltaOrientation = goal * current.inverse();
            tmpDeltaAA = tmpDeltaOrientation.block<3, 3>(0, 0);
            //Eigen::AngleAxis<float> aa(orientation.block<3, 3>(0, 0));
            // TODO: make sure that angle is >0!?
            delta.tail(3) = tmpDeltaAA.axis() * tmpDeltaAA.angle();
        }
    }

    Eigen::VectorXf
    DifferentialIK::computeStep(float stepSize)
    {
        VR_ASSERT(initialized);

        updateError(currentError, stepSize);
        updateJacobianMatrix(currentJacobian);
        //Eigen::VectorXf dTheta(nDoF);

        updatePseudoInverseJacobianMatrix(currentInvJacobian, currentJacobian);

        tmpComputeStepTheta = currentInvJacobian * currentError;

        /*if (jointWeights.rows() == dTheta.rows())
        {
            for (size_t i = 0; i < jointWeights.rows(); i++)
                dTheta(i) *= jointWeights(i);
        }*/
        if (verbose)
        {
            VR_INFO << "ERROR (TASK):" << endl << currentError << std::endl;
            VR_INFO << "JACOBIAN:" << endl << currentJacobian << std::endl;
            VR_INFO << "PSEUDOINVERSE JACOBIAN:" << endl << currentInvJacobian << std::endl;
            VR_INFO << "THETA (JOINT):" << endl << tmpComputeStepTheta << std::endl;
        }

        return tmpComputeStepTheta;
    }

    float
    DifferentialIK::getErrorPosition(SceneObjectPtr tcp)
    {
        if (modes[tcp] == IKSolver::Orientation)
        {
            return 0.0f; // ignoring position
        }

        if (!tcp)
        {
            tcp = getDefaultTCP();
        }

        Eigen::Vector3f position =
            targets[tcp].block(0, 3, 3, 1) - tcp->getGlobalPose().block(0, 3, 3, 1);
        float result = 0.0f;

        if (modes[tcp] & IKSolver::X)
        {
            result += position(0) * position(0);
        }

        if (modes[tcp] & IKSolver::Y)
        {
            result += position(1) * position(1);
        }

        if (modes[tcp] & IKSolver::Z)
        {
            result += position(2) * position(2);
        }

        return sqrtf(result);
    }

    float
    DifferentialIK::getErrorRotation(SceneObjectPtr tcp)
    {
        if (!(modes[tcp] & IKSolver::Orientation))
        {
            return 0.0f; // no error in this dimensions
        }

        if (!tcp)
        {
            tcp = getDefaultTCP();
        }

        Eigen::Matrix4f orientation = this->targets[tcp] * tcp->getGlobalPose().inverse();
        Eigen::AngleAxis<float> aa(orientation.block<3, 3>(0, 0));
        return aa.angle();
    }

    float
    DifferentialIK::getMeanErrorPosition()
    {
        if (tcp_set.size() == 0)
        {
            return 0.0f;
        }

        float res = 0;

        for (auto tcp : tcp_set)
        {
            res += getErrorPosition(tcp);
        }

        res /= float(tcp_set.size());
        return res;
    }

    bool
    DifferentialIK::checkTolerances()
    {
        bool result = true;

        for (auto tcp : tcp_set)
        {
            float currentErrorPos = getErrorPosition(tcp);
            float maxErrorPos = tolerancePosition[tcp];
            float currentErrorRot = getErrorRotation(tcp);
            float maxErrorRot = toleranceRotation[tcp];

            if (verbose)
            {
                VR_INFO << "TCP " << tcp->getName() << ", errPos:" << currentErrorPos
                        << ", errRot:" << currentErrorRot << ", maxErrPos:" << maxErrorPos
                        << ", maxErrorRot:" << maxErrorRot << std::endl;
            }

            if (currentErrorPos > maxErrorPos || currentErrorRot > maxErrorRot)
            {
                result = false;
                //break;
            }
        }

        return result;
    }

    void
    DifferentialIK::checkImprovements(bool enable)
    {
        checkImprovement = enable;
    }

    bool
    DifferentialIK::computeSteps(float stepSize, float minumChange, int maxNStep)
    {
        VR_ASSERT(rns);
        VR_ASSERT(nodes.size() == rns->getSize());

        RobotPtr robot = rns->getRobot();
        VR_ASSERT(robot);

        std::vector<float> jv(nodes.size(), 0.0f);
        std::vector<float> jvBest = rns->getJointValues();
        int step = 0;
        checkTolerances();
        float lastDist = FLT_MAX;

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

            robot->setJointValues(rns, jv);

            // check tolerances
            if (checkTolerances())
            {
                if (verbose)
                {
                    VR_INFO << "Tolerances ok, loop:" << step << std::endl;
                }

                return true;
            }

            float d = dTheta.norm();

            if (dTheta.norm() < minumChange)
            {
                if (verbose)
                {
                    VR_INFO << "Could not improve result any more (dTheta.norm()=" << d
                            << "), loop:" << step << std::endl;
                }

                robot->setJointValues(rns, jvBest);
                return false;
            }

            float posDist = getMeanErrorPosition();

            if (checkImprovement && posDist > lastDist)
            {
                if (verbose)
                {
                    VR_INFO << "Could not improve result any more (current position error="
                            << posDist << ", last loop's error:" << lastDist << "), loop:" << step
                            << std::endl;
                }

                robot->setJointValues(rns, jvBest);
                return false;
            }

            jvBest = jv;
            lastDist = posDist;
            step++;
        }

        if (verbose)
        {
            VR_INFO << "IK failed, loop:" << step << std::endl;
            VR_INFO << "pos error:" << getErrorPosition() << std::endl;
            VR_INFO << "rot error:" << getErrorRotation() << std::endl;
        }

        robot->setJointValues(rns, jvBest);
        return false;
    }

    bool
    DifferentialIK::solveIK(float stepSize /*= 0.2f*/,
                            float minChange /*= 0.0f */,
                            int maxSteps /*= 50*/)
    {
        return computeSteps(stepSize, minChange, maxSteps);
    }

    void
    DifferentialIK::convertModelScalingtoM(bool enable)
    {
        convertMMtoM = enable;
    }

    void
    DifferentialIK::setVerbose(bool enable)
    {
        verbose = enable;
    }

    void
    DifferentialIK::setMaxPositionStep(float s)
    {
        positionMaxStep = s;
    }

} // namespace VirtualRobot
