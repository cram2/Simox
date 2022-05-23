#include <Eigen/Geometry>
#include "JacobiProvider.h"

#include <VirtualRobot/MathTools.h>

#include <Eigen/Dense>

#include <algorithm>

//#define CHECK_PERFORMANCE

namespace VirtualRobot
{
    using std::cout;
    using std::endl;

    JacobiProvider::JacobiProvider(RobotNodeSetPtr rns, InverseJacobiMethod invJacMethod) :
        name("JacobiProvider"), rns(rns), inverseMethod(invJacMethod)
    {
        initialized = false;
        dampedSvdLambda = 0.1;
        jacobiMMRegularization = 50;
        jacobiRadianRegularization = 1;
    }

    JacobiProvider::~JacobiProvider()
    = default;

    Eigen::MatrixXd JacobiProvider::getJacobianMatrixD()
    {
        return getJacobianMatrix().cast<double>();
    }

    Eigen::MatrixXd JacobiProvider::getJacobianMatrixD(SceneObjectPtr tcp)
    {
        return getJacobianMatrix(tcp).cast<double>();
    }

    Eigen::MatrixXf JacobiProvider::getPseudoInverseJacobianMatrix(SceneObjectPtr tcp, const Eigen::VectorXf regularization)
    {
#ifdef CHECK_PERFORMANCE
        clock_t startT = clock();
#endif
        Eigen::MatrixXf Jacobian = this->getJacobianMatrix(tcp);
#ifdef CHECK_PERFORMANCE
        clock_t startT2 = clock();
#endif
        Eigen::MatrixXf res = computePseudoInverseJacobianMatrix(Jacobian, regularization);
#ifdef CHECK_PERFORMANCE
        clock_t endT = clock();
        float diffClock1 = (float)(((float)(startT2 - startT) / (float)CLOCKS_PER_SEC) * 1000.0f);
        float diffClock2 = (float)(((float)(endT - startT2) / (float)CLOCKS_PER_SEC) * 1000.0f);
        std::cout << "getPseudoInverseJacobianMatrix time1:" << diffClock1 << ", time2: " << diffClock2 << std::endl;
#endif
        return res;
    }

    Eigen::MatrixXd JacobiProvider::getPseudoInverseJacobianMatrixD(SceneObjectPtr tcp, const Eigen::VectorXd regularization)
    {
#ifdef CHECK_PERFORMANCE
        clock_t startT = clock();
#endif
        Eigen::MatrixXd Jacobian = this->getJacobianMatrixD(tcp);
#ifdef CHECK_PERFORMANCE
        clock_t startT2 = clock();
#endif
        Eigen::MatrixXd res = computePseudoInverseJacobianMatrixD(Jacobian, regularization);
#ifdef CHECK_PERFORMANCE
        clock_t endT = clock();
        float diffClock1 = (float)(((float)(startT2 - startT) / (float)CLOCKS_PER_SEC) * 1000.0f);
        float diffClock2 = (float)(((float)(endT - startT2) / (float)CLOCKS_PER_SEC) * 1000.0f);
        std::cout << "getPseudoInverseJacobianMatrix time1:" << diffClock1 << ", time2: " << diffClock2 << std::endl;
#endif
        return res;
    }

    Eigen::VectorXf JacobiProvider::getJacobiRegularization(IKSolver::CartesianSelection mode)
    {
        Eigen::VectorXf regularization(6);

        int i = 0;

        if (mode & IKSolver::X)
        {
            regularization(i++) = 1 / jacobiMMRegularization;
        }

        if (mode & IKSolver::Y)
        {
            regularization(i++) = 1 / jacobiMMRegularization;
        }

        if (mode & IKSolver::Z)
        {
            regularization(i++) = 1 / jacobiMMRegularization;
        }

        if (mode & IKSolver::Orientation)
        {
            regularization(i++) = 1 / jacobiRadianRegularization;
            regularization(i++) = 1 / jacobiRadianRegularization;
            regularization(i++) = 1 / jacobiRadianRegularization;
        }
        return regularization.topRows(i);

    }


    Eigen::MatrixXf JacobiProvider::getPseudoInverseJacobianMatrix(const Eigen::VectorXf regularization)
    {
        Eigen::MatrixXf Jacobian = this->getJacobianMatrix();
        return computePseudoInverseJacobianMatrix(Jacobian, regularization);
        //return getPseudoInverseJacobianMatrix(rns->getTCP());
    }

    Eigen::MatrixXd JacobiProvider::getPseudoInverseJacobianMatrixD(const Eigen::VectorXd regularization)
    {
        Eigen::MatrixXd Jacobian = this->getJacobianMatrixD();
        return computePseudoInverseJacobianMatrixD(Jacobian, regularization);
    }

    Eigen::MatrixXf JacobiProvider::computePseudoInverseJacobianMatrix(const Eigen::MatrixXf& m, const Eigen::VectorXf regularization) const
    {
        return computePseudoInverseJacobianMatrix(m, 0.0f, regularization);
    }

    Eigen::MatrixXd JacobiProvider::computePseudoInverseJacobianMatrixD(const Eigen::MatrixXd& m, const Eigen::VectorXd regularization) const
    {
        return computePseudoInverseJacobianMatrixD(m, 0.0, regularization);
    }

    Eigen::MatrixXf JacobiProvider::computePseudoInverseJacobianMatrix(const Eigen::MatrixXf& m, float invParameter, const Eigen::VectorXf regularization) const
    {
        Eigen::MatrixXf result(m.cols(), m.rows());
        updatePseudoInverseJacobianMatrix(result, m, invParameter, regularization);
        return result;
    }

    Eigen::MatrixXd JacobiProvider::computePseudoInverseJacobianMatrixD(const Eigen::MatrixXd& m, double invParameter, const Eigen::VectorXd regularization) const
    {
        Eigen::MatrixXd result(m.cols(), m.rows());
        updatePseudoInverseJacobianMatrixD(result, m, invParameter, regularization);
        return result;
    }

    void JacobiProvider::updatePseudoInverseJacobianMatrix(Eigen::MatrixXf& invJac, const Eigen::MatrixXf& m, float invParameter, Eigen::VectorXf regularization) const
    {
        Eigen::MatrixXf m2 = m;
        VR_ASSERT(regularization.rows() == 0 || regularization.rows() == m2.rows());
        if(regularization.rows() != m2.rows())
        {
            regularization = Eigen::VectorXf::Ones(m2.rows());
        }
        //std::cout << "regularization: " << regularization.transpose() << std::endl;
        m2 = regularization.asDiagonal() * m2;
        updatePseudoInverseJacobianMatrixInternal(invJac, m2, invParameter);
        invJac = invJac * regularization.asDiagonal();
    }

    void JacobiProvider::updatePseudoInverseJacobianMatrixInternal(Eigen::MatrixXf& invJac, const Eigen::MatrixXf& m, float invParameter) const
    {
#ifdef CHECK_PERFORMANCE
        clock_t startT = clock();
#endif

        VR_ASSERT(m.rows() > 0 && m.cols() > 0 && invJac.cols() == m.rows() && invJac.rows() == m.cols());

        switch (inverseMethod)
        {
            case eTranspose:
            {
                if (jointWeights.rows() == m.cols())
                {
                    Eigen::MatrixXf W = jointWeights.asDiagonal();
                    Eigen::MatrixXf W_1 = W.inverse();
                    invJac = W_1 * m.transpose() * (m * W_1 * m.transpose()).inverse();
                }
                else
                {
                    invJac = m.transpose() * (m * m.transpose()).inverse();
                }

                break;
            }

            case eSVD:
            {
                float pinvtoler = 0.00001f;

                if (invParameter != 0.0f)
                {
                    pinvtoler = invParameter;
                }

                if (jointWeights.rows() == m.cols())
                {
                    Eigen::MatrixXf W_12(jointWeights.rows(), jointWeights.rows());
                    W_12.setZero();

                    for (int i = 0; i < jointWeights.rows(); i++)
                    {
                        THROW_VR_EXCEPTION_IF(jointWeights(i) <= 0.f, "joint weights cannot be negative or zero");
                        W_12(i, i) = sqrt(1 / jointWeights(i));
                    }

                    invJac = W_12 * MathTools::getPseudoInverse(m * W_12, pinvtoler);
                }
                else
                {
                    invJac = MathTools::getPseudoInverse(m, pinvtoler);
                }

                break;
            }

            case eSVDDamped:
            {
                float lambda = dampedSvdLambda;

                if (invParameter != 0.0f)
                {
                    lambda = invParameter;
                }

                invJac = MathTools::getPseudoInverseDamped(m, lambda);
                break;
            }

            case eSVDDampedDynamic:
            {
                invJac = MathTools::getDampedLeastSquareInverse(m, MathTools::getDamping(m));
                break;
            }

            default:
                THROW_VR_EXCEPTION("Inverse Jacobi Method nyi...");
        }

#ifdef CHECK_PERFORMANCE
        clock_t endT = clock();
        float diffClock = (float)(((float)(endT - startT) / (float)CLOCKS_PER_SEC) * 1000.0f);
        //if (diffClock>10.0f)
        std::cout << "Inverse Jacobi time:" << diffClock << std::endl;
#endif
    }

    void JacobiProvider::updatePseudoInverseJacobianMatrixD(Eigen::MatrixXd& invJac, const Eigen::MatrixXd& m, double invParameter, Eigen::VectorXd regularization) const
    {
        Eigen::MatrixXd m2 = m;
        VR_ASSERT(regularization.rows() == 0 || regularization.rows() == m2.rows());
        if(regularization.rows() != m2.rows())
        {
            regularization = Eigen::VectorXd::Ones(m2.rows());
        }
        m2 = regularization.asDiagonal() * m2;
        updatePseudoInverseJacobianMatrixDInternal(invJac, m2, invParameter);
        invJac = invJac * regularization.asDiagonal();
    }

    void JacobiProvider::updatePseudoInverseJacobianMatrixDInternal(Eigen::MatrixXd& invJac, const Eigen::MatrixXd& m, double invParameter) const
    {
#ifdef CHECK_PERFORMANCE
        clock_t startT = clock();
#endif

        VR_ASSERT(m.rows() > 0 && m.cols() > 0 && invJac.cols() == m.rows() && invJac.rows() == m.cols());

        switch (inverseMethod)
        {
            case eTranspose:
            {
                if (jointWeights.rows() == m.cols())
                {
                    Eigen::MatrixXd W = jointWeights.cast<double>().asDiagonal();
                    Eigen::MatrixXd W_1 = W.inverse();
                    invJac = W_1 * m.transpose() * (m * W_1 * m.transpose()).inverse();
                }
                else
                {
                    invJac = m.transpose() * (m * m.transpose()).inverse();
                }

                break;
            }

            case eSVD:
            {
                double pinvtoler = 0.00001;

                if (invParameter != 0.0f)
                {
                    pinvtoler = invParameter;
                }

                if (jointWeights.rows() == m.cols())
                {
                    Eigen::MatrixXd W_12(jointWeights.rows(), jointWeights.rows());
                    W_12.setZero();

                    for (int i = 0; i < jointWeights.rows(); i++)
                    {
                        THROW_VR_EXCEPTION_IF(jointWeights(i) <= 0.f, "joint weights cannot be negative or zero");
                        W_12(i, i) = sqrt(1 / jointWeights(i));
                    }

                    invJac = W_12 * MathTools::getPseudoInverseD(m * W_12, pinvtoler);
                }
                else
                {
                    invJac = MathTools::getPseudoInverseD(m, pinvtoler);
                }

                break;
            }

            case eSVDDamped:
            {
                double pinvtoler = 1.0;

                if (invParameter != 0.0)
                {
                    pinvtoler = invParameter;
                }

                invJac = MathTools::getPseudoInverseDampedD(m, pinvtoler);
                break;
            }

            case eSVDDampedDynamic:
            {
                invJac = MathTools::getDampedLeastSquareInverse(m, MathTools::getDamping(m));
                break;
            }

            default:
                THROW_VR_EXCEPTION("Inverse Jacobi Method nyi...");
        }

#ifdef CHECK_PERFORMANCE
        clock_t endT = clock();
        float diffClock = (float)(((float)(endT - startT) / (float)CLOCKS_PER_SEC) * 1000.0f);
        //if (diffClock>10.0f)
        std::cout << "Inverse Jacobi time:" << diffClock << std::endl;
#endif
    }

    float JacobiProvider::getJacobiRadianRegularization() const
    {
        return jacobiRadianRegularization;
    }

    void JacobiProvider::setJacobiRadianRegularization(float value)
    {
        jacobiRadianRegularization = value;
    }

    float JacobiProvider::getJacobiMMRegularization() const
    {
        return jacobiMMRegularization;
    }

    void JacobiProvider::setJacobiMMRegularization(float value)
    {
        jacobiMMRegularization = value;
    }

    float JacobiProvider::getDampedSvdLambda() const
    {
        return dampedSvdLambda;
    }

    void JacobiProvider::setDampedSvdLambda(float value)
    {
        dampedSvdLambda = value;
    }



    VirtualRobot::RobotNodeSetPtr JacobiProvider::getRobotNodeSet()
    {
        return rns;
    }

    void JacobiProvider::setJointWeights(const Eigen::VectorXf& jointWeights)
    {
        this->jointWeights = jointWeights;
    }

    void JacobiProvider::print()
    {
        std::cout << "IK solver:" << name << std::endl;
        std::cout << "==========================" << std::endl;
        std::cout << "RNS:" << rns->getName() << " with " << rns->getSize() << " joints" << std::endl;
    }

    bool JacobiProvider::isInitialized()
    {
        return initialized;
    }

} // namespace VirtualRobot
