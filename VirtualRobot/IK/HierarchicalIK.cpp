#include "HierarchicalIK.h"

#include <VirtualRobot/MathTools.h>
#include "VirtualRobotException.h"

using namespace VirtualRobot;
using namespace std;


namespace VirtualRobot
{


    HierarchicalIK::HierarchicalIK(VirtualRobot::RobotNodeSetPtr rns, JacobiProvider::InverseJacobiMethod method)
        : rns(rns), method(method)
    {
        VR_ASSERT(this->rns);
        verbose = false;
    }

    HierarchicalIK::~HierarchicalIK()
    = default;

    void HierarchicalIK::setVerbose(bool v)
    {
        verbose = v;
    }

    Eigen::VectorXf HierarchicalIK::computeStep(const std::vector<JacobiProviderPtr>& jacDefs, float stepSize)
    {
        const double invDamped_lamba = 10.0;

        VR_ASSERT(jacDefs.size() > 0 && jacDefs[0] && jacDefs[0]->getRobotNodeSet());

        if (verbose)
        {
            VR_INFO << "Compute Step" << std::endl;
        }

        int ndof = jacDefs[0]->getRobotNodeSet()->getSize();
        Eigen::VectorXf result(ndof);
        result.setZero();
        std::vector<Eigen::MatrixXd> jacobies;
        std::vector<Eigen::MatrixXd> invJacobies;
        std::vector<Eigen::VectorXd> errors;

        for (size_t i = 0; i < jacDefs.size(); i++)
        {
            THROW_VR_EXCEPTION_IF(!jacDefs[i]->isInitialized(), "JacobiProvider is not initialized...");
            Eigen::MatrixXd j = jacDefs[i]->getJacobianMatrixD();// jacDefs[i].tcp);
            jacobies.push_back(j);

            if (verbose)
            {
                VR_INFO << "Jacoby " << i << ":\n" << j << std::endl;
            }

            j = jacDefs[i]->computePseudoInverseJacobianMatrixD(j);// jacDefs[i].tcp);
            invJacobies.push_back(j);

            errors.push_back(jacDefs[i]->getError().cast<double>());

            if (verbose)
            {
                VR_INFO << "Inv Jacoby " << i << ":\n" << j << std::endl;
            }

            if (jacobies[i].cols() != ndof)
            {
                THROW_VR_EXCEPTION("Expecting " << ndof << " DOFs, but Jacobi " << i << " has " << jacobies[i].cols() << " columns ");
            }

            if (jacobies[i].rows() != errors[i].rows())
            {
                THROW_VR_EXCEPTION("Jacobi " << i << " has " << jacobies[i].rows() << " rows, but delta has " << jacDefs[i]->getError().rows() << " rows ");
            }
        }

        VR_ASSERT(jacobies.size() == jacDefs.size());

        // generate hierarchical gradient descent

        // init with first jacobi
        Eigen::MatrixXd J_i = jacobies[0];
        Eigen::MatrixXd Jinv_i = invJacobies[0];


        Eigen::MatrixXd Jinv_i_min1;
        Eigen::VectorXd result_i = Jinv_i * errors[0] * stepSize;

        if (verbose)
        {
            VR_INFO << "result_i 0:\n" << result_i << std::endl;
        }

        Eigen::VectorXd result_i_min1;
        Eigen::MatrixXd PA_i_min1;
        Eigen::MatrixXd JA_i_min1;
        Eigen::MatrixXd JAinv_i_min1;
        Eigen::MatrixXd id_ndof(ndof, ndof);
        id_ndof.setIdentity();

        int accRowCount = J_i.rows();

        double pinvtoler = 0.00001f;

        for (size_t i = 1; i < jacobies.size(); i++)
        {
            result_i_min1 = result_i;
            Jinv_i_min1 = Jinv_i;
            Jinv_i = invJacobies[i];
            J_i = jacobies[i];
            JA_i_min1.resize(accRowCount, ndof);
            int rowPos = 0;

            for (size_t j = 0; j < i; j++)
            {
                //JA_i_min1.block(colPos,0,jacobies[j].cols(),jacobies[j].rows()) = jacobies[j];
                JA_i_min1.block(rowPos, 0, jacobies[j].rows(), jacobies[j].cols()) = jacobies[j];
                rowPos += jacobies[j].rows();
            }

            if (verbose)
            {
                VR_INFO << "JA_i_min1 " << i << ":\n" << endl << JA_i_min1 << std::endl;
            }

            switch (method)
            {
                case JacobiProvider::eTranspose:
                    JAinv_i_min1 = JA_i_min1.transpose();
                    break;

                case JacobiProvider::eSVD:
                    JAinv_i_min1 = MathTools::getPseudoInverseD(JA_i_min1, pinvtoler);
                    break;

                case JacobiProvider::eSVDDamped:
                    JAinv_i_min1 = MathTools::getPseudoInverseDampedD(JA_i_min1, invDamped_lamba);
                    break;
                default:
                    THROW_VR_EXCEPTION("Inverse Jacobi Method nyi...");
            }


            //JAinv_i_min1 = MathTools::getPseudoInverse(JA_i_min1, pinvtoler);
            if (verbose)
            {
                VR_INFO << "JAinv_i_min1 " << i << ":\n" << endl << JAinv_i_min1 << std::endl;
            }

            PA_i_min1 = id_ndof - JAinv_i_min1 * JA_i_min1;

            if (verbose)
            {
                VR_INFO << "PA_i_min1 " << i << ":\n" << endl << PA_i_min1 << std::endl;
            }

            Eigen::MatrixXd J_tilde_i = J_i * PA_i_min1;

            if (verbose)
            {
                VR_INFO << "J_tilde_i " << i << ":\n" << endl << J_tilde_i << std::endl;
            }

            Eigen::MatrixXd Jinv_tilde_i;

            switch (method)
            {
                case JacobiProvider::eTranspose:
                    Jinv_tilde_i = J_tilde_i.transpose();
                    break;

                case JacobiProvider::eSVD:
                    Jinv_tilde_i = MathTools::getPseudoInverseD(J_tilde_i, pinvtoler);
                    break;

                case JacobiProvider::eSVDDamped:
                    Jinv_tilde_i = MathTools::getPseudoInverseDampedD(J_tilde_i, invDamped_lamba);
                    break;
                default:
                    THROW_VR_EXCEPTION("Inverse Jacobi Method nyi...");
            }

            //Eigen::MatrixXf Jinv_tilde_i = MathTools::getPseudoInverse(J_tilde_i, pinvtoler);

            if (verbose)
            {
                VR_INFO << "Jinv_tilde_i " << i << ":\n" << endl << Jinv_tilde_i << std::endl;
            }

            if (verbose)
            {
                VR_INFO << "jacDefs[i]->getError() " << i << ":\n" << endl << errors[i].transpose() << std::endl;
            }

            result_i = result_i_min1 + Jinv_tilde_i * (errors[i] * stepSize - J_i * result_i_min1);

            if (verbose)
            {
                VR_INFO << "result_i " << i << ":\n" << result_i << std::endl;
            }

            accRowCount += J_i.rows();
        }

        return result_i.cast<float>();

    }

} // namespace VirtualRobot
