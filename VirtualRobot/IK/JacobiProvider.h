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
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2013 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#pragma once

#include <string>

#include "../VirtualRobot.h"
#include "IKSolver.h"

namespace VirtualRobot
{


    class VIRTUAL_ROBOT_IMPORT_EXPORT JacobiProvider :
        public std::enable_shared_from_this<JacobiProvider>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
          @brief Several methods are offered for inverting the Jacobi (i.e. building the Pseudoinverse)
        */
        enum InverseJacobiMethod
        {
            eSVD, //<! PseudoInverse Jacobian. Performing SVD and setting very small eigen values to zero results in a quite stable inverting of the Jacobi. (default)
            eSVDDamped, //<! Using the damped PseudoInverse algorithm
            eSVDDampedDynamic, //<! Using the damped PseudoInverse algorithm with a different computation of the damping factor which is dynamically calculated when near a singularity
            eTranspose //<! The Jacobi Transpose method is faster than SVD and works well for redundant kinematic chains.
        };

        /*!

        */
        JacobiProvider(RobotNodeSetPtr rns, InverseJacobiMethod invJacMethod = eSVD);

        virtual ~JacobiProvider();

        virtual Eigen::MatrixXf getJacobianMatrix() = 0;
        virtual Eigen::MatrixXd getJacobianMatrixD();
        virtual Eigen::MatrixXf getJacobianMatrix(SceneObjectPtr tcp) = 0;
        virtual Eigen::MatrixXd getJacobianMatrixD(SceneObjectPtr tcp);

        virtual Eigen::MatrixXf computePseudoInverseJacobianMatrix(
            const Eigen::MatrixXf& m,
            const Eigen::VectorXf regularization = Eigen::VectorXf()) const;
        virtual Eigen::MatrixXd computePseudoInverseJacobianMatrixD(
            const Eigen::MatrixXd& m,
            const Eigen::VectorXd regularization = Eigen::VectorXd()) const;
        virtual Eigen::MatrixXf computePseudoInverseJacobianMatrix(
            const Eigen::MatrixXf& m,
            float invParameter,
            const Eigen::VectorXf regularization = Eigen::VectorXf()) const;
        virtual Eigen::MatrixXd computePseudoInverseJacobianMatrixD(
            const Eigen::MatrixXd& m,
            double invParameter,
            const Eigen::VectorXd regularization = Eigen::VectorXd()) const;
        virtual void
        updatePseudoInverseJacobianMatrix(Eigen::MatrixXf& invJac,
                                          const Eigen::MatrixXf& m,
                                          float invParameter = 0.0f,
                                          Eigen::VectorXf regularization = Eigen::VectorXf()) const;
        virtual void updatePseudoInverseJacobianMatrixD(
            Eigen::MatrixXd& invJac,
            const Eigen::MatrixXd& m,
            double invParameter = 0.0,
            Eigen::VectorXd regularization = Eigen::VectorXd()) const;
        virtual Eigen::MatrixXf
        getPseudoInverseJacobianMatrix(const Eigen::VectorXf regularization = Eigen::VectorXf());
        virtual Eigen::MatrixXd
        getPseudoInverseJacobianMatrixD(const Eigen::VectorXd regularization = Eigen::VectorXd());
        virtual Eigen::MatrixXf
        getPseudoInverseJacobianMatrix(SceneObjectPtr tcp,
                                       const Eigen::VectorXf regularization = Eigen::VectorXf());
        virtual Eigen::MatrixXd
        getPseudoInverseJacobianMatrixD(SceneObjectPtr tcp,
                                        const Eigen::VectorXd regularization = Eigen::VectorXd());

        virtual Eigen::VectorXf
        getJacobiRegularization(IKSolver::CartesianSelection mode = IKSolver::All);

        VirtualRobot::RobotNodeSetPtr getRobotNodeSet();


        /*!
            The error vector. the value depends on the implementation.
        */
        virtual Eigen::VectorXf getError(float stepSize = 1.0f) = 0;
        virtual bool checkTolerances() = 0;

        bool isInitialized();
        /*
            If set, a weighted inverse Jacobian is computed. The weighting is only applied in eTranspose mode!
            jointScaling.rows() must be nDoF
            Large entries result in small joint deltas.
        */
        void setJointWeights(const Eigen::VectorXf& jointWeights);

        /*!
         * \brief print Print current status of the IK solver
         */
        virtual void print();

        float getDampedSvdLambda() const;
        void setDampedSvdLambda(float value);

        float getJacobiMMRegularization() const;
        void setJacobiMMRegularization(float value);

        float getJacobiRadianRegularization() const;
        void setJacobiRadianRegularization(float value);

    protected:
        virtual void updatePseudoInverseJacobianMatrixInternal(Eigen::MatrixXf& invJac,
                                                               const Eigen::MatrixXf& m,
                                                               float invParameter = 0.0f) const;
        virtual void updatePseudoInverseJacobianMatrixDInternal(Eigen::MatrixXd& invJac,
                                                                const Eigen::MatrixXd& m,
                                                                double invParameter = 0.0) const;

        std::string name;
        RobotNodeSetPtr rns;
        InverseJacobiMethod inverseMethod;
        bool initialized;
        Eigen::VectorXf jointWeights;
        float dampedSvdLambda;
        float jacobiMMRegularization;
        float jacobiRadianRegularization;
    };

    typedef std::shared_ptr<JacobiProvider> JacobiProviderPtr;
} // namespace VirtualRobot
