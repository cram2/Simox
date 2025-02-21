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
* @author     Andre Meixner
* @author     Noémie Jaquier
* @copyright  2021 Andre Meixner
*             GNU Lesser General Public License
*
*/
#pragma once

#include <eigen3/Eigen/Core>
#include <Eigen/Dense>

#include "VirtualRobot/IK/IKSolver.h"
#include "VirtualRobot/VirtualRobot.h"
#include <unsupported/Eigen/CXX11/Tensor>

namespace VirtualRobot
{

    class AbstractManipulability
    {
    public:
        enum Type
        {
            Velocity = 1,
            Force = 2
        };

        enum Mode
        {
            Whole = 1,
            Position = 2,
            Orientation = 3
        };

        AbstractManipulability(Mode mode,
                               Type type,
                               Eigen::MatrixXd weightMatrixInit = Eigen::MatrixXd());

        Eigen::MatrixXd computeJacobian();

        Eigen::MatrixXd computeFullJacobian();

        Eigen::MatrixXd
        getJacobianSubMatrix(const Eigen::Matrix<double, 6, Eigen::Dynamic>& jacobian);

        Eigen::MatrixXd
        getManipulabilitySubMatrix(const Eigen::Matrix<double, 6, 6>& manipulability);

        virtual Eigen::MatrixXd computeManipulability();

        Eigen::MatrixXd computeManipulability(const Eigen::MatrixXd& jacobian);

        virtual Eigen::MatrixXd computeManipulability(const Eigen::MatrixXd& jacobian,
                                                      Type type) = 0;

        VisualizationNodePtr getManipulabilityVis(const std::string& visualizationType = "",
                                                  double scaling = 1000.0);

        VisualizationNodePtr getManipulabilityVis(const Eigen::MatrixXd& manipulability,
                                                  const std::string& visualizationType = "",
                                                  double scaling = 1000.0);

        VisualizationNodePtr getManipulabilityVis(const Eigen::MatrixXd& manipulability,
                                                  const Eigen::Vector3f& position,
                                                  const std::string& visualizationType = "",
                                                  double scaling = 1000.0);

        void getEllipsoidOrientationAndScale(const Eigen::MatrixXd& manipulability,
                                             Eigen::Quaternionf& orientation,
                                             Eigen::Vector3d& scale);

        void getEllipsoidOrientationAndScale(Eigen::Quaternionf& orientation,
                                             Eigen::Vector3d& scale);

        IKSolver::CartesianSelection getCartesianSelection();

        Mode getMode();

        Type getType();

        int getTaskVars();

        virtual void setConvertMMtoM(bool value) = 0;

        virtual Eigen::Vector3f getGlobalPosition() = 0;

        virtual Eigen::Matrix4f getCoordinateSystem() = 0;

        virtual Eigen::Vector3f getLocalPosition() = 0;

        virtual std::vector<std::string> getJointNames() = 0;

        virtual Eigen::VectorXd getJointAngles() = 0;

        virtual Eigen::VectorXd getJointLimitsHigh() = 0;

        virtual Eigen::VectorXd getJointLimitsLow() = 0;

        static Eigen::MatrixXd
        GetJacobianSubMatrix(const Eigen::Matrix<double, 6, Eigen::Dynamic>& jacobian,
                             IKSolver::CartesianSelection mode);

        static Eigen::MatrixXd
        GetJacobianSubMatrix(const Eigen::Matrix<double, 6, Eigen::Dynamic>& jacobian, Mode mode);

        static IKSolver::CartesianSelection GetCartesianSelection(Mode mode);

        Eigen::MatrixXd weightMatrix;

    protected:
        virtual Eigen::MatrixXd computeJacobian(IKSolver::CartesianSelection mode) = 0;

        Mode mode;
        Type type;
    };

    typedef std::shared_ptr<AbstractManipulability> AbstractManipulabilityPtr;

} // namespace VirtualRobot
