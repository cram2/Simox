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

#include <VirtualRobot/VirtualRobot.h>

#include "AbstractManipulability.h"

namespace VirtualRobot
{


    class AbstractSingleChainManipulability : public AbstractManipulability
    {
    public:
        using AbstractManipulability::AbstractManipulability;

        using AbstractManipulability::computeManipulability;

        virtual Eigen::MatrixXd computeManipulability(const Eigen::MatrixXd& jacobian,
                                                      Type type) override;
    };

    typedef std::shared_ptr<AbstractSingleChainManipulability> AbstractSingleChainManipulabilityPtr;

    class SingleRobotNodeSetManipulability : public AbstractSingleChainManipulability
    {
    public:
        SingleRobotNodeSetManipulability(const RobotNodeSetPtr& rns,
                                         Mode mode,
                                         Type type,
                                         Eigen::MatrixXd weightMatrixInit = Eigen::MatrixXd(),
                                         bool convertMMtoM = true);

        /**
     * @param rns The robotNodes (i.e., joints) for which the Jacobians should be calculated.
     * @param node
     * @param coordSystem The coordinate system in which the Jacobians are defined. By default the global coordinate system is used.
     */
        SingleRobotNodeSetManipulability(const RobotNodeSetPtr& rns,
                                         const RobotNodePtr& node,
                                         Mode mode,
                                         Type type,
                                         const RobotNodePtr& coordSystem = RobotNodePtr(),
                                         Eigen::MatrixXd weightMatrixInit = Eigen::MatrixXd(),
                                         bool convertMMtoM = true);

        RobotNodeSetPtr getRobotNodeSet();

        virtual RobotPtr getRobot();

        virtual void setConvertMMtoM(bool value) override;

        virtual Eigen::Vector3f getLocalPosition() override;

        virtual Eigen::Vector3f getGlobalPosition() override;

        virtual Eigen::Matrix4f getCoordinateSystem() override;

        virtual std::vector<std::string> getJointNames() override;

        virtual Eigen::VectorXd getJointAngles() override;

        virtual Eigen::VectorXd getJointLimitsHigh() override;

        virtual Eigen::VectorXd getJointLimitsLow() override;

    protected:
        virtual Eigen::MatrixXd computeJacobian(IKSolver::CartesianSelection mode) override;

    private:
        RobotNodeSetPtr rns;
        RobotNodePtr node;
        RobotNodePtr coordSystem;
        DifferentialIKPtr ik;
    };

    typedef std::shared_ptr<SingleRobotNodeSetManipulability> SingleRobotNodeSetManipulabilityPtr;

    class SingleChainManipulability : public AbstractSingleChainManipulability
    {
    public:
        SingleChainManipulability(
            const Eigen::Matrix<double, 6, Eigen::Dynamic>& jacobian,
            Mode mode,
            Type type,
            const Eigen::Matrix<double, Eigen::Dynamic, 1>& jointAngles = Eigen::MatrixXd::Zero(0,
                                                                                                1),
            const Eigen::VectorXd& jointLimitsHigh = Eigen::MatrixXd::Zero(0, 1),
            const Eigen::VectorXd& jointLimitsLow = Eigen::MatrixXd::Zero(0, 1),
            const std::vector<std::string>& jointNames = std::vector<std::string>(),
            const Eigen::Vector3f& globalPosition = Eigen::Vector3f::Zero(),
            const Eigen::Vector3f& localPosition = Eigen::Vector3f::Zero());

        virtual Eigen::Vector3f getLocalPosition() override;

        virtual Eigen::Vector3f getGlobalPosition() override;

        virtual Eigen::Matrix4f getCoordinateSystem() override;

        virtual std::vector<std::string> getJointNames() override;

        virtual Eigen::VectorXd getJointAngles() override;

        virtual Eigen::VectorXd getJointLimitsHigh() override;

        virtual Eigen::VectorXd getJointLimitsLow() override;

        void setJacobian(const Eigen::Matrix<double, 6, Eigen::Dynamic>& jacobian);

        void setLocalPosition(const Eigen::Vector3f& localPosition);

        void setGlobalPosition(const Eigen::Vector3f& globalPosition);

        virtual void
        setConvertMMtoM(bool /*value*/) override
        {
            // do nothing
        }

    protected:
        virtual Eigen::MatrixXd computeJacobian(IKSolver::CartesianSelection mode) override;

    private:
        Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian;
        std::vector<std::string> jointNames;
        Eigen::Vector3f globalPosition;
        Eigen::Vector3f localPosition;
        Eigen::VectorXd jointAngles;
        Eigen::VectorXd jointLimitsHigh;
        Eigen::VectorXd jointLimitsLow;
    };

    typedef std::shared_ptr<SingleChainManipulability> SingleChainManipulabilityPtr;


} // namespace VirtualRobot
