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

#include "AbstractManipulability.h"

namespace VirtualRobot
{

    /**
 * Implementation of manipulability tracking, see
 * N. Jaquier, L. Rozo, D. G. Caldwell and S. Calinon. Geometry-aware Tracking of Manipulability Ellipsoids, in Robotics: Science and Systems (R:SS), 2018. (http://www.roboticsproceedings.org/rss14/p27.pdf)
 * N. Jaquier, L. Rozo, D. G. Caldwell and S. Calinon. Geometry-aware Manipulability Learning, Tracking and Transfer, International Journal of Robotics Research (IJRR), 2020.
 * @brief The Manipulability class
 */
    class BimanualManipulability : public AbstractManipulability
    {
    public:
        BimanualManipulability(const RobotNodeSetPtr& rnsLeft,
                               const RobotNodeSetPtr& rnsRight,
                               Mode mode,
                               Type type,
                               bool convertMMtoM = true);

        /**
     * @param rns The robotNodes (i.e., joints) for which the Jacobians should be calculated.
     * @param node
     * @param coordSystem The coordinate system in which the Jacobians are defined. By default the global coordinate system is used.
     */
        BimanualManipulability(const RobotNodeSetPtr& rnsLeft,
                               const RobotNodeSetPtr& rnsRight,
                               const RobotNodePtr& nodeLeft,
                               const RobotNodePtr& nodeRight,
                               Mode mode,
                               Type type,
                               const SceneObjectPtr& object = SceneObjectPtr(),
                               const RobotNodePtr& coordSystem = RobotNodePtr(),
                               bool convertMMtoM = true);

        Eigen::MatrixXd computeFullJacobianLeft();

        Eigen::MatrixXd computeFullJacobianRight();

        Eigen::MatrixXd computeBimanualJacobian(const Eigen::MatrixXd& jacobianLeft,
                                                const Eigen::MatrixXd& jacobianRight);

        using AbstractManipulability::computeManipulability;

        virtual Eigen::MatrixXd computeManipulability() override;

        virtual Eigen::MatrixXd computeManipulability(const Eigen::MatrixXd& jacobian,
                                                      Type type) override;

        Eigen::MatrixXd computeBimanualManipulability(const Eigen::MatrixXd& bimanualJacobian,
                                                      const Eigen::MatrixXd& bimanualGraspMatrix);

        Eigen::MatrixXd computeBimanualManipulability(const Eigen::MatrixXd& bimanualJacobian,
                                                      const Eigen::MatrixXd& bimanualGraspMatrix,
                                                      Type type);

        virtual Eigen::Vector3f getLocalPosition() override;

        virtual Eigen::Vector3f getGlobalPosition() override;

        virtual Eigen::Matrix4f getCoordinateSystem() override;

        Eigen::MatrixXd computeBimanualGraspMatrix();

        RobotNodeSetPtr getLeftRobotNodeSet();

        RobotNodeSetPtr getRightRobotNodeSet();

        virtual std::vector<std::string> getJointNames() override;

        virtual Eigen::VectorXd getJointAngles() override;

        virtual Eigen::VectorXd getJointLimitsHigh() override;

        virtual Eigen::VectorXd getJointLimitsLow() override;

        RobotPtr getRobot();

        RobotNodeSetPtr
        createRobotNodeSet(const std::string& name = "BimanualManipulabilityTracking");

        virtual void setConvertMMtoM(bool value) override;

    protected:
        virtual Eigen::MatrixXd computeJacobian(IKSolver::CartesianSelection mode) override;
        Eigen::MatrixXd computeJacobianLeft(IKSolver::CartesianSelection mode);

        Eigen::MatrixXd computeJacobianRight(IKSolver::CartesianSelection mode);

    private:
        Eigen::MatrixXd computeBimanualGraspMatrix(const Eigen::Vector3f& endeffectorLeftPosition,
                                                   const Eigen::Vector3f& endeffectorRightPosition,
                                                   const Eigen::MatrixXd& rotation);
        Eigen::MatrixXd computeGraspMatrix(const Eigen::VectorXf& endeffectorPosition,
                                           const Eigen::VectorXf& objectPosition,
                                           const Eigen::MatrixXd& rotation);

        RobotNodeSetPtr rnsLeft;
        RobotNodeSetPtr rnsRight;
        RobotNodePtr nodeLeft;
        RobotNodePtr nodeRight;
        SceneObjectPtr object;
        RobotNodePtr coordSystem;
        DifferentialIKPtr ikLeft;
        DifferentialIKPtr ikRight;
    };

    typedef std::shared_ptr<BimanualManipulability> BimanualManipulabilityPtr;

} // namespace VirtualRobot
