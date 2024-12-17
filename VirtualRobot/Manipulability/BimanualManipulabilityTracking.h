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

#include "AbstractManipulabilityTracking.h"

namespace VirtualRobot
{

    class BimanualManipulability;
    typedef std::shared_ptr<BimanualManipulability> BimanualManipulabilityPtr;

    /**
 * Implementation of manipulability tracking, see
 * N. Jaquier, L. Rozo, D. G. Caldwell and S. Calinon. Geometry-aware Tracking of Manipulability Ellipsoids, in Robotics: Science and Systems (R:SS), 2018. (http://www.roboticsproceedings.org/rss14/p27.pdf)
 * N. Jaquier, L. Rozo, D. G. Caldwell and S. Calinon. Geometry-aware Manipulability Learning, Tracking and Transfer, International Journal of Robotics Research (IJRR), 2020.
 * @brief The ManipulabilityTracking class
 */
    class BimanualManipulabilityTracking : public AbstractManipulabilityTracking
    {
    public:
        BimanualManipulabilityTracking(BimanualManipulabilityPtr manipulability);

        Eigen::VectorXf calculateVelocity(const Eigen::MatrixXd& manipulabilityDesired,
                                          const Eigen::MatrixXd& gainMatrix = Eigen::MatrixXd(),
                                          bool jointLimitAvoidance = false) override;

        Eigen::Tensor<double, 3>
        computeBimanualManipulabilityJacobian(const Eigen::MatrixXd& jacobianLeft,
                                              const Eigen::MatrixXd& jacobianRight,
                                              const Eigen::MatrixXd& bimanualGraspMatrix);

        Eigen::Tensor<double, 3>
        computeBimanualJacobianDerivative(const Eigen::MatrixXd& jacobianLeft,
                                          const Eigen::MatrixXd& jacobianRight);

        virtual int getTaskVars() override;

        virtual AbstractManipulability::Mode getMode() override;

        BimanualManipulabilityPtr getManipulability();

        Eigen::MatrixXd computeCurrentManipulability() override;

        virtual std::vector<std::string> getJointNames() override;

        VisualizationNodePtr getManipulabilityVis(const Eigen::MatrixXd& manipulability,
                                                  const std::string& visualizationType = "",
                                                  double scaling = 1000.0) override;

        void setConvertMMtoM(bool value) override;

    protected:
        BimanualManipulabilityPtr manipulability;
    };

    typedef std::shared_ptr<BimanualManipulabilityTracking> BimanualManipulabilityTrackingPtr;

} // namespace VirtualRobot
