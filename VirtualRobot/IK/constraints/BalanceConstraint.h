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
* @author     Peter Kaiser
* @copyright  2015 Peter Kaiser
*             GNU Lesser General Public License
*
*/

#pragma once

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/IK/Constraint.h>
#include <VirtualRobot/IK/CoMIK.h>
#include <VirtualRobot/IK/SupportPolygon.h>

class SoSeparator;
class SoNode;

namespace VirtualRobot
{
    class BalanceConstraintOptimizationFunction
    {
    public:
        BalanceConstraintOptimizationFunction(const SupportPolygonPtr &supportPolygon);

        void updateSupportPolygon();

        double evaluateOptimizationFunction(const Eigen::Vector2f &com);
        Eigen::VectorXf evaluateOptimizationGradient(const Eigen::Vector2f &com, const Eigen::MatrixXf &Jcom);

    protected:
        void update();

        double sigmoid(double x);
        double sigmoid_prime(double x);

    protected:
        SupportPolygonPtr supportPolygon;

        std::vector<Eigen::Matrix2f> matrices;
        std::vector<Eigen::Vector2f> displacements;
    };
    typedef std::shared_ptr<BalanceConstraintOptimizationFunction> BalanceConstraintOptimizationFunctionPtr;

    class VIRTUAL_ROBOT_IMPORT_EXPORT BalanceConstraint : public Constraint, public std::enable_shared_from_this<BalanceConstraint>
    {
    public:
        BalanceConstraint(const RobotPtr& robot, const RobotNodeSetPtr& joints, const RobotNodeSetPtr& bodies, const SceneObjectSetPtr& contactNodes,
                          float tolerance = 0.1f, float minimumStability = 0.5f, float maxSupportDistance = 10.0f, bool supportPolygonUpdates = true, bool considerCoMHeight = false);
        BalanceConstraint(const RobotPtr& robot, const RobotNodeSetPtr& joints, const RobotNodeSetPtr& bodies, const SupportPolygonPtr& supportPolygon,
                          float tolerance = 0.1f, float minimumStability = 0.5f, float maxSupportDistance = 10.0f, bool supportPolygonUpdates = true, bool considerCoMHeight = false);

        Eigen::MatrixXf getJacobianMatrix() override;
        Eigen::MatrixXf getJacobianMatrix(SceneObjectPtr tcp) override;
        Eigen::VectorXf getError(float stepSize = 1.0f) override;
        bool checkTolerances() override;

        bool getRobotPoseForConstraint(RobotPtr& robot, Eigen::Matrix4f& pose);
        Eigen::Vector3f getCoM();
        SupportPolygonPtr getSupportPolygon();

        void setCoMHeight(float height);

        double optimizationFunction(unsigned int id) override;
        Eigen::VectorXf optimizationGradient(unsigned int id) override;

    protected:
        void initialize(const RobotPtr& robot, const RobotNodeSetPtr& joints, const RobotNodeSetPtr& bodies, const SceneObjectSetPtr& contactNodes,
                        float tolerance, float minimumStability, float maxSupportDistance, bool supportPolygonUpdates, bool considerCoMHeight);

        void updateSupportPolygon();
        void visualizeSupportPolygon(SoSeparator* sep);

        float getDifferentiableStabilityIndex();
        Eigen::VectorXf getDifferentiableStabilityIndexGradient();

    protected:
        SupportPolygonPtr supportPolygon;

        float height;
        bool considerCoMHeight;

        RobotNodeSetPtr joints;
        RobotNodeSetPtr bodies;

        float minimumStability;
        float maxSupportDistance;
        float tolerance;
        bool supportPolygonUpdates;

    public:
        CoMIKPtr comIK;
        BalanceConstraintOptimizationFunctionPtr differnentiableStability;
    };

    typedef std::shared_ptr<BalanceConstraint> BalanceConstraintPtr;
}

