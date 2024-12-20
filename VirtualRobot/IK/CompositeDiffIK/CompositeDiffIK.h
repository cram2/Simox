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
* @author     André Meixner (andre dot meixner at kit dot edu)
* @author     Simon Ottenhaus (simon dot ottenhaus at kit dot edu)
* @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
*             GNU General Public License
*/

#pragma once
#include <memory>
#include <set>

#include <VirtualRobot/Controller/CartesianPositionController.h>
#include <VirtualRobot/Controller/CartesianVelocityController.h>
#include <VirtualRobot/IK/DifferentialIK.h>
#include <VirtualRobot/Manipulability/AbstractManipulabilityTracking.h>

namespace VirtualRobot
{
    typedef std::shared_ptr<class CompositeDiffIK> CompositeDiffIKPtr;

    class CompositeDiffIK
    {
    public:
        struct Parameters
        {
            Parameters()
            {
            }

            // IK params
            size_t steps = 40;

            float maxJointAngleStep = 0.01f;
            float stepSize = 0.5f;
            bool resetRnsValues = true;
            bool returnIKSteps = false;
            float jointRegularizationTranslation = 1000; // in mm
            float jointRegularizationRotation = 1;
            float jointRegularizationHemisphere = 10;
            float jointRegularizationFourBar = 1;

            std::set<std::string> maxJointAngleStepIgnore = {};
        };

        class NullspaceGradient
        {
        public:
            NullspaceGradient(const std::vector<std::string>& jointNames);

            virtual void init(Parameters& params) = 0;

            virtual Eigen::VectorXf getGradient(Parameters& params, int stepNr) = 0;

            //! Adjusts the gradient when using different robot node sets for ik solver and nullspace
            virtual Eigen::VectorXf getGradientAdjusted(Parameters& params, int stepNr);

            void setMapping(const RobotNodeSetPtr& rns);

            //! Returns a mapping from the nullspace robot node set to the robot node set used in the ik solver
            std::map<int, int> getMapping(const RobotNodeSetPtr& rns);

            float kP = 1;

            std::vector<std::string> getJointNames() const;

        protected:
            std::vector<std::string> jointNames;

        private:
            size_t ikSize;
            std::map<int, int> mapping;
        };
        typedef std::shared_ptr<class NullspaceGradient> NullspaceGradientPtr;

        struct NullspaceTargetStep
        {
            Eigen::Vector3f posDiff;
            Eigen::Vector3f oriDiff;
            Eigen::Matrix4f tcpPose;
            Eigen::VectorXf cartesianVel;
            Eigen::VectorXf jointVel;
        };

        class NullspaceTarget : public NullspaceGradient
        {
        public:
            NullspaceTarget(const RobotNodeSetPtr& rns,
                            const RobotNodePtr& tcp,
                            const Eigen::Matrix4f& target,
                            IKSolver::CartesianSelection mode);
            NullspaceTarget(const RobotNodeSetPtr& rns,
                            const RobotNodePtr& tcp,
                            const Eigen::Vector3f& target);

            RobotNodePtr tcp;
            Eigen::Matrix4f target;
            IKSolver::CartesianSelection mode;
            CartesianPositionController pCtrl;
            CartesianVelocityController vCtrl;

            void init(Parameters&) override;
            Eigen::VectorXf getGradient(Parameters& params, int stepNr) override;

            std::vector<NullspaceTargetStep> ikSteps;
        };
        typedef std::shared_ptr<class NullspaceTarget> NullspaceTargetPtr;

        class AdaptableNullspaceTarget : public NullspaceTarget
        {
        public:
            AdaptableNullspaceTarget(const RobotNodeSetPtr& rns,
                                     const RobotNodePtr& tcp,
                                     const Eigen::Matrix4f& target,
                                     IKSolver::CartesianSelection mode,
                                     int startStepNr,
                                     int endStepNr);
            AdaptableNullspaceTarget(const RobotNodeSetPtr& rns,
                                     const RobotNodePtr& tcp,
                                     const Eigen::Vector3f& target,
                                     int startStepNr,
                                     int endStepNr);

            int startStepNr;
            int endStepNr;

            Eigen::VectorXf getGradient(Parameters& params, int stepNr) override;
        };

        typedef std::shared_ptr<class AdaptableNullspaceTarget> AdaptableNullspaceTargetPtr;

        class NullspaceJointTarget : public NullspaceGradient
        {
        public:
            NullspaceJointTarget(const RobotNodeSetPtr& rns);
            NullspaceJointTarget(const RobotNodeSetPtr& rns,
                                 const Eigen::VectorXf& target,
                                 const Eigen::VectorXf& weight);
            void set(int index, float target, float weight);
            void set(const std::string& jointName, float target, float weight);
            void set(const RobotNodePtr& rn, float target, float weight);

            RobotNodeSetPtr rns;
            Eigen::VectorXf target;
            Eigen::VectorXf weight;

            void init(Parameters&) override;
            Eigen::VectorXf getGradient(Parameters& params, int stepNr) override;
        };
        typedef std::shared_ptr<class NullspaceJointTarget> NullspaceJointTargetPtr;

        class NullspaceJointLimitAvoidance : public NullspaceGradient
        {
        public:
            NullspaceJointLimitAvoidance(const RobotNodeSetPtr& rns);
            NullspaceJointLimitAvoidance(const RobotNodeSetPtr& rns, const Eigen::VectorXf& weight);
            void setWeight(int index, float weight);
            void setWeight(const std::string& jointName, float weight);
            void setWeight(const RobotNodePtr& rn, float weight);
            void setWeights(const RobotNodeSetPtr& rns, float weight);

            RobotNodeSetPtr rns;
            Eigen::VectorXf weight;

            void init(Parameters&) override;
            Eigen::VectorXf getGradient(Parameters& params, int stepNr) override;
        };
        typedef std::shared_ptr<class NullspaceJointLimitAvoidance> NullspaceJointLimitAvoidancePtr;

        struct TargetStep
        {
            Eigen::Vector3f posDiff;
            Eigen::Vector3f oriDiff;
            Eigen::Matrix4f tcpPose; // in mm or m
            Eigen::VectorXf cartesianVel;
        };

        class Target
        {
        public:
            Target(const RobotNodePtr& tcp,
                   const Eigen::Matrix4f& target,
                   IKSolver::CartesianSelection mode);

            RobotNodePtr tcp;
            Eigen::Matrix4f target;
            IKSolver::CartesianSelection mode;
            Eigen::MatrixXf jacobi;
            CartesianPositionController pCtrl;
            float maxPosError = 10.f;
            float maxOriError = 0.05f;
            std::vector<TargetStep> ikSteps;

            bool
            isReached()
            {
                return pCtrl.reached(target, mode, maxPosError, maxOriError);
            }
        };

        typedef std::shared_ptr<Target> TargetPtr;

        struct Result
        {
            Eigen::VectorXf jointValues;
            Eigen::Vector3f posDiff;
            Eigen::Vector3f oriDiff;
            float posError;
            float oriError;
            bool reached;
            Eigen::VectorXf jointLimitMargins;
            float minimumJointLimitMargin;
        };

        struct SolveState
        {
            int rows = 0;
            int cols = 0;
            Eigen::VectorXf jointRegularization;
            Eigen::VectorXf cartesianRegularization;
            Eigen::VectorXf jointValues;
            Eigen::MatrixXf jacobi;
            Eigen::MatrixXf invJac;
            Eigen::MatrixXf nullspace;
        };

        static Eigen::VectorXf LimitInfNormTo(Eigen::VectorXf vec, float maxValue);
        Eigen::VectorXf
        LimitInfNormTo(Eigen::VectorXf vec, float maxValue, const std::set<std::string>& ignore);

        CompositeDiffIK(const RobotNodeSetPtr& rns);

        void addTarget(const TargetPtr& target);
        TargetPtr addTarget(const RobotNodePtr& tcp,
                            const Eigen::Matrix4f& target,
                            IKSolver::CartesianSelection mode);
        void addNullspaceGradient(const NullspaceGradientPtr& gradient);
        NullspaceTargetPtr addNullspacePositionTarget(const RobotNodePtr& tcp,
                                                      const Eigen::Vector3f& target);


        Result solve(Parameters params);
        Result solve(Parameters params, SolveState& s);
        static Eigen::MatrixXf CalculateNullspaceSVD(const Eigen::Matrix4f& jacobi);
        static Eigen::MatrixXf CalculateNullspaceLU(const Eigen::Matrix4f& jacobi);
        void step(Parameters& params, SolveState& s, int stepNr);

        Result getLastResult();

        int CartesianSelectionToSize(IKSolver::CartesianSelection mode);

        RobotNodeSetPtr getRobotNodeSet();

        RobotPtr getRobot();

    private:
        RobotNodeSetPtr rns;
        RobotPtr robot;
        std::vector<TargetPtr> targets;
        std::vector<NullspaceGradientPtr> nullspaceGradients;
        DifferentialIKPtr ik;
    };
} // namespace VirtualRobot
