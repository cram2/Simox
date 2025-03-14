#pragma once

#include <set>
#include <map>

#include "../VirtualRobot.h"

namespace RigidBodyDynamics
{
    class Model;
    class Body;
}

namespace VirtualRobot
{

    /** @brief
     * Encapsulates dynamics simulations based on the RBDL library for the virtual robot.
     *
     */
    class Dynamics
    {
    public:
        /// Creates a Dynamics object given a RobotNodeSet.
        /// The rns has to be completely connected (avoid missing RobotNodes).
        /// The rns should end with a RobotNode that has a mass>0 specified, otherwise the last joint can not be added to the internal RBDL model
        ///
        Dynamics(RobotNodeSetPtr rns,
                 RobotNodeSetPtr rnsBodies = RobotNodeSetPtr(),
                 bool verbose = false);
        /// Calculates the Inverse Dynamics for given motion state defined by q, qdot and qddot
        Eigen::VectorXd getInverseDynamics(const Eigen::VectorXd& q,
                                           const Eigen::VectorXd& qdot,
                                           const Eigen::VectorXd& qddot);
        void getInverseDynamics(const Eigen::VectorXd& q,
                                const Eigen::VectorXd& qdot,
                                const Eigen::VectorXd& qddot,
                                Eigen::VectorXd& tau);
        /// Calculates the joint space Gravity Matrix given a joint position vector q and Number of DOF
        Eigen::VectorXd getGravityMatrix(const Eigen::VectorXd& q);
        void getGravityMatrix(const Eigen::VectorXd& q, Eigen::VectorXd& tau);
        /// Calculates the coriolis matrix given position vector q, velocity vector qdot and Number of DOF
        Eigen::VectorXd getCoriolisMatrix(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot);
        /// Calculates forward dynamics given position vector q velocity vector qdot and joint torques tau
        Eigen::VectorXd getForwardDynamics(const Eigen::VectorXd& q,
                                           const Eigen::VectorXd& qdot,
                                           Eigen::VectorXd tau);
        /**
         * @brief Calculates the joint space inertia matrix given a joint position vector q
         * @param q joint angles
         * @param updateKinematics if true, the forward kinematics are calculated based on q. Safer, but more costly.
         * @return InertiaMatrix with size nDoF x nDoF
         */
        Eigen::MatrixXd getInertiaMatrix(const Eigen::VectorXd& q, bool updateKinematics = true);
        void getInertiaMatrix(const Eigen::VectorXd& q,
                              Eigen::MatrixXd& inertiaMatrix,
                              bool updateKinematics = true);
        /// Sets the gravity vector of the dynamics system
        void setGravity(const Eigen::Vector3d& gravity);
        /// returns the number of Degrees of Freedom of the dynamics system
        int getnDoF();

        int
        getIdentifier(std::string name)
        {
            return identifierMap.at(name);
        }

        void print();
        /**
         * @brief computes the inertia matrix of multiple bodies, that must be between the same two joints (or the end of the kinematic chain)
         * @param nodes
         * @return tuple of new InertiaMatrix relative to new CoM, new global CoM and sum of masses
         */
        static std::tuple<Eigen::Matrix3d, Eigen::Vector3d, double>
        computeCombinedPhysics(const std::set<RobotNodePtr>& nodes,
                               const RobotNodePtr& referenceNode);
        std::shared_ptr<RigidBodyDynamics::Body> 
        computeCombinedBody(const std::set<RobotNodePtr>& nodes,
                            const RobotNodePtr& referenceNode) const;
        bool getVerbose() const;
        void setVerbose(bool value);

        std::shared_ptr<RigidBodyDynamics::Model> getModel() const;

    protected:
        RobotNodeSetPtr rns;
        RobotNodeSetPtr rnsBodies;
        std::shared_ptr<RigidBodyDynamics::Model> model;
        Eigen::Vector3d gravity;
        std::map<std::string, int> identifierMap;
        bool verbose = false;
        Eigen::VectorXd zeroVec;

        RobotNodePtr checkForConnectedMass(RobotNodePtr node);
        std::set<RobotNodePtr> getChildrenWithMass(const RobotNodePtr& node,
                                                   const RobotNodeSetPtr& nodeSet) const;

    private:
        void toRBDL(std::shared_ptr<RigidBodyDynamics::Model> model,
                    RobotNodePtr node,
                    RobotNodeSetPtr nodeSet,
                    RobotNodePtr parentNode = RobotNodePtr(),
                    int parentID = 0);
        void toRBDLRecursive(std::shared_ptr<RigidBodyDynamics::Model> model,
                             RobotNodePtr currentNode,
                             Eigen::Matrix4f accumulatedTransformPreJoint,
                             Eigen::Matrix4f accumulatedTransformPostJoint,
                             RobotNodePtr jointNode = RobotNodePtr(),
                             int parentID = 0);
    };

    typedef std::shared_ptr<Dynamics> DynamicsPtr;
} // namespace VirtualRobot
