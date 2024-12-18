#include <chrono>

#include <VirtualRobot/Dynamics/Dynamics.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RuntimeEnvironment.h>
#include <VirtualRobot/Tools/Gravity.h>
#include <VirtualRobot/XML/RobotIO.h>

#include "VirtualRobot/Logging.h"
#include "VirtualRobot/VirtualRobotException.h"
#include <rbdl/Kinematics.h>
using std::cout;
using namespace VirtualRobot;

int
main(int argc, char* argv[])
{
    std::string filename("robots/ArmarIII/ArmarIII.xml");
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(filename);
    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
    std::cout << "Using robot at " << filename << std::endl;
    RobotPtr rob;

    try
    {
        rob = RobotIO::loadRobot(filename, RobotIO::eStructure);
    }
    catch (VirtualRobotException& e)
    {
        std::cout << "Error: " << e.what() << std::endl;
        return -1;
    }

    if (rob)
    {

        RobotNodeSetPtr ns = rob->getRobotNodeSet("RightArm");
        RobotNodeSetPtr bodyNs = rob->getRobotNodeSet("RightArmHandColModel");
        //        RobotNodeSetPtr bodyNs = rob->getRobotNodeSet("RightArmCol");

        Gravity g(rob, ns, bodyNs);
        for (auto& pair : g.getMasses())
        {
            std::cout << pair.first << ": " << pair.second << std::endl;
        }
        VirtualRobot::Dynamics dynamics = VirtualRobot::Dynamics(ns, bodyNs, true);
        dynamics.print();
        int nDof = dynamics.getnDoF();
        Eigen::VectorXd q = Eigen::VectorXd::Zero(nDof);
        ns->setJointValues(q.cast<float>());
        q = ns->getJointValuesEigen().cast<double>(); // get joint values with joint limits applied
        Eigen::VectorXd qdot = Eigen::VectorXd::Random(nDof);
        Eigen::VectorXd qddot = Eigen::VectorXd::Random(nDof);
        Eigen::VectorXd tau = Eigen::VectorXd::Random(nDof);

        auto start = std::chrono::system_clock::now();
        int c = 1000;
        for (int i = 0; i < c; ++i)
        {
            Eigen::VectorXd q = Eigen::VectorXd::Random(nDof);
            Eigen::VectorXd qdot = Eigen::VectorXd::Random(nDof);
            Eigen::VectorXd qddot = Eigen::VectorXd::Random(nDof);
            Eigen::VectorXd tau = Eigen::VectorXd::Random(nDof);
            Eigen::VectorXd invDyn = dynamics.getInverseDynamics(q, qdot, qddot);
            Eigen::MatrixXd inertiaMatrix = dynamics.getInertiaMatrix(q);
            VR_INFO << inertiaMatrix;
        }
        auto end = std::chrono::system_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        std::cout << "duration:" << (elapsed.count() / c) << '\n';

        for (int i = 0; i < c; ++i)
        {
            //            break;
            Eigen::VectorXd q = Eigen::VectorXd::Random(nDof);
            Eigen::VectorXd qdot = Eigen::VectorXd::Random(nDof);
            Eigen::VectorXd qddot = Eigen::VectorXd::Random(nDof);
            Eigen::VectorXd tau = Eigen::VectorXd::Random(nDof);
            ns->setJointValues(q.cast<float>());
            q = ns->getJointValuesEigen()
                    .cast<double>(); // get joint values with joint limits applied
            Eigen::VectorXd gravityRBDL = dynamics.getGravityMatrix(q);
            int d = 0;
            std::vector<float> gravityVR;
            g.computeGravityTorque(gravityVR);
            for (auto& val : gravityVR)
            {
                auto diff = val - gravityRBDL(d);
                if (std::abs(diff) > 0.01)
                    throw std::runtime_error((std::to_string(i) + " dim: " + std::to_string(d) +
                                              " diff: " + std::to_string(diff).c_str()));
                d++;
            }
        }

        Eigen::VectorXd invDyn = dynamics.getInverseDynamics(q, qdot, qddot);
        std::cout << "Joint values:\n" << q << std::endl;
        ns->setJointValues(q.cast<float>());
        std::cout << "Joint values in VR:\n" << q << std::endl;
        std::vector<float> gravityVR;
        g.computeGravityTorque(gravityVR);

        //        std::cout << "joint torques from inverse dynamics: " << endl << invDyn << std::endl;
        std::cout << "joint space inertia matrix: " << std::endl
                  << dynamics.getInertiaMatrix(q) << std::endl;
        std::cout << "joint space gravitational matrix:" << std::endl
                  << dynamics.getGravityMatrix(q) << std::endl;
        std::cout << "joint space VR gravity :" << std::endl;
        int i = 0;
        for (auto& val : gravityVR)
        {
            std::cout << ns->getNode(i)->getName() << ": " << val << std::endl;
            i++;
        }
        std::cout << "joint space coriolis matrix:" << std::endl
                  << dynamics.getCoriolisMatrix(q, qdot) << std::endl;
        std::cout << "joint space accelerations from forward dynamics:" << std::endl
                  << dynamics.getForwardDynamics(q, qdot, tau) << std::endl;
        //        std::cout << "Identifier for Elbow R:" << endl << dynamics.getIdentifier("Elbow R") << std::endl;
    }
    else
    {
        std::cout << " ERROR while creating robobt" << std::endl;
    }
}
