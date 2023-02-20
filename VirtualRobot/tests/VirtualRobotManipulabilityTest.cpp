/**
* @package    VirtualRobot
* @author     Andre Meixner
* @copyright  2021 Andre Meixner
*/

#define BOOST_TEST_MODULE VirtualRobot_VirtualRobotManipulabilityTest

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/Manipulability/SingleChainManipulabilityTracking.h>
#include <VirtualRobot/Manipulability/SingleChainManipulability.h>
#include <SimoxUtility/algorithm/string/string_conversion_eigen.h>

#define CHECK_SMALL_VELOCITY(a, b, tolerance) { \
    BOOST_REQUIRE_EQUAL((a).rows(), (b).rows()); \
    for (unsigned int i = 0; i < (a).rows(); i++) { \
        BOOST_CHECK_SMALL(std::abs((a)(i) - (b)(i)), (tolerance)); \
    } \
}

#define CHECK_SMALL_DIFF_VELOCITY(a, b, tolerance) { \
    CHECK_SMALL_VELOCITY(a, b, tolerance) \
}


static const float DefaultTolerance = 1e-4f;


BOOST_AUTO_TEST_SUITE(ManipulabilityTest)

BOOST_AUTO_TEST_CASE(testPositionalVelocityManipulabilityTracking)
{
    // Armar6 average joints
    Eigen::MatrixXd jacobian(6, 8);
    jacobian  << -0.656626, -0.169947, 0.0867239, 0.098642, -0.594799, 0.0045068, -0.0390586, -0.253288, 0.860518, 0.231487, 1.13725e-08, -5.96046e-08, 0.168292, -1.30385e-08, 2.37487e-08, 0.0535143, 0, 0.634252, 0.570741, 0.649175, 0.0903796, 0.0296596, -0.25705, 0.0384871, 0, 0.965926, 1.31134e-07, 0.988652, 0.150226, 0.311744, -0.938216, 0.150226, 0, 1.26666e-07, -1, 1.29646e-07, -6.77231e-08, 0.948985, 0.315322, -2.62416e-08, 1, 0.258819, 0, -0.150226, 0.988652, -0.0473694, 0.142562, 0.988652;

    Eigen::MatrixXd desiredManipulability(3, 3);
    desiredManipulability <<    1.15186,  -0.735976,  -0.118876,
                                -0.735976,   0.653656, -0.0866033,
                                -0.118876, -0.0866033,    1.14306;

    Eigen::VectorXf expectedVelocity(8);
    expectedVelocity <<     0.1729,
                            -0.7210,
                            -3.4009,
                             1.9029,
                             0.5024,
                             0.2881,
                            -2.7081,
                            -0.2294;


    VirtualRobot::SingleChainManipulabilityPtr manipulability(
                new VirtualRobot::SingleChainManipulability(jacobian, VirtualRobot::AbstractManipulability::Position,
                                                            VirtualRobot::AbstractManipulability::Velocity));
    VirtualRobot::SingleChainManipulabilityTrackingPtr tracking(new VirtualRobot::SingleChainManipulabilityTracking(manipulability));
    Eigen::VectorXf velocity = tracking->calculateVelocity(desiredManipulability);

    CHECK_SMALL_DIFF_VELOCITY(velocity, expectedVelocity, DefaultTolerance)
}

//BOOST_AUTO_TEST_CASE(testOrientationalVelocityManipulabilityTracking)
//{
//    // Armar6 average joints
//    Eigen::MatrixXd jacobian(6, 8);
//    jacobian  << -0.656626, -0.169947, 0.0867239, 0.098642, -0.594799, 0.0045068, -0.0390586, -0.253288, 0.860518, 0.231487, 1.13725e-08, -5.96046e-08, 0.168292, -1.30385e-08, 2.37487e-08, 0.0535143, 0, 0.634252, 0.570741, 0.649175, 0.0903796, 0.0296596, -0.25705, 0.0384871, 0, 0.965926, 1.31134e-07, 0.988652, 0.150226, 0.311744, -0.938216, 0.150226, 0, 1.26666e-07, -1, 1.29646e-07, -6.77231e-08, 0.948985, 0.315322, -2.62416e-08, 1, 0.258819, 0, -0.150226, 0.988652, -0.0473694, 0.142562, 0.988652;

//    Eigen::MatrixXd desiredManipulability(3, 3);
//    desiredManipulability <<       2.85461, -0.0611938,   0.161456,
//                                   -0.0611938,    2.34351,  -0.424717,
//                                     0.161456,  -0.424717,    2.80188;

//    Eigen::VectorXf expectedVelocity(8);
//    expectedVelocity << 1.0036,
//                        4.3729,
//                       10.4689,
//                       -7.2922,
//                      -15.6157,
//                       -0.8079,
//                        7.3426,
//                       27.5755;


//    VirtualRobot::SingleChainManipulabilityPtr manipulability(
//                new VirtualRobot::SingleChainManipulability(jacobian, VirtualRobot::AbstractManipulability::Orientation,
//                                                            VirtualRobot::AbstractManipulability::Velocity));
//    VirtualRobot::SingleChainManipulabilityTrackingPtr tracking(new VirtualRobot::SingleChainManipulabilityTracking(manipulability));
//    Eigen::VectorXf velocity = tracking->calculateVelocity(desiredManipulability);

//    CHECK_SMALL_DIFF_VELOCITY(velocity, expectedVelocity)
//}

BOOST_AUTO_TEST_CASE(testWholeVelocityManipulabilityTracking)
{
    // Armar6 average joints
    Eigen::MatrixXd jacobian(6, 8);
    jacobian  << -0.656626, -0.169947, 0.0867239, 0.098642, -0.594799, 0.0045068, -0.0390586, -0.253288, 0.860518, 0.231487, 1.13725e-08, -5.96046e-08, 0.168292, -1.30385e-08, 2.37487e-08, 0.0535143, 0, 0.634252, 0.570741, 0.649175, 0.0903796, 0.0296596, -0.25705, 0.0384871, 0, 0.965926, 1.31134e-07, 0.988652, 0.150226, 0.311744, -0.938216, 0.150226, 0, 1.26666e-07, -1, 1.29646e-07, -6.77231e-08, 0.948985, 0.315322, -2.62416e-08, 1, 0.258819, 0, -0.150226, 0.988652, -0.0473694, 0.142562, 0.988652;

    Eigen::MatrixXd desiredManipulability(6, 6);
    desiredManipulability <<         1.15186,  -0.735976,  -0.118876,  -0.140627,  0.372079,   -1.64893,
                                     -0.735976,   0.653656, -0.0866033,  -0.220524, -0.0254303,   0.970515,
                                     -0.118876, -0.0866033,    1.14306,    1.49837,  -0.560595,   0.166057,
                                     -0.140627,  -0.220524,    1.49837,    2.85461, -0.0611938,   0.161456,
                                      0.372079, -0.0254303,  -0.560595, -0.0611938,    2.34351,  -0.424717,
                                      -1.64893,   0.970515,   0.166057,   0.161456,  -0.424717,    2.80188;

    Eigen::VectorXf expectedVelocity(8);
    expectedVelocity <<     0.0083,
                            -0.2695,
                             0.0478,
                            -2.2594,
                            -1.4446,
                            34.0653,
                            -2.2005,
                             3.7379;


    VirtualRobot::SingleChainManipulabilityPtr manipulability(
                new VirtualRobot::SingleChainManipulability(jacobian, VirtualRobot::AbstractManipulability::Whole,
                                                            VirtualRobot::AbstractManipulability::Velocity));
    VirtualRobot::SingleChainManipulabilityTrackingPtr tracking(new VirtualRobot::SingleChainManipulabilityTracking(manipulability));
    Eigen::VectorXf velocity = tracking->calculateVelocity(desiredManipulability);

    CHECK_SMALL_DIFF_VELOCITY(velocity, expectedVelocity, DefaultTolerance)
}

BOOST_AUTO_TEST_CASE(testPositionalForceManipulabilityTracking)
{
    // Armar6 average joints
    Eigen::MatrixXd jacobian(6, 8);
    jacobian  << -0.656626, -0.169947, 0.0867239, 0.098642, -0.594799, 0.0045068, -0.0390586, -0.253288, 0.860518, 0.231487, 1.13725e-08, -5.96046e-08, 0.168292, -1.30385e-08, 2.37487e-08, 0.0535143, 0, 0.634252, 0.570741, 0.649175, 0.0903796, 0.0296596, -0.25705, 0.0384871, 0, 0.965926, 1.31134e-07, 0.988652, 0.150226, 0.311744, -0.938216, 0.150226, 0, 1.26666e-07, -1, 1.29646e-07, -6.77231e-08, 0.948985, 0.315322, -2.62416e-08, 1, 0.258819, 0, -0.150226, 0.988652, -0.0473694, 0.142562, 0.988652;

    Eigen::MatrixXd desiredManipulability(3, 3);
    desiredManipulability <<        3.54843,  4.08521, 0.678543,
                                    4.08521,  6.24856, 0.898272,
                                   0.678543, 0.898272,  1.01347;

    Eigen::VectorXf expectedVelocity(8);
    expectedVelocity << 1.1371,
                        0.5298,
                       -0.2561,
                        2.5805,
                        0.3481,
                       -0.2669,
                        2.4973,
                       -1.7452;


    VirtualRobot::SingleChainManipulabilityPtr manipulability(
                new VirtualRobot::SingleChainManipulability(jacobian, VirtualRobot::AbstractManipulability::Position,
                                                            VirtualRobot::AbstractManipulability::Force));
    VirtualRobot::SingleChainManipulabilityTrackingPtr tracking(new VirtualRobot::SingleChainManipulabilityTracking(manipulability));
    Eigen::VectorXf velocity = tracking->calculateVelocity(desiredManipulability);

    CHECK_SMALL_DIFF_VELOCITY(velocity, expectedVelocity, DefaultTolerance)
}

//BOOST_AUTO_TEST_CASE(testOrientationalForceManipulabilityTracking)
//{
//    // Armar6 average joints
//    Eigen::MatrixXd jacobian(6, 8);
//    jacobian  << -0.656626, -0.169947, 0.0867239, 0.098642, -0.594799, 0.0045068, -0.0390586, -0.253288, 0.860518, 0.231487, 1.13725e-08, -5.96046e-08, 0.168292, -1.30385e-08, 2.37487e-08, 0.0535143, 0, 0.634252, 0.570741, 0.649175, 0.0903796, 0.0296596, -0.25705, 0.0384871, 0, 0.965926, 1.31134e-07, 0.988652, 0.150226, 0.311744, -0.938216, 0.150226, 0, 1.26666e-07, -1, 1.29646e-07, -6.77231e-08, 0.948985, 0.315322, -2.62416e-08, 1, 0.258819, 0, -0.150226, 0.988652, -0.0473694, 0.142562, 0.988652;

//    Eigen::MatrixXd desiredManipulability(3, 3);
//    desiredManipulability <<         0.351529, 0.00566362,  -0.019398,
//                                     0.00566362,   0.438855,  0.0661965,
//                                      -0.019398,  0.0661965,   0.368055;

//    Eigen::VectorXf expectedVelocity(8);
//    expectedVelocity << 1.0036,
//                        4.3729,
//                       10.4689,
//                       -7.2922,
//                      -15.6157,
//                       -0.8079,
//                        7.3426,
//                       27.5755;


//    VirtualRobot::SingleChainManipulabilityPtr manipulability(
//                new VirtualRobot::SingleChainManipulability(jacobian, VirtualRobot::AbstractManipulability::Orientation,
//                                                            VirtualRobot::AbstractManipulability::Force));
//    VirtualRobot::SingleChainManipulabilityTrackingPtr tracking(new VirtualRobot::SingleChainManipulabilityTracking(manipulability));
//    Eigen::VectorXf velocity = tracking->calculateVelocity(desiredManipulability);

//    CHECK_SMALL_DIFF_VELOCITY(velocity, expectedVelocity)
//}

BOOST_AUTO_TEST_CASE(testWholeForceManipulabilityTracking)
{
    // Armar6 average joints
    Eigen::MatrixXd jacobian(6, 8);
    jacobian  << -0.656626, -0.169947, 0.0867239, 0.098642, -0.594799, 0.0045068, -0.0390586, -0.253288, 0.860518, 0.231487, 1.13725e-08, -5.96046e-08, 0.168292, -1.30385e-08, 2.37487e-08, 0.0535143, 0, 0.634252, 0.570741, 0.649175, 0.0903796, 0.0296596, -0.25705, 0.0384871, 0, 0.965926, 1.31134e-07, 0.988652, 0.150226, 0.311744, -0.938216, 0.150226, 0, 1.26666e-07, -1, 1.29646e-07, -6.77231e-08, 0.948985, 0.315322, -2.62416e-08, 1, 0.258819, 0, -0.150226, 0.988652, -0.0473694, 0.142562, 0.988652;

    Eigen::MatrixXd desiredManipulability(6, 6);
    desiredManipulability <<           17.7117,   11.2696,  -3.55426,   3.20849,  -2.33407,   6.19188,
                                       11.2696,   10.7247,  -2.67026,   2.59712,  -1.76195,   2.65897,
                                      -3.55426,  -2.67026,   4.97508,  -2.90084,   1.45506,  -1.07393,
                                       3.20849,   2.59712,  -2.90084,   2.16073, -0.957258,  0.890947,
                                      -2.33407,  -1.76195,   1.45506, -0.957258,  0.984315, -0.645189,
                                       6.19188,   2.65897,  -1.07393,  0.890947, -0.645189,   2.99438;

    Eigen::VectorXf expectedVelocity(8);
    expectedVelocity <<      0.1131,
                             0.1082,
                             4.6499,
                            -0.7927,
                            -0.6206,
                           -25.2698,
                            -0.4688,
                             0.4812;


    VirtualRobot::SingleChainManipulabilityPtr manipulability(
                new VirtualRobot::SingleChainManipulability(jacobian, VirtualRobot::AbstractManipulability::Whole,
                                                            VirtualRobot::AbstractManipulability::Force));
    VirtualRobot::SingleChainManipulabilityTrackingPtr tracking(new VirtualRobot::SingleChainManipulabilityTracking(manipulability));
    Eigen::VectorXf velocity = tracking->calculateVelocity(desiredManipulability);

    CHECK_SMALL_DIFF_VELOCITY(velocity, expectedVelocity, 4e-4f)
}


BOOST_AUTO_TEST_SUITE_END()
