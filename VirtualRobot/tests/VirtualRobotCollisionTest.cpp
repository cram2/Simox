/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2010 Nikolaus Vahrenkamp
*/

#define BOOST_TEST_MODULE VirtualRobot_VirtualRobotCollisionTest

#include <string>

#include <eigen3/Eigen/Core>
#include <Eigen/Geometry>

#include <VirtualRobot/CollisionDetection/CollisionChecker.h>
#include <VirtualRobot/CollisionDetection/CollisionModel.h>
#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/XML/RobotIO.h>

#include <CollisionDetection/CDManager.h>


BOOST_AUTO_TEST_SUITE(CollisionModel)

BOOST_AUTO_TEST_CASE(testCollisionModel)
{
    VirtualRobot::CollisionCheckerPtr colChecker =
        VirtualRobot::CollisionChecker::getGlobalCollisionChecker();
    BOOST_REQUIRE(colChecker);

    VirtualRobot::ObstaclePtr obstacle1(VirtualRobot::Obstacle::createBox(100, 200, 300));
    VirtualRobot::ObstaclePtr obstacle2(VirtualRobot::Obstacle::createBox(10, 20, 30));
    Eigen::Matrix4f m1 = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f m2 = Eigen::Matrix4f::Identity();
    m2(0, 3) = 5000.0f;
    m2(1, 3) = 1000.0f;
    m2(2, 3) = 1500.0f;
    obstacle1->setGlobalPose(m1);
    obstacle2->setGlobalPose(m2);

    BOOST_REQUIRE(obstacle1);
    BOOST_REQUIRE(obstacle2);

    bool col1 =
        colChecker->checkCollision(obstacle1->getCollisionModel(), obstacle2->getCollisionModel());
    BOOST_CHECK_EQUAL(col1, false);

    m2(0, 3) = 50.0f;
    m2(1, 3) = 100.0f;
    m2(2, 3) = 150.0f;
    obstacle1->setGlobalPose(m1);
    obstacle2->setGlobalPose(m2);

    bool col2 =
        colChecker->checkCollision(obstacle1->getCollisionModel(), obstacle2->getCollisionModel());
    BOOST_CHECK_EQUAL(col2, true);
}

BOOST_AUTO_TEST_CASE(testCollisionManager)
{
    VirtualRobot::CDManager cdm;
    VirtualRobot::CollisionCheckerPtr colChecker =
        VirtualRobot::CollisionChecker::getGlobalCollisionChecker();
    BOOST_REQUIRE(colChecker);

    VirtualRobot::ObstaclePtr obstacle1(VirtualRobot::Obstacle::createBox(100, 200, 300));
    VirtualRobot::ObstaclePtr obstacle2(VirtualRobot::Obstacle::createBox(10, 20, 30));
    VirtualRobot::ObstaclePtr obstacle3(VirtualRobot::Obstacle::createBox(10, 20, 30));
    Eigen::Matrix4f m1 = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f m2 = Eigen::Matrix4f::Identity();
    m2(0, 3) = 5000.0f;
    m2(1, 3) = 1000.0f;
    m2(2, 3) = 1500.0f;
    obstacle1->setGlobalPose(m1);
    obstacle2->setGlobalPose(m2);
    obstacle3->setGlobalPose(m2);

    cdm.addCollisionModel(obstacle1);
    cdm.addCollisionModel(obstacle2);
    cdm.addCollisionModel(obstacle3);

    BOOST_REQUIRE(obstacle1);
    BOOST_REQUIRE(obstacle2);
    BOOST_REQUIRE(obstacle3);

    BOOST_CHECK_EQUAL(cdm.getDistance(), 0.0f);
    BOOST_CHECK(cdm.isInCollision());
}

BOOST_AUTO_TEST_SUITE_END()
