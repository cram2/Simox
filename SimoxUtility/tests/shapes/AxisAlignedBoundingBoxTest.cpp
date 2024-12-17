/**
* @package    SimoxUtility
* @author     Raphael Grimm
* @copyright  2019 Raphael Grimm
*/

#define BOOST_TEST_MODULE SimoxUtility / shapes / AxisAlignedBoundingBoxTest

#include <iostream>
#include <random>

#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/shapes/AxisAlignedBoundingBox.h>

namespace
{
    struct PointT
    {
        float x = 0, y = 0, z = 0;
    };
} // namespace

BOOST_AUTO_TEST_CASE(test_AABB_from_points_eigen)
{
    std::vector<Eigen::Vector3f> points{{0, 0, 0}, {-2, 0, 1}, {2, 0, -1}};

    simox::AxisAlignedBoundingBox aabb = simox::aabb::from_points(points);

    BOOST_CHECK_EQUAL(aabb.min(), Eigen::Vector3f(-2, 0, -1));
    BOOST_CHECK_EQUAL(aabb.max(), Eigen::Vector3f(2, 0, 1));
}

BOOST_AUTO_TEST_CASE(test_AABB_from_points_custom)
{
    std::vector<PointT> points{{0, 0, 0}, {-2, 0, 1}, {2, 0, -1}};

    simox::AxisAlignedBoundingBox aabb = simox::aabb::from_points(points);

    BOOST_CHECK_EQUAL(aabb.min(), Eigen::Vector3f(-2, 0, -1));
    BOOST_CHECK_EQUAL(aabb.max(), Eigen::Vector3f(2, 0, 1));
}

BOOST_AUTO_TEST_CASE(test_overlaps)
{
    using Eigen::Vector3f;
    using AABB = simox::AxisAlignedBoundingBox;
    using simox::aabb::is_colliding;

    const AABB lhs(Vector3f(-1, -2, -3), Vector3f(1, 2, 3));

    // Collide with itself.
    BOOST_CHECK(is_colliding(lhs, lhs));

    // Overlap when translated a bit.
    BOOST_CHECK(is_colliding(lhs, AABB(Vector3f(0, -2, -3), Vector3f(2, 2, 3))));
    BOOST_CHECK(is_colliding(lhs, AABB(Vector3f(-1, -3, -3), Vector3f(1, 1, 3))));
    BOOST_CHECK(is_colliding(lhs, AABB(Vector3f(-1, -2, 2.5), Vector3f(1, 2, 6))));
    BOOST_CHECK(is_colliding(lhs, AABB(Vector3f(0, -1, -2), Vector3f(2, 3, 4))));

    // No overlap when translated to much.
    BOOST_CHECK(not is_colliding(lhs, AABB(Vector3f(2, -2, -3), Vector3f(4, 2, 3))));
    BOOST_CHECK(not is_colliding(lhs, AABB(Vector3f(-1, -6, -3), Vector3f(1, -3, 3))));
    BOOST_CHECK(not is_colliding(lhs, AABB(Vector3f(-1, -2, 4), Vector3f(1, 2, 10))));
    BOOST_CHECK(not is_colliding(lhs, AABB(Vector3f(1.1, 2.1, 3.1), Vector3f(2, 3, 4))));
    BOOST_CHECK(not is_colliding(lhs, AABB(Vector3f(-2, -3, -4), Vector3f(-1.1, -2.1, -3.1))));

    // rhs in lhs.
    BOOST_CHECK(is_colliding(lhs, AABB(Vector3f(-0.5, -1.0, -1.5), Vector3f(0.5, 1.0, 1.5))));

    // lhs in rhs.
    BOOST_CHECK(is_colliding(lhs, AABB(Vector3f(-2, -4, -6), Vector3f(2, 4, 6))));

    // More difficult cases.
    BOOST_CHECK(is_colliding(lhs, AABB(Vector3f(-0.5, -10, -20), Vector3f(0.5, 10, 20))));
    BOOST_CHECK(is_colliding(lhs, AABB(Vector3f(0.5, 0.5, -20), Vector3f(1.5, 2.5, 20))));
}
