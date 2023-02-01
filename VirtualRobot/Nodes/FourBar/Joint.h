#pragma once

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Expressions.h"


namespace VirtualRobot::four_bar
{

    // this class represents the four bar mechanisms; in particular the actuated joint
    class Joint
    {
    public:
        using Jacobian = Eigen::Matrix<double, 3, 1>;

    public:
        struct Dimensions;

        Joint(double theta0, const Dimensions& dimensions);

        struct Dimensions
        {
            double shank = 280;
            double p1 = 84.375;
            double p2 = 270;
            double p3 = 45;

            // C.15
            double
            k1() const
            {
                return shank / p1;
            }

            // C.16
            double
            k2() const
            {
                return shank / p3;
            }

            // C.17
            double
            k3() const
            {
                constexpr auto squared = [](const double t) { return t * t; };

                return (squared(shank) + squared(p1) + squared(p3) - squared(p2)) / (2 * p1 * p3);
            }
        };

        double psi(double theta) const;

        // compute pose of actuated joint in passive joint frame
        Eigen::Isometry3d computeFk(double theta) const;

        Jacobian getJacobian(double theta, const Eigen::Vector3d& base_P_eef) const;

        // Eigen::Vector2d angleToPosition(const Eigen::Vector2d& alpha) const;


    public:
        const double theta0;
        const Dimensions dims;

        double limitLo = 0;
        double limitHi = 0;


        Expressions fk;

        // Eigen::Isometry3d transformation = Eigen::Isometry3d::Identity();


    };

} // namespace VirtualRobot::four_bar
