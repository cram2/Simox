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
        using Jacobian = Eigen::Matrix<double, 6, 1>;

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
                constexpr auto squared = [](const double t){ return t * t; };

                return (squared(shank) + squared(p1) + squared(p3) - squared(p2)) / (2 * p1 * p3);
            }
        };

        double
        psi(const double theta)
        {
            const double k1 = dims.k1();
            const double k2 = dims.k2();
            const double k3 = dims.k3();

            const double cosTheta = std::cos(theta);
            const double sinTheta = std::sin(theta);

            const double A = k1 * cosTheta + k2 + k3 + cosTheta; // C.34
            const double B = -2 * sinTheta; // C.35
            const double C = k1 * cosTheta - k2 + k3 - cosTheta; // C.36

            const double psi = 2 * std::atan((-B + std::sqrt(B * B - 4 * A * C)) / (2 * A)); // C.39

            return psi;
        }

        // compute pose of actuated joint in passive joint frame
        void computeFkOfAngle(double theta);


        Eigen::Vector3d getEndEffectorTranslation() const;
        Eigen::Matrix3d getEndEffectorRotation() const;
        Eigen::Matrix4d getEndEffectorTransform() const;
        Jacobian getJacobian() const;

        // Eigen::Vector2d angleToPosition(const Eigen::Vector2d& alpha) const;


    public:
        const double theta0;
        const Dimensions dims; 

        double limitLo = 0;
        double limitHi = 0;


        Expressions fk;

        Eigen::Isometry3d transformation = Eigen::Isometry3d::Identity();
    };

} // namespace VirtualRobot::four_bar
