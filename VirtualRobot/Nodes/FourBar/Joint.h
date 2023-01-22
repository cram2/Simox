#pragma once

#include <cmath>

#include <Eigen/Core>

#include "Expressions.h"


namespace VirtualRobot::four_bar
{

    class Joint
    {
    public:
        using Jacobian = Eigen::Matrix<double, 6, 1>;

    public:
        Joint();
        Joint(double lever, double theta0);

        struct Dimensions
        {
            float shank = 280;
            float p1 = 84.375;
            float p2 = 270;
            float p3 = 45;

            // C.15
            float
            k1() const
            {
                return shank / p1;
            }

            // C.16
            float
            k2() const
            {
                return shank / p3;
            }

            // C.17
            float
            k3() const
            {
                constexpr auto squared = [](const float t) -> decltype(t) { return t * t; };

                return (squared(shank) + squared(p1) + squared(p3) - squared(p2)) / (2 * p1 * p3);
            }
        };

        float
        psi(const float theta)
        {
            Dimensions dims; // TODO

            const float k1 = dims.k1();
            const float k2 = dims.k2();
            const float k3 = dims.k3();

            const float cosTheta = std::cos(theta);
            const float sinTheta = std::sin(theta);

            const float A = k1 * cosTheta + k2 + k3 + cosTheta; // C.34
            const float B = -2 * sinTheta; // C.35
            const float C = k1 * cosTheta - k2 + k3 - cosTheta; // C.36

            const float psi = 2 * std::atan((-B + std::sqrt(B * B - 4 * A * C)) / (2 * A)); // C.39

            return psi;
        }


        void setConstants(double lever, double theta0);


        void computeFkOfAngle(const Eigen::Vector2d& alpha12);

        void computeFkOfPosition(const Eigen::Vector2d& p12);
        void computeFkOfPosition(double p1, double p2);


        Eigen::Vector3d getEndEffectorTranslation() const;
        Eigen::Matrix3d getEndEffectorRotation() const;
        Eigen::Matrix4d getEndEffectorTransform() const;
        Jacobian getJacobian() const;

        Eigen::Vector2d angleToPosition(const Eigen::Vector2d& alpha) const;


    public:
        double lever = 0;
        double theta0 = 0;
        double radius = 0;

        double limitLo = 0;
        double limitHi = 0;


        Expressions fk;
    };

} // namespace VirtualRobot::four_bar
