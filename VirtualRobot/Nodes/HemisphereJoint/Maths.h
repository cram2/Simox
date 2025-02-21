#pragma once

#include <optional>

#include <eigen3/Eigen/Core>

#include "Expressions.h"

namespace VirtualRobot::hemisphere
{

    /**
     * @brief Hemisphere joint mathematics for FK and IK.
     *
     * This class is a wrapper about `Expressions`, which contains the actual
     * math code that was generated from Python sympy expressions
     * using `python/hemisphere-joint-demo/hemisphere_joint_demo/sympy_to_code.py`.
     */
    class Maths
    {
    public:
        using ActuatorPosition = Eigen::Vector2d;
        using ActuatorAngle = Eigen::Vector2d;
        using Jacobian = Eigen::Matrix<double, 6, 2>;

    public:
        Maths();
        Maths(double lever, double theta0);

        void setConstants(double lever,
                          double theta0,
                          std::optional<double> limitLoRadians,
                          std::optional<double> limitHiRadians);

        void computeFkOfAngle(const ActuatorAngle& alpha12);

        void computeFkOfPosition(const ActuatorPosition& p12);
        void computeFkOfPosition(double p1, double p2);


        Eigen::Vector3d getEndEffectorTranslation() const;
        Eigen::Matrix3d getEndEffectorRotation() const;
        Eigen::Matrix4d getEndEffectorTransform() const;

        Jacobian getJacobian() const;

        ActuatorPosition angleToPosition(const ActuatorAngle& alpha) const;


    public:
        double lever = 0;
        double theta0Rad = 0;
        double radiusOfRollingSpheres = 0;

        double limitLo = 0;
        double limitHi = 0;


        Expressions expressions;
    };

} // namespace VirtualRobot::hemisphere
